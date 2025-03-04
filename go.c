#include <stdio.h>
#include <math.h>

#define WIDTH 100
#define HEIGHT 100
#define R 4  // r을 4로 고정
#define KERNEL_SIZE (2 * R + 1)
#define MATRIX_SIZE 6
#define PATCH_SIZE (KERNEL_SIZE * KERNEL_SIZE)
#define MAX_CORNERS (WIDTH * HEIGHT)

// 2D 점 구조체
typedef struct {
    double x, y;
} point2d;

// 코너 구조체
typedef struct {
    point2d p[MAX_CORNERS];
    int r[MAX_CORNERS];
    point2d v1[MAX_CORNERS];
    point2d v2[MAX_CORNERS];
    point2d v3[MAX_CORNERS];
    double Score[MAX_CORNERS];
    int Size;
} Corner2;

// 이미지 패치 추출 (bilinear interpolation 포함)
void get_image_patch_with_mask(
    double img[HEIGHT][WIDTH], double mask[KERNEL_SIZE][KERNEL_SIZE], 
    double u, double v, int r, double img_sub[PATCH_SIZE], int *num_valid
) {
    int iu = (int)u;
    int iv = (int)v;
    double du = u - iu;
    double dv = v - iv;

    double a00 = 1 - du - dv + du * dv;
    double a01 = du - du * dv;
    double a10 = dv - du * dv;
    double a11 = du * dv;

    *num_valid = 0;
    for (int j = -r; j <= r; j++) {
        for (int i = -r; i <= r; i++) {
            if (mask[j + R][i + R] >= 1e-6) {
                img_sub[*num_valid] =
                    a00 * img[iv + j][iu + i] +
                    a01 * img[iv + j][iu + i + 1] +
                    a10 * img[iv + j + 1][iu + i] +
                    a11 * img[iv + j + 1][iu + i + 1];
                (*num_valid)++;
            }
        }
    }
}

// Saddle Point 검출
void polynomial_fit_saddle(double img[HEIGHT][WIDTH], Corner2* corners) {
    int max_iteration = 5;
    double eps = 0.01;

    double blur_kernel[KERNEL_SIZE][KERNEL_SIZE];
    double mask[KERNEL_SIZE][KERNEL_SIZE];
    create_cone_filter_kernel(blur_kernel);
    int nzs = create_cone_filter_kernel(mask);

    double blur_img[HEIGHT][WIDTH];
    apply_convolution(img, blur_kernel, blur_img);

    double A[PATCH_SIZE][MATRIX_SIZE] = {0};
    int A_row = 0;
    for (int j = -R; j <= R; j++) {
        for (int i = -R; i <= R; i++) {
            if (mask[j + R][i + R] >= 1e-6) {
                A[A_row][0] = i * i;
                A[A_row][1] = j * j;
                A[A_row][2] = i * j;
                A[A_row][3] = i;
                A[A_row][4] = j;
                A[A_row][5] = 1;
                A_row++;
            }
        }
    }

    double AtA[MATRIX_SIZE][MATRIX_SIZE];
    double AtA_inv[MATRIX_SIZE][MATRIX_SIZE];
    inverse_matrix_6x6(AtA, AtA_inv);

    int choose[MAX_CORNERS] = {0};
    Corner2 corners_out;
    corners_out.Size = 0;

    for (int i = 0; i < corners->Size; i++) {
        double u_cur = corners->p[i].x;
        double v_cur = corners->p[i].y;
        int is_saddle_point = 1;

        for (int num_it = 0; num_it < max_iteration; num_it++) {
            if (u_cur - R < 0 || u_cur + R >= WIDTH - 1 || v_cur - R < 0 || v_cur + R >= HEIGHT - 1) {
                is_saddle_point = 0;
                break;
            }

            double b[PATCH_SIZE] = {0};
            int num_valid;
            get_image_patch_with_mask(blur_img, mask, u_cur, v_cur, R, b, &num_valid);

            double b_trimmed[MATRIX_SIZE] = {0}; // 최소한으로 사용할 데이터
            for (int k = 0; k < MATRIX_SIZE && k < num_valid; k++) {
                b_trimmed[k] = b[k];
            }

            double k[MATRIX_SIZE];
            multiply_matrix_vector(AtA_inv, b_trimmed, k);

            double det = 4 * k[0] * k[1] - k[2] * k[2];
            if (det > 0) {
                is_saddle_point = 0;
                break;
            }

            double dx = (k[2] * k[4] - 2 * k[1] * k[3]) / det;
            double dy = (k[2] * k[3] - 2 * k[0] * k[4]) / det;

            u_cur += dx;
            v_cur += dy;

            if (sqrt(dx * dx + dy * dy) <= eps) {
                break;
            }
        }

        if (is_saddle_point) {
            corners_out.p[corners_out.Size] = corners->p[i];
            corners_out.Size++;
        }
    }

    *corners = corners_out;
}

// 메인 실행
int main() {
    double img[HEIGHT][WIDTH] = {0};
    Corner2 corners;
    corners.Size = 0;

    polynomial_fit_saddle(img, &corners);
    return 0;
}