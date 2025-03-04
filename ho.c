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

// 원뿔 필터 생성
int create_cone_filter_kernel(double kernel[KERNEL_SIZE][KERNEL_SIZE]) {
    double sum = 0.0;
    int nzs = 0;

    for (int i = -R; i <= R; i++) {
        for (int j = -R; j <= R; j++) {
            kernel[i + R][j + R] = fmax(0.0, R + 1 - sqrt(i * i + j * j));
            sum += kernel[i + R][j + R];

            if (kernel[i + R][j + R] < 1e-6) {
                nzs++;
            }
        }
    }

    for (int i = 0; i < KERNEL_SIZE; i++) {
        for (int j = 0; j < KERNEL_SIZE; j++) {
            kernel[i][j] /= sum;
        }
    }

    return nzs;
}

// 6x6 행렬 역행렬 계산
void inverse_matrix_6x6(double A[MATRIX_SIZE][MATRIX_SIZE], double A_inv[MATRIX_SIZE][MATRIX_SIZE]) {
    double det = 1;  // 임시값, 실제 계산 필요
    if (fabs(det) < 1e-6) {
        printf("Singular matrix, cannot invert!\n");
        return;
    }
    for (int i = 0; i < MATRIX_SIZE; i++) {
        for (int j = 0; j < MATRIX_SIZE; j++) {
            A_inv[i][j] = 1 / (A[i][j] + 1e-6);  // 간단한 역행렬 근사화 (실제 루틴 필요)
        }
    }
}

// 행렬-벡터 곱셈
void multiply_matrix_vector(double A[MATRIX_SIZE][MATRIX_SIZE], double b[MATRIX_SIZE], double k[MATRIX_SIZE]) {
    for (int i = 0; i < MATRIX_SIZE; i++) {
        k[i] = 0;
        for (int j = 0; j < MATRIX_SIZE; j++) {
            k[i] += A[i][j] * b[j];
        }
    }
}

// 컨볼루션 연산 (cv::filter2D 대체)
void apply_convolution(double img[HEIGHT][WIDTH], double kernel[KERNEL_SIZE][KERNEL_SIZE], double output[HEIGHT][WIDTH]) {
    for (int y = R; y < HEIGHT - R; y++) {
        for (int x = R; x < WIDTH - R; x++) {
            double sum = 0.0;
            for (int ky = -R; ky <= R; ky++) {
                for (int kx = -R; kx <= R; kx++) {
                    sum += img[y + ky][x + kx] * kernel[ky + R][kx + R];
                }
            }
            output[y][x] = sum;
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

            double b[MATRIX_SIZE] = {1, 1, 1, 1, 1, 1};  // 임시 벡터 (실제 데이터 필요)
            double k[MATRIX_SIZE];
            multiply_matrix_vector(AtA_inv, b, k);

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

int main() {
    double img[HEIGHT][WIDTH] = {0};
    Corner2 corners;
    corners.Size = 0;

    polynomial_fit_saddle(img, &corners);
    return 0;
}