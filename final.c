#include <stdio.h>
#include <math.h>

#define WIDTH 100
#define HEIGHT 100
#define R 4  
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

// ✅ 행렬-벡터 곱셈
void multiply_matrix_vector(double A[MATRIX_SIZE][MATRIX_SIZE], double b[MATRIX_SIZE], double k[MATRIX_SIZE]) {
    for (int i = 0; i < MATRIX_SIZE; i++) {
        k[i] = 0;
        for (int j = 0; j < MATRIX_SIZE; j++) {
            k[i] += A[i][j] * b[j];
        }
    }
}

// ✅ 행렬 전치
void transpose_matrix(double A[PATCH_SIZE][MATRIX_SIZE], double At[MATRIX_SIZE][PATCH_SIZE]) {
    for (int i = 0; i < PATCH_SIZE; i++)
        for (int j = 0; j < MATRIX_SIZE; j++)
            At[j][i] = A[i][j];
}

// ✅ 행렬 곱셈
void multiply_matrices(double A[MATRIX_SIZE][PATCH_SIZE], double B[PATCH_SIZE][MATRIX_SIZE], double C[MATRIX_SIZE][MATRIX_SIZE]) {
    for (int i = 0; i < MATRIX_SIZE; i++) {
        for (int j = 0; j < MATRIX_SIZE; j++) {
            C[i][j] = 0;
            for (int k = 0; k < PATCH_SIZE; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

// ✅ 6x6 행렬 역행렬 (가우스-조던 소거법)
void inverse_matrix_6x6(double A[MATRIX_SIZE][MATRIX_SIZE], double A_inv[MATRIX_SIZE][MATRIX_SIZE]) {
    double temp[MATRIX_SIZE][MATRIX_SIZE * 2];

    for (int i = 0; i < MATRIX_SIZE; i++) {
        for (int j = 0; j < MATRIX_SIZE; j++) {
            temp[i][j] = A[i][j];
            temp[i][j + MATRIX_SIZE] = (i == j) ? 1.0 : 0.0;
        }
    }

    for (int i = 0; i < MATRIX_SIZE; i++) {
        double pivot = temp[i][i];
        if (fabs(pivot) < 1e-6) {
            printf("Singular matrix, cannot invert!\n");
            return;
        }
        for (int j = 0; j < MATRIX_SIZE * 2; j++) {
            temp[i][j] /= pivot;
        }
        for (int k = 0; k < MATRIX_SIZE; k++) {
            if (k != i) {
                double factor = temp[k][i];
                for (int j = 0; j < MATRIX_SIZE * 2; j++) {
                    temp[k][j] -= factor * temp[i][j];
                }
            }
        }
    }

    for (int i = 0; i < MATRIX_SIZE; i++) {
        for (int j = 0; j < MATRIX_SIZE; j++) {
            A_inv[i][j] = temp[i][j + MATRIX_SIZE];
        }
    }
}

// ✅ 이미지 패치 추출 (bilinear interpolation)
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

// ✅ create_cone_filter_kernel() 추가
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




// ✅ Saddle Point 검출 (polynomial_fit_saddle)
void polynomial_fit_saddle(double img[HEIGHT][WIDTH], Corner2* corners) {
    double blur_kernel[KERNEL_SIZE][KERNEL_SIZE];
    double mask[KERNEL_SIZE][KERNEL_SIZE];
    double blur_img[HEIGHT][WIDTH] = {0}; 

    create_cone_filter_kernel(blur_kernel);
    int nzs = create_cone_filter_kernel(mask);
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

    double At[MATRIX_SIZE][PATCH_SIZE];
    double AtA[MATRIX_SIZE][MATRIX_SIZE];
    double AtA_inv[MATRIX_SIZE][MATRIX_SIZE];
    double invAtAAt[MATRIX_SIZE][PATCH_SIZE];

    transpose_matrix(A, At);
    multiply_matrices(At, A, AtA);
    inverse_matrix_6x6(AtA, AtA_inv);
    multiply_matrices(AtA_inv, At, invAtAAt);

    int choose[MAX_CORNERS] = {0};
    Corner2 corners_out;
    corners_out.Size = 0;

    for (int i = 0; i < corners->Size; i++) {
        double u_cur = corners->p[i].x;
        double v_cur = corners->p[i].y;
        int is_saddle_point = 1;

        double b[PATCH_SIZE] = {0};
        int num_valid;
        get_image_patch_with_mask(img, mask, u_cur, v_cur, R, b, &num_valid);

        double k[MATRIX_SIZE];
        multiply_matrix_vector(invAtAAt, b, k);

        double det = 4 * k[0] * k[1] - k[2] * k[2];
        if (det > 0) {
            is_saddle_point = 0;
            break;
        }

        double dx = (k[2] * k[4] - 2 * k[1] * k[3]) / det;
        double dy = (k[2] * k[3] - 2 * k[0] * k[4]) / det;

        u_cur += dx;
        v_cur += dy;

        if (sqrt(dx * dx + dy * dy) <= 0.01) {
            break;
        }

        if (is_saddle_point) {
            choose[i] = 1;
            corners_out.p[corners_out.Size] = corners->p[i];
            corners_out.Size++;
        }
    }

    *corners = corners_out;
}

// 🛠 기존 212줄 코드 유지!

int main() {
    double img[HEIGHT][WIDTH] = {0};
    Corner2 corners;
    corners.Size = 0;

    polynomial_fit_saddle(img, &corners);
    return 0;
}
