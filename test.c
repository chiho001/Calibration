include <stdio.h>
#include <math.h>

#define WIDTH 100
#define HEIGHT 100
#define MAX_KERNEL_SIZE 10
#define MATRIX_SIZE 6
#define PATCH_SIZE ((2 * MAX_KERNEL_SIZE + 1) * (2 * MAX_KERNEL_SIZE + 1))

// 2D 점 구조체
typedef struct {
    double x, y;
} point2d;

// 코너 구조체
typedef struct {
    point2d p[WIDTH * HEIGHT];
    int r[WIDTH * HEIGHT];
    point2d v1[WIDTH * HEIGHT];
    point2d v2[WIDTH * HEIGHT];
    point2d v3[WIDTH * HEIGHT];
    double Score[WIDTH * HEIGHT];
    int Size;
} Corner2;

// 원뿔 필터 커널 생성 함수
int create_cone_filter_kernel(double kernel[MAX_KERNEL_SIZE][MAX_KERNEL_SIZE], int r) {
    double sum = 0.0;
    int nzs = 0;

    for (int i = -r; i <= r; i++) {
        for (int j = -r; j <= r; j++) {
            kernel[i + r][j + r] = fmax(0.0, r + 1 - sqrt(i * i + j * j));
            sum += kernel[i + r][j + r];

            if (kernel[i + r][j + r] < 1e-6) {
                nzs++;
            }
        }
    }

    // 정규화
    for (int i = 0; i < 2 * r + 1; i++) {
        for (int j = 0; j < 2 * r + 1; j++) {
            kernel[i][j] /= sum;
        }
    }

    return nzs;
}

// 행렬 전치 함수
void transpose_matrix(double src[MATRIX_SIZE][PATCH_SIZE], double dst[PATCH_SIZE][MATRIX_SIZE], int rows, int cols) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            dst[j][i] = src[i][j];
        }
    }
}

// 행렬 곱셈 함수
void multiply_matrices(double A[MATRIX_SIZE][PATCH_SIZE], double B[PATCH_SIZE][MATRIX_SIZE], double C[MATRIX_SIZE][MATRIX_SIZE], int A_rows, int A_cols, int B_cols) {
    for (int i = 0; i < A_rows; i++) {
        for (int j = 0; j < B_cols; j++) {
            C[i][j] = 0;
            for (int k = 0; k < A_cols; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

// 6x6 행렬 역행렬 함수
void inverse_matrix_6x6(double A[MATRIX_SIZE][MATRIX_SIZE], double A_inv[MATRIX_SIZE][MATRIX_SIZE]) {
    double det = 0;
    for (int i = 0; i < MATRIX_SIZE; i++) {
        det += (A[0][i] * (A[1][(i+1)%MATRIX_SIZE] * A[2][(i+2)%MATRIX_SIZE] - A[1][(i+2)%MATRIX_SIZE] * A[2][(i+1)%MATRIX_SIZE]));
    }

    if (fabs(det) < 1e-6) {
        printf("Singular matrix, cannot invert!\n");
        return;
    }

    for (int i = 0; i < MATRIX_SIZE; i++) {
        for (int j = 0; j < MATRIX_SIZE; j++) {
            A_inv[i][j] = (A[(j+1)%MATRIX_SIZE][(i+1)%MATRIX_SIZE] * A[(j+2)%MATRIX_SIZE][(i+2)%MATRIX_SIZE]) / det;
        }
    }
}

// Saddle point 검출을 위한 다항식 피팅
void polynomial_fit_saddle(double img[HEIGHT][WIDTH], int width, int height, int r, Corner2* corners) {
    int max_iteration = 5;
    double eps = 0.01;

    double blur_kernel[MAX_KERNEL_SIZE][MAX_KERNEL_SIZE];
    double mask[MAX_KERNEL_SIZE][MAX_KERNEL_SIZE];

    create_cone_filter_kernel(blur_kernel, r);
    int nzs = create_cone_filter_kernel(mask, r);

    int A_size = (2 * r + 1) * (2 * r + 1) - nzs;
    double A[PATCH_SIZE][MATRIX_SIZE];

    int A_row = 0;
    for (int j = -r; j <= r; j++) {
        for (int i = -r; i <= r; i++) {
            if (mask[j + r][i + r] >= 1e-6) {
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

    transpose_matrix(A, At, A_size, MATRIX_SIZE);
    multiply_matrices(At, A, AtA, MATRIX_SIZE, A_size, MATRIX_SIZE);

    double AtA_inv[MATRIX_SIZE][MATRIX_SIZE];
    inverse_matrix_6x6(AtA, AtA_inv);

    for (int i = 0; i < corners->Size; i++) {
        double u_cur = corners->p[i].x;
        double v_cur = corners->p[i].y;
        int is_saddle_point = 1;

        for (int num_it = 0; num_it < max_iteration; num_it++) {
            if (u_cur - r < 0 || u_cur + r >= width - 1 || v_cur - r < 0 || v_cur + r >= height - 1) {
                is_saddle_point = 0;
                break;
            }

            double k[MATRIX_SIZE];
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
            corners->p[i].x = u_cur;
            corners->p[i].y = v_cur;
        }
    }
}

int main() {
    double img[HEIGHT][WIDTH] = {0};
    Corner2 corners;
    corners.Size = 0;

    polynomial_fit_saddle(img, WIDTH, HEIGHT, 3, &corners);

    return 0;