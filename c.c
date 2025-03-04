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

// 원뿔 필터 커널 생성 함수
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

    // 정규화
    for (int i = 0; i < KERNEL_SIZE; i++) {
        for (int j = 0; j < KERNEL_SIZE; j++) {
            kernel[i][j] /= sum;
        }
    }

    return nzs;
}

// 행렬 전치 함수
void transpose_matrix(double src[PATCH_SIZE][MATRIX_SIZE], double dst[MATRIX_SIZE][PATCH_SIZE], int rows, int cols) {
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

// 6x6 행렬 역행렬 함수 (가정: 행렬이 가역적임)
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
void polynomial_fit_saddle(double img[HEIGHT][WIDTH], int width, int height, Corner2* corners) {
    int max_iteration = 5;
    double eps = 0.01;

    double blur_kernel[KERNEL_SIZE][KERNEL_SIZE];
    double mask[KERNEL_SIZE][KERNEL_SIZE];

    create_cone_filter_kernel(blur_kernel);
    int nzs = create_cone_filter_kernel(mask);

    int A_size = PATCH_SIZE - nzs;
    double A[PATCH_SIZE][MATRIX_SIZE];

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

    transpose_matrix(A, At, A_size, MATRIX_SIZE);
    multiply_matrices(At, A, AtA, MATRIX_SIZE, A_size, MATRIX_SIZE);

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
            if (u_cur - R < 0 || u_cur + R >= width - 1 || v_cur - R < 0 || v_cur + R >= height - 1) {
                is_saddle_point = 0;
                break;
            }

            double k[MATRIX_SIZE] = {0};
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
            choose[i] = 1;
            corners->p[i].x = u_cur;
            corners->p[i].y = v_cur;
        }
    }

    for (int i = 0; i < corners->Size; i++) {
        if (choose[i] == 1) {
            corners_out.p[corners_out.Size] = corners->p[i];
            corners_out.r[corners_out.Size] = corners->r[i];
            corners_out.v1[corners_out.Size] = corners->v1[i];
            corners_out.v2[corners_out.Size] = corners->v2[i];
            corners_out.v3[corners_out.Size] = corners->v3[i];
            corners_out.Score[corners_out.Size] = corners->Score[i];
            corners_out.Size++;
        }
    }

    *corners = corners_out;
}

int main() {
    double img[HEIGHT][WIDTH] = {0};
    Corner2 corners;
    corners.Size = 0;

    polynomial_fit_saddle(img, WIDTH, HEIGHT, &corners);

    return 0;
}