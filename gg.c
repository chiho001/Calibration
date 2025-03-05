#include <stdio.h>
#include <math.h>

#define MATRIX_SIZE 6
#define PATCH_SIZE 25  // (2*R+1)^2에서 nzs 제거 후 추정값

// 전치 행렬 계산
void transpose_matrix(double A[PATCH_SIZE][MATRIX_SIZE], double At[MATRIX_SIZE][PATCH_SIZE]) {
    for (int i = 0; i < PATCH_SIZE; i++) {
        for (int j = 0; j < MATRIX_SIZE; j++) {
            At[j][i] = A[i][j];
        }
    }
}

// 행렬 곱셈 (A * B = C)
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

// 6x6 행렬 역행렬 (가우스-조던 소거법)
void inverse_matrix_6x6(double A[MATRIX_SIZE][MATRIX_SIZE], double A_inv[MATRIX_SIZE][MATRIX_SIZE]) {
    double temp[MATRIX_SIZE][MATRIX_SIZE * 2];

    // (A | I) 확장 행렬 생성
    for (int i = 0; i < MATRIX_SIZE; i++) {
        for (int j = 0; j < MATRIX_SIZE; j++) {
            temp[i][j] = A[i][j];
            temp[i][j + MATRIX_SIZE] = (i == j) ? 1.0 : 0.0;
        }
    }

    // 가우스-조던 소거법
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

    // 역행렬 추출
    for (int i = 0; i < MATRIX_SIZE; i++) {
        for (int j = 0; j < MATRIX_SIZE; j++) {
            A_inv[i][j] = temp[i][j + MATRIX_SIZE];
        }
    }
}

// 최종 변환 함수
void compute_invAtAAt(double A[PATCH_SIZE][MATRIX_SIZE], double invAtAAt[MATRIX_SIZE][PATCH_SIZE]) {
    double At[MATRIX_SIZE][PATCH_SIZE];
    double AtA[MATRIX_SIZE][MATRIX_SIZE];
    double AtA_inv[MATRIX_SIZE][MATRIX_SIZE];

    // 1. Aᵀ 계산
    transpose_matrix(A, At);

    // 2. Aᵀ * A 계산
    multiply_matrices(At, A, AtA);

    // 3. (Aᵀ * A)⁻¹ 계산
    inverse_matrix_6x6(AtA, AtA_inv);

    // 4. (Aᵀ * A)⁻¹ * Aᵀ 계산
    multiply_matrices(AtA_inv, At, invAtAAt);
}

// 테스트 실행
int main() {
    double A[PATCH_SIZE][MATRIX_SIZE] = {
        {1, 2, 3, 4, 5, 6},
        {2, 3, 4, 5, 6, 7},
        {3, 4, 5, 6, 7, 8},
        {4, 5, 6, 7, 8, 9},
        {5, 6, 7, 8, 9, 10},
        {6, 7, 8, 9, 10, 11},
        {7, 8, 9, 10, 11, 12},
        {8, 9, 10, 11, 12, 13},
        {9, 10, 11, 12, 13, 14},
        {10, 11, 12, 13, 14, 15},
        {11, 12, 13, 14, 15, 16},
        {12, 13, 14, 15, 16, 17},
        {13, 14, 15, 16, 17, 18},
        {14, 15, 16, 17, 18, 19},
        {15, 16, 17, 18, 19, 20},
        {16, 17, 18, 19, 20, 21},
        {17, 18, 19, 20, 21, 22},
        {18, 19, 20, 21, 22, 23},
        {19, 20, 21, 22, 23, 24},
        {20, 21, 22, 23, 24, 25}
    };

    double invAtAAt[MATRIX_SIZE][PATCH_SIZE];

    compute_invAtAAt(A, invAtAAt);

    // 결과 출력
    printf("Computed invAtAAt matrix:\n");
    for (int i = 0; i < MATRIX_SIZE; i++) {
        for (int j = 0; j < PATCH_SIZE; j++) {
            printf("%f ", invAtAAt[i][j]);
        }
        printf("\n");
    }

    return 0;
}