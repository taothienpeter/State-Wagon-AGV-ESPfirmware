#include "es_ekf.hpp"
#include <math.h>
#include <string.h>

// Hàm chuẩn hóa góc về khoảng [-pi, pi]
float ES_EKF::normalizeAngle(float angle) {
    return atan2f(sinf(angle), cosf(angle));
}

void ES_EKF::init(float delta_t, float init_x, float init_y, float init_theta) {
    this->dt = delta_t;
    
    // Khởi tạo trạng thái ban đầu
    x_nom[0] = init_x;
    x_nom[1] = init_y;
    x_nom[2] = init_theta;
    x_nom[3] = 0.0f;
    x_nom[4] = 0.0f;
    x_nom[5] = 0.0f;

    // Khởi tạo ma trận hiệp phương sai P (eye(6) * 0.1)
    memset(P, 0, sizeof(P));
    for (int i = 0; i < 6; i++) {
        P[i][i] = 0.1f;
    }

    // Khởi tạo ma trận Q (đường chéo)
    Q[0] = 0.01f; Q[1] = 0.01f; Q[2] = 0.004f; 
    Q[3] = 0.1f;  Q[4] = 0.1f;  Q[5] = 0.05f;
}

// --------- B1: PREDICT ---------
void ES_EKF::predict(float delta_x, float delta_y, float delta_theta, float dt_step) {
    if (!isfinite(delta_x) || !isfinite(delta_y) || !isfinite(delta_theta)) {
        return;
    }
    float cur_dt = (dt_step > 0.0001f) ? dt_step : this->dt;
    delta_theta = normalizeAngle(delta_theta);

    // Cập nhật vận tốc
    x_nom[3] = delta_x / cur_dt;
    x_nom[4] = delta_y / cur_dt;
    x_nom[5] = delta_theta / cur_dt;
    
    // Cập nhật vị trí
    x_nom[0] += delta_x;
    x_nom[1] += delta_y;
    x_nom[2] += delta_theta;
    x_nom[2] = normalizeAngle(x_nom[2]);

    // Ma trận Jacobian F_err (Tối ưu hóa phép nhân thay vì nhân ma trận 6x6)
    // F_err = I(6) + cur_dt * [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1]
    // Tính P = F * P * F^T + Q
    float P_new[6][6];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            float val = P[i][j];
            if (i < 3) val += cur_dt * P[i + 3][j];
            if (j < 3) val += cur_dt * P[i][j + 3];
            if (i < 3 && j < 3) val += cur_dt * cur_dt * P[i + 3][j + 3];
            
            P_new[i][j] = val;
            if (i == j) P_new[i][j] += Q[i]; // Cộng Q vào đường chéo
        }
    }
    memcpy(P, P_new, sizeof(P));
}

// --------- B2 & B3: UPDATE CHỈ VỚI IMU ---------
void ES_EKF::updateIMU(float imu_theta) {
    if (!isfinite(imu_theta)) return;

    // Không có UWB: Chỉ cập nhật Theta (từ IMU)
    // H = [0, 0, 1, 0, 0, 0]
    
    float z_k = imu_theta;
    float y_tilde = z_k - x_nom[2];
    y_tilde = normalizeAngle(y_tilde);

    // S = H * P * H^T + R -> Rút gọn: S = P[2][2] + R
    float S = P[2][2] + R_imu;
    if (fabsf(S) < 1e-6f) return;
    
    // K = P * H^T / S -> Rút gọn: K là cột 2 của P chia cho S
    float K[6];
    for (int i = 0; i < 6; i++) {
        K[i] = P[i][2] / S;
    }

    // dx = K * y_tilde
    float dx[6];
    for (int i = 0; i < 6; i++) {
        dx[i] = K[i] * y_tilde;
    }

    // Cập nhật P = (I - KH) * P * (I - KH)' + K*R*K' (Joseph form)
    float I_minus_KH[6][6];
    memset(I_minus_KH, 0, sizeof(I_minus_KH));
    for (int i = 0; i < 6; i++) {
        I_minus_KH[i][i] = 1.0f;
        I_minus_KH[i][2] -= K[i]; // Vì H chỉ có phần tử 1 tại index 2
    }

    // P = I_minus_KH * P
    float P_temp[6][6];
    memset(P_temp, 0, sizeof(P_temp));
    for(int i=0; i<6; i++) {
        for(int j=0; j<6; j++) {
            P_temp[i][j] = P[i][j] - K[i] * P[2][j];
        }
    }

    // P = P_temp * I_minus_KH' + K*R*K'
    for(int i=0; i<6; i++) {
        for(int j=0; j<6; j++) {
            P[i][j] = P_temp[i][j] - P_temp[i][2] * K[j] + K[i] * R_imu * K[j];
        }
    }

    // Inject & Reset
    for (int i = 0; i < 6; i++) {
        x_nom[i] += dx[i];
    }
    x_nom[2] = normalizeAngle(x_nom[2]);
}

// --------- B2 & B3: UPDATE VỚI UWB VÀ IMU ---------
void ES_EKF::updateUWB(float uwb_x, float uwb_y, float uwb_theta) {
    if (!isfinite(uwb_x) || !isfinite(uwb_y) || !isfinite(uwb_theta)) return;
    // H = [I(3x3), zeros(3x3)]
    float y_tilde[3];
    y_tilde[0] = uwb_x - x_nom[0];
    y_tilde[1] = uwb_y - x_nom[1];
    y_tilde[2] = normalizeAngle(uwb_theta - x_nom[2]);

    // S = H * P * H^T + R -> Rút gọn S chính là khối 3x3 góc trên trái của P + R
    float S[3][3];
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            S[i][j] = P[i][j];
            if(i==j) S[i][j] += R_uwb[i];
        }
    }

    // S_inv (Nghịch đảo ma trận 3x3)
    float S_inv[3][3];
    if(!invertMatrix3x3(S, S_inv)) return; // Tránh lỗi chia cho 0

    // K = P * H^T * S_inv
    // P * H^T chính là khối 6x3 bên trái của P
    float K[6][3];
    memset(K, 0, sizeof(K));
    for(int i=0; i<6; i++) {
        for(int j=0; j<3; j++) {
            for(int k=0; k<3; k++) {
                K[i][j] += P[i][k] * S_inv[k][j];
            }
        }
    }

    // dx = K * y_tilde
    float dx[6] = {0};
    for(int i=0; i<6; i++) {
        for(int j=0; j<3; j++) {
            dx[i] += K[i][j] * y_tilde[j];
        }
    }

    // Cập nhật P = (I - KH) * P * (I - KH)' + K*R*K'
    // Do H = [I, 0], K*H là lấy ma trận K làm 3 cột đầu của ma trận 6x6, 3 cột sau là 0.
    float I_minus_KH[6][6];
    memset(I_minus_KH, 0, sizeof(I_minus_KH));
    for(int i=0; i<6; i++) {
        I_minus_KH[i][i] = 1.0f;
        for(int j=0; j<3; j++) {
            I_minus_KH[i][j] -= K[i][j];
        }
    }

    float P_temp[6][6];
    matrixMult6x6(I_minus_KH, P, P_temp); // P_temp = I_minus_KH * P
    
    float I_minus_KH_T[6][6];
    matrixTranspose6x6(I_minus_KH, I_minus_KH_T); // Tính (I-KH)'
    
    float P_new[6][6];
    matrixMult6x6(P_temp, I_minus_KH_T, P_new); // P_new = (I-KH)*P*(I-KH)'

    // Cộng thêm K*R*K'
    for(int i=0; i<6; i++) {
        for(int j=0; j<6; j++) {
            float krk = 0;
            for(int k=0; k<3; k++) {
                krk += K[i][k] * R_uwb[k] * K[j][k];
            }
            P_new[i][j] += krk;
        }
    }
    memcpy(P, P_new, sizeof(P));

    // Inject & Reset
    for (int i = 0; i < 6; i++) {
        x_nom[i] += dx[i];
    }
    x_nom[2] = normalizeAngle(x_nom[2]);
}

// ---------------- CÁC HÀM MATRIX HELPER ----------------
void ES_EKF::matrixMult6x6(const float A[6][6], const float B[6][6], float C[6][6]) {
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            C[i][j] = 0;
            for (int k = 0; k < 6; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

void ES_EKF::matrixTranspose6x6(const float A[6][6], float A_T[6][6]) {
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            A_T[i][j] = A[j][i];
        }
    }
}

bool ES_EKF::invertMatrix3x3(const float m[3][3], float invOut[3][3]) {
    float det = m[0][0] * (m[1][1] * m[2][2] - m[2][1] * m[1][2]) -
                m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

    if (fabsf(det) < 1e-6f) return false;

    float invdet = 1.0f / det;
    invOut[0][0] = (m[1][1] * m[2][2] - m[2][1] * m[1][2]) * invdet;
    invOut[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * invdet;
    invOut[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invdet;
    invOut[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * invdet;
    invOut[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invdet;
    invOut[1][2] = (m[1][0] * m[0][2] - m[0][0] * m[1][2]) * invdet;
    invOut[2][0] = (m[1][0] * m[2][1] - m[2][0] * m[1][1]) * invdet;
    invOut[2][1] = (m[2][0] * m[0][1] - m[0][0] * m[2][1]) * invdet;
    invOut[2][2] = (m[0][0] * m[1][1] - m[1][0] * m[0][1]) * invdet;

    return true;
}