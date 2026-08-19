#ifndef ES_EKF_H
#define ES_EKF_H

#include <stdint.h>

class ES_EKF {
public:
    // Trạng thái hệ thống: [x, y, theta, vx, vy, vtheta]
    float x_nom[6]; 
    
    // Khởi tạo bộ lọc với dt (thời gian lấy mẫu)
    void init(float dt, float init_x, float init_y, float init_theta);
    
    // Bước 1: Predict (Cập nhật từ Encoder)
    void predict(float delta_x, float delta_y, float delta_theta, float dt_step = 0.0f);
    
    // Bước 2 & 3: Update và Inject từ IMU (khi không có UWB)
    void updateIMU(float imu_theta);
    
    // Bước 2 & 3: Update và Inject từ UWB + IMU
    void updateUWB(float uwb_x, float uwb_y, float uwb_theta);

private:
    float dt;
    float P[6][6];
    float Q[6];
    
    // Thông số nhiễu R
    const float R_uwb[3] = {0.2f, 0.2f, 0.04f}; // R cho [x, y, theta]
    const float R_imu = 0.02f;                  // R cho theta
    
    // Các hàm toán học hỗ trợ nội bộ
    float normalizeAngle(float angle);
    void matrixMult6x6(const float A[6][6], const float B[6][6], float C[6][6]);
    void matrixAdd6x6(const float A[6][6], const float B[6][6], float C[6][6]);
    void matrixSub6x6(const float A[6][6], const float B[6][6], float C[6][6]);
    void matrixTranspose6x6(const float A[6][6], float A_T[6][6]);
    bool invertMatrix3x3(const float m[3][3], float invOut[3][3]);
};

#endif // ES_EKF_H