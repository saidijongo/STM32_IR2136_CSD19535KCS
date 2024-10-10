#include "main.h"
#include "math.h"

// Motor parameters (to be updated based on your motor)
#define R 0.1       // Motor resistance (ohms)
#define L 0.001     // Motor inductance (Henries)
#define J 0.01      // Motor inertia (kg.m^2)
#define K_e 0.01    // Back-EMF constant (V/rad/s)
#define K_t 0.01    // Torque constant (Nm/A)
#define B 0.0005    // Damping coefficient

// Ziegler-Nichols tuning parameters (obtained from testing or estimation)
#define Ku 50.0     // Ultimate gain (this should be experimentally obtained)
#define Tu 0.1      // Ultimate period (this should be experimentally obtained)

// PI controller variables
float Kp = 0;
float Ki = 0;

void calculate_PI_params() {
    float tau_e = L / R;  // Electrical time constant
    float tau_m = J / B;  // Mechanical time constant

    // Calculate basic Kp and Ki based on motor parameters
    Kp = (K_e * K_t) / tau_e;
    Ki = Kp / tau_m;

    // Apply Ziegler-Nichols tuning for PI control
    Kp = 0.45 * Ku;
    Ki = (1.2 * Kp) / Tu;

    // Now Kp and Ki are tuned using Ziegler-Nichols method
}

void foc_control(uint32_t I_A, uint32_t I_B, uint32_t hall_position, float power) {
    calculate_PI_params();  // Automatically calculate PI values

    // Clarke and Park Transform
    float I_alpha = I_A;
    float I_beta = (I_A + 2 * I_B) / sqrtf(3);
    float I_d = I_alpha * cosf(theta) + I_beta * sinf(theta);
    float I_q = -I_alpha * sinf(theta) + I_beta * cosf(theta);

    // PI controllers
    float V_d = Kp * I_d;   // Proportional control for I_d
    float V_q = Kp * I_q + (Ki * I_q);   // PI control for I_q

    // Inverse Park Transform
    float V_alpha = V_d * cosf(theta) - V_q * sinf(theta);
    float V_beta = V_d * sinf(theta) + V_q * cosf(theta);

    // Space Vector Modulation (SVPWM)
    calculate_svpwm(V_alpha, V_beta);
}
