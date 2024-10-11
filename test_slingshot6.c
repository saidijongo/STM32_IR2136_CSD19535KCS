/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "math.h"
#include "foc_math.h"  // Include your FOC algorithm functions
#include "utils_math.h"

// Constants for power calculation
#define MAX_POWER 1000.0f   // Maximum motor power (adjust as needed)
#define MAX_TORQUE 100.0f   // Maximum torque value from sensor (example value)
#define MAX_THROTTLE_INPUT 4095.0f  // Max value for throttle input

// Global variables for FOC, PAS, and sensors
float theta = 0;
volatile uint32_t cadence_pulses = 0;
volatile uint32_t last_cadence_time = 0;
volatile uint32_t current_cadence = 0;  // RPM
volatile uint8_t pas_level = 1;  // PAS level (1 to 5)
volatile float throttle_value = 0;  // Throttle value (0 to 1)

// PI controller variables
float Kp = 0;
float Ki = 0;

// Observer state structure (to store observer data)
observer_state obs_state;

// Motor state structure (for motor parameters)
motor_all_state_t motor_state;

// Function prototypes
void foc_control(uint32_t I_A, uint32_t I_B, uint32_t hall_position, float power);
void calculate_svpwm(float V_alpha, float V_beta);
uint32_t read_hall_sensors(void);
float pi_controller_d(float I_d);
float pi_controller_q(float I_q);
uint32_t read_torque_sensor(void);
float calculate_power(float torque, uint32_t cadence, uint8_t pas_level, float throttle);
void calculate_theta(uint32_t hall_state);
void foc_observer_update(float v_alpha, float v_beta, float i_alpha, float i_beta, float dt);
void foc_svm(float v_alpha, float v_beta, uint32_t PWMFullDutyCycle, uint32_t* tAout, uint32_t* tBout, uint32_t* tCout);
void calculate_PI_params();  // For Ziegler-Nichols tuning

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_ADC4_Init(void);
static void MX_USART2_UART_Init(void);

int main(void) {
    /* Initialize all configured peripherals */
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM1_Init();  // PWM for high-side and low-side
    MX_ADC1_Init();  // Current sensing (Phase A)
    MX_ADC2_Init();  // Current sensing (Phase B)
    MX_ADC3_Init();  // Throttle (Potentiometer)
    MX_ADC4_Init();  // Torque sensor
    MX_USART2_UART_Init();  // Serial communication for PAS levels

    // Initialize observer and motor state
    memset(&obs_state, 0, sizeof(observer_state));
    memset(&motor_state, 0, sizeof(motor_all_state_t));

    // Start PWM and ADCs
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    
    HAL_ADC_Start(&hadc1);
    HAL_ADC_Start(&hadc2);
    HAL_ADC_Start(&hadc3);
    HAL_ADC_Start(&hadc4);

    while (1) {
        // Read current and Hall sensor inputs
        uint32_t current_A = HAL_ADC_GetValue(&hadc1);
        uint32_t current_B = HAL_ADC_GetValue(&hadc2);
        uint32_t hall_position = read_hall_sensors();
        
        // Read torque and cadence
        uint32_t torque = read_torque_sensor();
        // Read throttle value
        HAL_ADC_Start(&hadc3);
        HAL_ADC_PollForConversion(&hadc3, HAL_MAX_DELAY);
        throttle_value = HAL_ADC_GetValue(&hadc3) / MAX_THROTTLE_INPUT;  // Normalized throttle (0 to 1)
        
        // Calculate power from cadence, torque, PAS level, and throttle
        float power = calculate_power(torque, current_cadence, pas_level, throttle_value);
        
        // Apply FOC control based on calculated power
        foc_control(current_A, current_B, hall_position, power);

        // Small delay
        HAL_Delay(100);
    }
}

/* ------------------------------ FOC Control ------------------------------ */

void foc_control(uint32_t I_A, uint32_t I_B, uint32_t hall_position, float power) {
    float v_alpha = 0.0f;  // Replace with actual voltage alpha
    float v_beta = 0.0f;   // Replace with actual voltage beta

    // Update observer
    foc_observer_update(v_alpha, v_beta, I_A, I_B, 0.001f, &obs_state, &theta, &motor_state);

    // PI controllers for d-axis and q-axis currents
    float V_d = pi_controller_d(obs_state.i_alpha);  // Control d-axis current
    float V_q = pi_controller_q(obs_state.i_beta);   // Control q-axis current
    
    // Scale q-axis voltage by the power (throttle, PAS, etc.)
    V_q *= power;

    // Inverse Park Transform to calculate alpha-beta voltages
    float V_alpha = V_d * cosf(theta) - V_q * sinf(theta);
    float V_beta = V_d * sinf(theta) + V_q * cosf(theta);

    // Space Vector Modulation (SVM) to generate PWM signals
    uint32_t tAout, tBout, tCout;
    foc_svm(V_alpha, V_beta, 4096, &tAout, &tBout, &tCout);

    // Update PWM duty cycles for the 3 motor phases
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, tAout);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, tBout);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, tCout);
}

/* ------------------------------ Helper Functions ------------------------------ */

// Power calculation based on torque, cadence, PAS, and throttle
float calculate_power(float torque, uint32_t cadence, uint8_t pas_level, float throttle) {
    // Define maximum power for each PAS level (1 to 5)
    const float max_power_per_pas[5] = {200.0f, 400.0f, 600.0f, 800.0f, 1000.0f};  // Example values
    float max_power = max_power_per_pas[pas_level - 1];

    // Calculate the base power from torque and cadence
    float base_power = torque * cadence;

    // Apply smoothing to avoid sudden power bursts (slingshot effect)
    float smooth_power = base_power * throttle;  // Scale by throttle input (0 to 1)

    // Ensure the power does not exceed the maximum allowed by PAS level
    smooth_power = (smooth_power > max_power) ? max_power : smooth_power;

    return smooth_power;
}

// Implement the foc_observer_update function from your provided FOC algorithm
void foc_observer_update(float v_alpha, float v_beta, float i_alpha, float i_beta, float dt, observer_state *state, float *phase, motor_all_state_t *motor) {
    mc_configuration *conf_now = motor->m_conf;

    // Motor parameters
    float R = conf_now->foc_motor_r;
    float L = conf_now->foc_motor_l;
    float lambda = conf_now->foc_motor_flux_linkage;

    // Saturation compensation (based on your FOC code)
    if (conf_now->foc_sat_comp_mode == SAT_COMP_LAMBDA) {
        L = L * (state->lambda_est / lambda);
    } else if (conf_now->foc_sat_comp_mode == SAT_COMP_FACTOR) {
        float comp_fact = conf_now->foc_sat_comp * (motor->m_motor_state.i_abs_filter / conf_now->l_current_max);
        L -= L * comp_fact;
        lambda -= lambda * comp_fact;
    }

    // Temperature compensation
    if (conf_now->foc_temp_comp) {
        R = motor->m_res_temp_comp;
    }

    // Adjust inductance for saliency
    float ld_lq_diff = conf_now->foc_motor_ld_lq_diff;
    float id = motor->m_motor_state.id;
    float iq = motor->m_motor_state.iq;

    if (fabsf(id) > 0.1 || fabsf(iq) > 0.1) {
        L = L - ld_lq_diff / 2.0 + ld_lq_diff * SQ(iq) / (SQ(id) + SQ(iq));
    }

    float L_ia = L * i_alpha;
    float L_ib = L * i_beta;
    float R_ia = R * i_alpha;
    float R_ib = R * i_beta;
    float gamma_half = motor->m_gamma_now * 0.5;

    // Observer type selection
    switch (conf_now->foc_observer_type) {
        case FOC_OBSERVER_ORTEGA_ORIGINAL: {
            float err = SQ(lambda) - (SQ(state->x1 - L_ia) + SQ(state->x2 - L_ib));
            if (err > 0.0) {
                err = 0.0;
            }

            float x1_dot = v_alpha - R_ia + gamma_half * (state->x1 - L_ia) * err;
            float x2_dot = v_beta - R_ib + gamma_half * (state->x2 - L_ib) * err;

            state->x1 += x1_dot * dt;
            state->x2 += x2_dot * dt;
        } break;

        case FOC_OBSERVER_MXLEMMING:
            state->x1 += (v_alpha - R_ia) * dt - L * (i_alpha - state->i_alpha_last);
            state->x2 += (v_beta - R_ib) * dt - L * (i_beta - state->i_beta_last);

            if (conf_now->foc_observer_type == FOC_OBSERVER_MXLEMMING_LAMBDA_COMP) {
                float err = SQ(state->lambda_est) - (SQ(state->x1) + SQ(state->x2));
                state->lambda_est += 0.1 * gamma_half * state->lambda_est * -err * dt;
                utils_truncate_number(&(state->lambda_est), lambda * 0.3, lambda * 2.5);
                utils_truncate_number_abs(&(state->x1), state->lambda_est);
                utils_truncate_number_abs(&(state->x2), state->lambda_est);
            }
            break;

        default:
            break;
    }

    // Update current tracking for next iteration
    state->i_alpha_last = i_alpha;
    state->i_beta_last = i_beta;

    // Prevent NaN values
    UTILS_NAN_ZERO(state->x1);
    UTILS_NAN_ZERO(state->x2);

    // Prevent magnitude from getting too low (to stabilize angle estimation)
    float mag = NORM2_f(state->x1, state->x2);
    if (mag < (lambda * 0.5)) {
        state->x1 *= 1.1;
        state->x2 *= 1.1;
    }

    // Calculate the rotor angle (theta)
    if (phase) {
        *phase = utils_fast_atan2(state->x2 - L_ib, state->x1 - L_ia);
    }
}

// Space Vector PWM calculation (provided SVM function)
void foc_svm(float v_alpha, float v_beta, uint32_t PWMFullDutyCycle, uint32_t* tAout, uint32_t* tBout, uint32_t* tCout) {
    // Use the provided SVM logic from your FOC code
    uint32_t svm_sector;
    foc_svm(v_alpha, v_beta, PWMFullDutyCycle, tAout, tBout, tCout, &svm_sector);
}

// PI controller for d-axis
float pi_controller_d(float I_d) {
    static float integral_d = 0;
    float error = 0 - I_d;  // Reference value for I_d is typically 0 in FOC
    integral_d += error;
    return Kp * error + Ki * integral_d;
}

// PI controller for q-axis
float pi_controller_q(float I_q) {
    static float integral_q = 0;
    float error = 0 - I_q;
    integral_q += error;
    return Kp * error + Ki * integral_q;
}

// Hall sensor reading
uint32_t read_hall_sensors(void) {
    // Your existing hall sensor reading code
    uint32_t hall_state = 0;
    hall_state |= HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) << 0;
    hall_state |= HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) << 1;
    hall_state |= HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) << 2;
    return hall_state;
}

// Read torque sensor
uint32_t read_torque_sensor(void) {
    HAL_ADC_Start(&hadc4);  
    HAL_ADC_PollForConversion(&hadc4, HAL_MAX_DELAY);
    uint32_t torque_value = HAL_ADC_GetValue(&hadc4);
    return torque_value;
}

// PAS cadence sensor interrupt handler
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == PAS_CADENCE_PIN) {  // Assuming PAS_CADENCE_PIN is defined
        cadence_pulses++;
        uint32_t now = HAL_GetTick();
        if (now - last_cadence_time >= 1000) {  // Calculate every second
            current_cadence = (cadence_pulses * 60) / 12;  // RPM with 12 magnets
            cadence_pulses = 0;
            last_cadence_time = now;
        }
    }
}

/* ------------------------------ Ziegler-Nichols Tuning ------------------------------ */

void calculate_PI_params() {
    // Electrical and mechanical time constants based on motor parameters
    float tau_e = L / R;
    float tau_m = J / B;

    // Ziegler-Nichols tuning for PI control
    Kp = 0.45 * Ku;  // Apply Ziegler-Nichols formula for proportional gain
    Ki = (1.2 * Kp) / Tu;  // Apply Ziegler-Nichols formula for integral gain
}

/* ------------------------------ STM32CubeMX Generated Code ------------------------------ */

// System Clock Configuration
void SystemClock_Config(void) {
    // Auto-generated code for system clock setup
}

// GPIO Initialization
static void MX_GPIO_Init(void) {
    // Auto-generated code for GPIO initialization (Hall sensors and other GPIOs)
}

// Timer 1 Initialization (PWM)
static void MX_TIM1_Init(void) {
    // Auto-generated code for PWM generation with dead-time insertion
}

// ADC1 Initialization (Phase A current)
static void MX_ADC1_Init(void) {
    // Auto-generated code for ADC1 initialization (Phase A current sensing)
}

// ADC2 Initialization (Phase B current)
static void MX_ADC2_Init(void) {
    // Auto-generated code for ADC2 initialization (Phase B current sensing)
}

// ADC3 Initialization (Throttle)
static void MX_ADC3_Init(void) {
    // Auto-generated code for ADC3 initialization (Throttle)
}

// ADC4 Initialization (Torque sensor)
static void MX_ADC4_Init(void) {
    // Auto-generated code for ADC4 initialization (Torque sensor)
}

// UART Initialization (for PAS level input via serial)
static void MX_USART2_UART_Init(void) {
    // Auto-generated code for UART initialization (serial communication)
}
