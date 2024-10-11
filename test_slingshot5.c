#include "main.h"
#include "math.h"

// Constants for power calculation
#define MAX_POWER 1000.0f   // Maximum motor power (adjust as needed)
#define MAX_PEAK_POWER 1600.0f   // Peak power at PAS 5
#define MAX_TORQUE 160.0f   // Maximum torque value from sensor (adjust based on the motor specs)
#define TORQUE_SMOOTHING 0.1f   // Smoothing factor to make torque response gradual

// Global variables for FOC, PAS, and sensors
float theta = 0;
volatile uint32_t cadence_pulses = 0;
volatile uint32_t last_cadence_time = 0;
volatile uint32_t current_cadence = 0;  // RPM
volatile uint8_t pas_level = 1;  // PAS level (1 to 5)
volatile float throttle_value = 0;  // Throttle value (0 to 1)
volatile float smoothed_torque = 0;  // Smoothed torque value

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
        // Apply torque smoothing to prevent slingshot effect
        smoothed_torque += TORQUE_SMOOTHING * ((float)torque - smoothed_torque);
        
        // Read throttle value
        HAL_ADC_Start(&hadc3);
        HAL_ADC_PollForConversion(&hadc3, HAL_MAX_DELAY);
        throttle_value = HAL_ADC_GetValue(&hadc3) / 4095.0f;  // Normalized throttle (0 to 1)
        
        // Calculate power from cadence, torque, PAS level, and throttle
        float power = calculate_power(smoothed_torque, current_cadence, pas_level, throttle_value);
        
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

// Read torque sensor
uint32_t read_torque_sensor(void) {
    HAL_ADC_Start(&hadc4);  
    HAL_ADC_PollForConversion(&hadc4, HAL_MAX_DELAY);
    uint32_t torque_value = HAL_ADC_GetValue(&hadc4);
    return torque_value;
}

// Power calculation based on torque, cadence, PAS, and throttle
float calculate_power(float torque, uint32_t cadence, uint8_t pas_level, float throttle) {
    // Scale power based on torque and cadence, considering PAS level and throttle
    float power = torque * cadence * pas_level * throttle;
    
    // Cap the power based on the PAS level
    if (pas_level == 5) {
        return (power > MAX_PEAK_POWER) ? MAX_PEAK_POWER : power;  // Peak power in PAS 5
    } else {
        return (power > MAX_POWER) ? MAX_POWER : power;  // Maximum power in lower PAS levels
    }
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
