
#include "main.h"
#include "math.h"

// Global variables for FOC, PAS, and sensors
float theta = 0;
volatile uint32_t cadence_pulses = 0;
volatile uint32_t last_cadence_time = 0;
volatile uint32_t current_cadence = 0;  // RPM
volatile uint8_t pas_level = 1;  // PAS level (1 to 5)
volatile float throttle_value = 0;  // Throttle value (0 to 1)

// Function prototypes
void foc_control(uint32_t I_A, uint32_t I_B, uint32_t hall_position, float power);
void calculate_svpwm(float V_alpha, float V_beta);
uint32_t read_hall_sensors(void);
float pi_controller_d(float I_d);
float pi_controller_q(float I_q);
uint32_t read_torque_sensor(void);
float calculate_power(float torque, uint32_t cadence, uint8_t pas_level, float throttle);
void calculate_theta(uint32_t hall_state);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);  // For PAS cadence sensor interrupt

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
        throttle_value = HAL_ADC_GetValue(&hadc3) / 4095.0f;  // Normalized throttle (0 to 1)
        
        // Power calculated from cadence, torque, PAS level, and throttle
        float power = calculate_power(torque, current_cadence, pas_level, throttle_value);
        
        // Apply FOC control based on calculated power
        foc_control(current_A, current_B, hall_position, power);

        // Small delay
        HAL_Delay(100);
    }
}

/* ------------------------------ FOC Control ------------------------------ */

void foc_control(uint32_t I_A, uint32_t I_B, uint32_t hall_position, float power) {
    calculate_theta(hall_position);

    // Clarke and Park Transform
    float I_alpha = I_A;
    float I_beta = (I_A + 2 * I_B) / sqrtf(3);
    float I_d = I_alpha * cosf(theta) + I_beta * sinf(theta);
    float I_q = -I_alpha * sinf(theta) + I_beta * cosf(theta);

    // PI controllers
    float V_d = pi_controller_d(I_d);
    float V_q = pi_controller_q(I_q);

    // Apply power scaling from PAS and throttle
    V_q *= power;

    // Inverse Park Transform
    float V_alpha = V_d * cosf(theta) - V_q * sinf(theta);
    float V_beta = V_d * sinf(theta) + V_q * cosf(theta);

    // Space Vector Modulation
    calculate_svpwm(V_alpha, V_beta);
}

/* ------------------------------ Helper Functions ------------------------------ */

// Space Vector PWM calculation
void calculate_svpwm(float V_alpha, float V_beta) {
    // Placeholder function: Implement SVPWM logic here
    // Set the duty cycles for the PWM channels based on V_alpha and V_beta
}

// Read Hall sensor inputs
uint32_t read_hall_sensors(void) {
    uint32_t hall_state = 0;
    hall_state |= HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) << 0;
    hall_state |= HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) << 1;
    hall_state |= HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) << 2;
    return hall_state;
}

// PI controller for I_d
float pi_controller_d(float I_d) {
    static float integral_d = 0;
    float Kp_d = 0.1f;
    float Ki_d = 0.01f;
    float error = 0 - I_d;
    integral_d += error;
    return Kp_d * error + Ki_d * integral_d;
}

// PI controller for I_q
float pi_controller_q(float I_q) {
    static float integral_q = 0;
    float Kp_q = 0.1f;
    float Ki_q = 0.01f;
    float error = 0 - I_q;
    integral_q += error;
    return Kp_q * error + Ki_q * integral_q;
}

// Rotor angle calculation from Hall sensors
void calculate_theta(uint32_t hall_state) {
    switch (hall_state) {
        case 0b001: theta = 0; break;
        case 0b101: theta = M_PI / 3; break;
        case 0b100: theta = 2 * M_PI / 3; break;
        case 0b110: theta = M_PI; break;
        case 0b010: theta = 4 * M_PI / 3; break;
        case 0b011: theta = 5 * M_PI / 3; break;
        default: theta = 0;
    }
}

// Read torque sensor data (connected to ADC4)
uint32_t read_torque_sensor(void) {
    HAL_ADC_Start(&hadc4);  
    HAL_ADC_PollForConversion(&hadc4, HAL_MAX_DELAY);
    uint32_t torque_value = HAL_ADC_GetValue(&hadc4);
    return torque_value;
}

// Power calculation based on cadence, torque, PAS level, and throttle
float calculate_power(float torque, uint32_t cadence, uint8_t pas_level, float throttle) {
    // Power output scales with torque, cadence, PAS level, and throttle
    float power = torque * cadence * pas_level * throttle;
    return (power > MAX_POWER) ? MAX_POWER : power;  // Limit the maximum power
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

