#include "main.h"
#include "math.h" // For cos, sin functions

// Function prototypes
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_USART2_UART_Init(void);

void foc_control(uint32_t I_A, uint32_t I_B, uint32_t hall_position, float throttle);
void calculate_svpwm(float V_alpha, float V_beta);
void read_hall_sensors(void);
void pi_controller_d(float I_d);
void pi_controller_q(float I_q);

// Global Variables
UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim1;
ADC_HandleTypeDef hadc1, hadc2, hadc3;

float theta = 0;  // Rotor position (angle)
uint32_t hall_position = 0;  // Hall sensor inputs
float throttle = 0;  // Throttle value for speed control
float motor_speed = 0;  // Actual motor speed

int main(void) {
    // HAL Initialization and clock setup
    HAL_Init();
    SystemClock_Config();
    
    // Initialize peripherals
    MX_GPIO_Init();
    MX_TIM1_Init();  // For PWM generation
    MX_ADC1_Init();  // For current sense
    MX_ADC2_Init();  // For current sense
    MX_ADC3_Init();  // For throttle input
    MX_USART2_UART_Init();  // For serial debugging
    
    // Start PWM generation and ADC conversions
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    
    HAL_ADC_Start(&hadc1);
    HAL_ADC_Start(&hadc2);
    HAL_ADC_Start(&hadc3);

    while (1) {
        // Read phase currents using ADC
        uint32_t current_A = HAL_ADC_GetValue(&hadc1);
        uint32_t current_B = HAL_ADC_GetValue(&hadc2);
        
        // Read throttle (potentiometer) value for speed control
        throttle = HAL_ADC_GetValue(&hadc3) / 4095.0;  // Normalize to 0-1
        
        // Read Hall sensor inputs to determine rotor position
        hall_position = read_hall_sensors();
        
        // Perform FOC algorithm
        foc_control(current_A, current_B, hall_position, throttle);
        
        // Print motor speed for debugging
        char buffer[50];
        sprintf(buffer, "Motor Speed: %f RPM\n", motor_speed);
        HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);
        
        HAL_Delay(100);  // Small delay for serial communication
    }
}

void foc_control(uint32_t I_A, uint32_t I_B, uint32_t hall_position, float throttle) {
    // Use Hall sensor inputs to estimate rotor angle (theta)
    switch (hall_position) {
        case 0b001: theta = 0; break;
        case 0b101: theta = M_PI / 3; break;
        case 0b100: theta = 2 * M_PI / 3; break;
        case 0b110: theta = M_PI; break;
        case 0b010: theta = 4 * M_PI / 3; break;
        case 0b011: theta = 5 * M_PI / 3; break;
    }
    
    // Clarke Transform (convert 3-phase to 2-phase system)
    float I_alpha = I_A;
    float I_beta = (I_A + 2 * I_B) / sqrt(3);
    
    // Park Transform (convert 2-phase to dq frame)
    float I_d = I_alpha * cos(theta) + I_beta * sin(theta);
    float I_q = -I_alpha * sin(theta) + I_beta * cos(theta);
    
    // Apply PI control to I_d and I_q
    float V_d = pi_controller_d(I_d);
    float V_q = pi_controller_q(I_q);
    
    // Adjust speed with throttle input
    V_q *= throttle;
    
    // Inverse Park Transform (convert dq back to 2-phase system)
    float V_alpha = V_d * cos(theta) - V_q * sin(theta);
    float V_beta = V_d * sin(theta) + V_q * cos(theta);
    
    // Space Vector Modulation to generate PWM signals
    calculate_svpwm(V_alpha, V_beta);
}

void calculate_svpwm(float V_alpha, float V_beta) {
    // Space Vector PWM calculation (simplified)
    // Calculate PWM duty cycles and output to MOSFET gates
    float T1, T2, T0;
    
    // Insert code for SVPWM generation
    // Output the calculated duty cycle to TIM1 channels
}

uint32_t read_hall_sensors(void) {
    // Read the Hall sensor GPIO inputs and return the binary result
    uint32_t hall_state = 0;
    
    // Assuming Hall sensors connected to GPIO pins
    hall_state |= HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) << 0;
    hall_state |= HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) << 1;
    hall_state |= HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) << 2;
    
    return hall_state;
}

float pi_controller_d(float I_d) {
    // Proportional-Integral (PI) controller for I_d
    // Insert tuning values for Kp and Ki
    static float integral_d = 0;
    float Kp_d = 0.1;
    float Ki_d = 0.01;
    
    float error = 0 - I_d;  // Reference for I_d is typically 0
    integral_d += error;
    
    return Kp_d * error + Ki_d * integral_d;
}

float pi_controller_q(float I_q) {
    // Proportional-Integral (PI) controller for I_q
    static float integral_q = 0;
    float Kp_q = 0.1;
    float Ki_q = 0.01;
    
    float error = 0 - I_q;  // Adjust I_q reference based on throttle
    integral_q += error;
    
    return Kp_q * error + Ki_q * integral_q;
}
