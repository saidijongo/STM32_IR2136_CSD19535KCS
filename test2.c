
#include "main.h"
#include "math.h"  // For cos, sin functions

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
uint32_t read_hall_sensors(void);
float pi_controller_d(float I_d);
float pi_controller_q(float I_q);

// Global variables
UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim1;
ADC_HandleTypeDef hadc1, hadc2, hadc3;

float theta = 0;             // Rotor position (angle)
uint32_t hall_position = 0;   // Hall sensor inputs
float throttle = 0;           // Throttle value for speed control
float motor_speed = 0;        // Actual motor speed

int main(void) {
    // Initialize HAL and clock
    HAL_Init();
    SystemClock_Config();
    
    // Initialize GPIO, Timers, ADC, and UART
    MX_GPIO_Init();
    MX_TIM1_Init();  // For PWM generation (High-side and Low-side)
    MX_ADC1_Init();  // For current sense
    MX_ADC2_Init();  // For current sense
    MX_ADC3_Init();  // For throttle (potentiometer)
    MX_USART2_UART_Init();  // For serial debugging
    
    // Start PWM generation
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // High-side PWM
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);  // Low-side PWM (complementary)
    
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);  // High-side PWM
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);  // Low-side PWM (complementary)
    
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);  // High-side PWM
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);  // Low-side PWM (complementary)
    
    // Start ADC conversion for current sensing and throttle input
    HAL_ADC_Start(&hadc1);
    HAL_ADC_Start(&hadc2);
    HAL_ADC_Start(&hadc3);
    
    while (1) {
        // Read phase currents using ADC
        uint32_t current_A = HAL_ADC_GetValue(&hadc1);
        uint32_t current_B = HAL_ADC_GetValue(&hadc2);
        
        // Read throttle (potentiometer) value
        throttle = HAL_ADC_GetValue(&hadc3) / 4095.0f;  // Normalize throttle to 0-1
        
        // Read Hall sensor inputs to determine rotor position
        hall_position = read_hall_sensors();
        
        // Perform FOC algorithm
        foc_control(current_A, current_B, hall_position, throttle);
        
        // Print motor speed for debugging
        char buffer[50];
        sprintf(buffer, "Motor Speed: %f RPM\n", motor_speed);
        HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);
        
        HAL_Delay(100);  // Small delay for serial output
    }
}

// Field-Oriented Control (FOC) implementation
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
    float I_beta = (I_A + 2 * I_B) / sqrtf(3);
    
    // Park Transform (convert 2-phase to dq frame)
    float I_d = I_alpha * cosf(theta) + I_beta * sinf(theta);
    float I_q = -I_alpha * sinf(theta) + I_beta * cosf(theta);
    
    // Apply PI control to I_d and I_q
    float V_d = pi_controller_d(I_d);
    float V_q = pi_controller_q(I_q);
    
    // Adjust speed with throttle input
    V_q *= throttle;
    
    // Inverse Park Transform (convert dq back to 2-phase system)
    float V_alpha = V_d * cosf(theta) - V_q * sinf(theta);
    float V_beta = V_d * sinf(theta) + V_q * cosf(theta);
    
    // Space Vector Modulation to generate PWM signals
    calculate_svpwm(V_alpha, V_beta);
}

// Space Vector PWM calculation (simplified)
void calculate_svpwm(float V_alpha, float V_beta) {
    // Simplified Space Vector PWM (SVPWM) algorithm
    // Insert code for calculating T1, T2, and T0 for SVPWM
    
    // Update the PWM duty cycles based on the calculated values
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, calculated_pwm1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, calculated_pwm2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, calculated_pwm3);
}

// Read Hall sensor inputs
uint32_t read_hall_sensors(void) {
    uint32_t hall_state = 0;
    
    // Assuming Hall sensors are connected to GPIOB pins 0, 1, and 2
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
    
    float error = 0 - I_d;  // Reference I_d is typically 0 in FOC
    integral_d += error;
    
    return Kp_d * error + Ki_d * integral_d;
}

// PI controller for I_q
float pi_controller_q(float I_q) {
    static float integral_q = 0;
    float Kp_q = 0.1f;
    float Ki_q = 0.01f;
    
    float error = 0 - I_q;  // Adjust I_q reference based on throttle
    integral_q += error;
    
    return Kp_q * error + Ki_q * integral_q;
}

// Timer and ADC initialization functions (generated by STM32CubeMX)
static void MX_TIM1_Init(void) {
    // Timer setup code for TIM1 in PWM mode, including complementary PWM
    // Dead-time insertion is also configured here for shoot-through protection
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 4199;  // Set PWM frequency (adjust as needed)
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 2100;  // Set duty cycle (50%)
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }

    // Configure break and dead-time for TIM1 to protect the MOSFETs
    sBreakDeadTimeConfig.DeadTime = 50;  // 50ns dead-time (adjust based on your MOSFETs)
    
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) {
        Error_Handler();
    }

    // Start PWM generation
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);  // Start complementary PWM for low-side
}

// System Clock Configuration (Generated by STM32CubeMX)
void SystemClock_Config(void) {
    // Clock setup code here (provided by STM32CubeMX based on configuration)
}

static void MX_ADC1_Init(void) {
    // ADC1 initialization code for current sensing (provided by STM32CubeMX)
}

static void MX_ADC2_Init(void) {
    // ADC2 initialization code for current sensing (provided by STM32CubeMX)
}

static void MX_ADC3_Init(void) {
    // ADC3 initialization code for throttle control (provided by STM32CubeMX)
}

static void MX_USART2_UART_Init(void) {
    // UART initialization code for serial communication (provided by STM32CubeMX)
}

static void MX_GPIO_Init(void) {
    // GPIO initialization code (provided by STM32CubeMX)
}

