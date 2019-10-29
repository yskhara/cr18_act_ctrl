/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2018 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"


#include "led.hpp"

/* USER CODE BEGIN Includes */

#include <ros.h>
//#include <std_msgs/String.h>
//#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>

#include "stepper_position_ctrl.h"
#include "feet_ctrl.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void launcher_pitch_callback(const std_msgs::Int16& msg);
void feet_velocity_callback(const std_msgs::Float32MultiArray& msg);
void act_enable_callback(const std_msgs::Bool& msg);

void launcher_esc_callback(const std_msgs::Int16& msg);
void loader_servo_callback(const std_msgs::Int16& msg);
void arm_servo_callback(const std_msgs::Int16& msg);
void picker_esc_callback(const std_msgs::Int16& msg);

void disable_actuators(void);
void enable_actuators(void);
void on_shutdown_pressed(void);
void on_start_pressed(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

//StepperVelocityCtrl stepper_feet_a(GPIOA, GPIO_PIN_0, GPIOA, GPIO_PIN_1);
//StepperVelocityCtrl stepper_feet_b(GPIOA, GPIO_PIN_2, GPIOA, GPIO_PIN_3);
//StepperVelocityCtrl stepper_feet_c(GPIOA, GPIO_PIN_4, GPIOA, GPIO_PIN_5);
FeetCtrl feet_ctrl(GPIOA, GPIO_PIN_0, GPIO_PIN_2, GPIO_PIN_4, GPIOA, GPIO_PIN_1,
GPIO_PIN_3, GPIO_PIN_5);
StepperPositionCtrl stepper_lift(GPIOA, GPIO_PIN_6, GPIOA, GPIO_PIN_7);

enum class shutdown_status
    : uint8_t
    {
        operational = 0, soft_shutdown = 1, hard_shutdown = 2, recovering = 3,
};

ros::NodeHandle nh;
//std_msgs::UInt16 user_input_msg;
//ros::Publisher user_input_pub("user_input", &user_input_msg);
ros::Subscriber<std_msgs::Bool> act_enable_sub("act_enable", &act_enable_callback);
ros::Subscriber<std_msgs::Int16> launcher_pitch_sub("launcher_pitch", &launcher_pitch_callback);
ros::Subscriber<std_msgs::Float32MultiArray> feet_velocity("motor_cmd_vel", &feet_velocity_callback);

ros::Subscriber<std_msgs::Int16> launcher_esc_sub("launcher_esc", &launcher_esc_callback);
ros::Subscriber<std_msgs::Int16> loader_servo_sub("loader_servo", &loader_servo_callback);
ros::Subscriber<std_msgs::Int16> arm_servo_sub("arm_servo", &arm_servo_callback);
ros::Subscriber<std_msgs::Int16> picker_esc_sub("picker_esc", &picker_esc_callback);

std_msgs::Empty shutdown_input_msg;
std_msgs::Empty start_input_msg;
ros::Publisher shutdown_input_pub("shutdown_input", &shutdown_input_msg);
ros::Publisher start_input_pub("start_input", &start_input_msg);

volatile shutdown_status _shutdown_status = shutdown_status::soft_shutdown;

volatile bool is_actuators_enabled = false;
volatile bool is_node_enabled = false;

volatile bool is_shutdown_pressed = true;
volatile bool is_start_pressed = false;

constexpr unsigned int cmd_timeout = 100;     // in ms
unsigned int last_cmd_time = HAL_GetTick();

constexpr int servo_neutral = 1520;
constexpr int servo_range = 500;
constexpr int servo_min = servo_neutral - servo_range;
constexpr int servo_max = servo_neutral + servo_range;

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void)
{
    /* MCU Configuration----------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART1_UART_Init();
    MX_TIM1_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();

    //HAL_Delay(1000);

    HAL_TIM_Base_Start_IT(&htim3);
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_1);

    //__HAL_TIM_ENABLE()
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    TIM3->BDTR |= TIM_BDTR_MOE;

    TIM3->CCR1 = 1520;
    //TIM3->CCR1 = 2020;

    init_led();

    HAL_Delay(1000);

    /* Infinite loop */

    feet_ctrl.enable();
    float target[3] =
    { 1.0, 1.0, 1.0 };
    while (1)
    {

        GPIOC->BSRR = GPIO_BSRR_BR13;
        target[0] = 1.0 / 0.03;
        feet_ctrl.set_target((float *) target);
        TIM3->CCR1 = 1520;
        //HAL_Delay(2000);

        for(int i = 0; i < 100;i++)
        {
            //TIM1->CCR3 = i*25;
            //led_process();
            HAL_Delay(20);
        }

        GPIOC->BSRR = GPIO_BSRR_BS13;
        target[0] = -1.0 / (0.06 * M_PI);
        feet_ctrl.set_target((float *) target);
        TIM3->CCR1 = 2020;
        //HAL_Delay(2000);

        for(int i = 0; i < 100;i++)
        {
            //TIM1->CCR3 = (1000 - i*10);
            //led_process();
            HAL_Delay(20);
        }
    }

    nh.initNode();
    nh.advertise(shutdown_input_pub);
    nh.advertise(start_input_pub);

    nh.subscribe(act_enable_sub);
    nh.subscribe(launcher_pitch_sub);
    nh.subscribe(feet_velocity);

    nh.subscribe(launcher_esc_sub);
    nh.subscribe(loader_servo_sub);
    nh.subscribe(arm_servo_sub);
    nh.subscribe(picker_esc_sub);

    /* These uart interrupts halt any ongoing transfer if an error occurs, disable them */
    /* Disable the UART Parity Error Interrupt */
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_PE);
    /* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_ERR);

    constexpr unsigned int ctrl_interval = 50;
    unsigned int last_ctrl_time = HAL_GetTick();

    while (1)
    {
#if 1
        unsigned int now = HAL_GetTick();

        if (!is_node_enabled)
        {
            // do nothing for now
            if (nh.connected())
            {
                is_node_enabled = true;
            }
        }
        else if (!nh.connected())
        {
            disable_actuators();
            is_node_enabled = false;
        }
        else if (now - last_ctrl_time > ctrl_interval)
        {
#if 0
            if(now - last_cmd_time > cmd_timeout)
            {
                // timeout
                disable_actuators();
                is_node_enabled = false;
            }
#endif

            if ((_shutdown_status != shutdown_status::hard_shutdown) && ((GPIOB->IDR & GPIO_IDR_IDR3) == 0u))
            {
                is_shutdown_pressed = true;
            }

            if (is_shutdown_pressed)
            {
                _shutdown_status = shutdown_status::hard_shutdown;
                shutdown_input_pub.publish(&shutdown_input_msg);

                is_shutdown_pressed = false;
            }

            if (is_start_pressed)
            {
                if (_shutdown_status == shutdown_status::hard_shutdown)
                {
                    start_input_pub.publish(&start_input_msg);
                    _shutdown_status = shutdown_status::recovering;
                }

                is_start_pressed = false;
            }

            last_ctrl_time = HAL_GetTick();
        }

        nh.spinOnce();
#else
        GPIOC->BSRR = GPIO_PIN_13;
        stepper_feet[0].set_target(-16 * 200); //
        //while(1);
        //while (!stepper_feet[0].motion_completed())
        //;
        while(1)
        {
            HAL_Delay(1000);
            GPIOC->BRR = GPIO_PIN_13;
            HAL_Delay(1000);
            GPIOC->BSRR = GPIO_PIN_13;
        }

        stepper_feet[0].set_target(0);        //
        while (!stepper_feet[0].motion_completed())
        ;
        HAL_Delay(1000);
#endif
    }
}

void disable_actuators(void)
{
    GPIOA->BSRR = GPIO_BSRR_BS12;
    GPIOC->BSRR = GPIO_BSRR_BS13;

    GPIOB->BSRR = GPIO_BSRR_BR15;

    feet_ctrl.disable();
    stepper_lift.disable();

    is_actuators_enabled = false;
}

void enable_actuators(void)
{
    if (is_actuators_enabled)
    {
        return;
    }

    feet_ctrl.enable();
    stepper_lift.enable();

    GPIOA->BSRR = GPIO_BSRR_BR12;
    GPIOC->BSRR = GPIO_BSRR_BR13;

    //GPIOB->BSRR = GPIO_BSRR_BR15;

    is_actuators_enabled = true;
}

void on_shutdown_pressed(void)
{
    disable_actuators();
    is_shutdown_pressed = true;
}

void on_start_pressed(void)
{
    // do not enable actuators (yet)
    is_start_pressed = true;
}

void launcher_pitch_callback(const std_msgs::Int16& msg)
{
    if (!is_actuators_enabled)
    {
        return;
    }

    stepper_lift.set_target_position(msg.data);
}

void feet_velocity_callback(const std_msgs::Float32MultiArray& feet_vel_msg)
{
    if (!is_actuators_enabled)
    {
        return;
    }

    if (feet_vel_msg.data_length != 3)
    {
        return;
    }

    feet_ctrl.set_target(feet_vel_msg.data);

    last_cmd_time = HAL_GetTick();
}

void hand_cylinder_callback(const std_msgs::Bool& hand_cylinder_msg)
{
    if (!is_actuators_enabled)
    {
        return;
    }

    if (hand_cylinder_msg.data)
    {
        GPIOB->BSRR = GPIO_BSRR_BS15;
    }
    else
    {
        GPIOB->BSRR = GPIO_BSRR_BR15;
    }
}

int sanitize_servo(int a)
{
    if (servo_max < a)
    {
        return servo_max;
    }
    else if(a < servo_min)
    {
        return servo_min;
    }
    else
    {
        return a;
    }
}

void launcher_esc_callback(const std_msgs::Int16& msg)
{
    if (!is_actuators_enabled || msg.data < 0)
    {
        TIM3->CCR1 = 0;
        return;
    }
    TIM3->CCR1 = sanitize_servo(msg.data);
}

void loader_servo_callback(const std_msgs::Int16& msg)
{
    if (!is_actuators_enabled || msg.data < 0)
    {
        TIM3->CCR2 = 0;
        return;
    }
    TIM3->CCR2 = sanitize_servo(msg.data);
}

void arm_servo_callback(const std_msgs::Int16& msg)
{
    if (!is_actuators_enabled || msg.data < 0)
    {
        TIM3->CCR3 = 0;
        return;
    }
    TIM3->CCR3 = sanitize_servo(msg.data);
}

void picker_esc_callback(const std_msgs::Int16& msg)
{
    if (!is_actuators_enabled || msg.data < 0)
    {
        TIM3->CCR4 = 0;
        return;
    }
    TIM3->CCR4 = sanitize_servo(msg.data);
}

void act_enable_callback(const std_msgs::Bool& act_enable_msg)
{
    if (act_enable_msg.data)
    {
        // enable
        if (_shutdown_status == shutdown_status::recovering)
        {
            enable_actuators();
            _shutdown_status = shutdown_status::operational;
        }
    }
    else
    {
        // disable
        disable_actuators();

        if (_shutdown_status == shutdown_status::operational)
        {
            _shutdown_status = shutdown_status::soft_shutdown;
        }
    }
}

#if 0
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == htim4.Instance)
    {
        stepper_lift.tick();
    }
    else if (htim->Instance == htim3.Instance)
    {
        stepper_lift.calculate_profile();
    }
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == htim4.Instance)
    {
        stepper_lift.reset_step();
    }
}
#endif

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Initializes the CPU, AHB and APB busses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure the Systick interrupt time
     */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

    /**Configure the Systick
     */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72u;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000u;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 72u - 1u;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 4000u - 1u;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig={0};
    TIM_MasterConfigTypeDef sMasterConfig =
    { 0 };
    TIM_OC_InitTypeDef sConfigOC =
    { 0 };

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 72u - 1u;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 20000u - 1u;					// T = 20 ms; f=50Hz.
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 1000;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    //sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }
}

/* TIM3 init function */
static void MX_TIM4_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_OC_InitTypeDef sConfigOC;

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 72u - 1u;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 20u - 1u;                     // 72 MHz / 1440 -> 50 kHz
    //htim4.Init.Period = 50u - 1u;                         // 72 MHz / 3600 -> 20 kHz
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sConfigOC.OCMode = TIM_OCMODE_TIMING;
    sConfigOC.Pulse = 17u - 1u;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 921600;                         //460800;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}

/** 
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{
    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE()
    ;

    /* DMA interrupt init */
    /* DMA1_Channel4_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    /* DMA1_Channel5_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void)
{

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE()
    ;
    __HAL_RCC_GPIOD_CLK_ENABLE()
    ;
    __HAL_RCC_GPIOA_CLK_ENABLE()
    ;
    __HAL_RCC_GPIOB_CLK_ENABLE()
    ;

    GPIO_InitTypeDef sGPIOConfig;

    // PA0 thru 7 and 12: stepper driver signal output
    sGPIOConfig.Mode = GPIO_MODE_OUTPUT_PP;
    sGPIOConfig.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_12;
    sGPIOConfig.Pull = GPIO_NOPULL;
    sGPIOConfig.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &sGPIOConfig);

    // PC14: nES input, active-L
    sGPIOConfig.Mode = GPIO_MODE_IT_FALLING;
    sGPIOConfig.Pin = GPIO_PIN_14;
    sGPIOConfig.Pull = GPIO_PULLDOWN;
    sGPIOConfig.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &sGPIOConfig);

    //  PC15: START input, active-H
    sGPIOConfig.Mode = GPIO_MODE_IT_RISING;
    sGPIOConfig.Pin = GPIO_PIN_15;
    sGPIOConfig.Pull = GPIO_PULLDOWN;
    sGPIOConfig.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &sGPIOConfig);

    // PC13: onboard LED output
    sGPIOConfig.Mode = GPIO_MODE_OUTPUT_PP;
    sGPIOConfig.Pin = GPIO_PIN_13;
    sGPIOConfig.Pull = GPIO_NOPULL;
    sGPIOConfig.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &sGPIOConfig);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);

    HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
