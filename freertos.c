/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
  /* USER CODE END Header */

  /* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "stdio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "i2c-lcd.h"
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>


#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */
__STATIC_INLINE void Configure_TIMPWMOutput(void);
void lcd_init(void);
void lcd_send_cmd(char cmd);
void lcd_send_data(char data);
void lcd_send_string(char* str);
void lcd_put_cur(int row, int col);
void lcd_clear(void);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
std_msgs__msg__Float32 msg;
std_msgs__msg__Float32 txt;
rcl_subscription_t subscriber;
rcl_publisher_t publisher;
unsigned char data[100];
float angle[3];
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[3000];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t)osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
bool cubemx_transport_open(struct uxrCustomTransport* transport);
bool cubemx_transport_close(struct uxrCustomTransport* transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t* buf, size_t len, uint8_t* err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void* microros_allocate(size_t size, void* state);
void microros_deallocate(void* pointer, void* state);
void* microros_reallocate(void* pointer, size_t size, void* state);
void* microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void* state);

void subscription_callback(const void* msgin);
void timer_callback(rcl_timer_t* timer, int64_t last_call_time);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void* argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of defaultTask */
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
  /* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void* argument)
{
    /* USER CODE BEGIN StartDefaultTask */
    /* Infinite loop */
    // micro-ROS configuration

    rmw_uros_set_custom_transport(
        true,
        (void*)&huart3,
        cubemx_transport_open,
        cubemx_transport_close,
        cubemx_transport_write,
        cubemx_transport_read);

    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate = microros_allocate;
    freeRTOS_allocator.deallocate = microros_deallocate;
    freeRTOS_allocator.reallocate = microros_reallocate;
    freeRTOS_allocator.zero_allocate = microros_zero_allocate;

    if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
        printf("Error on default allocators (line %d)\n", __LINE__);
    }

    // micro-ROS app
    rclc_support_t support;
    rcl_allocator_t allocator;
    rcl_node_t node;

    allocator = rcl_get_default_allocator();

    //create init_options
    rclc_support_init(&support, 0, NULL, &allocator);

    // create node
    rclc_node_init_default(&node, "cubemx_node", "", &support);

    // create publisher
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "cubemx_publisher");

    // Initialize a reliable subscriber
    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "cubemx_publisher");

    // create timer,
    rcl_timer_t timer;
    const unsigned int timer_timeout = 1000;
    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback);

    // create executor
      // rclc_executor_t executor;
    // executor = rclc_executor_get_zero_initialized_executor();
    // rclc_executor_init(&executor, &support.context, 1, &allocator);

    // create executor
    rclc_executor_t executor_pub;
    rclc_executor_init(&executor_pub, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor_pub, &timer);
    rclc_executor_t executor_sub;
    rclc_executor_init(&executor_sub, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor_sub, &subscriber, &txt, &subscription_callback, ON_NEW_DATA);

    txt.data = 0;
    msg.data = 0;
    Configure_TIMPWMOutput();

    for (;;)
    {
        rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(100));
        rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100));
        osDelay(20);
    }
    /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
// Implementation example:
void timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        HAL_UART_Receive_IT(&huart6, data, sizeof(data));
        for (int i = 0; i < 89; i++) {
            if (data[i] == 0x55) {
                switch (data[i + 1]) {
                case 0x53:
                    //angle[0] = ((float)((data[i+3]<<8)|data[i+2]))/32768*180; // X
                    //angle[1] = ((float)((data[i+5]<<8)|data[i+4]))/32768*180; // Y
                    angle[2] = ((float)((data[i + 7] << 8) | data[i + 6])) / 32768 * 180; // Z
                    LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_0);
                    msg.data = angle[2];
                    rcl_publish(&publisher, &msg, NULL);
                    break;
                    //rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
                }
            }
        }
    }
}

void subscription_callback(const void* msgin)
{
    // Cast received message to used type
    const std_msgs__msg__Float32* txt = (const std_msgs__msg__Float32*)msgin;
    float x = txt->data;
    lcd_init();
    float floatValue =x;
    int intPart = (int)floatValue;
    int decimalPart = (int)((floatValue - intPart) * 100); // 两位小数
    char buffer[100];
    sprintf(buffer, "%d.%02d", intPart, decimalPart);
    lcd_send_string("angle=");
    lcd_send_string(buffer);
    LL_mDelay(1000); 
    lcd_put_cur(1, 0);
    lcd_send_string("team 9");
    LL_mDelay(1000); 
    lcd_clear();

    if (x > 355 && x < 360) {
        LL_TIM_OC_SetCompareCH1(TIM4, ((LL_TIM_GetAutoReload(TIM4) + 1) * 70 / 100));
        LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_8);
        LL_TIM_OC_SetCompareCH2(TIM4, ((LL_TIM_GetAutoReload(TIM4) + 1) * 70 / 100));
        LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_9);
    }
    else if (x > 180 && x < 355) {
        LL_TIM_OC_SetCompareCH1(TIM4, ((LL_TIM_GetAutoReload(TIM4) + 1) * 30 / 100)); //331yes
        LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_8);
        LL_TIM_OC_SetCompareCH2(TIM4, ((LL_TIM_GetAutoReload(TIM4) + 1) * 70 / 100));
        LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_9);
    }
    else if (x > 5 && x < 180) { //23yes
        LL_TIM_OC_SetCompareCH1(TIM4, ((LL_TIM_GetAutoReload(TIM4) + 1) * 70 / 100));
        LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_8);
        LL_TIM_OC_SetCompareCH2(TIM4, ((LL_TIM_GetAutoReload(TIM4) + 1) * 30 / 100));
        LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_9);
    }
    else if (x > 0 && x < 5) {
        LL_TIM_OC_SetCompareCH1(TIM4, ((LL_TIM_GetAutoReload(TIM4) + 1) * 70 / 100));
        LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_8);
        LL_TIM_OC_SetCompareCH2(TIM4, ((LL_TIM_GetAutoReload(TIM4) + 1) * 70 / 100));
        LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_9);
    }
    else {
        LL_TIM_OC_SetCompareCH1(TIM4, ((LL_TIM_GetAutoReload(TIM4) + 1) * 70 / 100));
        LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_8);
        LL_TIM_OC_SetCompareCH2(TIM4, ((LL_TIM_GetAutoReload(TIM4) + 1) * 70 / 100));
        LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_9);
    }
    // Process message
    LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_7);

}
__STATIC_INLINE void  Configure_TIMPWMOutput(void)
{
    /*************************/
    /* GPIO AF configuration */
    /*************************/
    /* Enable the peripheral clock of GPIOs */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);


    LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
    LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
    LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_14;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
    LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
    LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /***********************************************/
    /* Configure the NVIC to handle TIM4 interrupt */
    /***********************************************/
    NVIC_SetPriority(TIM4_IRQn, 0);
    NVIC_DisableIRQ(TIM4_IRQn);

    /******************************/
    /* Peripheral clocks enabling */
    /******************************/
    /* Enable the timer peripheral clock */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);

    /***************************/
    /* Time base configuration */
    /***************************/
    /* Set counter mode */
    /* Reset value is LL_TIM_COUNTERMODE_UP */
    LL_TIM_SetCounterMode(TIM4, LL_TIM_COUNTERMODE_UP);

    /* Set the pre-scaler value to have TIM1 counter clock equal to 10 kHz */
    LL_TIM_SetPrescaler(TIM4, __LL_TIM_CALC_PSC(SystemCoreClock, 10000));

    /* Enable TIM1_ARR register preload. Writing to or reading from the         */
    /* auto-reload register accesses the preload register. The content of the   */
    /* preload register are transferred into the shadow register at each update */
    /* event (UEV).                                                             */
    LL_TIM_EnableARRPreload(TIM4);

    /* Set the auto-reload value to have a counter frequency of 100 Hz */
    /* TIM1CLK = SystemCoreClock / (APB prescaler & multiplier)               */
    uint32_t TimOutClock = SystemCoreClock / 2;
    LL_TIM_SetAutoReload(TIM4, __LL_TIM_CALC_ARR(TimOutClock, LL_TIM_GetPrescaler(TIM4), 100));

    /*********************************/
    /* Output waveform configuration */
    /*********************************/
    /* Set output mode */
    /* Reset value is LL_TIM_OCMODE_FROZEN */
    LL_TIM_OC_SetMode(TIM4, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetMode(TIM4, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetMode(TIM4, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetMode(TIM4, LL_TIM_CHANNEL_CH4, LL_TIM_OCMODE_PWM1);

    /* Set output channel polarity */
    /* Reset value is LL_TIM_OCPOLARITY_HIGH */
    LL_TIM_OC_SetPolarity(TIM4, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetPolarity(TIM4, LL_TIM_CHANNEL_CH2, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetPolarity(TIM4, LL_TIM_CHANNEL_CH3, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetPolarity(TIM4, LL_TIM_CHANNEL_CH4, LL_TIM_OCPOLARITY_HIGH);

    /* Set compare value to half of the counter period (50% duty cycle ) */
    LL_TIM_OC_SetCompareCH1(TIM4, 0);
    LL_TIM_OC_SetCompareCH2(TIM4, 0);
    LL_TIM_OC_SetCompareCH3(TIM4, 0);
    LL_TIM_OC_SetCompareCH4(TIM4, 0);

    /* Enable TIM1_CCR1 register preload. Read/Write operations access the      */
    /* preload register. TIM4_CCR1~4 preload value is loaded in the active        */
    /* at each update event.                                                    */
    LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH3);
    LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH4);

    /**************************/
    /* TIM4 interrupts set-up */
    /**************************/
    /* Enable the capture/compare interrupt for channel 1~4*/
    LL_TIM_EnableIT_CC1(TIM4);
    LL_TIM_EnableIT_CC2(TIM4);
    LL_TIM_EnableIT_CC3(TIM4);
    LL_TIM_EnableIT_CC4(TIM4);

    /**********************************/
    /* Start output signal generation */
    /**********************************/
    /* Enable output channel 1~4 */
    LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH4);

    /* Enable counter */
    LL_TIM_EnableCounter(TIM4);

    /* Force update generation */
    LL_TIM_GenerateEvent_UPDATE(TIM4);
}
/* USER CODE END Application */