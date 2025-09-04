#include <cstdio>

#include <halx/core.hpp>
#include <halx/driver/c6x0.hpp>
#include <halx/dynamixel/port_handler.hpp>
#include <halx/peripheral.hpp>

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

extern FDCAN_HandleTypeDef hfdcan1;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

static uint8_t uart1_tx_buf[512];
static uint8_t uart1_rx_buf[512];
static uint8_t uart3_tx_buf[512];
static uint8_t uart3_rx_buf[512];

extern "C" void main_thread(void *) {
  using namespace halx::peripheral;

  HAL_UART_DeInit(&huart1);
  HAL_UART_DeInit(&huart3);

  huart1.Init.BaudRate = 57600;
  huart3.Init.BaudRate = 115200;

  HAL_UART_Init(&huart1);
  HAL_UART_Init(&huart3);

  Uart<&huart1, UartTxDma, UartRxDma> uart1{uart1_tx_buf, uart1_rx_buf}; // serial servo
  Uart<&huart3, UartTxDma, UartRxDma> uart3{uart3_tx_buf, uart3_rx_buf}; // stlink

  Can<&hfdcan1> can1;

  Pwm servo1{&htim1, TIM_CHANNEL_3};
  Pwm servo2{&htim1, TIM_CHANNEL_1};
  Pwm servo3{&htim3, TIM_CHANNEL_3};
  Pwm servo4{&htim3, TIM_CHANNEL_1};
  Pwm servo5{&htim15, TIM_CHANNEL_1};

  Gpio air1{GPIOA, GPIO_PIN_9};
  Gpio air2{GPIOC, GPIO_PIN_9};
  Gpio air3{GPIOC, GPIO_PIN_7};
  Gpio air4{GPIOB, GPIO_PIN_15};
  Gpio air5{GPIOB, GPIO_PIN_13};

  Gpio led_r{GPIOA, GPIO_PIN_6};
  Gpio led_g{GPIOA, GPIO_PIN_7};
  Gpio led_b{GPIOA, GPIO_PIN_5};

  Exti<EXTI_LINE_15> limit_sw{EXTI_MODE_INTERRUPT, EXTI_TRIGGER_RISING, EXTI_GPIOA, 5, 0};

  Tim<&htim16> tim16; // 1kHz
  Tim<&htim17> tim17; // 10kHz

  enable_stdout(uart3);

  // ここより上はbaud rate以外触らない

  using namespace halx::driver;

  halx::dynamixel::PortHandler halx_dynamixel_port_handler{uart1};
  dynamixel::PortHandler *portHandler = &halx_dynamixel_port_handler;

  C6x0Manager c6x0_manager{can1};
  C6x0 c6x0{c6x0_manager, C6x0Type::C610, C6x0Id::ID_1};

  can1.start();

  while (true) {
    c6x0_manager.update();

    printf("%d\r\n", c6x0.get_rpm());

    c6x0.set_current_ref(400.0f);

    c6x0_manager.transmit();

    halx::core::delay(10);
  }
}
