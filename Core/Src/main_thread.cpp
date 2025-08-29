#include <cstdio>

#include <halx/core.hpp>
#include <halx/driver/c6x0.hpp>
#include <halx/peripheral.hpp>

extern UART_HandleTypeDef huart1; // serial servo
extern UART_HandleTypeDef huart3; // stlink

extern FDCAN_HandleTypeDef hfdcan1;

static uint8_t dma_buf[8192];

extern "C" void main_thread(void *) {
  using namespace halx::peripheral;
  using namespace halx::driver;

  HAL_UART_DeInit(&huart1);
  HAL_UART_DeInit(&huart3);

  huart1.Init.BaudRate = 1000000;
  huart3.Init.BaudRate = 115200;

  HAL_UART_Init(&huart1);
  HAL_UART_Init(&huart3);

  std::span dma_span{dma_buf};
  Uart<&huart1, UartTxDma, UartRxDma> uart1{dma_span.subspan<512 * 0, 512>(), dma_span.subspan<512 * 1, 512>()};
  Uart<&huart3, UartTxDma, UartRxDma> uart3{dma_span.subspan<512 * 2, 512>(), dma_span.subspan<512 * 3, 512>()};

  Can<&hfdcan1> fdcan1;

  enable_stdout(uart3);

  C6x0Manager c6x0_manager{fdcan1};
  C6x0 c6x0{c6x0_manager, C6x0Type::C610, C6x0Id::ID_1};

  fdcan1.start();

  while (true) {
    c6x0_manager.update();

    printf("%d\r\n", c6x0.get_rpm());

    c6x0.set_current_ref(400.0f);

    c6x0_manager.transmit();

    halx::core::delay(10);
  }
}
