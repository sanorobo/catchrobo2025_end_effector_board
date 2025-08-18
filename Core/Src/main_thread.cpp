#include <cstdio>

#include <halx/core.hpp>
#include <halx/peripheral.hpp>

#include <halx/driver/c6x0.hpp>

extern UART_HandleTypeDef huart1; // serial servo
extern UART_HandleTypeDef huart3; // stlink

extern FDCAN_HandleTypeDef hfdcan1;

extern "C" void main_thread(void *) {
  using namespace halx::peripheral;
  using namespace halx::driver;

  Uart<&huart1> uart1;
  Uart<&huart3> uart3;

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
