#pragma once
// tcp_uart_bridge - flash mode of the EVSE bridge (see __init__.py).
#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include <ESP8266WiFi.h>

namespace esphome {
namespace tcp_uart_bridge {

class TcpUartBridge : public Component {
 public:
  void set_rx_uart(uart::UARTComponent *u) { this->rx_ = u; }
  void set_tx_uart(uart::UARTComponent *u) { this->tx_ = u; }
  void set_port(uint16_t port) { this->port_ = port; }
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }
  void loop() override;

  // UARTs to <baud> 8E1 (STM32 ROM bootloader) and TCP server on port_.
  void start(uint32_t baud);
  // Back to 115200 8N1 (the station firmware's link), TCP server closed.
  void stop();
  bool active() const { return this->active_; }

 protected:
  void apply_(uint32_t baud, uart::UARTParityOptions parity);

  uart::UARTComponent *rx_{nullptr};
  uart::UARTComponent *tx_{nullptr};
  uint16_t port_{6638};
  bool active_{false};
  WiFiServer *server_{nullptr};
  WiFiClient client_;
  uint32_t last_activity_{0};
  uint32_t bytes_up_{0}, bytes_down_{0};
};

}  // namespace tcp_uart_bridge
}  // namespace esphome
