#include "tcp_uart_bridge.h"
#include "esphome/core/log.h"

namespace esphome {
namespace tcp_uart_bridge {

static const char *const TAG = "tcp_uart_bridge";
static const uint32_t IDLE_TIMEOUT_MS = 600000;   // flash mode left on by mistake: back to normal after 10 min

void TcpUartBridge::apply_(uint32_t baud, uart::UARTParityOptions parity) {
  for (auto *u : {this->rx_, this->tx_}) {
    u->set_baud_rate(baud);
    u->set_parity(parity);
    u->load_settings(false);
  }
  // load_settings() re-runs begin() on the hardware UART0 (the RX side), which
  // puts it back on GPIO1/GPIO3: swap it again onto GPIO15/GPIO13 (RX = GPIO13).
  Serial.swap();
  uint8_t c;
  while (this->rx_->available()) this->rx_->read_byte(&c);   // drop bytes framed with the old settings
}

void TcpUartBridge::start(uint32_t baud) {
  if (this->active_) return;
  this->apply_(baud, uart::UART_CONFIG_PARITY_EVEN);
  if (this->server_ == nullptr) this->server_ = new WiFiServer(this->port_);
  this->server_->begin();
  this->server_->setNoDelay(true);
  this->active_ = true;
  this->last_activity_ = millis();
  this->bytes_up_ = this->bytes_down_ = 0;
  ESP_LOGI(TAG, "flash mode ON: TCP port %u <-> UART %u 8E1", this->port_, (unsigned) baud);
}

void TcpUartBridge::stop() {
  if (!this->active_) return;
  if (this->client_) this->client_.stop();
  this->server_->stop();
  this->apply_(115200, uart::UART_CONFIG_PARITY_NONE);
  this->active_ = false;
  ESP_LOGI(TAG, "flash mode OFF (TCP->UART %u bytes, UART->TCP %u bytes): UART back to 115200 8N1",
           (unsigned) this->bytes_up_, (unsigned) this->bytes_down_);
}

void TcpUartBridge::loop() {
  if (!this->active_) return;
  uint32_t now = millis();
  if (now - this->last_activity_ > IDLE_TIMEOUT_MS) {
    ESP_LOGW(TAG, "flash mode idle for 10 min");
    this->stop();
    return;
  }
  if (this->server_->hasClient()) {
    if (this->client_) this->client_.stop();          // one client at a time: the newest wins
    this->client_ = this->server_->available();
    this->client_.setNoDelay(true);
    this->last_activity_ = now;
    ESP_LOGI(TAG, "client connected");
  }
  if (!this->client_ || !this->client_.connected()) return;

  uint8_t buf[128];
  int n = this->client_.available();                  // TCP -> UART (stm32flash -> ROM bootloader)
  while (n > 0) {
    int k = this->client_.read(buf, n > (int) sizeof(buf) ? (int) sizeof(buf) : n);
    if (k <= 0) break;
    this->tx_->write_array(buf, k);
    this->bytes_up_ += k;
    n -= k;
    this->last_activity_ = now;
  }
  size_t m = this->rx_->available();                  // UART -> TCP (ACK / data back)
  while (m > 0) {
    size_t k = m > sizeof(buf) ? sizeof(buf) : m;
    if (!this->rx_->read_array(buf, k)) break;
    this->client_.write(buf, k);
    this->bytes_down_ += k;
    m -= k;
    this->last_activity_ = now;
  }
}

}  // namespace tcp_uart_bridge
}  // namespace esphome
