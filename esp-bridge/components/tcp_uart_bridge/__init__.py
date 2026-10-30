# tcp_uart_bridge - ESPHome external component (ESP8266, Arduino framework).
# Flash mode for the EVSE bridge: on demand, switches the two UART components
# (hardware RX, software TX) to 8E1 for the STM32 ROM bootloader and relays a
# TCP port to them, so stm32flash can rewrite the station's firmware remotely.
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID, CONF_PORT

DEPENDENCIES = ["wifi"]

tcp_uart_bridge_ns = cg.esphome_ns.namespace("tcp_uart_bridge")
TcpUartBridge = tcp_uart_bridge_ns.class_("TcpUartBridge", cg.Component)

CONF_RX_UART = "rx_uart"
CONF_TX_UART = "tx_uart"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(TcpUartBridge),
        cv.Required(CONF_RX_UART): cv.use_id(uart.UARTComponent),
        cv.Required(CONF_TX_UART): cv.use_id(uart.UARTComponent),
        cv.Optional(CONF_PORT, default=6638): cv.port,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    rx = await cg.get_variable(config[CONF_RX_UART])
    tx = await cg.get_variable(config[CONF_TX_UART])
    cg.add(var.set_rx_uart(rx))
    cg.add(var.set_tx_uart(tx))
    cg.add(var.set_port(config[CONF_PORT]))
