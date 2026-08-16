import esphome.codegen as cg
import esphome.config_validation as cv
import esphome.final_validate as fv
from esphome.components import climate, text_sensor, uart
from esphome.const import CONF_ID, CONF_NAME, CONF_NUMBER, CONF_RX_PIN, CONF_TX_PIN, CONF_UART_ID
from esphome.core import CORE

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["text_sensor"]
CODEOWNERS = ["@muxa"]

toshiba_ab_ns = cg.esphome_ns.namespace("toshiba_ab")
ToshibaAbClimate = toshiba_ab_ns.class_(
    "ToshibaAbClimate", climate.Climate, uart.UARTDevice, cg.Component
)
DiagnosticTextSensor = toshiba_ab_ns.class_("DiagnosticTextSensor", text_sensor.TextSensor)
Protocol = toshiba_ab_ns.enum("Protocol")
SystemType = toshiba_ab_ns.enum("SystemType")

CONF_MASTER_ADDRESS = "master_address"
CONF_ESP_ADDRESS = "esp_address"
CONF_FORMAT = "format"
CONF_SYSTEM_TYPE = "system_type"
CONF_DIAGNOSTIC_ID = "diagnostic_id"
CONF_HARDWARE_UART_RX_PIN = "hardware_uart_rx_pin"
AUTO_ADDRESS = 0xAA


def _address(value):
    if isinstance(value, str) and value.lower() == "auto":
        return AUTO_ADDRESS
    return cv.uint8_t(value)


FORMATS = {
    "auto": Protocol.AUTO,
    "tcc": Protocol.TCC,
    "tu2c": Protocol.TU2C,
    "a0": Protocol.A0,
}
SYSTEM_TYPES = {"air": SystemType.AIR, "water": SystemType.WATER}

CONFIG_SCHEMA = climate._CLIMATE_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(ToshibaAbClimate),
        cv.GenerateID(CONF_DIAGNOSTIC_ID): cv.declare_id(DiagnosticTextSensor),
        cv.Optional(CONF_MASTER_ADDRESS, default="auto"): _address,
        cv.Optional(CONF_ESP_ADDRESS, default="auto"): _address,
        cv.Optional(CONF_FORMAT, default="auto"): cv.enum(FORMATS, lower=True),
        cv.Optional(CONF_SYSTEM_TYPE, default="Air"): cv.enum(SYSTEM_TYPES, lower=True),
    }
).extend(uart.UART_DEVICE_SCHEMA).extend(cv.COMPONENT_SCHEMA)


def _pin_number(pin):
    return pin.get(CONF_NUMBER) if isinstance(pin, dict) else None


def _validate_uart(config):
    hardware_rx = None

    def inspect_hub(hub):
        nonlocal hardware_rx
        rx = _pin_number(hub.get(CONF_RX_PIN))
        tx = _pin_number(hub.get(CONF_TX_PIN))
        # Keep the existing ESP8266 UART0 GPIO13 swap. The UART hub remains
        # TX-only because this component owns the hardware RX FIFO.
        if CORE.is_esp8266 and rx == 13 and tx != 15:
            hardware_rx = 13
            hub.pop(CONF_RX_PIN, None)
        return hub

    cv.Schema(
        {cv.Required(CONF_UART_ID): fv.id_declaration_match_schema(inspect_hub)},
        extra=cv.ALLOW_EXTRA,
    )(config)
    uart.final_validate_device_schema(
        "toshiba_ab", baud_rate=2400, require_rx=hardware_rx is None, require_tx=False
    )(config)
    if hardware_rx is not None:
        config[CONF_HARDWARE_UART_RX_PIN] = hardware_rx
    return config


FINAL_VALIDATE_SCHEMA = _validate_uart


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await climate.register_climate(var, config)
    await uart.register_uart_device(var, config)
    cg.add(var.set_master_address(config[CONF_MASTER_ADDRESS]))
    cg.add(var.set_esp_address(config[CONF_ESP_ADDRESS]))
    cg.add(var.set_protocol(config[CONF_FORMAT]))
    cg.add(var.set_system_type(config[CONF_SYSTEM_TYPE]))
    if CONF_HARDWARE_UART_RX_PIN in config:
        cg.add(var.set_hardware_uart_rx_pin(config[CONF_HARDWARE_UART_RX_PIN]))

    diagnostic = cg.new_Pvariable(config[CONF_DIAGNOSTIC_ID])
    cg.add(diagnostic.set_name(f"{config[CONF_NAME]} Diagnostic"))
    cg.add(diagnostic.set_entity_category(cg.EntityCategory.ENTITY_CATEGORY_DIAGNOSTIC))
    cg.add(cg.App.register_text_sensor(diagnostic))
    cg.add(var.set_diagnostic_sensor(diagnostic))
