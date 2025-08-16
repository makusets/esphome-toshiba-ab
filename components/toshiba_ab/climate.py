import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import climate, uart, binary_sensor, sensor, switch, text_sensor, template
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    CONF_HARDWARE_UART,
    CONF_BAUD_RATE,
    CONF_UPDATE_INTERVAL,
    CONF_MODE,
    CONF_FAN_MODE,
    CONF_SWING_MODE,
    CONF_TRIGGER_ID,
    DEVICE_CLASS_CONNECTIVITY,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
)

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["climate", "binary_sensor", "sensor", "switch"]
CODEOWNERS = ["@muxa"]

toshiba_ab_ns = cg.esphome_ns.namespace("toshiba_ab")

CONF_CONNECTED = "connected"
CONF_VENT = "vent"
CONF_FAILED_CRCS = "failed_crcs"

CONF_ON_DATA_RECEIVED = "on_data_received"
CONF_MASTER = "master"

CONF_AUTONOMOUS = "autonomous"

#AC Sensors addresses

CONF_SENSORS = "sensors"
CONF_ADDRESS = "address"
CONF_SCALE = "scale"
CONF_INTERVAL = "interval"

SENSOR_ITEM_SCHEMA = cv.Schema({
    cv.Required(CONF_ADDRESS): cv.uint8_t,                           # sensor ID to query via 0x17
    cv.Optional(CONF_SCALE,   default=1.0): cv.float_,               # scale factor
    cv.Optional(CONF_INTERVAL, default="5min"): cv.positive_time_period_milliseconds,
    cv.Required("sensor"): sensor.sensor_schema(),                   # standard sensor schema (name, unit_of_measurement, etc.)
})

ToshibaAbClimate =  toshiba_ab_ns.class_(
    "ToshibaAbClimate", climate.Climate, uart.UARTDevice, cg.Component
)

ToshibaAbVentSwitch =  toshiba_ab_ns.class_(
    "ToshibaAbVentSwitch", switch.Switch, cg.Component
)

ToshibaAbOnDataReceivedTrigger = toshiba_ab_ns.class_(
    "ToshibaAbOnDataReceivedTrigger", automation.Trigger.template()
)

CONFIG_SCHEMA = climate.CLIMATE_SCHEMA.extend(
    {
        cv.Optional(CONF_MASTER, default=0x00): cv.uint8_t,
    
        cv.GenerateID(): cv.declare_id(ToshibaAbClimate),
        cv.Optional(CONF_CONNECTED): binary_sensor.binary_sensor_schema(
            device_class = DEVICE_CLASS_CONNECTIVITY,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_VENT): cv.maybe_simple_value(
            switch.SWITCH_SCHEMA.extend(
                cv.Schema(
                    {
                        cv.GenerateID(): cv.declare_id(ToshibaAbVentSwitch),
                    }
                )
            ),
            key=CONF_NAME,
        ),
        cv.Optional(CONF_FAILED_CRCS): sensor.sensor_schema(
            accuracy_decimals=0,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_ON_DATA_RECEIVED): automation.validate_automation(
            {
               cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(ToshibaAbOnDataReceivedTrigger),
            }
        ),
        cv.Optional(CONF_AUTONOMOUS, default=False): cv.boolean,
        cv.Optional(CONF_SENSORS, default=[]): cv.ensure_list(SENSOR_ITEM_SCHEMA),
    }
).extend(uart.UART_DEVICE_SCHEMA).extend(cv.COMPONENT_SCHEMA)

def validate_uart(config):
    uart.final_validate_device_schema(
        "tcc_link", baud_rate=2400, require_rx=True, require_tx=False
    )(config)


FINAL_VALIDATE_SCHEMA = validate_uart

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])

    await cg.register_component(var, config)
    await climate.register_climate(var, config)
    await uart.register_uart_device(var, config)

    if CONF_MASTER in config:
        cg.add(var.set_master_address(config[CONF_MASTER]))

    if CONF_CONNECTED in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_CONNECTED])
        cg.add(var.set_connected_binary_sensor(sens))

    if CONF_FAILED_CRCS in config:
        sens = await sensor.new_sensor(config[CONF_FAILED_CRCS])
        cg.add(var.set_failed_crcs_sensor(sens))

    if CONF_VENT in config:
        sw = await switch.new_switch(config[CONF_VENT], var)
        cg.add(var.set_vent_switch(sw))

    if CONF_ON_DATA_RECEIVED in config:
        for on_data_received in config.get(CONF_ON_DATA_RECEIVED, []):
            data_trigger = cg.new_Pvariable(on_data_received[CONF_TRIGGER_ID], var)
            await automation.build_automation(
                data_trigger, [(cg.std_vector.template(cg.uint8), "x")], on_data_received
            )
    if CONF_AUTONOMOUS in config:
        cg.add(var.set_autonomous(config[CONF_AUTONOMOUS]))
    
    for item in config.get(CONF_SENSORS, []):
        sens = await sensor.new_sensor(item["sensor"])  # creates the Sensor with name/units/etc.
        addr = item[CONF_ADDRESS]
        scale = item[CONF_SCALE]
        interval_ms = item[CONF_INTERVAL]
        cg.add(var.add_polled_sensor(addr, scale, cg.uint32(interval_ms), sens))