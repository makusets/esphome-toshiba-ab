import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
import esphome.final_validate as fv
from esphome.components import climate, uart, binary_sensor, sensor, switch, text_sensor, template
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    CONF_SENSOR,
    CONF_BAUD_RATE,
    CONF_NUMBER,
    CONF_RX_PIN,
    CONF_TX_PIN,
    CONF_UPDATE_INTERVAL,
    CONF_MODE,
    CONF_FAN_MODE,
    CONF_SWING_MODE,
    CONF_TRIGGER_ID,
    CONF_UART_ID,
    DEVICE_CLASS_CONNECTIVITY,
    DEVICE_CLASS_DURATION,
    DEVICE_CLASS_TEMPERATURE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_CELSIUS,
    UNIT_HOUR,
)
from esphome.core import CORE

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["climate", "binary_sensor", "sensor", "switch"]
CODEOWNERS = ["@muxa"]

toshiba_ab_ns = cg.esphome_ns.namespace("toshiba_ab")

CONF_CONNECTED = "connected"
CONF_VENT = "vent"
CONF_READ_ONLY_SWITCH = "read_only_switch"
CONF_FAILED_CRCS = "failed_crcs"

CONF_ON_DATA_RECEIVED = "on_data_received"
CONF_MASTER = "master"
CONF_REMOTE = "remote"
CONF_MASTER_ADDRESS_AUTO = "master_address_auto"
CONF_COMMAND_MODE_READ = "command_mode_read"
CONF_COMMAND_MODE_WRITE = "command_mode_write"
CONF_FRAME_FORMAT = "frame_format"
CONF_FILTER_FRAMES = "filter_frames"
CONF_PACKET_MIN_WAIT = "packet_min_wait"
CONF_AUTORESET_ERRORS = "autoreset_errors"
CONF_FRAME = "frame"

CONF_AUTONOMOUS = "autonomous"
CONF_READ_ONLY = "read_only"
CONF_PING = "ping"

# Estia-specific sensors
CONF_OUTDOOR_TEMPERATURE = "outdoor_temperature"
CONF_COMPRESSOR_HOURS = "compressor_hours"
CONF_WATERPUMP_HOURS = "waterpump_hours"
CONF_BACKUP_HEATER_HOURS = "backup_heater_hours"
CONF_DEMAND = "demand"
CONF_DEMAND_ENABLED = "demand_enabled"
CONF_ZONE1_SWITCH = "zone1_switch"
CONF_DHW_BOOST = "dhw_boost"
CONF_ZONE1_WATER_TEMPERATURE = "zone1_water_temperature"
CONF_ZONE1_TARGET_TEMPERATURE = "zone1_target_temperature"
CONF_DHW_CURRENT_TEMPERATURE = "dhw_current_temperature"

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

ToshibaAbReadOnlySwitch = toshiba_ab_ns.class_(
    "ToshibaAbReadOnlySwitch", switch.Switch, cg.Component
)
ToshibaAbEstiaZone1Switch = toshiba_ab_ns.class_(
    "ToshibaAbEstiaZone1Switch", switch.Switch, cg.Component
)
ToshibaAbEstiaDhwBoostSwitch = toshiba_ab_ns.class_(
    "ToshibaAbEstiaDhwBoostSwitch", switch.Switch, cg.Component
)

ToshibaAbOnDataReceivedTrigger = toshiba_ab_ns.class_(
    "ToshibaAbOnDataReceivedTrigger", automation.Trigger.template()
)

ToshibaAbSendRawFrameAction = toshiba_ab_ns.class_(
    "ToshibaAbSendRawFrameAction", automation.Action
)

FrameFormat = toshiba_ab_ns.enum("FrameFormat")
FRAME_FORMATS = {
    "auto": None,
    "n": FrameFormat.NORMAL,
    "normal": FrameFormat.NORMAL,
    "u": FrameFormat.TU2C,
    "wrapped": FrameFormat.TU2C,
    "tu2c": FrameFormat.TU2C,
    "tcc-link": FrameFormat.NORMAL,
    "a0": FrameFormat.A0,
    "estia_a0": FrameFormat.A0,
    "hm": FrameFormat.HM,
    "estia": FrameFormat.ESTIA,
}


CONF_REPORT_SENSOR_TEMP = "report_sensor_temp" # Report external temperature (from any ESPHome sensor) to the AC
CONF_FILTER_ALERT = "filter_alert" #report filter alert status as binary sensor

REPORT_SENSOR_TEMP_SCHEMA = cv.Schema({
    cv.Optional("enabled", default=True): cv.boolean,
    cv.Required(CONF_SENSOR): cv.use_id(sensor.Sensor),
    cv.Optional(CONF_INTERVAL, default="5min"): cv.positive_time_period_milliseconds,
})


CONF_HARDWARE_UART_RX_PIN = "hardware_uart_rx_pin"
CONF_HARDWARE_UART_PARITY = "hardware_uart_parity"
CONF_PARITY = "parity"

HARDWARE_UART_PARITY = {
    "NONE": 0,
    "EVEN": 1,
    "ODD": 2,
}


def _hardware_uart_rx_pin(value):
    """Accept GPIO3 or GPIO13 — the only ESP8266 UART0 hardware RX pins.

    Lets an ESP8266 install whose RX is on a hardware-UART pin (but whose TX is
    not, so ESPHome would otherwise bit-bang both) move RX onto the real hardware
    UART. This removes the software-serial RX busy-wait that starves Wi-Fi and
    trips the watchdog on busy buses. TX stays on the configured uart tx_pin.
    """
    original = value
    if isinstance(value, str) and value.strip().upper().startswith("GPIO"):
        value = value.strip()[4:]
    try:
        num = int(value)
    except (TypeError, ValueError):
        raise cv.Invalid(
            f"{CONF_HARDWARE_UART_RX_PIN} must be GPIO3 or GPIO13, got {original!r}"
        )
    if num not in (3, 13):
        raise cv.Invalid(
            f"{CONF_HARDWARE_UART_RX_PIN} must be GPIO3 or GPIO13 "
            "(the only ESP8266 hardware UART0 RX pins)"
        )
    return num


CONFIG_SCHEMA = climate._CLIMATE_SCHEMA.extend(
    {
        cv.Optional(CONF_MASTER, default=0x00): cv.uint8_t,
        cv.Optional(CONF_REMOTE): cv.uint8_t,
        cv.Optional(CONF_MASTER_ADDRESS_AUTO, default=True): cv.boolean,
        cv.Optional(CONF_COMMAND_MODE_READ, default=0x08): cv.uint8_t,
        cv.Optional(CONF_COMMAND_MODE_WRITE, default=0x80): cv.uint8_t,
        cv.Optional(CONF_FRAME_FORMAT, default="auto"): cv.one_of(*FRAME_FORMATS, lower=True),
        cv.Optional(CONF_FILTER_FRAMES, default=True): cv.boolean,
        cv.Optional(CONF_PACKET_MIN_WAIT, default="200ms"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_AUTORESET_ERRORS, default=False): cv.boolean,
        # Manual override for hardware UART0 RX. The common ESP8266 case
        # (uart.rx_pin GPIO13 with a non-UART0 TX pin) is auto-detected below.
        cv.Optional(CONF_HARDWARE_UART_RX_PIN): _hardware_uart_rx_pin,

        cv.GenerateID(): cv.declare_id(ToshibaAbClimate),
        cv.Optional(CONF_CONNECTED): binary_sensor.binary_sensor_schema(
            device_class = DEVICE_CLASS_CONNECTIVITY,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_VENT): cv.maybe_simple_value(
            switch._SWITCH_SCHEMA.extend(
                cv.Schema(
                    {
                        cv.GenerateID(): cv.declare_id(ToshibaAbVentSwitch),
                    }
                )
            ),
            key=CONF_NAME,
        ),
        cv.Optional(CONF_READ_ONLY_SWITCH): cv.maybe_simple_value(
            switch._SWITCH_SCHEMA.extend(
                cv.Schema(
                    {
                        cv.GenerateID(): cv.declare_id(ToshibaAbReadOnlySwitch),
                    }
                )
            ),
            key=CONF_NAME,
        ),

        cv.Optional(CONF_ZONE1_SWITCH): cv.maybe_simple_value(
            switch._SWITCH_SCHEMA.extend(
                cv.Schema(
                    {
                        cv.GenerateID(): cv.declare_id(ToshibaAbEstiaZone1Switch),
                    }
                )
            ),
            key=CONF_NAME,
        ),
        cv.Optional(CONF_DHW_BOOST): cv.maybe_simple_value(
            switch._SWITCH_SCHEMA.extend(
                cv.Schema(
                    {
                        cv.GenerateID(): cv.declare_id(ToshibaAbEstiaDhwBoostSwitch),
                    }
                )
            ),
            key=CONF_NAME,
        ),
        cv.Optional(CONF_ZONE1_WATER_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_ZONE1_TARGET_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_DHW_CURRENT_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
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
        cv.Optional(CONF_READ_ONLY, default=False): cv.boolean,
        cv.Optional(CONF_PING, default=True): cv.boolean,
        cv.Optional(CONF_SENSORS, default=[]): cv.ensure_list(SENSOR_ITEM_SCHEMA),
        cv.Optional(CONF_REPORT_SENSOR_TEMP): REPORT_SENSOR_TEMP_SCHEMA,
        cv.Optional(CONF_FILTER_ALERT): binary_sensor.binary_sensor_schema(),
        # Estia sensors
        cv.Optional(CONF_OUTDOOR_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_COMPRESSOR_HOURS): sensor.sensor_schema(
            unit_of_measurement=UNIT_HOUR,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_DURATION,
            state_class=STATE_CLASS_TOTAL_INCREASING,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_WATERPUMP_HOURS): sensor.sensor_schema(
            unit_of_measurement=UNIT_HOUR,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_DURATION,
            state_class=STATE_CLASS_TOTAL_INCREASING,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_BACKUP_HEATER_HOURS): sensor.sensor_schema(
            unit_of_measurement=UNIT_HOUR,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_DURATION,
            state_class=STATE_CLASS_TOTAL_INCREASING,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_DEMAND_ENABLED, default=False): cv.boolean,
        cv.Optional(CONF_DEMAND): sensor.sensor_schema(
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    }
).extend(uart.UART_DEVICE_SCHEMA).extend(cv.COMPONENT_SCHEMA)


def _pin_number(config):
    if not isinstance(config, dict):
        return None
    return config.get(CONF_NUMBER)


def _uart_hub_config(config, callback):
    cv.Schema(
        {cv.Required(CONF_UART_ID): fv.id_declaration_match_schema(callback)},
        extra=cv.ALLOW_EXTRA,
    )(config)


def _auto_hardware_uart_rx_pin(config):
    # On ESP8266, a bus wired as TX=software pin + RX=GPIO13 makes ESPHome use
    # software serial for both directions. Remove RX from the ESPHome UART hub so
    # it stays TX-only, then let this component read GPIO13 through UART0 swap.
    # For every other RX pin, leave the hub untouched and keep the normal ESPHome
    # UART receive path unless hardware_uart_rx_pin is set explicitly. If TX is
    # GPIO15, ESPHome can already use swapped UART0 by itself.
    if not CORE.is_esp8266 or CONF_HARDWARE_UART_RX_PIN in config:
        return None

    auto_pin = None

    def detect_hub(hub_config):
        nonlocal auto_pin
        rx_num = _pin_number(hub_config.get(CONF_RX_PIN))
        tx_num = _pin_number(hub_config.get(CONF_TX_PIN))
        if rx_num == 13 and tx_num != 15:
            auto_pin = rx_num
            hub_config.pop(CONF_RX_PIN, None)
        return hub_config

    _uart_hub_config(config, detect_hub)
    return auto_pin


def _hardware_uart_parity(config):
    parity = None

    def detect_hub(hub_config):
        nonlocal parity
        parity = hub_config.get(CONF_PARITY, "NONE")
        return hub_config

    _uart_hub_config(config, detect_hub)
    return HARDWARE_UART_PARITY[str(parity).upper()]


def validate_uart(config):
    # When hardware_uart_rx_pin is set explicitly, or when we can auto-detect the
    # ESP8266 GPIO13 RX/software-TX case, the component owns RX via UART0 and the
    # ESPHome `uart:` hub must be TX-only. Otherwise keep requiring uart.rx_pin.
    auto_rx_pin = _auto_hardware_uart_rx_pin(config)
    require_rx = CONF_HARDWARE_UART_RX_PIN not in config and auto_rx_pin is None
    uart.final_validate_device_schema(
        "tcc_link", baud_rate=2400, require_rx=require_rx, require_tx=False
    )(config)
    if auto_rx_pin is not None:
        config[CONF_HARDWARE_UART_RX_PIN] = auto_rx_pin
    if CONF_HARDWARE_UART_RX_PIN in config:
        config[CONF_HARDWARE_UART_PARITY] = _hardware_uart_parity(config)
    return config


FINAL_VALIDATE_SCHEMA = validate_uart


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])

    await cg.register_component(var, config)
    await climate.register_climate(var, config)
    await uart.register_uart_device(var, config)

    if CONF_MASTER in config:
        cg.add(var.set_master_address(config[CONF_MASTER]))
    if CONF_REMOTE in config:
        cg.add(var.set_remote_address(config[CONF_REMOTE]))
    
    if CONF_MASTER_ADDRESS_AUTO in config:
        cg.add(var.set_master_address_auto(config[CONF_MASTER_ADDRESS_AUTO]))
    if CONF_COMMAND_MODE_READ in config:
        cg.add(var.set_command_mode_read(config[CONF_COMMAND_MODE_READ]))
    if CONF_COMMAND_MODE_WRITE in config:
        cg.add(var.set_command_mode_write(config[CONF_COMMAND_MODE_WRITE]))
    if CONF_FRAME_FORMAT in config:
        if config[CONF_FRAME_FORMAT] == "auto":
            cg.add(var.set_frame_format_auto())
        else:
            cg.add(var.set_frame_format(FRAME_FORMATS[config[CONF_FRAME_FORMAT]]))
    if CONF_FILTER_FRAMES in config:
        cg.add(var.set_filter_frames(config[CONF_FILTER_FRAMES]))
    if CONF_PACKET_MIN_WAIT in config:
        cg.add(var.set_packet_min_wait(cg.uint32(config[CONF_PACKET_MIN_WAIT])))
    if CONF_AUTORESET_ERRORS in config:
        cg.add(var.set_autoreset_errors(config[CONF_AUTORESET_ERRORS]))
    if CONF_HARDWARE_UART_RX_PIN in config:
        cg.add(var.set_hardware_uart_rx_pin(config[CONF_HARDWARE_UART_RX_PIN]))
    if CONF_HARDWARE_UART_PARITY in config:
        cg.add(var.set_hardware_uart_parity(config[CONF_HARDWARE_UART_PARITY]))

    if CONF_CONNECTED in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_CONNECTED])
        cg.add(var.set_connected_binary_sensor(sens))

    if CONF_FAILED_CRCS in config:
        sens = await sensor.new_sensor(config[CONF_FAILED_CRCS])
        cg.add(var.set_failed_crcs_sensor(sens))

    if CONF_VENT in config:
        sw = await switch.new_switch(config[CONF_VENT], var)
        cg.add(var.set_vent_switch(sw))
    if CONF_READ_ONLY_SWITCH in config:
        sw = await switch.new_switch(config[CONF_READ_ONLY_SWITCH], var)
        cg.add(var.set_read_only_switch(sw))
    if CONF_ZONE1_SWITCH in config:
        sw = await switch.new_switch(config[CONF_ZONE1_SWITCH], var)
        cg.add(var.set_zone1_switch(sw))
    if CONF_DHW_BOOST in config:
        sw = await switch.new_switch(config[CONF_DHW_BOOST], var)
        cg.add(var.set_dhw_boost_switch(sw))

    if CONF_ON_DATA_RECEIVED in config:
        for on_data_received in config.get(CONF_ON_DATA_RECEIVED, []):
            data_trigger = cg.new_Pvariable(on_data_received[CONF_TRIGGER_ID], var)
            await automation.build_automation(
                data_trigger, [(cg.std_vector.template(cg.uint8), "x")], on_data_received
            )
    if CONF_AUTONOMOUS in config:
        cg.add(var.set_autonomous(config[CONF_AUTONOMOUS]))
    if CONF_READ_ONLY in config:
        cg.add(var.set_read_only(config[CONF_READ_ONLY]))
    if CONF_PING in config:
        cg.add(var.set_ping_enabled(config[CONF_PING]))
    
    if CONF_FILTER_ALERT in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_FILTER_ALERT])
        cg.add(var.set_filter_alert_sensor(sens))

    # Estia sensors
    if CONF_ZONE1_WATER_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_ZONE1_WATER_TEMPERATURE])
        cg.add(var.set_zone1_water_temp_sensor(sens))
    if CONF_ZONE1_TARGET_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_ZONE1_TARGET_TEMPERATURE])
        cg.add(var.set_zone1_target_temperature_sensor(sens))

    has_dhw_current_sensor = CONF_DHW_CURRENT_TEMPERATURE in config
    if has_dhw_current_sensor:
        dhw_current_sens = await sensor.new_sensor(config[CONF_DHW_CURRENT_TEMPERATURE])
        cg.add(var.set_dhw_current_temp_sensor(dhw_current_sens))
        cg.add(var.add_polled_sensor(0x0A, 1.0, cg.uint32(120000), dhw_current_sens))
    if CONF_OUTDOOR_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_OUTDOOR_TEMPERATURE])
        cg.add(var.set_outdoor_temp_sensor(sens))
    if CONF_COMPRESSOR_HOURS in config:
        sens = await sensor.new_sensor(config[CONF_COMPRESSOR_HOURS])
        cg.add(var.set_compressor_hours_sensor(sens))
    if CONF_WATERPUMP_HOURS in config:
        sens = await sensor.new_sensor(config[CONF_WATERPUMP_HOURS])
        cg.add(var.set_waterpump_hours_sensor(sens))
    if CONF_BACKUP_HEATER_HOURS in config:
        sens = await sensor.new_sensor(config[CONF_BACKUP_HEATER_HOURS])
        cg.add(var.set_backup_heater_hours_sensor(sens))
    if CONF_DEMAND_ENABLED in config:
        cg.add(var.set_demand_enabled(config[CONF_DEMAND_ENABLED]))
    if CONF_DEMAND in config:
        sens = await sensor.new_sensor(config[CONF_DEMAND])
        cg.add(var.set_demand_sensor(sens))
    
    has_dhw_current_poll = has_dhw_current_sensor
    for item in config.get(CONF_SENSORS, []):
        if item[CONF_ADDRESS] == 0x0A and has_dhw_current_sensor:
            continue
        sens = await sensor.new_sensor(item["sensor"])  # creates the Sensor with name/units/etc.
        addr = item[CONF_ADDRESS]
        scale = item[CONF_SCALE]
        interval_ms = item[CONF_INTERVAL]
        cg.add(var.add_polled_sensor(addr, scale, cg.uint32(interval_ms), sens))
        if addr == 0x0A:
            cg.add(var.set_dhw_current_temp_sensor(sens))
            has_dhw_current_poll = True

    if not has_dhw_current_poll:
        cg.add(var.add_polled_sensor(0x0A, 1.0, cg.uint32(120000), cg.nullptr))

    # Periodically report a local ESPHome temperature sensor to the AC as "remote temp"
    if (rst := config.get(CONF_REPORT_SENSOR_TEMP)) is not None:
        src = await cg.get_variable(rst[CONF_SENSOR])
        cg.add(var.set_ext_temp_source(src))
        cg.add(var.set_ext_temp_enabled(rst["enabled"]))
        cg.add(var.set_ext_temp_interval(cg.uint32(rst[CONF_INTERVAL])))
        cg.add(var.set_ext_temp_sensor_name(rst[CONF_SENSOR].id))


@automation.register_action(
    "toshiba_ab.send_raw_frame",
    ToshibaAbSendRawFrameAction,
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.use_id(ToshibaAbClimate),
            cv.Required(CONF_FRAME): cv.templatable(cv.string_strict),
        }
    ),
)
async def to_code_send_raw_frame(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg, await cg.get_variable(config[CONF_ID]))
    template_ = await cg.templatable(config[CONF_FRAME], args, cg.std_string)
    cg.add(var.set_frame(template_))
    return var
