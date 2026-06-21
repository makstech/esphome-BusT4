import esphome.codegen as cg
from esphome.components import cover, switch, uart
import esphome.config_validation as cv
from esphome.const import (
    CONF_ADDRESS,
    CONF_ID,
    CONF_NAME,
    CONF_TYPE,
    ENTITY_CATEGORY_CONFIG,
)

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["cover", "switch", "text_sensor"]

bus_t4_ns = cg.esphome_ns.namespace("bus_t4")
BusT4Component = bus_t4_ns.class_("BusT4Component", cg.Component, uart.UARTDevice)
BusT4Cover = bus_t4_ns.class_("BusT4Cover", cover.Cover, cg.Component)
BusT4Switch = bus_t4_ns.class_("BusT4Switch", switch.Switch, cg.Component)

CONF_BUS_T4_ID = "bus_t4_id"

# Config flags: YAML type -> (CFG_* byte, default friendly name). Bytes must match CFG_* in t4_packet.h.
CONFIG_TYPES = {
    "auto_close": (0x80, "Auto-close"),
    "photo_close": (0x84, "Close after photo"),
    "always_close": (0x88, "Always close"),
    "standby": (0x8C, "Standby"),
    "peak": (0x90, "Peak"),
    "pre_flash": (0x94, "Pre-flashing"),
    "slave": (0x98, "Slave mode"),
}
_FLAG_ENUM = {k: v[0] for k, v in CONFIG_TYPES.items()}

CONF_CONTROL_UNIT = "control_unit"
CONF_COVER = "cover"
CONF_FLAGS = "flags"
CONF_OPEN_DURATION = "open_duration"
CONF_CLOSE_DURATION = "close_duration"
CONF_AUTO_LEARN_TIMING = "auto_learn_timing"
CONF_POSITION_REPORT_INTERVAL = "position_report_interval"

COVER_SCHEMA = (
    cover.cover_schema(BusT4Cover, device_class="gate")
    .extend(cv.COMPONENT_SCHEMA)
    .extend(
        {
            cv.Optional(CONF_OPEN_DURATION, default="20s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_CLOSE_DURATION, default="20s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_AUTO_LEARN_TIMING, default=True): cv.boolean,
            cv.Optional(CONF_POSITION_REPORT_INTERVAL, default="1s"): cv.positive_time_period_milliseconds,
        }
    )
)

FLAG_SCHEMA = (
    switch.switch_schema(
        BusT4Switch,
        entity_category=ENTITY_CATEGORY_CONFIG,
        default_restore_mode="DISABLED",
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend({cv.Required(CONF_TYPE): cv.enum(_FLAG_ENUM, lower=True)})
)


def _flag(value):
    # Accept a bare type ("auto_close") or a full map; inject a friendly name from
    # CONFIG_TYPES before validating, since switch_schema requires id or name.
    if not isinstance(value, dict):
        value = {CONF_TYPE: value}
    if CONF_NAME not in value and CONF_TYPE in value:
        key = str(value[CONF_TYPE]).lower()
        if key in CONFIG_TYPES:
            value = {**value, CONF_NAME: CONFIG_TYPES[key][1]}
    return FLAG_SCHEMA(value)

CONTROL_UNIT_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_NAME): cv.string,
        cv.Optional(CONF_ADDRESS): cv.hex_uint16_t,  # control-unit address override (Step 2)
        cv.Optional(CONF_COVER): COVER_SCHEMA,
        cv.Optional(CONF_FLAGS, default=[]): cv.ensure_list(_flag),
    }
)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(BusT4Component),
            cv.Optional(CONF_ADDRESS, default=0x5090): cv.hex_uint16_t,
            cv.Optional(CONF_CONTROL_UNIT): CONTROL_UNIT_SCHEMA,
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)

FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "bus_t4_uart",
    require_tx=True,
    require_rx=True,
    baud_rate=19200,
    data_bits=8,
    parity="NONE",
    stop_bits=1,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    cg.add(var.set_address(config[CONF_ADDRESS]))

    if CONF_CONTROL_UNIT not in config:
        return
    cu = config[CONF_CONTROL_UNIT]

    cu_address = cu.get(CONF_ADDRESS)  # controller address override (pins the target)

    if CONF_COVER in cu:
        cc = cu[CONF_COVER]
        cov = await cover.new_cover(cc)
        await cg.register_component(cov, cc)
        cg.add(cov.set_parent(var))
        if cu_address is not None:
            cg.add(cov.set_target_address(cu_address))
        cg.add(cov.set_open_duration(cc[CONF_OPEN_DURATION]))
        cg.add(cov.set_close_duration(cc[CONF_CLOSE_DURATION]))
        cg.add(cov.set_auto_learn_timing(cc[CONF_AUTO_LEARN_TIMING]))
        cg.add(cov.set_position_report_interval(cc[CONF_POSITION_REPORT_INTERVAL]))

    for fc in cu[CONF_FLAGS]:
        sw = await switch.new_switch(fc)
        await cg.register_component(sw, fc)
        cg.add(sw.set_parent(var))
        if cu_address is not None:
            cg.add(sw.set_target_address(cu_address))
        cg.add(sw.set_param(fc[CONF_TYPE]))
