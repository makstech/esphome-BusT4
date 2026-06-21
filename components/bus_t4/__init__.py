import esphome.codegen as cg
from esphome.components import cover, number, select, switch, text_sensor, uart
import esphome.config_validation as cv
from esphome.const import (
    CONF_ADDRESS,
    CONF_ICON,
    CONF_ID,
    CONF_MODE,
    CONF_NAME,
    CONF_TYPE,
    CONF_UNIT_OF_MEASUREMENT,
    ENTITY_CATEGORY_CONFIG,
    ENTITY_CATEGORY_DIAGNOSTIC,
)

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["cover", "switch", "text_sensor", "number", "select"]

bus_t4_ns = cg.esphome_ns.namespace("bus_t4")
BusT4Component = bus_t4_ns.class_("BusT4Component", cg.Component, uart.UARTDevice)
BusT4Cover = bus_t4_ns.class_("BusT4Cover", cover.Cover, cg.Component)
BusT4Switch = bus_t4_ns.class_("BusT4Switch", switch.Switch, cg.Component)
BusT4Number = bus_t4_ns.class_("BusT4Number", number.Number, cg.Component)
BusT4Select = bus_t4_ns.class_("BusT4Select", select.Select, cg.Component)

CONF_BUS_T4_ID = "bus_t4_id"

# Config flags: YAML type -> (CFG_* byte, default name, default icon). Bytes must match CFG_* in t4_packet.h.
CONFIG_TYPES = {
    "auto_close": (0x80, "Auto-close", "mdi:gate-arrow-right"),
    "photo_close": (0x84, "Close after photo", "mdi:motion-sensor"),
    "always_close": (0x88, "Always close", "mdi:lock"),
    "standby": (0x8C, "Standby", "mdi:power-standby"),
    "peak": (0x90, "Peak", "mdi:speedometer"),
    "pre_flash": (0x94, "Pre-flashing", "mdi:alarm-light"),
    "disable_internal_radio": (0x9B, "Disable internal radio", "mdi:radio-off"),  # on = radio off
    "slave": (0x98, "Slave mode", "mdi:link-variant"),
}
_FLAG_ENUM = {k: v[0] for k, v in CONFIG_TYPES.items()}

# Numeric params: YAML type -> (param byte, min, max, step, unit, default name, icon).
# Ranges/units come from the controller's command-info (verified live on RBS400).
NUMBER_TYPES = {
    "pause_time": (0x81, 0, 240, 1, "s", "Auto-close pause time", "mdi:timer-sand"),
}
_NUMBER_ENUM = {k: v[0] for k, v in NUMBER_TYPES.items()}

# Enumerated params: YAML type -> (param byte, default name, default icon, [(raw, label), ...]).
# The controller reports the valid raw values via command-info; labels are ours,
# from the programming manual, indexed by raw value (verified live: step_by_step
# 0x61 is an 8-option list reporting raw 1-8).
SELECT_TYPES = {
    "step_by_step": (
        0x61,
        "Step-by-Step mode",
        "mdi:gesture-tap-button",
        [
            (1, "Open-Stop-Close-Stop"),
            (2, "Open-Stop-Close-Open"),
            (3, "Open-Close-Open-Close"),
            (4, "Condominium"),
            (5, "Condominium 2"),
            (6, "Step-by-Step 2"),
            (7, "Hold-to-run"),
            (8, "Semi-automatic"),
        ],
    ),
}
_SELECT_ENUM = {k: v[0] for k, v in SELECT_TYPES.items()}

CONF_CONTROL_UNIT = "control_unit"
CONF_COVER = "cover"
CONF_FLAGS = "flags"
CONF_NUMBERS = "numbers"
CONF_SELECTS = "selects"
CONF_DIAGNOSTICS = "diagnostics"
CONF_FIRMWARE = "firmware"
CONF_PRODUCT = "product"
CONF_HARDWARE = "hardware"
CONF_DESCRIPTION = "description"
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
    # Accept a bare type ("auto_close") or a full map; inject the default name and
    # icon from CONFIG_TYPES (switch_schema requires id or name). A map can override.
    if not isinstance(value, dict):
        value = {CONF_TYPE: value}
    key = str(value.get(CONF_TYPE, "")).lower()
    if key in CONFIG_TYPES:
        value.setdefault(CONF_NAME, CONFIG_TYPES[key][1])
        value.setdefault(CONF_ICON, CONFIG_TYPES[key][2])
    return FLAG_SCHEMA(value)


NUMBER_SCHEMA = (
    number.number_schema(BusT4Number, entity_category=ENTITY_CATEGORY_CONFIG)
    .extend(cv.COMPONENT_SCHEMA)
    .extend({cv.Required(CONF_TYPE): cv.enum(_NUMBER_ENUM, lower=True)})
)


def _number(value):
    # Bare type ("pause_time") or a map; inject the default name + unit from
    # NUMBER_TYPES before validating (number_schema requires id or name).
    if not isinstance(value, dict):
        value = {CONF_TYPE: value}
    key = str(value.get(CONF_TYPE, "")).lower()
    if key in NUMBER_TYPES:
        spec = NUMBER_TYPES[key]
        value.setdefault(CONF_NAME, spec[5])
        value.setdefault(CONF_UNIT_OF_MEASUREMENT, spec[4])
        value.setdefault(CONF_ICON, spec[6])
        value.setdefault(CONF_MODE, "box")
    return NUMBER_SCHEMA(value)


SELECT_SCHEMA = (
    select.select_schema(BusT4Select, entity_category=ENTITY_CATEGORY_CONFIG)
    .extend(cv.COMPONENT_SCHEMA)
    .extend({cv.Required(CONF_TYPE): cv.enum(_SELECT_ENUM, lower=True)})
)


def _select(value):
    # Bare type ("step_by_step") or a map; inject the default name + icon from
    # SELECT_TYPES before validating (select_schema requires id or name).
    if not isinstance(value, dict):
        value = {CONF_TYPE: value}
    key = str(value.get(CONF_TYPE, "")).lower()
    if key in SELECT_TYPES:
        spec = SELECT_TYPES[key]
        value.setdefault(CONF_NAME, spec[1])
        value.setdefault(CONF_ICON, spec[2])
    return SELECT_SCHEMA(value)


def _diagnostics(value):
    # `diagnostics: true` creates the sensors with default names; a map renames them.
    if value is True:
        value = {}
    diag = cv.maybe_simple_value(
        text_sensor.text_sensor_schema(entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
        key=CONF_NAME,
    )
    return cv.Schema(
        {
            cv.Optional(CONF_FIRMWARE, default="Firmware"): diag,
            cv.Optional(CONF_PRODUCT, default="Product"): diag,
            cv.Optional(CONF_HARDWARE, default="Hardware"): diag,
            cv.Optional(CONF_DESCRIPTION, default="Description"): diag,
        }
    )(value)


CONTROL_UNIT_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_NAME): cv.string,
        cv.Optional(CONF_ADDRESS): cv.hex_uint16_t,  # control-unit address override
        cv.Optional(CONF_COVER): COVER_SCHEMA,
        cv.Optional(CONF_FLAGS, default=[]): cv.ensure_list(_flag),
        cv.Optional(CONF_NUMBERS, default=[]): cv.ensure_list(_number),
        cv.Optional(CONF_SELECTS, default=[]): cv.ensure_list(_select),
        cv.Optional(CONF_DIAGNOSTICS): _diagnostics,
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

    for nc in cu[CONF_NUMBERS]:
        # cv.enum keeps the type name; look up its spec, then pass the byte to set_param.
        byte, nmin, nmax, nstep, _unit, _name, _icon = NUMBER_TYPES[str(nc[CONF_TYPE])]
        num = await number.new_number(nc, min_value=nmin, max_value=nmax, step=nstep)
        await cg.register_component(num, nc)
        cg.add(num.set_parent(var))
        if cu_address is not None:
            cg.add(num.set_target_address(cu_address))
        cg.add(num.set_param(byte))

    for sc in cu[CONF_SELECTS]:
        byte, _name, _icon, opts = SELECT_TYPES[str(sc[CONF_TYPE])]
        sel = await select.new_select(sc, options=[label for _raw, label in opts])
        await cg.register_component(sel, sc)
        cg.add(sel.set_parent(var))
        if cu_address is not None:
            cg.add(sel.set_target_address(cu_address))
        cg.add(sel.set_param(byte))
        for raw, _label in opts:
            cg.add(sel.add_option_value(raw))

    if CONF_DIAGNOSTICS in cu:
        v = cu[CONF_DIAGNOSTICS]
        cg.add(var.set_firmware_sensor(await text_sensor.new_text_sensor(v[CONF_FIRMWARE])))
        cg.add(var.set_product_sensor(await text_sensor.new_text_sensor(v[CONF_PRODUCT])))
        cg.add(var.set_hardware_sensor(await text_sensor.new_text_sensor(v[CONF_HARDWARE])))
        cg.add(var.set_description_sensor(await text_sensor.new_text_sensor(v[CONF_DESCRIPTION])))
