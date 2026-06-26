import esphome.codegen as cg
from esphome.components import button, cover, number, select, sensor, switch, text_sensor, uart
import esphome.config_validation as cv
from esphome.const import (
    CONF_ADDRESS,
    CONF_DEVICE_ID,
    CONF_ICON,
    CONF_ID,
    CONF_MODE,
    CONF_NAME,
    CONF_TYPE,
    CONF_UNIT_OF_MEASUREMENT,
    ENTITY_CATEGORY_CONFIG,
    ENTITY_CATEGORY_DIAGNOSTIC,
)

DEPENDENCIES = ["bus_t4"]
AUTO_LOAD = ["cover", "switch", "text_sensor", "number", "select", "sensor", "button"]

bus_t4_ns = cg.esphome_ns.namespace("bus_t4")
BusT4Component = bus_t4_ns.class_("BusT4Component", cg.Component, uart.UARTDevice)

bus_t4_control_unit_ns = cg.esphome_ns.namespace("bus_t4_control_unit")
BusT4ControlUnit = bus_t4_control_unit_ns.class_("BusT4ControlUnit", cg.Component)
BusT4Cover = bus_t4_control_unit_ns.class_("BusT4Cover", cover.Cover, cg.Component)
BusT4Switch = bus_t4_control_unit_ns.class_("BusT4Switch", switch.Switch, cg.Component)
BusT4Number = bus_t4_control_unit_ns.class_("BusT4Number", number.Number, cg.Component)
BusT4Select = bus_t4_control_unit_ns.class_("BusT4Select", select.Select, cg.Component)
BusT4Sensor = bus_t4_control_unit_ns.class_("BusT4Sensor", sensor.Sensor, cg.PollingComponent)
BusT4Button = bus_t4_control_unit_ns.class_("BusT4Button", button.Button, cg.Component)

# ESPHome sub-device, referenced by a control_unit `device:` key.
Device = cg.esphome_ns.class_("Device")

CONF_BUS_T4_ID = "bus_t4_id"

# Config flags: YAML type -> (CFG_* byte, default name, default icon). Bytes must match CFG_* in t4_packet.h.
CONFIG_TYPES = {
    "auto_close": (0x80, "Auto-close", "mdi:gate-arrow-right"),
    "photo_close": (0x84, "Close after photo", "mdi:motion-sensor"),
    "always_close": (0x88, "Always-close", "mdi:lock"),
    "standby": (0x8C, "Stand-by", "mdi:power-standby"),
    "surge": (0x90, "Surge", "mdi:rocket-launch"),
    "pre_flash": (0x94, "Pre-flashing", "mdi:alarm-light"),
    "disable_internal_radio": (0x9B, "Disable internal radio", "mdi:radio-off"),  # on = radio off
    "slave": (0x98, "Slave mode", "mdi:link-variant"),
    "automation_lock": (0x9A, "Automation lock", "mdi:lock-outline"),
    "keylock": (0x9C, "Keylock", "mdi:key-variant"),
    "decelerations": (0xA2, "Decelerations", "mdi:car-brake-hold"),
}
_FLAG_ENUM = {k: v[0] for k, v in CONFIG_TYPES.items()}

# Numeric params: YAML type -> (param byte, min, max, step, unit, default name, icon, width, scale).
# min/max/step are displayed values; the displayed value is raw*scale (0.1 = tenths).
# Ranges/units come from the controller's command-info (verified live on RBS400).
NUMBER_TYPES = {
    "pause_time": (0x81, 0, 240, 1, "s", "Auto-close pause time", "mdi:timer-sand", 1, 1),
    "speed_opening": (0x42, 25, 100, 1, "%", "Speed (opening)", "mdi:speedometer", 1, 1),
    "speed_closing": (0x43, 25, 100, 1, "%", "Speed (closing)", "mdi:speedometer-medium", 1, 1),
    "force_opening": (0x4A, 0, 100, 1, "%", "Force (opening)", "mdi:arm-flex", 1, 1),
    "force_closing": (0x4B, 0, 100, 1, "%", "Force (closing)", "mdi:arm-flex-outline", 1, 1),
    "decel_speed_opening": (0x45, 25, 50, 1, "%", "Deceleration speed (opening)", "mdi:speedometer-slow", 1, 1),
    "decel_speed_closing": (0x46, 25, 50, 1, "%", "Deceleration speed (closing)", "mdi:speedometer-slow", 1, 1),
    "decel_force_opening": (0x4D, 0, 100, 1, "%", "Deceleration force (opening)", "mdi:arm-flex", 1, 1),
    "decel_force_closing": (0x4E, 0, 100, 1, "%", "Deceleration force (closing)", "mdi:arm-flex-outline", 1, 1),
    "decel_sensitivity_opening": (0x3D, 0, 100, 1, "%", "Deceleration sensitivity (opening)", "mdi:car-brake-alert", 1, 1),
    "decel_sensitivity_closing": (0x3E, 0, 100, 1, "%", "Deceleration sensitivity (closing)", "mdi:car-brake-alert", 1, 1),
    "maintenance_threshold": (0xB1, 100, 20000, 100, "", "Maintenance threshold", "mdi:wrench-clock", 4, 1),
    "photo_close_time": (0x85, 0, 250, 1, "s", "Close after photo time", "mdi:timer-outline", 1, 1),
    "always_close_time": (0x89, 0, 20, 1, "s", "Always-close time", "mdi:timer-outline", 1, 1),
    "standby_time": (0x8D, 5, 250, 1, "s", "Stand-by time", "mdi:timer-outline", 1, 1),
    "surge_time": (0x91, 1, 10, 0.1, "s", "Surge time", "mdi:timer-outline", 1, 0.1),
    "pre_flash_open_time": (0x95, 1, 10, 1, "s", "Pre-flash time (opening)", "mdi:timer-outline", 1, 1),
    "pre_flash_close_time": (0x99, 1, 10, 1, "s", "Pre-flash time (closing)", "mdi:timer-outline", 1, 1),
    "max_work_time": (0xA7, 10, 250, 1, "s", "Maximum work time", "mdi:timer-alert-outline", 1, 1),
    "courtesy_light_time": (0x5B, 0, 240, 1, "s", "Courtesy light time", "mdi:lightbulb-on-outline", 1, 1),
    "electric_lock_time": (0x5A, 0.1, 10, 0.1, "s", "Electric lock time", "mdi:lock-clock", 1, 0.1),
    "suction_cup_time": (0x5C, 0.1, 10, 0.1, "s", "Suction cup time", "mdi:magnet", 1, 0.1),
    "brief_reversal": (0x31, 0.5, 5, 0.1, "s", "Brief reversal", "mdi:backup-restore", 1, 0.1),
}
_NUMBER_ENUM = {k: v[0] for k, v in NUMBER_TYPES.items()}

# Read-only numeric params: YAML type -> (param byte, default name, default icon, width).
SENSOR_TYPES = {
    "maintenance_count": (0xB2, "Maintenance counter", "mdi:counter", 4),
    "total_maneuvers": (0xB3, "Total maneuvers", "mdi:counter", 4),
}
_SENSOR_ENUM = {k: v[0] for k, v in SENSOR_TYPES.items()}

# Action params: YAML type -> (param byte, value written on press, default name, default icon).
BUTTON_TYPES = {
    "reset_maintenance": (0xB4, 1, "Reset maintenance counter", "mdi:restart"),
}
_BUTTON_ENUM = {k: v[0] for k, v in BUTTON_TYPES.items()}

# Output-function labels from the manual's output configuration table, raw -> label.
_OUTPUT_FUNCTIONS = {
    0: "None",
    1: "SCA (gate-open indicator)",
    2: "Gate open",
    3: "Gate closed",
    4: "Maintenance",
    5: "Warning light",
    6: "Courtesy light",
    7: "Electric lock 1",
    9: "Electric locking device 1",
    11: "Suction cup 1",
    13: "Red traffic light",
    14: "Green traffic light",
    15: "Radio channel 1",
    16: "Radio channel 2",
    17: "Radio channel 3",
    18: "Radio channel 4",
    19: "Warning light 1",
    23: "Warning light 24V",
    26: "One-way traffic light",
    30: "Door status",
    35: "Presence",
    37: "PhotoTest",
}
# Outputs 1-2 support 22 functions; outputs 3-6 support 18.
_OUT_FULL = [0, 1, 2, 3, 4, 5, 6, 7, 9, 11, 13, 14, 15, 16, 17, 18, 19, 23, 26, 30, 35, 37]
_OUT_BASIC = [0, 1, 2, 3, 4, 6, 13, 14, 15, 16, 17, 18, 19, 23, 26, 30, 35, 37]


def _out_opts(raws):
    return [(r, _OUTPUT_FUNCTIONS[r]) for r in raws]


# Enumerated params: YAML type -> (param byte, default name, default icon, [(raw, label), ...]).
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
    "output_1": (0x51, "Output 1 function", "mdi:electric-switch", _out_opts(_OUT_FULL)),
    "output_2": (0x52, "Output 2 function", "mdi:electric-switch", _out_opts(_OUT_FULL)),
    "output_3": (0x53, "Output 3 function", "mdi:electric-switch", _out_opts(_OUT_BASIC)),
    "output_4": (0x54, "Output 4 function", "mdi:electric-switch", _out_opts(_OUT_BASIC)),
    "output_5": (0x55, "Output 5 function", "mdi:electric-switch", _out_opts(_OUT_BASIC)),
    "output_6": (0x56, "Output 6 function", "mdi:electric-switch", _out_opts(_OUT_BASIC)),
    "maintenance_management": (
        0xB0,
        "Maintenance management",
        "mdi:wrench-cog",
        [(4, "Manual"), (5, "Automatic")],
    ),
    "photo_close_mode": (
        0x86,
        "Close after photo mode",
        "mdi:gate-open",
        [(16, "Open fully"), (17, "Open until disengagement")],
    ),
    "always_close_mode": (
        0x8A,
        "Always-close mode",
        "mdi:gate-alert",
        [(32, "Always close"), (33, "Save closing")],
    ),
    "standby_mode": (
        0x8E,
        "Stand-by mode",
        "mdi:sleep",
        [(48, "BlueBus"), (49, "Safety devices"), (50, "All"), (56, "All, Wi-Fi on")],
    ),
}
_SELECT_ENUM = {k: v[0] for k, v in SELECT_TYPES.items()}

CONF_DEVICE = "device"
CONF_COVER = "cover"
CONF_FLAGS = "flags"
CONF_NUMBERS = "numbers"
CONF_SELECTS = "selects"
CONF_SENSORS = "sensors"
CONF_BUTTONS = "buttons"
CONF_DIAGNOSTICS = "diagnostics"
CONF_FIRMWARE = "firmware"
CONF_PRODUCT = "product"
CONF_HARDWARE = "hardware"
CONF_DESCRIPTION = "description"
CONF_OPEN_DURATION = "open_duration"
CONF_CLOSE_DURATION = "close_duration"
CONF_AUTO_LEARN_TIMING = "auto_learn_timing"
CONF_POSITION_REPORT_INTERVAL = "position_report_interval"
CONF_FORCE_ESTIMATED_POSITION = "force_estimated_position"

COVER_SCHEMA = (
    cover.cover_schema(BusT4Cover, device_class="gate")
    .extend(cv.COMPONENT_SCHEMA)
    .extend(
        {
            cv.Optional(CONF_OPEN_DURATION, default="20s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_CLOSE_DURATION, default="20s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_AUTO_LEARN_TIMING, default=True): cv.boolean,
            cv.Optional(CONF_POSITION_REPORT_INTERVAL, default="1s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_FORCE_ESTIMATED_POSITION, default=False): cv.boolean,
        }
    )
)


def _cover(value):
    # Default to an empty name so the cover inherits its HA device's name.
    if isinstance(value, dict):
        value = dict(value)
        value.setdefault(CONF_NAME, "")
    return COVER_SCHEMA(value)

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


SENSOR_SCHEMA = (
    sensor.sensor_schema(
        BusT4Sensor,
        accuracy_decimals=0,
        state_class="total_increasing",
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    )
    .extend(cv.polling_component_schema("60s"))
    .extend({cv.Required(CONF_TYPE): cv.enum(_SENSOR_ENUM, lower=True)})
)


def _sensor(value):
    # Bare type ("total_maneuvers") or a map; inject the default name + icon from
    # SENSOR_TYPES before validating (sensor_schema requires id or name).
    if not isinstance(value, dict):
        value = {CONF_TYPE: value}
    key = str(value.get(CONF_TYPE, "")).lower()
    if key in SENSOR_TYPES:
        spec = SENSOR_TYPES[key]
        value.setdefault(CONF_NAME, spec[1])
        value.setdefault(CONF_ICON, spec[2])
    return SENSOR_SCHEMA(value)


BUTTON_SCHEMA = (
    button.button_schema(BusT4Button, entity_category=ENTITY_CATEGORY_CONFIG)
    .extend(cv.COMPONENT_SCHEMA)
    .extend({cv.Required(CONF_TYPE): cv.enum(_BUTTON_ENUM, lower=True)})
)


def _button(value):
    # Bare type ("reset_maintenance") or a map; inject the default name + icon
    # from BUTTON_TYPES before validating (button_schema requires id or name).
    if not isinstance(value, dict):
        value = {CONF_TYPE: value}
    key = str(value.get(CONF_TYPE, "")).lower()
    if key in BUTTON_TYPES:
        spec = BUTTON_TYPES[key]
        value.setdefault(CONF_NAME, spec[2])
        value.setdefault(CONF_ICON, spec[3])
    return BUTTON_SCHEMA(value)


_DIAG_DEFAULTS = {
    CONF_FIRMWARE: "Firmware",
    CONF_PRODUCT: "Product",
    CONF_HARDWARE: "Hardware",
    CONF_DESCRIPTION: "Description",
}


def _diagnostics(value):
    # `diagnostics: true` creates the controller-identity sensors with default
    # names; a map renames them.
    if value is True:
        value = {}
    diag = cv.maybe_simple_value(
        text_sensor.text_sensor_schema(entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
        key=CONF_NAME,
    )
    return cv.Schema(
        {cv.Optional(k, default=name): diag for k, name in _DIAG_DEFAULTS.items()}
    )(value)


def _with_device(entry, dev):
    # Stamp an entity entry with device_id so its name uniqueness is checked
    # per-device (lets the same name appear on the CU and OXI devices).
    entry = {CONF_TYPE: entry} if not isinstance(entry, dict) else dict(entry)
    entry.setdefault(CONF_DEVICE_ID, dev)
    return entry


def _assign_device(config):
    # The block-level `device:` groups every CU entity under one HA sub-device.
    # device_id must be present at validation time (not codegen) for the
    # per-device name-uniqueness check, so inject it here on the raw config.
    dev = config.get(CONF_DEVICE)
    if dev is None:
        return config
    config = dict(config)
    for key in (CONF_FLAGS, CONF_NUMBERS, CONF_SELECTS, CONF_SENSORS, CONF_BUTTONS):
        if isinstance(config.get(key), list):
            config[key] = [_with_device(e, dev) for e in config[key]]
    if isinstance(config.get(CONF_COVER), dict):
        config[CONF_COVER] = _with_device(config[CONF_COVER], dev)
    if CONF_DIAGNOSTICS in config:
        diag = config[CONF_DIAGNOSTICS]
        diag = dict(diag) if isinstance(diag, dict) else {}
        for k, name in _DIAG_DEFAULTS.items():
            sub = diag.get(k)
            sub = ({} if sub is None else {CONF_NAME: sub}) if not isinstance(sub, dict) else dict(sub)
            sub.setdefault(CONF_NAME, name)
            sub.setdefault(CONF_DEVICE_ID, dev)
            diag[k] = sub
        config[CONF_DIAGNOSTICS] = diag
    return config


CONFIG_SCHEMA = cv.All(
    _assign_device,
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(BusT4ControlUnit),
            cv.GenerateID(CONF_BUS_T4_ID): cv.use_id(BusT4Component),
            cv.Optional(CONF_DEVICE): cv.use_id(Device),  # group all CU entities under this device
            cv.Optional(CONF_ADDRESS): cv.hex_uint16_t,  # control-unit address override
            cv.Optional(CONF_COVER): _cover,
            cv.Optional(CONF_FLAGS, default=[]): cv.ensure_list(_flag),
            cv.Optional(CONF_NUMBERS, default=[]): cv.ensure_list(_number),
            cv.Optional(CONF_SELECTS, default=[]): cv.ensure_list(_select),
            cv.Optional(CONF_SENSORS, default=[]): cv.ensure_list(_sensor),
            cv.Optional(CONF_BUTTONS, default=[]): cv.ensure_list(_button),
            cv.Optional(CONF_DIAGNOSTICS): _diagnostics,
        }
    ),
)


async def to_code(config):
    bus = await cg.get_variable(config[CONF_BUS_T4_ID])

    cu = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(cu, config)
    cg.add(cu.set_parent(bus))

    cu_address = config.get(CONF_ADDRESS)  # controller address override (pins the target)

    if CONF_COVER in config:
        cc = config[CONF_COVER]
        cov = await cover.new_cover(cc)
        await cg.register_component(cov, cc)
        cg.add(cov.set_parent(bus))
        cg.add(cov.set_control_unit(cu))
        if cu_address is not None:
            cg.add(cov.set_target_address(cu_address))
        cg.add(cov.set_open_duration(cc[CONF_OPEN_DURATION]))
        cg.add(cov.set_close_duration(cc[CONF_CLOSE_DURATION]))
        cg.add(cov.set_auto_learn_timing(cc[CONF_AUTO_LEARN_TIMING]))
        cg.add(cov.set_position_report_interval(cc[CONF_POSITION_REPORT_INTERVAL]))
        cg.add(cov.set_force_estimated_position(cc[CONF_FORCE_ESTIMATED_POSITION]))

    for fc in config[CONF_FLAGS]:
        sw = await switch.new_switch(fc)
        await cg.register_component(sw, fc)
        cg.add(sw.set_parent(bus))
        if cu_address is not None:
            cg.add(sw.set_target_address(cu_address))
        cg.add(sw.set_param(fc[CONF_TYPE]))

    for nc in config[CONF_NUMBERS]:
        # cv.enum keeps the type name; look up its spec, then pass the byte to set_param.
        byte, nmin, nmax, nstep, _unit, _name, _icon, width, scale = NUMBER_TYPES[str(nc[CONF_TYPE])]
        num = await number.new_number(nc, min_value=nmin, max_value=nmax, step=nstep)
        await cg.register_component(num, nc)
        cg.add(num.set_parent(bus))
        if cu_address is not None:
            cg.add(num.set_target_address(cu_address))
        cg.add(num.set_param(byte))
        cg.add(num.set_width(width))
        cg.add(num.set_scale(scale))

    for nc in config[CONF_SENSORS]:
        byte, _name, _icon, width = SENSOR_TYPES[str(nc[CONF_TYPE])]
        sen = await sensor.new_sensor(nc)
        await cg.register_component(sen, nc)
        cg.add(sen.set_parent(bus))
        if cu_address is not None:
            cg.add(sen.set_target_address(cu_address))
        cg.add(sen.set_param(byte))
        cg.add(sen.set_width(width))

    for bc in config[CONF_BUTTONS]:
        byte, val, _name, _icon = BUTTON_TYPES[str(bc[CONF_TYPE])]
        btn = await button.new_button(bc)
        await cg.register_component(btn, bc)
        cg.add(btn.set_parent(bus))
        if cu_address is not None:
            cg.add(btn.set_target_address(cu_address))
        cg.add(btn.set_param(byte))
        cg.add(btn.set_value(val))

    for sc in config[CONF_SELECTS]:
        byte, _name, _icon, opts = SELECT_TYPES[str(sc[CONF_TYPE])]
        sel = await select.new_select(sc, options=[label for _raw, label in opts])
        await cg.register_component(sel, sc)
        cg.add(sel.set_parent(bus))
        if cu_address is not None:
            cg.add(sel.set_target_address(cu_address))
        cg.add(sel.set_param(byte))
        for raw, _label in opts:
            cg.add(sel.add_option_value(raw))

    if CONF_DIAGNOSTICS in config:
        v = config[CONF_DIAGNOSTICS]
        cg.add(cu.set_firmware_sensor(await text_sensor.new_text_sensor(v[CONF_FIRMWARE])))
        cg.add(cu.set_product_sensor(await text_sensor.new_text_sensor(v[CONF_PRODUCT])))
        cg.add(cu.set_hardware_sensor(await text_sensor.new_text_sensor(v[CONF_HARDWARE])))
        cg.add(cu.set_description_sensor(await text_sensor.new_text_sensor(v[CONF_DESCRIPTION])))
