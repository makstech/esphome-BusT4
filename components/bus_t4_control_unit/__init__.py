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


DOMAINS = ("switch", "number", "select", "sensor", "button")

# Built-in parameter presets. name -> {byte, domain, name, icon, + domain fields}:
#   number: min, max, step (default 1), unit (default ""), width (1), scale (1)
#   select: options [(raw, label), ...]
#   sensor: width (default 1)
#   button: value (written on press)
# Bytes must match the protocol; switch bytes are the CFG_* flags in t4_packet.h.
# Ranges/units verified live on RBS400. A user can define params not listed here
# inline in `expose:` with `byte` + `domain` (see _custom_row / README).
PARAMS = {
    # Switches (CFG_* flag bytes)
    "auto_close": {"byte": 0x80, "domain": "switch", "name": "Auto-close", "icon": "mdi:gate-arrow-right"},
    "photo_close": {"byte": 0x84, "domain": "switch", "name": "Close after photo", "icon": "mdi:motion-sensor"},
    "always_close": {"byte": 0x88, "domain": "switch", "name": "Always-close", "icon": "mdi:lock"},
    "standby": {"byte": 0x8C, "domain": "switch", "name": "Stand-by", "icon": "mdi:power-standby"},
    "surge": {"byte": 0x90, "domain": "switch", "name": "Surge", "icon": "mdi:rocket-launch"},
    "pre_flash": {"byte": 0x94, "domain": "switch", "name": "Pre-flashing", "icon": "mdi:alarm-light"},
    "disable_internal_radio": {"byte": 0x9B, "domain": "switch", "name": "Disable internal radio", "icon": "mdi:radio-off"},  # on = radio off
    "slave": {"byte": 0x98, "domain": "switch", "name": "Slave mode", "icon": "mdi:link-variant"},
    "automation_lock": {"byte": 0x9A, "domain": "switch", "name": "Automation lock", "icon": "mdi:lock-outline"},
    "keylock": {"byte": 0x9C, "domain": "switch", "name": "Keylock", "icon": "mdi:key-variant"},
    "decelerations": {"byte": 0xA2, "domain": "switch", "name": "Decelerations", "icon": "mdi:car-brake-hold"},
    # Numbers
    "pause_time": {"byte": 0x81, "domain": "number", "name": "Auto-close pause time", "icon": "mdi:timer-sand", "min": 0, "max": 240, "unit": "s"},
    "speed_opening": {"byte": 0x42, "domain": "number", "name": "Speed (opening)", "icon": "mdi:speedometer", "min": 25, "max": 100, "unit": "%"},
    "speed_closing": {"byte": 0x43, "domain": "number", "name": "Speed (closing)", "icon": "mdi:speedometer-medium", "min": 25, "max": 100, "unit": "%"},
    "force_opening": {"byte": 0x4A, "domain": "number", "name": "Force (opening)", "icon": "mdi:arm-flex", "min": 0, "max": 100, "unit": "%"},
    "force_closing": {"byte": 0x4B, "domain": "number", "name": "Force (closing)", "icon": "mdi:arm-flex-outline", "min": 0, "max": 100, "unit": "%"},
    "decel_speed_opening": {"byte": 0x45, "domain": "number", "name": "Deceleration speed (opening)", "icon": "mdi:speedometer-slow", "min": 25, "max": 50, "unit": "%"},
    "decel_speed_closing": {"byte": 0x46, "domain": "number", "name": "Deceleration speed (closing)", "icon": "mdi:speedometer-slow", "min": 25, "max": 50, "unit": "%"},
    "decel_force_opening": {"byte": 0x4D, "domain": "number", "name": "Deceleration force (opening)", "icon": "mdi:arm-flex", "min": 0, "max": 100, "unit": "%"},
    "decel_force_closing": {"byte": 0x4E, "domain": "number", "name": "Deceleration force (closing)", "icon": "mdi:arm-flex-outline", "min": 0, "max": 100, "unit": "%"},
    "decel_sensitivity_opening": {"byte": 0x3D, "domain": "number", "name": "Deceleration sensitivity (opening)", "icon": "mdi:car-brake-alert", "min": 0, "max": 100, "unit": "%"},
    "decel_sensitivity_closing": {"byte": 0x3E, "domain": "number", "name": "Deceleration sensitivity (closing)", "icon": "mdi:car-brake-alert", "min": 0, "max": 100, "unit": "%"},
    "maintenance_threshold": {"byte": 0xB1, "domain": "number", "name": "Maintenance threshold", "icon": "mdi:wrench-clock", "min": 100, "max": 20000, "step": 100, "width": 4},
    "photo_close_time": {"byte": 0x85, "domain": "number", "name": "Close after photo time", "icon": "mdi:timer-outline", "min": 0, "max": 250, "unit": "s"},
    "always_close_time": {"byte": 0x89, "domain": "number", "name": "Always-close time", "icon": "mdi:timer-outline", "min": 0, "max": 20, "unit": "s"},
    "standby_time": {"byte": 0x8D, "domain": "number", "name": "Stand-by time", "icon": "mdi:timer-outline", "min": 5, "max": 250, "unit": "s"},
    "surge_time": {"byte": 0x91, "domain": "number", "name": "Surge time", "icon": "mdi:timer-outline", "min": 1, "max": 10, "step": 0.1, "unit": "s", "scale": 0.1},
    "pre_flash_open_time": {"byte": 0x95, "domain": "number", "name": "Pre-flash time (opening)", "icon": "mdi:timer-outline", "min": 1, "max": 10, "unit": "s"},
    "pre_flash_close_time": {"byte": 0x99, "domain": "number", "name": "Pre-flash time (closing)", "icon": "mdi:timer-outline", "min": 1, "max": 10, "unit": "s"},
    "max_work_time": {"byte": 0xA7, "domain": "number", "name": "Maximum work time", "icon": "mdi:timer-alert-outline", "min": 10, "max": 250, "unit": "s"},
    "courtesy_light_time": {"byte": 0x5B, "domain": "number", "name": "Courtesy light time", "icon": "mdi:lightbulb-on-outline", "min": 0, "max": 240, "unit": "s"},
    "electric_lock_time": {"byte": 0x5A, "domain": "number", "name": "Electric lock time", "icon": "mdi:lock-clock", "min": 0.1, "max": 10, "step": 0.1, "unit": "s", "scale": 0.1},
    "suction_cup_time": {"byte": 0x5C, "domain": "number", "name": "Suction cup time", "icon": "mdi:magnet", "min": 0.1, "max": 10, "step": 0.1, "unit": "s", "scale": 0.1},
    "brief_reversal": {"byte": 0x31, "domain": "number", "name": "Brief reversal", "icon": "mdi:backup-restore", "min": 0.5, "max": 5, "step": 0.1, "unit": "s", "scale": 0.1},
    # Selects
    "step_by_step": {"byte": 0x61, "domain": "select", "name": "Step-by-Step mode", "icon": "mdi:gesture-tap-button", "options": [
        (1, "Open-Stop-Close-Stop"),
        (2, "Open-Stop-Close-Open"),
        (3, "Open-Close-Open-Close"),
        (4, "Condominium"),
        (5, "Condominium 2"),
        (6, "Step-by-Step 2"),
        (7, "Hold-to-run"),
        (8, "Semi-automatic"),
    ]},
    "output_1": {"byte": 0x51, "domain": "select", "name": "Output 1 function", "icon": "mdi:electric-switch", "options": _out_opts(_OUT_FULL)},
    "output_2": {"byte": 0x52, "domain": "select", "name": "Output 2 function", "icon": "mdi:electric-switch", "options": _out_opts(_OUT_FULL)},
    "output_3": {"byte": 0x53, "domain": "select", "name": "Output 3 function", "icon": "mdi:electric-switch", "options": _out_opts(_OUT_BASIC)},
    "output_4": {"byte": 0x54, "domain": "select", "name": "Output 4 function", "icon": "mdi:electric-switch", "options": _out_opts(_OUT_BASIC)},
    "output_5": {"byte": 0x55, "domain": "select", "name": "Output 5 function", "icon": "mdi:electric-switch", "options": _out_opts(_OUT_BASIC)},
    "output_6": {"byte": 0x56, "domain": "select", "name": "Output 6 function", "icon": "mdi:electric-switch", "options": _out_opts(_OUT_BASIC)},
    "maintenance_management": {"byte": 0xB0, "domain": "select", "name": "Maintenance management", "icon": "mdi:wrench-cog", "options": [(4, "Manual"), (5, "Automatic")]},
    "photo_close_mode": {"byte": 0x86, "domain": "select", "name": "Close after photo mode", "icon": "mdi:gate-open", "options": [(16, "Open fully"), (17, "Open until disengagement")]},
    "always_close_mode": {"byte": 0x8A, "domain": "select", "name": "Always-close mode", "icon": "mdi:gate-alert", "options": [(32, "Always close"), (33, "Save closing")]},
    "standby_mode": {"byte": 0x8E, "domain": "select", "name": "Stand-by mode", "icon": "mdi:sleep", "options": [(48, "BlueBus"), (49, "Safety devices"), (50, "All"), (56, "All, Wi-Fi on")]},
    # Sensors (read-only)
    "maintenance_count": {"byte": 0xB2, "domain": "sensor", "name": "Maintenance counter", "icon": "mdi:counter", "width": 4},
    "total_maneuvers": {"byte": 0xB3, "domain": "sensor", "name": "Total maneuvers", "icon": "mdi:counter", "width": 4},
    # Buttons (write a fixed value on press)
    "reset_maintenance": {"byte": 0xB4, "domain": "button", "name": "Reset maintenance counter", "icon": "mdi:restart", "value": 1},
}

CONF_DEVICE = "device"
CONF_COVER = "cover"
CONF_EXPOSE = "expose"
CONF_PARAM = "param"
CONF_DOMAIN = "domain"
CONF_BYTE = "byte"
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

# Resolved-definition keys carried alongside the entity config for to_code; not
# part of any entity schema, so stripped before validating the entity.
CONF_DEF = "param_def"
_DEF_KEYS = {CONF_PARAM, CONF_BYTE, CONF_DOMAIN, "min", "max", "step", "unit", "width", "scale", "options", "value"}

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


# Entity schemas, one per domain. The param/byte/range live in the definition
# (PARAMS or an inline custom row), so these carry no `type` field.
_DOMAIN_SCHEMA = {
    "switch": switch.switch_schema(
        BusT4Switch, entity_category=ENTITY_CATEGORY_CONFIG, default_restore_mode="DISABLED"
    ).extend(cv.COMPONENT_SCHEMA),
    "number": number.number_schema(BusT4Number, entity_category=ENTITY_CATEGORY_CONFIG).extend(cv.COMPONENT_SCHEMA),
    "select": select.select_schema(BusT4Select, entity_category=ENTITY_CATEGORY_CONFIG).extend(cv.COMPONENT_SCHEMA),
    "sensor": sensor.sensor_schema(
        BusT4Sensor, accuracy_decimals=0, state_class="total_increasing", entity_category=ENTITY_CATEGORY_DIAGNOSTIC
    ).extend(cv.polling_component_schema("60s")),
    "button": button.button_schema(BusT4Button, entity_category=ENTITY_CATEGORY_CONFIG).extend(cv.COMPONENT_SCHEMA),
}


def _option(o):
    o = list(o)
    if len(o) != 2:
        raise cv.Invalid("each select option must be [raw_value, label]")
    return (cv.int_(o[0]), cv.string(o[1]))


def _custom_row(value):
    # An inline param definition: `byte` + `domain` + the domain's fields. Lets a
    # user expose a parameter that isn't in the PARAMS catalog.
    domain = cv.one_of(*DOMAINS, lower=True)(value[CONF_DOMAIN])
    if CONF_PARAM in value:
        raise cv.Invalid("use either `param` (a built-in) or `byte`+`domain` (custom), not both")
    if CONF_BYTE not in value:
        raise cv.Invalid("a custom param requires `byte`")
    row = {"byte": cv.hex_uint8_t(value[CONF_BYTE]), "domain": domain, "name": value.get(CONF_NAME)}
    if CONF_ICON in value:
        row["icon"] = value[CONF_ICON]
    if domain == "number":
        for req in ("min", "max"):
            if req not in value:
                raise cv.Invalid(f"a custom number requires `{req}`")
        row["min"] = value["min"]
        row["max"] = value["max"]
        row["step"] = value.get("step", 1)
        row["unit"] = value.get("unit", "")
        row["width"] = cv.int_range(1, 4)(value.get("width", 1))
        row["scale"] = value.get("scale", 1)
    elif domain == "select":
        if "options" not in value:
            raise cv.Invalid("a custom select requires `options` ([raw, label] pairs)")
        row["options"] = [_option(o) for o in value["options"]]
    elif domain == "sensor":
        row["width"] = cv.int_range(1, 4)(value.get("width", 1))
    elif domain == "button":
        if "value" not in value:
            raise cv.Invalid("a custom button requires `value` (written on press)")
        row["value"] = value["value"]
    return row


def _expose(value):
    # One flat list of parameters. An entry is a built-in name (bare or
    # {param: name, ...overrides}), or a custom definition ({byte:, domain:, ...}).
    # The resolved definition is stashed under CONF_DEF for to_code.
    if not isinstance(value, dict):
        value = {CONF_PARAM: value}
    value = dict(value)

    if CONF_DOMAIN in value or CONF_BYTE in value:
        row = _custom_row(value)
    else:
        name = str(value.get(CONF_PARAM, "")).lower()
        if name not in PARAMS:
            raise cv.Invalid(
                f"Unknown parameter '{name}'. Valid: {', '.join(sorted(PARAMS))}. "
                f"For a parameter not listed, define it inline with `byte` and `domain`."
            )
        row = PARAMS[name]

    domain = row["domain"]
    entity = {k: v for k, v in value.items() if k not in _DEF_KEYS}
    if row.get("name"):
        entity.setdefault(CONF_NAME, row["name"])
    if row.get("icon"):
        entity.setdefault(CONF_ICON, row["icon"])
    if domain == "number":
        entity.setdefault(CONF_UNIT_OF_MEASUREMENT, row.get("unit", ""))
        entity.setdefault(CONF_MODE, "box")

    validated = _DOMAIN_SCHEMA[domain](entity)
    validated[CONF_DEF] = row
    return validated


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
    entry = {CONF_PARAM: entry} if not isinstance(entry, dict) else dict(entry)
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
    if isinstance(config.get(CONF_EXPOSE), list):
        config[CONF_EXPOSE] = [_with_device(e, dev) for e in config[CONF_EXPOSE]]
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
            cv.Optional(CONF_EXPOSE, default=[]): cv.ensure_list(_expose),
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

    expose = config[CONF_EXPOSE]

    def of(domain):
        return [e for e in expose if e[CONF_DEF]["domain"] == domain]

    for fc in of("switch"):
        d = fc[CONF_DEF]
        sw = await switch.new_switch(fc)
        await cg.register_component(sw, fc)
        cg.add(sw.set_parent(bus))
        if cu_address is not None:
            cg.add(sw.set_target_address(cu_address))
        cg.add(sw.set_param(d["byte"]))

    for nc in of("number"):
        d = nc[CONF_DEF]
        num = await number.new_number(nc, min_value=d["min"], max_value=d["max"], step=d.get("step", 1))
        await cg.register_component(num, nc)
        cg.add(num.set_parent(bus))
        if cu_address is not None:
            cg.add(num.set_target_address(cu_address))
        cg.add(num.set_param(d["byte"]))
        cg.add(num.set_width(d.get("width", 1)))
        cg.add(num.set_scale(d.get("scale", 1)))

    for nc in of("sensor"):
        d = nc[CONF_DEF]
        sen = await sensor.new_sensor(nc)
        await cg.register_component(sen, nc)
        cg.add(sen.set_parent(bus))
        if cu_address is not None:
            cg.add(sen.set_target_address(cu_address))
        cg.add(sen.set_param(d["byte"]))
        cg.add(sen.set_width(d.get("width", 1)))

    for bc in of("button"):
        d = bc[CONF_DEF]
        btn = await button.new_button(bc)
        await cg.register_component(btn, bc)
        cg.add(btn.set_parent(bus))
        if cu_address is not None:
            cg.add(btn.set_target_address(cu_address))
        cg.add(btn.set_param(d["byte"]))
        cg.add(btn.set_value(d["value"]))

    for sc in of("select"):
        d = sc[CONF_DEF]
        opts = d["options"]
        sel = await select.new_select(sc, options=[label for _raw, label in opts])
        await cg.register_component(sel, sc)
        cg.add(sel.set_parent(bus))
        if cu_address is not None:
            cg.add(sel.set_target_address(cu_address))
        cg.add(sel.set_param(d["byte"]))
        for raw, _label in opts:
            cg.add(sel.add_option_value(raw))

    if CONF_DIAGNOSTICS in config:
        v = config[CONF_DIAGNOSTICS]
        cg.add(cu.set_firmware_sensor(await text_sensor.new_text_sensor(v[CONF_FIRMWARE])))
        cg.add(cu.set_product_sensor(await text_sensor.new_text_sensor(v[CONF_PRODUCT])))
        cg.add(cu.set_hardware_sensor(await text_sensor.new_text_sensor(v[CONF_HARDWARE])))
        cg.add(cu.set_description_sensor(await text_sensor.new_text_sensor(v[CONF_DESCRIPTION])))
