import esphome.codegen as cg
from esphome.components import switch
import esphome.config_validation as cv
from esphome.const import CONF_TYPE, ENTITY_CATEGORY_CONFIG

from .. import CONF_BUS_T4_ID, BusT4Component, bus_t4_ns

DEPENDENCIES = ["bus_t4"]

BusT4Switch = bus_t4_ns.class_("BusT4Switch", switch.Switch, cg.Component)

# Maps the YAML `type:` to (param_byte, description).
# Byte values must match CFG_* in t4_packet.h.
CONFIG_TYPES = {
    "auto_close":   (0x80, "Auto-close after opening"),
    "photo_close":  (0x84, "Close after photo"),
    "always_close": (0x88, "Always close"),
    "standby":      (0x8C, "Standby"),
    "peak":         (0x90, "Peak"),
    "pre_flash":    (0x94, "Pre-flashing"),
    "slave":        (0x98, "Slave mode"),
}

# cv.enum expects {name: value}, not tuples
_CONFIG_ENUM = {k: v[0] for k, v in CONFIG_TYPES.items()}

CONFIG_SCHEMA = (
    switch.switch_schema(
        BusT4Switch,
        entity_category=ENTITY_CATEGORY_CONFIG,
        default_restore_mode="DISABLED",
    )
    .extend(
        {
            cv.GenerateID(CONF_BUS_T4_ID): cv.use_id(BusT4Component),
            cv.Required(CONF_TYPE): cv.enum(_CONFIG_ENUM, lower=True),
        }
    )
)


async def to_code(config):
    var = await switch.new_switch(config)
    await cg.register_component(var, config)

    parent = await cg.get_variable(config[CONF_BUS_T4_ID])
    cg.add(var.set_parent(parent))
    cg.add(var.set_param(config[CONF_TYPE]))
