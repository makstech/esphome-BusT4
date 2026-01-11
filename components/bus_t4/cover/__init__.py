import esphome.codegen as cg
from esphome.components import cover
import esphome.config_validation as cv

from .. import CONF_BUS_T4_ID, BusT4Component, bus_t4_ns

DEPENDENCIES = ['bus_t4']

BusT4Cover = bus_t4_ns.class_('BusT4Cover', cover.Cover, cg.Component)

CONF_OPEN_DURATION = 'open_duration'
CONF_CLOSE_DURATION = 'close_duration'
CONF_AUTO_LEARN_TIMING = 'auto_learn_timing'

CONFIG_SCHEMA = (
    cover.cover_schema(BusT4Cover, device_class="gate")
    .extend(cv.COMPONENT_SCHEMA)
    .extend(
        {
            cv.GenerateID(CONF_BUS_T4_ID): cv.use_id(BusT4Component),
            cv.Optional(CONF_OPEN_DURATION, default='20s'): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_CLOSE_DURATION, default='20s'): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_AUTO_LEARN_TIMING, default=True): cv.boolean,
        }
    )
)

async def to_code(config):
    var = await cover.new_cover(config)
    await cg.register_component(var, config)

    parent = await cg.get_variable(config[CONF_BUS_T4_ID])
    cg.add(var.set_parent(parent))
    
    cg.add(var.set_open_duration(config[CONF_OPEN_DURATION]))
    cg.add(var.set_close_duration(config[CONF_CLOSE_DURATION]))
    cg.add(var.set_auto_learn_timing(config[CONF_AUTO_LEARN_TIMING]))
