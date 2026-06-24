import esphome.codegen as cg
from esphome.components import text_sensor
import esphome.config_validation as cv
from esphome.const import ENTITY_CATEGORY_DIAGNOSTIC

from .. import CONF_BUS_T4_ID, BusT4Component, bus_t4_ns

DEPENDENCIES = ['bus_t4']

BusT4DeviceList = bus_t4_ns.class_('BusT4DeviceList', text_sensor.TextSensor, cg.Component)

CONFIG_SCHEMA = (
    text_sensor.text_sensor_schema(
        BusT4DeviceList,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    )
    .extend(
        {
            cv.GenerateID(CONF_BUS_T4_ID): cv.use_id(BusT4Component),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = await text_sensor.new_text_sensor(config)
    await cg.register_component(var, config)

    parent = await cg.get_variable(config[CONF_BUS_T4_ID])
    cg.add(var.set_parent(parent))
