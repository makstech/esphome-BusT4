import esphome.codegen as cg
from esphome.components import binary_sensor
import esphome.config_validation as cv

from .. import CONF_BUS_T4_ID, BusT4Component, bus_t4_ns

DEPENDENCIES = ['bus_t4']

BusT4Photocell = bus_t4_ns.class_('BusT4Photocell', binary_sensor.BinarySensor, cg.Component)

# Photocell index, matching Nice's FT/PHOTO numbering: 1 = PHOTO/FOTO (closing safety),
# 2 = PHOTO2, 3 = PHOTO3. Not a bus address. See .agent/PROTOCOL.md §3.
CONF_ADDRESS = 'address'

CONFIG_SCHEMA = (
    binary_sensor.binary_sensor_schema(BusT4Photocell)
    .extend(
        {
            cv.GenerateID(CONF_BUS_T4_ID): cv.use_id(BusT4Component),
            cv.Optional(CONF_ADDRESS, default=1): cv.int_range(min=1, max=3),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = await binary_sensor.new_binary_sensor(config)
    await cg.register_component(var, config)

    parent = await cg.get_variable(config[CONF_BUS_T4_ID])
    cg.add(var.set_parent(parent))
    cg.add(var.set_address(config[CONF_ADDRESS]))
