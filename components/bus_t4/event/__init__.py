import esphome.codegen as cg
from esphome.components import event
import esphome.config_validation as cv

from .. import CONF_BUS_T4_ID, BusT4Component, bus_t4_ns

DEPENDENCIES = ['bus_t4']

BusT4KeypadEvent = bus_t4_ns.class_('BusT4KeypadEvent', event.Event, cg.Component)

# Fixed set of commands the controller can report on the bus (see .agent/PROTOCOL.md §2).
# The firmware emits exactly these strings, so they are declared by the platform rather
# than the user (Event::trigger rejects any type not in this list).
EVENT_TYPES = [
    'open',
    'close',
    'step_by_step',
    'partial_1',
    'partial_2',
    'partial_3',
    'stop',
]

CONFIG_SCHEMA = (
    event.event_schema(BusT4KeypadEvent)
    .extend(
        {
            cv.GenerateID(CONF_BUS_T4_ID): cv.use_id(BusT4Component),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = await event.new_event(config, event_types=EVENT_TYPES)
    await cg.register_component(var, config)

    parent = await cg.get_variable(config[CONF_BUS_T4_ID])
    cg.add(var.set_parent(parent))
