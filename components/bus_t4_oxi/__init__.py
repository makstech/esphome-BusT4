import esphome.codegen as cg
from esphome.components import text_sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_DEVICE_ID,
    CONF_ID,
    CONF_NAME,
    ENTITY_CATEGORY_DIAGNOSTIC,
)

DEPENDENCIES = ["bus_t4"]
AUTO_LOAD = ["text_sensor"]

bus_t4_ns = cg.esphome_ns.namespace("bus_t4")
BusT4Component = bus_t4_ns.class_("BusT4Component", cg.Component)

bus_t4_oxi_ns = cg.esphome_ns.namespace("bus_t4_oxi")
BusT4Oxi = bus_t4_oxi_ns.class_("BusT4Oxi", cg.Component)

# ESPHome sub-device, referenced by the `device:` key.
Device = cg.esphome_ns.class_("Device")

CONF_BUS_T4_ID = "bus_t4_id"
CONF_DEVICE = "device"
CONF_PRODUCT = "product"
CONF_HARDWARE = "hardware"
CONF_FIRMWARE = "firmware"

_DEFAULTS = {CONF_PRODUCT: "Product", CONF_HARDWARE: "Hardware", CONF_FIRMWARE: "Firmware"}


def _assign_device(config):
    # device_id must be present at validation time (not codegen) for the per-device
    # name-uniqueness check, so the OXI identity sensors can reuse the control
    # unit's names ("Product"/"Firmware"/...) on a separate HA device.
    dev = config.get(CONF_DEVICE)
    if dev is None:
        return config
    config = dict(config)
    for k, name in _DEFAULTS.items():
        sub = config.get(k)
        sub = ({} if sub is None else {CONF_NAME: sub}) if not isinstance(sub, dict) else dict(sub)
        sub.setdefault(CONF_NAME, name)
        sub.setdefault(CONF_DEVICE_ID, dev)
        config[k] = sub
    return config


_diag = cv.maybe_simple_value(
    text_sensor.text_sensor_schema(entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
    key=CONF_NAME,
)

CONFIG_SCHEMA = cv.All(
    _assign_device,
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(BusT4Oxi),
            cv.GenerateID(CONF_BUS_T4_ID): cv.use_id(BusT4Component),
            cv.Optional(CONF_DEVICE): cv.use_id(Device),  # group OXI sensors under this device
            **{cv.Optional(k, default=name): _diag for k, name in _DEFAULTS.items()},
        }
    ),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    bus = await cg.get_variable(config[CONF_BUS_T4_ID])
    cg.add(var.set_parent(bus))
    cg.add(var.set_product_sensor(await text_sensor.new_text_sensor(config[CONF_PRODUCT])))
    cg.add(var.set_hardware_sensor(await text_sensor.new_text_sensor(config[CONF_HARDWARE])))
    cg.add(var.set_firmware_sensor(await text_sensor.new_text_sensor(config[CONF_FIRMWARE])))
