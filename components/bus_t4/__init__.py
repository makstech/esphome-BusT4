import esphome.codegen as cg
from esphome.components import sensor, uart
import esphome.config_validation as cv
from esphome.const import (
    CONF_ADDRESS,
    CONF_ID,
    CONF_NAME,
    ENTITY_CATEGORY_DIAGNOSTIC,
)

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor"]

bus_t4_ns = cg.esphome_ns.namespace("bus_t4")
BusT4Component = bus_t4_ns.class_("BusT4Component", cg.Component, uart.UARTDevice)

CONF_DIAGNOSTICS = "diagnostics"
CONF_BUS_ERRORS = "bus_errors"
CONF_BUS_TIMEOUTS = "bus_timeouts"


def _diagnostics(value):
    # Bus-level health counters (frame errors, request timeouts).
    if value is True:
        value = {}

    def busdiag(icon):
        return cv.maybe_simple_value(
            sensor.sensor_schema(
                sensor.Sensor,
                accuracy_decimals=0,
                state_class="total_increasing",
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon=icon,
            ),
            key=CONF_NAME,
        )

    return cv.Schema(
        {
            cv.Optional(CONF_BUS_ERRORS, default="Errors"): busdiag("mdi:alert-circle-outline"),
            cv.Optional(CONF_BUS_TIMEOUTS, default="Timeouts"): busdiag("mdi:timer-alert-outline"),
        }
    )(value)


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(BusT4Component),
            cv.Optional(CONF_ADDRESS, default=0x5090): cv.hex_uint16_t,
            cv.Optional(CONF_DIAGNOSTICS): _diagnostics,
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

    if CONF_DIAGNOSTICS in config:
        bd = config[CONF_DIAGNOSTICS]
        cg.add(var.set_bus_errors_sensor(await sensor.new_sensor(bd[CONF_BUS_ERRORS])))
        cg.add(var.set_bus_timeouts_sensor(await sensor.new_sensor(bd[CONF_BUS_TIMEOUTS])))
