import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

DEPENDENCIES = ["uart"]
# is the code owner of the relevant code base
CODEOWNERS = ["@limengdu"]
# The current component or platform can be configured or defined multiple times in the same configuration file.
MULTI_CONF = True

# This line of code creates a new namespace called ti6432_ns.
# This namespace will be used as a prefix for all classes, functions and variables associated with the ti6432_ns component, ensuring that they do not conflict with the names of other components.
ti6432_ns = cg.esphome_ns.namespace("ti6432")
# This TI6432Component class will be a periodically polled UART device
TI6432Component = ti6432_ns.class_(
    "TI6432Component", cg.Component, uart.UARTDevice
)

CONF_TI6432_ID = "ti6432_id"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(TI6432Component),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)

# This code extends the current CONFIG_SCHEMA by adding all the configuration parameters for the UART device and components.
# This means that in the YAML configuration file, the user can use these parameters to configure this component.
CONFIG_SCHEMA = cv.All(
    CONFIG_SCHEMA.extend(uart.UART_DEVICE_SCHEMA).extend(cv.COMPONENT_SCHEMA)
)

# A verification mode was created to verify the configuration parameters of a UART device named "ti6432".
# This authentication mode requires that the device must have transmit and receive functionality, a parity mode of "NONE", and a stop bit of one.
FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "ti6432",
    require_tx=True,
    require_rx=True,
    parity="NONE",
    stop_bits=1,
)


# The async def keyword is used to define a concurrent function.
# Concurrent functions are special functions designed to work with Python's asyncio library to support asynchronous I/O operations.
async def to_code(config):
    # This line of code creates a new Pvariable (a Python object representing a C++ variable) with the variable's ID taken from the configuration.
    var = cg.new_Pvariable(config[CONF_ID])
    # This line of code registers the newly created Pvariable as a component so that ESPHome can manage it at runtime.
    await cg.register_component(var, config)
    # This line of code registers the newly created Pvariable as a device.
    await uart.register_uart_device(var, config)
