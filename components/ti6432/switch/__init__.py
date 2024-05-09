import esphome.codegen as cg
from esphome.components import switch
import esphome.config_validation as cv
from esphome.const import (
    DEVICE_CLASS_SWITCH,
    ENTITY_CATEGORY_CONFIG,
)
from .. import CONF_TI6432_ID, TI6432Component, ti6432_ns

UnderlyingOpenFuncSwitch = ti6432_ns.class_(
    "UnderlyOpenFunctionSwitch", switch.Switch
)

CONF_UNDERLY_OPEN_FUNCTION = "underly_open_function"

CONFIG_SCHEMA = {
    cv.GenerateID(CONF_TI6432_ID): cv.use_id(TI6432Component),
    cv.Optional(CONF_UNDERLY_OPEN_FUNCTION): switch.switch_schema(
        UnderlyingOpenFuncSwitch,
        device_class=DEVICE_CLASS_SWITCH,
        entity_category=ENTITY_CATEGORY_CONFIG,
        icon="mdi:electric-switch",
    ),
}


async def to_code(config):
    ti6432_component = await cg.get_variable(config[CONF_TI6432_ID])
    if underly_open_function_config := config.get(CONF_UNDERLY_OPEN_FUNCTION):
        s = await switch.new_switch(underly_open_function_config)
        await cg.register_parented(s, config[CONF_TI6432_ID])
        cg.add(ti6432_component.set_underly_open_function_switch(s))
