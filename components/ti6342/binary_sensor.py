import esphome.codegen as cg
from esphome.components import binary_sensor
import esphome.config_validation as cv
from esphome.const import (
    DEVICE_CLASS_OCCUPANCY,
)
from . import CONF_TI6342_ID, TI6342Component

AUTO_LOAD = ["ti6342"]
CONF_HAS_TARGET = "has_target"

CONFIG_SCHEMA = {
    cv.GenerateID(CONF_TI6342_ID): cv.use_id(TI6342Component),
    cv.Optional(CONF_HAS_TARGET): binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_OCCUPANCY, icon="mdi:motion-sensor"
    ),
}


async def to_code(config):
    ti6342_component = await cg.get_variable(config[CONF_TI6342_ID])
    if has_target_config := config.get(CONF_HAS_TARGET):
        sens = await binary_sensor.new_binary_sensor(has_target_config)
        cg.add(ti6342_component.set_has_target_binary_sensor(sens))
