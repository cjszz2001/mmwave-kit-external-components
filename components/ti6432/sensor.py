import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    DEVICE_CLASS_DISTANCE,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_SPEED,
    STATE_CLASS_MEASUREMENT,
    UNIT_METER,
)
from . import CONF_TI6432_ID, TI6432Component

AUTO_LOAD = ["ti6432"]

CONF_CUSTOM_PRESENCE_OF_DETECTION = "custom_presence_of_detection"
CONF_MOVEMENT_SIGNS = "movement_signs"
CONF_CUSTOM_MOTION_DISTANCE = "custom_motion_distance"
CONF_CUSTOM_SPATIAL_STATIC_VALUE = "custom_spatial_static_value"
CONF_CUSTOM_SPATIAL_MOTION_VALUE = "custom_spatial_motion_value"
CONF_CUSTOM_MOTION_SPEED = "custom_motion_speed"
CONF_CUSTOM_MODE_NUM = "custom_mode_num"
CONF_HUMAN_ENTERED_IN_ROOM = "human_entered_in_room"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_TI6432_ID): cv.use_id(TI6432Component),
        cv.Optional(CONF_CUSTOM_PRESENCE_OF_DETECTION): sensor.sensor_schema(
            #device_class=DEVICE_CLASS_DISTANCE,
            #unit_of_measurement=UNIT_METER,
            #accuracy_decimals=2,  # Specify the number of decimal places
            #icon="mdi:signal-distance-variant",
            icon="mdi:human-greeting-variant",
        ),
        cv.Optional(CONF_MOVEMENT_SIGNS): sensor.sensor_schema(
            #state_class=STATE_CLASS_MEASUREMENT,
            icon="mdi:human-greeting-variant",
        ),
        cv.Optional(CONF_CUSTOM_MOTION_DISTANCE): sensor.sensor_schema(
            #unit_of_measurement=UNIT_METER,
            #accuracy_decimals=2,
            #icon="mdi:signal-distance-variant",
            icon="mdi:human-greeting-variant",
        ),
        cv.Optional(CONF_CUSTOM_SPATIAL_STATIC_VALUE): sensor.sensor_schema(
            #device_class=DEVICE_CLASS_ENERGY,
            icon="mdi:numeric",
        ),
        cv.Optional(CONF_CUSTOM_SPATIAL_MOTION_VALUE): sensor.sensor_schema(
            #device_class=DEVICE_CLASS_ENERGY,
            icon="mdi:percent",
        ),
        cv.Optional(CONF_CUSTOM_MOTION_SPEED): sensor.sensor_schema(
            unit_of_measurement="Persons",
            #device_class=DEVICE_CLASS_SPEED,
            #accuracy_decimals=2,
            #icon="mdi:run-fast",
            icon="mdi:counter",
        ),
        cv.Optional(CONF_CUSTOM_MODE_NUM): sensor.sensor_schema(
            unit_of_measurement="Persons",
            icon="mdi:counter",
        ),
        cv.Optional(CONF_HUMAN_ENTERED_IN_ROOM): sensor.sensor_schema(
            unit_of_measurement="Persons",
            icon="mdi:counter",
        ),
    }
)


async def to_code(config):
    ti6432_component = await cg.get_variable(config[CONF_TI6432_ID])
    if custompresenceofdetection_config := config.get(
        CONF_CUSTOM_PRESENCE_OF_DETECTION
    ):
        sens = await sensor.new_sensor(custompresenceofdetection_config)
        cg.add(ti6432_component.set_custom_presence_of_detection_sensor(sens))
    if movementsigns_config := config.get(CONF_MOVEMENT_SIGNS):
        sens = await sensor.new_sensor(movementsigns_config)
        cg.add(ti6432_component.set_movement_signs_sensor(sens))
    if custommotiondistance_config := config.get(CONF_CUSTOM_MOTION_DISTANCE):
        sens = await sensor.new_sensor(custommotiondistance_config)
        cg.add(ti6432_component.set_custom_motion_distance_sensor(sens))
    if customspatialstaticvalue_config := config.get(CONF_CUSTOM_SPATIAL_STATIC_VALUE):
        sens = await sensor.new_sensor(customspatialstaticvalue_config)
        cg.add(ti6432_component.set_custom_spatial_static_value_sensor(sens))
    if customspatialmotionvalue_config := config.get(CONF_CUSTOM_SPATIAL_MOTION_VALUE):
        sens = await sensor.new_sensor(customspatialmotionvalue_config)
        cg.add(ti6432_component.set_custom_spatial_motion_value_sensor(sens))
    if custommotionspeed_config := config.get(CONF_CUSTOM_MOTION_SPEED):
        sens = await sensor.new_sensor(custommotionspeed_config)
        cg.add(ti6432_component.set_custom_motion_speed_sensor(sens))
    if custommodenum_config := config.get(CONF_CUSTOM_MODE_NUM):
        sens = await sensor.new_sensor(custommodenum_config)
        cg.add(ti6432_component.set_custom_mode_num_sensor(sens))
    if humanenteredinroom_config := config.get(CONF_HUMAN_ENTERED_IN_ROOM):
        sens = await sensor.new_sensor(humanenteredinroom_config)
        cg.add(ti6432_component.set_human_entered_in_room_sensor(sens))
