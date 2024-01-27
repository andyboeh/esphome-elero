import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import spi
from esphome.const import CONF_ID

DEPENDENCIES = ["spi"]

elero_ns = cg.esphome_ns.namespace("elero")
elero = elero_ns.class_("Elero", spi.SPIDevice, cg.Component)

CONF_GDO0_PIN = "gdo0_pin"
CONF_ELERO_ID = "elero_id"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(elero),
            cv.Required(CONF_GDO0_PIN): pins.gpio_input_pin_schema,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(spi.spi_device_schema(cs_pin_required=True))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await spi.register_spi_device(var, config)

    gdo0_pin = await cg.gpio_pin_expression(config[CONF_GDO0_PIN])
    cg.add(var.set_gdo0_pin(gdo0_pin))
