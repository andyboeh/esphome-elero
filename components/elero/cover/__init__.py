import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import cover

from esphome.const import CONF_ID, CONF_NAME, CONF_CHANNEL
from .. import elero_ns, elero, CONF_ELERO_ID

DEPENDENCIES = ["elero"]
CODEOWNERS = ["@andyboeh"]

CONF_BLIND_ADDRESS = "blind_address"
CONF_REMOTE_ADDRESS = "remote_address"

EleroCover = elero_ns.class_("EleroCover", cover.Cover, cg.Component)

CONFIG_SCHEMA = cover.COVER_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(EleroCover),
        cv.GenerateID(CONF_ELERO_ID): cv.use_id(elero),
        cv.Required(CONF_BLIND_ADDRESS): cv.hex_int_range(min=0x0, max=0xffffff),
        cv.Required(CONF_CHANNEL): cv.int_range(min=0, max=255),
        cv.Required(CONF_REMOTE_ADDRESS): cv.hex_int_range(min=0x0, max=0xffffff),
        cv.Required(CONF_NAME): cv.string,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await cover.register_cover(var, config)

    parent = await cg.get_variable(config[CONF_ELERO_ID])
    cg.add(var.set_elero_parent(parent))
    cg.add(var.set_blind_address(config[CONF_BLIND_ADDRESS]))
    cg.add(var.set_channel(config[CONF_CHANNEL]))
    cg.add(var.set_remote_address(config[CONF_REMOTE_ADDRESS]))
    cg.add(var.set_name(config[CONF_NAME]))
