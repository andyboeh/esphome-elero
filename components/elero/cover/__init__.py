import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import cover

from esphome.const import CONF_ID, CONF_NAME, CONF_CHANNEL, CONF_OPEN_DURATION, CONF_CLOSE_DURATION
from .. import elero_ns, elero, CONF_ELERO_ID

DEPENDENCIES = ["elero"]
CODEOWNERS = ["@andyboeh"]

CONF_BLIND_ADDRESS = "blind_address"
CONF_REMOTE_ADDRESS = "remote_address"
CONF_PAYLOAD_1 = "payload_1"
CONF_PAYLOAD_2 = "payload_2"
CONF_PCKINF_1 = "pck_inf1"
CONF_PCKINF_2 = "pck_inf2"
CONF_HOP = "hop"
CONF_COMMAND_UP = "command_up"
CONF_COMMAND_DOWN = "command_down"
CONF_COMMAND_STOP = "command_stop"
CONF_COMMAND_CHECK = "command_check"
CONF_COMMAND_TILT = "command_tilt"
CONF_POLL_INTERVAL = "poll_interval"
CONF_SUPPORTS_TILT = "supports_tilt"

EleroCover = elero_ns.class_("EleroCover", cover.Cover, cg.Component)

def poll_interval(value):
    if value == "never":
        return 4294967295  # uint32_t max
    return cv.positive_time_period_milliseconds(value)

CONFIG_SCHEMA = cover.COVER_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(EleroCover),
        cv.GenerateID(CONF_ELERO_ID): cv.use_id(elero),
        cv.Required(CONF_BLIND_ADDRESS): cv.hex_int_range(min=0x0, max=0xffffff),
        cv.Required(CONF_CHANNEL): cv.int_range(min=0, max=255),
        cv.Required(CONF_REMOTE_ADDRESS): cv.hex_int_range(min=0x0, max=0xffffff),
        cv.Optional(CONF_POLL_INTERVAL, default="5min"): poll_interval,
        cv.Optional(CONF_OPEN_DURATION, default="0s"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_CLOSE_DURATION, default="0s"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_PAYLOAD_1, default=0x00): cv.hex_int_range(min=0x0, max=0xff),
        cv.Optional(CONF_PAYLOAD_2, default=0x04): cv.hex_int_range(min=0x0, max=0xff),
        cv.Optional(CONF_PCKINF_1, default=0x6a): cv.hex_int_range(min=0x0, max=0xff),
        cv.Optional(CONF_PCKINF_2, default=0x00): cv.hex_int_range(min=0x0, max=0xff),
        cv.Optional(CONF_HOP, default=0x0a): cv.hex_int_range(min=0x0, max=0xff),
        cv.Optional(CONF_COMMAND_UP, default=0x20): cv.hex_int_range(min=0x0, max=0xff),
        cv.Optional(CONF_COMMAND_DOWN, default=0x40): cv.hex_int_range(min=0x0, max=0xff),
        cv.Optional(CONF_COMMAND_STOP, default=0x10): cv.hex_int_range(min=0x0, max=0xff),
        cv.Optional(CONF_COMMAND_CHECK, default=0x00): cv.hex_int_range(min=0x0, max=0xff),
        cv.Optional(CONF_COMMAND_TILT, default=0x24): cv.hex_int_range(min=0x0, max=0xff),
        cv.Optional(CONF_SUPPORTS_TILT, default=False): cv.boolean,
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
    cg.add(var.set_open_duration(config[CONF_OPEN_DURATION]))
    cg.add(var.set_close_duration(config[CONF_CLOSE_DURATION]))
    cg.add(var.set_payload_1(config[CONF_PAYLOAD_1]))
    cg.add(var.set_payload_2(config[CONF_PAYLOAD_2]))
    cg.add(var.set_pckinf_1(config[CONF_PCKINF_1]))
    cg.add(var.set_pckinf_2(config[CONF_PCKINF_2]))
    cg.add(var.set_hop(config[CONF_HOP]))
    cg.add(var.set_command_up(config[CONF_COMMAND_UP]))
    cg.add(var.set_command_down(config[CONF_COMMAND_DOWN]))
    cg.add(var.set_command_check(config[CONF_COMMAND_CHECK]))
    cg.add(var.set_command_stop(config[CONF_COMMAND_STOP]))
    cg.add(var.set_command_tilt(config[CONF_COMMAND_TILT]))
    cg.add(var.set_poll_interval(config[CONF_POLL_INTERVAL]))
    cg.add(var.set_supports_tilt(config[CONF_SUPPORTS_TILT]))
