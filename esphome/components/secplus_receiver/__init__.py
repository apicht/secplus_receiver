"""
ESPHome external component: secplus_receiver

Decodes Security+ 2.0 (Manchester-encoded OOK) transmissions from
Chamberlain / LiftMaster / Craftsman garage door remotes using a CC1101
transceiver and the remote_receiver component.

The component attaches itself directly to a remote_receiver as a listener
(no on_raw lambda required), matching the structure of the acurite component:
https://github.com/swoboda1337/acurite-esphome

On first build this module downloads secplus.c and secplus.h from the
argilo/secplus repository (master branch) into the component directory.

Minimal YAML usage:

    remote_receiver:
      pin: GPIO2

    secplus_receiver:
      remote_fixed_data:
        name: "Fixed Unique ID"

      rolling_code_sensor:
        name: "Rolling Code"

      remote_id_sensor:
        name: "Remote ID"

      button_sensor:
        name: "Button ID"

      on_remote_received:
        then:
          - lambda: |-
              ESP_LOGI("main", "fixed_data=%llu remote_id=%llu button=%u rolling=%u",
                       (unsinged long long) fixed_data,
                       (unsigned long long) remote_id,
                       button, rolling);

Optional authentication (allowlist + rolling-counter replay protection):

    secplus_receiver:
      id: receiver
      authentication:
        max_remotes: 8        # learnable slots, each gets a "Remote Slot N" sensor
        window: 1024          # how far ahead of the stored counter a code may be
        learn_timeout: 30s
        remotes:              # optional remotes fixed in YAML
          - 70678577664
      on_authenticated:       # (fixed_data, remote_id, button, rolling, slot)
        then:
          - logger.log: "known remote pressed"

    button:
      - platform: template
        name: "Learn Remote"
        on_press:
          - secplus_receiver.learn: receiver

Actions: secplus_receiver.learn, secplus_receiver.stop_learn,
secplus_receiver.forget (slot: N), secplus_receiver.forget_all.

"""

import logging
import os
import urllib.request

import esphome.codegen as cg
from esphome import automation
from esphome.components import remote_base, remote_receiver, text_sensor
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_NAME, CONF_TRIGGER_ID, ENTITY_CATEGORY_DIAGNOSTIC

_LOGGER = logging.getLogger(__name__)

DOMAIN = "secplus_receiver"
CODEOWNERS = ["@greenbus4"]
DEPENDENCIES = ["remote_receiver", "text_sensor", "api"]
AUTO_LOAD = ["text_sensor"]

secplus_ns = cg.esphome_ns.namespace("secplus_receiver")
SecplusReceiverComponent = secplus_ns.class_("SecplusReceiverComponent", cg.Component)

# Trigger for on_remote_received: passes (fixed_data, remote_id, button, rolling)
RemoteReceivedTrigger = secplus_ns.class_(
    "RemoteReceivedTrigger",
    automation.Trigger.template(cg.uint64, cg.uint64, cg.uint8, cg.uint32),
)

CONF_FIXED_DATA_SENSOR   = "fixed_data_sensor"
CONF_ROLLING_CODE_SENSOR = "rolling_code_sensor"
CONF_REMOTE_ID_SENSOR = "remote_id_sensor"
CONF_BUTTON_SENSOR = "button_sensor"
CONF_FIRE_EVENT = "fire_homeassistant_event"
CONF_ON_REMOTE_RECEIVED = "on_remote_received"
CONF_ON_AUTHENTICATED = "on_authenticated"
CONF_ON_REJECTED = "on_rejected"
CONF_ON_REMOTE_LEARNED = "on_remote_learned"
CONF_AUTHENTICATION = "authentication"
CONF_MAX_REMOTES = "max_remotes"
CONF_WINDOW = "window"
CONF_LEARN_TIMEOUT = "learn_timeout"
CONF_SLOT_SENSORS = "slot_sensors"
CONF_REMOTES = "remotes"
CONF_SLOT = "slot"
# Internal: generated text sensor configs, one per learnable slot.
CONF_SLOT_SENSOR_CONFIGS = "_slot_sensor_configs"

AuthenticatedTrigger = secplus_ns.class_(
    "AuthenticatedTrigger",
    automation.Trigger.template(cg.uint64, cg.uint64, cg.uint8, cg.uint32, cg.uint8),
)
RejectedTrigger = secplus_ns.class_(
    "RejectedTrigger",
    automation.Trigger.template(cg.uint64, cg.uint32, cg.uint8),
)
RemoteLearnedTrigger = secplus_ns.class_(
    "RemoteLearnedTrigger",
    automation.Trigger.template(cg.uint64, cg.uint8),
)

LearnAction = secplus_ns.class_("LearnAction", automation.Action)
StopLearnAction = secplus_ns.class_("StopLearnAction", automation.Action)
ForgetAction = secplus_ns.class_("ForgetAction", automation.Action)
ForgetAllAction = secplus_ns.class_("ForgetAllAction", automation.Action)

_SLOT_SENSOR_SCHEMA = text_sensor.text_sensor_schema(
    icon="mdi:remote",
    entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
)


def _expand_slot_sensors(config: dict) -> dict:
    """Generate one "Remote Slot N" text sensor config per learnable slot."""
    config = dict(config)
    if config[CONF_SLOT_SENSORS]:
        config[CONF_SLOT_SENSOR_CONFIGS] = [
            _SLOT_SENSOR_SCHEMA({CONF_NAME: f"Remote Slot {i + 1}"})
            for i in range(config[CONF_MAX_REMOTES])
        ]
    return config


def _validate_authentication(config: dict) -> dict:
    remotes = config[CONF_REMOTES]
    if len(set(remotes)) != len(remotes):
        raise cv.Invalid("Duplicate fixed code", path=[CONF_REMOTES])
    if config[CONF_MAX_REMOTES] == 0 and not remotes:
        raise cv.Invalid(
            f"Set '{CONF_MAX_REMOTES}' above 0 or list at least one entry in '{CONF_REMOTES}'"
        )
    return config


AUTHENTICATION_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.Optional(CONF_MAX_REMOTES, default=8): cv.int_range(min=0, max=32),
            # The rolling code is a 28-bit counter that rises by one per press.
            cv.Optional(CONF_WINDOW, default=1024): cv.int_range(min=1, max=65535),
            cv.Optional(CONF_LEARN_TIMEOUT, default="30s"): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(min=cv.TimePeriod(seconds=1)),
            ),
            cv.Optional(CONF_SLOT_SENSORS, default=True): cv.boolean,
            # Fixed codes are 40 bits.
            cv.Optional(CONF_REMOTES, default=[]): cv.All(
                cv.ensure_list(cv.int_range(min=0, max=(1 << 40) - 1)),
                cv.Length(max=32),
            ),
        }
    ),
    _validate_authentication,
    _expand_slot_sensors,
)


def _validate_triggers_need_authentication(config: dict) -> dict:
    if CONF_AUTHENTICATION in config:
        return config
    for key in (CONF_ON_AUTHENTICATED, CONF_ON_REJECTED, CONF_ON_REMOTE_LEARNED):
        if key in config:
            raise cv.Invalid(f"'{key}' requires '{CONF_AUTHENTICATION}' to be configured")
    return config


# ── Upstream C library ────────────────────────────────────────────────────────
# argilo/secplus has no versioned releases; we track master.
# To pin to a specific commit, replace "master" with a SHA, e.g. "f62ed51".
SECPLUS_COMMIT = "master"
SECPLUS_BASE_URL = f"https://raw.githubusercontent.com/argilo/secplus/{SECPLUS_COMMIT}/src/"
SECPLUS_FILES = ["secplus.c", "secplus.h"]

CONFIG_SCHEMA = cv.All(
    cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(SecplusReceiverComponent),

        cv.GenerateID(remote_base.CONF_RECEIVER_ID): cv.use_id(
            remote_receiver.RemoteReceiverComponent
        ),

        cv.Optional(CONF_FIXED_DATA_SENSOR): text_sensor.text_sensor_schema(
            icon="mdi:identifier",
        ),

        cv.Optional(CONF_ROLLING_CODE_SENSOR): text_sensor.text_sensor_schema(
            icon="mdi:counter",
        ),

        cv.Optional(CONF_REMOTE_ID_SENSOR): text_sensor.text_sensor_schema(
            icon="mdi:remote",
        ),

        cv.Optional(CONF_BUTTON_SENSOR): text_sensor.text_sensor_schema(
            icon="mdi:button-pointer",
        ),

        cv.Optional(CONF_FIRE_EVENT, default=False): cv.boolean,

        cv.Optional(CONF_ON_REMOTE_RECEIVED): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(RemoteReceivedTrigger),
            }
        ),

        cv.Optional(CONF_AUTHENTICATION): AUTHENTICATION_SCHEMA,

        cv.Optional(CONF_ON_AUTHENTICATED): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(AuthenticatedTrigger),
            }
        ),

        cv.Optional(CONF_ON_REJECTED): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(RejectedTrigger),
            }
        ),

        cv.Optional(CONF_ON_REMOTE_LEARNED): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(RemoteLearnedTrigger),
            }
        ),
    }
    ).extend(cv.COMPONENT_SCHEMA),
    _validate_triggers_need_authentication,
)

_PARENT_ACTION_SCHEMA = automation.maybe_simple_id(
    {
        cv.GenerateID(): cv.use_id(SecplusReceiverComponent),
    }
)

_FORGET_ACTION_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(SecplusReceiverComponent),
        # Up to 32 learnable slots followed by up to 32 static remotes.
        cv.Required(CONF_SLOT): cv.templatable(cv.int_range(min=1, max=64)),
    }
)


async def _parent_action_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var


for _name, _class in (
    ("secplus_receiver.learn", LearnAction),
    ("secplus_receiver.stop_learn", StopLearnAction),
    ("secplus_receiver.forget_all", ForgetAllAction),
):
    automation.register_action(
        _name, _class, _PARENT_ACTION_SCHEMA, synchronous=True
    )(_parent_action_to_code)


@automation.register_action(
    "secplus_receiver.forget", ForgetAction, _FORGET_ACTION_SCHEMA, synchronous=True
)
async def _forget_action_to_code(config, action_id, template_arg, args):
    var = await _parent_action_to_code(config, action_id, template_arg, args)
    cg.add(var.set_slot(await cg.templatable(config[CONF_SLOT], args, cg.int_)))
    return var


def _fetch_secplus_sources() -> None:
    """Download secplus.c and secplus.h from GitHub if not already present."""
    component_dir = os.path.dirname(__file__)
    for filename in SECPLUS_FILES:
        dest = os.path.join(component_dir, filename)
        if os.path.isfile(dest):
            _LOGGER.debug(
                "secplus_receiver: %s already present, skipping download", filename
            )
            continue
        url = SECPLUS_BASE_URL + filename
        _LOGGER.info("secplus_receiver: downloading %s", url)
        try:
            urllib.request.urlretrieve(url, dest)
        except Exception as exc:  # noqa: BLE001
            raise RuntimeError(
                f"secplus_receiver: failed to download {filename} from {url}: {exc}\n"
                "Check your internet connection, or manually place secplus.c and "
                f"secplus.h in:\n  {component_dir}"
            ) from exc


async def to_code(config: dict) -> None:
    # Fetch the upstream C library before the compiler runs.
    _fetch_secplus_sources()

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await remote_base.register_listener(var, config)

    if fixed_config := config.get(CONF_FIXED_DATA_SENSOR):
        sens = await text_sensor.new_text_sensor(fixed_config)
        cg.add(var.set_fixed_data_sensor(sens))

    if rolling_config := config.get(CONF_ROLLING_CODE_SENSOR):
        sens = await text_sensor.new_text_sensor(rolling_config)
        cg.add(var.set_rolling_code_sensor(sens))

    if remote_id_config := config.get(CONF_REMOTE_ID_SENSOR):
        sens = await text_sensor.new_text_sensor(remote_id_config)
        cg.add(var.set_remote_id_sensor(sens))

    if button_config := config.get(CONF_BUTTON_SENSOR):
        sens = await text_sensor.new_text_sensor(button_config)
        cg.add(var.set_button_sensor(sens))

    if config.get(CONF_FIRE_EVENT):
        cg.add(var.set_fire_event(True))

    if auth_config := config.get(CONF_AUTHENTICATION):
        cg.add(
            var.set_authentication(
                auth_config[CONF_MAX_REMOTES],
                auth_config[CONF_WINDOW],
                auth_config[CONF_LEARN_TIMEOUT],
            )
        )
        for fixed in auth_config[CONF_REMOTES]:
            cg.add(var.add_static_remote(cg.RawExpression(f"{fixed}ULL")))
        for slot_config in auth_config.get(CONF_SLOT_SENSOR_CONFIGS, []):
            sens = await text_sensor.new_text_sensor(slot_config)
            cg.add(var.add_slot_sensor(sens))

    for conf_key, trigger_args in (
        (
            CONF_ON_AUTHENTICATED,
            [
                (cg.uint64, "fixed_data"),
                (cg.uint64, "remote_id"),
                (cg.uint8, "button"),
                (cg.uint32, "rolling"),
                (cg.uint8, "slot"),
            ],
        ),
        (
            CONF_ON_REJECTED,
            [(cg.uint64, "fixed_data"), (cg.uint32, "rolling"), (cg.uint8, "slot")],
        ),
        (CONF_ON_REMOTE_LEARNED, [(cg.uint64, "fixed_data"), (cg.uint8, "slot")]),
    ):
        for conf in config.get(conf_key, []):
            trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
            await automation.build_automation(trigger, trigger_args, conf)

    for conf in config.get(CONF_ON_REMOTE_RECEIVED, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(
            trigger,
            [
                (cg.uint64, "fixed_data"),
                (cg.uint64, "remote_id"),
                (cg.uint8, "button"),
                (cg.uint32, "rolling"),
            ],
            conf,
        )

