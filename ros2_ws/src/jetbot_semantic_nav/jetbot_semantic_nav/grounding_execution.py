"""Explicit, bounded simulation trials; unrestricted learned execution stays off."""


def permitted(runtime, grounding, text, object_id=None):
    policy = (grounding or {}).get('simulation_execution', {})
    if (runtime.get('simulation_only') is not True or
            runtime.get('operator_enabled') is not True or
            grounding.get('backend') != 'pretrained_chinese_encoder' or
            policy.get('enabled') is not True):
        return False
    commands = policy.get('commands', {})
    if not isinstance(commands, dict):
        return False
    expected = commands.get(text)
    if not isinstance(expected, str) or not expected:
        return False
    return object_id is None or expected == object_id
