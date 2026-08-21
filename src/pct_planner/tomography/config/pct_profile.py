"""Validation and loading for the public PCT map/traversability profile."""

from types import SimpleNamespace

import rospy


PARAMETER_NAMESPACE = '/pct'
_MAP_FIELDS = ('resolution', 'ground_h', 'slice_dh')
_TRAVERSABILITY_FIELDS = (
    'kernel_size', 'interval_min', 'interval_free', 'slope_max', 'step_max',
    'standable_ratio', 'cost_barrier', 'safe_margin', 'inflation',
    'cost_threshold',
)


def _required_section(values, name, fields):
    section = values.get(name)
    if not isinstance(section, dict):
        raise RuntimeError('PCT profile section must be a dictionary: {}'.format(name))
    missing = [field for field in fields if field not in section]
    if missing:
        raise RuntimeError('missing PCT {} parameter(s): {}'.format(
            name, ', '.join(missing)))
    return section


def load_public_profile():
    if not rospy.has_param(PARAMETER_NAMESPACE):
        raise RuntimeError('missing public PCT profile: {}'.format(PARAMETER_NAMESPACE))
    values = rospy.get_param(PARAMETER_NAMESPACE)
    if not isinstance(values, dict):
        raise RuntimeError('public PCT profile must be a dictionary')

    map_values = _required_section(values, 'map', _MAP_FIELDS)
    trav_values = _required_section(values, 'traversability', _TRAVERSABILITY_FIELDS)
    map_profile = SimpleNamespace(
        resolution=float(map_values['resolution']),
        ground_h=float(map_values['ground_h']),
        slice_dh=float(map_values['slice_dh']),
    )
    traversability = SimpleNamespace(
        kernel_size=int(trav_values['kernel_size']),
        interval_min=float(trav_values['interval_min']),
        interval_free=float(trav_values['interval_free']),
        slope_max=float(trav_values['slope_max']),
        step_max=float(trav_values['step_max']),
        standable_ratio=float(trav_values['standable_ratio']),
        cost_barrier=float(trav_values['cost_barrier']),
        safe_margin=float(trav_values['safe_margin']),
        inflation=float(trav_values['inflation']),
        cost_threshold=float(trav_values['cost_threshold']),
    )
    if map_profile.resolution <= 0.0 or map_profile.slice_dh <= 0.0 or \
            traversability.kernel_size < 1 or traversability.interval_min <= 0.0 or \
            traversability.interval_free < traversability.interval_min or \
            traversability.slope_max < 0.0 or traversability.step_max < 0.0 or \
            not 0.0 <= traversability.standable_ratio <= 1.0 or \
            traversability.cost_barrier <= 0.0 or traversability.safe_margin < 0.0 or \
            traversability.inflation < 0.0 or traversability.cost_threshold < 0.0:
        raise RuntimeError('invalid public PCT profile')
    return SimpleNamespace(map=map_profile, trav=traversability)
