"""Shared helpers for the launch files."""
import yaml

LEGS = ['L1', 'L2', 'L3', 'R1', 'R2', 'R3']


def geometry_params(path):
    """legs.yaml -> gait_node parameters."""
    with open(path) as f:
        g = yaml.safe_load(f)
    m = g['mounts']
    return {
        'coxa': g['coxa'],
        'femur': g['femur'],
        'tibia': g['tibia'],
        'mount_x': [float(m[leg]['x']) for leg in LEGS],
        'mount_y': [float(m[leg]['y']) for leg in LEGS],
        'mount_yaw': [float(m[leg]['yaw']) for leg in LEGS],
        'reach': g['stand']['reach'],
        'body_height': g['stand']['height'],
    }
