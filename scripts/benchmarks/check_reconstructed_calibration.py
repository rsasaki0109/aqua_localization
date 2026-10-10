"""Check actual executable calibration against official files, without a reference."""
import argparse
import json
from pathlib import Path
import subprocess

import numpy as np
from ruamel.yaml import YAML


def check(executable, root):
    yaml = YAML(typ='safe')
    camera = yaml.load((Path(root) / 'harbor_camera_calib.yaml').read_text())['cam0']
    external = yaml.load((Path(root) / 'harbor_imu_camera_calib.yaml').read_text())['cam0']
    imu = yaml.load((Path(root) / 'harbor_imu_noises.yaml').read_text())['imu0']
    actual = json.loads(subprocess.check_output([str(executable), '--describe-calibration'], text=True))
    if camera['camera_model'] != 'pinhole' or camera['distortion_model'] != 'equidistant' or camera['resolution'] != [640, 512]:
        raise ValueError('registered camera model does not match official calibration')
    np.testing.assert_allclose(actual['intrinsics'], camera['intrinsics'] + camera['distortion_coeffs'], rtol=0, atol=1e-10)
    transform = np.array(external['T_cam_imu'])
    np.testing.assert_allclose(np.array(actual['R_imu_to_camera']).reshape(3, 3), transform[:3, :3], rtol=0, atol=1e-6)
    np.testing.assert_allclose(actual['p_imu_in_camera'], transform[:3, 3], rtol=0, atol=1e-12)
    np.testing.assert_allclose(actual['imu_noise'], [imu[n] for n in ('gyroscope_noise_density',
        'gyroscope_random_walk', 'accelerometer_noise_density', 'accelerometer_random_walk')], rtol=0, atol=1e-12)
    inverse = np.array(external['T_imu_cam'])
    np.testing.assert_allclose(transform @ inverse, np.eye(4), rtol=0, atol=1e-6)
    assert round(external['timeshift_cam_imu'] * 1e9) == -40380655
    assert actual['additional_camera_offset_s'] == 0
    return {'status': 'passed', 'official_extrinsic_direction': 'IMU to camera',
            'camera_offset_applied_in_common_input_ns': -40380655,
            'additional_sdk_camera_offset_ns': 0, 'reference_access': 'not_performed'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable', type=Path, required=True)
    parser.add_argument('--calibration-root', type=Path, default=Path(__file__).resolve().parents[2] / 'docs/benchmarks/calibration')
    args = parser.parse_args()
    print(json.dumps(check(args.executable, args.calibration_root), sort_keys=True))
