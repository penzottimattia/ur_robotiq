#!/usr/bin/env python3
"""Compute transform of right base frame w.r.t left base frame.

Reads two YAML files that contain a camera->base transform (field
`camera_to_base`) for the left and right robot bases, computes
left_base -> right_base = (left_base -> camera) * inv(right_base -> camera)
and writes the result to an output YAML file (same format as existing
camera_to_base files).
"""

import argparse
import os
import sys
import yaml
import numpy as np

try:
    from tf_transformations import quaternion_matrix, quaternion_from_matrix, euler_from_matrix
except Exception:
    import tf_transformations as tft
    quaternion_matrix = tft.quaternion_matrix
    quaternion_from_matrix = tft.quaternion_from_matrix
    euler_from_matrix = tft.euler_from_matrix


def load_camera_to_base(path):
    with open(path, 'r') as fh:
        data = yaml.safe_load(fh)

    if 'camera_to_base' not in data:
        raise RuntimeError(f'File {path} does not contain key "camera_to_base"')

    hdr = data['camera_to_base'].get('header', {})
    frame_id = hdr.get('frame_id')
    child_frame_id = hdr.get('child_frame_id')

    tf = data['camera_to_base'].get('transform', {})
    trans = tf.get('translation', {})
    q = tf.get('rotation_xyzw') or tf.get('rotation') or {}

    # YAML stores quaternion as w,x,y,z keys — convert to x,y,z,w for tf utilities
    qw = float(q.get('w', 0.0))
    qx = float(q.get('x', 0.0))
    qy = float(q.get('y', 0.0))
    qz = float(q.get('z', 0.0))
    q_xyzw = [qx, qy, qz, qw]

    mat = quaternion_matrix(q_xyzw)
    mat[0:3, 3] = [float(trans.get('x', 0.0)), float(trans.get('y', 0.0)), float(trans.get('z', 0.0))]

    return mat, frame_id, child_frame_id


def matrix_to_yaml_dict(mat, parent_frame, child_frame):
    t = mat[0:3, 3]
    q_xyzw = quaternion_from_matrix(mat)  # returns x,y,z,w
    rpy = euler_from_matrix(mat)

    return {
        'camera_to_base': {
            'header': {
                'frame_id': parent_frame,
                'child_frame_id': child_frame,
            },
            'transform': {
                'translation': {'x': float(t[0]), 'y': float(t[1]), 'z': float(t[2])},
                'rotation': {'x': float(q_xyzw[0]), 'y': float(q_xyzw[1]), 'z': float(q_xyzw[2]), 'w': float(q_xyzw[3])},
                'rpy': {'r': float(rpy[0]), 'p': float(rpy[1]), 'y': float(rpy[2])},
            }
        }
    }


def main(argv=None):
    p = argparse.ArgumentParser(description='Compute right base frame w.r.t left base using camera->base YAMLs')
    p.add_argument('--left', '-l', required=True, help='left camera_to_base YAML (left_base -> camera)')
    p.add_argument('--right', '-r', required=True, help='right camera_to_base YAML (right_base -> camera)')
    p.add_argument('--output', '-o', default=os.path.join('config', 'right_base_in_left_base.yaml'),
                   help='output YAML file to write left_base -> right_base transform')
    args = p.parse_args(argv)

    left_mat, left_frame, left_child = load_camera_to_base(args.left)
    right_mat, right_frame, right_child = load_camera_to_base(args.right)

    # sanity checks
    if left_child != right_child and left_child is not None and right_child is not None:
        print(f'Warning: left child frame "{left_child}" != right child frame "{right_child}"', file=sys.stderr)

    if left_frame is None or right_frame is None:
        raise RuntimeError('Could not determine frame ids from input YAMLs')

    # left_base -> right_base = (left_base -> camera) * inv(right_base -> camera)
    try:
        inv_right = np.linalg.inv(right_mat)
    except np.linalg.LinAlgError as e:
        raise RuntimeError(f'Failed to invert right matrix: {e}')

    left_to_right = left_mat.dot(inv_right)

    out_dict = matrix_to_yaml_dict(left_to_right, parent_frame=left_frame, child_frame=right_frame)

    os.makedirs(os.path.dirname(args.output) or '.', exist_ok=True)
    with open(args.output, 'w') as fh:
        yaml.safe_dump(out_dict, fh, default_flow_style=False)

    print(f'Wrote transform left:"{left_frame}" -> right:"{right_frame}" to {args.output}')


if __name__ == '__main__':
    main()
