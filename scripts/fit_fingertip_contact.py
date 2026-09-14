#!/usr/bin/env python3
"""Fit a fixed finger-pad point in DIP-link coordinates from fixed-contact poses.

Input JSON: {"poses": [{"position_m": [x,y,z], "rotation": [[...], [...], [...]]}, ...]}.
All poses must use the SAME stationary reference frame. Rotate the DIP link while
maintaining the marked finger-pad point against the same fixture point.
"""
import argparse
import json
from pathlib import Path
import numpy as np
import yaml


def fit_contact(poses):
    if len(poses) < 6:
        raise ValueError("Need at least six varied DIP poses")
    positions = np.asarray([p['position_m'] for p in poses], dtype=float)
    rotations = np.asarray([p['rotation'] for p in poses], dtype=float)
    if positions.shape != (len(poses), 3) or rotations.shape != (len(poses), 3, 3):
        raise ValueError("Expected position[3] and rotation[3,3]")
    if not np.isfinite(positions).all() or not np.isfinite(rotations).all():
        raise ValueError("Non-finite sample")
    if not np.allclose(rotations.transpose(0,2,1) @ rotations, np.eye(3), atol=1e-5) or not np.allclose(np.linalg.det(rotations), 1., atol=1e-5):
        raise ValueError("Rotations must be proper orthonormal matrices")
    a = np.concatenate([np.concatenate([r, -np.eye(3)], axis=1) for r in rotations])
    solution, _, rank, singular = np.linalg.lstsq(a, -positions.reshape(-1), rcond=None)
    if rank < 6 or singular[0] / singular[-1] > 1e4:
        raise ValueError("Insufficient independent rotations; offset is not observable")
    offset, contact = solution[:3], solution[3:]
    errors = np.linalg.norm(positions + rotations @ offset - contact, axis=1)
    return dict(offset_m=offset.tolist(), fixture_point_m=contact.tolist(),
                rms_error_m=float(np.sqrt(np.mean(errors**2))), max_error_m=float(errors.max()),
                condition_number=float(singular[0]/singular[-1]), sample_count=len(poses))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('samples', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--model', required=True)
    parser.add_argument('--side', choices=['left','right'], required=True)
    parser.add_argument('--finger', choices=['thumb','index','middle','ring','little'], default='thumb')
    parser.add_argument('--parent-link', required=True)
    parser.add_argument('--reference-frame', required=True)
    parser.add_argument('--calibration-id', required=True)
    args = parser.parse_args()
    fit = fit_contact(json.loads(args.samples.read_text())['poses'])
    document = dict(schema_version=1, model=args.model, side=args.side, finger=args.finger,
                    parent_link=args.parent_link, reference_frame=args.reference_frame,
                    calibration_id=args.calibration_id, point_definition='marked_finger_pad_contact',
                    status='fitted_not_independently_validated', **fit)
    args.output.write_text(yaml.safe_dump(document, sort_keys=False))
    print(f"RMS={fit['rms_error_m']*1000:.3f} mm; validate on held-out poses before applying upstream")

if __name__ == '__main__':
    main()
