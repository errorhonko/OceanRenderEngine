"""Validate the saved measurement chain and emit aggregate checkpoint metrics.

Requires numpy and Pillow. Raw geometry, spectra, PCD and previews remain local.
"""
from pathlib import Path
import csv
import hashlib
import json
import sys
import numpy as np
from PIL import Image

root = Path(sys.argv[1])
scan = root / 'scan'
rgb = root / 'rgb'
with (scan / 'pulses.csv').open(encoding='utf-8', newline='') as stream:
    pulses = list(csv.DictReader(stream))
with (rgb / 'lidar_projection.csv').open(encoding='utf-8', newline='') as stream:
    projections = list(csv.DictReader(stream))
rig = json.loads((rgb / 'camera_rig.json').read_text(encoding='utf-8'))

def cloud(path):
    lines = path.read_text(encoding='utf-8').splitlines()
    begin = lines.index('DATA ascii') + 1
    header = dict(line.split(' ', 1) for line in lines[:begin] if ' ' in line)
    data = np.array([[float(value) for value in row.split()]
                     for row in lines[begin:] if row.strip()])
    assert int(header['POINTS']) == len(data)
    return data, header

world, world_header = cloud(scan / 'urban_world.pcd')
sensor, sensor_header = cloud(scan / 'urban_sensor.pcd')
assert len(pulses) == 11520
assert len(projections) == len(world) == len(sensor)
assert np.isfinite(world).all() and np.isfinite(sensor).all()
assert len({row['pulse_id'] for row in pulses}) == len(pulses)
detected = [row for row in pulses if row['detected'] == '1']
assert len(detected) == len(world)
assert all(float(row['received_energy_J']) >= 0 for row in pulses)
assert all(float(row['recorded_energy_J']) >= 0 for row in pulses)
assert all(float(row['recorded_energy_J']) <= float(row['received_energy_J']) * (1 + 1e-10)
           for row in pulses)
fields = world_header['FIELDS'].split()
range_index = fields.index('range')
assert np.allclose(np.linalg.norm(sensor[:, :3], axis=1), sensor[:, range_index],
                   rtol=1e-6, atol=2e-5)
ranges = np.array([float(row['range_m']) for row in detected])
assert np.allclose(world[:, range_index], ranges, rtol=1e-6, atol=2e-5)
assert all(row['adc_out_of_range'] == '0' for row in pulses)
empty = sum(int(row['mc_hits']) == 0 for row in pulses)
no_return = sum(int(row['mc_hits']) > 0 and int(row['mc_returns']) == 0 for row in pulses)
below_threshold = sum(int(row['mc_returns']) > 0 and row['detected'] == '0' for row in pulses)
assert empty + no_return + below_threshold + len(detected) == len(pulses)
differences = np.array([abs(float(row['range_minus_center_m'])) for row in detected
                        if row['center_hit'] == '1'])
with (scan / 'material_bindings.tsv').open(encoding='utf-8', newline='') as stream:
    materials = list(csv.DictReader(stream, delimiter='\t'))
assert sum(row['source'] == 'spectrum' for row in materials) == 1
assert sum(row['source'] == 'assumed_50_percent' for row in materials) == 76
with (scan / 'placement_report.csv').open(encoding='utf-8', newline='') as stream:
    placements = list(csv.DictReader(stream))
assert len(placements) == 42
assert sum(row['tire_supports'] == '1' for row in placements) == 9
assert sum(row['terrain_grounded'] == '1' for row in placements) == 20

hashes = {name: hashlib.sha256((scan / name).read_bytes()).hexdigest()
          for name in ('urban_world.pcd', 'urban_sensor.pcd', 'pulses.csv')}
metrics = {
    'checkpoint': '2026-10-19',
    'grounding': 'TerrainContact; translation-only sampled support constraints',
    'geometry': {'assets': 27, 'parts': len(placements), 'triangles': 2445341},
    'scan': {
        'channels': 16, 'azimuth_columns': 720, 'pulses': len(pulses),
        'rays_per_pulse': 16, 'emitted_rays': len(pulses) * 16,
        'measured_points': len(world), 'detection_fraction': len(world) / len(pulses),
        'no_mc_hit': empty, 'hit_without_positive_return': no_return,
        'positive_return_without_detection': below_threshold, 'adc_clipped_pulses': 0,
        'minimum_range_m': float(ranges.min()), 'maximum_range_m': float(ranges.max()),
        'points_without_center_ray_hit': sum(row['center_hit'] == '0' for row in detected),
        'center_reference_abs_difference_median_m': float(np.median(differences)),
        'center_reference_abs_difference_p95_m': float(np.quantile(differences, .95)),
        'center_reference_abs_difference_max_m': float(differences.max()),
        'center_reference_warning': 'Not pure range errors: finite footprints can measure different surfaces.',
    },
    'materials': {'spectral_entries': 1, 'assumed_half_reflectance_entries': 76,
                  'unassigned_faces': 'explicit empirical fallback'},
    'camera': {key: rig[key] for key in (
        'width', 'height', 'fx', 'fy', 'cx', 'cy', 'horizontal_fov_degrees',
        'rig_origin_world_m', 'rig_orientation_wxyz', 'camera_to_lidar_translation_m',
        'camera_to_lidar_orientation_wxyz')},
    'projection': {
        'in_view': sum(row['in_image'] == '1' for row in projections),
        'overlay_points': sum(row['center_geometry_consistent'] == '1' for row in projections),
        'warning': 'The geometry-consistency filter affects only the overlay, never measured PCD points.',
    },
    'sha256': hashes,
    'scope': 'Static, noise-free receiver; empirical NIR materials; diffuse RGB preview; not calibrated hardware or SLAM.',
}
(root / 'metrics.json').write_text(json.dumps(metrics, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
comparison = Image.open(rgb / 'rgb_lidar_comparison.png').convert('RGB')
scan_image = Image.open(scan / 'urban_scan.png').convert('RGB')
comparison = comparison.resize((scan_image.width, round(comparison.height * scan_image.width / comparison.width)), Image.Resampling.LANCZOS)
overview = Image.new('RGB', (scan_image.width, comparison.height + scan_image.height), 'white')
overview.paste(comparison, (0, 0))
overview.paste(scan_image, (0, comparison.height))
overview.save(root / 'urban_checkpoint.png')
print(json.dumps(metrics, ensure_ascii=False, indent=2))
print('[PASS] Saved PCD, pulse, material, placement and projection counts independently reconciled.')
