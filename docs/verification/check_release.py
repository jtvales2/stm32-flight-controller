#!/usr/bin/env python3
"""Check the preparation snapshot and Keil paths; this is not a hardware test.

Run with --original and --source to verify the untouched original copies too.
The expected five-file cumulative change set describes this preparation snapshot only.
Future intentional source changes require a new baseline.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import re
import xml.etree.ElementTree as ET
from pathlib import Path

EXPECTED_CHANGES = {
    'Core/Src/sbus.c',
    'Core/Src/imu_bringup.c',
    'FlightController/fc_core/fc_core.c',
    'FlightController/fc_cfg/fc_cfg.h',
    'MDK-ARM/fly1.0.uvprojx',
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--original', type=Path)
    parser.add_argument('--source', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    root = args.repo.resolve()
    fw = root / 'firmware/stm32'
    with (root / 'docs/original-manifest.csv').open(encoding='utf-8-sig', newline='') as stream:
        rows = list(csv.DictReader(stream))
    manifest = {row['path']: row for row in rows}
    require(len(manifest) == len(rows), 'Duplicate manifest paths')
    removed = set((root / 'docs/removed-files.txt').read_text(encoding='utf-8-sig').splitlines())
    require(removed <= manifest.keys(), 'Removal record contains an unknown file')
    retained, changed = {}, set()
    for relative, row in manifest.items():
        path = fw / relative
        if relative in removed:
            require(not path.exists(), f'Removed file reappeared: {relative}')
            continue
        require(path.is_file(), f'Missing retained file: {relative}')
        digest = sha256(path)
        retained[relative] = digest
        if digest != row['sha256']:
            changed.add(relative)
    require(changed == EXPECTED_CHANGES, f'Unexpected change set: {sorted(changed)}')
    for relative in manifest:
        if relative.startswith(('Core/', 'FlightController/')):
            require((fw / relative).is_file(), f'Lost application file: {relative}')
    originals_checked = {}
    for name, folder in (('original', args.original), ('source', args.source)):
        if folder:
            files = {str(p.relative_to(folder)).replace('\\', '/') for p in folder.rglob('*') if p.is_file()}
            require(files == manifest.keys(), f'{name}: file inventory changed')
            for relative, row in manifest.items():
                require(sha256(folder / relative) == row['sha256'], f'{name}: bytes changed: {relative}')
            originals_checked[name] = len(manifest)
    phase_one = json.loads((root / 'docs/verification/release-audit.json').read_text(encoding='utf-8'))
    phase_one_hashes = phase_one['firmware_sha256']
    phase_two_changes = {rel for rel, digest in retained.items() if digest != phase_one_hashes[rel]}
    require(phase_two_changes == {'Core/Src/imu_bringup.c', 'FlightController/fc_core/fc_core.c'},
            f'Unexpected follow-up firmware changes: {sorted(phase_two_changes)}')
    project = fw / 'MDK-ARM/fly1.0.uvprojx'
    xml = ET.parse(project)
    file_paths = [node.text for node in xml.findall('.//FilePath') if node.text]
    for relative in file_paths:
        require((project.parent / relative.replace('\\', '/')).is_file(), f'Missing project input: {relative}')
    include_paths = []
    for node in xml.findall('.//IncludePath'):
        if node.text:
            include_paths.extend(p for p in node.text.split(';') if p)
    for relative in include_paths:
        require('mavlink2' not in relative, f'Unused MAVLink path remains: {relative}')
        require((project.parent / relative.replace('\\', '/')).is_dir(), f'Missing include path: {relative}')
    cfg = (fw / 'FlightController/fc_cfg/fc_cfg.h').read_bytes()
    for name in (b'FC_MOTOR_TEST_ENABLE', b'FC_ESC_CAL_ENABLE'):
        require(re.search(rb'^#define\s+' + name + rb'\s+0\s*$', cfg, re.M) is not None,
                f'Tool mode remains enabled: {name.decode()}')
    for relative in ('Drivers/CMSIS/LICENSE.txt', 'Drivers/STM32F4xx_HAL_Driver/LICENSE.txt',
                     'Drivers/CMSIS/Device/ST/STM32F4xx/LICENSE.txt'):
        require(sha256(fw / relative) == manifest[relative]['sha256'], f'License changed: {relative}')
    for relative in ('Core/Src/sbus.c', 'Core/Src/imu_bringup.c', 'FlightController/fc_core/fc_core.c',
                     'FlightController/fc_cfg/fc_cfg.h'):
        if args.original:
            before = (args.original / relative).read_bytes()
            after = (fw / relative).read_bytes()
            require(bytes(b for b in before if b > 127) == bytes(b for b in after if b > 127),
                    f'Historical comment bytes changed: {relative}')
    unwanted_dirs = ('MDK-ARM/fly1.0', 'MDK-ARM/DebugConfig', 'mavlink2', 'Drivers/CMSIS/DSP',
                     'Drivers/CMSIS/NN', 'Drivers/CMSIS/Documentation', 'Drivers/CMSIS/DAP',
                     'Drivers/CMSIS/Core_A', 'Drivers/CMSIS/RTOS', 'Drivers/CMSIS/RTOS2')
    for relative in unwanted_dirs:
        require(not (fw / relative).exists(), f'Unwanted directory remains: {relative}')
    actual_files = {str(p.relative_to(fw)).replace('\\', '/') for p in fw.rglob('*') if p.is_file()}
    require(actual_files == retained.keys(), f'Unexpected firmware files: {sorted(actual_files - retained.keys())}')
    report = {
        'status': 'PASS',
        'original_files': len(manifest),
        'removed_files': len(removed),
        'retained_firmware_files': len(retained),
        'byte_identical_retained_files': len(retained) - len(changed),
        'modified_files': sorted(changed),
        'originals_hash_verified': originals_checked,
        'keil_file_references': len(file_paths),
        'keil_include_paths': len(include_paths),
        'firmware_sha256': dict(sorted(retained.items())),
        'phase_two_modified_files': sorted(phase_two_changes),
        'scope': 'Follow-up snapshot; phase-one baseline sealed; no hardware validation.',
    }
    if args.output:
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    compact = {key: value for key, value in report.items() if key != 'firmware_sha256'}
    print(json.dumps(compact, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, ET.ParseError) as error:
        raise SystemExit(f'FAIL: {error}')
