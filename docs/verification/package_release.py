#!/usr/bin/env python3
"""Package only the stm32-flight-controller repository, never its backup parent."""
from pathlib import Path
import argparse
import hashlib
import json
import zipfile

EXCLUDED_SUFFIXES = {'.o', '.obj', '.crf', '.d', '.dep', '.axf', '.hex', '.map', '.lst', '.lnp', '.sbr', '.pbi'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root, output = args.repo.resolve(), args.output.resolve()
    required = ['README.md', 'BUILD.md', 'THIRD_PARTY_NOTICES.md', '.gitignore',
                'firmware/stm32/MDK-ARM/fly1.0.uvprojx', 'firmware/stm32/fly1.0.ioc']
    if root.name != 'stm32-flight-controller' or not all((root / p).is_file() for p in required):
        raise SystemExit('Refusing to package a parent folder or unexpected repository layout.')
    if output.exists() or output.suffix.lower() != '.zip':
        raise SystemExit('Choose a new .zip output path; existing archives are not overwritten.')
    try:
        output.relative_to(root)
    except ValueError:
        pass
    else:
        raise SystemExit('Archive must be outside the repository root.')
    selected = []
    for path in sorted(root.rglob('*')):
        if not path.is_file():
            continue
        relative = path.relative_to(root)
        parts = relative.parts
        if any(part.lower() in {'original', '.git', '.qa', '__pycache__'} for part in parts):
            continue
        if relative.as_posix().startswith(('firmware/stm32/MDK-ARM/fly1.0/',
                                          'firmware/stm32/MDK-ARM/DebugConfig/')):
            continue
        if path.suffix.lower() in EXCLUDED_SUFFIXES or '.uvguix.' in path.name.lower() or path.suffix.lower() in {'.uvoptx', '.uvopt'}:
            continue
        try:
            path.resolve().relative_to(root)
        except ValueError:
            raise SystemExit(f'File escapes repository boundary: {relative}')
        selected.append((path, relative))
    output.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(output, 'x', compression=zipfile.ZIP_DEFLATED, compresslevel=9) as archive:
        for path, relative in selected:
            archive.write(path, root.name + '/' + relative.as_posix())
    with zipfile.ZipFile(output) as archive:
        names = archive.namelist()
        if archive.testzip() is not None:
            raise SystemExit('Archive CRC check failed.')
        assert len(names) == len(selected)
        assert all(name.startswith('stm32-flight-controller/') for name in names)
        assert not any('original' in [part.lower() for part in name.split('/')] for name in names)
        for path, relative in selected:
            assert archive.read(root.name + '/' + relative.as_posix()) == path.read_bytes(), relative
    print(json.dumps({'status': 'PASS', 'archive': str(output), 'files': len(selected),
                      'bytes': output.stat().st_size,
                      'sha256': hashlib.sha256(output.read_bytes()).hexdigest(),
                      'archive_root': 'stm32-flight-controller/', 'contains_original_backup': False,
                      'publication_status': 'Local preparation archive; license and hardware status remain documented.'}, indent=2))


if __name__ == '__main__':
    main()
