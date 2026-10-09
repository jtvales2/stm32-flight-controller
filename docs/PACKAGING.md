# Repository-only packaging

The repository root is `OpenSource/stm32-flight-controller/`. Never upload or zip the parent `FlightController/`, because it also contains the local `Original/` backup. The current local repository is on the desktop; the former D-drive path is no longer present.

From the repository root, create a new archive outside the repository:

```powershell
python docs/verification/package_release.py --output ..\stm32-flight-controller-pre-release.zip
```

The script checks the exact root name and required firmware/build files, rejects a parent directory, refuses to overwrite an existing archive, excludes backups, Git internals, user sessions and generated products, and uses `stm32-flight-controller/` as the single archive root. It verifies CRC and compares every archived file's bytes with the source. The output contains source, documentation and media; no `Original/` backup is included.

For GitHub, initialize or upload from this exact repository root. Keep the original backup locally. The archive is a local preparation deliverable: resolve the documented license/ownership questions and add the owner's real evidence before describing it as the final portfolio release. No upload or publication is performed by this preparation.
