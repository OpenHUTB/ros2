"""Validated package-resource path resolution."""

import os
from pathlib import Path


def resolve_package_file(package_share: Path, relative_path: str) -> Path:
    """Resolve a file below the package share directory without path escape.

    Symlink installs (``colcon build --symlink-install``) point resource files
    back into the source tree, so the containment check is done on the
    normalized lexical path rather than on ``resolve()``.
    """
    root = Path(os.path.normpath(str(Path(package_share).resolve())))
    normalized = Path(os.path.normpath(str(root / relative_path)))
    if normalized == root or root not in normalized.parents:
        raise ValueError('resource path must remain inside the package share directory')
    candidate = normalized.resolve()
    if not candidate.is_file():
        raise FileNotFoundError(f'package resource not found: {relative_path}')
    return candidate
