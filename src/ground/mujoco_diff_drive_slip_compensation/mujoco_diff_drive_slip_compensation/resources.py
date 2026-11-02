"""Validated package-resource path resolution."""

from pathlib import Path


def resolve_package_file(package_share: Path, relative_path: str) -> Path:
    """Resolve a file below the package share directory without path escape."""
    root = package_share.resolve()
    candidate = (root / relative_path).resolve()
    if candidate == root or root not in candidate.parents:
        raise ValueError('resource path must remain inside the package share directory')
    if not candidate.is_file():
        raise FileNotFoundError(f'package resource not found: {relative_path}')
    return candidate
