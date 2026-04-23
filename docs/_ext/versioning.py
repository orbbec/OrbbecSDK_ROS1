from __future__ import annotations

import json
import os
from pathlib import Path


EXTENSION_DIR = Path(__file__).resolve().parent
DOCS_DIR = EXTENSION_DIR.parent
DEFAULT_MANIFEST_PATH = DOCS_DIR / "version-manifest.json"


def load_manifest() -> dict:
    manifest_path = Path(os.getenv("DOCS_VERSION_MANIFEST_JSON", DEFAULT_MANIFEST_PATH)).resolve()
    return json.loads(manifest_path.read_text(encoding="utf-8"))


def get_content_root(conf_file: str) -> Path:
    value = os.getenv("DOCS_CONTENT_ROOT")
    if value:
        return Path(value).resolve()
    return Path(conf_file).resolve().parent


def get_conf_root(conf_file: str) -> Path:
    return Path(conf_file).resolve().parent


def _unique_paths(paths: list[Path]) -> list[str]:
    unique: list[str] = []
    seen: set[str] = set()
    for path in paths:
        resolved = str(path.resolve())
        if resolved in seen or not path.exists():
            continue
        seen.add(resolved)
        unique.append(str(path))
    return unique


def get_templates_paths(conf_root: Path) -> list[str]:
    candidates = [
        conf_root / "_templates",
        conf_root / "source" / "_templates",
    ]
    return _unique_paths(candidates)


def get_static_paths(conf_root: Path, content_root: Path) -> list[str]:
    candidates = [
        content_root / "source" / "_static",
        conf_root / "source" / "_static",
        content_root / "source" / "image",
    ]
    return _unique_paths(candidates)


def get_extra_paths(content_root: Path) -> list[str]:
    candidate = content_root / "source" / "image"
    return _unique_paths([candidate])


def get_asset_path(content_root: Path, relative_path: str, fallback: str) -> str:
    candidate = content_root / relative_path
    if candidate.exists():
        return str(candidate)
    return fallback


def build_version_context(language: str) -> dict:
    manifest = load_manifest()
    published_versions = [
        {
            "slug": version["slug"],
            "label": version.get("label", version["slug"]),
            "is_latest": bool(version.get("is_latest", False)),
        }
        for version in manifest.get("versions", [])
        if version.get("published", False)
    ]

    current_slug = os.getenv("DOCS_VERSION_SLUG", manifest.get("latest_slug", "latest"))
    matched = next((version for version in published_versions if version["slug"] == current_slug), None)
    current_label = os.getenv("DOCS_VERSION_LABEL") or (matched["label"] if matched else current_slug)

    current_is_latest_env = os.getenv("DOCS_VERSION_IS_LATEST")
    if current_is_latest_env is None:
        current_is_latest = bool(matched["is_latest"]) if matched else current_slug == manifest.get("latest_slug")
    else:
        current_is_latest = current_is_latest_env.lower() in {"1", "true", "yes", "on"}

    return {
        "default_language": manifest.get("default_language", "en"),
        "latest_slug": manifest.get("latest_slug", current_slug),
        "current": {
            "slug": current_slug,
            "label": current_label,
            "is_latest": current_is_latest,
            "language": language,
        },
        "versions": published_versions,
    }
