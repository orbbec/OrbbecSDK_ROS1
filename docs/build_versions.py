#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
DOCS_DIR = REPO_ROOT / "docs"
MANIFEST_PATH = DOCS_DIR / "version-manifest.json"
SITE_DIR = REPO_ROOT / "site"
WORKTREE_ROOT = REPO_ROOT / ".docs-worktrees"
LANGUAGES = ("en", "zh")


def run(command: list[str], *, cwd: Path | None = None, env: dict[str, str] | None = None) -> subprocess.CompletedProcess[str]:
    print("+", " ".join(command))
    return subprocess.run(
        command,
        cwd=str(cwd or REPO_ROOT),
        env=env,
        check=True,
        text=True,
    )


def capture(command: list[str], *, cwd: Path | None = None) -> str:
    result = subprocess.run(
        command,
        cwd=str(cwd or REPO_ROOT),
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    return result.stdout.strip()


def load_manifest() -> dict:
    return json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))


def ensure_ref_available(ref: str) -> None:
    if ref == "HEAD":
        return

    if ref_resolves(ref) or resolve_remote_ref(ref) is not None:
        return

    fetch_targets = [ref]
    if "/" not in ref and not ref.startswith("refs/"):
        fetch_targets.append(f"refs/heads/{ref}")

    for remote in list_remotes():
        for target in fetch_targets:
            try:
                run(["git", "fetch", "--no-tags", remote, target])
                if ref_resolves(ref) or resolve_remote_ref(ref) is not None:
                    return
            except subprocess.CalledProcessError:
                continue

    raise RuntimeError(f"Unable to resolve git ref '{ref}'.")


def list_remotes() -> list[str]:
    remotes = capture(["git", "remote"]).splitlines()
    ordered: list[str] = []
    for candidate in ("origin", "github"):
        if candidate in remotes:
            ordered.append(candidate)
    for remote in remotes:
        if remote not in ordered:
            ordered.append(remote)
    return ordered


def pick_remote() -> str:
    remotes = list_remotes()
    if not remotes:
        raise RuntimeError("No git remote found to fetch documentation refs.")
    return remotes[0]


def ref_resolves(ref: str) -> bool:
    result = subprocess.run(
        ["git", "rev-parse", "--verify", f"{ref}^{{commit}}"],
        cwd=str(REPO_ROOT),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    return result.returncode == 0


def resolve_ref(ref: str) -> str:
    if ref_resolves(ref):
        return ref

    remote_ref = resolve_remote_ref(ref)
    if remote_ref is not None:
        return remote_ref

    raise RuntimeError(f"Unable to resolve git ref '{ref}'.")


def resolve_remote_ref(ref: str) -> str | None:
    for remote in list_remotes():
        remote_ref = f"refs/remotes/{remote}/{ref}"
        if ref_resolves(remote_ref):
            return remote_ref
    return None


def should_use_workspace(version: dict) -> bool:
    flag = os.getenv("DOCS_BUILD_LATEST_FROM_WORKSPACE", "0").lower()
    return version.get("is_latest", False) and flag in {"1", "true", "yes", "on"}


def prepare_site_dir() -> None:
    if SITE_DIR.exists():
        shutil.rmtree(SITE_DIR)
    SITE_DIR.mkdir(parents=True, exist_ok=True)


def create_root_redirect(default_language: str) -> None:
    target = f"{default_language}/index.html"
    (SITE_DIR / "index.html").write_text(
        (
            "<!DOCTYPE html><html><head>"
            f'<meta http-equiv="refresh" content="0; url={target}">'
            f'<link rel="canonical" href="{target}">'
            "</head><body>Redirecting...</body></html>"
        ),
        encoding="utf-8",
    )
    (SITE_DIR / ".nojekyll").touch()


def copy_tree(source_dir: Path, target_dir: Path) -> None:
    if target_dir.exists():
        shutil.rmtree(target_dir)
    shutil.copytree(source_dir, target_dir)


def resolve_source_dir(content_root: Path, language: str) -> Path:
    candidates = [
        content_root / "docs" / language,
        content_root / "docs",
    ]
    for candidate in candidates:
        if candidate.exists() and ((candidate / "index.rst").exists() or (candidate / "index.md").exists()):
            return candidate

    fallback = REPO_ROOT / "docs" / language
    if content_root != REPO_ROOT and fallback.exists():
        print(
            "WARNING: "
            f"Falling back to current workspace docs for language '{language}' "
            f"because {content_root} does not contain a buildable multilingual docs layout."
        )
        return fallback

    raise RuntimeError(f"Missing documentation source directory for language '{language}' in {content_root}")


def build_language(version: dict, content_root: Path, language: str, output_dir: Path) -> None:
    source_dir = resolve_source_dir(content_root, language)
    conf_dir = DOCS_DIR / language

    env = os.environ.copy()
    env.update(
        {
            "DOCS_VERSION_SLUG": version["slug"],
            "DOCS_VERSION_LABEL": version.get("label", version["slug"]),
            "DOCS_VERSION_IS_LATEST": "1" if version.get("is_latest") else "0",
            "DOCS_VERSION_MANIFEST_JSON": str(MANIFEST_PATH),
            "DOCS_CONTENT_ROOT": str(source_dir),
        }
    )

    run(
        [
            "sphinx-build",
            "-b",
            "html",
            "-c",
            str(conf_dir),
            str(source_dir),
            str(output_dir),
        ],
        env=env,
    )


def build_version(version: dict) -> None:
    version_root = get_content_root(version)
    try:
        for language in LANGUAGES:
            temp_output_dir = WORKTREE_ROOT / ".build" / version["slug"] / language
            if temp_output_dir.exists():
                shutil.rmtree(temp_output_dir)
            temp_output_dir.parent.mkdir(parents=True, exist_ok=True)

            build_language(version, version_root, language, temp_output_dir)

            if version.get("is_latest"):
                target_dir = SITE_DIR / language
            else:
                target_dir = SITE_DIR / "versions" / version["slug"] / language
            target_dir.parent.mkdir(parents=True, exist_ok=True)
            copy_tree(temp_output_dir, target_dir)
    finally:
        cleanup_content_root(version_root)


def get_content_root(version: dict) -> Path:
    if should_use_workspace(version):
        return REPO_ROOT

    ref = version.get("ref", "HEAD")
    ensure_ref_available(ref)
    resolved_ref = resolve_ref(ref)

    worktree_dir = WORKTREE_ROOT / version["slug"]
    if worktree_dir.exists():
        run(["git", "worktree", "remove", "--force", str(worktree_dir)])

    worktree_dir.parent.mkdir(parents=True, exist_ok=True)
    run(["git", "worktree", "add", "--detach", str(worktree_dir), resolved_ref])
    return worktree_dir


def cleanup_content_root(content_root: Path) -> None:
    if content_root == REPO_ROOT:
        return

    if content_root.exists():
        run(["git", "worktree", "remove", "--force", str(content_root)])


def validate_manifest(manifest: dict) -> list[dict]:
    published = [version for version in manifest.get("versions", []) if version.get("published")]
    if not published:
        raise RuntimeError("No published documentation versions found in manifest.")
    if sum(1 for version in published if version.get("is_latest")) != 1:
        raise RuntimeError("Manifest must contain exactly one published latest version.")
    return published


def main() -> int:
    manifest = load_manifest()
    published_versions = validate_manifest(manifest)

    prepare_site_dir()
    WORKTREE_ROOT.mkdir(parents=True, exist_ok=True)

    for version in published_versions:
        build_version(version)

    create_root_redirect(manifest.get("default_language", "en"))
    print(f"Built documentation site in {SITE_DIR}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:  # noqa: BLE001
        print(f"ERROR: {error}", file=sys.stderr)
        raise
