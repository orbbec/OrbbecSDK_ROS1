"""Build a section-level JSON index for custom documentation search."""

from __future__ import annotations

import json
import re
from pathlib import Path

from docutils import nodes


INDEX_FILENAME = "section-search-index.json"
INDEX_SCRIPT_FILENAME = "section-search-index.js"
WHITESPACE_RE = re.compile(r"\s+")
EXTENSION_DIR = Path(__file__).resolve().parent
STATIC_DIR = EXTENSION_DIR / "static"
TEMPLATES_DIR = EXTENSION_DIR / "templates"


def setup(app):
    app.connect("config-inited", configure_assets)
    app.connect("build-finished", write_section_search_index)
    app.add_js_file("section_search.js")
    return {"version": "0.1", "parallel_read_safe": True}


def configure_assets(app, config):
    static_dir = str(STATIC_DIR)
    templates_dir = str(TEMPLATES_DIR)

    if static_dir not in config.html_static_path:
        config.html_static_path.append(static_dir)
    if templates_dir not in config.templates_path:
        config.templates_path.append(templates_dir)


def write_section_search_index(app, exception):
    if exception is not None or app.builder.format != "html":
        return

    entries = []
    env = app.builder.env

    for docname in sorted(env.found_docs):
        if docname in {"search", "genindex"}:
            continue

        doctree = env.get_doctree(docname)
        target_uri = app.builder.get_target_uri(docname)
        page_title = normalize_text(env.titles[docname].astext()) if docname in env.titles else docname

        document_text = collect_direct_text(doctree)
        if document_text or page_title:
            entries.append(
                {
                    "kind": "document",
                    "title": page_title,
                    "page_title": page_title,
                    "anchor": "",
                    "url": target_uri,
                    "text": document_text,
                }
            )

        for section in doctree.findall(nodes.section):
            title = get_section_title(section)
            text = collect_direct_text(section)
            section_ids = section.get("ids", [])
            anchor = section_ids[0] if section_ids else ""
            url = f"{target_uri}#{anchor}" if anchor else target_uri

            if not title and not text:
                continue

            entries.append(
                {
                    "kind": "section",
                    "title": title or page_title,
                    "page_title": page_title,
                    "anchor": anchor,
                    "url": url,
                    "text": text,
                }
            )

    payload = {
        "language": app.config.language or "en",
        "entries": entries,
    }
    output_dir = Path(app.outdir) / "_static"
    output_dir.mkdir(parents=True, exist_ok=True)

    (output_dir / INDEX_FILENAME).write_text(
        json.dumps(payload, ensure_ascii=False, separators=(",", ":")),
        encoding="utf-8",
    )
    (output_dir / INDEX_SCRIPT_FILENAME).write_text(
        "window.SECTION_SEARCH_INDEX = "
        + json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        + ";",
        encoding="utf-8",
    )


def get_section_title(section: nodes.section) -> str:
    for child in section.children:
        if isinstance(child, nodes.title):
            return normalize_text(child.astext())
    return ""


def collect_direct_text(container: nodes.Element) -> str:
    parts = []
    for child in container.children:
        if isinstance(child, (nodes.section, nodes.title)):
            continue

        text = normalize_text(child.astext())
        if text:
            parts.append(text)

    return " ".join(parts)


def normalize_text(value: str) -> str:
    return WHITESPACE_RE.sub(" ", value).strip()
