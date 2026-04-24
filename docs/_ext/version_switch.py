from __future__ import annotations

import json
from pathlib import Path

from docutils import nodes


STATIC_DIR = Path(__file__).resolve().parent / "static"


def setup(app):
    app.connect("config-inited", configure_assets)
    app.connect("doctree-resolved", inject_version_switch)
    app.add_css_file("version_switch.css")
    app.add_js_file("version_switch.js")
    return {"version": "0.1", "parallel_read_safe": True}


def configure_assets(app, config):
    static_dir = str(STATIC_DIR)
    if static_dir not in config.html_static_path:
        config.html_static_path.append(static_dir)


def inject_version_switch(app, doctree, docname):
    if app.builder.format != "html":
        return

    versioning = getattr(app.config, "html_context", {}).get("docs_versioning")
    if not versioning or not versioning.get("versions"):
        return

    payload = json.dumps(versioning, ensure_ascii=False).replace("</", "<\\/")
    current_label = versioning.get("current", {}).get("label", "")

    html = f"""
<div class="docs-version-switch" data-docs-version-switch>
  <div class="docs-version-switch__header">Version</div>
  <div class="docs-version-switch__control">
    <label class="docs-version-switch__label" for="docs-version-select">Version</label>
    <select id="docs-version-select" class="docs-version-switch__select" aria-label="Select documentation version">
      <option value="">{current_label}</option>
    </select>
  </div>
</div>
<script id="docs-version-data" type="application/json">{payload}</script>
"""
    doctree.insert(0, nodes.raw("", html, format="html"))
