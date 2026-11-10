#!/usr/bin/env python3
"""Build the public MAVMOOS documentation with the Python standard library."""

import argparse
import html
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path
from urllib.parse import quote


SITE = Path(__file__).resolve().parent
ROOT = SITE.parent
REPOSITORY = "https://github.com/cbenjamin23/mavmoos"
RELEASE_EVIDENCE = ROOT / "docs/implementation/2026-11-10-release-28/evidence/readback.json"
RELEASE_MATRIX = ROOT / "scripts/release/matrix.json"
CONTROL_MATRIX = "docs/development/control-coverage.md"
PAGES = [
    ("index", "Home", "MAVLink for MOOS-IvP"),
    ("start", "Get started", "Run MAVMOOS from scratch"),
    ("architecture", "Architecture", "How the bridge works"),
    ("maintenance", "Maintenance", "Validation and firmware maintenance"),
]
ASSETS = ("style.css", "site.js", "backseat-deployment.svg")
PROVIDER_GROUPS = {
    "Firmware-specific control": "Vehicle control",
    "Shared state and diagnostics": "State and diagnostics",
    "Shared navigation and sensor observations": "Navigation and sensors",
    "Shared protocols and payloads": "Protocols and payloads",
}
HIDDEN_SITE_PROVIDERS = {"px4_rover_control"}


def source_link(path, label, revision="main"):
    relative, _, anchor = path.partition("#")
    file = (ROOT / relative).resolve()
    if not file.is_relative_to(ROOT) or not file.is_file():
        raise ValueError(f"Missing documentation source: {relative}")
    fragment = f"#{quote(anchor)}" if anchor else ""
    url = f"{REPOSITORY}/blob/{revision}/{quote(relative)}{fragment}"
    return f'<a href="{html.escape(url)}">{html.escape(label)}</a>'


def render_page(page, title, content):
    navigation = []
    for name, label, _ in PAGES:
        current = ' aria-current="page"' if page == name else ""
        navigation.append(f'<a href="{name}.html"{current}>{label}</a>')
    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta name="description" content="MAVMOOS connects MOOS-IvP with ArduPilot and PX4 over MAVLink.">
  <title>{title} · MAVMOOS</title>
  <link rel="stylesheet" href="assets/style.css">
  <script src="assets/site.js" defer></script>
</head>
<body>
  <a class="skip" href="#main">Skip to main content</a>
  <header class="topbar">
    <div class="topbar-inner shell">
      <a class="wordmark" href="index.html">MAVMOOS</a>
      <nav aria-label="Primary navigation">{''.join(navigation)}</nav>
    </div>
  </header>
  <main class="shell" id="main" tabindex="-1">
{content}
  </main>
  <footer class="shell site-footer">
    <span>MAVMOOS</span>
    <a href="{REPOSITORY}">Source on GitHub</a>
  </footer>
</body>
</html>
"""


def read_workflows():
    command = [sys.executable, "-B", "scripts/reproduce.py", "list", "--json"]
    output = subprocess.check_output(command, cwd=ROOT, text=True)
    rows = json.loads(output)["workflows"]
    if not rows or len({row["id"] for row in rows}) != len(rows):
        raise ValueError("Expected a nonempty workflow catalog with unique IDs")
    return rows


def provider_index(workflows):
    # Use the plugin catalog to group the runnable provider recipes.
    groups = {group: [] for group in PROVIDER_GROUPS}
    path = ROOT / "docs/plugin-catalog.md"
    group = None
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.startswith("## "):
            group = line[3:]
        if group not in groups or not line.startswith(("| `", "| [`")):
            continue
        provider = line.split("|")[1].strip()
        match = re.match(r"(?:\[)?`([a-z0-9_]+)`", provider)
        if match is None:
            raise ValueError(f"Unsupported provider row in {path.name}: {provider}")
        groups[group].append(match[1])
    expected = {f"provider.{name}" for names in groups.values() for name in names}
    if expected != workflows.keys() or any(not names for names in groups.values()):
        raise ValueError("Provider groups and runnable recipes do not match")

    sections = []
    for group, names in groups.items():
        visible_names = [name for name in names if name not in HIDDEN_SITE_PROVIDERS]
        items = []
        for name in visible_names:
            workflow = workflows[f"provider.{name}"]
            label = html.escape(name)
            guide = source_link(workflow["guide"], f"Guide for {name}")
            description = html.escape(workflow["expected"])
            items.append(f"""
          <li class="provider-row">
            <div class="provider-head"><code>{label}</code>{guide}</div>
            <p>{description}</p>
          </li>""")
        sections.append(f"""
      <details class="surface plugin-group">
        <summary>{PROVIDER_GROUPS[group]} <span class="plugin-count">{len(visible_names)}</span></summary>
        <ul class="provider-list">{''.join(items)}</ul>
      </details>""")
    return "".join(sections)


def load_fragments(workflows):
    receipt = json.loads(RELEASE_EVIDENCE.read_text(encoding="utf-8"))
    if receipt["result"] != "PASS" or not receipt["independent_replays_equal"]:
        raise ValueError("Qualification summary expects the verified release verdict")
    matrix = json.loads(RELEASE_MATRIX.read_text(encoding="utf-8"))
    stress_cases = len(matrix["stress_cases"])
    catalog = {row["id"]: row for row in workflows}
    providers = {key: row for key, row in catalog.items() if key.startswith("provider.")}
    fragments = {
        name: (SITE / "pages" / f"{name}.html").read_text(encoding="utf-8")
        for name, _, _ in PAGES
    }
    substitutions = {
        "repository_url": html.escape(REPOSITORY, quote=True),
        "bridge_diagram": (SITE / "pages/bridge-diagram.html").read_text(encoding="utf-8"),
        "backseat_diagram": (SITE / "pages/backseat-diagram.html").read_text(encoding="utf-8"),
        "provider_index": provider_index(providers),
        "setup_guide": source_link("docs/development/from-scratch.md", "See the setup guide"),
        "copter_starter": source_link("docs/development/from-scratch.md#copter-starter", "Copter starter"),
        "external_plugins": source_link("docs/development/external-plugins.md", "Develop an external plugin"),
        "mavros_crosswalk": source_link("docs/mavros-crosswalk.md", "MAVROS crosswalk"),
        "software": str(receipt["software_executed"]),
        "native": str(receipt["native_sitl_scenarios"]),
        "stress": str(stress_cases),
        "control_matrix_guide": source_link(CONTROL_MATRIX, "Command coverage matrix"),
    }
    for name, template in fragments.items():
        fragments[name] = re.sub(
            r"\[\[([a-z_]+)\]\]", lambda match: substitutions[match[1]], template
        )
    return fragments


def build(output):
    if output.is_symlink() or (output.exists() and (not output.is_dir() or any(output.iterdir()))):
        raise ValueError("Choose a fresh or empty output directory")
    fragments = load_fragments(read_workflows())
    pages = {
        f"{name}.html": render_page(name, title, fragments[name]) for name, _, title in PAGES
    }
    assets = output / "assets"
    assets.mkdir(parents=True)
    for name in ASSETS:
        shutil.copyfile(SITE / "assets" / name, assets / name)
    for name, content in pages.items():
        (output / name).write_text(content, encoding="utf-8")
    (output / ".nojekyll").touch()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "build-site")
    args = parser.parse_args()
    try:
        build(args.output)
    except (KeyError, OSError, ValueError, subprocess.CalledProcessError) as error:
        parser.exit(1, f"Site build failed: {error}\n")
    print(f"Documentation built at {args.output.resolve() / 'index.html'}")


if __name__ == "__main__":
    main()
