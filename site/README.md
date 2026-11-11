# MAVMOOS documentation site

From the repository root, build the site into a fresh directory and serve the result locally. Python 3 is the only site build dependency.

```sh
python3 -B site/build.py --output build-site
python3 -m http.server 8080 --bind 127.0.0.1 --directory build-site
```

Open [the local preview](http://127.0.0.1:8080/). You can also open `build-site/index.html` directly. The local server enables command copying in browsers that require a secure context for clipboard access.

The home page renders the plugin list from `scripts/reproduce.py list --json` and the [plugin catalog](../docs/plugin-catalog.md). Maintenance shows the current release matrix and the latest archived release verdict. The output contains the generated pages, assets and `.nojekyll`.

## GitHub Pages

The [Publish documentation workflow](../.github/workflows/pages.yml) builds and deploys the site from `main` when its templates, assets, or data inputs change. It uploads only the generated site and retains the deployment artifact for one day. You can also run the workflow manually from `main`. Update the workflow’s path filters when adding a new build input.

The project site uses relative links and works under the repository path. Linked source guides are served from GitHub and reflect changes on `main` as soon as those changes are pushed.
