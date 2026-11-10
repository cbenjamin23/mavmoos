# MAVMOOS documentation site

From the repository root, build the site into a fresh directory and serve the result locally. Python 3 is the only site build dependency.

```sh
python3 -B site/build.py --output build-site
python3 -m http.server 8080 --bind 127.0.0.1 --directory build-site
```

Open [the local preview](http://127.0.0.1:8080/). You can also open `build-site/index.html` directly. The local server enables command copying in browsers that require a secure context for clipboard access.

The home page renders the plugin list from `scripts/reproduce.py list --json` and the [plugin catalog](../docs/plugin-catalog.md). Maintenance shows the current release matrix and the latest archived release verdict. The output contains the generated pages, assets and `.nojekyll`.

## GitHub Pages

Use the contents of the build directory as the root of a dedicated Pages source branch, or as the artifact for a manually dispatched Pages deployment. The project site uses relative links and works under the repository path. Detailed source guides use GitHub links and follow the source repository’s access settings. See [GitHub’s publishing-source guide](https://docs.github.com/en/pages/getting-started-with-github-pages/configuring-a-publishing-source-for-your-github-pages-site).
