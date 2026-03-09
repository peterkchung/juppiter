# Documentation

Welcome to the juppiter documentation site!

## Quick Start

To build and view the documentation locally:

```bash
cd docs
pip install -r requirements.txt
mkdocs serve
```

Then open http://127.0.0.1:8000 in your browser.

## Building

```bash
# Build for production
mkdocs build

# Build with strict mode (catches errors)
mkdocs build --strict

# Deploy to GitHub Pages (maintainers only)
mkdocs gh-deploy
```

## Structure

The documentation is organized as follows:

- **Getting Started** - Installation and first simulation
- **Architecture** - System design and components
- **User Guides** - Configuration and integration
- **Development** - Contributing and testing
- **Reference** - API and configuration reference

## Contributing to Docs

1. Edit the relevant `.md` files
2. Test locally with `mkdocs serve`
3. Submit PR to `develop` branch

## Online Version

The documentation is automatically deployed when changes are pushed to `main` or `develop`:

**Workflow:**
1. You edit documentation on `main` or `develop` branch
2. Push/merge triggers the CI workflow
3. CI builds the MkDocs site
4. Built site is deployed to the `docs` branch

**View deployed docs:**
https://github.com/opencode/juppiter/tree/docs

**The `docs` branch contains only the built HTML site**, not the source Markdown files.

## Questions?

See the main project [README.md](../README.md) or open an issue on GitHub.
