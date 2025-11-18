# Building the Documentation

This directory contains the Sphinx documentation for aiofranka.

## Installation

First, install the documentation dependencies:

```bash
pip install -e ".[docs]"
```

Or install Sphinx directly:

```bash
pip install sphinx sphinx-rtd-theme
```

## Building HTML Documentation

To build the HTML documentation:

```bash
cd docs
make html
```

The generated documentation will be in `_build/html/`. Open `_build/html/index.html` in a browser to view it.

## Building Other Formats

Sphinx supports various output formats:

```bash
make html      # HTML output
make latexpdf  # PDF output (requires LaTeX)
make epub      # EPUB output
make man       # Manual pages
```

## Cleaning Build Files

To remove all built documentation:

```bash
make clean
```

## Live Preview

For live preview during development, you can use `sphinx-autobuild`:

```bash
pip install sphinx-autobuild
sphinx-autobuild . _build/html
```

Then open http://127.0.0.1:8000 in your browser.

## Documentation Structure

- `index.rst` - Main landing page
- `installation.rst` - Installation instructions
- `quickstart.rst` - Quick start guide
- `api.rst` - API reference (auto-generated from docstrings)
- `examples.rst` - Usage examples
- `conf.py` - Sphinx configuration
- `_static/` - Static files (CSS, images, etc.)
- `_templates/` - Custom templates

## Contributing

When adding new modules or features, make sure to:

1. Add proper docstrings to all classes and functions
2. Update the relevant `.rst` files
3. Add examples if appropriate
4. Build the docs locally to check for errors
