# Quick Reference: Building Documentation

## Quick Start

```bash
# Install dependencies
pip install -e ".[docs]"

# Build HTML docs
cd docs
make html

# View documentation
open _build/html/index.html  # macOS
xdg-open _build/html/index.html  # Linux
start _build/html/index.html  # Windows
```

## Common Commands

```bash
make html       # Build HTML documentation
make clean      # Remove all build files
make help       # Show all available targets
```

## Documentation URLs

- **Local**: `file:///.../docs/_build/html/index.html`
- **GitHub**: Configure GitHub Pages to serve from `gh-pages` branch
- **Read the Docs**: Import repository on https://readthedocs.org

## Structure

- `index.rst` - Main page
- `installation.rst` - Setup guide
- `quickstart.rst` - Tutorial
- `api.rst` - API reference
- `examples.rst` - Code examples
- `conf.py` - Configuration

## Troubleshooting

**Import errors during build?**
- Install numpy and other dependencies: `pip install numpy scipy ruckig`
- Or mock them in `conf.py` (already configured)

**Changes not showing?**
- Run `make clean html` for a fresh build

**Live preview?**
- `pip install sphinx-autobuild`
- `sphinx-autobuild . _build/html`
- Open http://127.0.0.1:8000
