# Sphinx Documentation for aiofranka

## Overview

Complete Sphinx documentation has been set up for the aiofranka library with the following structure:

```
docs/
├── conf.py              # Sphinx configuration
├── Makefile             # Build automation (Linux/Mac)
├── make.bat             # Build automation (Windows)
├── README.md            # Documentation build guide
├── index.rst            # Main landing page
├── installation.rst     # Installation instructions
├── quickstart.rst       # Quick start tutorial
├── api.rst              # API reference
├── examples.rst         # Usage examples
├── _static/             # Static assets
└── _templates/          # Custom templates
```

## Building the Documentation

### 1. Install Dependencies

```bash
# Using pyproject.toml optional dependencies
pip install -e ".[docs]"

# Or install directly
pip install sphinx sphinx-rtd-theme
```

### 2. Build HTML Documentation

```bash
cd docs
make html
```

The generated HTML will be in `docs/_build/html/`. Open `docs/_build/html/index.html` to view.

### 3. Other Build Options

```bash
make clean      # Remove build artifacts
make latexpdf   # Generate PDF (requires LaTeX)
make epub       # Generate EPUB
make help       # Show all available commands
```

## Features

### Theme
- **sphinx-rtd-theme**: Modern, responsive Read the Docs theme

### Extensions Enabled
- **sphinx.ext.autodoc**: Auto-generate docs from docstrings
- **sphinx.ext.napoleon**: Support for Google/NumPy style docstrings
- **sphinx.ext.viewcode**: Add source code links
- **sphinx.ext.intersphinx**: Link to other projects (Python, NumPy, SciPy)
- **sphinx.ext.autosummary**: Generate summary tables

### Documentation Pages

1. **index.rst**: Main landing page with project overview
2. **installation.rst**: Detailed installation instructions
3. **quickstart.rst**: Step-by-step quick start guide
4. **api.rst**: Complete API reference (auto-generated from code)
5. **examples.rst**: Comprehensive usage examples including:
   - Basic control loop
   - Position control
   - Impedance control
   - Operational space control
   - Gain scheduling
   - Mode switching
   - Simulation mode

## Development Workflow

### Live Preview

For real-time documentation preview during development:

```bash
pip install sphinx-autobuild
cd docs
sphinx-autobuild . _build/html
```

Then open http://127.0.0.1:8000 in your browser.

### Adding New Content

1. **Add new modules**: Update `api.rst` with new automodule directives
2. **Add examples**: Update `examples.rst` or create new .rst files
3. **Update docstrings**: Use Google or NumPy style in your Python code
4. **Build and verify**: Run `make html` to check for errors

### Docstring Style

The configuration supports both Google and NumPy docstring styles:

```python
def example_function(param1, param2):
    """
    Short description.
    
    Args:
        param1: Description of param1
        param2: Description of param2
    
    Returns:
        Description of return value
    """
    pass
```

## Publishing Options

### GitHub Pages

1. Build the documentation:
   ```bash
   cd docs
   make html
   ```

2. Copy `_build/html/` contents to a `gh-pages` branch

3. Enable GitHub Pages in repository settings

### Read the Docs

1. Import your repository on https://readthedocs.org
2. RTD will automatically build using the configuration in `docs/conf.py`

### Custom Hosting

Simply upload the contents of `docs/_build/html/` to any web server.

## Configuration

The main configuration is in `docs/conf.py`:

- **Project info**: Update version, author, copyright
- **Theme settings**: Customize appearance
- **Extensions**: Add/remove Sphinx extensions
- **Napoleon settings**: Configure docstring parsing
- **Intersphinx**: Add links to other documentation

## .gitignore

The following patterns have been added to `.gitignore`:

```
docs/_build/
docs/_static/
docs/_templates/
```

## Dependencies Added

Added to `pyproject.toml`:

```toml
[project.optional-dependencies]
docs = [
    "sphinx>=5.0",
    "sphinx-rtd-theme>=1.0",
]
```

## Tips

1. **Incremental builds**: Use `make html` for faster rebuilds
2. **Clean builds**: Use `make clean html` to rebuild everything
3. **Check warnings**: Fix any warnings in the build output
4. **Test locally**: Always build and view locally before publishing
5. **Keep docstrings updated**: Good code documentation = good generated docs

## Next Steps

1. Review the generated documentation in your browser
2. Add more detailed docstrings to your code
3. Customize the theme if needed
4. Set up automated documentation builds (CI/CD)
5. Deploy to Read the Docs or GitHub Pages
