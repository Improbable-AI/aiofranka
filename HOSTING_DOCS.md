# Hosting Documentation on GitHub

Your documentation is currently showing raw RST files because GitHub is just displaying the source files. To properly host the HTML documentation, follow these steps:

## Option 1: GitHub Pages with GitHub Actions (Recommended)

This automatically builds and deploys your docs on every push.

### Setup Steps:

1. **Enable GitHub Pages:**
   - Go to your repository on GitHub
   - Navigate to **Settings** → **Pages**
   - Under "Source", select **GitHub Actions**

2. **Push the workflow file:**
   ```bash
   git add .github/workflows/docs.yml
   git commit -m "Add documentation build workflow"
   git push
   ```

3. **Wait for the build:**
   - Go to the **Actions** tab in your repository
   - Watch the workflow run
   - Once complete, your docs will be at: `https://<username>.github.io/<repo>/`

4. **Your documentation will be available at:**
   ```
   https://improbable-ai.github.io/aiofranka/
   ```

### What the workflow does:
- Automatically runs when you push to `main`
- Installs Sphinx and dependencies
- Builds the HTML documentation
- Deploys to the `gh-pages` branch
- GitHub Pages serves the HTML from there

## Option 2: Manual GitHub Pages Setup

If you prefer manual control:

1. **Build the docs locally:**
   ```bash
   cd docs/source
   make html
   ```

2. **Create/update gh-pages branch:**
   ```bash
   # From repo root
   git checkout --orphan gh-pages
   git rm -rf .
   cp -r docs/source/_build/html/* .
   touch .nojekyll
   git add .
   git commit -m "Deploy documentation"
   git push origin gh-pages
   ```

3. **Enable GitHub Pages:**
   - Go to **Settings** → **Pages**
   - Set source to `gh-pages` branch, `/ (root)` folder
   - Save

4. **Update documentation:**
   ```bash
   git checkout main
   # Make changes...
   cd docs/source
   make html
   git checkout gh-pages
   cp -r docs/source/_build/html/* .
   git add .
   git commit -m "Update documentation"
   git push origin gh-pages
   git checkout main
   ```

## Option 3: Read the Docs (RTD)

For automatic builds with more features:

1. **Go to https://readthedocs.org**
2. **Sign in with GitHub**
3. **Import your repository**
4. **RTD will automatically:**
   - Detect your Sphinx configuration
   - Build on every push
   - Host at `https://aiofranka.readthedocs.io/`

### RTD Configuration (optional):

Create `.readthedocs.yaml` in your repo root:

```yaml
version: 2

build:
  os: ubuntu-22.04
  tools:
    python: "3.10"

sphinx:
  configuration: docs/source/conf.py

python:
  install:
    - method: pip
      path: .
      extra_requirements:
        - docs
```

## Current Documentation Structure

```
docs/
└── source/
    ├── conf.py          # Sphinx config
    ├── index.rst        # Main page
    ├── *.rst            # Other pages
    ├── Makefile         # Build commands
    ├── _build/          # Generated HTML (gitignored)
    ├── _static/         # Static files
    └── _templates/      # Templates
```

## Updating .gitignore

Make sure your `.gitignore` includes:

```
# Sphinx build output
docs/source/_build/
docs/_build/
```

## Quick Commands

```bash
# Build locally
cd docs/source
make html

# View locally
open _build/html/index.html  # macOS
xdg-open _build/html/index.html  # Linux

# Clean build
make clean html
```

## Troubleshooting

**CSS not loading on GitHub Pages?**
- Add `.nojekyll` file (already created)
- Make sure `_static/` is not in `.gitignore`

**404 on GitHub Pages?**
- Check that GitHub Pages is enabled
- Verify the source branch is correct
- Wait a few minutes for deployment

**Import errors during build?**
- The workflow installs required packages
- For local builds: `pip install -e ".[docs]"`

## Recommended: Use GitHub Actions

I've created the workflow file at `.github/workflows/docs.yml`. Just:

1. Commit and push it
2. Enable GitHub Pages (source: GitHub Actions)
3. Your docs will auto-deploy on every push!

Your documentation will be live at:
**https://improbable-ai.github.io/aiofranka/**
