# Quick Fix: Hosting Your Documentation

## The Problem
GitHub is showing your raw `.rst` files instead of the built HTML documentation.

## The Solution (Choose One)

### ⭐ RECOMMENDED: GitHub Actions + GitHub Pages

**Steps:**
1. Go to your repo on GitHub: Settings → Pages
2. Under "Source", select **"GitHub Actions"**
3. Run these commands:
   ```bash
   git add .github/workflows/docs.yml
   git add .gitignore
   git commit -m "Setup documentation deployment"
   git push
   ```
4. Wait 2-3 minutes for the workflow to run
5. Your docs will be live at: **https://improbable-ai.github.io/aiofranka/**

That's it! Every time you push to `main`, your docs will automatically rebuild.

### Alternative: Read the Docs

1. Go to https://readthedocs.org
2. Sign in with GitHub
3. Import your repository
4. Done! Your docs will be at: **https://aiofranka.readthedocs.io/**

## What I've Set Up

✅ GitHub Actions workflow (`.github/workflows/docs.yml`)
✅ Updated `.gitignore` to exclude built HTML files
✅ Added `.nojekyll` file for GitHub Pages
✅ Documentation guide (`HOSTING_DOCS.md`)

## Next Steps

1. **Commit and push** the new files
2. **Enable GitHub Pages** in repository settings
3. Your documentation will be live!

## Why This Works

- GitHub cannot render Sphinx documentation directly
- You need to build the HTML files and host them
- GitHub Pages or Read the Docs will host the built HTML
- The workflow automatically builds on every push

## Need Help?

See `HOSTING_DOCS.md` for detailed instructions and troubleshooting.
