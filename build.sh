#!/bin/bash

# build.sh - Script to build Sphinx documentation for aiofranka
# This script builds the HTML documentation from the source files in docs/source
# and outputs them to docs/ directory

set -e  # Exit on error

echo "Building aiofranka documentation..."

# Clean previous build
echo "Cleaning previous build..."
rm -rf docs/_build docs/doctrees docs/*.html docs/*.js docs/_static docs/_sources docs/api

# Build HTML documentation
echo "Building HTML documentation..."
sphinx-build -b html docs/source docs

echo "Documentation built successfully!"
echo "Open docs/index.html in your browser to view the documentation."
