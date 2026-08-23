#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

# Use a placeholder version so setup.py doesn't fail on the {{VERSION_PLACEHOLDER}} token
sed -i "s/{{VERSION_PLACEHOLDER}}/0.0.0+devcontainer/g" setup.py
pip install --upgrade pip
pip install -e .[dev,test]
sed -i "s/0.0.0+devcontainer/{{VERSION_PLACEHOLDER}}/g" setup.py
