# Documentation Application (English Version)

## Build Environment

```bash
cd docs/en
pip install -r requirements.txt
```

## Build

```bash
make clean
make html
```

## Build Results

Open `_build/html/index.html` in a browser to view the build results

## Notes

- This is the English version of the documentation
- Language setting is `en`
- Ensure all dependencies are properly installed

## Documentation Structure

- `source/` - Documentation source files directory
- `_static/` - Static resource files (CSS, JS, images, etc.)
- `_templates/` - Template files
- `conf.py` - Sphinx configuration file (English version)

## Requirements

The required packages are listed in `requirements.txt`. Make sure to install them before building the documentation.
