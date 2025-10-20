# OrbbecSDK V2 ROS1 Wrapper Documentation

This repository contains the documentation for OrbbecSDK V2 ROS1 Wrapper in both English and Chinese.

## Documentation Versions

- **English Version**: `docs/en/` - Full English documentation
- **中文版本**: `docs/zh/` - 完整中文文档

## Quick Start

### For English Documentation:
```bash
cd docs/en
pip install -r requirements.txt
make clean && make html
```

### For Chinese Documentation:
```bash
cd docs/zh
pip install -r requirements.txt
make clean && make html
```

## View Documentation

After building, open the following files in your browser:
- English: `docs/en/_build/html/index.html`
- Chinese: `docs/zh/_build/html/index.html`

## Project Structure

```
docs/
├── en/                 # English documentation
│   ├── source/         # Source files
│   ├── conf.py         # Sphinx configuration
│   ├── requirements.txt
│   └── README.md
└── zh/                 # Chinese documentation
    ├── source/         # Source files
    ├── conf.py         # Sphinx configuration
    ├── requirements.txt
    └── README.md
```

## Documentation Content

The documentation covers:

1. **Overview** - Introduction and device compatibility
2. **Installation** - Build and installation instructions
3. **Quick Starts** - Getting started guides
4. **Application Guide** - Detailed usage instructions
5. **Advanced Guide** - Advanced configurations and features
6. **Benchmark** - Performance benchmarks
7. **Developer Guide** - Development guidelines
8. **FAQ** - Frequently asked questions

## Contributing

When updating documentation:
1. Make changes to the English version first (`docs/en/`)
2. Translate and adapt for the Chinese version (`docs/zh/`)
3. Ensure both versions build successfully
4. Test all links and references

## Requirements

- Python 3.6+
- Sphinx
- Required extensions (see requirements.txt in each language folder)

## Support

For questions about the documentation or the OrbbecSDK ROS1 Wrapper, please visit:
- GitHub: https://github.com/orbbec/OrbbecSDK_ROS1
- Gitee (中国用户): https://gitee.com/orbbecdeveloper/OrbbecSDK_ROS1
