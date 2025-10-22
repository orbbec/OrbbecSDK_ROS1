# 文档应用文档（中文版）

## 编译环境

```bash
cd docs/zh
pip install -r requirements.txt
```

## 编译

```bash
make clean
make html
```

## 编译结果

用浏览器打开 `_build/html/index.html` 查看编译结果

## 注意事项

- 本文档为中文版本，语言设置为 `zh_CN`
- 确保系统已安装中文字体支持
- 编译时需要确保所有中文内容正确编码为UTF-8

## 文档结构

- `source/` - 文档源文件目录
- `_static/` - 静态资源文件（CSS、JS、图片等）
- `_templates/` - 模板文件
- `conf.py` - Sphinx配置文件（中文版）
