# Feldfreund DevKit

TODO

## Tech Stack

- Python 3.11+
- [NiceGUI](https://nicegui.io) for web interface
- [Poetry](https://python-poetry.org) for dependency management
- [RoSys](https://rosys.io) framework
- [Copier](https://copier.readthedocs.io/) for template configuration (from [nicegui-template](https://github.com/zauberzeug/nicegui-template))

## Development

1. create a virtual environment and activate it:

```bash
virtualenv .venv # or without virtualenv:
python -m venv .venv

source .venv/bin/activate # to activate your virtual environment
```

2. install dependencies:

```bash
poetry install --all-extras --no-root
```

3. start your project:

```bash
poetry run ./main.py
```

4. run your tests:

```bash
poetry run pytest
```

### Updating template settings

To update your project configuration from the nicegui-template, run:

```bash
copier update --skip-answered
```

This will prompt you to review and update your template settings interactively.

## pre-commit

[pre-commit](https://pre-commit.com/) is a tool to help you manage and run pre-commit hooks in your code.
It is used to check your code for e.g. extra whitespace or formatting errors before committing it.
Install the pre-commit hooks by running:

```bash
pre-commit install
```

You can also run the hooks manually by running:

```bash
pre-commit run --all-files
```
