<img src="https://github.com/zauberzeug/feldfreund_devkit/raw/main/assets/feldfreund.webp"  alt="Feldfreund rendering" width="40%" align="right" />

# Feldfreund DevKit

This is the source code of the [Feldfreund Dev Kit](https://zauberzeug.com/products/field-friend-dev-kit) for autonomous outdoor robotics made by [Zauberzeug](https://zauberzeug.com/).
The software is based on [RoSys](https://rosys.io) and [NiceGUI](https://nicegui.io/).
The micro controller is programmed with [Lizard](https://github.com/zauberzeug/lizard).

Our agricultural weeding robot [Feldfreund](https://zauberzeug.com/feldfreund) is based on this platform and is intended to advance organic and regenerative agriculture.
There is also a [ROS2 implementation](https://github.com/zauberzeug/feldfreund_devkit_ros) based on this repository.

Please see the [documentation](https://docs.feldfreund.de) for details on installation, setup and usage.

## Development

1. create a virtual environment and activate it:

```bash
virtualenv .venv # or without virtualenv:
python -m venv .venv

source .venv/bin/activate # to activate your virtual environment
```

2. install dependencies:

```bash
poetry install --with dev,test
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
