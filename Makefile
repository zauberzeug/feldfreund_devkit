.PHONY: help sync install-ci mypy pylint pre-commit docs-serve docs-deploy

default: help

## Available commands for this Makefile, use 'make <command>' to execute:

##
## ---------------

## help		Print commands help.
help: Makefile
	@sed -n 's/^##//p' $<

## sync		Install all dependencies for development and testing.
sync:
	uv sync

## install-ci	Install all dependencies for CI testing.
install-ci:
	uv sync

## mypy		Run mypy type checks.
mypy:
	uv run --active mypy ./feldfreund_devkit ./odrive ./config

## pylint		Run pylint code analysis.
pylint:
	uv run --active pylint ./feldfreund_devkit ./config

## ruff		Run ruff code analysis.
ruff:
	uv run --active ruff check ./feldfreund_devkit ./odrive ./config
## pre-commit	Run pre-commit hooks on all files.
pre-commit:
	uv run --active pre-commit run --all-files

## check		Run all code checks (mypy, pre-commit, pylint).
check: mypy pre-commit pylint

## docs-serve	Serve documentation locally.
docs-serve:
	uv run --active mkdocs serve

## docs-deploy	Deploy documentation to GitHub Pages.
docs-deploy:
	uv run --active mkdocs gh-deploy --force && rm -rf site
