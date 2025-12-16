.PHONY: help sync install-ci mypy pylint pre-commit

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
	uv run mypy . --non-interactive

## pylint		Run pylint code analysis.
pylint:
	uv run pylint ./feldfreund_devkit

## ruff		Run ruff code analysis.
ruff:
	uv run ruff check ./feldfreund_devkit

## pre-commit	Run pre-commit hooks on all files.
pre-commit:
	uv run pre-commit run --all-files

## check		Run all code checks (mypy, pre-commit, pylint).
check: mypy pre-commit pylint
