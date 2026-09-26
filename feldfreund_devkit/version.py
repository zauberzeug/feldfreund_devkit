from importlib.metadata import PackageNotFoundError, version

try:
    __version__ = version('feldfreund_devkit')
except PackageNotFoundError:
    # the package is run from a checkout that was never installed
    __version__ = 'unknown'
