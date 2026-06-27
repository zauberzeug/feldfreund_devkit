import logging
import os
from pathlib import Path

from dotenv import load_dotenv


class Secrets:
    def __init__(self, env_path: str | Path = '.env'):
        """Load the app's ``.env`` into the environment, then read the required secrets from it.

        :param env_path: path to the ``.env`` file, by default ``'.env'`` relative to the working directory.

        An explicit path is used on purpose: a bare ``load_dotenv()`` resolves via ``find_dotenv()`` starting
        from this devkit file, so it never reaches the consuming app's ``.env`` when the devkit is installed
        as a (sibling) dependency. ``override=True`` lets the ``.env`` win over stale/empty pre-existing vars.
        """
        load_dotenv(env_path, override=True)
        self.log = logging.getLogger('devkit.secrets')
        self.TELTONIKA_PASSWORD = self._require_secret('TELTONIKA_PASSWORD')
        self.MJPEG_CAMERA_PASSWORD = self._require_secret('MJPEG_CAMERA_PASSWORD')

    def _require_secret(self, key: str) -> str:
        """Retrieve a secret from the environment variables. Raise if it's not set."""
        value = os.environ.get(key)
        if not value:
            raise RuntimeError(f'Missing required environment variable: {key}')
        return value

    def _get_secret(self, key: str) -> str | None:
        """Retrieve a secret from the environment variables. Log a warning if it's not set."""
        value = os.environ.get(key)
        if not value:
            self.log.warning('Missing environment variable: %s', key)
        return value
