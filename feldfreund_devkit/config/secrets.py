import logging
import os

from dotenv import load_dotenv


class Secrets:
    def __init__(self):
        load_dotenv()
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
