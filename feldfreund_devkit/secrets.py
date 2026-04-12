import os

from dotenv import load_dotenv

load_dotenv()


def _require_secret(key: str) -> str:
    value = os.environ.get(key)
    if not value:
        raise RuntimeError(f'Missing required environment variable: {key}')
    return value


TELTONIKA_PASSWORD = _require_secret('TELTONIKA_PASSWORD')
MJPEG_CAMERA_PASSWORD = _require_secret('MJPEG_CAMERA_PASSWORD')
