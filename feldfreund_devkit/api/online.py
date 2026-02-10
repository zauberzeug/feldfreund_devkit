from nicegui import app


class Online:
    """API endpoints for checking the robot's online status.

    - `GET /api/online` → `{'online': True | False}`
    """

    def __init__(self) -> None:
        app.get('/api/online')(self.online)

    def online(self) -> dict[str, bool]:
        return {'online': True}
