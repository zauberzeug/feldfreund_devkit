from collections import deque
from datetime import datetime
from typing import Any

import rosys
from nicegui import Event, ui


class LogMonitor(rosys.persistence.Persistable):
    MAX_LINES = 100

    def __init__(self, *, max_lines: int = MAX_LINES) -> None:
        super().__init__()
        self._max_lines = max_lines
        self._lines: deque[str] = deque([], max_lines)

        self.NEW_LINE: Event[str] = Event()
        """a new line was added to the log (argument: line)"""

        rosys.NEW_NOTIFICATION.subscribe(self._handle_notification)

    def _handle_notification(self, message: str) -> None:
        line = f'{datetime.now():%m/%d/%Y %H:%M:%S} {message}'
        self._lines.append(line)
        self.NEW_LINE.emit(line)
        self.request_backup()

    def backup_to_dict(self) -> dict[str, Any]:
        return {
            'logs': list(self._lines),
            'max_lines': self._max_lines,
        }

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        logs = data.get('logs', [])
        self._max_lines = data.get('max_lines', self.MAX_LINES)
        self._lines = deque(logs, self._max_lines)

    def ui(self) -> None:
        ui.label('Log Monitor').classes('text-center text-bold')
        with ui.log(max_lines=self._max_lines).classes('text-xs') as log:
            log.push('\n'.join(self._lines))
            self.NEW_LINE.subscribe(log.push)
            ui.run_javascript(f'getElement({log.id}).scrollTop = getElement({log.id}).scrollHeight')
