from abc import ABC, abstractmethod


class SettingsUI(ABC):
    """Something with controls the operator sets before a run."""

    @abstractmethod
    def settings_ui(self) -> None:
        """Draw the controls into the surrounding container."""
