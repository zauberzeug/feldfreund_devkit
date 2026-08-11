from abc import ABC, abstractmethod


class SettingsUI(ABC):
    """Something with controls the operator sets before a run.

    Settings worth showing are worth keeping, so an implementor is usually ``Persistable`` too and
    saves on change. Objects planned for a single run have nothing to set and stay off this.
    """

    @abstractmethod
    def settings_ui(self) -> None:
        """Draw the controls into the surrounding container."""
