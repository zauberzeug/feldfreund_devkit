import asyncio
from collections.abc import Awaitable, Callable
from contextlib import AbstractAsyncContextManager
from dataclasses import dataclass
from typing import Protocol

from rosys.driving.pose_provider import PoseProvider

from .path_driver import PathDriver


class Detection(Protocol):
    """Controls when the robot is looking for what it works on."""

    def running(self) -> AbstractAsyncContextManager[None]:
        """Keep detection running for the duration of the scope, and stop it on the way out."""


@dataclass(frozen=True)
class WorkContext:
    """What a tool may use while it works: how to move, where the robot is, and what it can see.

    Run-scoped, so it carries only what a tool cannot own itself. Hardware and the like stay wired
    into the implement at construction.
    """

    motion: PathDriver
    pose: PoseProvider
    detection: Detection


WorkFunction = Callable[[WorkContext], Awaitable[None]]
"""A tool's work loop. Runs while the robot drives a working stretch and is cancelled at its end,
so it must not return on its own -- see :func:`drive_and_work`."""


async def never() -> None:
    """Idle until cancelled.

    For a tool with nothing to do but stay alive for the stretch -- a mower, whose actuation runs
    from activation to deactivation, or a tool that only needs to act when the stretch ends.
    """
    await asyncio.Event().wait()


async def no_work(ctx: WorkContext) -> None:
    """The work loop for a run that only drives.

    Distinct from :func:`never`, which takes no context: a :data:`WorkFunction` is always called
    with one, and a run over workable segments would otherwise fail on the first of them.
    """
    await never()
