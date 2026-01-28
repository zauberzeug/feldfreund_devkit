from nicegui import ui
from rosys.hardware import Bms, EStop

from ...hardware import TeltonikaRouter


class HeaderBar:
    """Navigation header with logo, page links, and status indicators."""

    def __init__(self, pages: dict[str, str] | None = None, *,
                 estop: EStop | None = None,
                 bms: Bms | None = None,
                 bms_url: str | None = None,
                 teltonika_router: TeltonikaRouter | None = None):
        self._pages = pages or {}
        self.estop = estop
        self.bms = bms
        self.bms_url = bms_url
        self.teltonika_router = teltonika_router

    def content(self) -> None:
        ui.colors(primary='#6E93D6', secondary='#53B689', accent='#111B1E', positive='#53B689')
        with ui.header().classes('items-center py-3'):
            with ui.link(target='/'):
                ui.image('assets/zz_logo.png').classes('w-12')
            ui.link('FELDFREUND', '/').classes('text-2xl text-white !no-underline mr-auto')
            with ui.row().classes('items-right pr-4'):
                if self.estop:
                    self.estop_status(self.estop)
                with ui.row():
                    for title, url in self._pages.items():
                        ui.link(title, url).classes('text-white text-lg !no-underline')
                if self.teltonika_router:
                    self.teltonika_router.status()
                if self.bms:
                    self.battery_status(self.bms, page=self.bms_url)

    def battery_status(self, bms: Bms, *, page: str | None = None) -> ui.element:
        wrapper = ui.link(target=page).classes('!no-underline text-white') if page else ui.row()
        with wrapper:
            with ui.row().classes('items-center gap-1'):
                ui.icon('battery_charging_full', size='sm').bind_visibility_from(bms.state, 'is_charging')

                def get_battery_icon(p: float | None) -> str:
                    battery_icons = {
                        5: 'battery_0_bar',
                        20: 'battery_1_bar',
                        35: 'battery_2_bar',
                        50: 'battery_3_bar',
                        65: 'battery_4_bar',
                        80: 'battery_5_bar',
                        95: 'battery_6_bar',
                    }
                    if p is None:
                        return 'battery_unknown'
                    for threshold, icon in battery_icons.items():
                        if p < threshold:
                            return icon
                    return 'battery_full'
                ui.icon('', size='sm').bind_name_from(bms.state, 'percentage', get_battery_icon) \
                    .bind_visibility_from(bms.state, 'is_charging', lambda c: not c)
                ui.label().bind_text_from(bms.state, 'percentage', lambda p: f'{p:.0f}%' if p is not None else '?')
        return wrapper

    def estop_status(self, estop: EStop) -> ui.element:
        with ui.column() as column:
            with ui.row().classes('mr-auto bg-red-500 text-white p-2 rounded-md') \
                    .bind_visibility_from(estop, 'active', backward=lambda active: active and not estop.is_soft_estop_active):
                ui.icon('report').props('size=md').classes('text-white').props('elevated')
                ui.label().bind_text_from(estop, 'active_estops',
                                          lambda e: f'Emergency stop {", ".join(e)} is pressed!').classes('text-white text-xl').props('elevated')
            with ui.row().bind_visibility_from(estop, 'is_soft_estop_active').classes('mr-auto bg-red-500 rounded-md p-1'):
                ui.icon('report').props('size=md').classes('text-white').props('elevated')
                ui.label('Software ESTOP is active!').classes('text-white text-3xl').props('elevated')
        return column
