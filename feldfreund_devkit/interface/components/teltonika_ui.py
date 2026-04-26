from __future__ import annotations

from typing import TYPE_CHECKING

import rosys
from nicegui import ui

from .confirm_dialog import ConfirmDialog

if TYPE_CHECKING:
    from ...hardware import TeltonikaRouter


def _format_value(value: str | int | None, unit: str = '') -> str:
    """Format a value with optional unit for display, returning '-' for None."""
    return f'{value} {unit}'.strip() if value is not None else '-'


def _device_section(router: TeltonikaRouter) -> None:
    device = router.device_info
    title = device.model if device and device.model else 'Teltonika Router'
    ui.label(title).classes('text-bold')
    with ui.grid(columns=2).classes('w-full gap-x-4 gap-y-1'):
        ui.label('Connection:').tooltip('Active failover interface type')
        ui.label(router.connection_status.value.upper())
        ui.label('Firmware:').tooltip('RutOS firmware version')
        ui.label(_format_value(device.firmware_version) if device else '-')
        ui.label('Serial:').tooltip('Router serial number')
        ui.label(_format_value(device.serial) if device else '-')


def _mobile_section(router: TeltonikaRouter) -> None:
    ui.label('Mobile').classes('font-bold')
    modem = router.modem_status
    with ui.grid(columns=2).classes('w-full gap-x-4 gap-y-1'):
        ui.label('Operator:').tooltip('Mobile network operator')
        ui.label(_format_value(modem.operator) if modem else '-')
        ui.label('Network:').tooltip('Connection type (LTE, 3G, No service)')
        ui.label(_format_value(modem.network_type) if modem else '-')
        ui.label('RSSI:').tooltip('Total received power incl. noise (-50 great, -90 weak, -110 dead)')
        ui.label(_format_value(modem.rssi, 'dBm') if modem else '-')
        ui.label('RSRP:').tooltip('Reference signal power (-80 great, -100 weak, -120 dead)')
        ui.label(_format_value(modem.rsrp, 'dBm') if modem else '-')
        ui.label('SINR:').tooltip('Signal-to-noise ratio (>20 great, >0 usable, <0 unusable)')
        ui.label(_format_value(modem.sinr, 'dB') if modem else '-')
        ui.label('RSRQ:').tooltip('Signal quality factoring cell load (-5 great, -10 ok, -15 poor)')
        ui.label(_format_value(modem.rsrq, 'dB') if modem else '-')
        ui.label('Temperature:').tooltip('Modem module temperature')
        ui.label(_format_value(modem.temperature, '°C') if modem else '-')


def _ap_section(router: TeltonikaRouter) -> None:
    ui.label('AP').classes('font-bold')
    wifi = router.wifi_info
    with ui.grid(columns=2).classes('w-full gap-x-4 gap-y-1'):
        ui.label('SSID:').tooltip('Broadcast WiFi network name')
        ui.label(_format_value(wifi.ap_ssid) if wifi else '-')
        ui.label('Clients:').tooltip('Number of connected WiFi clients')
        ui.label(_format_value(wifi.ap_clients) if wifi else '-')


def _multi_ap_section(router: TeltonikaRouter) -> None:
    ui.label('Multi AP').classes('font-bold')
    wifi = router.wifi_info
    with ui.grid(columns=2).classes('w-full gap-x-4 gap-y-1'):
        ui.label('SSID:').tooltip('Upstream WiFi network the router connects to')
        ui.label(_format_value(wifi.sta_ssid) if wifi else '-')
        ui.label('Signal:').tooltip('Upstream WiFi signal strength (-30 great, -67 ok, -80 weak, -90 unusable)')
        ui.label(_format_value(wifi.sta_signal, 'dBm') if wifi else '-')


def teltonika_ui(router: TeltonikaRouter) -> None:
    @ui.refreshable
    def _ui() -> None:
        _device_section(router)
        ui.separator()
        _mobile_section(router)
        ui.separator()
        _ap_section(router)
        ui.separator()
        _multi_ap_section(router)

        async def handle_reboot() -> None:
            if not await ConfirmDialog('Really reboot the router?'):
                return
            if await router.reboot():
                ui.notify('Router reboot initiated', type='positive')
            else:
                ui.notify('Router reboot failed', type='negative')

        async def handle_ping() -> None:
            if await router.check_internet():
                rosys.notify('Internet reachable', 'positive')
            else:
                rosys.notify('No internet connection', 'negative')

        with ui.row():
            ui.button('Ping 8.8.8.8', icon='network_ping', on_click=handle_ping).props('outline')
            ui.button('Reboot Router', icon='restart_alt', on_click=handle_reboot, color='negative') \
                .props('outline')
    _ui()
    router.CONNECTION_CHANGED.subscribe(_ui.refresh, unsubscribe_on_delete=True)
    router.INFO_UPDATED.subscribe(_ui.refresh, unsubscribe_on_delete=True)
