from __future__ import annotations

import asyncio
from typing import TYPE_CHECKING

import rosys
from nicegui import app, ui

from .confirm_dialog import ConfirmDialog

if TYPE_CHECKING:
    from ...hardware import TeltonikaRouter, WifiClientNetwork

ENCRYPTION_OPTIONS = ['psk2', 'sae', 'none']
EXPANSION_STORAGE_PREFIX = 'teltonika_expansion_'


def _format_value(value: str | int | None, unit: str = '') -> str:
    """Format a value with optional unit for display, returning '-' for None."""
    return f'{value} {unit}'.strip() if value is not None else '-'


def _device_section(router: TeltonikaRouter) -> None:
    device = router.device_info
    with ui.grid(columns=2).classes('w-full gap-x-4 gap-y-1'):
        ui.label('Model:').tooltip('Router model name')
        ui.label(_format_value(device.model) if device else '-')
        ui.label('Connection:').tooltip('Active failover interface type')
        ui.label(router.connection_status.value.upper())
        ui.label('Firmware:').tooltip('RutOS firmware version')
        ui.label(_format_value(device.firmware_version) if device else '-')
        ui.label('Serial:').tooltip('Router serial number')
        ui.label(_format_value(device.serial) if device else '-')


def _mobile_section(router: TeltonikaRouter) -> None:
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
    wifi = router.wifi_info
    with ui.grid(columns=2).classes('w-full gap-x-4 gap-y-1'):
        ui.label('SSID:').tooltip('Broadcast WiFi network name')
        ui.label(_format_value(wifi.ap_ssid) if wifi else '-')
        ui.label('Clients:').tooltip('Number of connected WiFi clients')
        ui.label(_format_value(wifi.ap_clients) if wifi else '-')


def _multi_ap_section(router: TeltonikaRouter) -> None:
    wifi = router.wifi_info
    with ui.grid(columns=2).classes('w-full gap-x-4 gap-y-1'):
        ui.label('SSID:').tooltip('Upstream WiFi network the router connects to')
        ui.label(_format_value(wifi.sta_ssid) if wifi else '-')
        ui.label('Signal:').tooltip('Upstream WiFi signal strength (-30 great, -67 ok, -80 weak, -90 unusable)')
        ui.label(_format_value(wifi.sta_signal, 'dBm') if wifi else '-')


def teltonika_ui(router: TeltonikaRouter) -> None:
    async def toggle_network(network: WifiClientNetwork, target: bool) -> None:
        if target == network.enabled:
            return
        action = 'Enable' if target else 'Disable'
        if not await ConfirmDialog(f'{action} WiFi network "{network.ssid}"?'):
            _networks.refresh()  # revert the switch to the unchanged state
            return
        if await router.set_wifi_client_enabled(network.id, target):
            rosys.notify(f'{action}d WiFi network "{network.ssid}"', 'positive')
        else:
            rosys.notify(f'Failed to {action.lower()} WiFi network', 'negative')

    async def delete_network(network: WifiClientNetwork) -> None:
        if not await ConfirmDialog(f'Delete WiFi network "{network.ssid}"?'):
            return
        if await router.remove_wifi_client_network(network.id):
            rosys.notify(f'Deleted WiFi network "{network.ssid}"', 'positive')
        else:
            rosys.notify('Failed to delete WiFi network', 'negative')

    async def add_network() -> None:
        with ui.dialog() as dialog, ui.card():
            ui.label('Add WiFi network').classes('text-bold')
            ssid_input = ui.input('SSID').classes('w-full')
            password_input = ui.input('Password', password=True, password_toggle_button=True).classes('w-full')
            encryption_select = ui.select(ENCRYPTION_OPTIONS, value='psk2', label='Encryption').classes('w-full')
            with ui.row():
                ui.button('Add', on_click=lambda: dialog.submit(True))
                ui.button('Cancel', on_click=lambda: dialog.submit(False)).props('flat')
        if not await dialog:
            return
        ssid = ssid_input.value.strip()
        if not ssid:
            rosys.notify('SSID must not be empty', 'warning')
            return
        if await router.add_wifi_client_network(ssid, password_input.value, encryption=encryption_select.value):
            rosys.notify(f'Added WiFi network "{ssid}"', 'positive')
        else:
            rosys.notify('Failed to add WiFi network', 'negative')

    async def refresh_networks() -> None:
        await router.refresh_wifi_client_networks()
        rosys.notify(f'Found {len(router.wifi_client_networks)} upstream WiFi network(s)', 'info')

    @ui.refreshable
    def _networks() -> None:
        with ui.row().classes('w-full items-center justify-between no-wrap'):
            ui.label('Upstream WiFi networks').classes('text-sm text-grey')
            ui.button(icon='refresh', on_click=refresh_networks) \
                .props('flat dense round').tooltip('Reload the network list from the router')
        if not router.wifi_client_networks:
            ui.label('No upstream WiFi networks configured.').classes('text-sm text-grey')
        for network in router.wifi_client_networks:
            with ui.row().classes('w-full items-center justify-between no-wrap'):
                with ui.column().classes('gap-0 min-w-0'):
                    ui.label(network.ssid or '(hidden)').classes('truncate')
                    if network.encryption:
                        ui.label(network.encryption).classes('text-xs text-grey')
                with ui.row().classes('items-center gap-1 no-wrap'):
                    ui.switch(value=network.enabled,
                              on_change=lambda e, n=network: toggle_network(n, e.value)) \
                        .tooltip('Enable or disable this network')
                    ui.button(icon='delete', on_click=lambda _, n=network: delete_network(n)) \
                        .props('flat dense round color=negative').tooltip('Delete this network')
        ui.button('Add network', icon='add', on_click=add_network).props('flat dense')

    @ui.refreshable
    def _device() -> None:
        _device_section(router)

    @ui.refreshable
    def _mobile() -> None:
        _mobile_section(router)

    @ui.refreshable
    def _ap() -> None:
        _ap_section(router)

    @ui.refreshable
    def _multi_ap() -> None:
        _multi_ap_section(router)

    def _refresh_status() -> None:
        _device.refresh()
        _mobile.refresh()
        _ap.refresh()
        _multi_ap.refresh()

    async def handle_reboot() -> None:
        if not await ConfirmDialog('Really reboot the router?'):
            return
        if await router.reboot():
            ui.notify('Router reboot initiated', type='positive')
        else:
            ui.notify('Router reboot failed', type='negative')

    async def handle_ping() -> None:
        connectivity, dns = await asyncio.gather(router.check_internet(), router.check_dns())
        if connectivity and dns:
            rosys.notify('Internet reachable', 'positive')
        elif connectivity:
            rosys.notify('Internet reachable, but DNS not resolving', 'warning')
        elif dns:
            rosys.notify('DNS resolves, but no IP connectivity', 'warning')
        else:
            rosys.notify('No internet connection', 'negative')

    expansions: dict[str, ui.expansion] = {}
    with ui.column().classes('w-full gap-1'):
        with ui.expansion('Router', icon='router', value=True).classes('w-full').props('dense') as expansions['router']:
            _device()
        with ui.expansion('Mobile', icon='lte_mobiledata').classes('w-full').props('dense') as expansions['mobile']:
            _mobile()
        with ui.expansion('Access Point', icon='wifi_tethering') \
                .classes('w-full').props('dense') as expansions['access_point']:
            _ap()
        with ui.expansion('Upstream WiFi', icon='wifi').classes('w-full').props('dense') as expansions['upstream_wifi']:
            _multi_ap()
        with ui.expansion('WiFi Networks', icon='settings_ethernet') \
                .classes('w-full').props('dense') as expansions['wifi_networks']:
            _networks()
        with ui.row().classes('mt-2'):
            ui.button('Check Internet', icon='network_ping', on_click=handle_ping).props('outline')
            ui.button('Reboot Router', icon='restart_alt', on_click=handle_reboot, color='negative').props('outline')

    bound = False

    def _persist_expansions(_=None) -> None:
        nonlocal bound
        if bound:  # on_connect fires again on reconnect; bind only once
            return
        bound = True
        for name, expansion in expansions.items():
            key = f'{EXPANSION_STORAGE_PREFIX}{name}'
            if key not in app.storage.tab:
                app.storage.tab[key] = expansion.value  # seed the constructed default before binding
            expansion.bind_value(app.storage.tab, key)
    ui.context.client.on_connect(_persist_expansions)  # app.storage.tab needs an established connection

    router.CONNECTION_CHANGED.subscribe(_refresh_status, unsubscribe_on_delete=True)
    router.INFO_UPDATED.subscribe(_refresh_status, unsubscribe_on_delete=True)
    router.WIFI_NETWORKS_CHANGED.subscribe(_networks.refresh, unsubscribe_on_delete=True)
