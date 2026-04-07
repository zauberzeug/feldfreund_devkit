from __future__ import annotations

from typing import TYPE_CHECKING

from nicegui import ui

from ...hardware import ConnectionStatus

if TYPE_CHECKING:
    from ...hardware import TeltonikaRouter


def teltonika_status_widget(router: TeltonikaRouter) -> None:
    @ui.refreshable
    def _ui() -> None:
        connection = router.connection_status
        if connection == ConnectionStatus.WIFI and router.wifi_info and router.wifi_info.sta_signal is not None:
            signal = router.wifi_info.sta_signal
            if signal >= router.WIFI_SIGNAL_GOOD:
                icon_name = 'wifi'
            elif signal >= router.WIFI_SIGNAL_FAIR:
                icon_name = 'wifi_2_bar'
            else:
                icon_name = 'wifi_1_bar'
        else:
            icon_name = {'ether': 'cable', 'mobile': 'lte_mobiledata'}.get(connection.value, 'mobiledata_off')
        size = 'lg' if connection == ConnectionStatus.MOBILE else 'sm'
        # NiceGUI tooltips don't support newlines — space-separated is the best we can do
        parts = [f'Connection: {connection.value}']
        if router.modem_status and connection == ConnectionStatus.MOBILE:
            if router.modem_status.operator:
                parts.append(f'Operator: {router.modem_status.operator}')
            if router.modem_status.network_type:
                parts.append(f'Network: {router.modem_status.network_type}')
            if router.modem_status.rssi is not None:
                parts.append(f'RSSI: {router.modem_status.rssi} dBm')
            if router.modem_status.rsrp is not None:
                parts.append(f'RSRP: {router.modem_status.rsrp} dBm')
        if router.wifi_info and connection == ConnectionStatus.WIFI:
            if router.wifi_info.sta_ssid:
                parts.append(f'WiFi: {router.wifi_info.sta_ssid}')
            if router.wifi_info.sta_signal is not None:
                parts.append(f'Signal: {router.wifi_info.sta_signal} dBm')
        with ui.icon(icon_name, size=size):
            ui.tooltip(' '.join(parts))
    _ui()
    router.CONNECTION_CHANGED.subscribe(_ui.refresh, unsubscribe_on_delete=True)
    router.INFO_UPDATED.subscribe(_ui.refresh, unsubscribe_on_delete=True)
