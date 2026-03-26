import asyncio
import logging
from dataclasses import dataclass
from enum import Enum
from typing import Literal

import httpx
import rosys
from nicegui import Event


class ConnectionStatus(Enum):
    """Connection status of the Teltonika router."""
    ETHER = 'ether'
    WIFI = 'wifi'
    MOBILE = 'mobile'
    DISCONNECTED = 'disconnected'


@dataclass(slots=True, kw_only=True)
class ModemStatus:
    """Cellular modem signal quality and connection info."""
    rssi: int | None = None
    rsrp: int | None = None
    sinr: int | None = None
    rsrq: int | None = None
    operator: str | None = None
    network_type: str | None = None
    connection_state: str | None = None
    temperature: int | None = None


@dataclass(slots=True, kw_only=True)
class DeviceInfo:
    """Router device information."""
    firmware_version: str | None = None
    model: str | None = None
    serial: str | None = None


@dataclass(slots=True, kw_only=True)
class WifiInfo:
    """WiFi interface status."""
    ap_ssid: str | None = None
    ap_clients: int | None = None
    sta_ssid: str | None = None
    sta_signal: int | None = None


class TeltonikaRouter:
    """Implements the API of the builtin Teltonika RUT901 router."""
    WIFI_SIGNAL_GOOD = -67
    WIFI_SIGNAL_FAIR = -80
    MAX_CONNECTION_FAILURES = 3
    TOKEN_EXPIRY_SECONDS = 4 * 60
    AUTH_RETRY_INTERVAL = 60
    FAILOVER_KEY_ETHER = 'wan'
    FAILOVER_KEY_WIFI_PREFIXES = ('ifWan', 'wifi')
    FAILOVER_KEYS_MOBILE = ('mob1s1a1', 'mob1s2a1')

    def __init__(self, url: str, admin_password: str) -> None:
        self.log = logging.getLogger('feldfreund.hardware.teltonika_router')
        self._url = url
        self._admin_password = admin_password

        self._connection_status: ConnectionStatus = ConnectionStatus.DISCONNECTED
        self._modem_status: ModemStatus | None = None
        self._device_info: DeviceInfo | None = None
        self._wifi_info: WifiInfo | None = None
        self._client = httpx.AsyncClient(headers={'Content-Type': 'application/json'}, timeout=20.0)
        self._auth_token: str = ''
        self._token_time: float = 0.0
        self._token_lock = asyncio.Lock()
        self._connection_failures: int = 0

        self.CONNECTION_CHANGED: Event[ConnectionStatus] = Event()
        """Emitted when the connection status changes."""
        self.INFO_UPDATED: Event = Event()
        """Emitted after modem, WiFi, and device info have been polled."""

        rosys.on_repeat(self._check_connection, 5.0)
        rosys.on_repeat(self._poll_info, 30.0)
        rosys.on_startup(self._poll_device_info)
        rosys.on_shutdown(self._client.aclose)

    @property
    def connection_status(self) -> ConnectionStatus:
        return self._connection_status

    @property
    def modem_status(self) -> ModemStatus | None:
        return self._modem_status

    @property
    def device_info(self) -> DeviceInfo | None:
        return self._device_info

    @property
    def wifi_info(self) -> WifiInfo | None:
        return self._wifi_info

    async def reboot(self) -> bool:
        """Send a reboot command to the router. Returns True on success."""
        if await self._post('system/actions/reboot'):
            self.log.info('Router reboot initiated')
            return True
        self.log.error('Router reboot failed')
        return False

    async def _poll_info(self) -> None:
        tasks = [self._poll_modem_status(), self._poll_wifi_info()]
        if self._device_info is None:
            tasks.append(self._poll_device_info())
        results = await asyncio.gather(*tasks, return_exceptions=True)
        for result in results:
            if isinstance(result, Exception):
                self.log.warning('Polling task failed: %s', result)
        self.INFO_UPDATED.emit()

    async def _poll_modem_status(self) -> None:
        data = await self._get('modems/status')
        if data is None:
            self._modem_status = None
            return
        self.log.debug('Raw modems/status response: %s', data)
        modem_data = data[0] if isinstance(data, list) and data else data
        if not isinstance(modem_data, dict):
            self._modem_status = None
            return
        self._modem_status = ModemStatus(
            rssi=modem_data.get('rssi'),
            rsrp=modem_data.get('rsrp'),
            sinr=modem_data.get('sinr'),
            rsrq=modem_data.get('rsrq'),
            operator=modem_data.get('operator'),
            network_type=modem_data.get('conntype'),
            connection_state=modem_data.get('state'),
            temperature=modem_data.get('temperature'),
        )

    async def _poll_device_info(self) -> None:
        data = await self._get('system/device/status')
        if data is None or not isinstance(data, dict):
            return
        self.log.debug('Raw system/device/status response: %s', data)
        static = data.get('static', {})
        mnfinfo = data.get('mnfinfo', {})
        self._device_info = DeviceInfo(
            firmware_version=static.get('fw_version'),
            model=static.get('device_name'),
            serial=mnfinfo.get('serial'),
        )

    async def _poll_wifi_info(self) -> None:
        data = await self._get('wireless/interfaces/status')
        if data is None:
            self._wifi_info = None
            return
        self.log.debug('Raw wireless/interfaces/status response: %s', data)
        interfaces = self._normalize_interface_list(data)
        ap = next((i for i in interfaces if isinstance(i, dict) and i.get('mode') == 'ap'), None)
        sta = next((i for i in interfaces if isinstance(i, dict) and i.get('mode') == 'sta'), None)
        if ap is None and sta is None:
            self._wifi_info = None
            return
        ap_clients = ap.get('clients', []) if ap else []
        self._wifi_info = WifiInfo(
            ap_ssid=ap.get('ssid') if ap else None,
            ap_clients=len(ap_clients) if isinstance(ap_clients, list) else ap_clients,
            sta_ssid=sta.get('ssid') if sta else None,
            sta_signal=sta.get('signal') if sta else None,
        )

    async def _ensure_token(self) -> bool:
        """Refresh the auth token if expired. Returns True if a valid token is available."""
        async with self._token_lock:
            if rosys.time() - self._token_time > self.TOKEN_EXPIRY_SECONDS:
                await self._get_token()
        return bool(self._auth_token)

    async def _get(self, endpoint: str) -> dict | list | None:
        """Perform an authenticated GET request. Returns parsed JSON ``data`` or ``None``."""
        response = await self._request('GET', endpoint)
        return response.json().get('data') if response else None

    async def _post(self, endpoint: str, *, json: dict | None = None) -> bool:
        """Perform an authenticated POST request. Returns ``True`` on success."""
        return await self._request('POST', endpoint, json=json) is not None

    async def _request(self, method: Literal['GET', 'POST'], endpoint: str, *,
                       json: dict | None = None) -> httpx.Response | None:
        """Perform an authenticated request, refreshing the token if needed."""
        if not await self._ensure_token():
            return None
        try:
            response = await self._client.request(
                method, f'{self._url}/{endpoint}',
                headers={'Authorization': f'Bearer {self._auth_token}'},
                json=json,
            )
            if response.status_code == 401:
                self.log.warning('%s /%s returned 401, invalidating token', method, endpoint)
                async with self._token_lock:
                    self._auth_token = ''
                    self._token_time = 0.0
                return None
            response.raise_for_status()
            return response
        except httpx.HTTPError:
            self.log.warning('%s /%s failed', method, endpoint)
            return None

    async def _check_connection(self) -> None:
        data = await self._get('failover/status')
        if data is None or not isinstance(data, dict):
            if not self._auth_token:
                return
            self._connection_failures += 1
            if self._connection_failures >= self.MAX_CONNECTION_FAILURES and self._connection_status != ConnectionStatus.DISCONNECTED:
                self._connection_status = ConnectionStatus.DISCONNECTED
                self.CONNECTION_CHANGED.emit(self._connection_status)
            return
        self._connection_failures = 0
        self.log.debug('Raw failover/status response: %s', data)
        up_connection = 'disconnected'
        for key, value in data.items():
            if value.get('status') == 'online':
                up_connection = key
                break
        previous = self._connection_status
        if up_connection == self.FAILOVER_KEY_ETHER:
            self._connection_status = ConnectionStatus.ETHER
        elif any(prefix in up_connection for prefix in self.FAILOVER_KEY_WIFI_PREFIXES):
            self._connection_status = ConnectionStatus.WIFI
        elif up_connection in self.FAILOVER_KEYS_MOBILE:
            self._connection_status = ConnectionStatus.MOBILE
        else:
            self._connection_status = ConnectionStatus.DISCONNECTED
        if previous != self._connection_status:
            self.CONNECTION_CHANGED.emit(self._connection_status)

    async def _get_token(self) -> None:
        self.log.debug('Requesting authentication token...')
        try:
            response = await self._client.post(
                f'{self._url}/login',
                json={'username': 'admin', 'password': self._admin_password},
            )
            response.raise_for_status()
        except httpx.HTTPError:
            self.log.exception('Authentication request failed')
            self._auth_token = ''
            # Backoff: set token_time so that _ensure_token won't retry for AUTH_RETRY_INTERVAL seconds.
            # _ensure_token checks `now - token_time > TOKEN_EXPIRY_SECONDS`, so setting token_time to
            # `now - TOKEN_EXPIRY_SECONDS + AUTH_RETRY_INTERVAL` makes the condition false for ~60s.
            self._token_time = rosys.time() - self.TOKEN_EXPIRY_SECONDS + self.AUTH_RETRY_INTERVAL
            return
        body = response.json()
        token = None
        if 'data' in body and isinstance(body['data'], dict):
            token = body['data'].get('token')
        if token is None:
            # Older RutOS firmware returns the token as 'ubus_rpc_session' at the top level
            token = body.get('ubus_rpc_session')
        if not token:
            self.log.error('No token found in login response: %s', list(body.keys()))
            self._auth_token = ''
            # Same backoff as above — avoid hammering the router on repeated login failures.
            self._token_time = rosys.time() - self.TOKEN_EXPIRY_SECONDS + self.AUTH_RETRY_INTERVAL
            return
        self._auth_token = token
        self._token_time = rosys.time()
        self.log.debug('Authentication successful')

    @staticmethod
    def _normalize_interface_list(data: dict | list) -> list[dict]:
        """Convert a dict-keyed or list response into a flat list of interface dicts."""
        if isinstance(data, list):
            return data
        if isinstance(data, dict):
            return list(data.values())
        return []
