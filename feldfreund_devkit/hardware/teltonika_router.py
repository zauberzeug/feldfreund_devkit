import logging
from enum import Enum

import httpcore
import httpx
import rosys
from nicegui import Event, ui


class ConnectionStatus(Enum):
    ETHER = 'ether'
    WIFI = 'wifi'
    MOBILE = 'mobile'
    DISCONNECTED = 'disconnected'


class TeltonikaRouter:
    """Implements the api of the builtin Teltonika RUT955 router."""

    def __init__(self, url: str, admin_password: str) -> None:
        self.log = logging.getLogger('feldfreund.hardware.teltonika_router')
        self._url = url
        self._admin_password = admin_password

        self._connection_status: ConnectionStatus = ConnectionStatus.DISCONNECTED
        self._client = httpx.AsyncClient(headers={'Content-Type': 'application/json'}, timeout=20.0)
        self._auth_token: str = ''
        self._token_time: float = 0.0

        self.CONNECTION_CHANGED: Event[ConnectionStatus] = Event()
        """Emitted when the connection status changes."""

        rosys.on_repeat(self._check_connection, 5.0)

    @property
    def connection_status(self) -> ConnectionStatus:
        return self._connection_status

    async def _check_connection(self) -> None:
        if rosys.time() - self._token_time > 4 * 60:
            await self._get_token()
        if self._auth_token == '':
            self.log.error('No authentication token found.')
            return
        self.log.debug('Getting internet connection info...')
        try:
            response = await self._client.get(f'{self._url}/failover/status',
                                              headers={'Authorization': f'Bearer {self._auth_token}'})
            response.raise_for_status()
        except (httpx.RequestError, httpcore.ConnectError):
            self.log.error('Getting Internet Connection Info failed')
            return
        self.log.debug('Getting Internet Connection Info: success')
        up_connection = 'disconnected'
        for key, value in response.json()['data'].items():
            if value.get('status') == 'online':
                up_connection = key
                break
        last_connection = self.connection_status
        if up_connection == 'wan':
            self._connection_status = ConnectionStatus.ETHER
        elif 'ifWan' in up_connection or 'wifi' in up_connection:
            self._connection_status = ConnectionStatus.WIFI
        elif up_connection in ('mob1s1a1', 'mob1s2a1'):
            self._connection_status = ConnectionStatus.MOBILE
        else:
            self._connection_status = ConnectionStatus.DISCONNECTED
        if last_connection != self.connection_status:
            self.CONNECTION_CHANGED.emit(self.connection_status)
        return

    async def _get_token(self) -> None:
        try:
            self.log.info('Getting authentication token for Teltonika router...')
            response = await self._client.post(f'{self._url}/login',
                                               json={'username': 'admin', 'password': self._admin_password})
            response.raise_for_status()
        except (httpx.RequestError, httpx.ConnectError):
            self.log.exception('Teltonika router request failed.')
            self._auth_token = ''
            self._token_time = 0.0
            return
        try:
            self._auth_token = response.json()['data']['token']
        except KeyError:
            self.log.exception('No token in response.')
            self._auth_token = ''
            self._token_time = 0.0
            return
        self._token_time = rosys.time()
        self.log.info('Getting authentication token for Teltonika router: success')

    def status(self) -> ui.element:
        icon = ui.icon('wifi', size='sm')

        def update_icon(connection_status: ConnectionStatus) -> None:
            if connection_status == ConnectionStatus.ETHER:
                icon.name = 'cable'
                icon.props('size=sm')
            elif connection_status == ConnectionStatus.WIFI:
                icon.name = 'wifi'
                icon.props('size=sm')
            elif connection_status == ConnectionStatus.MOBILE:
                icon.name = 'lte_mobiledata'
                icon.props('size=lg')
            else:
                icon.name = 'wifi_off'
                icon.props('size=sm')

        self.CONNECTION_CHANGED.subscribe(update_icon)
        update_icon(self.connection_status)
        return icon
