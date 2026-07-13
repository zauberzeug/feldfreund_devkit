import json

import httpx
import rosys

from feldfreund_devkit.hardware.teltonika_router import TeltonikaRouter

BASE_PATH = '/api/wireless/interfaces/config'


def _client(section_id: str, ssid: str, *, enabled: str = '1', **extra: str) -> dict:
    """Build a raw upstream-client interface config entry as the RutOS API returns it.

    :param section_id: the config section id.
    :param ssid: the network name.
    :param enabled: the RutOS ``enabled`` flag ('1' or '0').
    :param extra: additional raw fields to merge in.
    :return: the raw interface dict.
    """
    return {'id': section_id, 'mode': 'multi_ap', 'ssid': ssid, 'enabled': enabled,
            'encryption': 'psk2', 'wifi_id': 'radio0', 'network': 'wwan', **extra}


class FakeRouterApi:
    """A stateful RutOS wireless-config endpoint backed by an in-memory interface list."""

    def __init__(self, interfaces: list[dict]) -> None:
        self.interfaces = interfaces
        self.requests: list[tuple[str, str, dict | None]] = []
        self._counter = 0

    def handler(self, request: httpx.Request) -> httpx.Response:
        """Serve GET/POST/PUT/DELETE against the in-memory interface list.

        :param request: the intercepted HTTP request.
        :return: the mocked response.
        """
        body = json.loads(request.content)['data'] if request.content else None
        self.requests.append((request.method, request.url.path, body))
        path = request.url.path
        if path == BASE_PATH:
            if request.method == 'GET':
                return httpx.Response(200, json={'data': self.interfaces})
            if request.method == 'POST':
                self._counter += 1
                section_id = f'cfg{self._counter}'
                self.interfaces.append({**body, 'id': section_id, '.name': section_id})
                return httpx.Response(200, json={'data': {'id': section_id}})
        if path.startswith(f'{BASE_PATH}/'):
            section_id = path.rsplit('/', 1)[1]
            if request.method == 'PUT':
                for interface in self.interfaces:
                    if interface.get('id') == section_id:
                        interface.update(body)
                return httpx.Response(200, json={'data': {'id': section_id}})
            if request.method == 'DELETE':
                self.interfaces = [i for i in self.interfaces if i.get('id') != section_id]
                return httpx.Response(200, json={'data': {}})
        return httpx.Response(404, json={'error': 'not found'})


def _make_router(api: FakeRouterApi) -> TeltonikaRouter:
    """Create a router wired to a mocked transport with a pre-seeded auth token.

    :param api: the fake API whose handler backs the mock transport.
    :return: the ready-to-use router.
    """
    router = TeltonikaRouter('http://router/api', 'password')
    # pylint: disable=protected-access
    router._client = httpx.AsyncClient(transport=httpx.MockTransport(api.handler),
                                       headers={'Content-Type': 'application/json'})
    router._auth_token = 'test-token'
    router._token_time = rosys.time()
    return router


async def test_refresh_lists_only_client_networks(rosys_integration):
    """Refreshing keeps only upstream-client interfaces and parses their enabled state."""
    api = FakeRouterApi([
        _client('cfg_a', 'Barn', enabled='1'),
        {'id': 'cfg_ap', 'mode': 'ap', 'ssid': 'RobotAP', 'enabled': '1'},
        _client('cfg_b', 'Shed', enabled='0'),
    ])
    router = _make_router(api)
    await router.refresh_wifi_client_networks()
    networks = router.wifi_client_networks
    assert [n.ssid for n in networks] == ['Barn', 'Shed']
    assert [n.enabled for n in networks] == [True, False]
    assert networks[0].id == 'cfg_a'


async def test_refresh_emits_event(rosys_integration):
    """Refreshing emits WIFI_NETWORKS_CHANGED."""
    api = FakeRouterApi([_client('cfg_a', 'Barn')])
    router = _make_router(api)
    events = 0

    def on_change() -> None:
        nonlocal events
        events += 1
    router.WIFI_NETWORKS_CHANGED.subscribe(on_change)
    await router.refresh_wifi_client_networks()
    assert events == 1


async def test_set_enabled_sends_put_and_refreshes(rosys_integration):
    """Toggling sends a PUT with the enabled flag and refreshes the cached list."""
    api = FakeRouterApi([_client('cfg_a', 'Barn', enabled='1')])
    router = _make_router(api)
    assert await router.set_wifi_client_enabled('cfg_a', False)
    put_requests = [r for r in api.requests if r[0] == 'PUT']
    assert put_requests == [('PUT', f'{BASE_PATH}/cfg_a', {'enabled': '0'})]
    assert router.wifi_client_networks[0].enabled is False


async def test_add_derives_wifi_id_and_network_from_existing(rosys_integration):
    """Adding copies wifi_id/network from an existing client entry and returns the new id."""
    api = FakeRouterApi([_client('cfg_a', 'Barn', wifi_id='radio1', network='wwan2')])
    router = _make_router(api)
    network_id = await router.add_wifi_client_network('Field', 'secret', encryption='sae', enabled=True)
    assert network_id == 'cfg1'
    post = next(r for r in api.requests if r[0] == 'POST')
    assert post[2] == {'mode': 'multi_ap', 'ssid': 'Field', 'key': 'secret', 'encryption': 'sae',
                       'wifi_id': 'radio1', 'network': 'wwan2', 'enabled': '1'}
    assert {n.ssid for n in router.wifi_client_networks} == {'Barn', 'Field'}


async def test_add_fails_without_template(rosys_integration):
    """Adding fails (returns None, sends no POST) when no client entry exists to copy from."""
    api = FakeRouterApi([])
    router = _make_router(api)
    assert await router.add_wifi_client_network('Field', 'secret') is None
    assert not any(r[0] == 'POST' for r in api.requests)


async def test_remove_deletes_and_refreshes(rosys_integration):
    """Removing deletes the entry and drops it from the cached list."""
    api = FakeRouterApi([_client('cfg_a', 'Barn'), _client('cfg_b', 'Shed')])
    router = _make_router(api)
    assert await router.remove_wifi_client_network('cfg_a')
    assert any(r[0] == 'DELETE' and r[1] == f'{BASE_PATH}/cfg_a' for r in api.requests)
    assert [n.ssid for n in router.wifi_client_networks] == ['Shed']


def test_parse_falls_back_to_disabled_field():
    """When only the UCI 'disabled' flag is present, enabled is its inverse."""
    # pylint: disable=protected-access
    assert TeltonikaRouter._parse_wifi_client({'id': 'x', 'ssid': 'A', 'disabled': '0'}).enabled is True
    assert TeltonikaRouter._parse_wifi_client({'id': 'x', 'ssid': 'A', 'disabled': '1'}).enabled is False
