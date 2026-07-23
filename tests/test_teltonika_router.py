import json

import httpx
import pytest
import rosys

from feldfreund_devkit.hardware.teltonika_router import ConnectionStatus, TeltonikaRouter

BASE_PATH = '/api/wireless/multi_ap/config'


def _entry(section_id: str, ssid: str, *, enabled: str = '1', priority: str, **extra: str) -> dict:
    """Build a raw MultiAP candidate config entry as the RutOS API returns it.

    :param section_id: the config section id.
    :param ssid: the network name.
    :param enabled: the RutOS ``enabled`` flag ('1' or '0').
    :param priority: the connection priority.
    :param extra: additional raw fields to merge in.
    :return: the raw entry dict.
    """
    return {'id': section_id, 'ssid': ssid, 'key': 'secret', 'enabled': enabled,
            'priority': priority, '.type': 'wifi-iface', **extra}


class FakeMultiApApi:
    """A stateful RutOS ``wireless/multi_ap/config`` endpoint backed by an in-memory entry list."""

    def __init__(self, entries: list[dict]) -> None:
        self.entries = entries
        self.requests: list[tuple[str, str, dict | None]] = []
        self._counter = len(entries)  # new ids continue past the seeded entries

    def handler(self, request: httpx.Request) -> httpx.Response:
        """Serve GET/POST/PUT/DELETE against the in-memory entry list.

        :param request: the intercepted HTTP request.
        :return: the mocked response.
        """
        body = json.loads(request.content)['data'] if request.content else None
        self.requests.append((request.method, request.url.path, body))
        path = request.url.path
        if path == BASE_PATH:
            if request.method == 'GET':
                return httpx.Response(200, json={'data': self.entries})
            if request.method == 'POST':
                self._counter += 1
                section_id = str(self._counter)
                self.entries.append({**body, 'id': section_id, '.type': 'wifi-iface'})
                return httpx.Response(200, json={'data': {'id': section_id}})
        if path.startswith(f'{BASE_PATH}/'):
            section_id = path.rsplit('/', 1)[1]
            if request.method == 'PUT':
                for entry in self.entries:
                    if entry.get('id') == section_id:
                        entry.update(body)
                return httpx.Response(200, json={'data': {'id': section_id}})
            if request.method == 'DELETE':
                self.entries = [e for e in self.entries if e.get('id') != section_id]
                return httpx.Response(200, json={'data': {}})
        return httpx.Response(404, json={'error': 'not found'})


def _make_router(api: FakeMultiApApi) -> TeltonikaRouter:
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


async def test_refresh_parses_and_sorts_by_priority(rosys_integration):
    """Refreshing parses enabled state and orders the candidates by priority."""
    api = FakeMultiApApi([
        _entry('2', 'Shed', enabled='0', priority='2'),
        _entry('1', 'Barn', enabled='1', priority='1'),
    ])
    router = _make_router(api)
    await router.refresh_wifi_client_networks()
    networks = router.wifi_client_networks
    assert [n.ssid for n in networks] == ['Barn', 'Shed']
    assert [n.enabled for n in networks] == [True, False]
    assert [n.priority for n in networks] == [1, 2]


async def test_refresh_emits_event(rosys_integration):
    """Refreshing emits WIFI_NETWORKS_CHANGED."""
    api = FakeMultiApApi([_entry('1', 'Barn', priority='1')])
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
    api = FakeMultiApApi([_entry('1', 'Barn', enabled='1', priority='1')])
    router = _make_router(api)
    assert await router.set_wifi_client_enabled('1', False)
    put_requests = [r for r in api.requests if r[0] == 'PUT']
    assert put_requests == [('PUT', f'{BASE_PATH}/1', {'enabled': '0'})]
    assert router.wifi_client_networks[0].enabled is False


async def test_add_appends_with_next_priority(rosys_integration):
    """Adding posts ssid/key/enabled with the next free priority and reports success."""
    api = FakeMultiApApi([_entry('1', 'Barn', priority='1'), _entry('2', 'Shed', priority='2')])
    router = _make_router(api)
    await router.refresh_wifi_client_networks()
    assert await router.add_wifi_client_network('Field', 'hunter2', enabled=True) is True
    post = next(r for r in api.requests if r[0] == 'POST')
    assert post[2] == {'ssid': 'Field', 'key': 'hunter2', 'priority': '3', 'enabled': '1'}
    assert {n.ssid for n in router.wifi_client_networks} == {'Barn', 'Shed', 'Field'}


async def test_add_on_empty_list_uses_priority_one(rosys_integration):
    """Adding to an empty list assigns priority 1."""
    api = FakeMultiApApi([])
    router = _make_router(api)
    await router.refresh_wifi_client_networks()
    assert await router.add_wifi_client_network('Field', 'hunter2') is True
    post = next(r for r in api.requests if r[0] == 'POST')
    assert post[2]['priority'] == '1'


async def test_remove_deletes_and_refreshes(rosys_integration):
    """Removing deletes the entry and drops it from the cached list."""
    api = FakeMultiApApi([_entry('1', 'Barn', priority='1'), _entry('2', 'Shed', priority='2')])
    router = _make_router(api)
    assert await router.remove_wifi_client_network('1')
    assert any(r[0] == 'DELETE' and r[1] == f'{BASE_PATH}/1' for r in api.requests)
    assert [n.ssid for n in router.wifi_client_networks] == ['Shed']


def test_parse_falls_back_to_disabled_field():
    """When only the UCI 'disabled' flag is present, enabled is its inverse."""
    # pylint: disable=protected-access
    assert TeltonikaRouter._parse_wifi_client({'id': 'x', 'ssid': 'A', 'disabled': '0'}).enabled is True
    assert TeltonikaRouter._parse_wifi_client({'id': 'x', 'ssid': 'A', 'disabled': '1'}).enabled is False


def _router_with_failover(interfaces: dict, monkeypatch: pytest.MonkeyPatch) -> TeltonikaRouter:
    """Create a router whose ``failover/status`` endpoint returns ``interfaces``, with a preset token.

    Neutralizes the rosys lifecycle hooks so that constructing the router does not schedule
    background polling or fire startup polls against the real HTTP client (startup has already
    finished inside the ``rosys_integration`` fixture, so ``on_startup`` would run immediately).

    :param interfaces: the raw mwan3 failover interfaces, keyed by name.
    :param monkeypatch: neutralizes the rosys lifecycle registration during construction.
    :return: the ready-to-use router.
    """
    monkeypatch.setattr(rosys, 'on_repeat', lambda *args, **kwargs: None)
    monkeypatch.setattr(rosys, 'on_startup', lambda *args, **kwargs: None)
    monkeypatch.setattr(rosys, 'on_shutdown', lambda *args, **kwargs: None)

    def handler(request: httpx.Request) -> httpx.Response:
        if request.url.path == '/api/failover/status':
            return httpx.Response(200, json={'data': interfaces})
        return httpx.Response(404, json={'error': 'not found'})
    router = TeltonikaRouter('http://router/api', 'password')
    # pylint: disable=protected-access
    router._client = httpx.AsyncClient(transport=httpx.MockTransport(handler),
                                       headers={'Content-Type': 'application/json'})
    router._auth_token = 'test-token'
    router._token_time = rosys.time()
    return router


async def test_connection_online_wifi_is_connected(rosys_integration, monkeypatch):
    """An online WiFi-WAN interface is reported as the wifi connection."""
    router = _router_with_failover({
        'wan': {'up': False, 'status': 'disabled'},
        'mob1s1a1': {'up': False, 'status': 'disabled'},
        'ifWan1': {'up': True, 'status': 'online'},
    }, monkeypatch)
    await router._check_connection()  # pylint: disable=protected-access
    assert router.connection_status is ConnectionStatus.WIFI


async def test_connection_online_ethernet_is_connected(rosys_integration, monkeypatch):
    """An online ethernet interface is reported as the ether connection."""
    router = _router_with_failover({
        'wan': {'up': True, 'status': 'online'},
        'ifWan1': {'up': False, 'status': 'offline'},
    }, monkeypatch)
    await router._check_connection()  # pylint: disable=protected-access
    assert router.connection_status is ConnectionStatus.ETHER


async def test_connection_prefers_ethernet_over_simultaneous_wifi(rosys_integration, monkeypatch):
    """With both a WiFi and the wired uplink online, the wired connection wins regardless of order."""
    router = _router_with_failover({
        'ifWan1': {'up': True, 'status': 'online'},
        'wan': {'up': True, 'status': 'online'},
    }, monkeypatch)
    await router._check_connection()  # pylint: disable=protected-access
    assert router.connection_status is ConnectionStatus.ETHER


async def test_connection_offline_and_disabled_is_disconnected(rosys_integration, monkeypatch):
    """A linked-but-offline interface (no internet) is not counted as connected."""
    router = _router_with_failover({
        'wan': {'up': False, 'status': 'disabled'},
        'ifWan1': {'up': True, 'status': 'offline'},
    }, monkeypatch)
    await router._check_connection()  # pylint: disable=protected-access
    assert router.connection_status is ConnectionStatus.DISCONNECTED
