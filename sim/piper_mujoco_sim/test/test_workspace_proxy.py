"""Workspace navigation uses the simulator origin, including forwarded ports."""
import json
import threading
from http.server import ThreadingHTTPServer
from urllib.error import HTTPError
from urllib.request import urlopen

import pytest

from piper_mujoco_sim.web import make_handler


def test_workspace_proxy():
    # A separate local backend represents the viewer process.
    from http.server import BaseHTTPRequestHandler
    class Backend(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            self.send_response(200)
            if self.path == '/':
                self.send_header('Content-Type', 'text/html')
                body = b'<html>Discovered workspace</html>'
            else:
                assert self.path == '/api/workspace'
                self.send_header('Content-Type', 'application/json')
                body = b'{"snapshot":{"workspace":{"revision":7}}}'
            self.end_headers()
            self.wfile.write(body)

    backend = ThreadingHTTPServer(('127.0.0.1', 0), Backend)
    server = ThreadingHTTPServer(('127.0.0.1', 0), make_handler(
        {}, workspace_port=backend.server_port))
    threads = [threading.Thread(target=s.serve_forever, daemon=True)
               for s in (backend, server)]
    for thread in threads:
        thread.start()
    root = f'http://127.0.0.1:{server.server_port}'
    try:
        assert json.load(urlopen(root + '/workspace-link'))['url'] == '/workspace'
        for path in ('/workspace', '/workspace/'):
            response = urlopen(root + path)
            assert response.headers['Content-Type'] == 'text/html'
            assert b'Discovered workspace' in response.read()
        assert json.load(urlopen(root + '/api/workspace'))['snapshot']['workspace']['revision'] == 7
        backend.shutdown()
        backend.server_close()
        with pytest.raises(HTTPError) as error:
            urlopen(root + '/workspace')
        assert error.value.code == 503
        assert b'workspace_viewer' in error.value.read()
    finally:
        backend.shutdown()
        backend.server_close()
        server.shutdown()
        server.server_close()
        for thread in threads:
            thread.join(timeout=2)


def test_workspace_disabled():
    server = ThreadingHTTPServer(('127.0.0.1', 0), make_handler({}))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        root = f'http://127.0.0.1:{server.server_port}'
        assert json.load(urlopen(root + '/workspace-link'))['url'] is None
        with pytest.raises(HTTPError) as error:
            urlopen(root + '/workspace')
        assert error.value.code == 404
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
