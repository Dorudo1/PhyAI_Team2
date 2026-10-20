"""Browser streams and the lab scene editor's HTTP API."""
import json
from pathlib import Path
import threading
from http.server import BaseHTTPRequestHandler
from urllib.parse import parse_qs, urlparse
from urllib.request import urlopen
from urllib.error import URLError


VIEWER_PAGE = Path(__file__).with_name('viewer.html').read_bytes()
EDITOR_PAGE = Path(__file__).with_name('editor.html').read_bytes()


class FrameHub:
    def __init__(self):
        self.cond = threading.Condition()
        self.frame_id, self.jpeg, self.clients = 0, b'', 0

    def publish(self, jpeg):
        with self.cond:
            self.frame_id += 1
            self.jpeg = jpeg
            self.cond.notify_all()

    def wait(self, last_id, timeout=1.0):
        with self.cond:
            self.cond.wait_for(lambda: self.frame_id != last_id, timeout)
            return self.frame_id, self.jpeg


def make_handler(hubs, editor=None, scene_editing=False, restart=None, workspace_port=0, block_action=None):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def respond(self, status, data, content_type='application/json'):
            body = data if isinstance(data, bytes) else json.dumps(data, allow_nan=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            request = urlparse(self.path)
            if request.path == '/':
                page = EDITOR_PAGE if scene_editing and editor else VIEWER_PAGE
                self.respond(200, page, 'text/html; charset=utf-8')
            elif request.path in ('/edit', '/edit/'):
                self.respond(200, EDITOR_PAGE, 'text/html; charset=utf-8')
            elif request.path == '/edit/scene':
                self.respond(200, editor.snapshot() if editor else {
                    'enabled': False, 'message': 'Scene editing requires the lab scene; custom MJCF models are view-only.'})
            elif request.path == '/workspace-link':
                self.respond(200, {'port': workspace_port, 'url': '/workspace' if workspace_port else None})
            elif request.path in ('/workspace', '/workspace/', '/api/workspace'):
                if not workspace_port:
                    self.respond(404, b'Workspace viewer is not enabled for this launch.', 'text/plain')
                    return
                upstream_path = '/api/workspace' if request.path == '/api/workspace' else '/'
                try:
                    with urlopen(f'http://127.0.0.1:{workspace_port}{upstream_path}', timeout=2) as upstream:
                        self.respond(200, upstream.read(), upstream.headers.get('Content-Type', 'application/octet-stream'))
                except (URLError, TimeoutError, OSError):
                    self.respond(503, b'Workspace viewer is unavailable. Check the workspace_viewer launch logs, then reload this page.', 'text/plain')
            elif request.path == '/stream':
                name = parse_qs(request.query).get('view', ['overview'])[0]
                if name not in hubs:
                    self.send_error(404)
                    return
                self.stream(hubs[name])
            else:
                self.send_error(404)

        def do_POST(self):
            action = urlparse(self.path).path
            if action not in ('/edit/scene/preview', '/edit/scene/save', '/edit/scene/revert',
                              '/edit/scene/pause', '/edit/scene/reload', '/simulation/restart',
                              '/simulation/blocks/scatter', '/simulation/blocks/reset'):
                self.respond(404, {'error': 'Unknown editor action.'})
                return
            if action == '/simulation/restart' and restart is None:
                self.respond(409, {'error': 'Simulation restart is unavailable.'})
                return
            if action.startswith('/simulation/blocks/') and block_action is None:
                self.respond(409, {'error': 'Block scattering requires the lab scene.'})
                return
            if not action.startswith('/simulation/') and editor is None:
                self.respond(409, {'error': 'Scene editing is unavailable for custom models.'})
                return
            # Browser writes come from this UI, never cross-origin form submissions.
            origin = self.headers.get('Origin')
            if origin and urlparse(origin).netloc != self.headers.get('Host'):
                self.respond(403, {'error': 'Use the editor on this simulator.'})
                return
            if self.headers.get_content_type() != 'application/json':
                self.respond(415, {'error': 'Expected JSON.'})
                return
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 65536:
                    raise ValueError('Invalid request size.')
                payload = json.loads(self.rfile.read(length))
                if not isinstance(payload, dict):
                    raise ValueError('Expected an object.')
                if action.startswith('/simulation/blocks/'):
                    result = block_action(action.rsplit('/', 1)[1], payload)
                else:
                    result = restart() if action == '/simulation/restart' else editor.action(action.rsplit('/', 1)[1], payload)
                self.respond(200, result)
            except (ValueError, KeyError, TypeError) as error:
                self.respond(400, {'error': str(error)})
            except OSError as error:
                self.respond(409, {'error': f'Could not save scene: {error}'})

        def stream(self, hub):
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            with hub.cond:
                hub.clients += 1
                last_id = hub.frame_id
                hub.cond.notify_all()
            try:
                while True:
                    frame_id, jpeg = hub.wait(last_id)
                    if frame_id == last_id or not jpeg:
                        continue
                    last_id = frame_id
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n'
                                     % len(jpeg) + jpeg + b'\r\n')
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                with hub.cond:
                    hub.clients -= 1

    return Handler
