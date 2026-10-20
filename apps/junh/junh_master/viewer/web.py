"""Local read-only browser display of discovery; no simulator geometry."""
import json
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import threading
import time


class WorkspaceWeb:
    def __init__(self,port,simulator_port=0):
        self.simulator_port=simulator_port
        self.latest=None
        self.received=None
        self.lock=threading.Lock()
        page=Path(__file__).with_name('workspace.html').read_bytes()
        owner=self
        class Handler(BaseHTTPRequestHandler):
            def log_message(self,*args):pass
            def do_GET(self):
                if self.path in ('/','/workspace'):
                    data=page;content='text/html; charset=utf-8';code=200
                elif self.path=='/api/workspace':
                    with owner.lock:
                        data=json.dumps({'snapshot':owner.latest,'simulator_port':owner.simulator_port,
                            'publisher_age':None if owner.received is None else time.monotonic()-owner.received},allow_nan=False).encode()
                    content='application/json';code=200
                else:data=b'Not found';content='text/plain';code=404
                self.send_response(code)
                self.send_header('Content-Type',content)
                self.send_header('Content-Length',str(len(data)))
                self.send_header('Cache-Control','no-store')
                self.end_headers();self.wfile.write(data)
        self.server=ThreadingHTTPServer(('127.0.0.1',port),Handler)
        self.server.daemon_threads=True
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True)
        self.thread.start()

    def update(self,snapshot):
        with self.lock:self.latest=snapshot;self.received=time.monotonic()

    def close(self):
        self.server.shutdown();self.server.server_close();self.thread.join(timeout=2)
