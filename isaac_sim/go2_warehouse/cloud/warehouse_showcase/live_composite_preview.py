"""Loopback-only preview. No ROS publishers, simulator imports or scene writes."""
import argparse,json,threading,time
from pathlib import Path
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from PIL import Image
from compose_local_ribbon import compose
p=argparse.ArgumentParser();p.add_argument('--case',type=Path,required=True);a=p.parse_args();latest=None;stop=False
class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        path=a.case/'showcase_preview.jpg'
        if self.path.startswith('/latest.jpg'):
            if not path.exists():self.send_error(503);return
            self.send_response(200);self.send_header('Content-Type','image/jpeg');self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(path.read_bytes());return
        self.send_response(200);self.send_header('Content-Type','text/html; charset=utf-8');self.end_headers();self.wfile.write(b'<html><body style="margin:0;background:#111"><img id="v" style="width:100vw;max-height:100vh;object-fit:contain"><script>setInterval(()=>v.src="/latest.jpg?t="+Date.now(),500)</script></body></html>')
    def log_message(self,*args):pass
server=ThreadingHTTPServer(('127.0.0.1',8766),Handler);threading.Thread(target=server.serve_forever,daemon=True).start()
try:
    while not (a.case/'RUNNER_RESULT.json').exists():
        files=sorted((a.case/'simulation/follow_video').glob('frame_*.json'))
        if files and files[-1]!=latest:
            latest=files[-1]
            try:
                d=json.loads(latest.read_text());im=compose(Image.open(latest.with_suffix('.jpg')).convert('RGB'),d)
                tmp=a.case/'showcase_preview.tmp.jpg';im.save(tmp,quality=91);tmp.replace(a.case/'showcase_preview.jpg')
            except (OSError,ValueError):pass
        time.sleep(.5)
finally:server.shutdown();server.server_close()
