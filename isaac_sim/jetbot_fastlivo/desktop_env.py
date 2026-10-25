"""Find the existing desktop's display paths without storing authentication material."""
import os
from pathlib import Path

def discover_desktop():
 for proc in sorted(Path('/proc').iterdir(),key=lambda p:int(p.name)if p.name.isdigit()else 0):
  if not proc.name.isdigit():continue
  try:
   if proc.stat().st_uid!=os.getuid():continue
   args=(proc/'cmdline').read_bytes().split(b'\0')
   if not any(Path(v.decode()).name in('gnome-session-binary','gnome-shell','xfce4-session','plasmashell')for v in args if v):continue
   selected={}
   for field in(proc/'environ').read_bytes().split(b'\0'):
    key,sep,value=field.partition(b'=')
    if sep and key in(b'DISPLAY',b'XAUTHORITY',b'XDG_RUNTIME_DIR'):selected[key.decode()]=value.decode()
   if selected.get('DISPLAY')and selected.get('XAUTHORITY')and Path(selected['XAUTHORITY']).is_file():return dict(environment=selected,source_pid=int(proc.name),source='existing desktop session; paths only, no credential contents')
  except (OSError,UnicodeDecodeError):continue
 raise RuntimeError('Requested GUI but no readable existing desktop display/Xauthority paths found')
