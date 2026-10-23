import argparse,json,sqlite3,zlib
from pathlib import Path
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image as CameraImage
from PIL import Image
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--case',type=Path,required=True);a=p.parse_args()
db=sqlite3.connect(f'file:{a.case/"recording/sensors.sqlite3"}?mode=ro',uri=True)
r=db.execute('SELECT stamp_ns,payload FROM records WHERE topic=? ORDER BY id DESC LIMIT 1',('/livo/image_raw',)).fetchone();m=deserialize_message(zlib.decompress(r[1]),CameraImage)
pixels=np.frombuffer(m.data,dtype=np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
if m.encoding=='bgr8':pixels=pixels[:,:,::-1]
Image.fromarray(pixels).save(a.case/'native_rgb_preview.png');(a.case/'native_rgb_preview.json').write_text(json.dumps(dict(stamp_s=r[0]/1e9,source='/livo/image_raw recorded native RGB',purpose='check viewport overlay isolation'))+'\n');print(r[0]/1e9,m.width,m.height)
