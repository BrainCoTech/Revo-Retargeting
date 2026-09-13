"""Extract a hash-verified FFmpeg tool; leave the Python environment unchanged."""
import json
from pathlib import Path
import subprocess
import urllib.request
import zipfile
from lab_common import checked,digest,run,write_json

def main():
 with run('video_tool') as (out,manifest):
  record=json.loads((Path(__file__).resolve().parents[1]/'configs/video_tool.json').read_text())
  wheel=checked('cache')/record['filename']
  if not wheel.exists():
   with urllib.request.urlopen(record['url'],timeout=40) as response,wheel.open('wb') as f:
    while chunk:=response.read(1024*1024):f.write(chunk)
  if digest(wheel)!=record['digests']['sha256']:raise RuntimeError('FFmpeg wheel hash mismatch')
  folder=checked('toolchains/ffmpeg-imageio-0.6.0');folder.mkdir(exist_ok=True)
  with zipfile.ZipFile(wheel) as z:
   for name in z.namelist():
    if '/binaries/ffmpeg-' in name and not name.endswith('/'):
     binary=folder/'ffmpeg';binary.write_bytes(z.read(name));binary.chmod(0o755)
    elif 'LICENSE' in name.upper():
     (folder/Path(name).name).write_bytes(z.read(name))
  version=subprocess.check_output([str(binary),'-version'],text=True).splitlines()[0]
  record.update(binary=str(binary),binary_sha256=digest(binary),version=version)
  write_json('registry/video_tool.json',record);write_json(out/'tool.json',record)
  print(record)

if __name__=='__main__':main()
