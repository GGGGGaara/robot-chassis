"""Restore the recorded upstream revision + team patch. Does not install or run DimOS."""
import hashlib
import json
from pathlib import Path
import subprocess

root=Path(__file__).resolve().parents[2]
lock=json.loads((root/'third_party/dependencies.lock.json').read_text(encoding='utf-8'))['dimos']
target=root/'ku/dimos-main'
patch=root/lock['local_patch']
if hashlib.sha256(patch.read_bytes()).hexdigest()!=lock['patch_sha256']:raise SystemExit('Patch checksum mismatch')
if target.exists():raise SystemExit('Target already exists. Existing user checkout left untouched: '+str(target))
target.parent.mkdir(parents=True,exist_ok=True)
subprocess.run(['git','clone','--no-checkout',lock['url'],str(target)],check=True)
subprocess.run(['git','-C',str(target),'checkout','--detach',lock['commit']],check=True)
if patch.stat().st_size:
    subprocess.run(['git','-C',str(target),'apply','--check',str(patch)],check=True)
    subprocess.run(['git','-C',str(target),'apply',str(patch)],check=True)
print('Pinned source + local patch restored. Native builds and Python 3.12 mapping environment remain separate setup steps.')
