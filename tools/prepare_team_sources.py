"""Snapshot build-required vendor sources and preserve DimOS's local patch; no deletion."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess

ROOT=Path(__file__).resolve().parents[1]
original=ROOT/'TRACER2.0/TRACER2.0'
target=ROOT/'third_party/tracer_vendor'
manifest=[]
def copy(source,destination):
    data=source.read_bytes()
    if destination.exists() and destination.read_bytes()!=data:
        raise RuntimeError('Refuse to overwrite a different snapshot: '+str(destination))
    destination.parent.mkdir(parents=True,exist_ok=True)
    if not destination.exists():shutil.copy2(source,destination)
    assert hashlib.sha256(destination.read_bytes()).digest()==hashlib.sha256(data).digest()
    manifest.append({'source':source.relative_to(ROOT).as_posix(),'path':destination.relative_to(ROOT).as_posix(),
                     'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest()})
for folder in ('include','src','cmake','sample'):
    for source in sorted((original/'sdk'/folder).rglob('*')):
        if source.is_file() and '.git' not in source.parts:
            copy(source,target/'sdk'/source.relative_to(original/'sdk'))
for name in ('CMakeLists.txt','LICENSE','README.md','package.xml'):
    copy(original/'sdk'/name,target/'sdk'/name)
for source in sorted((original/'ros2/tracer_msgs').rglob('*')):
    if source.is_file():copy(source,target/'ros2/tracer_msgs'/source.relative_to(original/'ros2/tracer_msgs'))
def git(folder,*args):
    return subprocess.check_output(['git','-C',str(folder),*args])
dimos=ROOT/'ku/dimos-main';sdk=original/'sdk'
patch=git(dimos,'diff','--binary','HEAD','--','dimos/mapping/cli/map.py')
patch_path=ROOT/'third_party/patches/dimos-local.patch';patch_path.parent.mkdir(parents=True,exist_ok=True);patch_path.write_bytes(patch)
lock={'schema_version':1,'snapshot_date':'2026-10-27',
    'dimos':{'url':git(dimos,'remote','get-url','origin').decode().strip(),'commit':git(dimos,'rev-parse','HEAD').decode().strip(),
             'local_patch':'third_party/patches/dimos-local.patch','patch_sha256':hashlib.sha256(patch).hexdigest(),
             'python':'3.12 (current validated mapping environment; separate from ROS Python 3.10)'},
    'ugv_sdk':{'url':git(sdk,'remote','get-url','origin').decode().strip(),'commit':git(sdk,'rev-parse','HEAD').decode().strip(),
               'source':'build-required local snapshot, including existing local changes; not a pristine upstream checkout',
               'license':'Apache-2.0; original LICENSE retained'},
    'ros_python_packages':{'lcm':'1.5.3','dimos_lcm':'0.1.4 (schema only; install with --no-deps)'},
    'mapping_python_packages_observed':{'dimos':'0.0.14','dimos_lcm':'0.1.4','lcm-dimos-fork':'1.5.2.post1','rerun-sdk':'0.32.0'},
    'vendor_files':manifest}
(ROOT/'third_party/dependencies.lock.json').write_text(json.dumps(lock,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
print(json.dumps({'verified_vendor_files':len(manifest),'bytes':sum(x['bytes'] for x in manifest),'dimos_commit':lock['dimos']['commit']},indent=2))
