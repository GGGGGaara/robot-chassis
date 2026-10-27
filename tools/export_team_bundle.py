"""Build local source and private-map ZIPs with hashes; never uploads or deletes data."""
import hashlib
import json
from pathlib import Path
import zipfile
from repo_audit import ROOT,audit

report=audit()
if not report['passed']:raise SystemExit('Git candidate audit failed: '+json.dumps(report['errors'],ensure_ascii=False))
out=ROOT/'archives/deliveries';out.mkdir(parents=True,exist_ok=True)
def write_bundle(name,rows,manifest_name,kind):
    target=out/name
    if target.exists():raise SystemExit('Refuse to overwrite existing delivery: '+str(target))
    with zipfile.ZipFile(target,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for row in rows:
            source=ROOT/row['path']
            if hashlib.sha256(source.read_bytes()).hexdigest()!=row['sha256']:raise RuntimeError('Source changed during packaging')
            info=zipfile.ZipInfo(row['path']);info.compress_type=zipfile.ZIP_DEFLATED
            info.create_system=3
            info.external_attr=((0o100755 if source.suffix=='.sh' else 0o100644)<<16)
            z.writestr(info,source.read_bytes())
        z.writestr(manifest_name,json.dumps({'date':'2026-10-27','kind':kind,'files':rows},indent=2,ensure_ascii=False)+'\n')
    with zipfile.ZipFile(target) as z:
        for row in rows:
            if hashlib.sha256(z.read(row['path'])).hexdigest()!=row['sha256']:raise RuntimeError('Archive verification failed')
    checksum=hashlib.sha256(target.read_bytes()).hexdigest()
    target.with_suffix('.zip.sha256').write_text(checksum+'  '+target.name+'\n',encoding='ascii')
    return {'file':str(target.relative_to(ROOT)),'files':len(rows),'bytes':target.stat().st_size,'sha256':checksum}

source=write_bundle('team_source_20261027.zip',report['manifest'],'SOURCE_MANIFEST.json','source snapshot, no Git history or private maps')
assets=[]
for name in ('global_map.pc2.lcm','global_costmap.lcm'):
    path=ROOT/'reconstruction_20261017_113920/raytrace'/name
    if not path.is_file():raise SystemExit('Private map missing: '+str(path))
    data=path.read_bytes();assets.append({'path':path.relative_to(ROOT).as_posix(),'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest()})
private=write_bundle('team_private_maps_20261027.zip',assets,'PRIVATE_ASSETS_MANIFEST.json','private lab reference maps; do not publish')
receipt={'date':'2026-10-27','source':source,'private_maps':private,'uploaded':False,'deleted':False}
(out/'team_delivery_20261027.json').write_text(json.dumps(receipt,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
print(json.dumps(receipt,indent=2,ensure_ascii=False))
