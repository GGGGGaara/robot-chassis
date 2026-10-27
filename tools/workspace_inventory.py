"""Inventory all local files without following links/junctions; no mutation/deletion."""
import collections
import csv
from datetime import datetime,timezone
import json
import os
from pathlib import Path
import stat

ROOT=Path(__file__).resolve().parents[1]
out=ROOT/'docs/organization';out.mkdir(parents=True,exist_ok=True)
csv_path=out/'文件清单_20261027.csv';summary_path=out/'inventory_20261027.json'
skip={csv_path,summary_path};errors=[];links=[];totals=collections.defaultdict(lambda:[0,0])
def category(relative):
    parts=Path(relative).parts;top=parts[0]
    if '.git' in parts:return '版本控制元数据（保留本地）'
    if any(p in parts for p in ('.ros_venv','.venv','__pycache__','build','install','sdk_install','target','node_modules')):return '依赖或可再生成产物（保留本地）'
    if top=='tracer_navigation' and len(parts)>1 and parts[1] in ('results','model_sessions','generated'):return '实验结果或模型（保留本地）'
    return {'archives':'历史交付与备份（保留本地）','data':'实验数据（保留本地）','ku':'DimOS 上游（锁定提交+补丁）',
        'TRACER2.0':'原厂源码原件（保留本地）','vendor':'厂商资料与工具（保留本地）',
        'reconstruction_20261017_113920':'当前参考地图（私有资产）','third_party':'团队依赖源码快照（Git）',
        'tracer_navigation':'导航源码与根层运行报告（见 Git 白名单）','realtime_mapping_usage':'建图适配与本地报告（见 Git 白名单）',
        'docs':'文档与本地索引（见 Git 白名单）','tools':'工具（Git）','config':'模板或个人配置（local.env不入Git）'}.get(top,'根层团队入口')
with csv_path.open('w',newline='',encoding='utf-8-sig') as handle:
    writer=csv.DictWriter(handle,fieldnames=['path','bytes','modified_utc','category']);writer.writeheader()
    stack=[ROOT]
    while stack:
        folder=stack.pop()
        try:
            with os.scandir(folder) as entries:
                for entry in entries:
                    path=Path(entry.path)
                    if path in skip:continue
                    try:
                        info=entry.stat(follow_symlinks=False)
                        if entry.is_symlink() or getattr(info,'st_file_attributes',0)&getattr(stat,'FILE_ATTRIBUTE_REPARSE_POINT',0):
                            links.append(path.relative_to(ROOT).as_posix());continue
                        if entry.is_dir(follow_symlinks=False):stack.append(path);continue
                        if not entry.is_file(follow_symlinks=False):continue
                        rel=path.relative_to(ROOT).as_posix();kind=category(rel)
                        writer.writerow({'path':rel,'bytes':info.st_size,'modified_utc':datetime.fromtimestamp(info.st_mtime,timezone.utc).isoformat(),'category':kind})
                        totals[rel.split('/')[0]][0]+=1;totals[rel.split('/')[0]][1]+=info.st_size
                    except OSError as e:errors.append({'path':str(path),'error':str(e)})
        except OSError as e:errors.append({'path':str(folder),'error':str(e)})
report={'snapshot_date':'2026-10-27','generated_at_utc':datetime.now(timezone.utc).isoformat(),'root':str(ROOT),
    'files':sum(v[0] for v in totals.values()),'bytes':sum(v[1] for v in totals.values()),
    'directories':{k:{'files':v[0],'bytes':v[1]} for k,v in sorted(totals.items())},
    'links_not_followed':links,'errors':errors,'deletions':0,
    'notes':'Metadata inventory, not semantic review or backup. Current output files excluded; original data retained.'}
summary_path.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
print(json.dumps({'files':report['files'],'bytes':report['bytes'],'links_not_followed':len(links),'errors':len(errors),'deletions':0},indent=2))
