"""Audit the exact Git candidate set, not the whole mixed local workspace."""
import ast
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
def candidates():
    output=subprocess.check_output(['git','-C',str(ROOT),'ls-files','--cached','--others','--exclude-standard','-z'])
    return sorted(set(p.decode('utf-8') for p in output.split(b'\0') if p))

def audit():
    rows=[];errors=[]
    secret_patterns=[re.compile(rb'ghp_[A-Za-z0-9]{30,}'),re.compile(rb'github_pat_[A-Za-z0-9_]{35,}'),
        re.compile(rb'AKIA[A-Z0-9]{16}'),re.compile(rb'^-----BEGIN (?:[A-Z ]+ )?PRIVATE KEY-----$',re.M)]
    blocked={'archives','data','ku','TRACER2.0','vendor','reconstruction_20261017_113920'}
    for name in candidates():
        path=ROOT/name
        if not path.is_file():errors.append({'path':name,'reason':'missing/non-file candidate'});continue
        if path.is_symlink() or not path.resolve().is_relative_to(ROOT.resolve()):
            errors.append({'path':name,'reason':'link or outside repository'});continue
        if Path(name).parts[0] in blocked or any(p in Path(name).parts for p in ('.git','.ros_venv','.venv','node_modules','model_sessions','generated','results')):
            errors.append({'path':name,'reason':'local assets/runtime leaked into Git candidates'})
        data=path.read_bytes()
        if len(data)>5*1024*1024:errors.append({'path':name,'reason':'larger than 5 MiB; review as external asset'})
        if path.suffix.lower() in ('.bag','.db3','.mcap','.lcm','.cdr','.ply','.zip','.exe','.dll','.pem','.key'):
            errors.append({'path':name,'reason':'private/binary/runtime asset extension'})
        if any(pattern.search(data) for pattern in secret_patterns):errors.append({'path':name,'reason':'possible credential; inspect locally, content omitted'})
        try:
            if path.suffix=='.py':ast.parse(data.decode('utf-8-sig'),filename=name)
            if path.suffix=='.xml':ET.fromstring(data)
        except (SyntaxError,UnicodeDecodeError,ET.ParseError) as e:errors.append({'path':name,'reason':str(e)})
        rows.append({'path':name,'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest()})
    return {'passed':not errors,'files':len(rows),'bytes':sum(x['bytes'] for x in rows),
            'errors':errors,'limits':'Pattern scan is not a complete security/privacy/license audit. No uploads performed.','manifest':rows}

if __name__=='__main__':
    report=audit()
    folder=ROOT/'docs/organization';folder.mkdir(parents=True,exist_ok=True)
    (folder/'git_candidates_20261027.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='manifest'},indent=2,ensure_ascii=False))
    sys.exit(0 if report['passed'] else 1)
