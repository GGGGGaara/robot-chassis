"""Read-only installation and CLI checks; never starts a sensor blueprint."""
import json
import os
from pathlib import Path
import subprocess
import sys

repo = Path(os.environ.get('TRACER_DIMOS_ROOT',str(Path(__file__).resolve().parent.parent/'ku/dimos-main'))).expanduser()
out = Path(__file__).resolve().parent
env = dict(os.environ, PYTHONPATH=str(repo), COLUMNS='160', NO_COLOR='1')
commands = {
    'raytrace_help': [sys.executable,'-m','dimos.cli.dimos','--transport','lcm','--rerun-open','web','run','mid360-fastlio-ray-trace','--help'],
    'raytrace_costmapper_help': [sys.executable,'-m','dimos.cli.dimos','--transport','lcm','--rerun-open','web','run','mid360-fastlio-ray-trace','cost-mapper','--help'],
    'memory_summary_help': [sys.executable,'-m','dimos.cli.dimos','mem','summary','--help'],
}
report = {'python':sys.executable, 'repository':str(repo),'commands':{},'binaries':{}}
for name, command in commands.items():
    result = subprocess.run(command, cwd=repo, env=env, capture_output=True, text=True,timeout=120)
    (out/f'{name}.txt').write_text(result.stdout+result.stderr,encoding='utf-8')
    report['commands'][name] = {'exit_code':result.returncode,'output_characters':len(result.stdout+result.stderr)}
    assert result.returncode==0, result.stderr
for name, relative in (
    ('FAST-LIO2','dimos/hardware/sensors/lidar/fastlio2/cpp/result/bin/fastlio2_native'),
    ('RayTracingVoxelMap','target/release/voxel_ray_tracing')):
    path = repo/relative
    report['binaries'][name] = {'path':str(path),'exists':path.is_file(),'executable':os.access(path,os.X_OK)}
(out/'installation_check.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2),flush=True)
