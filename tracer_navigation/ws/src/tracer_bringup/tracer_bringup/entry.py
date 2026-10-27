import os
from pathlib import Path
import sys

def run(script):
    value=os.environ.get('TRACER_PROJECT_ROOT')
    if not value:raise RuntimeError('Set TRACER_PROJECT_ROOT to the tracer_navigation directory')
    root=Path(value).resolve();python=root/'.ros_venv/bin/python';source=root/script
    if not python.is_file() or not source.is_file():raise RuntimeError('Incomplete TRACER project: '+str(root))
    os.execv(str(python),[str(python),str(source),*sys.argv[1:]])

def mapping_backend():run('ros_mapping_backend.py')
def lio_bridge():run('dimos_ros_bridge.py')
def model_adapter():run('ros_mapping_bridge.py')
def model_recorder():run('ros_model_recorder.py')
def visual_demo():run('demo_visualization.py')
