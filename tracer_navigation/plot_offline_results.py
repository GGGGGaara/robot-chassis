"""Render recorded virtual-CAN results; never communicates with a robot."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np

ROOT = Path(__file__).resolve().parent


def main():
    report = json.loads((ROOT / 'closed_loop_vcan_report.json').read_text())
    trace = json.loads((ROOT / 'closed_loop_vcan_trace.json').read_text())
    assert report['simulation_only'] and report['virtual_can_driver']
    moving = next(p['t'] for p in trace if abs(p['v']) + abs(p['w']) > .001)
    # Include startup and two seconds of stopped output; omit the separate soak.
    finish = max(p['t'] for p in trace if abs(p['v']) + abs(p['w']) > 1e-6)
    rows = [p for p in trace if moving - 1 <= p['t'] <= finish + 2]
    t = np.array([p['t'] - moving for p in rows])
    xy = np.array([[p['x'], p['y']] for p in rows])
    v = np.array([p['v'] for p in rows])
    w = np.array([p['w'] for p in rows])
    goal = np.array(report['goal'])
    obstacle = np.array([p['phase'] == 'obstacle_stop' for p in rows])
    plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False})
    fig = plt.figure(figsize=(13, 7), layout='constrained')
    grid = fig.add_gridspec(2, 2, width_ratios=[1.1, 1])
    ax = fig.add_subplot(grid[:, 0])
    lines = (ROOT / 'generated/static_map.yaml').read_text().splitlines()
    cfg = dict(line.split(':', 1) for line in lines if ':' in line)
    image = plt.imread(ROOT / 'generated/static_map.pgm')
    origin_line = lines.index('origin:')
    res = float(cfg['resolution'])
    ox, oy = [float(line.removeprefix('- ')) for line in lines[origin_line+1:origin_line+3]]
    ax.imshow(image, cmap='gray', vmin=0, vmax=255,
              extent=[ox, ox + image.shape[1]*res, oy, oy + image.shape[0]*res], origin='upper')
    ax.plot(xy[:, 0], xy[:, 1], color='#1775bb', lw=2, label='Simulated chassis trajectory')
    ax.scatter(*report['start'], color='#148747', s=70, label='A: start', zorder=4)
    ax.scatter(*goal, marker='*', color='#e89016', s=170, label='B: selected goal', zorder=5)
    ax.scatter(*report['final'], marker='x', color='#111111', s=50, label='Final position', zorder=6)
    ax.add_patch(Circle(goal, .15, fill=False, color='#e89016', linestyle='--'))
    if obstacle.any():
        ax.scatter(xy[obstacle, 0], xy[obstacle, 1], color='#cc3d3d', s=18, label='Obstacle hold', zorder=5)
    ax.set(xlim=(xy[:, 0].min()-.6, goal[0]+.5), ylim=(xy[:, 1].min()-.5, goal[1]+.5),
           xlabel='Map x (m)', ylabel='Map y (m)', title='Fixed map: A to B')
    ax.set_aspect('equal'); ax.legend(loc='upper left', fontsize=8)
    speed = fig.add_subplot(grid[0, 1])
    speed.plot(t, v, color='#1775bb', label='Linear velocity (m/s)')
    speed.plot(t, w, color='#9463ad', label='Angular velocity (rad/s)')
    speed.axhline(.05, color='#1775bb', linestyle=':', alpha=.6)
    speed.set(ylabel='CAN-decoded command', xlabel='Time from first motion (s)', title='Stop for obstacle, then resume')
    distance = fig.add_subplot(grid[1, 1])
    distance.plot(t, np.linalg.norm(xy-goal, axis=1), color='#148747')
    distance.axhline(.15, color='#e89016', linestyle='--', label='Goal tolerance: 0.15 m')
    distance.set(xlabel='Time from first motion (s)', ylabel='Distance to B (m)', title=f'Final position error: {report["position_error_m"]:.3f} m')
    if obstacle.any():
        lo, hi = t[obstacle].min(), t[obstacle].max()
        for target in (speed, distance):
            target.axvspan(lo, hi, color='#cc3d3d', alpha=.15, label='Obstacle present')
    for target in (speed, distance):
        target.grid(alpha=.2); target.legend(fontsize=8)
    fig.suptitle('OFFLINE SIMULATION / VIRTUAL CAN — no physical robot motion', fontsize=15, fontweight='bold')
    soak=report.get('soak',{})
    fig.supxlabel(f'Actual Nav2 + actual CAN driver + ideal kinematic model | Task: {report["duration_s"]:.1f} s | Post-arrival soak: {soak.get("duration_s",0):g} s / {soak.get("successful_plans",0)} plans', fontsize=10)
    out = ROOT / 'results'; out.mkdir(exist_ok=True)
    for ext in ('png', 'svg'):
        fig.savefig(out / f'offline_navigation_results.{ext}', dpi=180, facecolor='white')
    print(str(out / 'offline_navigation_results.png'))


if __name__ == '__main__':
    main()
