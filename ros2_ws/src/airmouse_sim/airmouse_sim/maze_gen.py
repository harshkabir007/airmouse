#!/usr/bin/env python3
"""ASCII maze layout -> Gazebo Harmonic world + matching arena-grid parameters.

World frame == AirMouse map frame: the drone starts at the origin facing +x
(up the page in the layout), +y is left. One layout character is 0.5 m x scale.

CLI:  ros2 run airmouse_sim gen_maze mazes/maze_01.txt -o /tmp/maze_01.sdf
"""
import argparse
import math
import random

BASE_CELL = 0.5             # metres per layout character at scale 1
WALL_HEIGHT = 2.44          # 8 ft, per the NIDAR brief
BASE_GRID_CELL = 1.0        # rescue-grid cell size at scale 1
FUEL = 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/'
SURVIVOR_MODELS = {
    'P': FUEL + 'Standing person',
    'F': FUEL + 'Casual female',
    'R': FUEL + 'Rescue Randy Sitting',
}


def parse_layout(text):
    rows = [ln.rstrip('\r\n') for ln in text.splitlines()
            if ln.strip() and not ln.lstrip().startswith(';')]
    width = max(len(r) for r in rows)
    return [r.ljust(width, '.') for r in rows]


def wall_rectangles(rows):
    """Greedy merge of '#' cells into as few boxes as possible."""
    h, w = len(rows), len(rows[0])
    used = [[False] * w for _ in range(h)]
    rects = []
    for r in range(h):
        for c in range(w):
            if rows[r][c] != '#' or used[r][c]:
                continue
            c2 = c
            while c2 + 1 < w and rows[r][c2 + 1] == '#' and not used[r][c2 + 1]:
                c2 += 1
            r2 = r
            while r2 + 1 < h and all(rows[r2 + 1][k] == '#' and not used[r2 + 1][k] for k in range(c, c2 + 1)):
                r2 += 1
            for rr in range(r, r2 + 1):
                for cc in range(c, c2 + 1):
                    used[rr][cc] = True
            rects.append((r, c, r2, c2))
    return rects


def generate(text, name='maze', seed=0, ceiling=False, scale=1.0):
    """Returns (world_sdf, arena_params, survivors) for a layout string.

    `scale` multiplies all horizontal dimensions (corridors, rooms, wall thickness and the
    rescue grid); wall height is unchanged. Use it to keep a stand-in sim vehicle in the
    same size ratio to the corridors as the real drone."""
    CELL = BASE_CELL * scale
    GRID_CELL = BASE_GRID_CELL * scale
    rows = parse_layout(text)
    starts = [(r, c) for r, row in enumerate(rows) for c, ch in enumerate(row) if ch == 'S']
    walls = [(r, c) for r, row in enumerate(rows) for c, ch in enumerate(row) if ch == '#']
    if not starts:
        raise ValueError('layout needs at least one S (drone start) cell')
    if not walls:
        raise ValueError('layout has no walls')
    rs = sum(r for r, _ in starts) / len(starts)
    cs = sum(c for _, c in starts) / len(starts)

    def xy(r, c):
        return (rs - r) * CELL, (cs - c) * CELL

    r0, r1 = min(r for r, _ in walls), max(r for r, _ in walls)
    c0, c1 = min(c for _, c in walls), max(c for _, c in walls)
    arena = {
        'origin_x': (rs - r1) * CELL - CELL / 2,
        'origin_y': (cs - c0) * CELL + CELL / 2,
        'cell_size': GRID_CELL,
        'cols': int(round((c1 - c0 + 1) * CELL / GRID_CELL)),
        'rows': int(round((r1 - r0 + 1) * CELL / GRID_CELL)),
    }

    links = []
    for i, (ra, ca, rb, cb) in enumerate(wall_rectangles(rows)):
        x, y = xy((ra + rb) / 2, (ca + cb) / 2)
        sx, sy = (rb - ra + 1) * CELL, (cb - ca + 1) * CELL
        geom = f'<geometry><box><size>{sx:.3f} {sy:.3f} {WALL_HEIGHT}</size></box></geometry>'
        links.append(
            f'      <link name="wall_{i}"><pose>{x:.3f} {y:.3f} {WALL_HEIGHT / 2:.3f} 0 0 0</pose>\n'
            f'        <collision name="c">{geom}</collision>\n'
            f'        <visual name="v">{geom}<material><ambient>0.55 0.45 0.33 1</ambient>'
            f'<diffuse>0.72 0.6 0.45 1</diffuse></material></visual>\n'
            f'      </link>')
    if ceiling:
        x, y = xy((r0 + r1) / 2, (c0 + c1) / 2)
        sx, sy = (r1 - r0 + 1) * CELL, (c1 - c0 + 1) * CELL
        geom = f'<geometry><box><size>{sx:.3f} {sy:.3f} 0.02</size></box></geometry>'
        links.append(
            f'      <link name="ceiling_net"><pose>{x:.3f} {y:.3f} {WALL_HEIGHT + 0.01:.3f} 0 0 0</pose>\n'
            f'        <collision name="c">{geom}</collision>\n'
            f'        <visual name="v">{geom}<transparency>0.7</transparency></visual>\n'
            f'      </link>')

    rng = random.Random(seed)
    survivors = []
    includes = []
    for r, row in enumerate(rows):
        for c, ch in enumerate(row):
            if ch in SURVIVOR_MODELS:
                x, y = xy(r, c)
                yaw = rng.uniform(-math.pi, math.pi)
                n = len(survivors) + 1
                survivors.append({'id': n, 'type': ch, 'x': round(x, 2), 'y': round(y, 2)})
                includes.append(
                    f'    <include><uri>{SURVIVOR_MODELS[ch]}</uri><name>survivor_{n}</name>'
                    f'<pose>{x:.3f} {y:.3f} 0 0 0 {yaw:.3f}</pose><static>true</static></include>')

    walls_xml = '\n'.join(links)
    surv_xml = '\n'.join(includes)
    world = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="{name}">
    <physics name="1ms" type="ignore">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.75 0.8 0.85 1</background>
      <shadows>false</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>80 80</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>80 80</size></plane></geometry>
          <material><ambient>0.4 0.4 0.42 1</ambient><diffuse>0.5 0.5 0.52 1</diffuse></material></visual>
      </link>
    </model>

    <model name="maze">
      <static>true</static>
{walls_xml}
    </model>

{surv_xml}

    <include>
      <uri>model://airmouse_drone</uri>
      <name>airmouse</name>
      <pose>0 0 0.195 0 0 0</pose>
    </include>
  </world>
</sdf>
"""
    return world, arena, survivors


def arena_yaml(a):
    return ('/**:\n  ros__parameters:\n    arena:\n'
            f"      origin_x: {a['origin_x']:.3f}\n"
            f"      origin_y: {a['origin_y']:.3f}\n"
            f"      cell_size: {a['cell_size']:.3f}\n"
            f"      cols: {a['cols']}\n"
            f"      rows: {a['rows']}\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('layout')
    ap.add_argument('-o', '--output', default='maze.sdf')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--ceiling', action='store_true')
    ap.add_argument('--scale', type=float, default=1.0, help='horizontal scale (1.0 = 1 m corridors)')
    args = ap.parse_args()
    with open(args.layout) as f:
        world, arena, survivors = generate(f.read(), seed=args.seed, ceiling=args.ceiling, scale=args.scale)
    with open(args.output, 'w') as f:
        f.write(world)
    print(f'wrote {args.output}')
    print(arena_yaml(arena))
    for s in survivors:
        print(f"ground truth survivor {s['id']} ({s['type']}): x={s['x']} y={s['y']}")


if __name__ == '__main__':
    main()
