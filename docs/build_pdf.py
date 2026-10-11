#!/usr/bin/env python3
"""Builds docs/AirMouse_Project_Documentation.pdf.

    python3 docs/build_pdf.py        (needs python3-reportlab: sudo apt install python3-reportlab)

Edit the content below when the project changes, then re-run and commit the PDF.
"""
import os
from datetime import date

from reportlab.graphics.shapes import Drawing, Line, Polygon, Rect, String
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (Image, KeepTogether, PageBreak, Paragraph, Preformatted, SimpleDocTemplate,
                                Spacer, Table, TableStyle)

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'AirMouse_Project_Documentation.pdf')
REPO_URL = 'https://github.com/harshkabir007/airmouse'

# ---------------------------------------------------------------- fonts & styles
FONT_DIR = '/usr/share/fonts/truetype/dejavu'
try:
    pdfmetrics.registerFont(TTFont('DV', os.path.join(FONT_DIR, 'DejaVuSans.ttf')))
    pdfmetrics.registerFont(TTFont('DV-B', os.path.join(FONT_DIR, 'DejaVuSans-Bold.ttf')))
    pdfmetrics.registerFont(TTFont('DV-I', os.path.join(FONT_DIR, 'DejaVuSans-Oblique.ttf')))
    pdfmetrics.registerFont(TTFont('DVM', os.path.join(FONT_DIR, 'DejaVuSansMono.ttf')))
    pdfmetrics.registerFontFamily('DV', normal='DV', bold='DV-B', italic='DV-I', boldItalic='DV-B')
    BODY, BOLD, MONO = 'DV', 'DV-B', 'DVM'
except Exception:  # noqa: BLE001 - fall back to built-in fonts
    BODY, BOLD, MONO = 'Helvetica', 'Helvetica-Bold', 'Courier'

INK = colors.HexColor('#1f2933')
MUTED = colors.HexColor('#5f6b7a')
ACCENT = colors.HexColor('#2563eb')
OK = colors.HexColor('#15803d')
WARN = colors.HexColor('#b45309')
BAD = colors.HexColor('#b91c1c')
LINE = colors.HexColor('#d5dae1')
SOFT = colors.HexColor('#f3f5f8')
CODE_BG = colors.HexColor('#f6f8fa')

ss = getSampleStyleSheet()
S = {
    'title': ParagraphStyle('title', fontName=BOLD, fontSize=26, leading=32, textColor=INK, alignment=TA_CENTER),
    'subtitle': ParagraphStyle('subtitle', fontName=BODY, fontSize=13, leading=18, textColor=MUTED,
                               alignment=TA_CENTER),
    'h1': ParagraphStyle('h1', fontName=BOLD, fontSize=17, leading=22, textColor=INK, spaceBefore=6,
                         spaceAfter=8),
    'h2': ParagraphStyle('h2', fontName=BOLD, fontSize=12.5, leading=17, textColor=ACCENT, spaceBefore=10,
                         spaceAfter=4),
    'body': ParagraphStyle('body', fontName=BODY, fontSize=9.6, leading=14, textColor=INK, spaceAfter=5),
    'small': ParagraphStyle('small', fontName=BODY, fontSize=8.4, leading=11.5, textColor=MUTED),
    'cell': ParagraphStyle('cell', fontName=BODY, fontSize=8.6, leading=11.5, textColor=INK),
    'cellb': ParagraphStyle('cellb', fontName=BOLD, fontSize=8.6, leading=11.5, textColor=INK),
    'bullet': ParagraphStyle('bullet', fontName=BODY, fontSize=9.6, leading=14, textColor=INK, leftIndent=14,
                             bulletIndent=3, spaceAfter=2),
    'code': ParagraphStyle('code', fontName=MONO, fontSize=8.2, leading=11, textColor=INK),
    'callout': ParagraphStyle('callout', fontName=BODY, fontSize=9.4, leading=13.5, textColor=INK),
}


def P(text, style='body'):
    return Paragraph(text, S[style])


def H1(text):
    return P(text, 'h1')


def H2(text):
    return P(text, 'h2')


def bullets(items):
    return [Paragraph(t, S['bullet'], bulletText='•') for t in items]


_step = [0]


def steps(items, restart=False):
    """Numbered steps; numbering continues across code blocks until restart=True."""
    if restart:
        _step[0] = 0
    out = []
    for t in items:
        _step[0] += 1
        out.append(Paragraph(t, S['bullet'], bulletText=f'{_step[0]}.'))
    return out


def code(text):
    t = Table([[Preformatted(text.strip('\n'), S['code'])]], colWidths=[170 * mm])
    t.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, -1), CODE_BG),
        ('BOX', (0, 0), (-1, -1), 0.5, LINE),
        ('LEFTPADDING', (0, 0), (-1, -1), 7), ('RIGHTPADDING', (0, 0), (-1, -1), 7),
        ('TOPPADDING', (0, 0), (-1, -1), 5), ('BOTTOMPADDING', (0, 0), (-1, -1), 5),
    ]))
    return KeepTogether([t, Spacer(1, 5)])


def callout(text, color=ACCENT):
    t = Table([[P(text, 'callout')]], colWidths=[170 * mm])
    t.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, -1), SOFT),
        ('LINEBEFORE', (0, 0), (0, -1), 3, color),
        ('LEFTPADDING', (0, 0), (-1, -1), 9), ('RIGHTPADDING', (0, 0), (-1, -1), 8),
        ('TOPPADDING', (0, 0), (-1, -1), 6), ('BOTTOMPADDING', (0, 0), (-1, -1), 6),
    ]))
    return KeepTogether([t, Spacer(1, 6)])


def table(rows, widths, header=True, status_col=None):
    data = []
    for r, row in enumerate(rows):
        style = 'cellb' if header and r == 0 else 'cell'
        data.append([c if not isinstance(c, str) else P(c, style) for c in row])
    t = Table(data, colWidths=[w * mm for w in widths], repeatRows=1 if header else 0)
    st = [
        ('GRID', (0, 0), (-1, -1), 0.4, LINE),
        ('VALIGN', (0, 0), (-1, -1), 'TOP'),
        ('LEFTPADDING', (0, 0), (-1, -1), 5), ('RIGHTPADDING', (0, 0), (-1, -1), 5),
        ('TOPPADDING', (0, 0), (-1, -1), 3.5), ('BOTTOMPADDING', (0, 0), (-1, -1), 3.5),
    ]
    if header:
        st.append(('BACKGROUND', (0, 0), (-1, 0), SOFT))
    t.setStyle(TableStyle(st))
    return KeepTogether([t, Spacer(1, 7)]) if len(rows) < 14 else t


def status(word):
    color = {'Working': OK, 'Verified in sim': OK, 'Partial': WARN, 'Implemented, untested': WARN,
             'Not started': BAD, 'Untested': BAD}.get(word, INK)
    return Paragraph(f'<font color="{color.hexval()}"><b>{word}</b></font>', S['cell'])


# ---------------------------------------------------------------- diagram
def arrow(d, x1, y1, x2, y2, color=MUTED):
    import math
    d.add(Line(x1, y1, x2, y2, strokeColor=color, strokeWidth=1.1))
    a = math.atan2(y2 - y1, x2 - x1)
    s = 6
    d.add(Polygon([x2, y2, x2 - s * math.cos(a - 0.4), y2 - s * math.sin(a - 0.4),
                   x2 - s * math.cos(a + 0.4), y2 - s * math.sin(a + 0.4)],
                  fillColor=color, strokeColor=color))


def box(d, x, y, w, h, title, sub='', fill=colors.white, stroke=ACCENT):
    d.add(Rect(x, y, w, h, rx=5, ry=5, fillColor=fill, strokeColor=stroke, strokeWidth=1))
    d.add(String(x + w / 2, y + h / 2 + (3 if sub else -3), title, fontName=BOLD, fontSize=8.5,
                 fillColor=INK, textAnchor='middle'))
    if sub:
        d.add(String(x + w / 2, y + h / 2 - 9, sub, fontName=BODY, fontSize=6.8, fillColor=MUTED,
                     textAnchor='middle'))


def architecture():
    W, H = 480, 330
    d = Drawing(W, H)
    d.add(Rect(150, 70, 330, 255, rx=8, ry=8, fillColor=colors.HexColor('#eef3fd'), strokeColor=None))
    d.add(String(160, 312, 'Raspberry Pi 5  ·  Docker  ·  ROS 2 Jazzy', fontName=BOLD, fontSize=8,
                 fillColor=ACCENT))
    bw, bh = 140, 38
    c0, c1, c2 = 0, 165, 330
    r0, r1, r2, r3 = 262, 190, 118, 12
    sensor = colors.HexColor('#fff7ed')
    ext = colors.HexColor('#f0fdf4')
    box(d, c0, r0, bw, bh, 'Livox Mid-360', '3D LiDAR + IMU (inverted)', sensor, WARN)
    box(d, c0, r1, bw, bh, 'USB camera', 'forward, 15° down', sensor, WARN)
    box(d, c1, r0, bw, bh, 'FAST-LIO2', 'LiDAR-inertial SLAM')
    box(d, c2, r0, bw, bh, 'pose_bridge', 'map frame · vision pose')
    box(d, c1, r1, bw, bh, 'survivor_detector', 'YOLO11n + LiDAR depth')
    box(d, c2, r1, bw, bh, 'mapper', '2D occupancy grid')
    box(d, c1, r2, bw, bh, 'gcs_server', 'web dashboard :8080')
    box(d, c2, r2, bw, bh, 'commander', 'state machine · frontiers')
    box(d, c1, r3, bw, bh, 'GCS laptop', 'browser + Mission Planner', ext, OK)
    box(d, c2, r3, bw, bh, 'ArduPilot FC', 'EKF3 ExternalNav (via MAVROS)', ext, OK)
    m = bh / 2
    arrow(d, c0 + bw, r0 + m, c1, r0 + m)
    arrow(d, c1 + bw, r0 + m, c2, r0 + m)
    arrow(d, c0 + bw, r1 + m, c1, r1 + m)
    arrow(d, c2 + bw / 2, r0, c2 + bw / 2, r1 + bh)
    arrow(d, c2 + bw / 2, r1, c2 + bw / 2, r2 + bh)
    arrow(d, c2 + bw / 2, r2, c2 + bw / 2, r3 + bh)
    arrow(d, c1 + bw / 2, r1, c1 + bw / 2, r2 + bh)
    arrow(d, c2, r2 + m, c1 + bw, r2 + m)
    arrow(d, c1 + bw / 2, r2, c1 + bw / 2, r3 + bh)
    # vision pose straight to the FC along the right edge
    d.add(Line(c2 + bw, r0 + m, c2 + bw + 8, r0 + m, strokeColor=OK, strokeWidth=1.1))
    d.add(Line(c2 + bw + 8, r0 + m, c2 + bw + 8, r3 + m, strokeColor=OK, strokeWidth=1.1))
    arrow(d, c2 + bw + 8, r3 + m, c2 + bw, r3 + m, OK)
    d.add(String(c2 + bw + 4, r1 - 8, 'vision', fontName=BODY, fontSize=6.5, fillColor=OK, textAnchor='end'))
    d.add(String(c1 + bw / 2 + 4, r2 - 10, 'local Wi-Fi', fontName=BODY, fontSize=6.5, fillColor=MUTED))
    return d


# ---------------------------------------------------------------- page decoration
def on_page(canvas, doc):
    canvas.saveState()
    if doc.page > 1:
        canvas.setFont(BODY, 7.5)
        canvas.setFillColor(MUTED)
        canvas.drawString(20 * mm, 287 * mm, 'AirMouse · NIDAR GPS-denied indoor search drone')
        canvas.drawRightString(190 * mm, 287 * mm, f'Project documentation · {date.today():%d %b %Y}')
        canvas.setStrokeColor(LINE)
        canvas.line(20 * mm, 285 * mm, 190 * mm, 285 * mm)
        canvas.drawCentredString(105 * mm, 10 * mm, str(doc.page))
    canvas.restoreState()


# ---------------------------------------------------------------- content
def build():
    st = []

    # ---- cover
    st += [Spacer(1, 55 * mm), P('AirMouse', 'title'), Spacer(1, 4),
           P('Autonomous GPS-denied indoor search, mapping and survivor localisation drone', 'subtitle'),
           Spacer(1, 3), P('NIDAR Track 1 · Drone Innovation', 'subtitle'), Spacer(1, 18 * mm)]
    cover = [
        ['Document', 'Project overview, tech stack, status, and how to install and run it'],
        ['Repository', REPO_URL],
        ['Version', f'main @ b1246f5 · {date.today():%d %B %Y}'],
        ['Status', 'Simulation: full autonomous mission working. Hardware: not started.'],
    ]
    st.append(table(cover, [32, 118], header=False))
    st.append(PageBreak())

    # ---- contents
    st.append(H1('Contents'))
    toc = ['1. Project overview', '2. System architecture', '3. Tech stack', '4. Software components',
           '5. Frames, map and rescue grid', '6. What works today (status)', '7. Simulation results',
           '8. Problems found and fixed during bring-up', '9. How to install (new machine)',
           '10. How to run the simulation', '11. Using the ground control station',
           '12. Running on the real drone (Raspberry Pi 5)', '13. Configuration reference',
           '14. Troubleshooting', '15. Repository layout', '16. Roadmap and next steps',
           'Appendix: command cheat sheet']
    st += [P(t) for t in toc]
    st.append(PageBreak())

    # ---- 1 overview
    st.append(H1('1. Project overview'))
    st.append(P('NIDAR AirMouse simulates a collapsed building: a covered, maze-like arena of up to '
                '15 m × 15 m with 1 m corridors, 2 m × 2 m rooms and 8 ft (2.44 m) walls. A drone has to enter '
                'through one entry point with <b>no GPS</b>, explore the maze, find up to <b>6 survivors</b>, '
                'build a <b>live 2D map</b> on the ground control station (GCS), tag the <b>grid cell</b> of each '
                'survivor, and leave through the same point within <b>30 minutes</b>. The operator may only '
                'start the mission and abort it. Everything else must be autonomous.'))
    st.append(P('AirMouse is our answer. The drone localises itself with a 3D LiDAR and IMU (LiDAR SLAM), '
                'and feeds that position to the flight controller in place of GPS. It explores with a frontier '
                'planner, detects people with a neural network on a camera, places them on the map using the '
                'LiDAR, and streams the map, survivor tags and video to a web dashboard over a local Wi-Fi link.'))
    st.append(H2('Design goals'))
    st += bullets([
        '<b>Same code in simulation and on the drone.</b> Gazebo and ArduPilot SITL replace the sensors and the '
        'flight controller; every autonomy node is unchanged.',
        '<b>Portable.</b> Everything runs in Docker. A new laptop needs only Docker (plus WSL2 on Windows) and '
        'a <font name="%s">git clone</font>.' % MONO,
        '<b>Safe by default.</b> Every failsafe ends in LAND. The drone never does a straight-line "return home" '
        'that would fly through walls.',
        '<b>Rule-compliant.</b> No GPS, no internet, no manual control: the operator only has START, RECALL and '
        'ABORT.',
    ])

    # ---- 2 architecture
    st.append(PageBreak())
    st.append(H1('2. System architecture'))
    st.append(P('The drone carries a Raspberry Pi 5 running ROS 2 in Docker. The LiDAR data goes through '
                'FAST-LIO2 to get the drone pose. That pose goes to the flight controller (as an external '
                'navigation source for its EKF3 position estimator) and to the mapping, planning and detection '
                'nodes. The web GCS is served directly from the drone.'))
    st.append(architecture())
    st.append(P('Data flow of the autonomy stack on the drone. In simulation, Gazebo provides the LiDAR, IMU '
                'and camera, and ArduPilot SITL replaces the flight controller.', 'small'))
    st.append(H2('Mission state machine'))
    st.append(code('IDLE -> PREFLIGHT -> ARMING -> TAKEOFF -> EXPLORE -> RETURN -> LANDING -> DONE\n'
                   '                         any flying state -> LANDING   (failsafe or ABORT)\n'
                   '                         EXPLORE          -> RETURN    (RECALL, time budget, low battery)'))
    st += bullets([
        '<b>PREFLIGHT</b>: wait for the flight-controller link and healthy SLAM, set the EKF origin, switch to '
        'GUIDED mode.',
        '<b>EXPLORE</b>: every second, pick the best frontier (boundary between known free space and unknown '
        'space inside the arena), plan a path on the map, and follow it. At each frontier the drone does a 360° '
        'yaw sweep so the camera sees the room.',
        '<b>RETURN</b>: plan a path back to the entry point through the mapped corridors, then land.',
    ])

    # ---- 3 tech stack
    st.append(PageBreak())
    st.append(H1('3. Tech stack'))
    rows = [
        ['Layer', 'Choice', 'Why'],
        ['Flight controller firmware', 'ArduPilot ArduCopter 4.6.3', 'Mission Planner support; mature EKF3 '
         '"ExternalNav" mode that accepts a SLAM pose instead of GPS.'],
        ['SLAM', 'FAST-LIO2 (hku-mars, ROS 2 branch)', 'Tightly coupled LiDAR + IMU, robust to fast motion, '
         'runs well on ARM. Standard choice for Livox drones.'],
        ['LiDAR', 'Livox Mid-360, mounted upside-down', '360° × 59° field of view, built-in IMU, 265 g. Inverted, '
         'it sees the floor, so it also sees survivors lying down.'],
        ['Companion computer', 'Raspberry Pi 5 (16 GB)', 'Enough CPU for FAST-LIO, mapping, planning and a small '
         'detector at a few Hz.'],
        ['Middleware', 'ROS 2 Jazzy (Ubuntu 24.04)', 'Long-term support release; native Gazebo Harmonic pairing.'],
        ['FC link', 'MAVROS (MAVLink)', 'Vision pose in, setpoints / mode / arming out.'],
        ['Simulation', 'Gazebo Harmonic + ArduPilot SITL', 'SITL runs the real ArduPilot code, so EKF and '
         'failsafe behaviour transfers to the real drone.'],
        ['Person detection', 'YOLO11n (Ultralytics), CPU', 'Small and fast; can be exported to NCNN for the Pi.'],
        ['Autonomy nodes', 'Python 3 (rclpy, NumPy, SciPy, OpenCV)', 'Easy to read and change; heavy lifting '
         'is vectorised.'],
        ['Ground control', 'Web dashboard (no internet needed) + Mission Planner', 'Live map, grid, survivor '
         'tags, video and buttons in any browser.'],
        ['Packaging', 'Docker + docker compose + VS Code dev container', 'Identical environment on every laptop '
         'and on the Pi.'],
        ['Dev environment', 'Windows 11 + WSL2 (Ubuntu 24.04), or native Linux', 'Tested on Windows 11 with '
         'WSL2.'],
    ]
    st.append(table(rows, [36, 50, 84]))

    # ---- 4 components
    st.append(H1('4. Software components'))
    st.append(P('All AirMouse code is in <font name="%s">ros2_ws/src/</font>, split into three ROS 2 packages.'
                % MONO))
    rows = [
        ['Node (package)', 'What it does', 'Main topics'],
        ['pose_bridge (airmouse)', 'Turns FAST-LIO output into the drone-centred "map" frame (handles the '
         'upside-down LiDAR). Sends the pose to ArduPilot. Flags SLAM as unhealthy if it goes stale or jumps.',
         'in: /Odometry, /cloud_registered<br/>out: /airmouse/odom, /airmouse/cloud, '
         '/mavros/vision_pose/pose, /airmouse/slam_ok'],
        ['mapper (airmouse)', 'Builds the live 2D occupancy grid (5 cm cells) from the 3D scans: slices a height '
         'band, marks walls, ray-traces free space, locks confirmed walls.', 'out: /airmouse/map'],
        ['commander (airmouse)', 'Mission state machine, frontier exploration, path planning, carrot path '
         'following, safe yaw handling, return to exit, all failsafes.',
         'out: /mavros/setpoint_position/local, /airmouse/status, /airmouse/path'],
        ['survivor_detector (airmouse)', 'YOLO person detection on the camera. Localises each detection with '
         'LiDAR points inside the box (or a floor-plane ray), fuses repeated sightings, tags the grid cell.',
         'out: /airmouse/survivors, /airmouse/detections'],
        ['gcs_server (airmouse)', 'Web dashboard and HTTP API: map image, state JSON, MJPEG video, START / '
         'RECALL / ABORT.', 'http://&lt;drone&gt;:8080'],
        ['lidar_filter (airmouse_sim)', 'Simulation only: removes "no hit" points and adds exact point timing '
         'to Gazebo LiDAR scans so FAST-LIO works correctly.', 'in: /lidar/points<br/>out: /lidar/points_clean'],
        ['maze_gen, drone_model (airmouse_sim)', 'Turn an ASCII maze drawing into a Gazebo world, and add the '
         'LiDAR, IMU and camera to ArduPilot\'s simulated quad.', '-'],
        ['launch files (airmouse_bringup)', 'sim.launch.py (whole simulation), robot.launch.py (real drone), '
         'core.launch.py (shared nodes). All parameters live in config/.', '-'],
    ]
    st.append(table(rows, [36, 82, 52]))

    # ---- 5 frames
    st.append(H1('5. Frames, map and rescue grid'))
    st += bullets([
        '<b>Map frame</b> = the drone at take-off. +x points into the maze from the entry (shown as "up" on the '
        'GCS), +y to the left, z up. ArduPilot\'s local frame is the same frame.',
        '<b>Rescue grid</b> (configurable): the origin is the left-hand corner of the entry wall, seen when '
        'facing into the maze. Columns are letters (A, B, C …) left to right; rows are numbers (1, 2, 3 …) from '
        'the entry wall inward. Default cell size is 1 m. So "A1" is the cell at the entry wall on your left.',
        'On competition day, set <font name="%s">arena.origin_x / origin_y</font> from where the launch box is, '
        'and change the cell size and labelling to match the organisers\' grid.' % MONO,
    ])

    # ---- 6 status
    st.append(PageBreak())
    st.append(H1('6. What works today (status)'))
    st.append(callout('<b>Summary.</b> The complete mission works autonomously in simulation: take-off, '
                      'exploration of the whole maze, live map, survivor tagging, return and landing at the '
                      'exit. It has been tested on one laptop with one maze layout and is not yet reliable '
                      'enough to call finished. Nothing has been tested on real hardware yet.', OK))
    rows = [
        ['Feature', 'Status', 'Notes'],
        ['Docker simulation image (laptop, amd64)', status('Working'), 'Builds from scratch; about 12 GB.'],
        ['Gazebo maze world from ASCII layout', status('Working'), 'Includes 6 survivor models.'],
        ['Simulated drone with LiDAR, IMU, camera', status('Working'), 'ArduPilot iris + injected sensors.'],
        ['FAST-LIO2 SLAM in simulation', status('Working'), 'Stable over a 13.5 min flight after the timing fix.'],
        ['ArduPilot EKF3 using SLAM instead of GPS', status('Working'), 'Vision pose and EKF agree to a few cm.'],
        ['Autonomous preflight, arming, take-off', status('Working'), 'GUIDED mode, 1.2 m altitude.'],
        ['Live 2D occupancy map', status('Working'), 'All 16 rooms and corridors clearly visible.'],
        ['Frontier exploration of the full maze', status('Working'), 'Covered the whole maze; some areas '
         'visited twice. One frontier was skipped after a timeout.'],
        ['Survivor detection and grid tagging', status('Partial'), '5/6 in the best complete run, 6/6 in an '
         'earlier run; positions within about 0.3 m.'],
        ['Return to exit and landing', status('Working'), 'Landed within 7 cm of the start point.'],
        ['Failsafe: SLAM lost → land', status('Verified in sim'), 'Triggered during earlier crashes.'],
        ['Failsafe: geofence → land', status('Verified in sim'), 'Triggered when SLAM diverged.'],
        ['Failsafe: low battery, time budget, abort, recall', status('Implemented, untested'),
         'Code paths exist; not yet exercised in a test.'],
        ['Web GCS dashboard', status('Partial'), 'Server, map and state API verified over HTTP; the browser '
         'page and video stream still need a visual check.'],
        ['Mission Planner connection', status('Implemented, untested'), 'TCP 5762 in sim / UDP 14550 on drone.'],
        ['Robot image on Raspberry Pi 5 (arm64)', status('Untested'), 'Same Dockerfile; never built on a Pi.'],
        ['Livox Mid-360 driver, USB camera on the Pi', status('Untested'), 'Configs written; no hardware yet.'],
        ['Real flight', status('Not started'), 'Phases 6–8 of the implementation plan.'],
    ]
    st.append(table(rows, [62, 34, 74]))

    # ---- 7 results
    st.append(PageBreak())
    st.append(H1('7. Simulation results'))
    st.append(P('Best complete run (maze_01, default 1.3× maze scale, Windows 11 laptop with Intel Iris Xe '
                'graphics, simulation running at about 0.7× real time):'))
    rows = [
        ['Metric', 'Result'],
        ['Mission outcome', 'Fully autonomous: START → explore → return → landed at exit'],
        ['Mission time (sim time)', '13.5 min of the 30 min limit (exploration done at 12.3 min)'],
        ['Area mapped', 'about 197 m² of free space inside the arena'],
        ['Survivors tagged', '5 of 6, all in the correct grid cell (F2, C6, J10, L15, M6)'],
        ['Survivor position error', 'about 0.1–0.3 m against Gazebo ground truth'],
        ['Landing accuracy', '7 cm from the take-off point'],
        ['Crashes', 'None in this run (earlier runs crashed; see section 8)'],
    ]
    st.append(table(rows, [50, 120]))
    img = os.path.join(HERE, 'img', 'sim_map_maze01.png')
    if os.path.exists(img):
        im = Image(img, width=95 * mm, height=95 * mm * 490 / 470)
        cap = P('The live 2D map exactly as the GCS showed it at the end of that run. White is free space, '
                'dark grey is unknown, black is walls. The entry is at the bottom. Small dark blobs inside rooms '
                'are the survivors.', 'small')
        st.append(KeepTogether([Table([[im]], colWidths=[170 * mm], style=[('ALIGN', (0, 0), (-1, -1), 'CENTER')]),
                                Spacer(1, 3), cap]))

    # ---- 8 lessons
    st.append(PageBreak())
    st.append(H1('8. Problems found and fixed during bring-up'))
    st.append(P('Every crash in testing was traced in flight logs to a root cause before fixing it. These are '
                'worth knowing because several will matter on the real drone too.'))
    rows = [
        ['Problem', 'Root cause', 'Fix'],
        ['FAST-LIO and Livox driver did not compile', 'Code written for older ROS 2 / C++14',
         'Build script switches to the Jazzy code path and C++17; versions pinned.'],
        ['Gazebo picked the wrong drone model', 'model://airmouse matched the repo folder',
         'Generated model renamed to airmouse_drone.'],
        ['FAST-LIO never started (sim)', 'Gazebo reports missed beams as "inf" points', 'lidar_filter node '
         'removes them.'],
        ['SLAM froze right after take-off (sim)', 'No per-point time in Gazebo scans; FAST-LIO\'s guess ran away',
         'lidar_filter adds exact capture times.'],
        ['Drone flipped while turning in a doorway', 'Fast 90° yaw in a 1 m door caused drift into the wall',
         'Ramped yaw; turn on the spot only where clearance allows.'],
        ['Drone chased "frontiers" on walls', 'Unseen insides of thick walls look like unexplored space',
         'Frontier cells must not touch walls.'],
        ['Drone hit door frames', 'Simulated iris is about 1.5× wider than our airframe',
         'Sim maze scaled 1.3× (maze_scale) to keep the right proportions.'],
        ['Drone clipped wall corners', 'Grazing LiDAR rays slowly erased thin corners from the map',
         'Confirmed wall cells are locked.'],
        ['Drone drifted off its path through doors', 'Far waypoints let small errors carry diagonally',
         'Carrot-following along the planned segment.'],
        ['Same survivor tagged twice', 'First far-away sightings landed >1 m apart', 'Tracks fused when they '
         'converge.'],
        ['Commander kept "exploring" after a crash', 'Flight controller disarmed but mode stayed GUIDED',
         'Abort when disarmed in flight.'],
        ['Simulation ran at 10% speed', 'WSL GPU translation is slow for sensor rendering',
         'Software rendering (llvmpipe) by default: about 0.7× real time.'],
    ]
    st.append(table(rows, [52, 58, 60]))

    # ---- 9 install
    st.append(PageBreak())
    st.append(H1('9. How to install (new machine)'))
    st.append(P('You need about 30 GB of free disk, 16 GB of RAM (recommended), and an internet connection '
                'for the first build (30–60 min). After that, the simulation runs offline except for the '
                'first download of the survivor models.'))
    st.append(H2('Windows 11 (tested)'))
    st += steps([
        'Open <b>PowerShell as Administrator</b> and install WSL2 with Ubuntu 24.04. Reboot if asked.',
    ], restart=True)
    st.append(code('wsl --install -d Ubuntu-24.04'))
    st += steps([
        'Give WSL enough memory for the builds. Create the file <font name="%s">C:\\Users\\&lt;you&gt;\\'
        '.wslconfig</font> with this content, then run <font name="%s">wsl --shutdown</font>.' % (MONO, MONO),
    ])
    st.append(code('[wsl2]\nmemory=10GB\nswap=8GB\nprocessors=12'))
    st += steps([
        'Open the <b>Ubuntu</b> app from the Start menu. On first launch, create a Linux username and password.',
        'Install Docker inside Ubuntu, then close and reopen the Ubuntu window:',
    ])
    st.append(code('curl -fsSL https://get.docker.com | sudo sh\nsudo usermod -aG docker $USER'))
    st += steps([
        'Clone the project <b>inside the Linux home folder</b>, not under /mnt/c or OneDrive (a Windows path '
        'makes builds slow and breaks file links):',
    ])
    st.append(code('cd ~\ngit clone %s\ncd airmouse' % REPO_URL))
    st += steps(['Build the simulation image (30–60 minutes the first time):'])
    st.append(code('docker compose build sim'))
    st.append(H2('Linux (Ubuntu 22.04 / 24.04)'))
    st.append(code('curl -fsSL https://get.docker.com | sudo sh\nsudo usermod -aG docker $USER   # then log out '
                   'and in\ngit clone %s && cd airmouse\ndocker compose build sim' % REPO_URL))
    st.append(callout('<b>Sharing a ready-made image.</b> To skip the long build on another laptop, export it '
                      'once with <font name="%s">docker save airmouse:sim | gzip &gt; airmouse_sim.tar.gz</font> '
                      'and import it there with <font name="%s">docker load &lt; airmouse_sim.tar.gz</font>.'
                      % (MONO, MONO)))

    # ---- 10 run sim
    st.append(PageBreak())
    st.append(H1('10. How to run the simulation'))
    st.append(callout('<b>Windows:</b> keep an Ubuntu window open the whole time. WSL shuts Ubuntu down, '
                      'and the simulation with it, when no Ubuntu window is attached.', WARN))
    st += steps([
        '<b>Start the container</b> (from the airmouse folder in Ubuntu). On Windows use the WSL override, '
        'which enables Gazebo windows and fast rendering:',
    ], restart=True)
    st.append(code('# Windows (WSL)\ndocker compose -f compose.yaml -f docker/compose.wsl.yaml up -d sim\n\n'
                   '# Linux\nxhost +local:docker\ndocker compose up -d sim'))
    st += steps(['<b>Open a shell inside the container.</b> You can open as many as you like this way:'])
    st.append(code('docker compose exec sim bash'))
    st += steps(['<b>Build the AirMouse code</b> (needed once, and again after you change code):'])
    st.append(code('am-build'))
    st += steps(['<b>Launch the full simulation</b> (Gazebo, ArduPilot SITL, FAST-LIO, MAVROS and all AirMouse '
                 'nodes). Wait about one minute until the log settles.'])
    st.append(code('am-sim                                                      # with Gazebo window\n'
                   'ros2 launch airmouse_bringup sim.launch.py headless:=true   # faster, no window'))
    st += steps([
        '<b>Open the dashboard</b> in a browser on your laptop: <font name="%s">http://localhost:8080</font>'
        % MONO,
        '<b>Press START.</b> The drone takes off, explores, tags survivors and lands at the exit by itself. '
        'Use <b>RECALL</b> to send it home early, or <b>ABORT / LAND</b> to land immediately.',
        '<b>Stop</b> with Ctrl+C in the launch shell. To shut the container down: '
        '<font name="%s">docker compose down</font>.' % MONO,
    ])
    st.append(H2('Useful options'))
    rows = [
        ['Option', 'Effect'],
        ['maze:=maze_01', 'Which layout to load (name in airmouse_sim/mazes, or a path to a .txt file).'],
        ['maze_scale:=1.3', 'Horizontal maze scale. Default 1.3 keeps the oversized sim drone in proportion; '
         'use 1.0 for exact competition dimensions.'],
        ['headless:=true', 'No Gazebo window; uses less CPU/GPU.'],
        ['am-start / am-abort', 'Send START / ABORT from a terminal instead of the dashboard.'],
    ]
    st.append(table(rows, [42, 128]))
    st.append(H2('Making your own maze'))
    st.append(P('Copy <font name="%s">ros2_ws/src/airmouse_sim/mazes/maze_01.txt</font> and edit it. Each '
                'character is 0.5 m (times maze_scale): <b>#</b> wall, <b>.</b> free, <b>S</b> drone start (the '
                'drone faces up the page), <b>P</b>/<b>F</b>/<b>R</b> survivors (standing person, woman, sitting '
                'dummy). Then run <font name="%s">am-sim maze:=/airmouse/my_maze.txt</font>. The launch log '
                'prints the true survivor positions so you can check the results.' % (MONO, MONO)))

    # ---- 11 GCS
    st.append(PageBreak())
    st.append(H1('11. Using the ground control station'))
    st += bullets([
        '<b>Top bar</b>: mission state, mission timer, flight mode, battery, SLAM health, explored area, number '
        'of survivors, and the latest status message.',
        '<b>Map (left)</b>: the live 2D map with the rescue grid (letters along the entry wall, numbers up the '
        'side), the drone (blue triangle), the planned path (dashed), the entry/exit (green square) and '
        'survivors (red dots with their grid cell; their cell is shaded).',
        '<b>Live camera (right)</b>: camera feed with person detections boxed in red.',
        '<b>Survivors table</b>: number, grid cell, position in metres and detection confidence.',
        '<b>Buttons</b>: START (only in IDLE), RECALL (only while exploring), ABORT / LAND (always; asks for '
        'confirmation). These are the only operator inputs, as the rules require.',
        '<b>Mission Planner</b> (optional): in sim connect TCP to 127.0.0.1 port 5762; on the drone listen on '
        'UDP 14550. Each confirmed survivor also appears there as a "SURVIVOR n at Xn" message.',
    ])

    # ---- 12 real drone
    st.append(H1('12. Running on the real drone (Raspberry Pi 5)'))
    st.append(callout('These steps are written but have <b>not been tested on hardware yet</b>. Expect to fix '
                      'small issues the first time.', WARN))
    st += steps([
        'Flash <b>Raspberry Pi OS Lite 64-bit</b>, enable SSH, and install Docker '
        '(<font name="%s">curl -fsSL https://get.docker.com | sh</font>).' % MONO,
    ], restart=True)
    st += steps([
        'Wiring: Mid-360 Ethernet → Pi Ethernet (give the Pi the static IP 192.168.1.50/24); flight controller '
        'TELEM2 → Pi UART (GPIO 14/15, /dev/ttyAMA0, 921600 baud); USB camera → Pi USB; Pi powered from a '
        '5 V / 5 A BEC.',
        'Set the LiDAR IP (192.168.1.1XX, where XX = last two digits of its serial number) in '
        '<font name="%s">config/livox/MID360_config.json</font>.' % MONO,
        'Load <font name="%s">config/ardupilot/real.parm</font> into the flight controller with Mission Planner '
        '(Full Parameter List → Load from file → Write), then reboot it. Set battery voltages and RC switches '
        'for your build.' % MONO,
        'Measure and fill in every value marked <b>MEASURE</b> in <font name="%s">config/airmouse.yaml</font> '
        '(LiDAR and camera mounting, floor height, turn clearance).' % MONO,
        'On the Pi, build and start the robot container:',
    ])
    st.append(code('git clone %s && cd airmouse\ndocker compose build robot      # about 1 hour on the Pi\n'
                   'docker compose up -d robot       # restarts automatically on boot\n'
                   'docker compose exec robot bash\nam-build\n'
                   'yolo export model=/opt/models/yolo11n.pt format=ncnn imgsz=416   # faster detector\n'
                   'am-robot' % REPO_URL))
    st += steps(['Join the GCS laptop to the drone\'s Wi-Fi and open <font name="%s">http://&lt;pi-ip&gt;:8080'
                 '</font>. Test with props off first, then tethered, before any free flight.' % MONO])

    # ---- 13 config
    st.append(PageBreak())
    st.append(H1('13. Configuration reference'))
    st.append(P('All settings are in <font name="%s">ros2_ws/src/airmouse_bringup/config/</font>. '
                '<b>airmouse.yaml</b> is for the real drone; <b>sim.yaml</b> overrides it in simulation.' % MONO))
    rows = [
        ['Parameter', 'Default', 'Meaning'],
        ['arena.origin_x / origin_y', '1.0 / 7.5 m', 'Where the rescue grid starts, relative to the drone start.'],
        ['arena.cell_size, cols, rows', '1.0 m, 15, 15', 'Rescue grid size.'],
        ['commander.takeoff_alt', '1.2 m', 'Flight altitude above the take-off point.'],
        ['commander.inflation_radius', '0.30 m (sim 0.42)', 'How far the drone centre stays from walls.'],
        ['commander.turn_clearance', '0.35 m (sim 0.48)', 'Minimum wall clearance before the drone may yaw.'],
        ['commander.spin_at_goals', 'true', '360° camera sweep at each frontier.'],
        ['commander.max_explore_time_s', '1260 s', 'Then return to the exit (30 min limit).'],
        ['commander.hard_time_limit_s', '1680 s', 'Land wherever the drone is.'],
        ['commander.battery_return / land', '30% / 18%', 'Battery levels for return and land.'],
        ['survivor_detector.confidence', '0.45', 'YOLO score threshold.'],
        ['survivor_detector.confirm_hits', '3', 'Sightings needed before a survivor is tagged.'],
        ['pose_bridge.lidar_mount_*', 'inverted, 8 cm below', 'LiDAR position and orientation on the drone.'],
        ['ArduPilot WPNAV_SPEED', '70 cm/s', 'Cruise speed (sitl.parm / real.parm).'],
    ]
    st.append(table(rows, [55, 32, 83]))

    # ---- 14 troubleshooting
    st.append(H1('14. Troubleshooting'))
    rows = [
        ['Symptom', 'What to do'],
        ['Simulation stops by itself on Windows', 'Keep an Ubuntu window open while it runs.'],
        ['Sim runs very slowly', 'Use the WSL compose override (software rendering). Use headless:=true. Close '
         'other heavy apps. Around 0.7× real time is normal on a laptop.'],
        ['"service sim is not running"', 'Start it: docker compose -f compose.yaml -f docker/compose.wsl.yaml '
         'up -d sim'],
        ['Dashboard says "waiting for SLAM"', 'Check: ros2 topic hz /Odometry. If it is empty, look for '
         'fast_lio errors in the launch log.'],
        ['PreArm messages in the log while IDLE', 'Normal ("waiting for home"). The commander sets the EKF '
         'origin after you press START.'],
        ['Drone lands with "SLAM lost" or "geofence"', 'A safety failsafe fired. Check the log just before it '
         'for the reason (often a collision).'],
        ['Port 8080 or 5762 already in use', 'An old run is still alive: docker compose restart sim'],
        ['Code changes have no effect', 'Run am-build again in the container, then relaunch.'],
        ['Build fails on a new laptop', 'Check disk space and WSL memory (.wslconfig), then rebuild with '
         'docker compose build sim.'],
    ]
    st.append(table(rows, [58, 112]))

    # ---- 15 layout
    st.append(PageBreak())
    st.append(H1('15. Repository layout'))
    st.append(code(
        'airmouse/\n'
        '├─ compose.yaml                 docker services: sim (laptop) and robot (Pi 5)\n'
        '├─ docker/                      Dockerfile, WSL / NVIDIA overrides, shell env\n'
        '├─ deps.repos                   pinned FAST-LIO and Livox driver versions\n'
        '├─ scripts/                     build_deps.sh, build_ws.sh\n'
        '├─ docs/                        implementation plan, this PDF and its generator\n'
        '└─ ros2_ws/src/\n'
        '   ├─ airmouse/                 autonomy nodes (Python) + web/index.html\n'
        '   │  └─ airmouse/              pose_bridge, mapper, commander, survivor_detector,\n'
        '   │                            gcs_server, planning, arena, geometry, detectors\n'
        '   ├─ airmouse_sim/             maze generator, drone sensor model, lidar_filter, mazes/\n'
        '   └─ airmouse_bringup/         launch/ (sim, robot, core) and config/ (all parameters)'))
    st.append(H2('Working with git'))
    st.append(code('cd ~/airmouse\ngit pull                       # get teammates\' changes\n'
                   'git add -A && git commit -m "describe the change"\ngit push'))

    # ---- 16 roadmap
    st.append(H1('16. Roadmap and next steps'))
    rows = [
        ['Step', 'Goal'],
        ['Simulation reliability', 'Run 10+ missions on several new layouts; aim for 6/6 survivors and no '
         'crashes. Add a batch script that runs headless missions and scores them.'],
        ['Frontier retry', 'Retry skipped frontiers before returning, so no room is missed.'],
        ['Speed', 'Cut exploration time: fewer revisits, skip spins in rooms already seen, raise cruise speed.'],
        ['Smaller sim drone', 'Replace the oversized iris with a model matching the real airframe, then test '
         'at maze_scale 1.0.'],
        ['GCS check', 'Visually test the dashboard and video in a browser; connect Mission Planner.'],
        ['Hardware bench test', 'Build the robot image on the Pi; run the Mid-360 + FAST-LIO handheld; check '
         'CPU and temperature.'],
        ['First flights', 'Tethered hover on the SLAM pose, then a mock corridor and room, then full missions.'],
        ['Competition prep', 'Practise the 5-minute setup; confirm the grid convention with the organisers.'],
    ]
    st.append(table(rows, [42, 128]))
    st.append(P('The full phased plan with exit criteria, hardware notes, rule-compliance checklist and risk '
                'list is in <font name="%s">docs/IMPLEMENTATION_PLAN.md</font>.' % MONO))

    # ---- appendix
    st.append(KeepTogether([H1('Appendix: command cheat sheet'), code(
        '# --- on the laptop, in Ubuntu (WSL) ---\n'
        'cd ~/airmouse\n'
        'docker compose -f compose.yaml -f docker/compose.wsl.yaml up -d sim   # start container\n'
        'docker compose exec sim bash                                          # shell inside\n'
        'docker compose restart sim                                            # kill a stuck run\n'
        'docker compose down                                                   # stop everything\n'
        '\n'
        '# --- inside the container ---\n'
        'am-build                       # build AirMouse code\n'
        'am-sim                         # run the simulation (add headless:=true)\n'
        'am-start / am-abort            # mission start / land from a terminal\n'
        'ros2 topic hz /Odometry        # is SLAM running?\n'
        'ros2 topic echo /airmouse/status --once\n'
        '\n'
        '# --- on the drone (Pi) ---\n'
        'docker compose up -d robot && docker compose exec robot bash\n'
        'am-build && am-robot')]))

    doc = SimpleDocTemplate(OUT, pagesize=A4, leftMargin=20 * mm, rightMargin=20 * mm, topMargin=18 * mm,
                            bottomMargin=16 * mm, title='AirMouse Project Documentation',
                            author='AirMouse team', subject='NIDAR AirMouse GPS-denied indoor drone')
    doc.build(st, onFirstPage=on_page, onLaterPages=on_page)
    print(f'wrote {OUT}')


if __name__ == '__main__':
    build()
