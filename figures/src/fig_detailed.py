"""Figure: Falcon DDT architecture for technical readers: the overview's pipeline with the hardware, ports, drivers and system layer.

Combines ddt_architecture (software layers) and ddt_hardware_bw (what connects to what) in the overview's colours.
Blocks name their job; "options:" lists the methods we compare in simulation, none of them fixed.
Run from figures/src (needs Pillow and cairosvg):  python fig_detailed.py ../ddt_architecture_detailed.svg ../ddt_architecture_detailed.png
The layout check prints any text overflow, arrow through a box, or label overlap."""
import sys
from diag import D, LAY, tw

NAVY, BLUE, ORANGE, SLATE = '#000c3d', '#002eef', '#b86e00', '#475569'   # overview colours, plus slate for the system layer
DATA, TIME, POWER, RADIO = '#374151', '#002eef', '#b91c1c', '#0e7490'
LAY['phy'] = (NAVY, '#eef0f6', '#ffffff')       # hardware: sensors, ports, the car
LAY['ai'] = (BLUE, '#eef2ff', '#f7f9ff')        # software
LAY['slam'] = (ORANGE, '#fff3d6', '#fffaee')    # map + localise
LAY['sy'] = (SLATE, '#f1f3f6', '#f8f9fb')       # system layer
GREY = '#5b6170'

d = D(1900, 1600, 'Falcon DDT architecture: sensors, the MIO-5379 ports and software layers, and the ADS-DV connections')
d.text(32, 30, 'Falcon FS-AI DDT: hardware and software architecture', 18, 'bold', '#111827')

# columns
C1, C2, C3, C4, C5 = 56, 326, 596, 866, 1196
W3, W4, W5 = 240, 270, 310
mid = lambda x, w: x + w // 2


def group(x, y, w, h, lay, title, sub=None, op=0.5):
    st, _, tint = LAY[lay]
    d.el.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="12" fill="{tint}" stroke="{st}" stroke-opacity="{op}" stroke-width="1.4"/>')
    if title:
        d.text(x + 14, y + 21, title, 13, 'bold', st)
    if sub:
        d.text(x + 14 + tw(title, 13, True) + 10, y + 21, sub, 11, 'normal', GREY)


# ---------------------------------------------------------------- panels
group(32, 46, 1118, 160, 'phy', 'SENSORS', op=0.45)
group(1560, 46, 322, 454, 'phy', 'OFF THE CAR', op=0.45)
group(32, 236, 1498, 1174, 'ai', '')
d.text(C5, 257, 'CAR COMPUTER · Advantech MIO-5379', 13, 'bold', BLUE)   # top right: the sensor arrows enter on the left
group(46, 506, 530, 262, 'ai', 'PERCEIVE')
group(46, 790, 1104, 160, 'slam', 'MAP + LOCALISE', 'SLAM: both at the same time', op=0.6)
group(46, 972, 1104, 178, 'ai', 'PLAN + CONTROL')
group(46, 1172, 790, 226, 'sy', 'SYSTEM', 'AS state, missions, health')
group(32, 1440, 1850, 130, 'phy', 'ADS-DV', 'the shared IMechE car: what it gives a team', op=0.45)

# ---------------------------------------------------------------- sensors and off the car
d.box('lid', C1, 80, W3, 110, 'phy', 'Hesai ATX lidar', ['120° × 20°, 256 channels', 'interface box: Ethernet', '  and 12 V power', 'clock: gPTP slave'], rate='10 Hz')
d.box('zed', C2, 80, W3, 110, 'phy', 'ZED2 camera', ['stereo + colour, 110°', 'powered over USB', 'no clock input'], rate='30 fps')
d.box('lvl', C3, 80, W3, 110, 'phy', 'Level shifter', ['MAX3232', '3.3 V PPS → RS-232'])
d.box('mti', C4, 80, W4, 110, 'phy', 'Xsens MTi-8 dev kit', ['MTi-8: IMU + GNSS/INS', 'ZED-F9P card: RTK GNSS', '  and the PPS pulse', 'two USB ports'])
d.box('rtk', 1576, 80, 290, 110, 'phy', 'RTK corrections', ['our second F9P on a tripod,', '  or a network RTK service', 'NTRIP caster on the internet'])
d.box('pit', 1576, 384, 290, 100, 'phy', 'Pit laptop (Foxglove)', ['live topics and health', 'view only: sends no', '  commands to the car'])

# ---------------------------------------------------------------- ports and devices on the board
PY, PH = 272, 66
d.box('lan', C1, PY, W3, PH, 'phy', 'LAN2 · Intel i226', ['lidar Ethernet', 'gPTP grandmaster'])
d.box('u3', C2, PY, W3, PH, 'phy', 'USB 3.2', ['ZED2 images'])
d.box('com', C3, PY, W3, PH, 'phy', 'COM1 · RS-232', ['PPS on the DCD pin'])
d.box('u2', C4, PY, W4, PH, 'phy', 'USB 2.0', ['MTi-8 kit: IMU, F9P'])
d.box('lte', C5, PY, W5, PH, 'phy', '4G USB modem', ['internet for NTRIP', 'good to have'], dashed=True)

# ---------------------------------------------------------------- drivers and time
DY, DH = 384, 100
d.box('dl', C1, DY, W3, DH, 'ai', 'Lidar driver', ['HesaiLidar_ROS_2.0', 'packets → point cloud', 'stamped at capture,', '  in GNSS time'])
d.box('dc', C2, DY, W3, DH, 'ai', 'Camera driver', ['zed-open-capture', '  (no NVIDIA GPU, no CUDA)', 'left colour image', 'arrival time − fixed delay'])
d.box('dt', C3, DY, W3, DH, 'ai', 'Time and frames', ['chrony: clock on the PPS', 'ptp4l: gPTP to the lidar', 'tf: sensor poses (CAD)'])
d.box('dx', C4, DY, W4, DH, 'ai', 'IMU and GNSS drivers', ['Xsens + u-blox ROS 2', 'fix type and covariance', 'NTRIP client: RTCM', '  → F9P over USB'])
d.box('wifi', C5, DY, W5, 66, 'phy', 'Wi-Fi · M.2 E-key', ['Intel AX210, 2.4 GHz'])

# ---------------------------------------------------------------- perceive
d.box('pl', C1, 540, W3, 100, 'ai', 'Lidar cones', ['de-skew, ground removal', 'clustering, cone-shape check', 'options: geometric or a', '  learned detector'])
d.box('pc', C2, 540, W3, 100, 'ai', 'Camera colour', ['colour of each cone', 'options: HSV rule, small', '  CNN, YOLO', 'OpenVINO: Arc GPU or NPU'])
d.box('pf', C1, 670, 510, 86, 'ai', 'Lidar–camera fusion', ['lidar cones projected into', '  the image of the same time', 'position from the lidar', 'colour from the camera', 'no match → "unknown"'], cols=2)

# ---------------------------------------------------------------- map + localise
SY, SH = 822, 116
d.box('map', C1, SY, 444, SH, 'slam', 'Map', ['cone positions + colours', '  in the map frame', 'lap 1 builds it, later', '  laps reuse it',
                                           'options: EKF-SLAM,', '  FastSLAM, GraphSLAM', 'association options:', '  nearest neighbour, JCBB'], cols=2)
d.box('loc', 580, SY, 205, SH, 'slam', 'Localise', ['car pose in the map', '  frame', 'removes odometry drift', '  (map → odom)'])
d.box('odo', 880, SY, 256, SH, 'slam', 'State estimation', ['fuses IMU, wheel speeds,', '  steering angle, GNSS', 'out: pose, velocity, yaw rate', 'options: EKF, UKF; LIO', '  (lidar + IMU) as an input'])

# ---------------------------------------------------------------- plan + control
QY, QH = 1004, 134
d.box('path', C1, QY, 444, QH, 'ai', 'Path planner', ['one mode per mission:', '  accel, skidpad,', '  autocross, trackdrive', 'finds the start/finish', '  line, reports crossings',
                                                    'unknown track: centre line', '  options: Delaunay, search', 'saved map: race line', '  options: min curvature,', '  min time'], cols=2)
d.box('spd', 580, QY, 220, QH, 'ai', 'Speed profile', ['speed limits from grip', '  and steering', 'options: forward–backward', '  pass, or inside an MPC'])
d.box('ctl', 880, QY, 256, QH, 'ai', 'Controller', ['tracks the path and speeds', 'options: pure pursuit,', '  Stanley, PID, MPC', 'out: steering angle, axle', '  torque, brake pressure', 'clamped, rate-limited'])

# ---------------------------------------------------------------- system, CAN, board
d.box('bt', C1, 1204, 444, 96, 'sy', 'Mission behaviour tree', ['one tree per mission', 'starts on "go"', 'counts laps', 'last lap: stops in the zone'], cols=2)
d.box('as', 580, 1204, 220, 96, 'sy', 'AS state manager', ['follows the VCU\'s AS state', 'mission status → VCU', 'EBS request on a fault'])
d.box('sup', C1, 1320, 744, 68, 'sy', 'Supervisor (health)', ['rate and age of every topic', 'node heartbeats', 'CPU and GPU temperature, power', 'kicks the hardware watchdog'], cols=2)
d.box('can', 880, 1204, 256, 86, 'ai', 'CAN bridge (FS-AI API)', ['VCU2AI → ROS topics', 'commands → AI2VCU, 10 ms', 'answers the handshake'], rate='100 Hz')
d.box('cnb', 880, 1330, 256, 66, 'phy', 'CANable 2.0 · USB', ['USB–CAN, SocketCAN', 'vehicle CAN, 500 kbit/s'])
d.box('tel', C5, 540, W5, 100, 'ai', 'Telemetry bridge', ['Foxglove bridge: live', '  topics to the pit', 'read-only'])
d.box('log', C5, 670, W5, 100, 'sy', 'Logger (black box)', ['rosbag2 (MCAP): all topics', 'raw CAN log (candump)', 'GNSS time, git hash per run'])
d.box('mio', C5, 1204, W5, 184, 'phy', 'MIO-5379 hardware', ['Core Ultra 7 255H, 16 cores', 'Arc GPU + NPU: OpenVINO', '32 GB DDR5, 1 TB NVMe', 'DC in 12–24 V: 13.7 V direct',
                                                         'hardware watchdog', 'on-board CAN-FD unused:', '  no Linux driver', '3.5 Grms, 0–60 °C', 'Ubuntu 24.04, ROS 2, Docker'])

# ---------------------------------------------------------------- the car
d.box('skt', C1, 1474, 300, 82, 'phy', 'Sensor socket', ['TE DT06-2S, front plate', '12 V, up to 5 A'])
d.box('plug', 880, 1474, 460, 82, 'phy', 'Compute plug (7W2)', ['13.7 V, 30 A fuse', 'vehicle CAN, 500 kbit/s'])
d.box('vcu', 1420, 1474, 446, 82, 'phy', 'VCU', ['AS state machine', 'steering, motors, brakes', 'wheel speeds every 10 ms'], cols=2)

# ---------------------------------------------------------------- arrows
L = dict(size=10.5, bg=False)
# sensors -> ports
x = mid(C1, W3); d.arrow([(x, 190), (x, PY)], DATA, src='lid', dst='lan', both=True); d.label(x + 8, 226, 'Ethernet: points, gPTP', 'start', **L)
x = mid(C2, W3); d.arrow([(x, 190), (x, PY)], DATA, src='zed', dst='u3'); d.label(x + 8, 226, 'USB 3: images', 'start', **L)
d.arrow([(C4, 160), (C3 + W3, 160)], TIME, dash='5 4', width=2, src='mti', dst='lvl'); d.label(851, 153, 'PPS', size=10, color=TIME, bg=False, bold=True)
x = mid(C3, W3); d.arrow([(x, 190), (x, PY)], TIME, dash='5 4', width=2, src='lvl', dst='com'); d.label(x + 8, 226, 'PPS, RS-232 level', 'start', size=10.5, color=TIME, bg=False, bold=True)
x = mid(C4, W4); d.arrow([(x, 190), (x, PY)], DATA, src='mti', dst='u2', both=True); d.label(x + 8, 226, 'USB: IMU, GNSS; RTCM in', 'start', **L)
d.arrow([(1720, 190), (1720, 305), (C5 + W5, 305)], RADIO, dash='2 3', width=2, src='rtk', dst='lte'); d.label(1612, 299, 'RTCM', size=10, color=RADIO, bg=False, bold=True)
# ports -> drivers
d.arrow([(mid(C1, W3), PY + PH), (mid(C1, W3), DY)], DATA, src='lan', dst='dl')
d.arrow([(mid(C2, W3), PY + PH), (mid(C2, W3), DY)], DATA, src='u3', dst='dc')
d.arrow([(mid(C3, W3), PY + PH), (mid(C3, W3), DY)], TIME, dash='5 4', width=2, src='com', dst='dt')
d.arrow([(mid(C4, W4), PY + PH), (mid(C4, W4), DY)], DATA, src='u2', dst='dx', both=True)
d.arrow([(1240, PY + PH), (1240, 362), (1110, 362), (1110, DY)], DATA, dash='6 4', src='lte', dst='dx'); d.label(1175, 356, 'RTCM', **L)
# telemetry
d.arrow([(1351, 540), (1351, DY + 66)], DATA, src='tel', dst='wifi'); d.label(1359, 500, 'live topics', 'start', **L)
d.arrow([(C5 + W5, 417), (1576, 417)], RADIO, dash='2 3', width=2, src='wifi', dst='pit'); d.label(1554, 410, '2.4 GHz', 'end', size=10, color=RADIO, bg=False, bold=True)
# drivers -> perceive
d.arrow([(mid(C1, W3), DY + DH), (mid(C1, W3), 540)], DATA, src='dl', dst='pl'); d.label(mid(C1, W3) + 8, 500, 'point cloud', 'start', **L)
d.arrow([(mid(C2, W3), DY + DH), (mid(C2, W3), 540)], DATA, src='dc', dst='pc'); d.label(mid(C2, W3) + 8, 500, 'image', 'start', **L)
d.arrow([(mid(C1, W3), 640), (mid(C1, W3), 670)], BLUE, src='pl', dst='pf'); d.label(mid(C1, W3) + 8, 659, 'cones', 'start', **L)
d.arrow([(mid(C2, W3), 640), (mid(C2, W3), 670)], BLUE, src='pc', dst='pf'); d.label(mid(C2, W3) + 8, 659, 'colours', 'start', **L)
# perceive -> map; IMU, GNSS -> state estimation
d.arrow([(450, 756), (450, SY)], BLUE, src='pf', dst='map'); d.label(458, 784, 'coloured cones', 'start', **L)
d.arrow([(mid(C4, W4), DY + DH), (mid(C4, W4), SY)], DATA, src='dx', dst='odo'); d.label(mid(C4, W4) + 8, 660, 'IMU, GNSS fix', 'start', **L)
# map <-> localise <-> state estimation
d.arrow([(500, 860), (580, 860)], ORANGE, width=2, src='map', dst='loc'); d.label(540, 853, 'map cones', size=10.5, color=ORANGE, bg=False, bold=True)
d.arrow([(580, 905), (500, 905)], ORANGE, width=2, src='loc', dst='map'); d.label(540, 898, 'pose', size=10.5, color=ORANGE, bg=False, bold=True)
d.arrow([(880, 860), (785, 860)], ORANGE, width=2, src='odo', dst='loc'); d.label(832, 853, 'odometry', size=10.5, color=ORANGE, bg=False, bold=True)
d.arrow([(785, 905), (880, 905)], ORANGE, width=2, src='loc', dst='odo'); d.label(832, 898, 'map → odom', size=10.5, color=ORANGE, bg=False, bold=True)
# -> plan + control
d.arrow([(450, SY + SH), (450, QY)], ORANGE, src='map', dst='path'); d.label(458, 975, 'cone map, pose', 'start', **L)
d.arrow([(960, SY + SH), (960, QY)], ORANGE, src='odo', dst='ctl'); d.label(968, 975, 'pose, velocity', 'start', **L)
d.arrow([(500, 1060), (580, 1060)], BLUE, src='path', dst='spd'); d.label(540, 1053, 'path', **L)
d.arrow([(800, 1060), (880, 1060)], BLUE, src='spd', dst='ctl'); d.label(840, 1053, 'speeds', **L)
# planner <-> behaviour tree
d.arrow([(360, QY + QH), (360, 1204)], SLATE, src='path', dst='bt'); d.label(352, 1166, 'line crossed', 'end', **L)
d.arrow([(460, 1204), (460, QY + QH)], SLATE, src='bt', dst='path'); d.label(468, 1166, 'mission, mode', 'start', **L)
# behaviour tree <-> AS state manager <-> CAN bridge; supervisor
d.arrow([(580, 1235), (500, 1235)], SLATE, src='as', dst='bt'); d.label(540, 1228, 'AS state, go', size=10)
d.arrow([(500, 1270), (580, 1270)], SLATE, src='bt', dst='as'); d.label(540, 1263, 'finished', size=10)
d.arrow([(880, 1235), (800, 1235)], SLATE, src='can', dst='as'); d.label(840, 1228, 'AS state, go', size=10)
d.arrow([(800, 1270), (880, 1270)], SLATE, src='as', dst='can'); d.label(840, 1263, 'status, EBS', size=10)
d.arrow([(690, 1320), (690, 1300)], SLATE, src='sup', dst='as'); d.label(698, 1314, 'fault', 'start', size=10, bg=False)
# commands and vehicle feedback
d.arrow([(1008, QY + QH), (1008, 1204)], NAVY, width=2.4, src='ctl', dst='can'); d.label(1000, 1166, 'steering, torque, brake', 'end', size=10.5, color=NAVY, bg=False, bold=True)
d.arrow([(C4 + W4, 1240), (1166, 1240), (1166, 920), (1136, 920)], DATA, src='can', dst='odo'); d.label(1171, 1080, 'wheel speeds, steering angle', size=10.5, rot=-90)
d.arrow([(1008, 1290), (1008, 1330)], NAVY, width=2.4, src='can', dst='cnb', both=True)
d.arrow([(1008, 1396), (1008, 1474)], NAVY, width=2.4, src='cnb', dst='plug', both=True); d.label(1016, 1428, 'CAN', 'start', size=10.5, color=NAVY, bg=False, bold=True)
d.arrow([(1340, 1515), (1420, 1515)], NAVY, width=2.4, src='plug', dst='vcu', both=True); d.label(1380, 1508, 'CAN', size=10.5, color=NAVY, bg=False, bold=True)
# power
d.arrow([(1300, 1474), (1300, 1388)], POWER, dash='12 3 2 3', width=2, src='plug', dst='mio'); d.label(1308, 1428, '13.7 V', 'start', size=10.5, color=POWER, bg=False, bold=True)
d.arrow([(C1, 1515), (16, 1515), (16, 135), (C1, 135)], POWER, dash='12 3 2 3', width=2, src='skt', dst='lid'); d.label(20, 840, '12 V to the lidar', size=10.5, color=POWER, rot=-90, bold=True)

# ---------------------------------------------------------------- key (right of the computer)
kx, ky = 1576, 548
d.text(kx, ky, 'Key', 13, 'bold', '#111827')
ky += 26
for lay, t, dash in [('phy', 'hardware', False), ('ai', 'software', False), ('slam', 'map + localise', False), ('sy', 'system layer', False), ('phy', 'good to have', True)]:
    st, fill, _ = LAY[lay]
    da = ' stroke-dasharray="6 4"' if dash else ''
    d.el.append(f'<rect x="{kx}" y="{ky - 11}" width="30" height="15" rx="4" fill="{fill}" stroke="{st}" stroke-width="1.5"{da}/>')
    d.text(kx + 40, ky, t, 11.5, 'normal', '#374151'); ky += 24
ky += 6
for col, dash, w, t in [(DATA, None, 1.6, 'data'), (NAVY, None, 2.4, 'CAN: commands and feedback'), (TIME, '5 4', 2, 'time (PPS)'),
                        (POWER, '12 3 2 3', 2, 'power'), (RADIO, '2 3', 2, 'radio or internet')]:
    da = f' stroke-dasharray="{dash}"' if dash else ''
    d.el.append(f'<line x1="{kx}" y1="{ky - 4}" x2="{kx + 30}" y2="{ky - 4}" stroke="{col}" stroke-width="{w}"{da}/>')
    d.text(kx + 40, ky, t, 11.5, 'normal', '#374151'); ky += 24
ky += 10
for line in ['Arrowheads at both ends: both', 'directions.', '',
             '"options:" lists the methods we', 'compare in simulation; none', 'is fixed yet.', '',
             'The pose reaches every block', 'through tf2 and is not drawn.', '',
             'One clock: the F9P PPS sets the', 'computer clock (chrony), which', 'passes it to the lidar (gPTP).']:
    if line:
        d.text(kx, ky, line, 11.5, 'normal', GREY)
    ky += 17

d.save(sys.argv[1], sys.argv[2])
