"""Figure 1: Falcon DDT system architecture, five layers, sensors inside the vehicle layer.

Run from figures/src (needs Pillow and cairosvg):  python fig_arch.py ../ddt_architecture.svg ../ddt_architecture.png
The layout check prints any text overflow, arrow through a box, or label overlap."""
import sys
from diag import D, LAY, GREY, tw

d = D(1850, 1130, 'Falcon DDT system architecture: vehicle, perception, localisation, planning and control, system')
C = {k: v[0] for k, v in LAY.items()}
d.text(20, 28, 'Falcon FS-AI DDT: system architecture', 18, 'bold', '#111827')

# columns
HX, HW = 34, 196            # hardware
DX, DW = 280, 232           # drivers
P1X, P1W = 610, 214         # lidar / camera cones
P3X, P3W = 898, 214         # fusion
LX, LW = 1210, 232          # localisation
PX, PW = 1540, 232          # planning and control
ROUTE = 1830                # mission and mode, behaviour tree → planner
LINE = 1800                 # line crossings, planner → behaviour tree
# rows
R1, R2 = (92, 196), (212, 316)
R3A, R3B = (340, 432), (446, 538)
R4, R5 = (556, 636), (654, 764)
PB = 780                    # panel bottom
LANE = 804
SY0, SY1 = 846, 1030        # system panel
B0 = 884                    # system boxes top

d.panel(20, 40, 506, PB - 40, 'veh', '1 · VEHICLE', 'sensors, drivers, time, CAN')
d.panel(596, 40, 530, PB - 40, 'per', '2 · PERCEPTION', 'where the cones are, and their colour')
d.panel(1196, 40, 260, PB - 40, 'loc', '3 · LOCALISATION')
d.panel(1526, 40, 260, PB - 40, 'pla', '4 · PLANNING + CONTROL')
d.panel(20, SY0, 1766, SY1 - SY0, 'sys', '5 · SYSTEM', 'AS state, health, missions')
d.text(HX, 84, 'hardware', 11, 'bold', GREY)
d.text(DX, 84, 'drivers and time', 11, 'bold', GREY)

def b(name, x, y0y1, w, lay, title, bullets=(), **k):
    y0, y1 = y0y1
    return d.box(name, x, y0, w, y1 - y0, lay, title, bullets, **k)

# hardware
b('lid', HX, R1, HW, 'hw', 'Lidar, one or two', ['main, to buy: 120°', '  solid-state or 360°', '  32-channel', '2nd, optional: VLP-16'], rate='10 Hz')
b('zed', HX, R2, HW, 'hw', 'ZED 2 camera', ['stereo, colour, 110°', 'owned; the shared car', '  also has one'], rate='30 fps')
b('f9p', HX, (R3A[0], R3B[1]), HW, 'hw', 'u-blox ZED-F9P', ['position, velocity', 'PPS time pulse', 'RTK corrections:', '  NTRIP over 4G,', '  if IMechE allows', 'owned (two boards)'], rate='10 Hz')
b('imu', HX, R4, HW, 'hw', 'Xsens MTi-3 IMU', ['yaw rate, acceleration', 'to buy (dev kit)'])
b('vcu', HX, R5, HW, 'hw', 'ADS-DV VCU', ['the shared IMechE car', 'CAN, 500 kbit/s', 'steering, motor, brakes'], rate='100 Hz')
# drivers
b('dl', DX, R1, DW, 'veh', 'Lidar drivers', ['maker\'s ROS 2 driver,', '  one per lidar', 'packets → point cloud', 'stamped in GNSS time'])
b('dc', DX, R2, DW, 'veh', 'Camera driver', ['GPU fitted: ZED wrapper', 'no GPU: zed-open-capture', 'left colour image', 'stamped in GNSS time'])
b('dg', DX, R3A, DW, 'veh', 'GNSS driver', ['u-blox ROS 2 driver', 'NTRIP client over 4G', 'fix type and covariance'])
b('dt', DX, R3B, DW, 'veh', 'Time and frames', ['chrony: PC clock on PPS', 'tf: sensor poses from CAD', '  (per vehicle profile)'])
b('di', DX, R4, DW, 'veh', 'IMU driver', ['Xsens ROS 2 driver', 'stamped on arrival'])
b('can', DX, R5, DW, 'veh', 'CAN bridge', ['FS-AI API on a CANable', 'answers 50 ms handshake', 'VCU2AI → ROS topics', 'commands → AI2VCU, clamped'], rate='100 Hz')
# perception
b('pl', P1X, R1, P1W, 'per', 'Lidar cones', ['one detector per lidar', 'de-skew, ground removal', 'clustering, size check', '505 mm tall: big orange'], rate='≤30 ms')
b('pc', P1X, R2, P1W, 'per', 'Camera cones', ['YOLO: OpenVINO, or', '  TensorRT with the GPU', 'blue, yellow, orange,', '  big orange'], rate='≤25 ms')
b('pf', P3X, (R1[0], R2[1]), P3W, 'per', 'Lidar–camera fusion', ['once per lidar scan', 'project its cones into', '  the image taken at', '  the same time', 'match boxes (Hungarian)', 'position from lidar,', '  colour from camera', 'no match → "unknown"'], rate='10 Hz')
# localisation
LS = (92, 296)
b('slam', LX, LS, LW, 'loc', 'Cone SLAM + mapping', ['data association (JCBB)', 'EKF-SLAM first,', '  GraphSLAM (iSAM2) later', 'builds the cone map', 'lap 1: map + localise', 'laps 2–10: map frozen', 'out: map → odom'], rate='10 Hz')
b('ekf', LX, (340, R5[1]), LW, 'loc', 'Odometry EKF', ['predict: IMU', 'update: wheel speeds,', '  steering angle, GNSS', 'state: x, y, ψ, vx, vy, r', 'gyro bias as a state', 'out: odom → base_link', 'pose in map = SLAM', '  correction ∘ odometry'], rate='100 Hz')
# planning and control
b('path', PX, LS, PW, 'pla', 'Path planner', ['one mode per mission:', 'accel: straight line', 'skidpad: fixed figure 8', 'autocross: Delaunay', '  mid-line of map cones', 'trackdrive: race line', '  on the saved map', 'finds the big orange', '  pair; flags crossings'], rate='10 Hz')
b('speed', PX, (340, 440), PW, 'pla', 'Speed profile', ['forward–backward pass', 'grip + steering limits', '  from the profile'])
b('ctrl', PX, (484, R5[1]), PW, 'pla', 'Controller', ['steering: pure pursuit,', '  MPC later', 'speed: PID → axle', '  torque, brake pressure', 'rate limits from the', '  vehicle profile'], rate='50 Hz')
# system
d.box('as', 34, B0, 478, 132, 'sys', 'AS state manager', ['follows the VCU\'s AS state: off, ready,', '  driving, finished, emergency brake', 'reports mission status: selected,', '  running, finished', 'requests the emergency stop on a fault'])
d.box('sup', 610, B0, 832, 78, 'sys', 'Supervisor (health)', ['rate and age of every topic', 'node heartbeats', 'CPU and GPU temperature', 'logs everything (rosbag2 / MCAP)'], cols=2)
d.box('bt', PX, B0, PW, 132, 'sys', 'Mission behaviour tree', ['one tree per mission', 'starts on "go"', 'counts line crossings', 'last lap → stop inside', '  the zone after the line'])

mid = lambda r: (r[0] + r[1]) // 2
A = '#4b5563'
# sensor -> driver
for s_, t_, y, lab in [('lid', 'dl', mid(R1), 'UDP'), ('zed', 'dc', mid(R2), 'USB 3'), ('f9p', 'dg', mid(R3A), 'UBX'), ('imu', 'di', mid(R4), 'USB')]:
    d.arrow([(HX + HW, y), (DX, y)], A, src=s_, dst=t_)
    d.label((HX + HW + DX) / 2, y - 7, lab, size=10.5, bg=False)
d.arrow([(HX + HW, mid(R3B)), (DX, mid(R3B))], C['veh'], dash='5 4', src='f9p', dst='dt')
d.label((HX + HW + DX) / 2, mid(R3B) - 7, 'PPS', size=10.5, bg=False, color=C['veh'])
d.arrow([(HX + HW, 690), (DX, 690)], A, src='vcu', dst='can')
d.label((HX + HW + DX) / 2, 683, 'CAN', size=10.5, bg=False)
d.arrow([(DX, 734), (HX + HW, 734)], C['pla'], src='can', dst='vcu')
d.label((HX + HW + DX) / 2, 727, 'CAN', size=10.5, bg=False, color=C['pla'])
# drivers -> perception
d.arrow([(DX + DW, 144), (P1X, 144)], A, src='dl', dst='pl'); d.label((DX + DW + P1X) / 2, 137, 'point cloud', size=10.5, bg=False)
d.arrow([(DX + DW, 264), (P1X, 264)], A, src='dc', dst='pc'); d.label((DX + DW + P1X) / 2, 257, 'image', size=10.5, bg=False)
d.arrow([(P1X + P1W, 144), (P3X, 144)], C['per'], src='pl', dst='pf'); d.label((P1X + P1W + P3X) / 2, 137, 'positions', size=10.5, bg=False)
d.arrow([(P1X + P1W, 264), (P3X, 264)], C['per'], src='pc', dst='pf'); d.label((P1X + P1W + P3X) / 2, 257, 'boxes', size=10.5, bg=False)
# perception -> localisation
d.arrow([(P3X + P3W, 200), (LX, 200)], C['per'], src='pf', dst='slam'); d.label((P3X + P3W + LX) / 2, 193, 'coloured cones', size=10.5, bg=False)
# drivers -> odometry EKF (through the empty lower half of perception)
for s_, y, lab in [('dg', mid(R3A), 'position, velocity'), ('di', mid(R4), 'yaw rate, acceleration'), ('can', 700, 'wheel speeds, steering angle')]:
    d.arrow([(DX + DW, y), (LX, y)], A, src=s_, dst='ekf')
    d.label(860, y - 7, lab, size=11, bg=False)
# EKF <-> SLAM
d.arrow([(1240, 340), (1240, 296)], C['loc'], src='ekf', dst='slam'); d.label(1248, 323, 'odometry', 'start', size=10.5, bg=False)
d.arrow([(1404, 296), (1404, 340)], C['loc'], src='slam', dst='ekf'); d.label(1396, 323, 'map → odom', 'end', size=10.5, bg=False)
# localisation -> planning
d.arrow([(LX + LW, 180), (PX, 180)], C['loc'], src='slam', dst='path'); d.label((LX + LW + PX) / 2, 173, 'cone map', size=10.5, bg=False)
d.arrow([(LX + LW, 600), (PX, 600)], C['loc'], src='ekf', dst='ctrl'); d.label((LX + LW + PX) / 2, 593, 'pose, 100 Hz', size=10.5, bg=False)
d.arrow([(1656, 296), (1656, 340)], C['pla'], src='path', dst='speed'); d.label(1664, 323, 'path', 'start', size=10.5, bg=False)
d.arrow([(1656, 440), (1656, 484)], C['pla'], src='speed', dst='ctrl'); d.label(1664, 467, 'target speeds', 'start', size=10.5, bg=False)
# command lane back to the CAN bridge
d.arrow([(1600, R5[1]), (1600, LANE), (490, LANE), (490, R5[1])], C['pla'], width=2.2, src='ctrl', dst='can')
d.label(1030, LANE + 4, 'steering angle, axle torque, brake pressure  →  AI2VCU, every 10 ms', size=11.5, color=C['pla'], bold=True)
# system links
d.arrow([(310, R5[1]), (310, B0)], C['sys'], src='can', dst='as'); d.label(302, 836, 'AS state, mission, go', 'end', size=10.5, bg=False)
d.arrow([(370, B0), (370, R5[1])], C['sys'], src='as', dst='can'); d.label(378, 836, 'mission status, EBS request', 'start', size=10.5, bg=False)
d.arrow([(610, 922), (512, 922)], C['sys'], src='sup', dst='as'); d.label(561, 915, 'fault', size=10.5, bg=False)
d.arrow([(512, 984), (PX, 984)], C['sys'], src='as', dst='bt'); d.label(820, 990, 'AS state, mission, go', size=10.5)
d.arrow([(PX, 1004), (512, 1004)], C['sys'], src='bt', dst='as'); d.label(1240, 1010, 'mission finished', size=10.5)
d.arrow([(PX + PW, 950), (ROUTE, 950), (ROUTE, 130), (PX + PW, 130)], C['sys'], src='bt', dst='path')
d.label(ROUTE + 5, 540, 'mission and mode: explore, race, stop', size=11, rot=-90, color=C['sys'])
d.arrow([(PX + PW, 270), (LINE, 270), (LINE, 920), (PX + PW, 920)], C['pla'], src='path', dst='bt')
d.label(LINE + 5, 600, 'start / finish line crossed', size=11, rot=-90, color=C['pla'])

# legend and notes
ly = 1062
x = 20
d.text(x, ly, 'Layers:', 11.5, 'bold', '#374151'); x += 62
for k, t in [('hw', 'hardware'), ('veh', 'vehicle'), ('per', 'perception'), ('loc', 'localisation'), ('pla', 'planning + control'), ('sys', 'system')]:
    d.el.append(f'<rect x="{x}" y="{ly - 11}" width="26" height="13" rx="4" fill="{LAY[k][1]}" stroke="{LAY[k][0]}" stroke-width="1.5"/>')
    d.text(x + 33, ly, t, 11.5, 'normal', '#374151'); x += 33 + tw(t, 11.5) + 26
d.el.append(f'<line x1="{x}" y1="{ly - 4}" x2="{x + 34}" y2="{ly - 4}" stroke="{GREY}" stroke-width="1.6"/>'); d.text(x + 40, ly, 'data', 11.5); x += 40 + tw('data', 11.5) + 24
d.el.append(f'<line x1="{x}" y1="{ly - 4}" x2="{x + 34}" y2="{ly - 4}" stroke="{C["veh"]}" stroke-width="1.6" stroke-dasharray="5 4"/>'); d.text(x + 40, ly, 'time pulse', 11.5); x += 40 + tw('time pulse', 11.5) + 24
d.el.append(f'<line x1="{x}" y1="{ly - 4}" x2="{x + 34}" y2="{ly - 4}" stroke="{C["pla"]}" stroke-width="2.2"/>'); d.text(x + 40, ly, 'commands to the car', 11.5)
d.text(20, 1088, 'Data flows left to right; commands return along the bottom. The pose reaches every block through tf2 and is not drawn: lidar de-skew, fusion and the planner\'s line-crossing check read it there.', 11.5, 'normal', GREY)
d.text(20, 1106, 'Rates are targets. Only the vehicle layer\'s drivers change between the ADS-DV and 1:10 profiles; the other layers take the profile\'s numbers (size, limits) as parameters.', 11.5, 'normal', GREY)
d.save(sys.argv[1], sys.argv[2])
