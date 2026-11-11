"""Figure 3: stage 3 in detail: the stack in the simulator drives the real 1:10 car over Wi-Fi, and how it is verified.

Run from figures/src (needs Pillow and cairosvg):  python fig_scaled.py ../ddt_stage3_scaled_car.svg ../ddt_stage3_scaled_car.png
The layout check prints any text overflow, arrow through a box, or label overlap."""
import sys
from diag import D, LAY, GREY, tw

d = D(1620, 678, 'Stage 3: the simulated stack drives the real 1:10 car over Wi-Fi DDS, with live verification')
C = {k: v[0] for k, v in LAY.items()}
d.text(20, 30, 'Stage 3: the stack runs in the simulator, the real 1:10 car executes its commands', 18, 'bold', '#111827')

d.panel(20, 48, 1120, 582, 'sys', 'SIM PC', 'Ubuntu, ROS 2: the full stack with the 1:10 profile')
d.panel(1280, 48, 320, 432, 'hw', '1:10 CAR', 'Raspberry Pi 5')

d.box('sim', 40, 100, 220, 180, 'sim', 'Simulator', ['1:10 track: cones scaled', '  by the wheelbase ratio', 'ego placed at the', '  measured pose, every', '  frame (no physics)', 'renders lidar + camera'])
X = [320, 530, 740, 950]
CW = 170
chips = [('veh', 'Vehicle', ['sim bridge in', 'DDS commands out']), ('per', 'Perception', ['on rendered', '  lidar + camera']),
         ('loc', 'Localisation', ['on simulated', '  IMU, GNSS, wheels']), ('pla', 'Planning + control', ['pure pursuit + PID', 'path + prediction'])]
for (lay, t, bl), x in zip(chips, X):
    d.box(lay, x, 100, CW, 96, lay, t, bl, size=11.5)
w = tw('1:10', 10.5, True) + 14
d.el.append(f'<rect x="{X[0] + CW - 8 - w:.1f}" y="108" width="{w:.1f}" height="17" rx="8.5" fill="{C["net"]}"/>')
d.text(X[0] + CW - 8 - w / 2, 120.5, '1:10', 10.5, 'bold', '#ffffff', 'middle')
for a, b_, xa, xb in [('veh', 'per', X[0], X[1]), ('per', 'loc', X[1], X[2]), ('loc', 'pla', X[2], X[3])]:
    d.arrow([(xa + CW, 148), (xb, 148)], '#4b5563', src=a, dst=b_)
d.arrow([(260, 148), (X[0], 148)], '#4b5563', src='sim', dst='veh'); d.label(290, 141, 'sensors', size=10, bg=False)
d.box('sys', X[0], 216, X[2] + CW - X[0], 30, 'sys', '', check=False)
d.text(X[0] + 11, 236, '5 · System: ', 12, 'bold', C['sys']); d.text(X[0] + 11 + tw('5 · System: ', 12, True), 236, 'missions on the scaled track, supervisor, logs', 11.5, 'normal', '#374151')

d.box('ver', X[0], 330, 750, 150, 'sys', 'Verification: live, and again on the logs', [
    'plan vs track: cross-track, heading and speed error of the real car against the planner\'s path',
    'prediction vs track: the controller\'s pose 0.5 s ahead, against where RTK saw the car',
    'shadow model: the 1:10 model, fed the same commands, reset to the measured pose every 2 s',
    'replay: the logged planner inputs give the same outputs offline (nothing dropped or late)',
    'latency: command stamp → received on the car → first yaw-rate response',
    'pass: every error inside the spread of the same track in stage 1, 1:10 profile'])
d.arrow([(1010, 196), (1010, 330)], C['pla'], src='pla', dst='ver'); d.label(1018, 268, 'path,', 'start', size=10.5, bg=False); d.label(1018, 283, 'prediction', 'start', size=10.5, bg=False)

# car side
d.box('rx', 1300, 100, 280, 140, 'veh', 'Command receiver', ['DDS, best effort, keep last 1', 'clamped to the 1:10 limits', 'watchdog: no command', '  for 100 ms → brake', 'VESC speed, servo angle'])
d.box('pose', 1300, 304, 280, 150, 'hw', 'Pose on the car', ['MTi-8 kit: RTK fix from', '  its F9P, yaw rate', 'VESC wheel speed', 'EKF → pose at 50 Hz,', '  stamped in GNSS time'])
d.box('base', 1300, 540, 280, 72, 'hw', 'RTK base: F9P on a tripod', ['RTCM 3 corrections'])
d.arrow([(1440, 240), (1440, 304)], '#111827', width=3, src='rx', dst='pose'); d.label(1448, 277, 'the real car moves', 'start', size=10.5, bg=False, bold=True)
d.arrow([(1440, 540), (1440, 454)], GREY, dash='5 4', src='base', dst='pose'); d.label(1448, 494, 'RTCM over Wi-Fi', 'start', size=10.5, bg=False); d.label(1448, 509, '(str2str on the PC)', 'start', size=10.5, bg=False)

# Wi-Fi links
d.arrow([(X[3] + CW, 148), (1300, 148)], C['net'], dash='7 4', width=2.2, src='pla', dst='rx')
d.label(1210, 141, 'steer, speed, stamp', size=10.5, color=C['net'], bg=False)
d.label(1210, 168, 'Wi-Fi DDS', size=10.5, color=C['net'], bold=True, bg=False)
d.arrow([(1300, 400), (1070, 400)], C['net'], dash='7 4', width=2.2, src='pose', dst='ver')
d.label(1210, 393, 'measured pose,', size=10.5, color=C['net'], bold=True, bg=False)
d.label(1210, 424, 'echoed stamp', size=10.5, color=C['net'], bg=False)
d.arrow([(1110, 400), (1110, 600), (150, 600), (150, 280)], C['net'], dash='7 4', width=2.2, dst='sim', dot_start=True)
d.label(640, 604, 'measured pose → the simulator puts its car here', size=11, color=C['net'], bold=True)

d.text(20, 660, 'Wi-Fi: a dedicated 5 GHz access point; Cyclone DDS with a unicast peer list, or Zenoh. The delay is measured on every run and the same delay is added to the stage 1 reference run.', 11.5, 'normal', GREY)
d.save(sys.argv[1], sys.argv[2])
