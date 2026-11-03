"""Figure 2: the test stages, each drawn over the same five layers.

Run from figures/src (needs Pillow and cairosvg):  python fig_test.py ../ddt_test_stages.svg ../ddt_test_stages.png
The layout check prints any text overflow, arrow through a box, or label overlap."""
import sys
from diag import D, LAY, GREY, tw

d = D(1610, 912, 'Falcon DDT test plan: which layers each stage runs, and what feeds them')
C = {k: v[0] for k, v in LAY.items()}
d.text(20, 30, 'Falcon FS-AI DDT: test stages over the same five layers', 18, 'bold', '#111827')

SX, SW = 20, 230
FX, FW = 290, 240
CH = [('veh', '1 · Vehicle', 630), ('per', '2 · Perception', 880), ('loc', '3 · Localisation', 1130), ('pla', '4 · Planning + control', 1380)]
CW, CHH = 200, 74
HY = 70
d.text(SX, HY, 'Stage', 12, 'bold', GREY)
d.text(FX, HY, 'What feeds the stack', 12, 'bold', GREY)
for lay, t, x in CH:
    d.text(x, HY, t, 12.5, 'bold', C[lay])

def pill(x, y, t, lay):
    w = tw(t, 10.5, True) + 14
    d.el.append(f'<rect x="{x - w:.1f}" y="{y}" width="{w:.1f}" height="17" rx="8.5" fill="{C[lay]}"/>')
    d.text(x - w / 2, y + 12.5, t, 10.5, 'bold', '#ffffff', 'middle')
    return w

def strip(i, y0, stage, sb, src, chips, system, ret=None, open_note=None, profile='ADS-DV', split=None):
    d.box(f's{i}', SX, y0, SW, 134, 'sys', stage, sb)
    if split is None:
        lay, title, bl, lab = src
        d.box(f'f{i}', FX, y0, FW, 134, lay, title, bl)
        d.arrow([(FX + FW, y0 + 37), (CH[0][2], y0 + 37)], '#4b5563', src=f'f{i}', dst=f'c{i}0')
        d.label((FX + FW + CH[0][2]) / 2, y0 + 30, lab, size=10, bg=False)
    else:
        (l1, t1, b1, lab), (l2, t2, b2) = split
        d.box(f'f{i}', FX, y0, FW, 52, l1, t1, b1)
        d.box(f'g{i}', FX, y0 + 82, FW, 52, l2, t2, b2)
        d.arrow([(FX + FW, y0 + 26), (CH[0][2], y0 + 26)], '#4b5563', src=f'f{i}', dst=f'c{i}0')
        d.label((FX + FW + CH[0][2]) / 2, y0 + 19, lab, size=10, bg=False)
        d.arrow([(FX + 60, y0 + 82), (FX + 60, y0 + 52)], C['net'], dash='6 4', width=1.8, src=f'g{i}', dst=f'f{i}')
        d.label(FX + 68, y0 + 71, 'measured pose (Wi-Fi)', 'start', size=10, bg=False, color=C['net'])
    for j, ((lay, t, x), (on, bl)) in enumerate(zip(CH, chips)):
        d.box(f'c{i}{j}', x, y0, CW, CHH, lay if on else 'off', t.split(' · ')[1], bl, dashed=not on, size=11.5)
        if j == 0:
            pill(x + CW - 8, y0 + 8, profile, 'veh' if profile == 'ADS-DV' else 'net')
        if j:
            px = CH[j - 1][2] + CW
            d.arrow([(px, y0 + 37), (x, y0 + 37)], '#4b5563', src=f'c{i}{j - 1}', dst=f'c{i}{j}')
    lane = y0 + 92
    x_ret = CH[3][2] + 110
    if ret:
        lab, color, dash, dst = ret
        tgt = FX + FW
        d.arrow([(x_ret, y0 + CHH), (x_ret, lane), (tgt, lane)], color, dash=dash, width=2, src=f'c{i}3', dst=dst)
        d.label(1005, lane + 4, lab, size=10.5, color=color, bold=True)
    if open_note:
        d.label(1005, lane + 4, open_note, size=10.5, color=GREY, bg=False)
    d.box(f'y{i}', CH[0][2], y0 + 106, CH[3][2] + CW - CH[0][2], 28, 'sys', '', check=False)
    d.text(CH[0][2] + 11, y0 + 125, '5 · System: ', 12, 'bold', C['sys'])
    d.text(CH[0][2] + 11 + tw('5 · System: ', 12, True), y0 + 125, system, 11.5, 'normal', '#374151')
    assert tw(system, 11.5) + tw('5 · System: ', 12, True) + 20 < CH[3][2] + CW - CH[0][2]

Y = [86, 238, 390, 542, 694]
# separators between the test loops
d.el.append(f'<line x1="20" y1="{HY + 8}" x2="{d.W - 20}" y2="{HY + 8}" stroke="#94a3b8" stroke-width="1.4"/>')
for y in Y[1:]:
    d.el.append(f'<line x1="20" y1="{y - 9}" x2="{d.W - 20}" y2="{y - 9}" stroke="#94a3b8" stroke-width="1.4"/>')
strip(1, Y[0], '1 · Full simulation', ['FSDS or a CARLA FS sim', 'one PC, no hardware', 'truth: the simulator'],
      ('sim', 'Simulator', ['ADS-DV model, FS tracks', 'lidar, camera, IMU, GNSS', 'same tracks again with', '  the 1:10 profile'], 'sensors'),
      [(1, ['sim bridge: sim topics', '  in driver formats']), (1, ['rendered lidar + image', 'vs true cone positions']),
       (1, ['sim IMU, GNSS, wheels', 'vs the true pose']), (1, ['closed loop, sim car', 'thousands of laps'])],
      'all missions; faults injected: dropped sweeps, late images, a stuck wheel speed',
      ret=('commands → sim car', C['pla'], None, 'f1'))
strip(2, Y[1], '2a · Pushable sensor rig', ['real sensors, ADS-DV heights', 'pushed by hand, our cones', 'truth: RTK + surveyed cones'],
      ('hw', 'Real world', ['lidar(s), ZED 2, F9P, MTi-3', 'encoder → ESP32 → CAN', '  (fakes the VCU wheel', '  speed frames)'], 'sensors'),
      [(1, ['real drivers, PPS time', 'CAN bridge on ESP32']), (1, ['real lidar + camera', 'our hand-made cones']),
       (1, ['real IMU, GNSS, encoder', 'vs RTK truth']), (0, ['runs, output logged', 'nothing to steer'])],
      'supervisor only: topic rates, latency per stage, timestamp alignment',
      open_note='open loop: commands are logged, not sent anywhere')
strip(3, Y[2], '2b · CarMaker HiL + CARLA', ['official ADS-DV model + VCU', 'real CAN cable to our PC', 'truth: CarMaker state'],
      ('sim', 'CarMaker HiL (Windows PC)', ['ADS-DV 2026 model, ZF VCU', 'lidar, camera: CarMaker', '  RSI, or CARLA driven', '  by CarMaker\'s pose'], 'CAN + sensors'),
      [(1, ['CAN bridge on real CAN', 'rendered sensors in']), (1, ['rendered lidar + camera', 'realistic scenes']),
       (1, ['CAN odometry, sim IMU', 'vs CarMaker truth']), (1, ['ADS-DV dynamics', 'real 50 ms handshake'])],
      'AS states from the organisers\' VCU model: ready, driving, finished, emergency brake',
      ret=('AI2VCU over the CAN cable', C['pla'], None, 'f3'))
strip(4, Y[3], '3 · 1:10 car over Wi-Fi', ['stack runs in the simulator', 'the real car drives it', 'truth: RTK on the car'], None,
      [(1, ['sim sensors in, commands', '  out over Wi-Fi DDS']), (1, ['simulated, seen from', '  the real car\'s pose']),
       (1, ['simulated, at the real', '  pose; vs the car\'s RTK']), (1, ['drives the real car', 'plan vs track checked'])],
      'missions on the scaled track; the car stops itself if commands stop for 100 ms',
      ret=('Wi-Fi DDS: steering, speed', C['net'], '7 4', 'g4'), profile='1:10',
      split=(('sim', 'Simulator, 1:10 track', ['ego at the measured pose'], 'sensors'), ('hw', '1:10 car (Raspberry Pi 5)', ['VESC, servo, RTK + IMU'])))
strip(5, Y[4], 'Event · shared ADS-DV', ['test day, then the 4 events', 'same software image', 'truth: none; log everything'],
      ('hw', 'The ADS-DV', ['our sensors + our PC', 'on the IMechE car', 'its VCU over CAN'], 'CAN + sensors'),
      [(1, ['real drivers', 'real CAN, real VCU']), (1, ['as on the rig', 'record new images']),
       (1, ['as on the rig', 'slow first laps']), (1, ['as on the HiL', 'step tests to tune'])],
      'first real AS run; steering and speed step responses logged to refit the profile',
      ret=('AI2VCU over CAN', C['pla'], None, 'f5'))

# legend
ly = 866
x = 20
def chip(x, lay, dashed=False):
    st, fill, _ = LAY[lay]
    da = ' stroke-dasharray="4 3"' if dashed else ''
    d.el.append(f'<rect x="{x}" y="{ly - 11}" width="26" height="13" rx="4" fill="{fill}" stroke="{st}" stroke-width="1.5"{da}/>')
for lay, t, da in [('per', 'layer runs and is tested here', False), ('off', 'runs, not tested here', True), ('sim', 'simulated source', False), ('hw', 'real hardware', False)]:
    chip(x, lay, da); d.text(x + 33, ly, t, 11.5); x += 33 + tw(t, 11.5) + 26
pw = pill(x + tw('ADS-DV', 10.5, True) + 14, ly - 13, 'ADS-DV', 'veh'); x += pw + 6
pw = pill(x + tw('1:10', 10.5, True) + 14, ly - 13, '1:10', 'net'); x += pw + 8
d.text(x, ly, 'vehicle profile', 11.5)
d.text(20, 892, 'Arrows inside a stage are the same data flow as the architecture figure. A stage with no return arrow is open loop. The ADS-DV profile is used everywhere except stage 3.', 11.5, 'normal', GREY)
d.save(sys.argv[1], sys.argv[2])
