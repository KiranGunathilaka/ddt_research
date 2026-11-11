"""Figure: Falcon DDT hardware, what connects to what: sensors, the MIO-5379, the ADS-DV's plugs, the Orin NX option and the RTK base.

Run from figures/src (needs Pillow and cairosvg):  python fig_hardware.py ../ddt_hardware.svg ../ddt_hardware.png
Black and white, without the Orin NX option and the notes (the version on the page):  python fig_hardware.py ../ddt_hardware_bw.svg ../ddt_hardware_bw.png --bw
The layout check prints any text overflow, arrow through a box, or label overlap."""
import sys
from diag import D, LAY, GREY, tw

BW = '--bw' in sys.argv
if BW:
    for k in LAY:
        LAY[k] = ('#111111', '#ffffff', '#ffffff')
d = D(1680, 864 if BW else 912, 'Falcon DDT hardware: sensors, the car computer, the ADS-DV connections' + (' and the RTK base' if BW else ', the Orin NX option and the RTK base'))
C = {k: v[0] for k, v in LAY.items()}
d.text(20, 30, 'Falcon FS-AI DDT: hardware and connections', 18, 'bold', '#111827')

DATA, TIME, CAN, POWER, RADIO = '#4b5563', C['veh'], C['pla'], '#b91c1c', C['net']
# line styles: in black and white the dash pattern and width tell the kinds apart
DS = dict(time='5 4', can=None, power=None, radio='7 4', option='6 4')
WD = dict(data=1.6, time=2, can=2, power=2, radio=2)
if BW:
    DATA = TIME = CAN = POWER = RADIO = '#111111'
    DS = dict(time='5 4', can=None, power='12 3 2 3', radio='2 3', option='6 4')
    WD = dict(data=1.4, time=1.8, can=3.2, power=2.2, radio=2)

# panels
d.panel(20, 46, 1640, 168, 'sim', 'ADS-DV', 'the shared IMechE car: what it gives a team')
d.panel(20, 240, 560, 584, 'hw', 'SENSORS')
d.panel(610, 240, 680, 584, 'per', 'CAR COMPUTER')
d.panel(1310, 240, 350, 584, 'off', 'OFF THE CAR' if BW else 'OPTION AND OFF THE CAR')

# ADS-DV connections
d.box('dt06', 330, 90, 230, 100, 'sim', 'Sensor socket', ['TE DT06-2S, front plate', '12 V, up to 5 A'])
d.box('plug', 900, 90, 260, 100, 'sim', 'Compute plug (7W2)', ['13.7 V, 30 A fuse', 'vehicle CAN, 500 kbit/s', 'rear bay or front plate'])
d.box('vcu', 1330, 90, 310, 100, 'sim', 'VCU', ['AS state machine', 'steering, motor, brakes', 'wheel speeds, every 10 ms'])

# sensors
d.box('atx', 40, 280, 250, 110, 'hw', 'Hesai ATX lidar', ['120° × 20°, 256 channels', '8 W, to buy', 'clock: gPTP slave'], rate='10 Hz')
d.box('abx', 330, 280, 230, 110, 'hw', 'ATX interface box', ['auto Ethernet → RJ45', 'power to the lidar'])
d.box('zed', 40, 430, 250, 100, 'hw', 'ZED2 camera', ['stereo + colour, 110°', 'owned', 'powered by its USB'], rate='30 fps')
d.box('mti', 40, 570, 250, 230, 'hw', 'Xsens MTi-8 dev kit', ['MTi-8: IMU + GNSS/INS', 'ZED-F9P card: RTK', '  GNSS, PPS output', 'on the kit: F9P → MTi-8', '  UBX data + PPS', 'GNSS antenna in the kit', 'two USB ports', 'to buy'])
d.box('lvl', 330, 715, 230, 85, 'hw', 'Level shifter', ['MAX3232', '3.3 V PPS → RS-232'])

# car computer
d.box('mio', 730, 280, 320, 520, 'per', 'Advantech MIO-5379', [
    'Core Ultra 7 255H, 16 cores', 'Arc GPU + NPU: OpenVINO', '32 GB DDR5, 1 TB NVMe', 'Ubuntu 24.04, ROS 2',
    'runs the whole stack', 'LAN2 (i226): lidar,', '  gPTP grandmaster'] + ([] if BW else ['LAN1 (i210): Orin NX option']) + [
    'USB 3.2: ZED2, CAN adapter', 'USB 2.0: MTi-8 board, F9P', '  card, 4G modem', 'COM1: PPS on DCD, chrony',
    'DC in: 12–24 V, direct', 'on-board CAN-FD unused:', '  no Linux driver', '3.5 Grms vibration rating'])
d.box('can', 1090, 280, 180, 90, 'per', 'CANable 2.0', ['USB–CAN adapter', 'SocketCAN'])
d.box('lte', 1090, 560, 180, 90, 'per', '4G USB modem', ['NTRIP client', 'good to have'], dashed=True)

# option and off the car
if not BW: d.box('orin', 1330, 280, 310, 210, 'off', 'Jetson Orin NX 8 GB (option)', [
    'only if the full ZED SDK', '  is needed', 'ZED2 moves to its USB', 'ZED SDK depth + YOLO', '  on TensorRT', 'sends a cone list, not', '  images', 'clock: PTP from the MIO'], dashed=True)
d.box('base', 1330, 560, 310, 120, 'off', 'RTK corrections', ['our second F9P on a tripod,', '  or a network RTK service', 'NTRIP caster on the internet'])

# power from the car
d.arrow([(445, 190), (445, 280)], POWER, dash=DS['power'], width=WD['power'], src='dt06', dst='abx'); d.label(453, 231, '12 V', 'start', size=10.5, color=POWER, bg=False, bold=True)
d.arrow([(990, 190), (990, 280)], POWER, dash=DS['power'], width=WD['power'], src='plug', dst='mio'); d.label(998, 231, '13.7 V', 'start', size=10.5, color=POWER, bg=False, bold=True)
# CAN
d.arrow([(1130, 190), (1130, 280)], CAN, dash=DS['can'], width=WD['can'], src='plug', dst='can', both=True); d.label(1138, 231, 'CAN', 'start', size=10.5, color=CAN, bg=False, bold=True)
d.arrow([(1160, 140), (1330, 140)], CAN, dash=DS['can'], width=WD['can'], src='plug', dst='vcu', both=True); d.label(1245, 133, 'vehicle CAN', size=10.5, color=CAN, bg=False, bold=True)
d.arrow([(1090, 325), (1050, 325)], DATA, src='can', dst='mio', both=True); d.label(1070, 318, 'USB', size=10, bg=False)
# lidar
d.arrow([(290, 335), (330, 335)], DATA, src='atx', dst='abx', both=True)
d.arrow([(560, 335), (730, 335)], DATA, src='abx', dst='mio', both=True)
d.label(645, 327, 'Ethernet: points', size=10.5, bg=False); d.label(645, 355, 'gPTP time', size=10.5, color=TIME, bg=False, bold=True)
# camera
d.arrow([(290, 480), (730, 480)], DATA, src='zed', dst='mio'); d.label(645, 473, 'USB 3: images', size=10.5, bg=False)
# MTi-8 kit
d.arrow([(290, 610), (730, 610)], DATA, src='mti', dst='mio'); d.label(645, 603, 'USB: IMU, INS, UTC stamps', size=10.5, bg=False)
d.arrow([(290, 660), (730, 660)], DATA, src='mti', dst='mio', both=True); d.label(645, 653, 'USB: F9P data, RTCM in', size=10.5, bg=False)
d.arrow([(290, 760), (330, 760)], TIME, dash=DS['time'], width=WD['time'], src='mti', dst='lvl'); d.label(310, 752, 'PPS', size=10, color=TIME, bg=False, bold=True)
d.arrow([(560, 760), (730, 760)], TIME, dash=DS['time'], width=WD['time'], src='lvl', dst='mio'); d.label(645, 752, 'PPS → COM1 DCD', size=10.5, color=TIME, bg=False, bold=True)
# option and RTK
if not BW:
    d.arrow([(1330, 420), (1050, 420)], DATA, dash='6 4', src='orin', dst='mio', both=True)
    d.label(1180, 413, 'LAN1: cone list', size=10.5, bg=False); d.label(1180, 441, 'PTP time', size=10.5, color=TIME, bg=False, bold=True)
d.arrow([(1090, 605), (1050, 605)], DATA, dash='6 4', src='lte', dst='mio'); d.label(1070, 598, 'USB', size=10, bg=False)
d.arrow([(1330, 605), (1270, 605)], RADIO, dash=DS['radio'], width=WD['radio'], src='base', dst='lte'); d.label(1300, 597, 'RTCM', size=10, color=RADIO, bg=False, bold=True)

# legend
ly = 854
x = 20
for col, dash, w, t in [(DATA, None, WD['data'], 'data'), (TIME, DS['time'], WD['time'], 'time (PPS, PTP)'), (CAN, DS['can'], WD['can'], 'CAN'), (POWER, DS['power'], WD['power'], 'power'), (RADIO, DS['radio'], WD['radio'], 'over the internet')]:
    da = f' stroke-dasharray="{dash}"' if dash else ''
    d.el.append(f'<line x1="{x}" y1="{ly - 4}" x2="{x + 34}" y2="{ly - 4}" stroke="{col}" stroke-width="{w}"{da}/>')
    d.text(x + 40, ly, t, 11.5); x += 40 + tw(t, 11.5) + 26
d.el.append(f'<rect x="{x}" y="{ly - 12}" width="30" height="15" rx="4" fill="#ffffff" stroke="{GREY}" stroke-width="1.5" stroke-dasharray="6 4"/>')
d.text(x + 38, ly, 'dashed box: good to have' if BW else 'dashed box: option or good to have', 11.5)
if not BW:
  d.text(20, 880, 'Arrowheads at both ends: both directions. One clock for everything: the F9P\'s PPS sets the computer\'s clock (chrony), the computer passes it to the lidar (gPTP), and the MTi-8 takes the same PPS on the kit.', 11.5, 'normal', GREY)
  d.text(20, 899, 'The ZED2 has no clock input: its frames are stamped on arrival in the computer, less a fixed delay found by calibration on the rig.', 11.5, 'normal', GREY)
d.save(sys.argv[1], sys.argv[2])
