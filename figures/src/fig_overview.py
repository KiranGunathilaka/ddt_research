"""Figure: Falcon DDT proposed architecture, one page: sensors -> perceive -> (map <-> localise) -> plan -> control -> ADS-DV, wheel speeds fed back.

The brochure version of the architecture. No algorithm names or rates: the methods are not fixed yet (control may be MPC, odometry may be LIO).
Run from figures/src (needs Pillow and cairosvg):  python fig_overview.py ../ddt_architecture_overview.svg ../ddt_architecture_overview.png
The layout check prints any text overflow, arrow through a box, or label overlap."""
import sys
from diag import D, LAY, tw

NAVY, BLUE, ORANGE, GREY, A = '#000c3d', '#002eef', '#b86e00', '#5b6170', '#374151'   # brochure colours
LAY['phy'] = (NAVY, '#eef0f6', '#ffffff')       # the physical world: sensors and the car
LAY['ai'] = (BLUE, '#eef2ff', '#f7f9ff')        # software blocks
LAY['slam'] = (ORANGE, '#fff3d6', '#fffaee')    # the map <-> localise loop

d = D(760, 632, 'Falcon DDT proposed architecture: perceive, map and localise, plan, control')

# sensors
d.box('atx', 20, 10, 220, 56, 'phy', 'Hesai ATX LiDAR', ['120° × 20°, 256 channels'])
d.box('zed', 254, 10, 190, 56, 'phy', 'ZED2 camera', ['colour, 110° wide'])
d.box('mti', 458, 10, 182, 56, 'phy', 'Xsens MTi-8', ['IMU + RTK GNSS, time'])

# the AI driver
d.el.append(f'<rect x="20" y="92" width="620" height="440" rx="14" fill="{LAY["ai"][2]}" stroke="{BLUE}" stroke-opacity="0.5" stroke-width="1.6"/>')
d.text(34, 113, 'AI DRIVER', 14, 'bold', BLUE)
d.text(34, 522, 'on the car computer: Advantech MIO-5379 · Core Ultra 7 255H · ROS 2', 11, 'normal', GREY)

d.box('per', 40, 122, 360, 68, 'ai', 'PERCEIVE', ['Create cone point clusters from the LiDAR', 'Acquire cone colors from the camera'])

d.el.append(f'<rect x="40" y="225" width="580" height="136" rx="10" fill="{LAY["slam"][2]}" stroke="{ORANGE}" stroke-opacity="0.55" stroke-width="1.4"/>')
d.text(52, 245, 'SLAM', 13, 'bold', ORANGE)
d.text(52 + tw('SLAM', 13, True) + 10, 245, 'map and localise at the same time', 11, 'normal', GREY)
d.box('map', 60, 257, 240, 90, 'slam', 'MAP', ['Estimated positions of the cones', '  on a common frame'])
d.box('loc', 390, 257, 210, 90, 'slam', 'LOCALISE', ['Estimating car position,', '  orientation in the same', '  common frame/map frame'])

d.box('pla', 40, 400, 240, 90, 'ai', 'PLAN', ['Estimating the path, racing line', '  through the cones'])
d.box('ctl', 380, 400, 240, 90, 'ai', 'CONTROL', ['Calculating optimum brake,', '  steering angle, throttle for', '  following the estimated path'])

# the car
d.box('car', 20, 562, 720, 56, 'phy', 'ADS-DV (IMechE shared car provided for the DDT entry candidates)',
      ['VCU: steering, motor, brakes', 'wheel speeds, steering angle'], cols=2)

# arrows
d.arrow([(150, 66), (150, 122)], A, src='atx', dst='per'); d.label(156, 110, 'points', 'start', size=11, bg=False)
d.arrow([(340, 66), (340, 122)], A, src='zed', dst='per'); d.label(346, 110, 'images', 'start', size=11, bg=False)
d.arrow([(560, 66), (560, 257)], A, src='mti', dst='loc'); d.label(554, 110, 'motion, position', 'end', size=11, bg=False)
d.arrow([(220, 190), (220, 225)], A, src='per'); d.label(226, 212, 'coloured cones', 'start', size=11, bg=False)
d.arrow([(390, 283), (300, 283)], ORANGE, width=2, src='loc', dst='map'); d.label(345, 276, 'pose', size=11, color=ORANGE, bg=False, bold=True)
d.arrow([(300, 320), (390, 320)], ORANGE, width=2, src='map', dst='loc'); d.label(345, 338, 'map cones', size=11, color=ORANGE, bg=False, bold=True)
d.arrow([(140, 361), (140, 400)], A, dst='pla'); d.label(146, 384, 'pose + cone map', 'start', size=11, bg=False)
d.arrow([(280, 445), (380, 445)], A, src='pla', dst='ctl'); d.label(330, 438, 'path', size=11, bg=False)
d.arrow([(500, 490), (500, 562)], NAVY, width=2, src='ctl', dst='car'); d.label(488, 551, 'steering, torque, brake (CAN)', 'end', size=11, bg=False)
# feedback from the car
d.arrow([(690, 562), (690, 302), (600, 302)], NAVY, width=2, src='car', dst='loc')
d.label(708, 432, 'wheel speeds, steering angle (CAN)', size=11, color=NAVY, rot=-90, bg=False)

d.save(sys.argv[1], sys.argv[2])
