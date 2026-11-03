"""Figure: how teams with one 120° lidar cover the inside of a tight hairpin in software (FaSTTUBe and ISC Racing code).

Run from figures/src (needs Pillow and cairosvg):  python fig_hairpin.py ../lidar_hairpin_software.svg ../lidar_hairpin_software.png
The layout check prints any text overflow, arrow through a box, or label overlap."""
import sys
from diag import D, LAY

W, BW, GAP, Y, BH = 1348, 220, 52, 58, 130
d = D(W, Y + BH + 22, 'How teams with one 120° lidar cover the inside of a tight hairpin in software')
C = {k: v[0] for k, v in LAY.items()}
d.text(20, 32, 'One 120° lidar through a tight hairpin: the inside edge comes from the map and the planner', 17, 'bold', '#111827')

boxes = [
    ('det', 'per', 'Lidar cones', 'every 120° team', ['120° view, ~20 m range', 'inside cones seen on', '  the approach only']),
    ('map', 'loc', 'Cone SLAM map', 'ISC', ['keeps every cone seen', 'new cones at a corner', '  exit are not a fault']),
    ('inp', 'pla', 'Planner input', 'ISC', ['map cones, not the scan', 'within 25 m', 'ahead of the car: 180°']),
    ('edg', 'pla', 'Edges + virtual cones', 'FaSTTUBe', ['each edge sorted alone', 'lone cone: virtual cone', '  3 m across', 'path cut at 12 m (ISC)']),
    ('ctl', 'pla', 'Speed and steering', 'ISC', ['lateral accel ≤ 3 m/s²', 'slows for R < 3 m', 'look-ahead ≥ 1 m']),
]
xs = [20 + i * (BW + GAP) for i in range(len(boxes))]
for (name, lay, title, tag, bullets), x in zip(boxes, xs):
    d.box(name, x, Y, BW, BH, lay, title, bullets, tag=tag)
mid = Y + BH / 2
for (a, *_), (b, *_), xa, xb, lab in zip(boxes, boxes[1:], xs, xs[1:], ['cones', 'map', 'cones', 'path']):
    d.arrow([(xa + BW, mid), (xb, mid)], '#4b5563', src=a, dst=b)
    d.label((xa + BW + xb) / 2, mid - 8, lab, size=10.5, bg=False)

d.save(sys.argv[1], sys.argv[2])
