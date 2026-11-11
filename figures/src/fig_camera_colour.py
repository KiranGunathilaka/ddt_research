"""Figure: cone colour from the ZED2 without an NVIDIA GPU (the lidar gives the position, the camera only the colour).

Run from figures/src (needs Pillow and cairosvg):  python fig_camera_colour.py ../camera_colour_no_gpu.svg ../camera_colour_no_gpu.png
The layout check prints any text overflow, arrow through a box, or label overlap."""
import sys
from diag import D

W, BW, GAP, BH = 1348, 226, 44, 132
Y0, Y1 = 58, 190
d = D(W, Y1 + BH + 22, 'Cone colour from the ZED2 without an NVIDIA GPU')
d.text(20, 32, 'Cone colour without an NVIDIA GPU: the lidar gives the position, the camera only the colour', 17, 'bold', '#111827')

xs = [20 + i * (BW + GAP) for i in range(5)]
row = [
    ('lid', 'per', 'Lidar cones', 'CPU', ['clusters at cone size', 'x, y, z in the car frame', 'one detector per lidar']),
    ('prj', 'per', 'Project into image', 'CPU', ['lidar to camera extrinsics', 'ZED2 factory intrinsics', 'pixel for each cone']),
    ('col', 'per', 'Colour of the patch', 'iGPU / NPU', ['crop around the pixel', 'HSV rule or small CNN', 'OpenVINO on the 255H']),
    ('cc', 'loc', 'Coloured cone', None, ['lidar position', 'colour probability', 'scan time stamp']),
    ('map', 'loc', 'Cone SLAM map', 'CPU', ['matched by position', 'colour votes added', '  to the map cone']),
]
for (name, lay, title, tag, bullets), x in zip(row, xs):
    d.box(name, x, Y1, BW, BH, lay, title, bullets, tag=tag)
d.box('cam', xs[2], Y0, BW, 82, 'veh', 'ZED2 left image', ['zed-open-capture, no CUDA', 'rectified, factory calibration'])

mid = Y1 + BH / 2
for (a, *_), (b, *_), xa, xb, lab in zip(row, row[1:], xs, xs[1:], ['cones', 'pixels', 'colour', 'cones']):
    d.arrow([(xa + BW, mid), (xb, mid)], '#4b5563', src=a, dst=b)
    d.label((xa + BW + xb) / 2, mid - 8, lab, size=10.5, bg=False)
cx = xs[2] + BW / 2
d.arrow([(cx, Y0 + 82), (cx, Y1)], '#4b5563', src='cam', dst='col')
d.label(cx + 8, (Y0 + 82 + Y1) / 2 + 4, 'image', anchor='start', size=10.5, bg=False)

d.save(sys.argv[1], sys.argv[2])
