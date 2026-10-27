"""Small SVG block-diagram helper with layout checks (text fit, arrows through boxes, label overlaps)."""
from xml.sax.saxutils import escape
from PIL import ImageFont

FONT = "DejaVu Sans, Segoe UI, Helvetica, Arial, sans-serif"
_F = {}
def tw(s, size, bold=False):
    key = (size, bold)
    if key not in _F:
        _F[key] = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans%s.ttf' % ('-Bold' if bold else ''), size)
    return _F[key].getlength(s)

# layer palette: stroke, box fill, panel tint
LAY = {
    'hw':  ('#5b6170', '#f4f5f7', '#ffffff'),
    'veh': ('#6d28d9', '#f5f0ff', '#faf7ff'),
    'per': ('#0b5cad', '#ebf3fc', '#f6f9fe'),
    'loc': ('#1a7f4b', '#e8f5ee', '#f5fbf7'),
    'pla': ('#b45309', '#fdf2e4', '#fffaf3'),
    'sys': ('#334155', '#eef1f5', '#f8f9fb'),
    'sim': ('#9a3412', '#fff1ea', '#fff8f4'),
    'off': ('#9ca3af', '#ffffff', '#ffffff'),
    'net': ('#0e7490', '#e7f6f9', '#f4fbfc'),
}
GREY = '#6b7280'

class D:
    def __init__(s, W, H, title):
        s.W, s.H, s.title = W, H, title
        s.el, s.boxes, s.arrows, s.labels, s.colors = [], [], [], [], set()

    def text(s, x, y, t, size=12, weight='normal', fill='#1f2937', anchor='start', italic=False, rot=None):
        st = ' font-style="italic"' if italic else ''
        tr = f' transform="rotate({rot} {x} {y})"' if rot is not None else ''
        s.el.append(f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" font-weight="{weight}" fill="{fill}" text-anchor="{anchor}"{st}{tr}>{escape(t)}</text>')

    def panel(s, x, y, w, h, lay, title, sub=None):
        st, _, tint = LAY[lay]
        s.el.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="12" fill="{tint}" stroke="{st}" stroke-opacity="0.45" stroke-width="1.4"/>')
        s.text(x + 14, y + 22, title, 14, 'bold', st)
        if sub:
            s.text(x + 14 + tw(title, 14, True) + 12, y + 22, sub, 11.5, 'normal', GREY)

    def box(s, name, x, y, w, h, lay, title, bullets=(), rate=None, dashed=False, tag=None, size=12, cols=1, check=True):
        st, fill, _ = LAY[lay]
        da = ' stroke-dasharray="6 4"' if dashed else ''
        s.el.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="8" fill="{fill}" stroke="{st}" stroke-width="1.6"{da}/>')
        s.text(x + 11, y + 21, title, 13, 'bold', st)
        right = x + w - 10
        if rate:
            s.text(right, y + 21, rate, 11.5, 'bold', st, 'end')
            if check: assert tw(title, 13, True) + tw(rate, 11.5, True) + 30 < w, f'title+rate overflow: {name}'
        elif check:
            assert tw(title, 13, True) + 20 < w, f'title overflow: {name} ({tw(title, 13, True):.0f} > {w - 20})'
        ty = y + 21
        if tag:
            tx = x + 11
            ty += 8
            tww = tw(tag, 10.5, True) + 14
            s.el.append(f'<rect x="{tx}" y="{ty}" width="{tww:.1f}" height="17" rx="8.5" fill="{st}"/>')
            s.text(tx + tww / 2, ty + 12.5, tag, 10.5, 'bold', '#ffffff', 'middle')
            ty += 17
        per = -(-len(bullets) // cols)
        cw = (w - 22) / cols
        for i, b in enumerate(bullets):
            c, r = divmod(i, per)
            bx = x + 11 + c * cw
            by = ty + 19 + r * 16.5
            cont = b.startswith('  ')
            t = b.strip()
            col = '#9ca3af' if dashed and lay == 'off' else '#374151'
            if not cont:
                s.el.append(f'<circle cx="{bx + 3}" cy="{by - 4}" r="2.1" fill="{st}"/>')
            s.text(bx + 11, by, t, size, 'normal', col)
            if check:
                assert tw(t, size) + 11 + 8 < cw, f'bullet overflow in {name}: "{t}" {tw(t, size):.0f} > {cw - 19:.0f}'
        if check and bullets:
            assert ty + 19 + (per - 1) * 16.5 + 8 < y + h, f'box too short: {name} needs {ty + 19 + (per - 1) * 16.5 + 8 - y:.0f}, has {h}'
        s.boxes.append((name, x, y, x + w, y + h))
        return (x, y, w, h)

    def _marker(s, color):
        s.colors.add(color)
        return 'ah' + color.lstrip('#')

    def arrow(s, pts, color=GREY, dash=None, width=1.6, src=None, dst=None, both=False, dot_start=False):
        """Orthogonal polyline; src/dst name the boxes it may touch (checked)."""
        d = 'M ' + ' L '.join(f'{px},{py}' for px, py in pts)
        da = f' stroke-dasharray="{dash}"' if dash else ''
        m = s._marker(color)
        ms = f' marker-start="url(#{m})"' if both else ''
        s.el.append(f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{width}"{da} marker-end="url(#{m})"{ms}/>')
        if dot_start:
            s.el.append(f'<circle cx="{pts[0][0]}" cy="{pts[0][1]}" r="3.4" fill="{color}"/>')
        s.arrows.append((pts, src, dst))

    def label(s, x, y, t, anchor='middle', color='#374151', size=11, rot=None, bg=True, bold=False):
        w = tw(t, size, bold)
        if rot is None:
            x0 = x - w / 2 if anchor == 'middle' else (x - w if anchor == 'end' else x)
            if bg:
                s.el.append(f'<rect x="{x0 - 4:.1f}" y="{y - size + 1:.1f}" width="{w + 8:.1f}" height="{size + 4}" rx="3" fill="#ffffff" opacity="0.96"/>')
            s.labels.append((t, x0 - 4, y - size + 1, x0 + w + 4, y + 5))
        else:
            if bg:
                s.el.append(f'<rect x="{x - size + 1 - 2:.1f}" y="{y - w / 2 - 4:.1f}" width="{size + 6}" height="{w + 8:.1f}" rx="3" fill="#ffffff" opacity="0.96"/>')
            s.labels.append((t, x - size - 1, y - w / 2 - 4, x + 5, y + w / 2 + 4))
        s.text(x, y, t, size, 'bold' if bold else 'normal', color, anchor, rot=rot)

    def check(s):
        errs = []
        def inside(px, py, b, pad=0.5):
            _, x0, y0, x1, y1 = b
            return x0 + pad < px < x1 - pad and y0 + pad < py < y1 - pad
        for pts, src, dst in s.arrows:
            for (ax, ay), (bx, by) in zip(pts, pts[1:]):
                assert ax == bx or ay == by, f'non-orthogonal segment {pts}'
                for b in s.boxes:
                    # sample the segment; the boxes it connects may only be touched at the ends
                    n = int(max(abs(bx - ax), abs(by - ay)) // 2) + 1
                    for k in range(n + 1):
                        px, py = ax + (bx - ax) * k / n, ay + (by - ay) * k / n
                        if b[0] in (src, dst) and min(abs(px - q[0]) + abs(py - q[1]) for q in (pts[0], pts[-1])) < 9:
                            continue
                        if inside(px, py, b):
                            errs.append(f'arrow {src}->{dst} crosses box {b[0]} at {px:.0f},{py:.0f}')
                            break
            # endpoints must lie on the named boxes' edges
            for nm, (px, py) in ((src, pts[0]), (dst, pts[-1])):
                if nm:
                    b = next(bb for bb in s.boxes if bb[0] == nm)
                    _, x0, y0, x1, y1 = b
                    on = (x0 - 1 <= px <= x1 + 1 and y0 - 1 <= py <= y1 + 1) and (abs(px - x0) < 1.5 or abs(px - x1) < 1.5 or abs(py - y0) < 1.5 or abs(py - y1) < 1.5)
                    if not on:
                        errs.append(f'arrow {src}->{dst} end {px},{py} not on edge of {nm}')
        for t, x0, y0, x1, y1 in s.labels:
            for b in s.boxes:
                _, bx0, by0, bx1, by1 = b
                if x0 < bx1 and x1 > bx0 and y0 < by1 and y1 > by0:
                    errs.append(f'label "{t}" overlaps box {b[0]}')
        L = s.labels
        for i in range(len(L)):
            for j in range(i + 1, len(L)):
                a, b = L[i], L[j]
                if a[1] < b[3] and a[3] > b[1] and a[2] < b[4] and a[4] > b[2]:
                    errs.append(f'labels overlap: "{a[0]}" / "{b[0]}"')
        return errs

    def svg(s):
        defs = ''.join(f'<marker id="ah{c.lstrip("#")}" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7.5" markerHeight="7.5" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="{c}"/></marker>' for c in sorted(s.colors))
        return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{s.W}" height="{s.H}" viewBox="0 0 {s.W} {s.H}" font-family="{FONT}">\n'
                f'<title>{escape(s.title)}</title>\n<defs>{defs}</defs>\n<rect width="{s.W}" height="{s.H}" fill="#ffffff"/>\n'
                + '\n'.join(s.el) + '\n</svg>\n')

    def save(s, out_svg, out_png):
        errs = s.check()
        for e in errs:
            print('CHECK:', e)
        sv = s.svg()
        open(out_svg, 'w').write(sv)
        import cairosvg
        cairosvg.svg2png(bytestring=sv.encode(), write_to=out_png, output_width=s.W * 2)
        print('ok', out_svg, len(errs), 'issues')
