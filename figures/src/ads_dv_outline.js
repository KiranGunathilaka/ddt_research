// Samples a coarse outline of the ADS-DV bodywork from the official IMechE STEP file and writes it
// into ads_dv_explorer.html (between the /*BODY*/ markers). The outline is a grid of top/bottom
// heights, not the CAD geometry itself: the CAD stays on IMechE's GitHub and the page loads it from there.
//
// Run (needs occt-import-js@0.0.23 somewhere on NODE_PATH):
//   NODE_PATH=<dir>/node_modules node --max-old-space-size=6000 figures/src/ads_dv_outline.js \
//     03_fsuk_ddt_ads-dv/official_fs-ai_github/FS-AI_ADS-DV_CAD/FS-AI_ADS-DV_CAD_2026.step ads_dv_explorer.html
//
// Page coordinates (mm): x forward from the wheelbase midpoint, y up from the ground, z to the right.
// CAD coordinates: x left, y rearward from the nose datum, z up; the ground is at CAD z = -49.
const fs = require('fs');

const [stepPath, htmlPath] = process.argv.slice(2);

require('occt-import-js')().then((occt) => {
  const res = occt.ReadStepFile(new Uint8Array(fs.readFileSync(stepPath)), {
    linearUnit: 'millimeter', linearDeflectionType: 'bounding_box_ratio', linearDeflection: 0.001, angularDeflection: 0.3,
  });
  const want = new Set(['BODYWORK_V4_NOSE', 'BODYWORK_V4_SIDES_2', 'BODYWORK_V4_TOP_2020 v7']);
  const tris = [];
  res.meshes.forEach((m) => {
    if (!want.has(m.name)) return;
    const p = m.attributes.position.array, idx = m.index.array;
    const P = (i) => [1593 - p[3 * i + 1], p[3 * i + 2] + 49, -p[3 * i]];
    for (let k = 0; k < idx.length; k += 3) tris.push([P(idx[k]), P(idx[k + 1]), P(idx[k + 2])]);
  });

  // Heights where a vertical line at (x, z) crosses the bodywork.
  const boxes = tris.map((t) => [Math.min(t[0][0], t[1][0], t[2][0]), Math.max(t[0][0], t[1][0], t[2][0]),
    Math.min(t[0][2], t[1][2], t[2][2]), Math.max(t[0][2], t[1][2], t[2][2])]);
  const hits = (x, z) => {
    const out = [];
    tris.forEach(([a, c, d], i) => {
      const b = boxes[i];
      if (x < b[0] || x > b[1] || z < b[2] || z > b[3]) return;
      const v0x = c[0] - a[0], v0z = c[2] - a[2], v1x = d[0] - a[0], v1z = d[2] - a[2], v2x = x - a[0], v2z = z - a[2];
      const den = v0x * v1z - v1x * v0z;
      if (Math.abs(den) < 1e-9) return;
      const u = (v2x * v1z - v1x * v2z) / den, v = (v0x * v2z - v2x * v0z) / den;
      if (u < -1e-6 || v < -1e-6 || u + v > 1 + 1e-6) return;
      out.push(a[1] + u * (c[1] - a[1]) + v * (d[1] - a[1]));
    });
    return out;
  };

  const xs = [];
  for (let x = 1485; x >= -1328; x -= 35) xs.push(x);
  xs.push(-1328);
  const zs = [];
  for (let z = 0; z <= 330; z += 10) zs.push(z);
  const top = [], bot = [];
  for (const x of xs) {
    const rt = [], rb = [];
    for (const z of zs) {
      const h = hits(x, z);
      rt.push(h.length ? Math.round(Math.max(...h)) : -1);
      rb.push(h.length ? Math.round(Math.min(...h)) : -1);
    }
    top.push(rt);
    bot.push(rb);
  }

  const html = fs.readFileSync(htmlPath, 'utf8');
  const json = JSON.stringify({ xs, zs, top, bot });
  const out = html.replace(/\/\*BODY\*\/[\s\S]*?\/\*BODY\*\//, `/*BODY*/${json}/*BODY*/`);
  if (out === html && !html.includes(json)) throw new Error('BODY markers not found in ' + htmlPath);
  fs.writeFileSync(htmlPath, out);
  console.log(`wrote outline: ${xs.length} stations x ${zs.length} bins from ${tris.length} triangles`);
});
