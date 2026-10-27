# Falcon Driverless

Research archive and design notes for Falcon's Formula Student UK FS-AI entry (DDT class, on the IMechE ADS-DV).

Open these in a browser:

- `sensors_and_autonomy_stack.html` — which sensors and computer to fit, and the autonomy stack we plan to build
- `ddt_architecture_and_testing.html` — system architecture, vehicle profiles and test plan; FS-AI results, how teams without a car did it, sensors, cones, the ADS-DV vehicle model and landed cost in LKR
- `xlsx/ddt_essential_costs.xlsx` — essential parts with landed cost, power budget, Jetson vs PC, FSG x86 teams
- `ddt_compute_options.html` — which computer to fit to the ADS-DV without the Orin: seven options compared, landed cost in LKR
- `ads_dv_explorer.html` — interactive 3D ADS-DV with see-through, exploded and cut-away views: every plug, switch, internal unit and colour-coded cable, plus dimensions, CAN interface and rules, each linked to its source; self-contained, so it can be shared on its own (the official CAD loads from IMechE's GitHub on request; `figures/src/ads_dv_outline.js` rebuilds the simplified model's outline)
- `index.html` — inventory of everything collected here, with source URLs and commits (`sources.json` is the same data)

## Layout

| Folder | Contents |
| --- | --- |
| `01_fsd_open_source_code/` | Snapshots of open-source Formula Student Driverless code: team stacks, individual modules, simulators and datasets |
| `02_fsd_papers/` | Papers and theses, grouped by team |
| `03_fsuk_ddt_ads-dv/` | FS-AI rules, ADS-DV documents, datasheets and community CAN code |
| `tools/` | Repository housekeeping |

Keep this repository private. It holds third-party code under mixed licences and copyrighted papers.

## Cloning

Files of 1 MiB or more are in Git LFS (about 7 GiB). Install [Git LFS](https://git-lfs.com) first.

To clone without downloading them, then fetch only what you need:

```sh
GIT_LFS_SKIP_SMUDGE=1 git clone <url>
git lfs pull --include "02_fsd_papers/**"
```

The five official FS-AI repositories are not stored here. Fetch them into place with:

```sh
mkdir -p 03_fsuk_ddt_ads-dv/official_fs-ai_github && cd 03_fsuk_ddt_ads-dv/official_fs-ai_github
for r in FS-AI_API FS-AI_ADS-DV_Documentation FS-AI_IMechE_ADS-DV_HiL FS-AI_ADS-DV_CAD FS-AI_Compute; do
  git clone https://github.com/FS-AI/$r.git
done
```

## Adding material

Run this before `git add`, from the repository root:

```sh
python3 tools/lfs_attributes.py
```

It routes new large files to LFS and ignores LFS placeholder files left in third-party snapshots, which would otherwise break pushes.
