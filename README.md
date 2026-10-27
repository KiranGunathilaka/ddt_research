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
| `01_fsd_open_source_code/` | Open-source Formula Student Driverless code as Git submodules: team stacks, individual modules, simulators and datasets |
| `02_fsd_papers/` | Papers and theses, grouped by team |
| `03_fsuk_ddt_ads-dv/` | FS-AI rules, ADS-DV documents, datasheets, community CAN code and the official FS-AI repositories (submodules) |
| `tools/` | Repository housekeeping |

Keep this repository private. It holds copyrighted papers and links to third-party code under mixed licences.

## Setup

Install Git and [Git LFS](https://git-lfs.com), and have SSH access to the FalconE-Racing organisation. Then:

```sh
git lfs install
git clone git@github.com:FalconE-Racing/ddt_research.git
cd ddt_research
```

That brings the pages, papers and DDT documents (about 0.8 GiB, mostly LFS). To skip the LFS files and pull only some, clone with `GIT_LFS_SKIP_SMUDGE=1` and then run `git lfs pull --include "02_fsd_papers/**"`.

The code is 398 submodules, each pinned to the commit we studied. None are downloaded until you ask. Fetch only what you need, shallow:

```sh
export GIT_LFS_SKIP_SMUDGE=1
git submodule update --init --depth 1 -- 03_fsuk_ddt_ads-dv
git submodule update --init --depth 1 -- 01_fsd_open_source_code/team_stacks/AMZ_ETH-Zurich
```

Or fetch all of it (about 15 GiB on disk):

```sh
GIT_LFS_SKIP_SMUDGE=1 git submodule update --init --depth 1 --jobs 8
```

- `GIT_LFS_SKIP_SMUDGE=1` leaves each upstream repo's own LFS files as small pointer files. Without it, a repo whose LFS objects are missing or over quota upstream fails to check out.
- Leave out `--recursive`: the upstream submodules we studied are already here as their own entries.
- `git submodule status` lists the pinned commits; `index.html` lists every source with its URL.

Four repos are stored here as plain files because upstream deleted them or rewrote their history: `Carnegie-Mellon-Racing/driverless-ml-dev`, `Bristol-FS-AI/bristol_fsai_sim-docker`, `Bristol-FS-AI/bristol_fsai_sim2-docker` and `individual_modules_and_theses/bisharahakim__fs_autonomous_controller`.

## Adding material

Add code from a Git repository as a shallow submodule, from the repository root:

```sh
git submodule add --depth 1 <url> <path>
git config -f .gitmodules submodule.<path>.shallow true
```

For other files, run this before `git add`:

```sh
python3 tools/lfs_attributes.py
```

It routes files of 1 MiB or more to LFS and ignores LFS placeholder files left in third-party code, which would otherwise break pushes.
