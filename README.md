# Falcon Driverless

Research archive and design notes for Falcon's Formula Student UK FS-AI entry (DDT class, on the IMechE ADS-DV).

Open `index.html` in a browser. It links every doc, and each doc has a bar to move between them. The same pages are public at https://kirangunathilaka.github.io/ddt_research/.

## Docs

Each doc is an `.html` file in the repository root with these in its `<head>`:

```html
<title>Short Name</title>
<meta name="description" content="One sentence for the index.">
<meta name="doc-section" content="Findings"> <!-- Plan, Findings, Tools or Reference -->
<meta name="date" content="2026-10-28">
```

After adding or changing a doc:

```sh
python3 tools/build_docs.py            # rebuild index.html and the bar on every doc
python3 tools/build_docs.py --publish  # push the public copy to the gh-pages branch
```

The public copy holds only the docs, their figures and the files listed on the index. Links to code go to upstream GitHub at the commit we studied, links to papers go to their public source, and anything else goes to this private repository.

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
- `git submodule status` lists the pinned commits; `sources.html` lists every source with its URL.

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
