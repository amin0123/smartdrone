# Installation and isolation

Read [COMPATIBILITY.md](COMPATIBILITY.md) first. The repository uses ignored
`.local/` for binary installations and `vendor/` for immutable external source.
No `sudo`, driver replacement, shell-profile edits or global pip are part of the
automated setup. Preserve the host's existing ROS Jazzy installation.

## Development environment

```bash
cd /path/to/smartdrone
conda env create -f environment.yml
source scripts/activate.sh
python --version                 # 3.12.11, environment drone
python scripts/fetch_sources.py
```

Run project Python and build commands from this activated shell. `activate.sh`
sources Jazzy and the built workspace when available. It does not modify your
login profile. Conda provides development dependencies; ROS native libraries
are provided by the already installed `/opt/ros/jazzy`. The exact same Python
minor version matters for rclpy's binary ABI.

## Simulator

```bash
bash scripts/install_isaac.sh
```

This downloads the approximately 8.8 GB 5.1.0 archive, checks ZIP integrity,
records its SHA256 and extracts to `.local/isaac-sim-5.1.0`. Allow room for both
archive and extracted files. A recorded local hash identifies this download;
it is not an independently published NVIDIA signature. HTTPS and upstream
versioned release locations are the source of provenance.

The simulator launcher deliberately invokes the vendor's bundled Python 3.11.
Host ROS bindings cannot be imported into it. Its internal ROS bridge publishes
standard sensor messages over DDS to external Jazzy nodes. Never pip-install
Isaac into the `drone` Python 3.12 environment.

Further build, first-flight and mission commands are added as each milestone
is verified. See PROJECT_STATUS.md for the current tested boundary.
