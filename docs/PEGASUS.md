# Pegasus integration

Pegasus 5.1.0 supplies its stock Iris USD model, four-rotor thrust/drag dynamics,
IMU, GPS, barometer and magnetometer. Its first vehicle backend is the PX4
MAVLink backend, so rotor references come from PX4. This project does not use
Pegasus's direct ROS rotor-control backend.

The selected release's standalone API is initialized only after SimulationApp
loads Kit. Its documented world ownership pattern assigns `PegasusInterface`'s
world backing member; this version-specific dependency is contained in
isaac_sim/scripts/run_simulation.py. Keep it out of reusable ROS autonomy code.

PX4 runs separately through scripts/run_px4.sh with a persistent private instance
directory for parameters and flight logs. The simulation starts that process by
default. `--external-px4` supports a separate terminal; `--no-px4` is explicitly
a spawn-only diagnostic. Do not launch a second PX4 instance on the same ports.

Pegasus's setup.py modifies Isaac's base application file to enable an extension.
Instead, this project's standalone wrapper exposes the pinned source on
PYTHONPATH, installs its extra Python dependencies under `.local/isaac-python`,
and enables the required extensions in the launcher. This avoids modifying the
user's existing simulator or relying on global extension-manager settings.

Any integration fixes must be recorded as a tracked patch or a narrowly scoped
adapter. Never silently replace Pegasus with a kinematic vehicle and call the
result a physics flight test.
