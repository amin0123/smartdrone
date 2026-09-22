# Troubleshooting by failure boundary

Start with `source scripts/activate.sh` and `python scripts/doctor.py`. Check
PROJECT_STATUS.md for the exact locally qualified profile. An imported Python
module or successful build does not imply a simulator/aircraft milestone passed.

| Symptom | Evidence and remedy |
|---|---|
| `PX4 server already running for instance 0` | A duplicate or old SITL owns the instance. Confirm landed state, inspect the exact PID and stop that owned process. The simulator parent-death wrapper prevents recurrence after native shutdown. |
| `AMENT_TRACE_SETUP_FILES: unbound variable` | Native ROS setup scripts do not support Bash nounset. Project scripts that source ROS use `set -eo pipefail`; source activation without `set -u`. |
| Isaac 5.1 crashes in `librtx.scenedb.plugin.so` before scene creation | This host's R595/Kit 107.3.3 incompatibility. See COMPATIBILITY.md. Use the explicitly qualified Isaac 6 profile, or separately review an R580-open driver change/reboot. Do not reinstall ROS to fix a renderer crash. |
| `omni.isaac.*` / `World` imports missing in Isaac 6 | A Pegasus 5 script is being loaded into 6. Use the pinned PR-144 checkout and the 6 wrapper. |
| `_rclpy_pybind11` import or CPython ABI error | Host Jazzy is Python 3.12; Isaac 5 uses bundled 3.11. Use the launcher isolation boundary instead of mixing library paths. |
| XRCE Agent build: `invalid reference: 2.12.x` | Apply the tracked Fast DDS commit pin; do not choose an arbitrary new middleware major. |
| GCC 13 missing `uint64_t` or `-Warray-bounds` build failures | Use the documented portability patches/build scripts. Read the diagnostics; do not disable all warnings. |
| PX4 version parser or NuttX tag error | Fetch the pinned annotated release/metadata tags using `fetch_sources.py`. |
| QGC 4.4 fatal serial-permission dialog | This project uses stable 5.0.8 for UDP-only SITL. Serial group/modem changes are unnecessary here. |
| QGC reports missing RC calibration | Expected in this stick-free SITL profile. PX4's native arming checks still apply. Do not transfer these parameters to hardware. |
| QGC FTP metadata missing | Launch PX4 against the compiled `build/px4_sitl_default/etc` tree, which contains generated extras. |
| No PX4 state topics | Check UDP 8888 Agent, ROS domain, matching schemas, patched DDS export list and `logs/px4-runtime.log`. MAVLink/QGC and DDS are separate links. |
| Native takeoff stabilizes below requested altitude | PX4 accepts the target within `NAV_MC_ALT_RAD`; the project sets 0.2 m for the smoke test. Use consistent local/global altitude references. |
| Preflight waits forever on heading | `heading_good_for_control` in 1.14 means final in-flight magnetic alignment. Ground arming uses the actual preflight status and valid position/velocity. |
| Offboard loss despite 20 Hz ROS publication | Inspect PX4 ULog timestamps, not only `ros2 topic hz`. Slow SITL caused wall-clock time-sync drift; use the documented simulation `receive_time` timestamp mode. |
| Slow land-detection topic appears stale | It is published at 1 Hz in simulation time. The gateway uses ROS time for slow status and a separate wall watchdog for high-rate pose. |
| Image metadata arrives but raw RGB drops | Raw RGB is large. Both ends use reliable, shallow image queues for this local simulation. Match camera and subscriber QoS; best-effort subscribers may still drop frames. |
| Camera input rejected | Read the logged age/frame/calibration/pose reason. Check `/clock`, CameraInfo frame/dimensions, pose stream and topic QoS before changing thresholds. |
| No detections | Confirm actual RGB arrives, mission state is INSPECTION, calibrated pose association succeeds, and the HSV/min-area settings match saved rendered images. Ground truth labels are not detector input. |
| Mission reached READY with incomplete survey | Inspect RETURN_HOME transition reason; a fault return can legitimately dock and recharge. `mission_demo.py` requires `coverage_complete` for a normal pass. |
| Cannot rearm after LAND | PX4 may retain AUTO_LAND while disarmed. Preflight must stream and select OFFBOARD before checking readiness in that mode. |
| Charging does not occur | Verify fresh localized landed **and disarmed** state inside dock radius/height, CHARGING state, simulation clock progression, and `battery.source=simulated`. |

Logs: simulator output in `logs/isaac*.log`; PX4 console in
`logs/px4-runtime.log`; ROS launch output in `logs/ros-runtime.log`; native ULogs
under `.local/px4-instance/log`; mission SQLite/JPEG/JSON under `logs/inspection`.
Use `pyulog` in conda drone to inspect native modes, setpoints, failsafe flags and
estimator reset counters. Preserve the failing log before retrying.

Stop duplicate processes by their known PID, not a broad `pkill python`. Each
private PX4 instance uses TCP 4560 and UDP 8888/14550/14540 as documented. Stop
flight through the abort/landing path, confirm landed/disarmed, then close ROS
and the simulator. Killing the entire physics engine is not a landing test.
