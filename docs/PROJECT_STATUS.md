# Project status

Updated 2026-09-22. **The complete autonomous mission has passed on the actual
Isaac 6 / Pegasus / PX4 simulation**, including RGB inspection, marker-assisted
landing, disarming, charging and a second mission from READY. The pinned Pegasus
6 port remains experimental. The original released 5.1 pairing remains blocked
by this host's driver. See [compatibility](COMPATIBILITY.md) and
[recorded evidence](evidence/README.md).

| Subsystem | Status | Verified result / scope |
|---|---|---|
| Machine and compatibility audit | DONE | Audit committed before installation; exact versions and sources documented. |
| Architecture, roadmap, repository | DONE | Clear simulation, flight-control, transport and autonomy ownership; logical Git commits. |
| Conda drone | DONE | Python 3.12.11, native Jazzy ABI and pinned dependencies; actual ROS nodes run in drone. |
| Isaac 5.1 / Pegasus 5.1 profile | BLOCKED | Renderer SIGSEGV on NVIDIA 595.84; details and alternatives below. |
| Experimental Isaac 6 / Pegasus PR 144 | DONE | Physical Iris, actual PX4 actuator loop, rendered field and calibrated RGB/depth. Local qualification only. |
| PX4 SITL and DDS interface | DONE | Native and offboard flight, ACKs/state, matching 10 wire schemas, lease watchdog. |
| QGroundControl 5.0.8 | DONE | Connected PX4 1.14.3 vehicle and telemetry visible. |
| Coordinate transforms | DONE | Named axes, attitudes, translated map origin and camera extrinsic tests; TF visualization. |
| Navigation | DONE | Speed-bounded deterministic coverage, static obstacle checks, actual full-row flight. Moving-obstacle avoidance is future work. |
| Mission state machine | DONE | Explicit reasons/states, preflight, arm, coverage, return, landing, charging, READY and abort. |
| RGB perception and records | DONE | 15 plant clusters, 11 healthy / 4 simulated stressed; images, SQLite and post-landing JSON. |
| Battery and safety | DONE | Low and critical battery flights; killed mission/gateway both recover to landed/disarmed. |
| Stage 1 and marker-assisted landing | DONE | Both actual flight gates pass; final known-station fallback remains enabled. |
| Simulated recharge / repeat sortie | DONE | 23.29% → 95.29%; subsequent full sortie from READY without simulator reset. |
| Observability | DONE | QGC/RViz, live RGB/route/odometry and a verified rosbag capture. |
| Documentation / migration / NIR design | DONE | Full runbook and subsystem explanations; hardware/NIR limits explicit. |
| Pause/resume qualification | DONE | 3 s wall-time pause triggered stale-state landing; confirmed landed/disarmed after resume. |
| Real aircraft, trained crop model, physical charging, realistic NIR | NOT STARTED | Deliberate future extensions; not represented as implemented simulation capabilities. |

## Milestone gates

All results below refer to the explicitly selected **experimental 6 profile**.
The 5.1 profile stops at its failed renderer gate; downstream results do not apply
to it. A unit test or ideal-vehicle contract test never advances a flight gate.

| # | Acceptance | Status | Evidence |
|---|---|---|---|
| 1 | Spawn physical Pegasus aircraft | DONE | Spawn log; stable resting Iris. |
| 2 | PX4 sensor/actuator connection | DONE | EKF initialization and actual aircraft response. |
| 3 | QGC connected vehicle | DONE | `logs/qgc5-after-dialog.png`. |
| 4 | Native PX4 arm/takeoff/land | DONE | `logs/flight-smoke.json`. |
| 5 | ROS receives PX4 state | DONE | Live DDS/gateway state; schema check. |
| 6 | ROS offboard flight | DONE | Actual complete missions and native ULogs. |
| 7 | Autonomous waypoint flight | DONE | Position evidence and route completion. |
| 8 | RGB/depth reaches ROS | DONE | Actual images, CameraInfo and bag records. |
| 9 | Detect synthetic inspection targets | DONE | Image-derived green/yellow detections. |
| 10 | Complete row route | DONE | `coverage_complete`; 15 clusters. |
| 11 | Low battery return | DONE | 25% injection → return → dock/charge. |
| 12 | Known-station autonomous landing | DONE | Stage 1 full mission. |
| 13 | Marker-assisted precision landing | DONE | Actual `visual_marker` descent control updates. |
| 14 | Charging to READY | DONE | Depleted-battery charge trace. |
| 15 | Full autonomous mission and repeat | DONE | Normal visual-docking mission after low-battery READY; failure trials also pass. |

## Current validation

* **44 automated tests passed**, no skips; **11 ROS packages built**.
* **10 checked PX4 message schemas match** the pinned firmware.
* Actual flights passed: native smoke, Stage 1 complete survey, low-battery visual
  return/recharge, full visual-docking survey, critical-battery landing, killed
  mission process and killed gateway. Raw data lives in ignored `logs/`;
  concise reviewable evidence is checked in under `docs/evidence/`.
* Depth publication and image/TF/route display were exercised. Depth navigation,
  dynamic avoidance, crop disease inference and hardware docking are not claimed.

## Change / failure log

This is a chronological development record. Earlier pending/failed entries are
superseded by later verified results and the current tables above.

* Audit: host already has Jazzy and Isaac 6.0 RC. Selected a separate Isaac 5.1
  install matching Pegasus instead of altering the existing simulator.
* System-level changes: none. Downloads, source builds and Python dependencies
  will remain in the repository's ignored directories and conda environment.
* PX4 first configuration failed: `CMakeLists.txt:129 list index: 1 out of
  range`, version printed as `1dacb4c..NOTFOUND`. Cause: shallow SHA-only fetch
  omitted the annotated v1.14.3 tag expected by PX4's version parser. Fix:
  fetch that exact tag without changing HEAD; make fetch script preserve it.
* Pure logic suite: 17 passed. These do not advance the flight milestone table.
* ROS interface build: `drone_core`, `drone_interfaces`, `px4_msgs` passed in
  conda drone; all 10 gateway wire definitions match the firmware sources.
* PX4 next build exposed `px_update_git_header.py:153 IndexError` because the
  shallow NuttX checkout has no version tags. Fetch the existing upstream
  `nuttx-10.3.0` metadata tag without changing its submodule commit. NuttX is not
  executed in SITL.
* Agent build failed `fatal: invalid reference: 2.12.x`. The upstream 2.4.2
  superbuild refers to a deleted Fast DDS branch. Apply the tracked one-line
  `patches/xrce-agent-fastdds-pin.patch` selecting official Fast DDS v2.12.2 by
  SHA. No protocol version or runtime code was changed. Rebuild pending.
* Host GCC 13 then exposed missing `<cstdint>` includes in PX4's POSIX shell and
  Fast DDS generated statistics code. Tracked header/build patches supply the
  standard header. PX4's optional legacy LPE also triggers `-Warray-bounds` in
  generic matrix inversion; retain this diagnostic as a warning using the one
  scoped `-Wno-error=array-bounds` flag. Stock EKF2 remains the active estimator.
* Inspection of firmware DDS configuration found landed, battery and command ACK
  outputs absent from the stock export list. Add these existing uORB topics via
  the supported `dds_topics.yaml` mechanism, using a tracked patch; no message
  schema changes are required. Rebuild is required before flight tests.
* Pure logic suite: 25 passed after mission FSM and independent safety policy.
* Build repairs verified: PX4 linked successfully; XRCE Agent installed locally
  and started on UDP 8888. The ROS gateway initializes in conda drone. No flight
  claim follows from these checks.
* Pure logic suite: 29 passed, including lease expiry/replay and dock calculations.
  One initial lease test used 1.4 < 14×0.1 due to floating representation; changed
  the observation time to 1.45, preserving the strict backward-time rejection.
* Isaac archive downloaded and ZIP integrity passed. The official 5.1.0 archive
  identifies its internal build as `5.1.0-rc.19+release.26219.9c81211b.gl`.
  Its vendor post-install script unexpectedly rewrote the user's application
  shortcut. Reset that shortcut to the existing `~/isaacsim` installation, and
  changed our installer to perform only the private examples symlink operation.
  Original custom shortcut content was not backed up; simulator files were not
  replaced. No driver/OS/system-package changes occurred.

* Pure logic suite: 36 passed, including camera geometry, RGB detector and
  persistent inspection serialization. These do not advance physical gates.
* **M1 BLOCKED:** official Isaac 5.1 exits with SIGSEGV in
  `librtx.scenedb.plugin.so` before field construction. Log:
  `logs/isaac-spawn.log`. Likely cause: installed R595 driver incompatibility
  with Kit 107.3.3, matching NVIDIA upstream diagnosis (COMPATIBILITY.md).
  Attempt: isolated official runtime and headless rendering. Next action:
  qualify the separate upstream Isaac 6 port, or review an R580 open driver
  change and reboot. No flight or downstream camera claim is made.

* Experimental Isaac 6 renderer startup passed (`logs/isaac6-renderer-smoke.log`).
  Pegasus PR 144 then spawned a physical Iris and local agricultural field for
  5 simulation seconds: final ENU (-0.0037, -0.0001, 0.0618), resting on dock.
  No callback errors: `logs/isaac6-spawn.log`. Only M1 advances for this profile.
* Port inspection found the same attitude timing defect documented by upstream
  PR 147: position from PhysX, rotation from less-frequent USD updates. The
  tracked `pegasus6-physics-attitude.patch` uses the quaternion from the same
  physics read and converts WXYZ to XYZW. Flight qualification follows.

* M2: PX4 receives Pegasus sensors, initializes EKF, produces actuator controls;
  native takeoff/landing move the physical aircraft. Agent exports state topics.
* M3 diagnostic: QGC 4.4.3 had a fatal serial-permission startup dialog, not a
  connected UI. Source inspection confirms this; switch to audited stable 5.0.8
  whose notification permits UDP SITL, without changing system groups.
* M4 first and second trials timed out on an altitude assertion. Native PX4
  takeoff switched to Hold at approximately 2.2 m, then stabilized near it.
  Source/ULog diagnosis: `NAV_MC_ALT_RAD` defaults to 0.8 m; the requested 3 m
  takeoff is considered complete at 2.2 m. An initial hypothesis about a home
  datum mismatch was disproved by inspecting the second ULog. Set this native
  acceptance parameter to 0.2 m and repeat the same 0.3 m test tolerance.
  The diagnostic now uses current global altitude and checks relative local
  climb explicitly. First timeout landed/disarmed; the second run hit its
  simulator duration limit during landing, so future flight trials run without
  a fixed simulator duration. Evidence remains in `logs/flight-smoke*.json` and
  `.local/px4-instance/log`. No M4 pass is claimed yet.
* All 11 ROS packages build in drone. Pure suite: 37 passed. Live mission
  integration and marker flight remain unverified.

* M4 repeat PASSED with `NAV_MC_ALT_RAD=0.2`: normal arm, requested takeoff
  altitude (within 0.3 m), native land and autonomous disarm all confirmed.
  `logs/flight-smoke.json`, `logs/flight-smoke.log`, `logs/isaac6-flight.log`.
  Six newly connected ROS node classes also initialized successfully in drone.

* M3: QGC 5.0.8 displayed the connected PX4 1.14.3 vehicle, sensors and native
  battery parameters (`logs/qgc5-after-dialog.png`). Its RC-setup warning is
  expected for this stick-free SITL profile. Using compiled PX4 `etc/` also
  supplies the FTP metadata absent from source ROMFS.
* M5/8: live DDS state and Isaac RGB (640x480) reached ROS. Initial preflight
  correctly inhibited arming on a misinterpreted EKF flag. PX4 1.14's
  `heading_good_for_control` requires final **in-flight** magnetic alignment.
  Gateway now exposes the actual `pre_flight_checks_pass` value and uses valid
  position/velocity for localization. Normal heading alignment resets are not
  interpreted as a change of position origin.
* M6 first trial hit native offboard-loss despite a 20 Hz stream. ULog showed
  Agent/PX4 time-offset drift at sub-real-time simulation speed. PX4 1.14 has no
  `UXRCE_DDS_SYNCT` parameter (introduced in 1.15). Its existing generated ucdr
  decoder treats timestamp zero as PX4 receive time. The simulation profile uses
  that supported path; ROS intent age/lease and expiring stream QoS remain active.
* M6/7/9/12: subsequent aircraft flew offboard to the field, recorded 33 RGB
  observations / 9 plant clusters (7 healthy, 2 stressed), then **perception_lost**
  triggered return. It landed/disarmed at the station and reached READY.
  Mission `a2e6fbb787894a998cc68ad51fdd611d`. Complete row coverage is NOT yet
  verified; camera interruption diagnosis continues. Charging states passed,
  but a substantial simulated recharge still needs a depleted-battery trial.
* Pure/ROS-contract suite: 40 passed, including complete executive, return and
  critical-fault paths using an explicitly ideal API test double on DDS domain
  93. These tests are not evidence of physical flight.

* **Complete Stage-1 mission PASSED** with reliable raw-image QoS at both ends.
  Mission `f060dc5bc0f8434b9b03384693f365ef`: all rows, 58 observations, all
  15 plant clusters, healthy/stressed classes, coverage-complete return, confirmed
  docking, actual battery increase and READY. Evidence:
  `logs/normal-mission-evidence.json` and mission `summary.json`.
  The best-effort camera diagnostic had received only 5 of 10 raw frames while
  CameraInfo continued at the full rate; the reliable stream resolved this trial.
  Marker-guided and depleted-battery flight trials remain next.


* M11/13/14 **PASSED** together. Injected 25% at inspection entry;
  `battery_return_reserve` caused return. The controller used `visual_marker`
  on 151/180 descent updates, landed/disarmed 0.0335 m from dock center, and
  charged from 23.29% to 95.29% / READY. Mission
  `77b10ea3f14941cc9660bbb4d9f4552e`; evidence
  `logs/precision-low-battery-evidence.json` and its `docking.jsonl`.
* Restart diagnosis: native Kit shutdown bypassed ordinary Python cleanup,
  leaving the previous PX4 child alive. The next PX4 reported `PX4 server already
  running for instance 0`. Removed that known, landed test process. A Linux
  parent-death wrapper now binds only the child we start to the simulator's
  lifetime; a dedicated process test passes. Simulator exceptions now retain
  traceback and exit code instead of being hidden by fast Kit shutdown.
* Automated suite now **41 passed**; 10 PX4 wire schemas match; all 11 ROS packages
  rebuild successfully. RViz initialized and displayed actual RGB/route/odometry;
  rosbag capture was exercised on the live system.


* M15 **PASSED** with marker refinement enabled: mission
  `324352cc1f764268a118802662df780b`, 56 observations / all 15 plants, 11 healthy
  and 4 stressed, complete coverage, visual approach, station landing/disarm,
  charge and READY. This was the next sortie after low-battery recharge without
  resetting the world. Final horizontal station error was 0.0516 m.
* Critical-battery test **PASSED**: 10% injected in takeoff, FAILSAFE →
  EMERGENCY_LANDING → MISSION_ABORTED after fresh landed/disarmed confirmation.
* Mission-node SIGKILL test **PASSED**: at 1.504 m, lease expiry requested native
  LAND; landed/autodisarmed in 36.83 wall seconds, horizontal drift 0.0347 m.
* Gateway SIGKILL test **PASSED**: PX4 native offboard-loss recovery landed and
  autodisarmed in 40.99 wall seconds, horizontal drift 0.0063 m. The observer
  subscribed directly to PX4 outputs; it did not rely on the dead gateway.
* Installer replay initially failed with `AMENT_TRACE_SETUP_FILES: unbound
  variable`. ROS setup is incompatible with Bash nounset; aligned the Pegasus 6
  installer with the existing ROS build wrappers, then verified installation
  and idempotent patch application successfully.
* Added explicit map-origin translation for a moved station/hardware map. Default
  zero translation preserves the qualified flight path; three additional tests
  cover translation, inverse and invalid values. Nonzero-origin flight remains
  a configuration-specific qualification, not an observed trial here.

* Pause/resume test **PASSED**: SIGSTOP for 3 wall seconds at 1.50 m caused
  `px4_or_ros_state_stale` and native LAND; after resume PX4 landed/disarmed and
  the mission recorded MISSION_ABORTED. Evidence: `logs/pause-evidence.json`.

* Final restart/cleanup **PASSED** after all source/setup changes: physical Iris,
  cameras and a fresh PX4 child ran for 5 simulation seconds, exited with status
  0, and left no PX4 or simulator process. Previous SIGINT shutdown also left no
  orphan PX4. Evidence: `logs/isaac6-final-restart.log`. ROS/Agent/RViz test
  processes were stopped after landed/disarmed confirmation; the QGC test GUI
  required a final forced close after ignoring termination signals. All project
  runtimes are now stopped so the documented first-launch commands can be used.
* Final source checks: all Python modules compile, shell wrappers pass `bash -n`,
  local documentation links resolve, Git whitespace checks pass. The final
  source build remains 11 packages; test result remains 44 passing tests.
