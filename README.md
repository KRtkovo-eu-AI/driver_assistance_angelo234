# Advanced Driver Assistance Systems Plus (ADAS+) for BeamNG

**Version:** 2.00 
**Author:** KRtekTM, angelo234  
**First Release:** March 31, 2021  
**Last Update:** September 22, 2025  

---

## Description

This mod adds advanced driver assistance systems (ADAS) commonly found in modern vehicles. It allows these systems to be installed on *any* vehicle in BeamNG (vanilla or modded).  It is an extended version of [ADAS mod from Angelo234](https://www.beamng.com/resources/advanced-driver-assistance-systems.17384/), so it is recommended to have only one of the modes installed and allowed simultaneously.

---

## Features

The current functionality includes:

- **Forward Automatic Emergency Braking (AEB)**  
  Applies full braking when a potential collision with another vehicle is detected. First alerts with an audible tone. Relies on road markings (AI paths) to check if the oncoming vehicle is in your path.

- **Rear Automatic Emergency Braking (Rear AEB)**  
  Works in reverse gear, up to ~25 km/h (15 mph). Detects both static objects and moving vehicles. Does *not* require lane markings.

- **Audible Rear Parking Sensors**  
  Provides an audible warning when reversing and approaching obstacles. The warning tones increase with proximity.

- **Adaptive Cruise Control (ACC)**  
  Maintains a set speed, but will reduce speed when detecting a vehicle ahead and maintain a safe following distance. Minimum selectable speed: ~30 km/h (20 mph). Can stop and resume if the vehicle in front stops and then accelerates.

- **Back-up Camera**  
  Rear-facing camera with wide field of view. Shows guiding lines relative to nearby objects (wall, curb) and, for some vehicles, a trajectory line showing where the car will move with current steering input.

- **Hill Start Assist**  
  When stopping on an incline, holds the brakes when you lift off the brake until you press the gas. If the brake is released for more than 3 seconds, or you activate throttle, brakes release automatically.

- **Automatic Headlight Dimming**
  High beams are automatically dimmed when detecting another vehicle ahead. When no vehicles are detected, high beams return.

- **Obstacle Collision Automatic Emergency Braking**  
  Uses the front radar array together with a virtual LiDAR sweep to recognise static hazards, sound escalating warnings, flash the hazards and apply maximum braking when a collision is imminent.

- **12-phase LFO LiDAR**  
  Virtual LiDAR performing 12-phase surrounding scanning, capturing also traffic and player vehicles. When equipped, it is used for increasing precision of the Obstacle Collision System and Lane Centering Assist. Also allows PCD export & streaming.

- **Virtual LiDAR – PCD export & streaming**  
  Captures the environment as a point cloud that can be exported to disk or streamed over TCP with intensity tags for easy filtering in external tooling.

- **Lane Centering Assist**  
  Tracks lane geometry and AI navigation data to blend steering corrections, warns as you drift toward the edge of the lane and chimes when the assist arms or disengages.

- **Autopilot**  
  Hands full control to the BeamNG AI driver once a navigation destination is selected, automatically disengaging if prerequisites are no longer met or if Lane Centering Assist is active.

---

## Virtual LiDAR – PCD export & streaming

> ⚠️ Repeatedly writing full point clouds can put noticeable load on your drive, so the module caps file updates at four per second. For the lowest latency prefer the TCP stream over polling the export file.

### Activation

Control the export and stream from the in-game console (``~``) with:

```lua
extensions.driver_assistance_angelo234.setVirtualLidarPcdExportEnabled(true)
extensions.driver_assistance_angelo234.setVirtualLidarPcdStreamEnabled(true)
```

Pass `false` to either function to turn the feature off. Enabling the export (or changing its path) prints the currently active location to the console, and you can set a custom file path with `extensions.driver_assistance_angelo234.setVirtualLidarPcdOutputPath('D:/scans/latest.pcd')`.

### Default file location

If you do not override the path, the module writes `latest.pcd` into `settings/krtektm_lidar` inside your BeamNG user profile (for example `C:\Users\{user}\AppData\Local\BeamNG.drive\current\settings\krtektm_lidar\latest.pcd`).

### Options (path, port, intensity)

- **Path** – adjust it with `setVirtualLidarPcdOutputPath(...)`; the module creates missing folders and stages writes through a temporary file when permitted (falling back to direct overwrites on sandboxed builds).
- **Stream port** – defaults to `23511` on `127.0.0.1`; change it with `setVirtualLidarPcdStreamPort(9000)`.
- **Intensity** – each point includes an `intensity` channel used to categorize samples: main scan (1.0), ground (0.2), and vehicle outline (0.8).

### Update frequency

The virtual LiDAR refreshes internally at 20 Hz, but exporting/streaming is throttled to at most one PCD frame every 0.25 s (≈4 Hz) to limit I/O overhead.

You can find full workflows and ready-to-run client examples in [docs/virtual-lidar-pcd.md](docs/virtual-lidar-pcd.md).

---

## Self-driving Assist Suite

### Obstacle Collision Automatic Emergency Braking

The obstacle mitigation module mixes front radar sensors with the optional LiDAR scan to assemble a dense picture of the road ahead. It continuously estimates the remaining stopping distance and, when a collision becomes unavoidable, drops the throttle, applies maximum braking, enables ABS, flashes the hazard lights and sounds a rapid warning tone. After stopping it keeps the brakes held until you press the pedal or throttle so the vehicle does not roll away.

### Lane Centering Assist

Lane Centering Assist relies on the dedicated sensor pack and AI lane data. When you toggle it on the system arms itself, waiting for sufficient speed and a valid lane model before fading in steering assistance. A double chime announces activation, and another plays if the driver overrides the assist or if it disengages because lane markings disappear, the vehicle leaves the road or you signal a manual manoeuvre. The assist can preview navigation nodes to anticipate turns, blends its steering request with your input, and warns when you drift toward the edge of the lane.

### Autopilot

Autopilot piggybacks on BeamNG’s navigation AI: once the hardware slot is installed and a destination is marked on the map, toggling the system hands full control to the AI driver. The module keeps the AI retargeted as the route updates and automatically disengages if the destination is cleared, the route cannot be resolved, the hardware is removed or Lane Centering Assist is active.

---

## Usage Instructions

1. **Installing the ADAS on vehicles:**

   - Use the included vehicle config files that already have ADAS installed. These vehicle configs have names with the suffix `w/ ADAS`.  
   - Or install manually:
     1. Open the *Vehicle Customization UI*.  
     2. Click on the *License Plate Design* slot.  
     3. Select the part called *“Driver Assistance System Plus”*.  
     4. By default, all systems are added. To adjust, you can toggle individual systems via their slots (add or remove parts).  

2. **Using / activating systems:**

   - Forward AEB and Rear AEB will work automatically once installed (note: driver must press gas or brake to disengage when vehicle is fully stopped).  
   - Back-up Camera:
     - Put the vehicle in **neutral or reverse** gear.  
     - Press the **“8”** key (`“9”` for buses).  
     - When leaving reverse/neutral gear, camera switches back to last used camera.  

   - Adaptive Cruise Control (ACC):
     - You can enable by first setting the target speed and following distance, then enabling ACC.  
     - To disable: use the ACC toggle key or press the brake.  

   - Hill Start Assist:  
     - Vehicle should be on an incline (~4° or more), in first gear or drive (or reverse if rear is uphill), press brakes until you see “Hill Start Assist Activated.”  
     - Release occurs when accelerator is pressed or after 3 seconds.  

   - Automatic Headlight Dimming (if enabled later):
     - Toggle via key binding. After enabling, when using high beams, system will auto-dim if other vehicles are detected.

   - Obstacle Collision Automatic Emergency Braking:
     - Monitors for static obstacles ahead once the sensor suite is installed. The assist can be toggled via the Vehicle control binding and will flash the hazard lights while braking.

   - Lane Centering Assist:
     - Requires the dedicated lane sensor part. Toggle the assist via the Vehicle control binding—once armed it engages automatically when lane data and speed criteria are met, and it disengages if you steer past the lane edge or leave the road.

   - Autopilot:
     - Requires a navigation target on the world map and the Autopilot hardware slot. Activate it with the Vehicle control binding; the system relinquishes control automatically if the destination is cleared or Lane Centering Assist is active.

   - Virtual LiDAR:
     - Enable export or streaming from the in-game console using the commands listed in the section below. Configure the output path or TCP port as needed for external tools.

---

## Controls (Key Bindings)

- **‘8’** key (or **‘9’** for buses) — Toggle back-up camera when in neutral or reverse.  
- Vehicle control bindings are provided for toggling the collision mitigation systems (forward, reverse, obstacle), adjusting Adaptive Cruise Control, and enabling Lane Centering Assist or Autopilot—configure them under **Options → Controls → Vehicle**.

---

## Notes & Limitations

- AEB systems depend on good brakes and tire grip.  
- If the game is not updated to the latest version, or if Lua scripts are cached, some features may not work correctly. In that case **press `Ctrl + L`** to reload Lua scripts.  
- False positives may occur (i.e. detecting things incorrectly). Time to time certain systems may fail depending on vehicle, terrain, or mod compatibility.  
- Back-up camera may not disappear immediately after leaving reverse or neutral — use camera switch manually.  

---

## FAQ

- **Q:** Other systems don’t work; only the back-up camera works.  
  **A:** Ensure you added the parts for those systems via the parts screen (License Plate Design slot). They are not automatically enabled unless included in vehicle config.  

- **Q:** What if the mod isn’t working properly?  
  **A:** Make sure BeamNG is up to date, and press **Ctrl + L** to reload Lua scripts.  

- **Q:** Can I equip other vehicles with these systems?  
  **A:** Yes — any vehicle (vanilla or modded) can be equipped.  

- **Q:** Is there a way to show the back-up camera on the infotainment display?  
  **A:** Not currently; due to engine limitations (cannot have multiple cameras in scene for that purpose).  

- **Q:** Will there be lane centering assist or similar features?  
  **A:** Yes, it's already implemented.

---

## Version History

- **angelo234's ADAS mod v1.35** — April 8, 2025 (the base code)  
- Earlier versions include fixes, optimizations and updates to adapt to BeamNG updates.  
- **KRtekTM's ADAS+ mode v2.00** - September 22, 2025 (latest)
- Introduced Self-driving system: LiDAR, Obstacle Collision System, Lane Centering Assist, Autopilot

---

## License & Credits

- Mod created by **KRtekTM**, extended version based on original code of **angelo234**.  
- Beta testers of the original mod: nick12345, ShaunJunior2006112, atomcrash07, crashitall.  
- If you use or modify this mod, please retain credit to the original author.  

---

## Acknowledgements

Special thanks to the BeamNG community for suggestions & feedback.  

---

Enjoy driving safer! 🚗

---

## Development & Testing

The repository includes automated specs written for the [Laura](https://github.com/dknight/laura) Lua test framework. Install
the runner once per machine, then execute the helper script in this repo to run all specs.

### Installing Laura

Choose one of the official installation paths:

- **LuaRocks (system-wide):**

  ```bash
  luarocks install laura
  ```

- **LuaRocks (per-user tree):**

  ```bash
  luarocks --local install laura
  ```

- **Build from source with `make`:**

  ```bash
  git clone https://github.com/dknight/laura.git
  cd laura
  make install
  # Optional install prefix overrides:
  # PREFIX=/opt/lua/libs BINDIR=/opt/bin LIBDIR=/opt/share make install
  ```

### Running the specs

After installing Laura (and ensuring the `lua` interpreter is on your `PATH`), run the test suite from the repository root with:

```bash
lua .scripts/run_laura_tests.lua spec reports/laura.xml reports/laura_summary.json
```
