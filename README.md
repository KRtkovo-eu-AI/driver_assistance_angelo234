# Advanced Driver Assistance Systems (ADAS) for BeamNG

**Version:** 1.35  
**Author:** angelo234  
**First Release:** March 31, 2021  
**Last Update:** April 8, 2025  
**Downloads:** Over 440,000 :contentReference[oaicite:0]{index=0}  

---

## Description

This mod adds advanced driver assistance systems (ADAS) commonly found in modern vehicles. It allows these systems to be installed on *any* vehicle in BeamNG (vanilla or modded). :contentReference[oaicite:1]{index=1}  

---

## Features

The current functionality includes:

- **Forward Automatic Emergency Braking (AEB)**  
  Applies full braking when a potential collision with another vehicle is detected. First alerts with an audible tone. Relies on road markings (AI paths) to check if the oncoming vehicle is in your path. :contentReference[oaicite:2]{index=2}

- **Rear Automatic Emergency Braking (Rear AEB)**  
  Works in reverse gear, up to ~25 km/h (15 mph). Detects both static objects and moving vehicles. Does *not* require lane markings. :contentReference[oaicite:3]{index=3}

- **Audible Rear Parking Sensors**  
  Provides an audible warning when reversing and approaching obstacles. The warning tones increase with proximity. :contentReference[oaicite:4]{index=4}

- **Adaptive Cruise Control (ACC)**  
  Maintains a set speed, but will reduce speed when detecting a vehicle ahead and maintain a safe following distance. Minimum selectable speed: ~30 km/h (20 mph). Can stop and resume if the vehicle in front stops and then accelerates. :contentReference[oaicite:5]{index=5}

- **Back-up Camera**  
  Rear-facing camera with wide field of view. Shows guiding lines relative to nearby objects (wall, curb) and, for some vehicles, a trajectory line showing where the car will move with current steering input. :contentReference[oaicite:6]{index=6}

- **Hill Start Assist**  
  When stopping on an incline, holds the brakes when you lift off the brake until you press the gas. If the brake is released for more than 3 seconds, or you activate throttle, brakes release automatically. :contentReference[oaicite:7]{index=7}

- **Automatic Headlight Dimming** *(Disabled for now)*  
  High beams are automatically dimmed when detecting another vehicle ahead. When no vehicles are detected, high beams return. :contentReference[oaicite:8]{index=8}

---

## Usage Instructions

1. **Installing the ADAS on vehicles:**

   - Use the included vehicle config files that already have ADAS installed. These vehicle configs have names with the suffix `w/ ADAS`. :contentReference[oaicite:9]{index=9}  
   - Or install manually:
     1. Open the *Vehicle Customization UI*.  
     2. Click on the *License Plate Design* slot. :contentReference[oaicite:10]{index=10}  
     3. Select the part called *“angelo234’s Driver Assistance System”*.  
     4. By default, all systems are added. To adjust, you can toggle individual systems via their slots (add or remove parts). :contentReference[oaicite:11]{index=11}  

2. **Using / activating systems:**

   - Forward AEB and Rear AEB will work automatically once installed (note: driver must press gas or brake to disengage when vehicle is fully stopped). :contentReference[oaicite:12]{index=12}  
   - Back-up Camera:
     - Put the vehicle in **neutral or reverse** gear.  
     - Press the **“8”** key (`“9”` for buses).  
     - When leaving reverse/neutral gear, camera switches back to last used camera. :contentReference[oaicite:13]{index=13}  

   - Adaptive Cruise Control (ACC):
     - You can enable by first setting the target speed and following distance, then enabling ACC.  
     - To disable: use the ACC toggle key or press the brake. :contentReference[oaicite:14]{index=14}  

   - Hill Start Assist:  
     - Vehicle should be on an incline (~4° or more), in first gear or drive (or reverse if rear is uphill), press brakes until you see “Hill Start Assist Activated.”  
     - Release occurs when accelerator is pressed or after 3 seconds. :contentReference[oaicite:15]{index=15}  

   - Automatic Headlight Dimming (if enabled later):  
     - Toggle via key binding. After enabling, when using high beams, system will auto-dim if other vehicles are detected. :contentReference[oaicite:16]{index=16}  

---

## Controls (Key Bindings)

- **‘8’** key (or **‘9’** for buses) — Toggle back-up camera when in neutral or reverse. :contentReference[oaicite:17]{index=17}  
- Additional controls exist for toggling other systems (ACC, AEB etc.) — see in-game parts screen / controls menu. :contentReference[oaicite:18]{index=18}  

---

## Notes & Limitations

- AEB systems depend on good brakes and tire grip. :contentReference[oaicite:19]{index=19}  
- If the game is not updated to the latest version, or if Lua scripts are cached, some features may not work correctly. In that case **press `Ctrl + L`** to reload Lua scripts. :contentReference[oaicite:20]{index=20}  
- False positives may occur (i.e. detecting things incorrectly). Time to time certain systems may fail depending on vehicle, terrain, or mod compatibility. :contentReference[oaicite:21]{index=21}  
- Back-up camera may not disappear immediately after leaving reverse or neutral — use camera switch manually. :contentReference[oaicite:22]{index=22}  

---

## FAQ

- **Q:** Other systems don’t work; only the back-up camera works.  
  **A:** Ensure you added the parts for those systems via the parts screen (License Plate Design slot). They are not automatically enabled unless included in vehicle config. :contentReference[oaicite:23]{index=23}  

- **Q:** What if the mod isn’t working properly?  
  **A:** Make sure BeamNG is up to date, and press **Ctrl + L** to reload Lua scripts. :contentReference[oaicite:24]{index=24}  

- **Q:** Can I equip other vehicles with these systems?  
  **A:** Yes — any vehicle (vanilla or modded) can be equipped. :contentReference[oaicite:25]{index=25}  

- **Q:** Is there a way to show the back-up camera on the infotainment display?  
  **A:** Not currently; due to engine limitations (cannot have multiple cameras in scene for that purpose). :contentReference[oaicite:26]{index=26}  

- **Q:** Will there be lane centering assist or similar features?  
  **A:** No plans at the moment. :contentReference[oaicite:27]{index=27}  

---

## Version History

- **v1.35** — April 8, 2025 (latest) :contentReference[oaicite:28]{index=28}  
- Earlier versions include fixes, optimizations and updates to adapt to BeamNG updates. :contentReference[oaicite:29]{index=29}  

---

## License & Credits

- Mod created by **angelo234**. :contentReference[oaicite:30]{index=30}  
- Thanks to community members who helped in beta testing: nick12345, ShaunJunior2006112, atomcrash07, crashitall. :contentReference[oaicite:31]{index=31}  
- If you use or modify this mod, please retain credit to the original author. :contentReference[oaicite:32]{index=32}  

---

## Acknowledgements

Special thanks to the BeamNG community for suggestions & feedback. :contentReference[oaicite:33]{index=33}  

---

Enjoy driving safer! 🚗  
