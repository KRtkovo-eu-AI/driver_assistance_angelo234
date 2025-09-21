# You may need to reload Lua (press Ctrl + L) if the mod is not working for you.

# Advanced Driver Assistance Systems Plus (ADAS+)

This mod adds advanced driver assistance systems you would typically find in today's modern vehicles. These systems can be added to any vehicle (vanilla and modded) and now include the expanded ADAS+ suite maintained by KRtekTM and angelo234.

Before I begin, I want to thank BeamNG for the existing game files/code, @nick12345, ShaunJunior2006112, @atomcrash07, and @crashitall for beta testing the mod, and the community for your ideas/feedback, which all have made this mod possible!

Also check out [Giani Beil's updated YouTube Player mod](https://www.beamng.com/resources/youtube-player.14769/).

**If you really enjoy what I make and want to show your support, you can donate [here](https://www.paypal.com/donate?business=WV4H4Q6KHXGPL&currency_code=USD).**

---

# Mod Showcase

**WhyBeAre**
[YouTube Video](https://www.youtube.com/watch?v=-6-CpZ3uPAI)

**BeamDriver**
[YouTube Video](https://www.youtube.com/watch?v=-nC5KpK5c4Y)

---

# Current Features (refer to "Usage Instructions" section below for usage)

## Automatic Emergency Braking (AEB)

![](https://www.beamng.com/attachments/aeb-gif-gif.786129/)

Automatic Emergency Braking (AEB) applies full braking power when it detects a possible collision with another vehicle. It is designed to mitigate or even prevent a collision from occurring. The system will first alert the driver of a possible collision with an audible tone before applying the brakes. The system uses the road markings (AI paths) to determine if an oncoming vehicle is in our vehicle's path.

**Disclaimer:** System will only work effectively if lane markings are present (technically if there are AI paths mapped).

## Rear Automatic Emergency Braking (Rear AEB)

Rear Automatic Emergency Braking (Rear AEB) is just like AEB except it applies the brakes when it detects a collision with both static objects and vehicles in reverse gear up to 25 km/h (15 mph). The system works without the need for lane markings.

## Audible Rear Parking Sensors

This system provides an audible alert as the vehicle approaches an obstacle in reverse. The faster the warning tone is played, the closer the vehicle is to the obstacle.

## Adaptive Cruise Control (ACC)

![](https://www.beamng.com/attachments/acc-gif-gif.797172/)

Adaptive Cruise Control works like a traditional cruise control system to maintain a set speed. The difference is when it detects a vehicle ahead, it will slow down to adapt to its speed and maintain a safe following distance apart. The minimum target speed you may select is 30 km/h (20 mph) but the system can stop and accelerate the vehicle if the vehicle in front comes to a stop and accelerates afterward.

## Back-up Camera

![](https://www.beamng.com/attachments/786112/)

The camera is attached to the rear of the vehicle with a wide FOV to allow the driver to see what is directly behind them. Depending on the vehicle, the camera can display guiding lines to show where the vehicle is relative to other objects (e.g. curb, wall) and where it will go in a straight line. And trajectory lines can also be displayed to show where the car will head with the current steering input.

## Hill Start Assist

When the driver takes their foot off the brakes to switch over to the gas pedal, the vehicle may roll back depending on the incline. This system eliminates this issue by holding the brakes until the driver hits the gas pedal. If the user is off the brake pedal for 3 seconds, the system will release the brakes.

## Automatic Headlight Dimming

This system automatically dims the high beams if it detects a vehicle up ahead. When no vehicles are in sight, the high beams will turn back on.

## Obstacle Collision Automatic Emergency Braking

This module fuses the forward radar array with the virtual LiDAR sweep to recognize static hazards in your lane. It escalates audible warnings, flashes the hazard lights, drops the throttle and then applies maximum braking when a collision is imminent, holding the brakes once stopped until you intervene.

## 12-phase LFO LiDAR

Equip the dedicated sensor pack to add a 12-phase virtual LiDAR that continuously scans the surroundings, improving object recognition for the Obstacle Collision system and Lane Centering Assist. The scan also powers point-cloud capture for external tools.

## Virtual LiDAR – PCD export & streaming

Capture the environment as a point cloud that you can either write to disk or stream over TCP. The export throttles itself to roughly four frames per second to reduce drive wear, and each point includes an intensity tag for easier filtering in external applications.

## Lane Centering Assist

Lane Centering Assist tracks the lane geometry and AI navigation data to blend gentle steering corrections. It arms when toggled on, automatically engages once lane data and speed conditions are met, and warns you when you drift toward the edge or override it with steering.

## Autopilot

Autopilot hands full control to the BeamNG AI once you set a navigation destination. It maintains the route, disengages if prerequisites are no longer met, and automatically yields to Lane Centering Assist when that system is active.

---

# Usage Instructions

## 1a. Use the included vehicle configs to use the ADAS systems

This mod adds vehicle configs with these ADAS installed on modern vehicles. The config names added are postfixed with **"w/ ADAS"**. Example:

![](https://www.beamng.com/attachments/797160/)

## 1b. Or install the systems manually

1. Open the Vehicle Customization UI and click on the "License Plate Design" slot.
   ![](https://www.beamng.com/attachments/790335/)

2. Find the "Driver Assistance System Plus" part and click on that.
   ![](https://www.beamng.com/attachments/790336/)

3. All systems are added by default. You can choose the systems to add/remove by clicking on the appropriate slots.
   ![](https://www.beamng.com/attachments/790337/)

---

## 2. Using the systems

After installing the systems, the core assists will be active (except Adaptive Cruise Control, Autopilot, Lane Centering Assist, Obstacle Collision AEB toggles, LiDAR output, and the back-up camera which require input).
**To toggle the systems on/off and use them with the default keybindings, refer to the "Controls" section.**

### Forward AEB
No action is required until the system completely stops the vehicle, in which case the driver must press either the gas or brake pedal.

### Rear AEB
No action is required until the system completely stops the vehicle, in which case the driver must press the brake pedal.

### Audible Rear Parking Sensors
No action is required here.

### Adaptive Cruise Control
Two ways to engage:

1. Increment/decrement ACC target speed
   Set following distance
   Enable ACC system

2. Bring vehicle up to desired speed
   Set ACC target speed
   Set following distance
   Enable ACC system

Deactivate by toggling the system off or pressing the brake pedal.

### Back-up Camera
Put vehicle in neutral or reverse, then press the **'8' key** (or '9' for bus).
When leaving reverse/neutral, camera switches back to last camera used.

### Hill Start Assist
Stop on incline (≥4°), car in 1st gear or Drive (Reverse if facing uphill), press brakes until message appears "Hill Start Assist Activated". Brakes release after throttle or 3s.

### Automatic Headlight Dimming
Toggle with **"Toggle Auto Headlight Dimming"** key. After enabling, switch to high beams – system auto-changes between low/high when traffic is detected.

### Obstacle Collision Automatic Emergency Braking
Toggle the assist via its Vehicle control binding. Once the sensor suite is installed it continuously watches for static obstacles, sounding escalating warnings, flashing hazards, cutting throttle and braking when impact is imminent.

### 12-phase LFO LiDAR & Virtual LiDAR output
Please follow the information on our [Github pages][(https://github.com/KRtkovo-eu-AI/driver_assistance_angelo234) README section and also check the [virtual-lidar-pcd.md](https://github.com/KRtkovo-eu-AI/driver_assistance_angelo234/blob/main/virtual-lidar-pcd.md) documentation.

### Lane Centering Assist
Requires the dedicated lane sensor hardware. Toggle the assist via its Vehicle control binding—once armed it engages automatically when lane data and speed criteria are met, and it disengages if you steer past the lane edge, leave the road, signal a maneuver or remove the hardware.

### Autopilot
Requires the Autopilot hardware slot and an active navigation target. Activate it via the Vehicle control binding and the AI will take control, automatically disengaging if the route is cleared, unresolved, hardware is removed or Lane Centering Assist is active.

---

# Controls

![](https://www.beamng.com/attachments/806284/)

- '8' key ('9' key for bus) → show back-up camera
- Vehicle control bindings → toggle Forward/Rear/Obstacle AEB, set Adaptive Cruise Control, arm Lane Centering Assist, start Autopilot, and enable LiDAR export/streaming

**Note:**
The AEB systems only work as well as your brakes and tires, plus this mod is still in beta so some systems may not work 100% of the time and false positives may occur. If things behave oddly after updating BeamNG, press **Ctrl + L** to reload Lua.

---

# FAQ

**Q: Only the back-up camera works, what do I do?**
A: Add other systems in the parts screen.

**Q: The mod isn't working properly, what do I do?**
A: Update to latest game version and press Ctrl + L to reload Lua.

**Q: How do I activate the back-up camera?**
A: Be in reverse/neutral, press '8' ('9' for bus). If other camera mods are installed, try other number keys.

**Q: Can I equip other vehicles with these systems?**
A: Yes!

**Q: Is it possible to put back-up camera on the infotainment display?**
A: Not possible currently – BeamNG doesn’t allow multiple cameras at once.

**Q: Lane centering assist when?**
A: It's here! Install the Lane Centering hardware and toggle it via the Vehicle controls.

**Q: Can I modify or reuse this mod’s code?**
A: Yes, just credit us. [GitHub repo](https://github.com/angelo234/driver_assistance_angelo234)

---

# Current Features

- Forward Automatic Emergency Braking (AEB)
- Rear Automatic Emergency Braking (Rear AEB)
- Back-up Camera
- Rear Parking Sensors
- Adaptive Cruise Control
- Ability to add systems to any vehicle
- Key bindings to enable/disable systems
- Automatic headlight dimming
- Hill Start Assist
- Obstacle Collision Automatic Emergency Braking
- 12-phase LFO LiDAR sensor pack
- Virtual LiDAR point cloud export & streaming
- Lane Centering Assist
- Autopilot

---

## Anyways enjoy! :)
