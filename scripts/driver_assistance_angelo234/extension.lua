-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

--global table of all vehicles acceleration vectors
--veh_accs_angelo234[id][1] = lateral (-x = right, +x = left)
--veh_accs_angelo234[id][2] = longitudinal (-x = accelerating, +x = braking)
--veh_accs_angelo234[id][3] = up/down direction (-x = down, +x = up)
veh_accs_angelo234 = {}

local M = {}

local extra_utils = require('scripts/driver_assistance_angelo234/extraUtils')

local sensor_system = require('scripts/driver_assistance_angelo234/sensorSystem')
local fcm_system = require('scripts/driver_assistance_angelo234/forwardCollisionMitigationSystem')
local rcm_system = require('scripts/driver_assistance_angelo234/reverseCollisionMitigationSystem')
local acc_system = require('scripts/driver_assistance_angelo234/accSystem')
local hsa_system = require('scripts/driver_assistance_angelo234/hillStartAssistSystem')
local auto_headlight_system = require('scripts/driver_assistance_angelo234/autoHeadlightSystem')
local logger = require('scripts/driver_assistance_angelo234/logger')
local obstacle_aeb_system = require('scripts/driver_assistance_angelo234/obstacleBrakingSystem')
local lane_assist_system = require('scripts/driver_assistance_angelo234/laneAssistSystem')
local virtual_lidar = require('scripts/driver_assistance_angelo234/virtualLidar')

local first_update = true

local system_params = nil
local aeb_params = nil
local rev_aeb_params = nil
local parking_lines_params = nil
local beeper_params = nil

local fcm_system_on = true
local rcm_system_on = true
local auto_headlight_system_on = false
local prev_auto_headlight_system_on = false
local acc_system_on = false
local obstacle_aeb_system_on = true
local lane_assist_system_on = true

local front_sensor_data = nil
local rear_sensor_data = nil

local other_systems_timer = 0
local hsa_system_update_timer = 0
local auto_headlight_system_update_timer = 0
local virtual_lidar_update_timer = 0
local VIRTUAL_LIDAR_PHASES = 8
local virtual_lidar_point_cloud = {}

local function resetVirtualLidarPointCloud()
  virtual_lidar_point_cloud = {}
  for i = 1, VIRTUAL_LIDAR_PHASES do
    virtual_lidar_point_cloud[i] = {}
  end
end

resetVirtualLidarPointCloud()
local virtual_lidar_phase = 0

M.curr_camera_mode = "orbit"
M.prev_camera_mode = "orbit"

local function init(player)
  local veh = be:getPlayerVehicle(player)

  if not veh then return end

  local veh_name = veh:getJBeamFilename()

  local default_param_file_dir = 'vehicles/common/parameters'
  local param_file_dir = 'vehicles/' .. veh_name .. '/parameters'

  if FS:fileExists(param_file_dir .. ".lua") then
    --load parameter lua file dependent on vehicle
    system_params = require(param_file_dir)
  else
    --use default parameters if they don't exist for current vehicle
    system_params = require(default_param_file_dir)
  end

  aeb_params = system_params.fwd_aeb_params
  beeper_params = system_params.beeper_params
  rev_aeb_params = system_params.rev_aeb_params
  parking_lines_params = system_params.rev_cam_params.parking_lines_params

  -- enable auto headlight system by default when the part is installed
  if extra_utils.getPart("auto_headlight_angelo234") then
    auto_headlight_system_on = true
    prev_auto_headlight_system_on = false
  else
    auto_headlight_system_on = false
    prev_auto_headlight_system_on = false
  end
end

local function onExtensionLoaded()
  init(0)
end

local function onVehicleSwitched(_oid, _nid, player)
  init(player)
end

local function onHeadlightsOff()
  auto_headlight_system.onHeadlightsOff()
end

local function onHeadlightsOn()
  auto_headlight_system.onHeadlightsOn()
end

--Functions called with key binding
local function toggleFCMSystem()
  if not extra_utils.getPart("forward_collision_mitigation_angelo234") then return end

  fcm_system_on = not fcm_system_on
  local state = fcm_system_on and "ON" or "OFF"
  ui_message("Forward Collision Mitigation System switched " .. state)
end

local function toggleRCMSystem()
  if not extra_utils.getPart("reverse_collision_mitigation_angelo234") then return end

  rcm_system_on = not rcm_system_on
  local state = rcm_system_on and "ON" or "OFF"
  ui_message("Reverse Collision Mitigation System switched " .. state)
end

local function toggleObstacleAEBSystem()
  if not extra_utils.getPart("obstacle_collision_mitigation_angelo234") then return end

  obstacle_aeb_system_on = not obstacle_aeb_system_on
  local state = obstacle_aeb_system_on and "ON" or "OFF"
  ui_message("Obstacle Collision Mitigation System switched " .. state)
end

local function toggleLaneAssistSystem()
  if not extra_utils.getPart("lane_assist_angelo234") then return end

  lane_assist_system_on = not lane_assist_system_on
  local state = lane_assist_system_on and "ON" or "OFF"
  ui_message("Lane Assist System switched " .. state)
end

local function toggleAutoHeadlightSystem()
  if not extra_utils.getPart("auto_headlight_angelo234") then return end

  auto_headlight_system_on = not auto_headlight_system_on

  local msg = nil

  if auto_headlight_system_on then
    msg = "ON"
  else
    msg = "OFF"
  end

  ui_message("Auto Headlight Dimming switched " .. msg)
end

local function setACCSystemOn(on)
  if not extra_utils.getPart("acc_angelo234") then return end

  if acc_system_on ~= on then
    acc_system_on = on

    acc_system.onToggled(acc_system_on)
  end
end

local function toggleACCSystem()
  if not extra_utils.getPart("acc_angelo234") then return end

  acc_system_on = not acc_system_on

  acc_system.onToggled(acc_system_on)
end

local function setACCSpeed()
  if not extra_utils.getPart("acc_angelo234") then return end

  acc_system.setACCSpeed()
end

local function changeACCSpeed(amt)
  if not extra_utils.getPart("acc_angelo234") then return end

  acc_system.changeACCSpeed(amt)
end

local function changeACCFollowingDistance(amt)
  if not extra_utils.getPart("acc_angelo234") then return end

  acc_system.changeACCFollowingDistance(amt)
end

local function toggleDebugLogging()
  local enabled = logger.toggle()
  local msg = enabled and "enabled" or "disabled"
  ui_message("Driver assistance logs " .. msg)
end

local function toggleFrontSensorLogging()
  local enabled = logger.toggleSensor('front_sensor')
  local msg = enabled and "enabled" or "disabled"
  ui_message("Front sensor logs " .. msg)
end

local function toggleRearSensorLogging()
  local enabled = logger.toggleSensor('rear_sensor')
  local msg = enabled and "enabled" or "disabled"
  ui_message("Rear sensor logs " .. msg)
end

local function toggleLidarLogging()
  local enabled = logger.toggleSensor('lidar')
  local msg = enabled and "enabled" or "disabled"
  ui_message("Lidar logs " .. msg)
end

local function isPlayerVehicle(veh)
  local i = 0
  while true do
    local playerVeh = be:getPlayerVehicle(i)
    if playerVeh == nil then break end
    if playerVeh:getID() == veh:getID() then return true end
    i = i + 1
  end
  return false
end

--Used for what camera to switch the player to when the player gets out of reverse gear using reverse camera
local function onCameraModeChanged(new_camera_mode)
  if new_camera_mode ~= M.curr_camera_mode then
    M.prev_camera_mode = M.curr_camera_mode
    M.curr_camera_mode = new_camera_mode
  end
end

local function getAllVehiclesPropertiesFromVELua(my_veh)
  for i = 0, be:getObjectCount() - 1 do
    local this_veh = be:getObject(i)
    local id = this_veh:getID()

    local acc_cmd =
      'obj:queueGameEngineLua("veh_accs_angelo234[' .. id .. '] = {" .. sensors.gx2 .. ",' ..
      '" .. sensors.gy2 .. "," .. sensors.gz2 .. "}")'
    this_veh:queueLuaCommand(acc_cmd)
  end

  --Get properties of my vehicle
  local throttle_cmd =
    "if input.throttle ~= nil then obj:queueGameEngineLua('input_throttle_angelo234 = ' .. input.throttle ) end"
  my_veh:queueLuaCommand(throttle_cmd)
  local brake_cmd =
    "if input.brake ~= nil then obj:queueGameEngineLua('input_brake_angelo234 = ' .. input.brake ) end"
  my_veh:queueLuaCommand(brake_cmd)
  local clutch_cmd =
    "if input.clutch ~= nil then obj:queueGameEngineLua('input_clutch_angelo234 = ' .. input.clutch ) end"
  my_veh:queueLuaCommand(clutch_cmd)
  local parking_cmd =
    "if input.parkingbrake ~= nil then obj:queueGameEngineLua('input_parkingbrake_angelo234 = ' .. " ..
    "input.parkingbrake ) end"
  my_veh:queueLuaCommand(parking_cmd)

  local electrics_cmd =
    'obj:queueGameEngineLua("electrics_values_angelo234 = (\'" .. jsonEncode(electrics.values) .. "\')")'
  my_veh:queueLuaCommand(electrics_cmd)
  local ang_cmd =
    "obj:queueGameEngineLua('angular_speed_angelo234 = ' .. obj:getYawAngularVelocity() )"
  my_veh:queueLuaCommand(ang_cmd)
  local rot_cmd =
    "obj:queueGameEngineLua('rotation_angelo234 = ' .. vec3(obj:getRollPitchYaw()):__tostring() )"
  my_veh:queueLuaCommand(rot_cmd)

  --Gets whether gearbox is in arcade or realistic mode
  local gear_cmd =
    'if controller.mainController.onSerialize ~= nil then obj:queueGameEngineLua("gearbox_mode_angelo234 = (\'" .. ' ..
    'jsonEncode(controller.mainController.onSerialize()) .. "\')") end'
  my_veh:queueLuaCommand(gear_cmd)

  if electrics_values_angelo234 == nil then
    return false
  end

  return veh_accs_angelo234 ~= nil
    and #electrics_values_angelo234 ~= 0
    and angular_speed_angelo234 ~= nil
    and input_throttle_angelo234 ~= nil
    and input_brake_angelo234 ~= nil
    and input_clutch_angelo234 ~= nil
    and input_parkingbrake_angelo234 ~= nil
    and gearbox_mode_angelo234 ~= nil
    and gearbox_mode_angelo234 ~= "null"
    and type(gearbox_mode_angelo234) ~= "table"
end

local yawSmooth = newExponentialSmoothing(10) --exponential smoothing for yaw rate

local function processVELuaData()
  --Decode json results
  electrics_values_angelo234 = jsonDecode(electrics_values_angelo234)
  gearbox_mode_angelo234 = jsonDecode(gearbox_mode_angelo234)

  --Smoothes angular velocity
  angular_speed_angelo234 = yawSmooth:get(angular_speed_angelo234)
end

--local p = LuaProfiler("my profiler")

local function updateVirtualLidar(dt, veh)
  if not extra_utils.getPart("lidar_angelo234") then
    resetVirtualLidarPointCloud()
    return
  end
  if not aeb_params then return end
  if not veh or not veh.getPosition or not veh.getDirectionVector or not veh.getDirectionVectorUp then return end
  if virtual_lidar_update_timer >= 1.0 / 20.0 then
    local pos = veh:getPosition()
    local dir = veh:getDirectionVector()
    local up = veh:getDirectionVectorUp()
    dir.z = 0
    dir = dir:normalized()
    up = up:normalized()
    local origin = vec3(pos.x, pos.y, pos.z + 1.8)
    -- In BeamNG's left-handed system, forward × up yields the vehicle's right
    local right = dir:cross(up):normalized()
    local max_dist = aeb_params.sensor_max_distance
    local hits = virtual_lidar.scan(
      origin,
      dir,
      up,
      max_dist,
      math.rad(360),
      math.rad(30),
      60,
      15,
      0,
      veh:getID(),
      {hStart = virtual_lidar_phase, hStep = VIRTUAL_LIDAR_PHASES}
    )

    -- cache properties of the player's vehicle for later filtering
    local veh_props = extra_utils.getVehicleProperties(veh)
    local self_bb = veh_props.bb
    local self_center = self_bb:getCenter()
    local self_axes = {
      vec3(self_bb:getAxis(0)),
      vec3(self_bb:getAxis(1)),
      vec3(self_bb:getAxis(2))
    }
    local self_half = self_bb:getHalfExtents()
    local self_margin = 0.25

    -- return true if the world-space point lies within our own vehicle's bounds
    local function insideSelf(pt)
      local rel = pt - self_center
      return math.abs(rel:dot(self_axes[1])) <= self_half.x + self_margin
        and math.abs(rel:dot(self_axes[2])) <= self_half.y + self_margin
        and math.abs(rel:dot(self_axes[3])) <= self_half.z + self_margin
    end

    local detections = {}
    local processed = {[veh:getID()] = true}

    local function addVehicle(vehObj, props)
      if not vehObj or not props then return end
      if props.id == veh:getID() or processed[props.id] then return end
      processed[props.id] = true
      local bb = props.bb
      if not bb then return end
      local center = props.center_pos
      local x = vec3(bb:getAxis(0)) * bb:getHalfExtents().x
      local y = vec3(bb:getAxis(1)) * bb:getHalfExtents().y
      local z = vec3(bb:getAxis(2)) * bb:getHalfExtents().z
      local corners = {
        center + x + y + z,
        center + x - y + z,
        center - x + y + z,
        center - x - y + z,
        center + x + y - z,
        center + x - y - z,
        center - x + y - z,
        center - x - y - z
      }
      for _, c in ipairs(corners) do
        local rel = c - origin
        if rel:length() < max_dist then
          hits[#hits + 1] = c
        end
      end
      local id = vehObj.getJBeamFilename and vehObj:getJBeamFilename() or tostring(vehObj:getID())
      local veh_type = isPlayerVehicle(vehObj) and 'player vehicle' or 'traffic vehicle'
      detections[#detections + 1] = {pos = center + z, desc = string.format('%s %s', veh_type, id)}
    end

    if front_sensor_data and front_sensor_data[2] then
      for _, data in ipairs(front_sensor_data[2]) do
        if data.other_veh and data.other_veh_props then
          addVehicle(data.other_veh, data.other_veh_props)
        end
      end
    end

    if rear_sensor_data and rear_sensor_data[1] then
      local vehRear = rear_sensor_data[1]
      if vehRear then
        local propsRear = extra_utils.getVehicleProperties(vehRear)
        addVehicle(vehRear, propsRear)
      end
    end

    for i = 0, be:getObjectCount() - 1 do
      local other = be:getObject(i)
      if other:getID() ~= veh:getID() and other:getJBeamFilename() ~= "unicycle" then
        addVehicle(other, extra_utils.getVehicleProperties(other))
      end
    end

    local groundThreshold = -1.5
    local current_cloud = {}
    for _, p in ipairs(hits) do
      local rel = p - origin
      if rel:dot(up) >= groundThreshold and not insideSelf(p) then
        current_cloud[#current_cloud + 1] = {
          x = rel:dot(right),
          y = rel:dot(dir),
          z = rel:dot(up)
        }
      end
    end
    virtual_lidar_point_cloud[virtual_lidar_phase + 1] = current_cloud

    for _, d in ipairs(detections) do
      local rel = d.pos - origin
      local x = rel:dot(right)
      local y = rel:dot(dir)
      local z = rel:dot(up)
      local dist = rel:length()
      local ang = math.deg(math.atan2(x, y))
      logger.log('I', 'lidar', string.format('Detected %s at %.1f m %.1f° (%.1f, %.1f, %.1f)', d.desc, dist, ang, x, y, z))
    end
    virtual_lidar_phase = (virtual_lidar_phase + 1) % VIRTUAL_LIDAR_PHASES
    virtual_lidar_update_timer = 0
  else
    virtual_lidar_update_timer = virtual_lidar_update_timer + dt
  end
end

local phase = 0

local function onUpdate(dt)
  --p:start()

  if first_update then
    -- sensor_system.init()
    -- fcm_system.init()
    -- rcm_system.init()
    -- acc_system.init()
    -- hsa_system.init()
    first_update = false
  end

  local my_veh = be:getPlayerVehicle(0)
  if my_veh == nil then return end

  local ready = getAllVehiclesPropertiesFromVELua(my_veh)
  --If Vehicle Lua data is nil then return
  if not ready then return end

  --Process data gathered from Vehicle Lua to be usable in our context
  processVELuaData()

  if not be:getEnabled() or not system_params then return end

  local veh_props = extra_utils.getVehicleProperties(my_veh)
  local need_front_sensors = extra_utils.getPart("acc_angelo234")
    or extra_utils.getPart("forward_collision_mitigation_angelo234")
    or extra_utils.getPart("obstacle_collision_mitigation_angelo234")
    or (extra_utils.getPart("auto_headlight_angelo234") and auto_headlight_system_on)
  local need_rear_sensors = extra_utils.getPart("reverse_collision_mitigation_angelo234")
    or extra_utils.getPart("obstacle_collision_mitigation_angelo234")

  if need_front_sensors or need_rear_sensors then
    --Update at 120 Hz
    if other_systems_timer >= 1.0 / 120.0 then
      if phase == 0 then
        --Get sensor data
        if need_front_sensors then
          front_sensor_data = sensor_system.pollFrontSensors(other_systems_timer * 2, veh_props, system_params, aeb_params)
        end
        if need_rear_sensors then
          rear_sensor_data = sensor_system.pollRearSensors(other_systems_timer * 2, veh_props, system_params, rev_aeb_params)
        end

        phase = 1

      elseif phase == 1 then
        --Update Adaptive Cruise Control
        if extra_utils.getPart("acc_angelo234") and acc_system_on then
          acc_system.update(other_systems_timer * 2, my_veh, system_params, aeb_params, front_sensor_data)
        end

          --Update Forward Collision Mitigation System
          if extra_utils.getPart("forward_collision_mitigation_angelo234") and fcm_system_on then
            fcm_system.update(
              other_systems_timer * 2,
              my_veh,
              system_params,
              aeb_params,
              beeper_params,
              front_sensor_data
            )
          end

          --Update Obstacle Collision Mitigation System
          if extra_utils.getPart("obstacle_collision_mitigation_angelo234")
            and extra_utils.getPart("obstacle_aeb_angelo234")
            and obstacle_aeb_system_on then
            obstacle_aeb_system.update(
              other_systems_timer * 2,
              my_veh,
              system_params,
              aeb_params,
              beeper_params,
              front_sensor_data,
              rear_sensor_data
            )
          end

          --Update Reverse Collision Mitigation System
          if extra_utils.getPart("reverse_collision_mitigation_angelo234") and rcm_system_on then
            rcm_system.update(
              other_systems_timer * 2,
              my_veh,
              system_params,
              parking_lines_params,
              rev_aeb_params,
              beeper_params,
              rear_sensor_data
            )
          end

          --Update Lane Assist System
          if extra_utils.getPart("lane_assist_angelo234") and lane_assist_system_on then
            lane_assist_system.update(
              other_systems_timer * 2,
              my_veh,
              system_params
            )
          end

        phase = 0
      end

      other_systems_timer = 0
    end
  end

  --Update at 10 Hz
  if hsa_system_update_timer >= 0.1 then
    --Update Hill Start Assist System
    if extra_utils.getPart("hill_start_assist_angelo234") then
      hsa_system.update(hsa_system_update_timer, my_veh)
    end

    hsa_system_update_timer = 0

    --p:add("hsa update")
  end

  --Update Auto Headlight System at 4 Hz
  if auto_headlight_system_update_timer >= 0.25 then
    if extra_utils.getPart("auto_headlight_angelo234") and auto_headlight_system_on then
      if front_sensor_data ~= nil then
        if prev_auto_headlight_system_on ~= auto_headlight_system_on then
          auto_headlight_system.systemSwitchedOn()
        end

        auto_headlight_system.update(auto_headlight_system_update_timer, my_veh, front_sensor_data[2])
      end

      auto_headlight_system_update_timer = 0
    end

    prev_auto_headlight_system_on = auto_headlight_system_on

    --p:add("auto headlight update")
  end

  --Update timers for updating systems
  other_systems_timer = other_systems_timer + dt
  hsa_system_update_timer = hsa_system_update_timer + dt
  auto_headlight_system_update_timer = auto_headlight_system_update_timer + dt

  updateVirtualLidar(dt, my_veh)

  --p:finish(true)
end

local function getVirtualLidarPointCloud()
  local combined = {}
  for i = 1, #virtual_lidar_point_cloud do
    for j = 1, #virtual_lidar_point_cloud[i] do
      combined[#combined + 1] = virtual_lidar_point_cloud[i][j]
    end
  end
  return combined
end

local function getLaneSensorData()
  return lane_assist_system.getSensorData()
end

local function onInit()
  setExtensionUnloadMode(M, "manual")
end

M.onExtensionLoaded = onExtensionLoaded
M.onVehicleSwitched = onVehicleSwitched
M.onHeadlightsOff = onHeadlightsOff
M.onHeadlightsOn = onHeadlightsOn
M.toggleFCMSystem = toggleFCMSystem
M.toggleRCMSystem = toggleRCMSystem
M.toggleObstacleAEBSystem = toggleObstacleAEBSystem
M.toggleLaneAssistSystem = toggleLaneAssistSystem
M.toggleAutoHeadlightSystem = toggleAutoHeadlightSystem
M.setACCSystemOn = setACCSystemOn
M.toggleACCSystem = toggleACCSystem
M.setACCSpeed = setACCSpeed
M.changeACCSpeed = changeACCSpeed
M.changeACCFollowingDistance = changeACCFollowingDistance
M.toggleDebugLogging = toggleDebugLogging
M.toggleFrontSensorLogging = toggleFrontSensorLogging
M.toggleRearSensorLogging = toggleRearSensorLogging
M.toggleLidarLogging = toggleLidarLogging
M.onCameraModeChanged = onCameraModeChanged
M.onUpdate = onUpdate
M.onInit = onInit
M.getVirtualLidarPointCloud = getVirtualLidarPointCloud
M.getLaneSensorData = getLaneSensorData

return M
