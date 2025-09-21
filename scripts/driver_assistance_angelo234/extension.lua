-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

--global table of all vehicles acceleration vectors
--veh_accs_angelo234[id][1] = lateral (-x = right, +x = left)
--veh_accs_angelo234[id][2] = longitudinal (-x = accelerating, +x = braking)
--veh_accs_angelo234[id][3] = up/down direction (-x = down, +x = up)
veh_accs_angelo234 = {}

local M = {}

-- Expose the extension instance through the legacy global name that other
-- driver assistance modules expect. Some modules (such as the forward
-- collision mitigation system) reference
-- `scripts_driver__assistance__angelo234_extension` directly and crash if it
-- is missing.  When the extension is loaded we immediately publish the module
-- table under that global so that dependent modules can safely call back into
-- the extension without needing to guard against a missing reference.
rawset(_G, 'scripts_driver__assistance__angelo234_extension', M)
rawset(_G, 'laneCenteringPrevEnableElectrics', nil)

local extra_utils = require('scripts/driver_assistance_angelo234/extraUtils')

local sensor_system = require('scripts/driver_assistance_angelo234/sensorSystem')
local fcm_system = require('scripts/driver_assistance_angelo234/forwardCollisionMitigationSystem')
local rcm_system = require('scripts/driver_assistance_angelo234/reverseCollisionMitigationSystem')
local acc_system = require('scripts/driver_assistance_angelo234/accSystem')
local hsa_system = require('scripts/driver_assistance_angelo234/hillStartAssistSystem')
local auto_headlight_system = require('scripts/driver_assistance_angelo234/autoHeadlightSystem')
local logger = require('scripts/driver_assistance_angelo234/logger')
local obstacle_aeb_system = require('scripts/driver_assistance_angelo234/obstacleBrakingSystem')
local lane_centering_system = require('scripts/driver_assistance_angelo234/laneCenteringAssistSystem')
local virtual_lidar = require('scripts/driver_assistance_angelo234/virtualLidar')
local drift_proximity = require('scripts/driver_assistance_angelo234/driftProximity')
local lidarPcdPublisher = require('scripts/driver_assistance_angelo234/lidarPcdPublisher')
local lidarPcdStream = require('scripts/driver_assistance_angelo234/lidarPcdStreamServer')

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
local obstacle_aeb_system_on = false
local lane_centering_assist_on = false
local lane_centering_ai_active = false
local lane_centering_ai_last_speed = nil
local lane_centering_ai_refresh_timer = 0
local lane_centering_ai_last_reason = nil
local lane_centering_ai_uses_speed_control = false

local front_sensor_data = nil
local rear_sensor_data = nil

local other_systems_timer = 0
local hsa_system_update_timer = 0
local auto_headlight_system_update_timer = 0
local virtual_lidar_update_timer = 0
local VIRTUAL_LIDAR_PHASES = 8
local FRONT_LIDAR_PHASES = 4
local virtual_lidar_point_cloud = {}
local virtual_lidar_ground_point_cloud = {}
local virtual_lidar_overhead_point_cloud = {}
local virtual_lidar_frames = {}
local front_lidar_point_cloud = {}
local front_lidar_ground_point_cloud = {}
local front_lidar_frames = {}
local front_lidar_point_cloud_wip = {}
local front_lidar_ground_point_cloud_wip = {}
local front_lidar_frames_wip = {}
local vehicle_lidar_point_cloud = {}
local vehicle_lidar_frame = nil

local function computeDefaultVirtualLidarPath()
  if FS and FS.getUserPath then
    local ok, base = pcall(function()
      return FS:getUserPath()
    end)
    if ok and type(base) == 'string' and base ~= '' then
      return base .. 'settings/krtektm_lidar/latest.pcd'
    end
  end
  return 'settings/krtektm_lidar/latest.pcd'
end

local DEFAULT_VIRTUAL_LIDAR_PCD_PATH = computeDefaultVirtualLidarPath()
local virtual_lidar_pcd = {
  enabled = false,
  path = DEFAULT_VIRTUAL_LIDAR_PCD_PATH,
  requestedPath = DEFAULT_VIRTUAL_LIDAR_PCD_PATH,
  actualPath = DEFAULT_VIRTUAL_LIDAR_PCD_PATH,
  sandboxBlockedPath = nil,
  sandboxReason = nil,
  dirty = false
}

local virtual_lidar_stream = {
  enabled = false,
  host = '127.0.0.1',
  port = 23511,
  running = false,
  activePort = nil
}

local function reportLidarError(msg)
  logger.log('E', 'lidar', msg)
end

local function ensureVirtualLidarStreamServer()
  local running = lidarPcdStream.isRunning and lidarPcdStream.isRunning() or false
  if not running then
    virtual_lidar_stream.activePort = nil
  end
  if running and virtual_lidar_stream.activePort == virtual_lidar_stream.port then
    virtual_lidar_stream.running = true
    return true
  end
  local ok, err = lidarPcdStream.start(virtual_lidar_stream.host, virtual_lidar_stream.port)
  if not ok then
    virtual_lidar_stream.running = running
    return false, err
  end
  virtual_lidar_stream.running = true
  virtual_lidar_stream.activePort = virtual_lidar_stream.port
  return true
end

local function buildFrameBasis(forward, up)
  if not forward or not up then return end
  if forward:length() < 1e-6 then return end
  local dir = forward:normalized()
  if up:length() < 1e-6 then return end
  local upNorm = up:normalized()
  local right = dir:cross(upNorm)
  if right:length() < 1e-6 then
    local fallbackUp = vec3(0, 0, 1)
    right = dir:cross(fallbackUp)
    if right:length() < 1e-6 then
      fallbackUp = vec3(0, 1, 0)
      right = dir:cross(fallbackUp)
    end
  end
  if right:length() < 1e-6 then return end
  right = right:normalized()
  local orthoUp = right:cross(dir)
  if orthoUp:length() < 1e-6 then return end
  orthoUp = orthoUp:normalized()
  return dir, right, orthoUp
end

local function clampValue(value, minValue, maxValue)
  if value < minValue then return minValue end
  if value > maxValue then return maxValue end
  return value
end

local VIRTUAL_LIDAR_PITCH_BLEND_START = math.rad(4)
local VIRTUAL_LIDAR_PITCH_BLEND_FULL = math.rad(12)

local function buildVirtualLidarFrame(forward, up)
  if not forward or not up then return end
  if forward:length() < 1e-6 then return end
  if up:length() < 1e-6 then return end

  local upNorm = up:normalized()
  local forwardNorm = forward:normalized()
  local planar = vec3(forward.x, forward.y, 0)
  local planarLen = planar:length()

  if planarLen > 1e-6 then
    planar = planar / planarLen
    local pitch = math.asin(clampValue(forwardNorm:dot(upNorm), -1, 1))
    local absPitch = math.abs(pitch)
    local blend = 0
    if absPitch > VIRTUAL_LIDAR_PITCH_BLEND_START then
      if absPitch >= VIRTUAL_LIDAR_PITCH_BLEND_FULL then
        blend = 1
      else
        blend = (absPitch - VIRTUAL_LIDAR_PITCH_BLEND_START)
          / (VIRTUAL_LIDAR_PITCH_BLEND_FULL - VIRTUAL_LIDAR_PITCH_BLEND_START)
      end
    end

    local baseDir
    if blend > 0 then
      baseDir = planar * (1 - blend) + forwardNorm * blend
    else
      baseDir = planar
    end

    local dir, right, orthoUp = buildFrameBasis(baseDir, upNorm)
    if dir and right and orthoUp then
      return dir, right, orthoUp
    end
  end

  return buildFrameBasis(forward, up)
end

local function stopVirtualLidarStreamServer()
  lidarPcdStream.stop()
  virtual_lidar_stream.running = false
  virtual_lidar_stream.activePort = nil
end

lidarPcdPublisher.configure({path = DEFAULT_VIRTUAL_LIDAR_PCD_PATH, enabled = false})
lidarPcdPublisher.setEnabled(false)

local maybePublishVirtualLidarPcd

local VEHICLE_POINT_MIN_SPACING = 0.3
local VEHICLE_POINT_MAX_SPACING = 2.0
local VEHICLE_POINT_MAX_STEPS = 20
local VIRTUAL_LIDAR_H_FOV = math.rad(360)
local VIRTUAL_LIDAR_V_FOV = math.rad(40)
local VIRTUAL_LIDAR_H_RES = 60
local VIRTUAL_LIDAR_V_RES = 15
local VIRTUAL_LIDAR_MAX_RAYS = 80
local FRONT_LIDAR_MAX_RAYS = 120
local VIRTUAL_LIDAR_H_STEP = VIRTUAL_LIDAR_H_FOV / math.max(1, VIRTUAL_LIDAR_H_RES - 1)
local VIRTUAL_LIDAR_V_STEP = VIRTUAL_LIDAR_V_FOV / math.max(1, VIRTUAL_LIDAR_V_RES - 1)
local VIRTUAL_LIDAR_FRONT_EXTENSION = 120
local VIRTUAL_LIDAR_SIDE_EXTENSION = 60
local VIRTUAL_LIDAR_REAR_EXTENSION = 40
local LIDAR_GROUND_THRESHOLD = -1.5
local LIDAR_LOW_OBJECT_BAND = 1.0

local function resetVirtualLidarPointCloud()
  virtual_lidar_point_cloud = {}
  virtual_lidar_ground_point_cloud = {}
  virtual_lidar_overhead_point_cloud = {}
  virtual_lidar_frames = {}
  front_lidar_point_cloud = {}
  front_lidar_ground_point_cloud = {}
  front_lidar_frames = {}
  front_lidar_point_cloud_wip = {}
  front_lidar_ground_point_cloud_wip = {}
  front_lidar_frames_wip = {}
  vehicle_lidar_point_cloud = {}
  vehicle_lidar_frame = nil
  for i = 1, VIRTUAL_LIDAR_PHASES do
    virtual_lidar_point_cloud[i] = {}
    virtual_lidar_ground_point_cloud[i] = {}
    virtual_lidar_overhead_point_cloud[i] = {}
    virtual_lidar_frames[i] = nil
  end
  for i = 1, FRONT_LIDAR_PHASES do
    front_lidar_point_cloud[i] = {}
    front_lidar_ground_point_cloud[i] = {}
    front_lidar_frames[i] = nil
    front_lidar_point_cloud_wip[i] = {}
    front_lidar_ground_point_cloud_wip[i] = {}
    front_lidar_frames_wip[i] = nil
  end
end

resetVirtualLidarPointCloud()
local virtual_lidar_phase = 0
local front_lidar_phase = 0
local front_lidar_update_timer = 0
local front_lidar_thread = nil

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
  ensureVirtualLidarStreamServer()
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
  ui_message("Obstacle Collision AEB switched " .. state)
end

local function hasLaneCenteringAssistPart()
  return extra_utils.getPart("lane_centering_assist_angelo234")
    or extra_utils.getPart("lane_centering_assist_system_angelo234")
end

local function formatLaneCenteringReason(reason)
  if reason == "driver_override" then
    return "Driver steering input"
  end
  if reason == "low_speed" then
    return "Speed below 40 km/h"
  end
  if reason == "off_road" then
    return "Vehicle not on road"
  end
  if reason == "ai_route_conflict" then
    return "AI route conflict"
  end
  if reason and reason ~= "" and reason ~= "user_toggle" then
    return tostring(reason)
  end
  return nil
end

local function determineLaneCenteringTargetSpeed(veh)
  if not veh then return 0, false end

  local target_speed = nil

  if acc_system_on and acc_system and acc_system.getCurrentTargetSpeed then
    local acc_set_speed, acc_ramped_speed = acc_system.getCurrentTargetSpeed()
    if acc_ramped_speed and acc_ramped_speed > 0 then
      target_speed = acc_ramped_speed
    elseif acc_set_speed and acc_set_speed > 0 then
      target_speed = acc_set_speed
    end
    if target_speed and target_speed > 0 then
      return target_speed, true
    end
  end

  local velocity = veh.getVelocity and veh:getVelocity()
  if velocity then
    local speed = vec3(velocity):length()
    if speed and speed > 0 then
      target_speed = speed
    end
  end

  if not target_speed or target_speed < 0 then
    target_speed = 0
  end

  return target_speed, false
end

local lane_centering_ai_manual_enable_command = [[
    if ai then
      local prevEnableElectrics = true
      if ai.getParameters then
        local params = ai.getParameters()
        if params and params.enableElectrics ~= nil then
          prevEnableElectrics = params.enableElectrics and true or false
        end
      end
      if laneCenteringPrevEnableElectrics == nil then
        laneCenteringPrevEnableElectrics = prevEnableElectrics
      end
      ai.setState({mode='traffic', manoeuvre=false})
      if ai.setParameters then
        ai.setParameters({enableElectrics = false})
      end
      ai.setSpeedMode('set')
      ai.driveInLane('on')
      ai.setAvoidCars('off')
    end
    if input and input.setAllowedInputSource then
      input.setAllowedInputSource('steering', 'ai', true)
      input.setAllowedInputSource('steering', 'local', true)
      input.setAllowedInputSource('throttle', nil)
      input.setAllowedInputSource('brake', nil)
      input.setAllowedInputSource('parkingbrake', nil)
      input.setAllowedInputSource('throttle', 'local', true)
      input.setAllowedInputSource('brake', 'local', true)
      input.setAllowedInputSource('parkingbrake', 'local', true)
    end
  ]]

local lane_centering_ai_manual_refresh_command = [[
    if ai then
      if ai.setParameters then
        ai.setParameters({enableElectrics = false})
      end
      ai.setSpeedMode('set')
      ai.driveInLane('on')
      ai.setAvoidCars('off')
    end
    if input and input.setAllowedInputSource then
      input.setAllowedInputSource('steering', 'ai', true)
      input.setAllowedInputSource('steering', 'local', true)
    end
  ]]

local lane_centering_ai_speed_enable_template = [[
    if ai then
      local prevEnableElectrics = true
      if ai.getParameters then
        local params = ai.getParameters()
        if params and params.enableElectrics ~= nil then
          prevEnableElectrics = params.enableElectrics and true or false
        end
      end
      if laneCenteringPrevEnableElectrics == nil then
        laneCenteringPrevEnableElectrics = prevEnableElectrics
      end
      ai.setState({mode='traffic', manoeuvre=false})
      if ai.setParameters then
        ai.setParameters({enableElectrics = false})
      end
      ai.setSpeedMode('set')
      ai.driveInLane('on')
      ai.setAvoidCars('off')
      ai.setSpeed(%f)
    end
    if input and input.setAllowedInputSource then
      input.setAllowedInputSource('steering', 'ai', true)
      input.setAllowedInputSource('steering', 'local', true)
      input.setAllowedInputSource('throttle', 'ai', false)
      input.setAllowedInputSource('brake', 'ai', false)
      input.setAllowedInputSource('parkingbrake', 'ai', false)
      input.setAllowedInputSource('throttle', 'local', true)
      input.setAllowedInputSource('brake', 'local', true)
      input.setAllowedInputSource('parkingbrake', 'local', true)
    end
  ]]

local lane_centering_ai_speed_refresh_template = [[
    if ai then
      if ai.setParameters then
        ai.setParameters({enableElectrics = false})
      end
      ai.setSpeedMode('set')
      ai.driveInLane('on')
      ai.setAvoidCars('off')
      ai.setSpeed(%f)
    end
    if input and input.setAllowedInputSource then
      input.setAllowedInputSource('steering', 'ai', true)
      input.setAllowedInputSource('steering', 'local', true)
      input.setAllowedInputSource('throttle', 'ai', false)
      input.setAllowedInputSource('brake', 'ai', false)
      input.setAllowedInputSource('parkingbrake', 'ai', false)
      input.setAllowedInputSource('throttle', 'local', true)
      input.setAllowedInputSource('brake', 'local', true)
      input.setAllowedInputSource('parkingbrake', 'local', true)
    end
  ]]

local function queueLaneCenteringAiEnable(veh, target_speed, use_speed_control)
  local command = nil
  if use_speed_control then
    command = string.format(lane_centering_ai_speed_enable_template, target_speed or 0)
  else
    command = lane_centering_ai_manual_enable_command
  end
  veh:queueLuaCommand(command)
end

local function queueLaneCenteringAiDisable(veh)
  local command = [[
    local prevEnableElectrics = laneCenteringPrevEnableElectrics
    laneCenteringPrevEnableElectrics = nil
    if ai then
      if prevEnableElectrics == nil then
        prevEnableElectrics = true
      end
      if ai.setParameters then
        ai.setParameters({enableElectrics = prevEnableElectrics and true or false})
      end
      ai.setSpeed(0)
      ai.setSpeedMode('set')
      ai.driveInLane('off')
      ai.setAvoidCars('on')
      ai.setState({mode='disabled'})
    end
    if input and input.setAllowedInputSource then
      input.setAllowedInputSource('steering', nil)
      input.setAllowedInputSource('throttle', nil)
      input.setAllowedInputSource('brake', nil)
      input.setAllowedInputSource('parkingbrake', nil)
    end
  ]]
  veh:queueLuaCommand(command)
end

local function applyLaneCenteringAiState(active, reason)
  lane_centering_ai_last_reason = reason
  local veh = be:getPlayerVehicle(0)
  if not veh then
    lane_centering_ai_active = false
    lane_centering_ai_last_speed = nil
    lane_centering_ai_refresh_timer = 0
    lane_centering_ai_uses_speed_control = false
    rawset(_G, 'lane_centering_ai_mode_active_angelo234', nil)
    rawset(_G, 'lane_centering_ai_speed_control_active_angelo234', nil)
    return
  end

  if active then
    local target_speed, use_speed_control = determineLaneCenteringTargetSpeed(veh)
    queueLaneCenteringAiEnable(veh, target_speed, use_speed_control)
    lane_centering_ai_active = true
    lane_centering_ai_uses_speed_control = use_speed_control and true or false
    lane_centering_ai_last_speed = use_speed_control and target_speed or nil
    lane_centering_ai_refresh_timer = 0
    rawset(_G, 'lane_centering_ai_mode_active_angelo234', true)
    if lane_centering_ai_uses_speed_control then
      rawset(_G, 'lane_centering_ai_speed_control_active_angelo234', true)
    else
      rawset(_G, 'lane_centering_ai_speed_control_active_angelo234', nil)
    end
  else
    queueLaneCenteringAiDisable(veh)
    lane_centering_ai_active = false
    lane_centering_ai_uses_speed_control = false
    lane_centering_ai_last_speed = nil
    lane_centering_ai_refresh_timer = 0
    rawset(_G, 'lane_centering_ai_mode_active_angelo234', nil)
    rawset(_G, 'lane_centering_ai_speed_control_active_angelo234', nil)
  end
end

local function refreshLaneCenteringAiState(dt, veh)
  if not lane_centering_ai_active or not lane_centering_assist_on or not veh then
    lane_centering_ai_refresh_timer = 0
    return
  end

  lane_centering_ai_refresh_timer = lane_centering_ai_refresh_timer + (dt or 0)

  local target_speed, use_speed_control = determineLaneCenteringTargetSpeed(veh)

  if (use_speed_control and not lane_centering_ai_uses_speed_control)
    or (not use_speed_control and lane_centering_ai_uses_speed_control) then
    queueLaneCenteringAiEnable(veh, target_speed, use_speed_control)
    lane_centering_ai_uses_speed_control = use_speed_control and true or false
    lane_centering_ai_last_speed = use_speed_control and target_speed or nil
    if lane_centering_ai_uses_speed_control then
      rawset(_G, 'lane_centering_ai_speed_control_active_angelo234', true)
    else
      rawset(_G, 'lane_centering_ai_speed_control_active_angelo234', nil)
    end
    lane_centering_ai_refresh_timer = 0
    return
  end

  if not use_speed_control then
    lane_centering_ai_last_speed = nil
    if lane_centering_ai_refresh_timer >= 0.5 then
      veh:queueLuaCommand(lane_centering_ai_manual_refresh_command)
      lane_centering_ai_refresh_timer = 0
    end
    return
  end

  if lane_centering_ai_last_speed == nil then
    lane_centering_ai_last_speed = target_speed
  end

  local speed_delta = math.abs((target_speed or 0) - (lane_centering_ai_last_speed or 0))

  if speed_delta > 0.25 or lane_centering_ai_refresh_timer >= 0.5 then
    veh:queueLuaCommand(string.format(lane_centering_ai_speed_refresh_template, target_speed or 0))
    lane_centering_ai_last_speed = target_speed
    lane_centering_ai_refresh_timer = 0
  end
end

local function setLaneCenteringAssistActive(active, reason)
  if not hasLaneCenteringAssistPart() then
    if lane_centering_ai_active then
      applyLaneCenteringAiState(false, 'missing_part')
    end
    if active then
      ui_message("Lane Centering Assist not installed")
    end
    if lane_centering_assist_on then
      lane_centering_assist_on = false
      ui_message("Lane Centering Assist disengaged")
    end
    return
  end

  if lane_centering_assist_on == active then
    if active then
      if reason ~= "user_toggle" then
        if reason == "speed_ready" and not lane_centering_ai_active then
          ui_message("Lane Centering Assist engaged")
        end
        applyLaneCenteringAiState(true, reason)
      end
    else
      applyLaneCenteringAiState(false, reason)
      local detail = formatLaneCenteringReason(reason)
      if detail then
        ui_message("Lane Centering Assist disengaged: " .. detail)
      end
    end
    return
  end

  if active and lane_centering_system and lane_centering_system.isVehicleOnRoad then
    if not lane_centering_system.isVehicleOnRoad() then
      local detail = formatLaneCenteringReason("off_road") or "Vehicle not on road"
      ui_message("Lane Centering Assist unavailable: " .. detail)
      if lane_centering_ai_active then
        applyLaneCenteringAiState(false, 'off_road_block')
      end
      lane_centering_assist_on = false
      return
    end
  end

  lane_centering_assist_on = active

  if active then
    if reason == "user_toggle" then
      ui_message("Lane Centering Assist armed")
    else
      applyLaneCenteringAiState(true, reason)
      ui_message("Lane Centering Assist engaged")
    end
  else
    applyLaneCenteringAiState(false, reason)
    local detail = formatLaneCenteringReason(reason)
    if detail then
      ui_message("Lane Centering Assist disengaged: " .. detail)
    else
      ui_message("Lane Centering Assist disengaged")
    end
  end
end

local function handleLaneCenteringActivationRequest(active, reason)
  setLaneCenteringAssistActive(active, reason or 'system')
end

local function toggleLaneCenteringSystem()
  if not hasLaneCenteringAssistPart() then
    ui_message("Lane Centering Assist not installed")
    lane_centering_assist_on = false
    return
  end

  setLaneCenteringAssistActive(not lane_centering_assist_on, "user_toggle")
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
  local steering_cmd = [[
    local driver = 0
    if input.lastInputs and input.lastInputs["local"] and input.lastInputs["local"].steering ~= nil then
      driver = input.lastInputs["local"].steering
    end

    if type(driver) ~= 'number' then
      driver = tonumber(driver) or 0
    end
    obj:queueGameEngineLua(string.format("input_steering_driver_angelo234 = %.6f", driver or 0))

    local assist = 0
    if input.lastInputs and input.lastInputs.lane_centering and input.lastInputs.lane_centering.steering ~= nil then
      assist = input.lastInputs.lane_centering.steering
    end
    if type(assist) ~= 'number' then
      assist = tonumber(assist) or 0
    end
    obj:queueGameEngineLua(string.format("input_steering_assist_angelo234 = %.6f", assist or 0))

    if input.steering ~= nil then
      local combined = input.steering
      if type(combined) ~= 'number' then
        combined = tonumber(combined) or 0
      end
      obj:queueGameEngineLua(string.format("input_steering_angelo234 = %.6f", combined))
    else
      obj:queueGameEngineLua("input_steering_angelo234 = 0")
    end
  ]]
  my_veh:queueLuaCommand(steering_cmd)

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

local function frontLidarLoop()
  while true do
    local dt, veh = coroutine.yield()
    if extra_utils.getPart("lidar_angelo234") and aeb_params and veh and veh.getPosition and veh.getDirectionVector and veh.getDirectionVectorUp then
      front_lidar_update_timer = front_lidar_update_timer + dt
      if front_lidar_update_timer >= 1.0 / 20.0 then
        local pos = veh:getPosition()
        local forward = veh:getDirectionVector()
        local upVec = veh:getDirectionVectorUp()
        local dir, right, up = buildVirtualLidarFrame(forward, upVec)
        if dir and right and up then
          local origin = vec3(pos.x, pos.y, pos.z + 1.8)
          front_lidar_frames_wip[front_lidar_phase + 1] = {
            origin = origin,
            dir = dir,
            right = right,
            up = up
          }
          local base_dist = aeb_params.sensor_max_distance
          local front_dist = base_dist + VIRTUAL_LIDAR_FRONT_EXTENSION
          local hits = virtual_lidar.scan(
            origin,
            dir,
            up,
            front_dist,
            math.rad(170),
            math.rad(30),
            60,
            15,
            0,
            veh:getID(),
            {
              hStart = front_lidar_phase,
              hStep = FRONT_LIDAR_PHASES,
              maxRays = FRONT_LIDAR_MAX_RAYS
            }
          )
          local groundThreshold = LIDAR_GROUND_THRESHOLD
          local lowObjectMin = groundThreshold - LIDAR_LOW_OBJECT_BAND
          local current_cloud = {}
          local low_cloud = {}
          for _, p in ipairs(hits) do
            local rel = p - origin
            local z = rel:dot(up)
            if rel:length() <= front_dist then
              local point = {
                x = rel:dot(right),
                y = rel:dot(dir),
                z = z
              }
              if z >= groundThreshold then
                current_cloud[#current_cloud + 1] = point
              elseif z >= lowObjectMin then
                low_cloud[#low_cloud + 1] = point
              end
            end
          end
          front_lidar_point_cloud_wip[front_lidar_phase + 1] = current_cloud
          front_lidar_ground_point_cloud_wip[front_lidar_phase + 1] = low_cloud
          front_lidar_phase = (front_lidar_phase + 1) % FRONT_LIDAR_PHASES
          if front_lidar_phase == 0 then
            front_lidar_point_cloud, front_lidar_point_cloud_wip = front_lidar_point_cloud_wip, front_lidar_point_cloud
            front_lidar_ground_point_cloud, front_lidar_ground_point_cloud_wip = front_lidar_ground_point_cloud_wip, front_lidar_ground_point_cloud
            front_lidar_frames, front_lidar_frames_wip = front_lidar_frames_wip, front_lidar_frames
          end
        end
        front_lidar_update_timer = 0
      end
    end
  end
end

front_lidar_thread = coroutine.create(frontLidarLoop)

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
    local forward = veh:getDirectionVector()
    local upVec = veh:getDirectionVectorUp()
    local dir, right, up = buildVirtualLidarFrame(forward, upVec)
    if dir and right and up then
      local origin = vec3(pos.x, pos.y, pos.z + 1.8)
      -- In BeamNG's left-handed system, forward × up yields the vehicle's right
      virtual_lidar_frames[virtual_lidar_phase + 1] = {
        origin = origin,
        dir = dir,
        right = right,
        up = up
      }
      local base_dist = aeb_params.sensor_max_distance
      -- extend coverage beyond the base sensor range to match LiDAR reach
      local front_dist = base_dist + VIRTUAL_LIDAR_FRONT_EXTENSION
      local rear_dist = base_dist + VIRTUAL_LIDAR_REAR_EXTENSION
      local side_dist = base_dist + VIRTUAL_LIDAR_SIDE_EXTENSION
      local scan_range = math.max(front_dist, rear_dist, side_dist)
      local ANG_FRONT = 95
      local ANG_REAR = 120
      local scan_hits = virtual_lidar.scan(
        origin,
        dir,
        up,
        scan_range,
        VIRTUAL_LIDAR_H_FOV,
        VIRTUAL_LIDAR_V_FOV,
        VIRTUAL_LIDAR_H_RES,
        VIRTUAL_LIDAR_V_RES,
        0,
        veh:getID(),
        {
          hStart = virtual_lidar_phase,
          hStep = VIRTUAL_LIDAR_PHASES,
          maxRays = VIRTUAL_LIDAR_MAX_RAYS,
          includeDynamic = false
        }
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
      local vehicle_hits = {}

      local function allowedDistance(rel)
        local ang = math.deg(math.atan2(rel:dot(right), rel:dot(dir)))
        local absAng = math.abs(ang)
        if absAng <= ANG_FRONT then
          return front_dist
        elseif absAng >= ANG_REAR then
          return rear_dist
        else
          return side_dist
        end
      end

      local function addVehicle(vehObj, props)
        if not vehObj then return end
        local freshProps = extra_utils.getVehicleProperties(vehObj)
        if freshProps then
          props = freshProps
        end
        if not props then return end
        local id = props.id or vehObj:getID()
        if id == veh:getID() or processed[id] then return end
        processed[id] = true
        if extra_utils.isVehicleGhost(vehObj, props) then return end
        local bb = props.bb
        if not bb then return end
        local center = props.center_pos or bb:getCenter()
        local half_extents = bb:getHalfExtents()
        local axis_x = vec3(bb:getAxis(0))
        local axis_y = vec3(bb:getAxis(1))
        local axis_z = vec3(bb:getAxis(2))
        local top = center + axis_z * half_extents.z
        local relCenter = center - origin
        local relTop = top - origin
        local forwardDist = relCenter:dot(dir)
        local sideDist = relCenter:dot(right)
        local planarDist = math.max(0.5, math.sqrt(forwardDist * forwardDist + sideDist * sideDist))
        local heightOffset = math.abs(relCenter:dot(up))
        local function clampSpacing(value)
          if value < VEHICLE_POINT_MIN_SPACING then return VEHICLE_POINT_MIN_SPACING end
          if value > VEHICLE_POINT_MAX_SPACING then return VEHICLE_POINT_MAX_SPACING end
          return value
        end
        local spacing_x = clampSpacing(planarDist * math.tan(VIRTUAL_LIDAR_H_STEP))
        local spacing_y = clampSpacing(planarDist * math.tan(VIRTUAL_LIDAR_V_STEP) + heightOffset * math.tan(VIRTUAL_LIDAR_V_STEP))
        local function stepsForHalfSpan(halfSpan, spacing)
          local steps = math.floor(halfSpan / math.max(spacing, VEHICLE_POINT_MIN_SPACING) + 0.5)
          if steps < 1 then steps = 1 end
          if steps > VEHICLE_POINT_MAX_STEPS then steps = VEHICLE_POINT_MAX_STEPS end
          return steps
        end
        local steps_x = stepsForHalfSpan(half_extents.x, spacing_x)
        local steps_y = stepsForHalfSpan(half_extents.y, spacing_y)
        for xi = -steps_x, steps_x do
          local offset_x = axis_x * (half_extents.x * xi / steps_x)
          for yi = -steps_y, steps_y do
            local offset_y = axis_y * (half_extents.y * yi / steps_y)
            local p = top + offset_x + offset_y
            local rel = p - origin
            if rel:length() < allowedDistance(rel) then
              vehicle_hits[#vehicle_hits + 1] = p
            end
          end
        end
        if relTop:length() <= allowedDistance(relTop) then
          local id = vehObj.getJBeamFilename and vehObj:getJBeamFilename() or tostring(vehObj:getID())
          local veh_type = isPlayerVehicle(vehObj) and 'player vehicle' or 'traffic vehicle'
          detections[#detections + 1] = {pos = top, desc = string.format('%s %s', veh_type, id)}
        end
      end

      if front_sensor_data and front_sensor_data[2] then
        for _, data in ipairs(front_sensor_data[2]) do
          if data.other_veh and data.other_veh_props and not extra_utils.isVehicleGhost(data.other_veh, data.other_veh_props) then
            addVehicle(data.other_veh, data.other_veh_props)
          end
        end
      end

      if rear_sensor_data and rear_sensor_data[1] then
        local vehRear = rear_sensor_data[1]
        if vehRear then
          local propsRear = extra_utils.getVehicleProperties(vehRear)
          if not extra_utils.isVehicleGhost(vehRear, propsRear) then
            addVehicle(vehRear, propsRear)
          end
        end
      end

      for i = 0, be:getObjectCount() - 1 do
        local other = be:getObject(i)
        if other:getID() ~= veh:getID() and other:getJBeamFilename() ~= "unicycle" then
          local otherProps = extra_utils.getVehicleProperties(other)
          if not extra_utils.isVehicleGhost(other, otherProps) then
            addVehicle(other, otherProps)
          end
        end
      end

      local groundThreshold = LIDAR_GROUND_THRESHOLD
      local lowObjectMin = groundThreshold - LIDAR_LOW_OBJECT_BAND
      local current_cloud = {}
      local ground_cloud = {}
      local overhead_cloud = {}
      for _, p in ipairs(scan_hits) do
        local rel = p - origin
        local withinRange = rel:length() <= allowedDistance(rel)
        local z = rel:dot(up)
        local point = {
          x = rel:dot(right),
          y = rel:dot(dir),
          z = z
        }
        if insideSelf(p) then
          if withinRange and z >= groundThreshold then
            overhead_cloud[#overhead_cloud + 1] = point
          end
        elseif withinRange then
          if z >= groundThreshold then
            current_cloud[#current_cloud + 1] = point
          elseif z >= lowObjectMin then
            ground_cloud[#ground_cloud + 1] = point
          end
        end
      end
      virtual_lidar_point_cloud[virtual_lidar_phase + 1] = current_cloud
      virtual_lidar_ground_point_cloud[virtual_lidar_phase + 1] = ground_cloud
      virtual_lidar_overhead_point_cloud[virtual_lidar_phase + 1] = overhead_cloud

      local veh_cloud = {}
      for _, p in ipairs(vehicle_hits) do
        local rel = p - origin
        if rel:dot(up) >= groundThreshold and rel:length() <= allowedDistance(rel) then
          veh_cloud[#veh_cloud + 1] = {
            x = rel:dot(right),
            y = rel:dot(dir),
            z = rel:dot(up)
          }
        end
      end
      vehicle_lidar_point_cloud = veh_cloud
      vehicle_lidar_frame = {origin = origin, dir = dir, right = right, up = up}

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
      if virtual_lidar_phase == 0 then
        maybePublishVirtualLidarPcd(veh)
      end
    end
    virtual_lidar_update_timer = 0
  else
    virtual_lidar_update_timer = virtual_lidar_update_timer + dt
  end
end

local phase = 0

local function onUpdate(dt)
  --p:start()

  lidarPcdStream.update(dt)

  if first_update then
    if sensor_system.init then sensor_system.init() end
    if fcm_system.init then fcm_system.init() end
    if rcm_system.init then rcm_system.init() end
    if acc_system.init then acc_system.init() end
    if hsa_system.init then hsa_system.init() end
    first_update = false
  end

  local my_veh = be:getPlayerVehicle(0)
  if my_veh == nil then return end

  local ready = getAllVehiclesPropertiesFromVELua(my_veh)
  --If Vehicle Lua data is nil then return
  if not ready then return end

  --Process data gathered from Vehicle Lua to be usable in our context
  processVELuaData()

  if front_lidar_thread and coroutine.status(front_lidar_thread) ~= "dead" then
    coroutine.resume(front_lidar_thread, dt, my_veh)
  end

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

          --Update Self-driving Assist
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

          --Update Lane Centering Assist System
          lane_centering_system.update(
            other_systems_timer * 2,
            my_veh,
            system_params,
            lane_centering_assist_on
          )

          local lane_centering_state = lane_centering_system.getLaneData()
          local lane_centering_status = lane_centering_state and lane_centering_state.status or nil
          local assist_active = lane_centering_status and lane_centering_status.active

          if lane_centering_assist_on and assist_active then
            if not lane_centering_ai_active then
              applyLaneCenteringAiState(true, 'refresh')
            end
            refreshLaneCenteringAiState(other_systems_timer * 2, my_veh)
          elseif lane_centering_ai_active then
            applyLaneCenteringAiState(false, 'sync')
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

local function buildCurrentFrame(veh)
  if not veh or not veh.getPosition or not veh.getDirectionVector or not veh.getDirectionVectorUp then return nil end
  local pos = veh:getPosition()
  local forward = veh:getDirectionVector()
  local upVec = veh:getDirectionVectorUp()
  if not pos or not forward or not upVec then return nil end
  local dir, right, up = buildVirtualLidarFrame(forward, upVec)
  if not dir or not right or not up then return nil end
  return {
    origin = vec3(pos.x, pos.y, pos.z + 1.8),
    dir = dir,
    right = right,
    up = up
  }
end

local function accumulateTransformedPoints(buffers, frames, curr, combined)
  for i = 1, #buffers do
    local frame = frames[i]
    if frame then
      local bucket = buffers[i]
      for j = 1, #bucket do
        combined[#combined + 1] = drift_proximity.apply(bucket[j], frame, curr)
      end
    end
  end
end

local function resolveCurrentFrame(curr, veh)
  if curr then return curr end
  return buildCurrentFrame(veh or be:getPlayerVehicle(0))
end

local function getVirtualLidarPointCloud(curr, veh)
  local frame = resolveCurrentFrame(curr, veh)
  if not frame then return {} end
  local combined = {}
  accumulateTransformedPoints(virtual_lidar_point_cloud, virtual_lidar_frames, frame, combined)
  accumulateTransformedPoints(front_lidar_point_cloud, front_lidar_frames, frame, combined)
  if vehicle_lidar_frame then
    for i = 1, #vehicle_lidar_point_cloud do
      combined[#combined + 1] = drift_proximity.apply(vehicle_lidar_point_cloud[i], vehicle_lidar_frame, frame)
    end
  end
  return combined
end

local function getVirtualLidarGroundPointCloud(curr, veh)
  local frame = resolveCurrentFrame(curr, veh)
  if not frame then return {} end
  local combined = {}
  accumulateTransformedPoints(virtual_lidar_ground_point_cloud, virtual_lidar_frames, frame, combined)
  accumulateTransformedPoints(front_lidar_ground_point_cloud, front_lidar_frames, frame, combined)
  return combined
end

local function getVirtualLidarOverheadPointCloud(curr, veh)
  local frame = resolveCurrentFrame(curr, veh)
  if not frame then return {} end
  local combined = {}
  accumulateTransformedPoints(virtual_lidar_overhead_point_cloud, virtual_lidar_frames, frame, combined)
  return combined
end

local function roundToMillis(value)
  if value >= 0 then
    return math.floor(value * 1000 + 0.5)
  end
  return -math.floor(-value * 1000 + 0.5)
end

local function makePointKey(pt)
  if not pt then return nil end
  return string.format('%d:%d:%d', roundToMillis(pt.x or 0), roundToMillis(pt.y or 0), roundToMillis(pt.z or 0))
end

local function toWorldPoint(pt, frame)
  if not pt or not frame then return nil end
  local relX = pt.x or pt[1] or 0
  local relY = pt.y or pt[2] or 0
  local relZ = pt.z or pt[3] or 0
  local world = frame.origin + frame.right * relX + frame.dir * relY + frame.up * relZ
  return {x = world.x, y = world.y, z = world.z}
end

local function convertPointsToWorld(points, frame, skipMap)
  local worldPoints = {}
  if not points or not frame then return worldPoints end
  for i = 1, #points do
    local worldPt = toWorldPoint(points[i], frame)
    if worldPt then
      local key = skipMap and makePointKey(worldPt)
      if not key or not skipMap[key] then
        worldPoints[#worldPoints + 1] = worldPt
      end
    end
  end
  return worldPoints
end

local function gatherVehicleWorldPoints(frame)
  local vehiclePoints = {}
  local keyMap = {}
  if vehicle_lidar_frame and vehicle_lidar_point_cloud then
    for i = 1, #vehicle_lidar_point_cloud do
      local drifted = drift_proximity.apply(vehicle_lidar_point_cloud[i], vehicle_lidar_frame, frame)
      local worldPt = toWorldPoint(drifted, frame)
      if worldPt then
        vehiclePoints[#vehiclePoints + 1] = worldPt
        local key = makePointKey(worldPt)
        if key then
          keyMap[key] = true
        end
      end
    end
  end
  return vehiclePoints, keyMap
end

local function ensureVirtualLidarPcdConfigured(active)
  if not virtual_lidar_pcd.path or virtual_lidar_pcd.path == '' then
    virtual_lidar_pcd.path = DEFAULT_VIRTUAL_LIDAR_PCD_PATH
    virtual_lidar_pcd.requestedPath = virtual_lidar_pcd.path
    virtual_lidar_pcd.dirty = true
  end
  if virtual_lidar_pcd.dirty then
    lidarPcdPublisher.configure({
      path = virtual_lidar_pcd.path,
      enabled = active and true or false
    })
    virtual_lidar_pcd.dirty = false
  end

  if lidarPcdPublisher.getOutputPath then
    local actualPath = lidarPcdPublisher.getOutputPath()
    if actualPath and actualPath ~= '' then
      virtual_lidar_pcd.actualPath = actualPath
      virtual_lidar_pcd.path = actualPath
    end
  end
  if lidarPcdPublisher.getRequestedPath then
    local requested = lidarPcdPublisher.getRequestedPath()
    if requested and requested ~= '' then
      virtual_lidar_pcd.requestedPath = requested
    end
  end
  if lidarPcdPublisher.getSandboxFallback then
    local fallback = lidarPcdPublisher.getSandboxFallback()
    if fallback then
      virtual_lidar_pcd.sandboxBlockedPath = fallback.blockedPath
      virtual_lidar_pcd.sandboxReason = fallback.reason
    else
      virtual_lidar_pcd.sandboxBlockedPath = nil
      virtual_lidar_pcd.sandboxReason = nil
    end
  end
end

local function announceVirtualLidarExportPath()
  if not virtual_lidar_pcd.enabled then return end
  local pathMessage = string.format('Virtual LiDAR PCD export path: %s', virtual_lidar_pcd.path)
  ui_message(pathMessage)
  if virtual_lidar_pcd.sandboxBlockedPath and virtual_lidar_pcd.sandboxBlockedPath ~= virtual_lidar_pcd.path then
    local reason = virtual_lidar_pcd.sandboxReason and string.format(' (%s)', virtual_lidar_pcd.sandboxReason) or ''
    ui_message(string.format(
      'Sandbox blocked %s%s; writing to %s instead',
      virtual_lidar_pcd.sandboxBlockedPath,
      reason,
      virtual_lidar_pcd.path
    ))
  end
end

function maybePublishVirtualLidarPcd(veh)
  local exportActive = virtual_lidar_pcd.enabled
  ensureVirtualLidarPcdConfigured(exportActive)
  lidarPcdPublisher.setEnabled(exportActive)

  local streamActive = virtual_lidar_stream.enabled
  if not exportActive and not streamActive then
    return
  end

  local streamReady = false
  if streamActive then
    local ok = ensureVirtualLidarStreamServer()
    streamReady = ok and true or false
  end

  local frame = buildCurrentFrame(veh)
  if not frame then
    if exportActive then
      lidarPcdPublisher.setEnabled(false)
    end
    return
  end

  local vehicleWorld, vehicleKeyMap = gatherVehicleWorldPoints(frame)
  local mainPoints = convertPointsToWorld(getVirtualLidarPointCloud(frame, veh), frame, vehicleKeyMap)
  local groundPoints = convertPointsToWorld(getVirtualLidarGroundPointCloud(frame, veh), frame)
  local overheadPoints = convertPointsToWorld(getVirtualLidarOverheadPointCloud(frame, veh), frame)

  local payload = lidarPcdPublisher.publish(frame, {
    main = mainPoints,
    ground = groundPoints,
    vehicle = vehicleWorld,
    overhead = overheadPoints
  }, {
    writeFile = exportActive,
    wantPayload = streamActive
  })

  if streamActive and streamReady and payload then
    lidarPcdStream.broadcast(payload)
  end
end

local function getVehicleColor(veh)
  veh = veh or be:getPlayerVehicle(0)
  if veh then
    -- Some vehicle APIs expose color via a "color" field rather than a getter.
    -- Fall back to veh:getColor() when available for older versions.
    local c = veh.color or (veh.getColor and veh:getColor())
    if c then
      local r = c.x or c.r or 1
      local g = c.y or c.g or 1
      local b = c.z or c.b or 1
      return {r = r * 255, g = g * 255, b = b * 255}
    end
  end
  return {r = 255, g = 255, b = 255}
end

local function getPlayerVehicleBounds(veh)
  veh = veh or be:getPlayerVehicle(0)
  if not veh then return nil end
  local props = extra_utils.getVehicleProperties(veh)
  if not props or not props.bb then return nil end
  local half = props.bb:getHalfExtents()
  return {width = half.x * 2, length = half.y * 2, height = half.z * 2}
end

local function getVirtualLidarData()
  local veh = be:getPlayerVehicle(0)
  local frame = buildCurrentFrame(veh)
  local points = {}
  local groundPoints = {}
  if frame then
    points = getVirtualLidarPointCloud(frame, veh)
    groundPoints = getVirtualLidarGroundPointCloud(frame, veh)
  end
  return {
    points = points,
    groundPoints = groundPoints,
    color = getVehicleColor(veh),
    bounds = getPlayerVehicleBounds(veh)
  }
end

local function getLaneCenteringData()
  local data = lane_centering_system.getLaneData()
  if data then
    data.color = getVehicleColor()
  end
  return data
end

local function receiveLaneCenteringAiRoute(payload)
  if lane_centering_system and lane_centering_system.updateAiRouteData then
    lane_centering_system.updateAiRouteData(payload)
  end
end

local function setVirtualLidarPcdExportEnabled(flag)
  local enabled = flag and true or false
  virtual_lidar_pcd.enabled = enabled
  local active = enabled
  ensureVirtualLidarPcdConfigured(active)
  lidarPcdPublisher.setEnabled(active)
  announceVirtualLidarExportPath()
end

local function setVirtualLidarPcdOutputPath(path)
  if type(path) ~= 'string' or path == '' then
    path = DEFAULT_VIRTUAL_LIDAR_PCD_PATH
  end
  virtual_lidar_pcd.path = path
  virtual_lidar_pcd.requestedPath = path
  virtual_lidar_pcd.dirty = true
  local active = virtual_lidar_pcd.enabled
  ensureVirtualLidarPcdConfigured(active)
  announceVirtualLidarExportPath()
end

local function setVirtualLidarPcdStreamEnabled(flag)
  local enabled = flag and true or false
  if virtual_lidar_stream.enabled == enabled then return end
  if enabled then
    local ok, err = ensureVirtualLidarStreamServer()
    if not ok then
      local msg = string.format(
        'Failed to enable virtual LiDAR PCD streaming on %s:%d (%s)',
        virtual_lidar_stream.host,
        virtual_lidar_stream.port,
        tostring(err or 'unknown')
      )
      reportLidarError(msg)
      return
    end
    virtual_lidar_stream.enabled = true
    if ui_message then
      ui_message(string.format(
        'Virtual LiDAR PCD streaming enabled on %s:%d',
        virtual_lidar_stream.host,
        virtual_lidar_stream.port
      ))
    end
  else
    virtual_lidar_stream.enabled = false
    if ui_message then
      ui_message('Virtual LiDAR PCD streaming disabled')
    end
  end
end

local function setVirtualLidarPcdStreamPort(port)
  local numeric = tonumber(port)
  if not numeric or math.floor(numeric) ~= numeric or numeric < 1 or numeric > 65535 then
    local msg = string.format('Invalid virtual LiDAR PCD stream port: %s', tostring(port))
    reportLidarError(msg)
    if ui_message then ui_message(msg) end
    return
  end
  if virtual_lidar_stream.port == numeric then return end
  local previousPort = virtual_lidar_stream.port
  virtual_lidar_stream.port = numeric
  local ok, err = ensureVirtualLidarStreamServer()
  if not ok then
    local msg = string.format(
      'Failed to bind virtual LiDAR PCD stream to port %d (%s)',
      numeric,
      tostring(err or 'unknown')
    )
    reportLidarError(msg)
    if ui_message then
      ui_message(string.format('%s. Reverting to port %d', msg, previousPort or virtual_lidar_stream.port))
    end
    virtual_lidar_stream.port = previousPort
    virtual_lidar_stream.activePort = previousPort
    ensureVirtualLidarStreamServer()
    return
  end
  if ui_message then
    ui_message(string.format('Virtual LiDAR PCD stream port set to %d', numeric))
  end
end

local function onInit()
  setExtensionUnloadMode(M, "manual")
end

local function onExtensionUnloaded()
  stopVirtualLidarStreamServer()
end

if lane_centering_system and lane_centering_system.setActivationCallback then
  lane_centering_system.setActivationCallback(handleLaneCenteringActivationRequest)
end

M.onExtensionLoaded = onExtensionLoaded
M.onVehicleSwitched = onVehicleSwitched
M.onHeadlightsOff = onHeadlightsOff
M.onHeadlightsOn = onHeadlightsOn
M.toggleFCMSystem = toggleFCMSystem
M.toggleRCMSystem = toggleRCMSystem
M.toggleObstacleAEBSystem = toggleObstacleAEBSystem
M.toggleLaneCenteringSystem = toggleLaneCenteringSystem
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
M.setVirtualLidarPcdExportEnabled = setVirtualLidarPcdExportEnabled
M.setVirtualLidarPcdOutputPath = setVirtualLidarPcdOutputPath
M.setVirtualLidarPcdStreamEnabled = setVirtualLidarPcdStreamEnabled
M.setVirtualLidarPcdStreamPort = setVirtualLidarPcdStreamPort
M.onCameraModeChanged = onCameraModeChanged
M.onUpdate = onUpdate
M.onInit = onInit
M.onExtensionUnloaded = onExtensionUnloaded
M.getVirtualLidarPointCloud = getVirtualLidarPointCloud
M.getVirtualLidarOverheadPointCloud = getVirtualLidarOverheadPointCloud
M.getVehicleColor = getVehicleColor
M.getVirtualLidarData = getVirtualLidarData
M.getLaneCenteringData = getLaneCenteringData
M.receiveLaneCenteringAiRoute = receiveLaneCenteringAiRoute
M._resetVirtualLidarPointCloud = resetVirtualLidarPointCloud
M._virtual_lidar_point_cloud = function() return virtual_lidar_point_cloud end
M._virtual_lidar_overhead_point_cloud = function() return virtual_lidar_overhead_point_cloud end
M._front_lidar_point_cloud = function() return front_lidar_point_cloud end
M._setFrontLidarThread = function(th) front_lidar_thread = th end

return M
