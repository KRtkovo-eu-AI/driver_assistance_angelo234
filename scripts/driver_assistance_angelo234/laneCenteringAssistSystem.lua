-- luacheck: globals vec3 laneCentering_aiCommand

local M = {}

local extra_utils = require('scripts/driver_assistance_angelo234/extraUtils')

local abs = math.abs
local max = math.max
local clamp = function(value, low, high)
  if value < low then return low end
  if value > high then return high end
  return value
end

local TRAJECTORY_MAX = 90

local latest_data = {
  status = {
    installed = false,
    enabled = false,
    active = false,
    available = false,
    reason = 'init'
  },
  ai = {
    command = {steer = 0, throttle = 0, brake = 0, finalSteer = 0, blend = 0},
    trajectory = {points = {}, limit = TRAJECTORY_MAX},
    debug = {mode = 'off'}
  },
  driver = {steering = 0, throttle = 0, brake = 0},
  vehicle = {speed = 0}
}

local activation_handler = nil
local override_timer = 0
local ai_session = {active = false}
local trajectory_buffer = {}

local function resetTrajectory()
  trajectory_buffer = {}
  latest_data.ai.trajectory = {points = {}, limit = TRAJECTORY_MAX}
end

local function updateStatus(status)
  latest_data.status = status
end

local function queueVehicleCommand(veh, cmd)
  if veh and cmd then
    veh:queueLuaCommand(cmd)
  end
end

local function fetchAiCommand()
  local command = rawget(_G, 'laneCentering_aiCommand')
  if type(command) ~= 'table' then
    return {steer = 0, throttle = 0, brake = 0}
  end
  return {
    steer = clamp(command.steer or 0, -1, 1),
    throttle = clamp(command.throttle or 0, -1, 1),
    brake = clamp(command.brake or 0, -1, 1)
  }
end

local function pollAiCommand(veh)
  if not veh then return end
  local cmd = [[
    local steer, throttle, brake = 0, 0, 0
    if input.lastInputs and input.lastInputs.ai then
      local aiIn = input.lastInputs.ai
      if type(aiIn.steering) == 'number' then steer = aiIn.steering end
      if type(aiIn.throttle) == 'number' then throttle = aiIn.throttle end
      if type(aiIn.brake) == 'number' then brake = aiIn.brake end
    end
    obj:queueGameEngineLua(string.format(
      "laneCentering_aiCommand = {steer = %.4f, throttle = %.4f, brake = %.4f}",
      steer,
      throttle,
      brake
    ))
  ]]
  queueVehicleCommand(veh, cmd)
end

local function startAiSession(veh)
  if ai_session.active then return end
  ai_session.active = true
  local cmd = [[
    laneCenteringAssist = laneCenteringAssist or {}
    local state = ai.getState()
    laneCenteringAssist.prevState = {
      mode = state.mode,
      speedMode = state.speedMode,
      driveInLaneFlag = state.driveInLaneFlag,
      debugMode = state.debugMode
    }
    ai.setState({mode = 'traffic'})
    ai.driveInLane('on')
    ai.setSpeedMode('off')
    ai.setVehicleDebugMode({debugMode = 'trajectory'})
    laneCenteringAssist.active = true
  ]]
  queueVehicleCommand(veh, cmd)
end

local function restoreAiState()
  latest_data.ai.debug = {mode = 'off'}
  rawset(_G, 'laneCentering_aiCommand', nil)
end

local function stopAiSession(veh)
  if not ai_session.active then return end
  ai_session.active = false
  local cmd = [[
    if laneCenteringAssist and laneCenteringAssist.prevState then
      local st = laneCenteringAssist.prevState
      laneCenteringAssist.prevState = nil
      if st.mode then
        ai.setState({mode = st.mode})
      else
        ai.setState({mode = 'disabled'})
      end
      if st.driveInLaneFlag then
        ai.driveInLane(st.driveInLaneFlag)
      else
        ai.driveInLane('off')
      end
      if st.speedMode then
        ai.setSpeedMode(st.speedMode)
      else
        ai.setSpeedMode('off')
      end
      ai.setVehicleDebugMode({debugMode = st.debugMode or 'off'})
    else
      ai.setState({mode = 'disabled'})
      ai.driveInLane('off')
      ai.setSpeedMode('off')
      ai.setVehicleDebugMode({debugMode = 'off'})
    end
    laneCenteringAssist = nil
  ]]
  queueVehicleCommand(veh, cmd)
  restoreAiState()
end

local function applyDriverInputs(veh, throttle, brake)
  if not veh then return end
  if throttle then
    queueVehicleCommand(veh, string.format(
      "input.event('throttle', %.4f, 'FILTER_LANE_CENTERING', nil, nil, nil, 'lane_centering')",
      clamp(throttle, -1, 1)
    ))
  end
  if brake then
    queueVehicleCommand(veh, string.format(
      "input.event('brake', %.4f, 'FILTER_LANE_CENTERING', nil, nil, nil, 'lane_centering')",
      clamp(brake, -1, 1)
    ))
  end
end

local function applySteering(veh, ai_command, driver_input, params)
  local limit = params.steer_limit or 0.6
  local driver = clamp(driver_input or 0, -1, 1)
  local ai_steer = clamp(ai_command.steer or 0, -limit, limit)
  local gain = params.driver_mix_gain or 4.0
  local blend = clamp(1 - abs(driver) * gain, 0, 1)
  local final = clamp(driver * (1 - blend) + ai_steer * blend, -1, 1)
  queueVehicleCommand(veh, string.format(
    "input.event('steering', %.4f, 'FILTER_LANE_CENTERING', nil, nil, nil, 'lane_centering')",
    final
  ))
  return final, ai_steer, blend
end

local function recordTrajectory(veh_props, ai_command, active)
  if not active then return end
  if not veh_props or not veh_props.center_pos then return end
  local position = vec3(veh_props.center_pos)
  local forward = veh_props.velocity and veh_props.velocity:dot(veh_props.dir) or 0
  trajectory_buffer[#trajectory_buffer + 1] = {
    pos = position,
    speed = forward,
    steer = ai_command.steer or 0,
    throttle = ai_command.throttle or 0,
    brake = ai_command.brake or 0
  }
  if #trajectory_buffer > TRAJECTORY_MAX then
    table.remove(trajectory_buffer, 1)
  end
end

local function buildTrajectoryData(veh_props)
  local data = {points = {}, limit = TRAJECTORY_MAX}
  if not veh_props or not veh_props.center_pos then
    return data
  end

  local base = veh_props.center_pos
  local forward = vec3(veh_props.dir.x, veh_props.dir.y, 0)
  if forward:length() < 1e-6 then
    return data
  end
  forward = forward:normalized()
  local right = vec3(forward.y, -forward.x, 0)

  for i = #trajectory_buffer, 1, -1 do
    local entry = trajectory_buffer[i]
    local delta = entry.pos - base
    data.points[#data.points + 1] = {
      x = delta:dot(right),
      y = delta:dot(forward),
      speed = entry.speed,
      steer = entry.steer,
      throttle = entry.throttle,
      brake = entry.brake
    }
  end

  return data
end

local function computeDriverInput()
  local driver = rawget(_G, 'input_steering_driver_angelo234')
  if type(driver) ~= 'number' then
    driver = rawget(_G, 'input_steering_angelo234') or 0
  end
  return clamp(driver or 0, -1, 1)
end

local function computeDriverThrottle()
  local throttle = rawget(_G, 'input_throttle_angelo234')
  if type(throttle) ~= 'number' then throttle = 0 end
  return clamp(throttle, -1, 1)
end

local function computeDriverBrake()
  local brake = rawget(_G, 'input_brake_angelo234')
  if type(brake) ~= 'number' then brake = 0 end
  return clamp(brake, 0, 1)
end

local function update(dt, veh, system_params, enabled)
  override_timer = max((override_timer or 0) - (dt or 0), 0)

  local params = (system_params and system_params.lane_centering_params) or {}
  local installed = extra_utils.getPart('lane_centering_assist_system_angelo234')
    or extra_utils.getPart('lane_centering_assist_angelo234')

  local status = {
    installed = installed and true or false,
    enabled = installed and enabled or false,
    active = false,
    available = false,
    driverOverride = false,
    reason = nil
  }

  if not installed then
    stopAiSession(veh)
    resetTrajectory()
    status.reason = 'not_installed'
    updateStatus(status)
    latest_data.ai.command = {steer = 0, throttle = 0, brake = 0, finalSteer = 0, blend = 0}
    latest_data.ai.debug = {mode = 'off'}
    latest_data.driver = {steering = 0, throttle = 0, brake = 0}
    latest_data.vehicle = {speed = 0}
    return
  end

  if not enabled then
    stopAiSession(veh)
    status.reason = 'user_disabled'
    updateStatus(status)
    latest_data.ai.debug = {mode = 'off'}
    latest_data.driver = {steering = computeDriverInput(), throttle = computeDriverThrottle(), brake = computeDriverBrake()}
    latest_data.vehicle = {speed = 0}
    return
  end

  if not veh then
    status.reason = 'no_vehicle'
    updateStatus(status)
    latest_data.ai.debug = {mode = 'off'}
    return
  end

  local veh_props = extra_utils.getVehicleProperties(veh)
  if not veh_props then
    status.reason = 'no_vehicle_props'
    updateStatus(status)
    return
  end

  local forward_speed = veh_props.velocity and veh_props.velocity:dot(veh_props.dir) or 0
  local driver_input = computeDriverInput()
  local driver_throttle = computeDriverThrottle()
  local driver_brake = computeDriverBrake()

  local min_speed = params.min_active_speed or 1.0
  local override_threshold = params.override_threshold or 0.35

  if abs(driver_input) > override_threshold then
    status.driverOverride = true
    status.reason = 'driver_override'
    override_timer = params.override_cooldown or 3
    stopAiSession(veh)
    if activation_handler then
      activation_handler(false, 'driver_override')
    end
    updateStatus(status)
    latest_data.ai.command = {steer = 0, throttle = 0, brake = 0, finalSteer = driver_input, blend = 0}
    latest_data.ai.debug = {mode = 'off'}
    latest_data.driver = {steering = driver_input, throttle = driver_throttle, brake = driver_brake}
    latest_data.vehicle = {speed = forward_speed}
    return
  end

  local ready = forward_speed > min_speed and override_timer <= 0
  status.available = ready

  if ready then
    startAiSession(veh)
    pollAiCommand(veh)
    local ai_command = fetchAiCommand()

    applyDriverInputs(veh, driver_throttle, driver_brake)
    local final, ai_steer, blend = applySteering(veh, ai_command, driver_input, params)

    ai_command.finalSteer = final
    ai_command.aiSteer = ai_steer
    ai_command.blend = blend

    recordTrajectory(veh_props, ai_command, true)

    status.active = true
    status.reason = nil
    latest_data.ai.command = ai_command
    latest_data.ai.trajectory = buildTrajectoryData(veh_props)
    latest_data.ai.debug = {mode = 'trajectory'}
    latest_data.driver = {
      steering = driver_input,
      throttle = driver_throttle,
      brake = driver_brake
    }
    latest_data.vehicle = {speed = forward_speed}
  else
    if ai_session.active then
      stopAiSession(veh)
    end
    if override_timer > 0 then
      status.reason = 'cooldown'
    elseif forward_speed <= min_speed then
      status.reason = 'low_speed'
    else
      status.reason = status.reason or 'standby'
    end
    recordTrajectory(veh_props, {steer = 0, throttle = 0, brake = 0}, false)
    latest_data.ai.command = {steer = 0, throttle = 0, brake = 0, finalSteer = driver_input, blend = 0}
    latest_data.ai.trajectory = buildTrajectoryData(veh_props)
    latest_data.ai.debug = {mode = 'off'}
    latest_data.driver = {
      steering = driver_input,
      throttle = driver_throttle,
      brake = driver_brake
    }
    latest_data.vehicle = {speed = forward_speed}
  end

  updateStatus(status)
end

local function getLaneData()
  return latest_data
end

local function setActivationCallback(cb)
  activation_handler = cb
end

M.update = update
M.getLaneData = getLaneData
M.setActivationCallback = setActivationCallback

return M
