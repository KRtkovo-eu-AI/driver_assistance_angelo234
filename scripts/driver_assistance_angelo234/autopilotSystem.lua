local M = {}

local extra_utils = require('scripts/driver_assistance_angelo234/extraUtils')
local acc_system = require('scripts/driver_assistance_angelo234/accSystem')

local lane_pid = nil
local autopilot_params = nil

local clamp = clamp

local function configure(system_params)
  autopilot_params = system_params.autopilot_params or {}
  local gains = autopilot_params.lane_pid or {p = 0.5, i = 0.0, d = 0.05}
  lane_pid = newPIDStandard(gains.p, gains.i, gains.d, -1, 1, 1, 1, 0, 2)
end

local function onToggled(on, system_params)
  local veh = be:getPlayerVehicle(0)
  if not veh then return end

  if on then
    configure(system_params or {})
    acc_system.onToggled(true)
    acc_system.setACCSpeed()

    if autopilot_params and autopilot_params.target_speed then
      local veh_speed = vec3(veh:getVelocity()):length()
      local diff = autopilot_params.target_speed - veh_speed
      local units = settings.getValue("uiUnitLength")
      if units == "metric" then
        acc_system.changeACCSpeed(diff * 3.6)
      elseif units == "imperial" then
        acc_system.changeACCSpeed(diff * 2.24)
      else
        acc_system.changeACCSpeed(diff)
      end
    end

    ui_message("Autopilot switched ON")
  else
    acc_system.onToggled(false)
    if lane_pid then lane_pid:reset() end
    veh:queueLuaCommand("electrics.values.steeringOverride = nil")
    ui_message("Autopilot switched OFF")
  end
end

local function update(dt, veh, system_params, aeb_params, front_sensor_data)
  if not lane_pid then
    configure(system_params)
  end

  local steer_input = electrics_values_angelo234["steering_input"] or 0
  if input_throttle_angelo234 > 0.1 or input_brake_angelo234 > 0.1 or math.abs(steer_input) > 0.1 then
    scripts_driver__assistance__angelo234_extension.setAutopilotOn(false)
    return
  end

  acc_system.update(dt, veh, system_params, aeb_params, front_sensor_data)

  local veh_props = extra_utils.getVehicleProperties(veh)
  local wps_props = nil

  if front_sensor_data and front_sensor_data[3] and next(front_sensor_data[3]) ~= nil then
    wps_props = front_sensor_data[3][1].my_veh_wps_props
  end
  if not wps_props then
    wps_props = extra_utils.getWaypointStartEndAdvanced(veh_props, veh_props, veh_props.front_pos)
  end

  if wps_props and lane_pid then
    local steer = lane_pid:get(wps_props.lat_dist_from_wp, 0, dt)
    steer = clamp(steer, -1, 1)
    veh:queueLuaCommand("electrics.values.steeringOverride = " .. steer)
  end
end

M.onToggled = onToggled
M.update = update

return M
