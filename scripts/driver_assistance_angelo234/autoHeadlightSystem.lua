local M = {}

local logger = require('scripts/driver_assistance_angelo234/logger')


--Based on US Law (about 500 feet)
local dim_distance = 150

local headlights_turned_off = false
local armed = false
local desired_light_state = nil
local pending_light_state = nil
local last_applied_light_state = nil
local manual_override_state = nil

--Called when headlights get turned off by user
local function onHeadlightsOff()
  armed = false
  manual_override_state = 0
  desired_light_state = nil
  pending_light_state = nil
  headlights_turned_off = true
end

--Called when headlights get turned on by user
local function onHeadlightsOn()
  headlights_turned_off = false
end

--If system just switched on, then check if headlights are already in high-beam mode
--if they are, then make note of it
local function getLightState()
  return electrics_values_angelo234["lights_state"]
    or electrics_values_angelo234["lights"]
    or 0
end

local function systemSwitchedOn()
  local light_state = getLightState()

  if electrics_values_angelo234 ~= nil then
    light_state = electrics_values_angelo234["lights_state"]
  end

  manual_override_state = nil
  pending_light_state = nil
  desired_light_state = nil
  last_applied_light_state = light_state

  headlights_turned_off = light_state == 0
  armed = light_state == 2

  if armed then
    desired_light_state = 2
  end
end

local function getClosestVehicle(other_vehs_data)
  local distance = 9999
  local other_veh = nil

  for _, other_veh_data in pairs(other_vehs_data) do
    local veh = other_veh_data.other_veh
    local this_distance = other_veh_data.shortest_dist

    if this_distance <= distance then
      distance = this_distance
      other_veh = veh
    end
  end

  return {other_veh, distance}
end

local function autoHeadlightFunction(veh, vehs_in_front_table, light_state)
  local closest_veh_data = getClosestVehicle(vehs_in_front_table)
  local other_veh = closest_veh_data[1]
  local distance = closest_veh_data[2]

  if other_veh ~= nil then
    local id
    if other_veh.getJBeamFilename then
      id = other_veh:getJBeamFilename()
    elseif other_veh.getID then
      id = tostring(other_veh:getID())
    else
      id = "unknown"
    end
    logger.log('I', 'auto_headlight_system', string.format('Detected vehicle %s at %.1f', id, distance))
  end

  --If vehicle in front exists and distance , then dim headlights
  if distance <= dim_distance then
    desired_light_state = 1

    if light_state ~= 1 then
      logger.log('I', 'auto_headlight_system', 'Attempting to switch to low beams')
      pending_light_state = 1
      veh:queueLuaCommand("electrics.highbeam = false; electrics.setLightsState(1)")
    end
  else
    desired_light_state = 2

    if light_state ~= 2 then
      logger.log('I', 'auto_headlight_system', 'Attempting to restore high beams')
      pending_light_state = 2
      veh:queueLuaCommand("electrics.highbeam = true; electrics.setLightsState(2)")
    end
  end
end

local function update(dt, veh, vehs_in_front_table)
  local reported_light_state = getLightState()
  local light_state = reported_light_state

  if headlights_turned_off then
    light_state = 0

    if reported_light_state == 0 then
      headlights_turned_off = false
    end
  end

  if pending_light_state and light_state == pending_light_state then
    last_applied_light_state = light_state
    pending_light_state = nil
  end

  if manual_override_state ~= nil then
    if light_state == 2 then
      manual_override_state = nil
      headlights_turned_off = false
      armed = true
      desired_light_state = 2
      last_applied_light_state = 2
    else
      manual_override_state = light_state
      headlights_turned_off = light_state == 0
      return
    end
  end

  if armed and desired_light_state ~= nil then
    local awaiting_command = pending_light_state ~= nil and pending_light_state == desired_light_state
    local manual_change = last_applied_light_state ~= nil and light_state ~= last_applied_light_state

    if light_state ~= desired_light_state and light_state ~= 2 and (not awaiting_command or manual_change) then
      manual_override_state = light_state
      headlights_turned_off = light_state == 0
      armed = false
      desired_light_state = nil
      pending_light_state = nil
      return
    end
  end

  if not armed then
    if light_state == 2 then
      armed = true
      manual_override_state = nil
      headlights_turned_off = false
      desired_light_state = 2
      last_applied_light_state = 2
    else
      if light_state == 0 then
        headlights_turned_off = true
      end
      return
    end
  end

  last_applied_light_state = last_applied_light_state or light_state

  autoHeadlightFunction(veh, vehs_in_front_table or {}, light_state)
end

M.onHeadlightsOff = onHeadlightsOff
M.onHeadlightsOn = onHeadlightsOn
M.systemSwitchedOn = systemSwitchedOn
M.update = update

return M
