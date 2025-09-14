local M = {}


--Based on US Law (about 500 feet)
local dim_distance = 150

local headlights_turned_off = false
local armed = false

--Called when headlights get turned off by user
local function onHeadlightsOff()
  armed = false
  headlights_turned_off = true
end

--Called when headlights get turned on by user
local function onHeadlightsOn()
  --If in dimmed headlight mode then switch to off
  if armed then
    be:getPlayerVehicle(0):queueLuaCommand("electrics.setLightsState(0)")

    armed = false
    headlights_turned_off = true
  end
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

  if light_state == 2 then
    headlights_turned_off = false
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
    log('I', 'auto_headlight_system', string.format('Detected vehicle %s at %.1f', id, distance))
  end

  --If vehicle in front exists and distance , then dim headlights
  if distance <= dim_distance then
    if light_state ~= 1 then
      log('I', 'auto_headlight_system', 'Switching to low beams')
      veh:queueLuaCommand("electrics.highbeam = false; electrics.setLightsState(1)")
    end
  else
    if light_state ~= 2 then
      log('I', 'auto_headlight_system', 'Restoring high beams')
      veh:queueLuaCommand("electrics.highbeam = true; electrics.setLightsState(2)")
    end
  end
end

local function update(dt, veh, vehs_in_front_table)
  local light_state

  if not headlights_turned_off then
    light_state = getLightState()
  else
    light_state = 0

    if getLightState() == 0 then
      headlights_turned_off = false
    end
  end

  if not armed then
    if light_state == 2 then
      armed = true
    end
  else
    autoHeadlightFunction(veh, vehs_in_front_table or {}, light_state)
  end
end

M.onHeadlightsOff = onHeadlightsOff
M.onHeadlightsOn = onHeadlightsOn
M.systemSwitchedOn = systemSwitchedOn
M.update = update

return M
