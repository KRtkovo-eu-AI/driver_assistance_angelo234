local M = {}

local extra_utils = require('scripts/driver_assistance_angelo234/extraUtils')

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

--If system just switched on, then check if highbeams are already on
--if they are on, then make note of it
local function systemSwitchedOn()
  local highbeam = electrics_values_angelo234["highbeam"]

  if highbeam == 1 then
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

local function autoHeadlightFunction(veh, vehs_in_front_table, highbeam_state)
  local closest_veh_data = getClosestVehicle(vehs_in_front_table)
  local distance = closest_veh_data[2]

  --If vehicle in front exists and distance , then dim headlights
  if distance <= dim_distance then
    if highbeam_state ~= 0 then
      veh:queueLuaCommand("electrics.setLightsState(1); electrics.highbeam=0")
    end
  else
    if highbeam_state ~= 1 then
      veh:queueLuaCommand("electrics.highbeam=1; electrics.setLightsState(2)")
    end
  end
end

local function update(dt, veh, vehs_in_front_table)
  local highbeam_state

  if not headlights_turned_off then
    highbeam_state = electrics_values_angelo234["highbeam"]
  else
    highbeam_state = 0

    if electrics_values_angelo234["lights"] == 0 then
      headlights_turned_off = false
    end
  end

  if not armed then
    if highbeam_state == 1 then
      armed = true
    end
  else
    autoHeadlightFunction(veh, vehs_in_front_table or {}, highbeam_state)
  end
end

M.onHeadlightsOff = onHeadlightsOff
M.onHeadlightsOn = onHeadlightsOn
M.systemSwitchedOn = systemSwitchedOn
M.update = update

return M
