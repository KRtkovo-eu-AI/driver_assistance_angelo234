-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local timer = 1
local totalSimTime = 0
local totalDistance = 0
local previousFuel = 0
local startingFuel
local data = {}

local function reset()
  totalDistance = 0
  timer = 1
  totalSimTime = 0
  previousFuel = 0
  startingFuel = nil
end

local function updateGFX(dtSim)
  local dtReal = obj:getRealdt()
  startingFuel = startingFuel or electrics.values.fuelVolume
  local speed = next(wheels.wheels) and electrics.values.wheelspeed or electrics.values.airspeed
  totalSimTime = totalSimTime + dtSim
  totalDistance = totalDistance + (dtSim * speed)

  timer = timer - dtReal
  if timer < 0 then
    local avgFuelConsumptionRate = 0
    local fuelConsumptionRate
    local range

    if startingFuel then
      if previousFuel > electrics.values.fuelVolume and (previousFuel - electrics.values.fuelVolume) > 0.0002 then
        fuelConsumptionRate = (previousFuel - electrics.values.fuelVolume) / ((1 - timer) * speed)
      else
        fuelConsumptionRate = 0
      end
      previousFuel = electrics.values.fuelVolume

      if fuelConsumptionRate > 0 then
        range = (speed > 0.1) and (electrics.values.fuelVolume / fuelConsumptionRate) or 0
      else
        range = (speed > 0.1) and math.huge or 0
      end

      avgFuelConsumptionRate = (startingFuel - electrics.values.fuelVolume) / totalDistance
    end

    data.totalDistance = totalDistance
    data.avgSpeed = totalDistance / totalSimTime
    data.avgFuelConsumptionRate = avgFuelConsumptionRate
    data.fuelConsumptionRate = fuelConsumptionRate
    data.range = range
    guihooks.trigger("tripData", data)
    timer = 1
  end
end

local function shouldExtensionLoad()
  return next(wheels.wheels) or not tableIsEmpty(powertrain.getDevices())
end

local function onExtensionLoaded()
  if not shouldExtensionLoad() then
    return false
  end
  reset()
end

local function onPlayersChanged(anyPlayerSeated)
  if not anyPlayerSeated or not shouldExtensionLoad() then
    extensions.unload("simpleTripApp")
  end
end

-- public interface
M.updateGFX = updateGFX
M.onExtensionLoaded = onExtensionLoaded
M.onPlayersChanged = onPlayersChanged

M.reset = reset

return M
