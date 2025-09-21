-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
--Mandatory controller parameters
M.type = "auxiliary"
-----

local min = math.min
local max = math.max
local abs = math.abs

local trimSpeed = 0
local trimModes = {"elevator", "aileron", "rudder"}
local lastTrimInputs = {elevator = 0, aileron = 0, rudder = 0}
local lastTrimValues = {elevator = 0, aileron = 0, rudder = 0}
local currentTrimMode = nil
local currentTrimModeIndex = 1
local trimming = nil
local flap = 0
local slat = 0
local lastFlap = 0
local lastSlat = 0
local lastFlapInput = nil
local lastSlatInput = nil
local flapSpeed = 0
local slatSpeed = 0

local function updateGFX(dt)
  local rudder = input.rudder or 0
  local aileron = input.aileron or 0
  local elevator = input.elevator or 0

  trimming[currentTrimMode] = min(max(trimming[currentTrimMode] + (input.trimChange or 0) * dt * trimSpeed, -1), 1)

  trimming.rudder = input.trimRudder ~= lastTrimInputs.rudder and input.trimRudder or trimming.rudder
  trimming.elevator = input.trimElevator ~= lastTrimInputs.elevator and input.trimElevator or trimming.elevator
  trimming.aileron = input.trimAileron ~= lastTrimInputs.aileron and input.trimAileron or trimming.aileron

  slat = electrics.values.slat
  flap = electrics.values.flap

  flap = input.flap ~= lastFlapInput and input.flap or min(max(flap + (input.flapChange or 0) * dt * flapSpeed, 0), 1)
  slat = input.slat ~= lastSlatInput and input.slat or min(max(slat + (input.slatChange or 0) * dt * slatSpeed, 0), 1)

  electrics.values.rudder = rudder + trimming.rudder
  electrics.values.aileron = aileron + trimming.aileron
  electrics.values.elevator = elevator + trimming.elevator
  electrics.values.slat = slat
  electrics.values.flap = flap

  lastFlapInput = input.flap
  lastSlatInput = input.slat

  for k, v in pairs(trimming) do
    if abs(v - lastTrimValues[k]) > 0.005 then
      guihooks.message(string.format("Trim (%s): %d%%", k, v * 100), 1, "vehicle.trimvalue" .. k)
      lastTrimValues[k] = v
    end
  end

  if abs(flap - lastFlap) > 0.005 then
    guihooks.message(string.format("Flaps: %d%%", flap * 100), 1, "vehicle.flaps")
    lastFlap = flap
  end
  if abs(slat - lastSlat) > 0.005 then
    guihooks.message(string.format("Slats: %d%%", slat * 100), 1, "vehicle.slats")
    lastSlat = slat
  end
end

local function toggleTrimMode()
  currentTrimModeIndex = currentTrimModeIndex + 1
  if currentTrimModeIndex > #trimModes then
    currentTrimModeIndex = 1
  end
  currentTrimMode = trimModes[currentTrimModeIndex]

  guihooks.message(string.format("Trim mode: %s", currentTrimMode), 5, "vehicle.trimmode")
end

local function setTrimValue(input, value)
  if trimming[input] then
    trimming[input] = value
  end
end

local function init(jbeamData)
  electrics.values.rudder = 0
  electrics.values.aileron = 0
  electrics.values.elevator = 0
  electrics.values.slat = 0
  electrics.values.flap = 0

  flap = jbeamData.flapStart or 0
  slat = jbeamData.slatStart or 0
  electrics.values.slat = slat
  electrics.values.flap = flap
  lastFlap = flap
  lastSlat = slat

  lastFlapInput = 0
  lastSlatInput = 0

  lastTrimInputs = {elevator = 0, aileron = 0, rudder = 0}

  currentTrimModeIndex = 1
  currentTrimMode = trimModes[currentTrimModeIndex]
  trimming = {
    rudder = jbeamData.rudderTrimmingStart or 0,
    aileron = jbeamData.aileronTrimmingStart or 0,
    elevator = jbeamData.elevatorTrimmingStart or 0
  }
  lastTrimValues = {elevator = trimming.elevator, aileron = trimming.aileron, rudder = trimming.rudder}

  trimSpeed = jbeamData.trimSpeed or 0.05
  flapSpeed = jbeamData.flapSpeed or 0.1
  slatSpeed = jbeamData.slatSpeed or 0.1
end

M.init = init
M.updateGFX = updateGFX
M.toggleTrimMode = toggleTrimMode
M.setTrimValue = setTrimValue

return M
