-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local abs = math.abs

local yCurrent = 0
local previousGearIndex = 0
local previousShiftImpulse = 0
local impulseCoordinates = {[-1] = 1, [0] = 0, [1] = -1}

local shiftSoundNodeId
local shiftSoundEventGearUp
local shiftSoundEventGearDown
local shiftSoundVolumeGearUp
local shiftSoundVolumeGearDown

local electricsNameYAxis
local relevantGearbox

local ySmoother = newTemporalSmoothing(10, 10)

local function updateGFX(dt)
  if not relevantGearbox then
    return
  end
  local gearIndex = relevantGearbox.gearIndex or 0

  local shiftAggression = controller.mainController.shiftingAggression or 0
  local shiftImpulse = impulseCoordinates[clamp(gearIndex - previousGearIndex, -1, 1)]

  local impulseIsShiftOut = abs(shiftImpulse) > 0
  local event = impulseIsShiftOut and shiftSoundEventGearDown or shiftSoundEventGearUp
  local volume = impulseIsShiftOut and shiftSoundVolumeGearDown or shiftSoundVolumeGearUp
  yCurrent = ySmoother:getUncapped(shiftImpulse, dt)
  if shiftImpulse ~= previousShiftImpulse then
    obj:playSFXOnceCT(event, shiftSoundNodeId, volume, 1, shiftAggression, 0)
  end
  if yCurrent == shiftImpulse then
    previousGearIndex = gearIndex
  end
  previousShiftImpulse = shiftImpulse

  electrics.values[electricsNameYAxis] = yCurrent
end

local function reset(jbeamData)
  previousGearIndex = 0
  yCurrent = 0
  electrics.values[electricsNameYAxis] = yCurrent
  ySmoother:reset()
end

local function init(jbeamData)
  electricsNameYAxis = jbeamData.electricsNameYAxis or "sequentialLeverY"

  local supportedGearboxTypes = jbeamData.supportedGearboxTypes or {"sequentialGearbox"}
  local supportedGearboxLookup = {}
  for _, gearboxType in ipairs(supportedGearboxTypes) do
    supportedGearboxLookup[gearboxType] = true
  end
  local gearboxName = jbeamData.relevantGearboxName or "gearbox"
  relevantGearbox = powertrain.getDevice(gearboxName)
  if type(jbeamData.shiftSoundNode_nodes) == "table" and jbeamData.shiftSoundNode_nodes[1] and type(jbeamData.shiftSoundNode_nodes[1]) == "number" then
    shiftSoundNodeId = jbeamData.shiftSoundNode_nodes[1]
  else
    log("W", "propAnimation/sequentialLever.init", "Can't find node id for sound location. Specified data: " .. dumps(jbeamData.shiftSoundNode_nodes))
    shiftSoundNodeId = 0
  end

  shiftSoundEventGearUp = jbeamData.shiftSoundEventSequentialGearUp or "event:>Vehicle>Interior>Gearshift>sequential_01_in"
  shiftSoundEventGearDown = jbeamData.shiftSoundEventSequentialGearDown or "event:>Vehicle>Interior>Gearshift>sequential_01_out"
  shiftSoundVolumeGearUp = jbeamData.shiftSoundVolumeSequentialGearUp or 0.5
  shiftSoundVolumeGearDown = jbeamData.shiftSoundVolumeSequentialGearDown or 0.5

  bdebug.setNodeDebugText("PropAnimation", shiftSoundNodeId, "Sequential Lever Up: " .. shiftSoundEventGearUp)
  bdebug.setNodeDebugText("PropAnimation", shiftSoundNodeId, "Sequential Lever Down: " .. shiftSoundEventGearDown)

  if not relevantGearbox then
    --no gearbox device
    log("E", "propAnimation/sequentialLever.init", "Can't find relevant gearbox device with name: " .. gearboxName)
    M.updateGFX = nop
  end

  if relevantGearbox and not supportedGearboxLookup[relevantGearbox.type] then
    --gearbox device is the wrong type
    log("E", "propAnimation/sequentialLever.init", "Relevant gearbox device is the wrong type. Expected: " .. dumps(supportedGearboxTypes) .. ", actual: " .. relevantGearbox.type)
    M.updateGFX = nop
  end

  previousGearIndex = 0
  yCurrent = 0
  electrics.values[electricsNameYAxis] = yCurrent
end

M.init = init
M.reset = reset
M.updateGFX = updateGFX

return M
