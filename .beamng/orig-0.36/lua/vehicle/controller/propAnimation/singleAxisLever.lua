-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local min = math.min
local max = math.max

local currentPosition = 0
local gearCoordinates
local maxExistingGearCoordinateIndex
local minExistingGearCoordinateIndex

local shiftSoundNodeId
local shiftSoundEvent
local shiftSoundVolume

local electricsNamePosition

local positionSmoother = newTemporalSmoothing(2, 2)

local hasPlayedSound = false

local function updateGFX(dt)
  local gearModeIndex = electrics.values.gearModeIndex or 0
  local defaultIndex = gearModeIndex > maxExistingGearCoordinateIndex and maxExistingGearCoordinateIndex or minExistingGearCoordinateIndex
  local targetPosition = gearCoordinates[gearModeIndex] or gearCoordinates[defaultIndex] or 0

  if targetPosition ~= currentPosition then
    currentPosition = positionSmoother:getUncapped(targetPosition, dt)
    if not hasPlayedSound then
      hasPlayedSound = true
      obj:playSFXOnceCT(shiftSoundEvent, shiftSoundNodeId, shiftSoundVolume, 1, 0, 0)
    end
  else
    hasPlayedSound = false
  end

  --move back to _center_ neutral position (X) only when the shifting process is over
  electrics.values[electricsNamePosition] = currentPosition
end

local function reset(jbeamData)
  currentPosition = 0
  electrics.values[electricsNamePosition] = currentPosition
  positionSmoother:reset()

  hasPlayedSound = false
end

local function init(jbeamData)
  local gearCoordinateTable = tableFromHeaderTable(jbeamData.gearCoordinates or {})
  gearCoordinates = {}
  maxExistingGearCoordinateIndex = 0
  minExistingGearCoordinateIndex = 0
  for _, coordinate in pairs(gearCoordinateTable) do
    gearCoordinates[coordinate.gearIndex] = coordinate.value
    maxExistingGearCoordinateIndex = max(maxExistingGearCoordinateIndex, coordinate.gearIndex)
    minExistingGearCoordinateIndex = min(minExistingGearCoordinateIndex, coordinate.gearIndex)
  end
  electricsNamePosition = jbeamData.electricsNamePosition or "gearModeIndex"

  if type(jbeamData.shiftSoundNode_nodes) == "table" and jbeamData.shiftSoundNode_nodes[1] and type(jbeamData.shiftSoundNode_nodes[1]) == "number" then
    shiftSoundNodeId = jbeamData.shiftSoundNode_nodes[1]
  else
    log("W", "propAnimation/singleAxisLever.init", "Can't find node id for sound location. Specified data: " .. dumps(jbeamData.shiftSoundNode_nodes))
    shiftSoundNodeId = 0
  end

  shiftSoundEvent = jbeamData.shiftSoundEventSingleAxisLever or "event:>Vehicle>Interior>Gearshift>automatic_01_in"
  shiftSoundVolume = jbeamData.shiftSoundVolumeSingleAxisLever or 0.5
  hasPlayedSound = false

  bdebug.setNodeDebugText("PropAnimation", shiftSoundNodeId, "Single Axis Lever: " .. shiftSoundEvent)

  currentPosition = 0
  electrics.values[electricsNamePosition] = currentPosition
end

M.init = init
M.reset = reset
M.updateGFX = updateGFX

return M
