-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local dequeue = require("dequeue")

local M = {}
M.type = "auxiliary"

local abs = math.abs
local min = math.min
local max = math.max

local xCurrent = 0
local yCurrent = 0
local previousGearIndex = 0
local gearCoordinates
local maxExistingGearCoordinateIndex
local minExistingGearCoordinateIndex

local shiftSoundNodeId
local shiftSoundEventGearIn
local shiftSoundEventGearOut
local shiftSoundVolumeGearIn
local shiftSoundVolumeGearOut

local electricsNameXAxis
local electricsNameYAxis
local relevantGearbox

local xSmoother = newTemporalSmoothing(10, 10)
local ySmoother = newTemporalSmoothing(10, 10)
local isShiftingSmoother = newTemporalSmoothing(1, 1000000)

local xSmootherRate = 15
local ySmootherRate = 15
local hasPlayedSound = false

local isMoving = false
local targetCoordinates
local currentTargetGearIndex

local demoMode = false
local demoGearIndex = 0
local demoGearIndexOffset = 1
local demoGearTimer = 0
local demoIsNeutral = false

local gearIndexQueue

local function getGearIndexDemo(dt)
  demoGearTimer = demoGearTimer + dt
  if demoGearTimer > 0.2 and demoIsNeutral then
    demoIsNeutral = false
    demoGearTimer = demoGearTimer - 0.2
  end

  if demoGearTimer > 1 and not demoIsNeutral then
    demoGearTimer = demoGearTimer - 1
    demoGearIndex = demoGearIndex + demoGearIndexOffset
    if not gearCoordinates[demoGearIndex] then
      demoGearIndexOffset = -demoGearIndexOffset
      demoGearIndex = demoGearIndex + 2 * demoGearIndexOffset
    end
    demoIsNeutral = true
  end
  return demoIsNeutral and 0 or demoGearIndex
end

local function updateGFX(dt)
  if not relevantGearbox and not demoMode then
    return
  end
  --local gearIndex = demoMode and getGearIndexDemo(dt) or relevantGearbox.gearIndex or 0

  local currentGearIndex = demoMode and getGearIndexDemo(dt) or relevantGearbox.gearIndex or 0
  if currentGearIndex ~= gearIndexQueue:peek_left() then
    gearIndexQueue:push_left(currentGearIndex)
  end

  if not isMoving then
    currentTargetGearIndex = gearIndexQueue:pop_right()
    local defaultIndex = currentTargetGearIndex > maxExistingGearCoordinateIndex and maxExistingGearCoordinateIndex or minExistingGearCoordinateIndex
    targetCoordinates = gearCoordinates[currentTargetGearIndex] or gearCoordinates[defaultIndex] or {x = 0, y = 0}
  end

  local shiftAggression = controller.mainController.shiftingAggression or 0
  local shiftAnimationSpeedCoef = linearScale(shiftAggression, 0, 1, 0.3, 1)

  if targetCoordinates then
    if abs(currentTargetGearIndex) > abs(previousGearIndex) then
      --shifting _into_ gear, move X first, then Y
      isMoving = true
      if targetCoordinates.x ~= xCurrent then
        xCurrent = xSmoother:getWithRateUncapped(targetCoordinates.x, dt, xSmootherRate * shiftAnimationSpeedCoef)
      elseif targetCoordinates.y ~= yCurrent then
        if not hasPlayedSound then
          hasPlayedSound = true
          obj:playSFXOnceCT(shiftSoundEventGearIn, shiftSoundNodeId, shiftSoundVolumeGearIn, 1, shiftAggression, 0)
        end
        yCurrent = ySmoother:getWithRateUncapped(targetCoordinates.y, dt, ySmootherRate * shiftAnimationSpeedCoef)
      else
        previousGearIndex = currentTargetGearIndex
        hasPlayedSound = false
        isMoving = false
      end
    elseif abs(currentTargetGearIndex) < abs(previousGearIndex) then
      --shifting out of gear, move Y first then X
      isMoving = true
      if targetCoordinates.y ~= yCurrent then
        if not hasPlayedSound then
          hasPlayedSound = true
          obj:playSFXOnceCT(shiftSoundEventGearOut, shiftSoundNodeId, shiftSoundVolumeGearOut, 1, shiftAggression, 0)
        end
        yCurrent = ySmoother:getWithRateUncapped(targetCoordinates.y, dt, ySmootherRate * shiftAnimationSpeedCoef)
      elseif targetCoordinates.x ~= xCurrent and currentTargetGearIndex ~= 0 then
        --we are not changing the X position when going into neutral so that the movement looks smoother
        xCurrent = xSmoother:getWithRateUncapped(targetCoordinates.x, dt, xSmootherRate * shiftAnimationSpeedCoef)
      else
        previousGearIndex = currentTargetGearIndex
        hasPlayedSound = false
        isMoving = false
      end
    end
  end

  --move back to _center_ neutral position (X) only when the shifting process is over
  local isShifting = electrics.values.isShifting or false
  local isShiftingSmooth = isShiftingSmoother:get(isShifting and 1 or 0, dt)
  if currentTargetGearIndex == 0 and isShiftingSmooth <= 0 and not isMoving and gearIndexQueue:is_empty() and xCurrent ~= 0 then
    xCurrent = (gearCoordinates[0] or {x = 0, y = 0}).x
    xSmoother:set(xCurrent)
    hasPlayedSound = false
  end

  electrics.values[electricsNameXAxis] = xCurrent
  electrics.values[electricsNameYAxis] = yCurrent
end

local function reset(jbeamData)
  previousGearIndex = 0
  xCurrent = 0
  yCurrent = 0
  electrics.values[electricsNameXAxis] = xCurrent
  electrics.values[electricsNameYAxis] = yCurrent
  xSmoother:reset()
  ySmoother:reset()
  gearIndexQueue = dequeue.new()

  demoGearIndex = 0
  demoGearIndexOffset = 1
  hasPlayedSound = false
  isMoving = false
  targetCoordinates = nil
end

local function init(jbeamData)
  local gearCoordinateTable = tableFromHeaderTable(jbeamData.gearCoordinates or {})
  gearCoordinates = {}
  maxExistingGearCoordinateIndex = 0
  minExistingGearCoordinateIndex = 0
  for _, coordinate in pairs(gearCoordinateTable) do
    gearCoordinates[coordinate.gearIndex] = {x = coordinate.x, y = coordinate.y}
    maxExistingGearCoordinateIndex = max(maxExistingGearCoordinateIndex, coordinate.gearIndex)
    minExistingGearCoordinateIndex = min(minExistingGearCoordinateIndex, coordinate.gearIndex)
  end
  electricsNameXAxis = jbeamData.electricsNameXAxis or "hPatternAxisX"
  electricsNameYAxis = jbeamData.electricsNameYAxis or "hPatternAxisY"

  local supportedGearboxTypes = jbeamData.supportedGearboxTypes or {"manualGearbox"}
  local supportedGearboxLookup = {}
  for _, gearboxType in ipairs(supportedGearboxTypes) do
    supportedGearboxLookup[gearboxType] = true
  end
  local gearboxName = jbeamData.relevantGearboxName or "gearbox"
  relevantGearbox = powertrain.getDevice(gearboxName)
  if type(jbeamData.shiftSoundNode_nodes) == "table" and jbeamData.shiftSoundNode_nodes[1] and type(jbeamData.shiftSoundNode_nodes[1]) == "number" then
    shiftSoundNodeId = jbeamData.shiftSoundNode_nodes[1]
  else
    log("W", "propAnimation/hPattern.init", "Can't find node id for sound location. Specified data: " .. dumps(jbeamData.shiftSoundNode_nodes))
    shiftSoundNodeId = 0
  end

  shiftSoundEventGearIn = jbeamData.shiftSoundEventHPatternGearIn or "event:>Vehicle>Interior>Gearshift>manual_modern_01_in"
  shiftSoundEventGearOut = jbeamData.shiftSoundEventHPatternGearOut or "event:>Vehicle>Interior>Gearshift>manual_modern_01_out"
  shiftSoundVolumeGearIn = jbeamData.shiftSoundVolumeHPatternGearIn or 0.5
  shiftSoundVolumeGearOut = jbeamData.shiftSoundVolumeHPatternGearOut or 0.5
  hasPlayedSound = false

  bdebug.setNodeDebugText("PropAnimation", shiftSoundNodeId, "H-Pattern Shifter In: " .. shiftSoundEventGearIn)
  bdebug.setNodeDebugText("PropAnimation", shiftSoundNodeId, "H-Pattern Shifter Out: " .. shiftSoundEventGearOut)

  if not relevantGearbox then
    --no gearbox device
    log("E", "propAnimation/hPattern.init", "Can't find relevant gearbox device with name: " .. gearboxName)
    M.updateGFX = nop
  end

  if relevantGearbox and not supportedGearboxLookup[relevantGearbox.type] then
    --gearbox device is the wrong type
    log("E", "propAnimation/hPattern.init", "Relevant gearbox device is the wrong type. Expected: " .. dumps(supportedGearboxTypes) .. ", actual: " .. relevantGearbox.type)
    M.updateGFX = nop
  end

  gearIndexQueue = dequeue.new()
  isMoving = false
  previousGearIndex = 0
  xCurrent = 0
  yCurrent = 0
  electrics.values[electricsNameXAxis] = xCurrent
  electrics.values[electricsNameYAxis] = yCurrent

  demoMode = jbeamData.demoMode or false
  demoGearIndex = 0
  demoGearIndexOffset = 1
  demoIsNeutral = false
  if demoMode then
    M.updateGFX = updateGFX
  end
end

M.init = init
M.reset = reset
M.updateGFX = updateGFX

return M
