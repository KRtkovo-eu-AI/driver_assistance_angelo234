-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local dequeue = require("dequeue")

local M = {}
M.type = "auxiliary"

local abs = math.abs

local xCurrent = 0
local yCurrent = 0
local modeCoordinates
local modeCoordinatesIndexLookup
local modeChanges
local intermediateCoordinateIndex

local moveSoundNodeId
local moveSoundEvent
local moveSoundVolume

local electricsNameXAxis
local electricsNameYAxis

local xSmoother
local ySmoother

local targetCoordinates
local currentMode
local lastCurrentMode
local electricsNameMode

local demoModes = false
local demoGearIndex
local demoGearIndexOffset
local demoGearTimer = 0

local targetModeQueue
local lastHandledDesiredMode

local lastMModeGearIndex

local movementStates = {
  none = "none",
  moving = "moving",
  reachedPosition = "reachedPosition"
}
local movementState = movementStates.none

local gearModeStates = {
  none = "none",
  actualPosition = "actualPosition",
  intermediatePosition = "intermediatePosition",
  manualModePosition = "manualModePosition"
}
local gearModeState = gearModeStates.none

local getDesiredModeFunction

local function getDemoGearMode(dt)
  if targetModeQueue:is_empty() then
    demoGearTimer = demoGearTimer + dt
  else
    demoGearTimer = 0
  end

  if demoGearTimer > 0.5 then
    demoGearTimer = 0
    demoGearIndex = demoGearIndex + demoGearIndexOffset
    if not demoModes[demoGearIndex] then
      demoGearIndexOffset = -demoGearIndexOffset
      demoGearIndex = demoGearIndex + 2 * demoGearIndexOffset
      if not demoModes[demoGearIndex] then
        demoGearIndex = 1
      end
    end
  end
  return demoModes[demoGearIndex]
end

local function getDesiredModeDemo(dt)
  if demoModes then
    return getDemoGearMode(dt)
  end
end

local function getDesiredMode(dt)
  return electrics.values[electricsNameMode]
end

local function handleMSModes(desiredMode)
  local firstLetter = desiredMode:sub(1, 1)
  if firstLetter == "S" then
    desiredMode = "S"
  elseif firstLetter == "M" then
    local gearIndex = desiredMode:sub(2, desiredMode:len())
    desiredMode = "M"
    local secondDesiredMode

    if lastMModeGearIndex then
      local postfix
      if gearIndex > lastMModeGearIndex then
        postfix = "+"
      elseif gearIndex < lastMModeGearIndex then
        postfix = "-"
      end
      if postfix then
        desiredMode = desiredMode .. postfix
        secondDesiredMode = "M"
      end
      lastMModeGearIndex = gearIndex
    else
      lastMModeGearIndex = gearIndex
    end

    return desiredMode, secondDesiredMode
  end

  return desiredMode
end

local function updateTargetQueue(dt)
  if not targetModeQueue:is_empty() then
    return
  end
  local desiredMode = getDesiredModeFunction(dt)
  -- ignore any non string values here (like from a manual transmission)
  if not desiredMode or type(desiredMode) ~= "string" or desiredMode == "" then
    return
  end
  if currentMode == desiredMode and movementState ~= movementStates.moving then
    targetModeQueue:reset()
    return
  end

  if desiredMode == lastHandledDesiredMode then
    return
  end

  local adjustedDesiredMode, secondDesiredMode = handleMSModes(desiredMode)

  --push the actual target to the queue
  targetModeQueue:reset()
  targetModeQueue:push_left(adjustedDesiredMode)
  if secondDesiredMode then
    targetModeQueue:push_right(secondDesiredMode)
  end
  lastHandledDesiredMode = desiredMode

  if not currentMode or not adjustedDesiredMode or not modeCoordinates[currentMode] or not modeCoordinates[adjustedDesiredMode] then
    return
  end

  --calculate how far away the next target mode is from the current one
  local modeDistance = modeCoordinates[currentMode].position - modeCoordinates[adjustedDesiredMode].position
  --if our next target is max 1 mode away from current, set that as the next target
  local distanceSign = sign(modeDistance)
  local nextModeId = modeCoordinates[adjustedDesiredMode].position
  while abs(modeDistance) > 1 do
    nextModeId = nextModeId + distanceSign
    --we can't reach the desired mode immediately, instead get the next mode in order and approach the actual target that way
    local modeStep = modeCoordinatesIndexLookup[nextModeId]
    targetModeQueue:push_left(modeStep)
    modeDistance = modeCoordinates[modeStep].position - modeCoordinates[currentMode].position
  end
end

local function updateTargetPosition(dt)
  local currentModeTarget = targetModeQueue:peek_left()
  if not currentModeTarget or movementState == movementStates.moving then
    return
  end

  if lastCurrentMode then
    --generate lookup key for intermediate coordinates
    local intermediateCoordinateKey = lastCurrentMode .. currentModeTarget
    --if intermediate coordinates exist for this move
    if modeChanges[intermediateCoordinateKey] then
      if not intermediateCoordinateIndex or modeChanges[intermediateCoordinateKey][intermediateCoordinateIndex] then
        intermediateCoordinateIndex = intermediateCoordinateIndex or 1
        --fetch them
        targetCoordinates = modeChanges[intermediateCoordinateKey][intermediateCoordinateIndex]
        --and increase the intermediate coordinate index for the next move
        intermediateCoordinateIndex = intermediateCoordinateIndex + 1
        gearModeState = gearModeStates.intermediatePosition
      else
        --if not, fetch the final target coordinates from the mode
        targetCoordinates = modeCoordinates[currentModeTarget]
        --and disable the intermediate animation
        intermediateCoordinateIndex = nil
        gearModeState = gearModeStates.actualPosition
      end
    else
      targetCoordinates = modeCoordinates[currentModeTarget]
      gearModeState = gearModeStates.actualPosition
    end
  else
    targetCoordinates = modeCoordinates[currentModeTarget]
    gearModeState = gearModeStates.actualPosition
  end
  movementState = movementStates.moving
end

local function updateActualPosition(dt)
  if targetCoordinates and movementState == movementStates.moving then
    local hasReachedTargetPosition = true
    if targetCoordinates.x ~= xCurrent then
      xCurrent = xSmoother:get(targetCoordinates.x, dt)

      hasReachedTargetPosition = false
    end
    if targetCoordinates.y ~= yCurrent then
      yCurrent = ySmoother:get(targetCoordinates.y, dt)
      hasReachedTargetPosition = false
    end

    movementState = hasReachedTargetPosition and movementStates.reachedPosition or movementStates.moving
  end

  electrics.values[electricsNameXAxis] = xCurrent
  electrics.values[electricsNameYAxis] = yCurrent
end

local function updateCurrentMode(dt)
  if movementState == movementStates.reachedPosition then
    movementState = movementStates.none
    if gearModeState == gearModeStates.actualPosition then
      currentMode = targetModeQueue:pop_left()
      lastCurrentMode = currentMode
      gearModeState = gearModeStates.none
      obj:playSFXOnceCT(moveSoundEvent, moveSoundNodeId, moveSoundVolume, 1, 0, 0)
    end
  end
  if movementState == movementStates.moving then
    currentMode = nil
  end
end

local function updateGFX(dt)
  updateTargetQueue(dt)
  updateTargetPosition(dt)
  updateActualPosition(dt)
  updateCurrentMode(dt)
end

local function reset(jbeamData)
  xCurrent = 0
  yCurrent = 0
  electrics.values[electricsNameXAxis] = xCurrent
  electrics.values[electricsNameYAxis] = yCurrent
  xSmoother:reset()
  ySmoother:reset()
  targetModeQueue:reset()

  demoGearIndex = 1
  demoGearIndexOffset = 1
  demoGearTimer = 0
  targetCoordinates = nil
  currentMode = nil
  lastCurrentMode = nil

  movementState = movementStates.none
  gearModeState = gearModeStates.none
  intermediateCoordinateIndex = nil
  lastHandledDesiredMode = nil
end

local function init(jbeamData)
  targetModeQueue = dequeue.new()
  local modeCoordinateTable = tableFromHeaderTable(jbeamData.orderedModeCoordinates or {})
  modeCoordinates = {}
  modeCoordinatesIndexLookup = {}

  currentMode = nil
  lastCurrentMode = nil

  local modeId = 1
  for _, coordinate in pairs(modeCoordinateTable) do
    modeCoordinates[coordinate.modeName] = {x = coordinate.x, y = coordinate.y, position = modeId}
    --maxExistingGearCoordinateIndex = max(maxExistingGearCoordinateIndex, coordinate.gearIndex)
    --minExistingGearCoordinateIndex = min(minExistingGearCoordinateIndex, coordinate.gearIndex)
    modeId = modeId + 1
    table.insert(modeCoordinatesIndexLookup, coordinate.modeName)
  end

  --"impulse" mode like "M+" and "M-" need special treatment so that both their positions are just 1 away from the base
  local impulseModeCoordinateTable = tableFromHeaderTable(jbeamData.impulseModeCoordinates or {})
  for _, impulseMode in pairs(impulseModeCoordinateTable) do
    --we only support modes here with a length of at least 2
    local modeNameLength = impulseMode.modeName:len()
    if modeNameLength < 2 then
      log("E", "propAnimation/dualAxisLever.init", "Impulse mode names need to be at least 2 characters long and typically end in + or -. Name in question: " .. impulseMode.modeName)
      return
    else
      --get the base this mode relates to
      local modeBaseName = impulseMode.modeName:sub(1, modeNameLength - 1)
      --we need to actually have the base mode for this to work
      local modeBaseData = modeCoordinates[modeBaseName]
      if not modeBaseData then
        log("E", "propAnimation/dualAxisLever.init", "Can't find base mode for impulse mode. Impulse mode name and expected base mode name in question: " .. impulseMode.modeName .. ", " .. modeBaseName)
        return
      else
        --find the position of our base mode
        local baseId = modeBaseData.position
        --add our impulse mode with a position distance of just 1 so that both impulse modes are reachable directly from the base mode
        modeCoordinates[impulseMode.modeName] = {x = impulseMode.x, y = impulseMode.y, position = baseId + 1}
      end
    end
  end

  local modeChangesTable = tableFromHeaderTable(jbeamData.modeChanges or {})
  modeChanges = {}

  for _, modeChange in pairs(modeChangesTable) do
    local index1 = modeChange.mode1 .. modeChange.mode2
    local index2 = modeChange.mode2 .. modeChange.mode1
    modeChanges[index1] = {}
    modeChanges[index2] = {}
    for _, pos in pairs(modeChange.intermediateCoordinates) do
      --normal order as per jbeam
      table.insert(modeChanges[index1], pos)
      --reverse order for backwards traveling
      table.insert(modeChanges[index2], 1, pos)
    end
  end

  intermediateCoordinateIndex = nil

  --move right to the first mode
  targetModeQueue:push_left(modeCoordinatesIndexLookup[1])

  electricsNameXAxis = jbeamData.electricsNameXAxis or "dualAxisX"
  electricsNameYAxis = jbeamData.electricsNameYAxis or "dualAxisY"
  electricsNameMode = jbeamData.electricsNameMode or "gear"

  if type(jbeamData.moveSoundNode_nodes) == "table" and jbeamData.moveSoundNode_nodes[1] and type(jbeamData.moveSoundNode_nodes[1]) == "number" then
    moveSoundNodeId = jbeamData.moveSoundNode_nodes[1]
  else
    log("W", "propAnimation/dualAxisLever.init", "Can't find node id for sound location. Specified data: " .. dumps(jbeamData.moveSoundNode_nodes))
    moveSoundNodeId = 0
  end

  moveSoundEvent = jbeamData.moveSoundEventDualAxisDirectionChange or "event:>Vehicle>Interior>Gearshift>manual_modern_01_in"
  moveSoundVolume = jbeamData.moveSoundVolumeDualAxisDirectionChange or 0.5

  local xSmootherRate = jbeamData.smootherRateX or 4
  local ySmootherRate = jbeamData.smootherRateY or 4
  xSmoother = newTemporalSmoothing(xSmootherRate, ySmootherRate)
  ySmoother = newTemporalSmoothing(xSmootherRate, ySmootherRate)

  bdebug.setNodeDebugText("PropAnimation", moveSoundNodeId, "Dual Axis Lever: " .. moveSoundEvent)

  xCurrent = 0
  yCurrent = 0
  electrics.values[electricsNameXAxis] = xCurrent
  electrics.values[electricsNameYAxis] = yCurrent

  getDesiredModeFunction = getDesiredMode

  demoModes = jbeamData.demoModes or false
  demoGearIndex = 1
  demoGearIndexOffset = 1
  if demoModes then
    getDesiredModeFunction = getDesiredModeDemo
  end

  M.updateGFX = updateGFX
end

M.init = init
M.reset = reset
M.updateGFX = nop

return M
