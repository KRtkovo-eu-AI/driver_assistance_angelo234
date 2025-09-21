-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"
M.defaultOrder = 80

local abs = math.abs
local max = math.max
local clamp = clamp

local defaultTargetLengthCalibrationTime = 1.5 -- seconds

local actuator = nil
local controlGroups = {}
local actuationStartDistance = 0
local actuationEndDistance = 0
local actuationDelay = 0
local minValveOpenAmount = 0

local function iterateGroups(groupNameOrNames)
  if type(groupNameOrNames) == "string" then
    groupNameOrNames = {groupNameOrNames}
  end

  local groups = {}

  for _, name in ipairs(groupNameOrNames) do
    local group = controlGroups[name]
    if group then
      table.insert(groups, group)
    else
      log("W", "autoLevelSuspension.iterateGroups", "Suspension control group not found: " .. name)
    end
  end

  return pairs(groups)
end

local function setTemporarilyDisabled(groupNames, disabled)
  for _, group in iterateGroups(groupNames) do
    group.temporarilyDisabled = disabled
  end
end

local function setAdjustmentRate(groupNames, rate)
  for _, group in iterateGroups(groupNames) do
    group.adjustmentRate = clamp(rate, -1, 1)
    group.commitNewHeight = false
  end
end

local function stopAdjusting(groupNames, commitNewHeight)
  for _, group in iterateGroups(groupNames) do
    group.adjustmentRate = 0
    group.commitNewHeight = commitNewHeight == true
  end
end

local function setMomentaryIncrease(groupNames, increase)
  log("W", "autoLevelSuspension.setMomentaryIncrease", "This function is deprecated. Use setAdjustmentRate with a rate of 1.0 instead.")
  if increase then
    setAdjustmentRate(groupNames, 1)
  else
    stopAdjusting(groupNames, true)
  end
end

local function setMomentaryDecrease(groupNames, decrease)
  log("W", "autoLevelSuspension.setMomentaryDecrease", "This function is deprecated. Use setAdjustmentRate with a rate of -1.0 instead.")
    if decrease then
      setAdjustmentRate(groupNames, -1)
    else
      stopAdjusting(groupNames, true)
    end
end

local function setTargetLength(groupNames, targetLength, immediate)
  for _, group in iterateGroups(groupNames) do
    group.targetLength = targetLength

    if immediate then
      group.timeOutsideTarget = actuationDelay
    end
  end
end

local function getTargetLength(groupName)
  local group = controlGroups[groupName]
  if not group then
    log("E", "autoLevelSuspension.getTargetLength", "Suspension control group not found: " .. groupName)
    return 0
  end
  return group.targetLength
end

local function getCurrentLength(groupName)
  local group = controlGroups[groupName]
  if not group then
    log("E", "autoLevelSuspension.getCurrentLength", "Suspension control group not found: " .. groupName)
    return 0
  end
  return group.currentLength
end

local function getAverageFlowRate(groupName)
  local group = controlGroups[groupName]
  if not group then
    log("E", "autoLevelSuspension.getAverageFlowRate", "Suspension control group not found: " .. groupName)
    return 0
  end
  return actuator.getBeamGroupsAverageFlowRate(group.beamGroups)
end

local function isMoving(groupName)
  local group = controlGroups[groupName]
  if not group then
    log("E", "autoLevelSuspension.isMoving", "Suspension control group not found: " .. groupName)
    return false
  end
  return abs(actuator.getBeamGroupsAverageFlowRate(group.beamGroups)) > 1e-4
end

local function isCalibrating(groupName)
  local group = controlGroups[groupName]
  if not group then
    log("E", "autoLevelSuspension.isCalibrating", "Suspension control group not found: " .. groupName)
    return false
  end
  return group.calibratingTime > 0
end

local function updateFixedStep(dt)
  for _, controlGroup in pairs(controlGroups) do
    local controlBeamId = controlGroup.controlBeamId
    local curLength = obj:getBeamLength(controlBeamId)

    controlGroup.currentLength = curLength

    if controlGroup.debug then
      streams.drawGraph(controlGroup.name .. "_currentLength", { value = controlGroup.currentLength, unit = "m" })
      streams.drawGraph(controlGroup.name .. "_targetLength", { value = controlGroup.targetLength, unit = "m" })
    end

    if controlGroup.calibratingTime > 0 then
      controlGroup.calibratingTime = controlGroup.calibratingTime - dt

      if controlGroup.calibratingTime <= 0 then
        controlGroup.targetLength = curLength
      end
    elseif controlGroup.adjustmentRate ~= 0 then
      actuator.setBeamGroupsValveState(controlGroup.beamGroups, controlGroup.adjustmentRate)
    else
      if controlGroup.commitNewHeight then
        controlGroup.targetLength = curLength
        controlGroup.commitNewHeight = false
      end

      if controlGroup.temporarilyDisabled then
        actuator.setBeamGroupsValveState(controlGroup.beamGroups, 0)
      else
        local targetLength = controlGroup.targetLength
        local lengthError = targetLength - curLength
        local actuationCoef = 0

        if abs(lengthError) > actuationStartDistance then
          actuationCoef = linearScale(abs(lengthError), actuationStartDistance, actuationEndDistance, minValveOpenAmount, 1) * sign(lengthError)
          controlGroup.timeOutsideTarget = controlGroup.timeOutsideTarget + dt
        else
          controlGroup.timeOutsideTarget = 0
        end

        if controlGroup.timeOutsideTarget >= actuationDelay then
          actuator.setBeamGroupsValveState(controlGroup.beamGroups, actuationCoef)
          if controlGroup.debug then
            streams.drawGraph(controlGroup.name .. "_control", actuationCoef)
          end
        else
          actuator.setBeamGroupsValveState(controlGroup.beamGroups, 0)
          if controlGroup.debug then
            streams.drawGraph(controlGroup.name .. "_control", 0)
          end
        end
      end
    end
  end
end

local function reset()
  for _, g in pairs(controlGroups) do
    g.targetLength = g.defaultTargetLength or 0

    if not g.defaultTargetLength then
      -- need to automatically determine default target length from initial beam length; give vehicle time to settle
      g.calibratingTime = defaultTargetLengthCalibrationTime
    end

    g.currentLength = obj:getBeamLength(g.controlBeamId)
    g.timeOutsideTarget = 0
    g.temporarilyDisabled = false
    g.adjustmentRate = 0
    g.commitNewHeight = 0
  end
end

local function init(jbeamData)
  local actuatorGroupsData = v.data[jbeamData.actuatorGroups] or {}
  local controlBeamNames = {}

  for _, v in pairs(actuatorGroupsData) do
    controlBeamNames[v.controlBeamName] = true
  end

  local relevantBeamIds = {}
  local beams = v.data.beams

  for _, v in pairs(beams) do
    if v.name and controlBeamNames[v.name] then
      relevantBeamIds[v.name] = v.cid
    end
  end

  controlGroups = {}
  for _, groupData in pairs(actuatorGroupsData) do
    local controlBeamName = groupData.controlBeamName
    local controlGroupName = groupData.controlGroupName or controlBeamName
    local controlBeamId = relevantBeamIds[controlBeamName]

    if not controlBeamId then
      log("W", "autoLevelSuspension.init", "Can't find beam with name: " .. controlBeamName)
    else
      local beamGroupName = groupData.beamGroup
      local defaultTargetLength = groupData.defaultTargetLength
      local initialCurrentLength = obj:getBeamLength(controlBeamId)

      if not controlGroups[controlGroupName] then
        controlGroups[controlGroupName] = {
          name = controlGroupName,
          controlBeamId = controlBeamId,
          beamGroups = {},
          didStartMoving = false,
          calibratingTime = 0,
          defaultTargetLength = defaultTargetLength,
          targetLength = defaultTargetLength or 0,
          currentLength = initialCurrentLength,
          timeOutsideTarget = 0,
          temporarilyDisabled = false,
          adjustmentRate = 0,
          commitNewHeight = false,
          debug = groupData.debug == true,
        }

        if not defaultTargetLength then
          -- need to automatically determine default target length from initial beam length; give vehicle time to settle
          controlGroups[controlGroupName].calibratingTime = defaultTargetLengthCalibrationTime
        end
      end

      table.insert(controlGroups[controlGroupName].beamGroups, beamGroupName)
    end
  end

  actuationStartDistance = jbeamData.actuationStartDistance or 0.01 -- 1 cm
  actuationEndDistance = jbeamData.actuationEndDistance or 0.06 -- 6 cm
  actuationDelay = jbeamData.actuationDelay or 0
  minValveOpenAmount = jbeamData.minValveOpenAmount or 0.15
end

local function initSecondStage(jbeamData)
  local actuatorName = jbeamData.actuatorName or "airbags"

  actuator = controller.getController(actuatorName)

  if not actuator then
    log("E", "suspension.init", "Actuator controller not found: " .. actuatorName)
    M.updateFixedStep = nop
    return
  end
end

M.setTemporarilyDisabled = setTemporarilyDisabled
M.setAdjustmentRate = setAdjustmentRate
M.stopAdjusting = stopAdjusting
M.setMomentaryIncrease = setMomentaryIncrease
M.setMomentaryDecrease = setMomentaryDecrease
M.setTargetLength = setTargetLength
M.getTargetLength = getTargetLength
M.getCurrentLength = getCurrentLength
M.getAverageFlowRate = getAverageFlowRate
M.isMoving = isMoving
M.isCalibrating = isCalibrating

M.init = init
M.initSecondStage = initSecondStage
M.reset = reset
M.updateFixedStep = updateFixedStep

return M
