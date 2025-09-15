-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local min = math.min
local abs = math.abs
local clamp = clamp

local psiToPascal = 6894.757293178

local beamGroups = nil

local function setBeamPressureCore(cid, pressurePSI, maxPressure, spring, damp)
  obj:setBeamPressureRel(cid, pressurePSI * psiToPascal, maxPressure * psiToPascal, spring, damp)
end

local function adjustBeamGroupSounds(groupData, pressureDiff)
  obj:stopSFX(groupData.soundLoopUp)
  obj:stopSFX(groupData.soundLoopDown)
  groupData.isPlayingUp = false
  groupData.isPlayingDown = false

  if abs(pressureDiff) > 0.1 then
    if pressureDiff > 0 then
      obj:setVolume(groupData.soundLoopUp, groupData.volumeUp)
      obj:cutSFX(groupData.soundLoopUp)
      obj:playSFX(groupData.soundLoopUp)
      groupData.isPlayingUp = true
    elseif pressureDiff < 0 then
      obj:setVolume(groupData.soundLoopDown, groupData.volumeDown)
      obj:cutSFX(groupData.soundLoopDown)
      obj:playSFX(groupData.soundLoopDown)
      groupData.isPlayingDown = true
    end
  end
end

local function setBeamGroupPressureRaw(groupName, pressure)
  local groupData = beamGroups[groupName]
  if not groupData then
    log("W", "pneumatics.setBeamGroupPressureRaw", "Can't find pressure beam group: " .. groupName)
  else
    local maxPressureDiff = 0
    for _, v in pairs(groupData.beams) do
      local oldTarget = v.targetPressure
      v.targetPressure = clamp(pressure, v.minPressure, v.maxPressure)
      local pressureDiff = oldTarget - v.targetPressure
      if abs(pressureDiff) > abs(maxPressureDiff) then
        maxPressureDiff = pressureDiff
      end
    end
    adjustBeamGroupSounds(groupData, maxPressureDiff)
  end
end

local function setBeamGroupsPressureRaw(groupNames, pressure)
  for _, g in pairs(groupNames) do
    local groupData = beamGroups[g]
    if not groupData then
      log("W", "pneumatics.setBeamPressure", "Can't find pressure beam group: " .. g)
    else
      setBeamGroupPressureRaw(g, pressure)
    end
  end
end

local function setBeamGroupPressureLevel(groupName, pressureName)
  local groupData = beamGroups[groupName]
  if not groupData then
    log("W", "pneumatics.setBeamPressureGroup", "Can't find pressure beam group: " .. groupName)
  else
    local maxPressureDiff = 0
    for _, v in pairs(groupData.beams) do
      local oldTarget = v.targetPressure
      v.targetPressure = clamp(v[pressureName] or 0, v.minPressure, v.maxPressure)
      local pressureDiff = oldTarget - v.targetPressure
      if abs(pressureDiff) > abs(maxPressureDiff) then
        maxPressureDiff = pressureDiff
      end
    end
    adjustBeamGroupSounds(groupData, maxPressureDiff)
  end
end

local function setBeamGroupsPressureLevel(groupNames, pressureName)
  for _, g in pairs(groupNames) do
    setBeamGroupPressureLevel(g, pressureName)
  end
end

local function setBeamGroupsMinPressure(groupNames)
  setBeamGroupsPressureLevel(groupNames, "minPressure")
end

local function setBeamGroupsMaxPressure(groupNames)
  setBeamGroupsPressureLevel(groupNames, "maxPressure")
end

local function setBeamGroupsDefaultPressure(groupNames)
  setBeamGroupsPressureLevel(groupNames, "defaultPressure")
end

local function toggleBeamGroupsMinMax(groupNames)
  for _, g in pairs(groupNames) do
    local groupData = beamGroups[g]
    if not groupData then
      log("W", "pneumatics.toggleBeamMinMax", "Can't find pressure beam group: " .. g)
    else
      local isMax = true
      for _, v in pairs(groupData.beams) do
        isMax = isMax and abs(v.targetPressure - v.maxPressure) < 0.1
      end
      if isMax then
        setBeamGroupPressureLevel(g, "minPressure")
      else
        setBeamGroupPressureLevel(g, "maxPressure")
      end
    end
  end
end

local function setBeamGroupsMomentaryIncrease(groupNames, enabled)
  for _, g in pairs(groupNames) do
    local groupData = beamGroups[g]
    if not groupData then
      log("W", "pneumatics.setBeamGroupsMomentaryIncrease", "Can't find pressure beam group: " .. g)
    else
      groupData.momentaryIncrease = enabled
      groupData.momentaryDecrease = false

      obj:stopSFX(groupData.soundLoopDown)
      if enabled and not groupData.isPlayingUp then
        obj:setVolume(groupData.soundLoopUp, groupData.volumeUp)
        obj:cutSFX(groupData.soundLoopUp)
        obj:playSFX(groupData.soundLoopUp)
        groupData.isPlayingUp = true
      end
    end
  end
end

local function setBeamGroupsMomentaryDecrease(groupNames, enabled)
  for _, g in pairs(groupNames) do
    local groupData = beamGroups[g]
    if not groupData then
      log("W", "pneumatics.setBeamGroupsMomentaryDecrease", "Can't find pressure beam group: " .. g)
    else
      groupData.momentaryIncrease = false
      groupData.momentaryDecrease = enabled

      obj:stopSFX(groupData.soundLoopUp)
      if enabled and not groupData.isPlayingDown then
        obj:setVolume(groupData.soundLoopDown, groupData.volumeDown)
        obj:cutSFX(groupData.soundLoopDown)
        obj:playSFX(groupData.soundLoopDown)
        groupData.isPlayingDown = true
      end
    end
  end
end

local function isBeamGroupAtPressureLevel(groupName, levelName)
  local groupData = beamGroups[groupName]
  if not groupData then
    --log("W", "pneumatics.isBeamGroupAtPressureLevel", "Can't find pressure beam group: "..groupName)
    return false
  else
    local isAtLevel = true
    for _, v in pairs(groupData.beams) do
      isAtLevel = isAtLevel and v.currentPressure == v[levelName]
    end
    return isAtLevel
  end
end

local function updateGFX(dt)
  for _, g in pairs(beamGroups) do
    local isFinishedChangingPressure = true -- used to check if we should stop the sound based on pressure change
    local isFinishedChangingVelocity = false -- used to check if we should stop the sound based on the velocity of expansion/contraction of the pressured beams

    local avgVelocity = 0
    for _, v in pairs(g.beams) do
      if g.momentaryIncrease then
        v.targetPressure = clamp(v.targetPressure + v.increaseRate * dt, v.minPressure, v.maxPressure)
      elseif g.momentaryDecrease then
        v.targetPressure = clamp(v.targetPressure - v.decreaseRate * dt, v.minPressure, v.maxPressure)
      end

      avgVelocity = avgVelocity + abs(obj:getBeamVelocity(v.cid))

      if v.currentPressure ~= v.targetPressure then
        isFinishedChangingPressure = false
        local diff = v.targetPressure - v.currentPressure
        local rateLimitedDiff = min(abs(diff), (diff >= 0 and v.increaseRate or v.decreaseRate) * dt) * sign(diff)
        v.currentPressure = clamp(v.currentPressure + rateLimitedDiff, v.minPressure, v.maxPressure)
        setBeamPressureCore(v.cid, v.currentPressure, v.maxBeamPressure, v.spring, v.damp)
      end
    end
    avgVelocity = avgVelocity / g.beamCount

    --find the right threshold depending on the movement direction
    local velocityThreshold = g.isPlayingUp and g.stopVelocityThresholdPressureIncrease or g.stopVelocityThresholdPressureDecrease

    --if we did go past the threshold once and now are below it again, stop the sound
    if g.hasChangedVelocity and velocityThreshold and avgVelocity < velocityThreshold then
      isFinishedChangingVelocity = true
    end

    --monitor if we went above the speed threshold once
    if velocityThreshold and avgVelocity >= velocityThreshold and not g.hasChangedVelocity then
      g.hasChangedVelocity = true
    end

    local isFinishedChanging = isFinishedChangingPressure or isFinishedChangingVelocity

    if isFinishedChanging then
      g.hasChangedVelocity = false
      if g.isPlayingUp then
        obj:stopSFX(g.soundLoopUp)
        g.isPlayingUp = false
      end
      if g.isPlayingDown then
        obj:stopSFX(g.soundLoopDown)
        g.isPlayingDown = false
      end
    end
  end
end

local function reset()
  for _, g in pairs(beamGroups) do
    for _, v in pairs(g.beams) do
      v.targetPressure = v.defaultPressure
      v.currentPressure = v.defaultPressure
    end
    g.hasChangedVelocity = false
  end
end

local function init(jbeamData)
  local pressureLevels = {minPressure = 0, maxPressure = 0}
  if jbeamData.pressureLevels then
    for _, v in pairs(tableFromHeaderTable(jbeamData.pressureLevels)) do
      pressureLevels[v.name] = v.pressure
    end
  end

  local increaseRate = jbeamData.increaseRate or 0
  local decreaseRate = jbeamData.decreaseRate or 0

  local pressureBeamData = v.data[jbeamData.pressuredBeams] or {}

  local pressuredBeamNames = {}
  for _, v in pairs(pressureBeamData) do
    local name = v.beamName
    pressuredBeamNames[name] = true
  end

  local relevantBeams = {}
  local beams = v.data.beams
  for i = 0, tableSizeC(beams) - 1 do
    local v = beams[i]
    if v.name and pressuredBeamNames[v.name] then
      relevantBeams[v.name] = {cid = v.cid, defaultPressure = v.pressurePSI, maxPressure = v.maxPressure, spring = v.beamSpring, damp = v.beamDamp}
    end
  end

  beamGroups = {}
  for _, pressureData in pairs(pressureBeamData) do
    local name = pressureData.beamName
    local groupName = pressureData.groupName
    local beamPressureLevels = shallowcopy(pressureLevels)

    for k, v2 in pairs(pressureData) do
      local pressureStart = k:find("Pressure")
      if pressureStart then
        beamPressureLevels[k] = v2
      end
    end

    local beamIncreaseRate = pressureData.increaseRate or increaseRate
    local beamDecreaseRate = pressureData.decreaseRate or decreaseRate
    if not beamGroups[groupName] then
      beamGroups[groupName] = {beams = {}, beamCount = 0, hasChangedVelocity = false}
    end
    if not relevantBeams[name] then
      log("W", "pneumatics.init", "Can't find beam with name: " .. name)
    else
      local beamData = {
        name = name,
        cid = relevantBeams[name].cid,
        maxBeamPressure = relevantBeams[name].maxPressure,
        spring = relevantBeams[name].spring,
        damp = relevantBeams[name].damp,
        defaultPressure = relevantBeams[name].defaultPressure,
        currentPressure = relevantBeams[name].defaultPressure,
        targetPressure = relevantBeams[name].defaultPressure,
        increaseRate = beamIncreaseRate,
        decreaseRate = beamDecreaseRate
      }

      for k, v in pairs(beamPressureLevels) do
        beamData[k] = v
      end

      table.insert(beamGroups[groupName].beams, beamData)
      beamGroups[groupName].beamCount = beamGroups[groupName].beamCount + 1
    end
  end
end

local function initSounds(jbeamData)
  local pressureBeamSoundData = v.data[jbeamData.groupSounds] or {}
  --dump(pressureBeamSoundData)

  for _, v in pairs(pressureBeamSoundData) do
    local groupData = beamGroups[v.groupName]
    if groupData then
      groupData.volumeUp = v.volumeUp or 1
      groupData.volumeDown = v.volumeDown or 1
      groupData.soundNode = v.node or 0
      groupData.stopVelocityThresholdPressureIncrease = v.stopVelocityThresholdPressureIncrease --no default since this is just one of two deactivation schemes
      groupData.stopVelocityThresholdPressureDecrease = v.stopVelocityThresholdPressureDecrease --no default since this is just one of two deactivation schemes
      groupData.hasChangedVelocity = false
      if v.soundDown then
        groupData.soundLoopDown = obj:createSFXSource(v.soundDown, "AudioDefaultLoop3D", "pneumatics_down_" .. v.groupName, groupData.soundNode)
      end
      if v.soundUp then
        groupData.soundLoopUp = obj:createSFXSource(v.soundUp, "AudioDefaultLoop3D", "pneumatics_up_" .. v.groupName, groupData.soundNode)
      end
    else
      log("W", "pneumatics.initSounds", "Can't find group with name: " .. v.groupName)
    end
  end

  --dump(beamGroups)
end

local function resetSounds()
  for _, g in pairs(beamGroups) do
    if g.soundLoopUp then
      obj:setVolume(g.soundLoopUp, 0)
      g.isPlayingUp = false
    end
    if g.soundLoopDown then
      obj:setVolume(g.soundLoopDown, 0)
      g.isPlayingDown = false
    end
  end
end

M.init = init
M.reset = reset
M.initSounds = initSounds
M.resetSounds = resetSounds
M.updateGFX = updateGFX

M.setBeamMin = setBeamGroupsMinPressure
M.setBeamMax = setBeamGroupsMaxPressure
M.setBeamDefault = setBeamGroupsDefaultPressure
M.setBeamPressure = setBeamGroupsPressureRaw
M.setBeamPressureLevel = setBeamGroupsPressureLevel
M.toggleBeamMinMax = toggleBeamGroupsMinMax
M.setBeamMomentaryIncrease = setBeamGroupsMomentaryIncrease
M.setBeamMomentaryDecrease = setBeamGroupsMomentaryDecrease

M.isBeamGroupAtPressureLevel = isBeamGroupAtPressureLevel

return M
