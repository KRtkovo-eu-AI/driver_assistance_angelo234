-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local min = math.min
local abs = math.abs
local clamp = clamp
local sqrt = math.sqrt

local beamGroups
local beams

local basePumpSpeed
local maxPumpSpeed

local pumpSoundLoop
local isPumpSoundPlaying
local currentPumpFlow
local maxPumpFlow
local lastPumpFlow

local releaseSoundLoop
local lastFrameBeamLengths
local releaseSoundVolumeSmoother = newTemporalSmoothing(1, 1)

local function updateBeamPhysics(beamData)
  obj:setBeamLengthRefRatio(beamData.cid, beamData.currentPosition)

  local spring = beamData.bleedCoef <= 0 and beamData.originalSpring or 0
  local damp = spring <= 0 and (beamData.minimumDamp + (beamData.originalDamp - beamData.minimumDamp) * clamp(1 - (beamData.bleedCoef or 1), 0, 1)) or 0

  obj:setBeamSpringDamp(beamData.cid, spring, damp, -1, -1)
end

local function setGroupsPosition(groupNames, position, speedCoef)
  for _, g in pairs(groupNames) do
    local groupData = beamGroups[g]
    if not groupData then
      log("W", "hydraulicSuspension.setGroupsPosition", "Can't find pressure beam group: " .. g)
    else
      for _, id in pairs(groupData.beamIds) do
        local beam = beams[id]
        local minExtension = groupData.minExtensions[beam.name]
        local maxExtension = groupData.maxExtensions[beam.name]
        beam.targetPosition = clamp(minExtension + (maxExtension - minExtension) * position, minExtension, maxExtension)
        beam.speedCoef = speedCoef or 1
        beam.bleedCoef = 0
        updateBeamPhysics(beam)
      end
    end
  end
end

local function setGroupsBleed(groupNames, bleedCoef)
  for _, g in pairs(groupNames) do
    local groupData = beamGroups[g]
    if not groupData then
      log("W", "hydraulicSuspension.setGroupsBleed", "Can't find pressure beam group: " .. g)
    else
      for _, id in pairs(groupData.beamIds) do
        local beam = beams[id]
        beam.speedCoef = 1
        beam.bleedCoef = clamp(bleedCoef, 0, 1)
        updateBeamPhysics(beam)
      end
    end
  end
end

local function setGroupsMomentaryIncrease(groupNames, enabled, speedCoef)
  for _, g in pairs(groupNames) do
    local groupData = beamGroups[g]
    if not groupData then
      log("W", "hydraulicSuspension.setGroupsMomentaryIncrease", "Can't find pressure beam group: " .. g)
    else
      for _, id in pairs(groupData.beamIds) do
        local beam = beams[id]
        beam.momentaryIncrease = enabled
        beam.momentaryIncreaseMinExtension = groupData.minExtensions[beam.name]
        beam.momentaryIncreaseMaxExtension = groupData.maxExtensions[beam.name]
        beam.speedCoef = speedCoef or 1
        beam.bleedCoef = 0
      end
    end
  end
end

local function updateGFX(dt)
  if currentPumpFlow > 0 then
    if currentPumpFlow ~= lastPumpFlow then
      local volume = 0.5
      local pitch = clamp(currentPumpFlow / maxPumpFlow, 0, 1)
      lastPumpFlow = currentPumpFlow
      obj:setVolumePitch(pumpSoundLoop, volume, pitch)
    end
    if not isPumpSoundPlaying then
      obj:cutSFX(pumpSoundLoop)
      obj:playSFX(pumpSoundLoop)
      isPumpSoundPlaying = true
    end
  elseif isPumpSoundPlaying then
    obj:stopSFX(pumpSoundLoop)
    isPumpSoundPlaying = false
    lastPumpFlow = 0
  end

  local avgBleedCoef = 0
  for _, beam in ipairs(beams) do
    if beam.bleedCoef > 0 then
      local currentLength = math.floor(obj:getBeamCurLengthRefRatio(beam.cid) * 100) / 100
      if currentLength < lastFrameBeamLengths[beam.cid] then
        avgBleedCoef = avgBleedCoef + beam.bleedCoef
      end

      lastFrameBeamLengths[beam.cid] = currentLength
    end
  end

  avgBleedCoef = avgBleedCoef / #beams

  local volume = releaseSoundVolumeSmoother:getUncapped(sqrt(avgBleedCoef), dt)
  obj:setVolumePitch(releaseSoundLoop, volume, 1)
end

local function updateFixedStep(dt)
  currentPumpFlow = 0
  for _, beam in ipairs(beams) do
    if beam.momentaryIncrease then
      beam.targetPosition = clamp(beam.targetPosition + basePumpSpeed * dt, beam.momentaryIncreaseMinExtension, beam.momentaryIncreaseMaxExtension)
    elseif beam.bleedCoef > 0 then
      beam.currentPosition = obj:getBeamCurLengthRefRatio(beam.cid)
      beam.targetPosition = beam.currentPosition
    end

    if beam.currentPosition ~= beam.targetPosition then
      local diff = beam.targetPosition - beam.currentPosition
      local changeSpeed = diff >= 0 and (min(basePumpSpeed * beam.speedCoef, maxPumpSpeed)) or 10
      currentPumpFlow = currentPumpFlow + changeSpeed
      local rateLimitedDiff = min(abs(diff), changeSpeed * dt) * sign(diff)
      beam.currentPosition = beam.currentPosition + rateLimitedDiff
      updateBeamPhysics(beam)
    end
  end
end

local function reset()
  lastFrameBeamLengths = {}
  for _, beam in ipairs(beams) do
    beam.currentPosition = beam.originalBeamPosition
    beam.targetPosition = beam.originalBeamPosition
    beam.bleedCoef = 1
    beam.speedCoef = 1
    beam.momentaryIncrease = false
    lastFrameBeamLengths[beam.cid] = beam.currentPosition
    updateBeamPhysics(beam)
  end
end

local function init(jbeamData)
  local hydraulicsData = v.data[jbeamData.hydraulicBeams or "hydraulicsData"] or {}

  local hydraulicBeamNames = {}
  for _, v in pairs(hydraulicsData) do
    local name = v.beamName
    hydraulicBeamNames[name] = true
  end

  local relevantBeams = {}
  local jbeamBeams = v.data.beams
  for i = 0, tableSizeC(jbeamBeams) - 1 do
    local v = jbeamBeams[i]
    if v.name and hydraulicBeamNames[v.name] then
      relevantBeams[v.name] = {cid = v.cid, spring = v.beamSpring, damp = v.beamDamp, minDamp = v.hydraulicsMinDamp or 0, defaultBleed = clamp(v.defaultBleedCoef or 1, 0, 1)}
    end
  end

  beamGroups = {}
  beams = {}
  lastFrameBeamLengths = {}
  local beamLookup = {}
  for _, hydraulicData in pairs(hydraulicsData) do
    local name = hydraulicData.beamName
    local groupName = hydraulicData.groupName
    local minExtension = hydraulicData.min
    local maxExtension = hydraulicData.max

    if not beamGroups[groupName] then
      beamGroups[groupName] = {beamIds = {}, minExtensions = {}, maxExtensions = {}}
    end
    if not relevantBeams[name] then
      log("W", "hydraulicSuspension.init", "Can't find beam with name: " .. name)
    else
      if not beamLookup[name] then
        local relevantBeam = relevantBeams[name]
        local beamData = {
          name = name,
          cid = relevantBeam.cid,
          originalSpring = relevantBeam.spring,
          originalDamp = relevantBeam.damp,
          minimumDamp = relevantBeam.minDamp,
          currentPosition = obj:getBeamLengthRefRatio(relevantBeam.cid),
          speedCoef = 1,
          momentaryIncrease = false,
          momentaryIncreaseMinExtension = 0,
          momentaryIncreaseMaxExtension = 0,
          bleedCoef = relevantBeam.defaultBleed
        }
        beamData.originalBeamPosition = beamData.currentPosition
        lastFrameBeamLengths[beamData.cid] = beamData.currentPosition

        updateBeamPhysics(beamData)
        table.insert(beams, beamData)
        beamLookup[name] = #beams
      end
      table.insert(beamGroups[groupName].beamIds, beamLookup[name])
      beamGroups[groupName].minExtensions[name] = minExtension
      beamGroups[groupName].maxExtensions[name] = maxExtension
    end
  end

  basePumpSpeed = jbeamData.basePumpSpeed or 0.5
  maxPumpSpeed = jbeamData.maxPumpSpeed or basePumpSpeed * 20
end

local function initSounds(jbeamData)
  local pumpSoundEvent = jbeamData.pumpSample or "event:>Vehicle>Hydraulics>Pump_01"
  pumpSoundLoop = obj:createSFXSource2(pumpSoundEvent, "AudioDefaultLoop3D", "", jbeamData.pumpNode, 0)
  isPumpSoundPlaying = false
  maxPumpFlow = maxPumpSpeed * #beams
  lastPumpFlow = 0
  currentPumpFlow = 0

  local releaseSoundEvent = jbeamData.releaseSample or "event:>Vehicle>Hydraulics>Release_01"
  releaseSoundLoop = obj:createSFXSource(releaseSoundEvent, "AudioDefaultLoop3D", "", jbeamData.pumpNode)
  obj:setVolumePitch(releaseSoundLoop, 0, 0)

  bdebug.setNodeDebugText("Hydraulics", jbeamData.pumpNode, M.name .. " - Pump: " .. (pumpSoundEvent or "no event"))
  bdebug.setNodeDebugText("Hydraulics", jbeamData.pumpNode, M.name .. " - Release: " .. (releaseSoundEvent or "no event"))
end

local function resetSounds()
  lastPumpFlow = 0
  currentPumpFlow = 0
  isPumpSoundPlaying = false
  obj:setVolumePitch(releaseSoundLoop, 0, 0)
  releaseSoundVolumeSmoother:reset()
end

M.init = init
M.reset = reset
M.initSounds = initSounds
M.resetSounds = resetSounds
M.updateGFX = updateGFX
M.updateFixedStep = updateFixedStep

M.setGroupsPosition = setGroupsPosition
M.setGroupsBleed = setGroupsBleed
M.setGroupsMomentaryIncrease = setGroupsMomentaryIncrease

return M
