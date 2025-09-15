-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"
M.defaultOrder = 80

local sign = sign
local abs = math.abs

local dampers = {}
local damperZones = {}

local soundSettings = {
  eventCompression = "",
  eventRebound = "",
  maxVolume = 1,
  minVelocity = 0,
  maxValocity = 0
}

local function setDamperZone(damper, zoneId)
  --print(string.format("Damper %d set to zone %f", damper.beamCid, zoneCid))
  local zone = damperZones[zoneId]
  if not zone then
    log("E", "bypassDamper.setDamperZone", "Can't find zone with Id: " .. tostring(zoneId))
    return
  end
  obj:setBoundedBeamDamp(damper.beamCid, zone.beamDamp, zone.beamDampRebound, zone.beamDampFast, zone.beamDampReboundFast, zone.beamDampVelocitySplit, zone.beamDampVelocitySplitRebound)
end

local function update(dt)
  for _, damper in ipairs(dampers) do
    local beamLength = obj:getBeamLength(damper.beamCid)
    local displacement = -beamLength + damper.initialBeamLength
    local velocity = (damper.previousDisplacement - displacement) / dt
    damper.previousDisplacement = displacement
    local direction = sign(velocity)
    local smoothDirection = damper.directionSmoother:get(direction, dt)
    if abs(smoothDirection) >= 1 and smoothDirection ~= damper.previousDirection then
      local volume = linearScale(abs(velocity), soundSettings.minVelocity, soundSettings.maxValocity, 0, 1) -- input value, min input value, max input value, output value at min input, output value at max input
      -- if M.name == "frontBypass" and _ == 1 then
      --   print(string.format("CLICK! Switch from %d to %d at velocity %.3fm/s, volume: %.2f", damper.previousDirection, smoothDirection, velocity, volume))
      -- end
      damper.previousDirection = smoothDirection
      local event = smoothDirection > 0 and soundSettings.eventCompression or soundSettings.eventRebound
      --make the clickety-clack
      if soundSettings.emitValveSounds then
        sounds.playSoundOnceFollowNode(event, damper.soundNodeCid or 0, volume, 1, soundSettings.maxVolume, 1)
      end
    end

    local currentZoneId = damper.currentZoneId
    local updatedZoneId = nil
    if currentZoneId then
      local currentZone = damperZones[currentZoneId]
      --check if we are still within the current zone
      if not (displacement >= currentZone.zoneStart and displacement < currentZone.zoneEnd) then
        local previousZone = damperZones[currentZoneId - 1]
        --check if we are within the previous zone
        if previousZone and displacement >= previousZone.zoneStart and displacement < previousZone.zoneEnd then
          updatedZoneId = currentZoneId - 1
        else
          local nextZone = damperZones[currentZoneId + 1]
          --check if we are within the next zone
          if nextZone and displacement >= nextZone.zoneStart and displacement < nextZone.zoneEnd then
            updatedZoneId = currentZoneId + 1
          elseif not previousZone and displacement < currentZone.zoneStart then
            --check if we are longer than the design spec
            updatedZoneId = currentZoneId
          elseif not nextZone and displacement >= currentZone.zoneEnd then
            --check if we are shorter than the design spec
            updatedZoneId = currentZoneId
          end
        end
      else
        --no zone change happened
        updatedZoneId = currentZoneId
      end
    end

    --if we can't find the updated zone within our neighbours, iterate over all of them to find the right one
    if not updatedZoneId then
      updatedZoneId = 1 --default in case the damper is outside its design specs (shorter or longer than expected)
      --find the updated zone the old fashioned way by iterating over all of them until we find the right one
      for zoneId, zoneData in ipairs(damperZones) do
        if displacement >= zoneData.zoneStart and displacement < zoneData.zoneEnd then
          updatedZoneId = zoneId
          break
        end
      end
    end

    if updatedZoneId ~= damper.currentZoneId then
      setDamperZone(damper, updatedZoneId)
      damper.currentZoneId = updatedZoneId
    end
  end
end

-- local function debugDraw(focusPos)
--   for _, damper in ipairs(dampers) do
--     obj.debugDrawProxy:drawNodeText(v.data.beams[damper.beamCid].id1, color(255, 0, 0, 255), damperZones[damper.currentZoneId].zoneStart or "N/A", 0)
--   end
-- end

local function reset()
end

local function initSounds(jbeamData)
  soundSettings = {
    emitValveSounds = jbeamData.emitValveSounds == nil and true or jbeamData.emitValveSounds,
    eventCompression = jbeamData.valveSoundEventCompression or "event:>Vehicle>Suspension>bypassShock_01>bypassShock_in",
    eventRebound = jbeamData.valveSoundEventRebound or "event:>Vehicle>Suspension>bypassShock_01>bypassShock_out",
    maxVolume = jbeamData.valveSoundMaxVolume or 0.5,
    minVelocity = jbeamData.valveSoundMinVelocity or 0.05,
    maxValocity = jbeamData.valveSoundMaxVelocity or 0.2
  }
end

local function init(jbeamData)
  dampers = {}
  local beamNameLookup = {}
  for _, b in pairs(v.data.beams) do
    if b.name then
      beamNameLookup[b.name] = b.cid
    end
  end

  local damperJbeamData = tableFromHeaderTable(jbeamData.dampers or {})
  for _, damperData in pairs(damperJbeamData) do
    local beamCid = beamNameLookup[damperData.beamName]
    local damper = {
      name = damperData.name,
      beamCid = beamCid,
      soundNodeCid = v.data.beams[beamCid].id1,
      initialBeamLength = obj:getBeamRestLength(beamNameLookup[damperData.beamName]),
      directionSmoother = newTemporalSmoothing(500, 500),
      currentZoneId = nil,
      previousZoneId = -1,
      previousDisplacement = 0,
      previousDirection = 0
    }
    damper.previousDisplacement = damper.initialBeamLength
    table.insert(dampers, damper)
  end

  local zoneJbeamData = tableFromHeaderTable(jbeamData.zones or {})

  local beamZones = {}
  for _, zone in pairs(zoneJbeamData) do
    local zoneData = {
      zoneStart = zone.zoneDistance,
      beamDamp = zone.beamDamp or 1,
      beamDampFast = zone.beamDampFast or 1,
      beamDampRebound = zone.beamDampRebound or 1,
      beamDampReboundFast = zone.beamDampReboundFast or 1,
      beamDampVelocitySplit = zone.beamDampVelocitySplit or 1,
      beamDampVelocitySplitRebound = zone.beamDampVelocitySplitRebound or zone.beamDampVelocitySplit
    }
    beamZones[zone.zoneDistance] = zoneData
  end

  local sortedZoneDistances = {}
  for zoneDistance, _ in pairs(beamZones) do
    table.insert(sortedZoneDistances, zoneDistance)
  end
  table.sort(sortedZoneDistances)

  local i = 1
  for _, distance in ipairs(sortedZoneDistances) do
    local zoneData = beamZones[distance]
    local nextZone = beamZones[sortedZoneDistances[i + 1]]
    zoneData.zoneEnd = nextZone and nextZone.zoneStart or math.huge
    table.insert(damperZones, zoneData)
    i = i + 1
  end

  --dump(dampers)
  --dump(beamZones)
  --dump(damperZones)
end

local function initLastStage()
end

M.init = init
M.initSounds = initSounds
M.reset = reset
M.initLastStage = initLastStage
M.update = update
--M.debugDraw = debugDraw

return M
