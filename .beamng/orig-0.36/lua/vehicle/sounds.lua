-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local max, min, abs, random, sqrt = math.max, math.min, math.abs, math.random, math.sqrt

-- SURFACE TYPES (taken from game\lua\common\particles.json)
--  2 : metal
--  3 : plastic
--  4 : rubber
--  5 : glass
--  6 : wood
--  7 : foliage
--  9 : water
-- 10 : asphalt
-- 11 : asphalt wet
-- 11 : asphalt slippery
-- 13 : rock
-- 14 : dirt dusty
-- 15 : dirt
-- 16 : sand
-- 17 : sandy road
-- 18 : mud
-- 19 : gravel
-- 20 : grass
-- 21 : ice
-- 22 : snow
-- 29 : rumble strip
-- 30 : cobble stone
-- 31 : foliage_thin

--impact sounds emitted when nodes of a certain material impact something
local impactGenericEvent = "event:>Destruction>Vehicle>vehicle_part_impact"
local impactMetalEvent = "event:>Destruction>Vehicle>vehicle_part_impact"
local impactPlasticEvent = "event:>Destruction>Vehicle>vehicle_part_impact_plastic"
local impactSoundVolumeCoef = 1
local breakGenericEvent = "event:>Destruction>Vehicle>vehicle_part_break"
local breakPlasticEvent = "event:>Destruction>Vehicle>vehicle_part_break_plastic"

local windSoundEvent = "event:>Vehicle>Aero>windbuffet"
local windSoundEventNode

M.scrapeLoosenessMap = {
  -- [2] = 0.0, -- METAL (0 is default)
  -- [6] = 0.0, -- WOOD (0 is default)
  [4] = 0.05, -- RUBBER
  [13] = 0.1, -- ROCK
  [19] = 0.2, -- GRAVEL
  [20] = 0.3, -- GRASS
  [7] = 0.4, -- FOLIAGE
  [31] = 0.5, -- FOLIAGE_THIN
  [14] = 0.6, -- DIRT_DUSTY
  [15] = 0.7, -- DIRT
  [18] = 0.8, -- MUD
  [16] = 0.9 -- SAND
}

local scrapeMap = {
  [2] = "event:>Destruction>Scrapes>asphalt_metal", -- metal
  [3] = "event:>Destruction>Scrapes>asphalt_plastic" -- plastic
}

local scrapeAbsorbing = {
  [4] = true, -- rubber
  [8] = true, -- cloth
  [16] = "wheel", -- SAND
  [18] = "wheel" -- MUD
}

local scrapeSounds = {}

M.engineNode = nil
M.refNode = nil
M.usesOldCustomSounds = false
M.objType = 0

local cabinFilterCoef = 1

local crashSoundTimer = 0
local sbeamVolumeFactor = 1.5

local windSound
local wheelsSounds

local soundBank = {sounds = {}}
local sfxprofilecounter = 0 -- local counter to enumerate the profiles without collisions, do not reset ever

local beamSounds = {}
local beamResetTimer = 0
local usingNewEngineSounds = false
local twoPi = math.pi * 2

-- local rattleSoundTimer = 0

local boolToNum = {[true] = 1, [false] = 0}

M.uiDebugging = false

local soundObj = {}
soundObj.__index = soundObj

local function createSoundscapeSound(name)
  local soundscapes = v.data.soundscape
  if not soundscapes or not soundscapes[name] then
    return
  end
  local soundscape = soundscapes[name]
  return obj:createSFXSource2(soundscape.src, soundscape.descriptor or "AudioDefaultLoop3D", "", type(soundscape.node) == "number" and soundscape.node or M.refNode, 0)
end

local function getSoundscapeNode(name)
  local soundscapes = v.data.soundscape
  if not soundscapes or not soundscapes[name] then
    return
  end
  local soundscape = soundscapes[name]
  return type(soundscape.node) == "number" and soundscape.node or nil
end

function soundObj:setVolumePitch(vol, pitch, color, texture)
  if vol < 0.01 then
    vol = 0
  end
  if vol == 0 then
    if self.lastVol == 0 then
      return false
    end
    color, texture = color or 0, texture or 0
  else
    color, texture = color or 0, texture or 0
    if abs(pitch - self.lastPitch) < 0.0025 and max(abs(vol - self.lastVol), abs(color - self.lastColor), abs(texture - self.lastTexture)) < 0.01 then
      return false
    end
  end
  self.lastVol = vol
  self.lastPitch = pitch
  self.lastColor = color
  self.lastTexture = texture
  obj:setVolumePitchCT(self.obj, vol, pitch, color, texture)
  return true
end

-- For low frequency use only
function soundObj:setParameter(keyName, value)
  obj:setSFXparameter(self.obj, keyName, value)
end

local function playSoundSkipAI(sound, volume)
  volume = volume or 1
  if (not playerInfo.anyPlayerSeated) and ai.isDriving() and volume > 0 then
    return
  end
  if sound then
    obj:setVolume(sound, volume)
    obj:cutSFX(sound)
    obj:playSFX(sound)
  end
end

local function playSoundOnceAtNode(soundName, nodeID, volume, pitch, color, texture)
  pitch = pitch or 1
  if volume >= 0.01 and pitch >= 0.0039 then
    obj:playSFXOnceStaticCT(soundName, nodeID, volume, pitch, color or 0, texture or 0)
  end
end

local function playSoundOnceFollowNode(soundName, nodeID, volume, pitch, color, texture)
  pitch = pitch or 1
  if volume >= 0.01 and pitch >= 0.0039 then
    obj:playSFXOnceCT(soundName, nodeID, volume, pitch, color or 0, texture or 0)
  end
end

local function getSourceValue(sourcename)
  --in the future possibly replace with the same system props uses for source
  return sourcename == "gear" and drivetrain.gear or electrics.values[sourcename]
end

local function getSoundModifier(modName)
  local modifier = soundBank.modifiersNamed[modName]
  if modifier == nil then
    return 1
  end

  local mVal = getSourceValue(modifier.source)
  if mVal == nil then
    return 1
  end

  return clamp(modifier.factor * (mVal + modifier.offset), modifier.min, modifier.max)
end

local function createSFXSource(filename, description, SFXProfileName, nodeID)
  local snd = obj:createSFXSource(filename, description, SFXProfileName, nodeID)
  if snd == nil then
    M.update = nop
    M.playSoundOnceAtNode = nop
    M.playSoundOnceFollowNode = nop
    log("W", "sounds.createSFXSource", "failed to create sfx source: " .. SFXProfileName .. " from file " .. filename .. " with description " .. description)
    return nil
  end
  return snd
end

local dummySoundObj = {setVolumePitch = nop}

local function createSoundObj(filename, description, SFXProfileName, nodeID)
  local sndObj = obj:createSFXSource(filename, description, SFXProfileName, nodeID)
  if sndObj == nil then
    return dummySoundObj
  end
  local data = {obj = sndObj, lastVol = 0, lastPitch = 0, lastColor = 0, lastTexture = 0}
  setmetatable(data, soundObj)
  return data
end

local function updateGFX(dt)
  --crash sounds
  crashSoundTimer = crashSoundTimer + dt
  if crashSoundTimer > 0.04 then
    crashSoundTimer = 0
    local impactEnergy, breakEnergy, breakNode, mat1, mat2 = obj:getImpactDeformEnergyNode()
    -- if not (scrapeAbsorbing[mat1] or scrapeAbsorbing[mat2]) then // removed because it was stopping some small impacts from happening as the system thinks it's a scrape
    -- local volImpact = impactEnergy / (impactEnergy + 6)
    impactEnergy = impactEnergy * impactSoundVolumeCoef
    local volImpact = impactEnergy / (impactEnergy + 4.5)
    -- local volBreak = breakEnergy / (breakEnergy + 1000)
    local volBreak = breakEnergy / (breakEnergy + 600)
    -- if volImpact > volBreak then
    if ((volImpact + 0.01) * 3) > volBreak then
      if volImpact > 0.004 then
        if (mat1 == 3 or mat2 == 3) then
          local nodeImpactPlasticEvent = v.data.nodes[breakNode].impactPlasticEvent or impactPlasticEvent
          if nodeImpactPlasticEvent then
            --print(string.format("%d: Impact Plastic (%.2f) -> %q", objectId, volImpact, nodeImpactPlasticEvent))
            -- print (string.format(" PLASTIC IMPACT / mat1=%.2d / mat2=%.2d / impactEnergy=%9.2f / breakEnergy=%9.2f / volImpact=%.3f  ", mat1, mat2, impactEnergy, breakEnergy, volImpact))
            sounds.playSoundOnceFollowNode(nodeImpactPlasticEvent, breakNode, volImpact)
          end
        elseif (mat1 == 2 or mat2 == 2) then
          local nodeImpactMetalEvent = v.data.nodes[breakNode].impactMetalEvent or impactMetalEvent
          if nodeImpactMetalEvent then
            --print(string.format("%d: Impact Metal (%.2f) -> %q", objectId, volImpact, nodeImpactMetalEvent))
            --print (string.format("    PART IMPACT / mat1=%.2d / mat2=%.2d / impactEnergy=%9.2f / breakEnergy=%9.2f /                 / volImpact=%.3f  ", mat1, mat2, impactEnergy, breakEnergy, volImpact))
            sounds.playSoundOnceFollowNode(nodeImpactMetalEvent, breakNode, volImpact)
          end
        else
          local nodeImpactGenericEvent = v.data.nodes[breakNode].impactGenericEvent or impactGenericEvent
          if nodeImpactGenericEvent then
            --print(string.format("%d: Impact Generic (%.2f) -> %q", objectId, volImpact, nodeImpactGenericEvent))
            -- print (string.format(" GENERIC IMPACT / mat1=%.2d / mat2=%.2d / impactEnergy=%9.2f / breakEnergy=%9.2f /                 / volImpact=%.3f  ", mat1, mat2, impactEnergy, breakEnergy, volImpact))
            sounds.playSoundOnceFollowNode(nodeImpactGenericEvent, breakNode, volImpact)
          end
        end
      end
    else
      if volBreak > 0.004 then
        if (mat1 == 3 and mat2 == 3) then
          local nodeBreakPlasticEvent = v.data.nodes[breakNode].breakPlasticEvent or breakPlasticEvent
          if nodeBreakPlasticEvent then
            --print(string.format("%d: Break Plastic (%.2f) -> %q", objectId, volBreak, nodeBreakPlasticEvent))
            sounds.playSoundOnceFollowNode(nodeBreakPlasticEvent, breakNode, volBreak)
          end
        else
          local nodeBreakGenericEvent = v.data.nodes[breakNode].breakGenericEvent or breakGenericEvent
          if nodeBreakGenericEvent then
            --print(string.format("%d: Break Generic (%.2f) -> %q", objectId, volBreak, nodeBreakGenericEvent))
            sounds.playSoundOnceFollowNode(nodeBreakGenericEvent, breakNode, volBreak)
          end
        end
      -- print (string.format("     PART BREAK / mat1=%.2d / mat2=%.2d / impactEnergy=%9.2f / breakEnergy=%9.2f /                 /                 / volBreak=%.3f ", mat1, mat2, impactEnergy, breakEnergy, volBreak))
      end
    end
  -- end
  end

  -- sound bank
  for _, snd in pairs(soundBank.sounds) do
    if snd.active then
      local val = getSourceValue(snd.source) or 0
      val = snd.factor * (val + snd.offset)
      snd.lastVal = val

      local sndVol = 0

      --check volume conditions
      if val < snd.volumeBlendInStartValue or val > snd.volumeBlendOutEndValue then
        sndVol = snd.minVolume
      end
      if val > snd.volumeBlendInStartValue and val < snd.volumeBlendInEndValue then
        --blend in volume
        sndVol = lerp(snd.minVolume, snd.maxVolume, (val - snd.volumeBlendInStartValue) / (snd.volumeBlendInEndValue - snd.volumeBlendInStartValue))
      elseif val > snd.volumeBlendInEndValue and val < snd.volumeBlendOutStartValue then
        sndVol = snd.maxVolume
      elseif val > snd.volumeBlendOutStartValue and val < snd.volumeBlendOutEndValue then
        --blend out volume
        sndVol = lerp(snd.minVolume, snd.maxVolume, (((val - snd.volumeBlendOutStartValue) / (snd.volumeBlendOutEndValue - snd.volumeBlendOutStartValue)) - 1) * -1)
      end

      --apply modifiers if applicable
      for _, s in pairs(snd.volumeModifiers) do
        sndVol = sndVol * getSoundModifier(s)
      end

      --blend pitch
      local sndPitch = lerp(snd.minPitch, snd.maxPitch, clamp((val - snd.pitchBlendInStartValue) / (snd.pitchBlendInEndValue - snd.pitchBlendInStartValue), 0, 1))

      for _, s in pairs(snd.pitchModifiers) do
        sndPitch = sndPitch * getSoundModifier(s)
      end

      snd.clip:setVolumePitch(sndVol, sndPitch)
    end
  end

  beamResetTimer = min(beamResetTimer + dt, 1)

  -- beam sounds, suspension
  local maxStressVolume = 0
  for bi, snd in ipairs(beamSounds) do
    local beamVel = obj:getBeamVelocity(snd.beam)

    -- noise filter
    snd.noiseTrail = snd.noiseTrail + beamVel * dt
    if abs(snd.noiseTrail) < snd.noiseFactor then
      beamVel = 0
    else
      snd.noiseTrail = signApply(snd.noiseTrail, snd.noiseFactor)
    end

    beamVel = min(1, abs(beamVel))
    local currentStress = clamp(obj:getBeamStress(snd.beam) / snd.maxStress, -1, 1) * beamVel * beamVel -- find the stress on the current sound beam (unsmoothed)
    -- local currentStress = clamp(obj:getBeamStress(snd.beam), -1, 1) * beamVel * beamVel -- find the stress on the current sound beam (unsmoothed)
    local smoothStress = snd.smoothing:get(currentStress, dt)
    local impulse = min(abs(smoothStress - currentStress), abs(smoothStress)) -- beam stress difference between instananeous stress and smooth stress
    -- local impulse = abs(currentStress)
    local volume = snd.volumeFactor * impulse -- normalize volume (cancel out maxStress factor)
    local pitch = (snd.pitchFactor * snd.volumeFactor) * impulse -- loud suspension sounds also gain a higher pitch

    snd.clip:setVolumePitch(volume * beamResetTimer, pitch, snd.colorFactor)
    maxStressVolume = max(maxStressVolume, volume)

    -- store the outputs so we can view with vehicle editor
    snd.impulse = impulse
    snd.pitch = pitch
    snd.volume = volume

    -- Audio Debug. DON'T FORGET THERE IS SMOOTHING ON THE VOLUME RTPC WHICH ALTERS FINAL RESULT - VOLUME SMOOTHING IS NOW TURNED OFF
    -- streams.drawGraph('susp VOL w'..bi..' '..snd.volumeFactor, {value = volume, min = 0, max = 1})
    -- streams.drawGraph('w'..bi, {value = -smoothStress, min = 0, max = 1})
    -- if volume >= 0.01 then print (string.format(" Suspension%.0f   currentStress %.2f x smoothStress %.2f x volumeFactor %.2f = Volume=%.2f  Impulse=%.2f  Pitch=%.2f  colorFactor=%.2f  beamResetTimer=%.2f", bi, currentStress, smoothStress, snd.volumeFactor, volume, impulse, pitch, snd.colorFactor, beamResetTimer)); end

    -- -- one shot cabin rattles
    -- -- Teri - things required please - the event needs to be set per vehicle, potentially in x_interior.jbeam - also, should the emitters be on different nodes (they are currently set as suspension.
    -- rattleSoundTimer = rattleSoundTimer + dt
    -- if rattleSoundTimer > 0.08 then
    -- rattleSoundTimer = 0
    -- if volume > 0.25 then
    -- volume = linearScale(volume,0.25,0.95,0,1)
    -- sounds.playSoundOnceAtNode("event:>Vehicle>Interior>Rattles>car>multi_test", 0, volume)
    -- print (volume)
    -- end
    -- end
  end

  local aeroSpeed = obj:getAirflowSpeed() -- speed against wind

  -- wind
  if aeroSpeed > 3 and windSoundEvent then
    -- TODO: Find a better place to emit wind sounds. Maybe at the windows?
    windSound = windSound or createSoundObj(windSoundEvent, "AudioDefaultLoop3D", "WindTestSound", windSoundEventNode)
    --local vol = clamp(aeroSpeed * 0.015, 0, 1)
    local vol = aeroSpeed * 0.02
    --local pitch = clamp(aeroSpeed * 0.012, 0, 1) -- controls pitch of large buffet so goes up at a slower rate
    local pitch = aeroSpeed * 0.012
    windSound:setVolumePitch(vol, pitch)
  -- print (string.format("WINDSPEED KPH=%.0f MPH=%.0f  Wind vol=%.2f  pitch=%.2f", (speed*3.657), (speed*2.285), vol, pitch))
  end

  -- wheels
  local groundSpeed = obj:getGroundSpeed()

  for wi, wd in pairs(wheels.wheels) do
    local wheelSound = wheelsSounds[wi]

    local rigidSurfaceType = 0 -- textureRTPC - asphalt=0.05, cobble=15, metal=0.25, wood=0.45
    local rigidRollVolume = 0
    local rigidRollPitch = 0
    local rigidRollColor = 0 -- unused
    local rigidSkidVolume = 0
    local rigidSkidPitch = 0
    local rigidSkidSlip = 0 -- colorRTPC

    local looseSurfaceType = 0 -- textureRTPC - dirt=0.05, grass=0.15, gravel=0.25
    local looseRollVolume = 0
    local looseRollPitch = 0
    local looseRollDepth = 0
    local looseSkidVolume = 0
    local looseSkidPitch = 0
    local looseSkidDepth = 0

    --shared calculations for most surfaces
    local absWheelSpeed = abs(wd.angularVelocity * wd.radius)
    local slip = wd.lastSlip * min(wd.downForce * 0.1, 1)
    local sideSlip = abs(wd.lastSideSlip)
    local tirePressure = sqrt(wd.downForce * 0.00002) --replacing tirePatchPressure so the tireGoem from tirePatchPressure can become a static RTPC. Was using a linear range of 0 - 30,000
    -- local vehicleWheelSpeedDiff = sign(absWheelSpeed - groundSpeed) * wd.lastSlip * 0.7
    local vehicleWheelSpeedDiff = absWheelSpeed - groundSpeed

    -- if wd.name == "RR" then print(string.format("RR skids = slip %6.3f / slipEnergy %6.3f / sideSlip %6.3f / lastSlip %0.3f", slip, wd.slipEnergy * 0.000005, sideSlip, wd.lastSlip * 0.0125)); end

    -- if wd.name == "RR" or wd.name == "RL" then streams.drawGraph(wd.name.." sideSlip", {value = sideSlip * 0.0125, min = 0, max = 0.3}); end

    --Release settings
    local vehicleWheelSpeedDiffSlip
    if vehicleWheelSpeedDiff > 0 then
      --vehicleWheelSpeedDiffSlip = (vehicleWheelSpeedDiff * 0.01 + sideSlip * 0.0125) * 0.5
      vehicleWheelSpeedDiffSlip = sideSlip * 0.0125
    else
      vehicleWheelSpeedDiffSlip = vehicleWheelSpeedDiff * 0.01
    end

    local mat, mat2 = wd.contactMaterialID1, wd.contactMaterialID2
    if mat == 4 then
      mat, mat2 = mat2, mat
    end
    local isRubberTire = mat2 == 4 and wd.hasTire -- if the tire is rubber
    local maxContactBase = obj:inWater(wd.node1) and math.huge or 0
    local maxContact = maxContactBase

    -- print (string.format(" wd.downf=%6.3d : tirePressure=%5.2f : wd.lastSlip*0.01=%7.3f : slip=%7.3f ; absWhlSpd=%6.1f/MPH%3.0f : c_tirePropVol=%6.3f, c_tirePropPit=%6.3f", wd.downForce, tirePressure, wd.lastSlip * 0.01, slip, absWheelSpeed, (absWheelSpeed*2.285), wheelSound.tirePropertiesVolRoll, wheelSound.tirePropertiesPitch).." "..wd.name)
    -- print (string.format("tireVolumePitch=%7.3f : wd.tireVolume=%7.3f : material=%2.0f : wd.contactDepth=%0.2f", wheelSound.tireVolumePitch, wd.tireVolume, mat, wd.contactDepth).." "..wd.name)

    -- streams.drawGraph(wd.name.." vehicleWheelSpeedDiffSlip", {value = vehicleWheelSpeedDiffSlip + 0.5, min = 0, max = 1})
    -- streams.drawGraph(wd.name.." tirePressure", {value = tirePressure, min = 0, max = 1})
    -- streams.drawGraph(wd.name.." slip", {value = slip, min = 0, max = 1})
    -- streams.drawGraph(wd.name.." wd.downForce * 0.00002", {value = sqrt(wd.downForce * 0.00002), min = 0, max = 1})
    -- streams.drawGraph(wd.name.." lastSlip", {value = wd.lastSlip * 0.0125, min = 0, max = 1})
    -- streams.drawGraph(wd.name.." slipEnergy", {value = wd.slipEnergy * 0.000005, min = 0, max = 1})
    -- streams.drawGraph(wd.name.." sideSlip", {value = sideSlip * 0.0125, min = 0, max = 1})

    -- shows the differences between slip and spin/lock
    -- if wd.name == "RR" then
    -- streams.drawGraph(wd.name.." lastSlip", {value = wd.lastSlip * 0.0125, min = 0, max = 1})
    -- streams.drawGraph(wd.name.." vehicleWheelSpeedDiff", {value = (vehicleWheelSpeedDiff * 0.01) + 0.5, min = 0, max = 1})
    -- streams.drawGraph(wd.name.." tirePressure", {value = tirePressure, min = 0, max = 1})
    -- end

    -- RIGID asphalt
    local asphaltContactSmooth = wheelSound.asphaltContactSmoother:getUncapped(boolToNum[(mat == 10 or mat == 29) and wd.contactDepth == 0 and isRubberTire], dt)
    if asphaltContactSmooth > maxContact then
      maxContact = asphaltContactSmooth
      rigidSurfaceType = 0.025
      -- rigidRollVolume = min(10, absWheelSpeed * 0.015) * asphaltContactSmooth
      rigidRollVolume = min(1, max(abs(absWheelSpeed - slip), sideSlip) * 0.015) * asphaltContactSmooth
      rigidRollPitch = tirePressure
      rigidSkidVolume = slip * wheelSound.tirePropertiesSlip * 0.015 * asphaltContactSmooth
      -- rigidSkidVolume = slip * wheelSound.tirePropertiesSlip * asphaltContactSmooth
      -- rigidSkidVolume = sideSlip * wheelSound.tirePropertiesSlip * asphaltContactSmooth * 0.0125
      rigidSkidPitch = tirePressure
      rigidSkidSlip = vehicleWheelSpeedDiffSlip * wheelSound.tirePropertiesSlip

    --Mark Test lock spin on color, slide on something else skid asphalt v6a
    -- rigidSkidVolume = (sideSlip * 0.05) * wheelSound.tirePropertiesSlip * asphaltContactSmooth
    -- rigidSkidPitch = tirePressure
    -- rigidSkidSlip = vehicleWheelSpeedDiffSlip * wheelSound.tirePropertiesSlip * asphaltContactSmooth
    -- DON'T DELETE
    -- if wd.name == "RR" then
    -- if rigidRollVolume > 0.01  then print (string.format("ASPHAT KPH=%3.0f MPH=%3.0f / absWhlSpeed %5.1f / rollVolume %4.2f / RollPitch %4.2f / tirePressure%6.2f / Contact %.1f", (absWheelSpeed*3.656), (absWheelSpeed*2.285), absWheelSpeed, rigidRollVolume, rigidRollPitch, tirePressure, asphaltContactSmooth).." "..wd.name); end
    -- if rigidSkidVolume > 0.01 then print (string.format(" "..wd.name.." ".."Skid Volume %.2f : Pitch %.2f : Color(+0.5) %.2f : tirePressure %.2f : Slip %6.2f : wd.lastSlip %6.2f : wd.slipEnergy %6.0f", rigidSkidVolume, rigidSkidPitch, rigidSkidSlip + 0.5, tirePressure, slip, wd.lastSlip, wd.slipEnergy)); end
    -- end

      -- RIGID asphalt kickup
      local wheelPeripherySpeedKickup = max(slip * wheelSound.tirePropertiesKickup * 10, absWheelSpeed * wheelSound.tirePropertiesKickup * 1)
      wheelSound.looseSurfaceKickupLimit = wheelSound.looseSurfaceKickupLimit - dt * wheelPeripherySpeedKickup
      if wheelSound.looseSurfaceKickupLimit <= 0 and wheelPeripherySpeedKickup > 2 and wd.tireSoundVolumeCoef > 0 then
        local kickupVolume = min(1, wheelPeripherySpeedKickup * 0.01 * wheelSound.tirePropertiesKickup)
        playSoundOnceAtNode("event:>Surfaces>kickup_asphalt", wd.node1, kickupVolume * wd.tireSoundVolumeCoef, wheelSound.tirePropertiesVolRoll, wheelSound.tirePropertiesPitch, 1)
        wheelSound.looseSurfaceKickupLimit = randomGauss3() * 8 / wheelSound.tirePropertiesKickup
      -- print(string.format("KICKUP ASPHALT Vol=%.2f : Pitch=%.2f : Color=%.2f : tirePropertiesKickup=%.2f", kickupVolume, wheelSound.tirePropertiesVolRoll, wheelSound.tirePropertiesPitch, wheelSound.tirePropertiesKickup) .. " " .. wd.name)
      -- streams.drawGraph(wd.name.." kickupVolume", {value = kickupVolume, min = 0, max = 1})
      end
    end

    -- RIGID asphalt wet
    local asphaltwetContactSmooth = wheelSound.asphaltwetContactSmoother:getUncapped(boolToNum[mat == 11 and wd.contactDepth == 0 and isRubberTire], dt)
    if asphaltwetContactSmooth > maxContact then
      maxContact = asphaltwetContactSmooth
      rigidSurfaceType = 0.125
      -- rigidRollVolume = min(10, absWheelSpeed * 0.015) * asphaltwetContactSmooth
      rigidRollVolume = min(1, max(abs(absWheelSpeed - slip), sideSlip) * 0.015) * asphaltwetContactSmooth
      rigidRollPitch = tirePressure
      rigidSkidVolume = slip * wheelSound.tirePropertiesSlip * 0.025 * asphaltwetContactSmooth
      rigidSkidPitch = tirePressure
      rigidSkidSlip = vehicleWheelSpeedDiffSlip * wheelSound.tirePropertiesSlip
    -- if rigidSkidVolume > 0.01 then print (string.format(" ASPHALT WET SKID / rigidSkidVolume %.2f / rigidSkidPitch %.2f / rigidSkidColor(+0.5) %.2f", rigidSkidVolume, rigidSkidPitch, rigidSkidColor + 0.5).." "..wd.name); end
    end

    -- RIGID cobble stone
    local cobbleStoneContactSmooth = wheelSound.cobbleStoneContactSmoother:getUncapped(boolToNum[mat == 30 and wd.contactDepth == 0 and isRubberTire], dt)
    if cobbleStoneContactSmooth > maxContact then
      maxContact = cobbleStoneContactSmooth
      rigidSurfaceType = 0.175
      -- rigidRollVolume = min(1, max(absWheelSpeed, min(slip, groundSpeed)) * 0.025) * (randomGauss3() * 0.33 + 0.5) * cobbleStoneContactSmooth
      rigidRollVolume = min(1, max(abs(absWheelSpeed - slip), sideSlip) * 0.02) * cobbleStoneContactSmooth
      rigidRollPitch = tirePressure
      rigidSkidVolume = slip * wheelSound.tirePropertiesSlip * 0.025 * cobbleStoneContactSmooth
      rigidSkidPitch = tirePressure
      rigidSkidSlip = vehicleWheelSpeedDiffSlip * wheelSound.tirePropertiesSlip
    -- this should really do slip + sideSlip and keep the roll playing, and then slip on its own to mute the roll (for wheelspin)

    -- if slip > 1 and sideSlip < 0.05 then rigidRollVolume = rigidRollVolume * (aeroSpeed * 0.01) else
    -- if sideSlip > 0.05 then rigidRollVolume = rigidRollVolume * 0.25 else
    -- rigidRollVolume = rigidRollVolume
    -- end
    -- end

    -- if rigidSkidVolume > 0.01 then print (string.format(" COBBLE SKID / rigidSkidVolume %.2f / rigidSkidPitch %.2f / rigidSkidSlip(+0.5) %.2f", rigidSkidVolume, rigidSkidPitch, rigidSkidSlip + 0.5).." "..wd.name); end
    -- if rigidSkidVolume > 0.01 then print (string.format(" COBBLE SKID / slip %.2f / sideSlip %.2f / absWheelSpeed %.2f / aeroSpeed %.2f", slip, sideSlip * 0.0125, absWheelSpeed, aeroSpeed).." "..wd.name); end
    end

    -- RIGID ice
    local iceContactSmooth = wheelSound.iceContactSmoother:getUncapped(boolToNum[mat == 21 and wd.contactDepth == 0 and isRubberTire], dt)
    if iceContactSmooth > maxContact then
      maxContact = iceContactSmooth
      rigidSurfaceType = 0.225
      -- rigidRollVolume = min(1, absWheelSpeed * 0.015) * iceContactSmooth
      rigidRollVolume = min(1, max(abs(absWheelSpeed - slip), sideSlip) * 0.015) * iceContactSmooth
      rigidRollPitch = tirePressure
      rigidSkidVolume = slip * wheelSound.tirePropertiesSlip * 0.025 * iceContactSmooth
      rigidSkidPitch = tirePressure
      rigidSkidSlip = vehicleWheelSpeedDiffSlip * wheelSound.tirePropertiesSlip
    -- if rigidSkidVolume > 0.01 then print (string.format(" ICE SKID / rigidSkidVolume %.2f / rigidSkidPitch %.2f / rigidSkidSlip(+0.5) %.2f", rigidSkidVolume, rigidSkidPitch, rigidSkidSlip + 0.5).." "..wd.name); end
    end

    -- RIGID metal
    local metalContactSmooth = wheelSound.metalContactSmoother:getUncapped(boolToNum[mat == 2 and wd.contactDepth == 0 and isRubberTire], dt)
    if metalContactSmooth > maxContact then
      maxContact = metalContactSmooth
      rigidSurfaceType = 0.275
      -- rigidRollVolume = min(1, absWheelSpeed * 0.03) * metalContactSmooth
      rigidRollVolume = min(1, max(abs(absWheelSpeed - slip), sideSlip) * 0.02) * metalContactSmooth
      rigidRollPitch = tirePressure
      rigidSkidVolume = slip * wheelSound.tirePropertiesSlip * 0.025 * metalContactSmooth
      rigidSkidPitch = tirePressure
      rigidSkidSlip = vehicleWheelSpeedDiffSlip * wheelSound.tirePropertiesSlip
    -- if rigidSkidVolume > 0.01 then print (string.format(" METAL SKID / rigidSkidVolume %.2f / rigidSkidPitch %.2f / rigidSkidSlip(+0.5) %.2f", rigidSkidVolume, rigidSkidPitch, rigidSkidSlip + 0.5).." "..wd.name); end
    end

    -- RIGID wood
    local woodContactSmooth = wheelSound.woodContactSmoother:getUncapped(boolToNum[mat == 6 and wd.contactDepth == 0 and isRubberTire], dt)
    if woodContactSmooth > maxContact then
      maxContact = woodContactSmooth
      rigidSurfaceType = 0.375
      -- rigidRollVolume = min(1, max(absWheelSpeed, slip) * 0.025) * randomGauss3() * 0.66 * woodContactSmooth
      rigidRollVolume = min(1, max(abs(absWheelSpeed - slip), sideSlip) * 0.025) * woodContactSmooth
      rigidRollPitch = tirePressure
      rigidSkidVolume = slip * wheelSound.tirePropertiesSlip * 0.025 * woodContactSmooth
      rigidSkidPitch = tirePressure
      rigidSkidSlip = vehicleWheelSpeedDiffSlip * wheelSound.tirePropertiesSlip
    -- if rigidSkidVolume > 0.01 then print (string.format(" WOOD SKID / rigidSkidVolume %.2f / rigidSkidPitch %.2f / rigidSkidSlip(+0.5) %.2f", rigidSkidVolume, rigidSkidPitch, rigidSkidSlip + 0.5).." "..wd.name); end
    end

    -- Audio Debug - prints for roll/skid RTPC's for all wheels
    -- streams.drawGraph(wd.name.." rigidRollVolume", {value = rigidRollVolume, min = 0, max = 1})
    -- streams.drawGraph(wd.name.." rigidRollPitch", {value = rigidRollPtich, min = 0, max = 1})
    -- streams.drawGraph(wd.name.." rigidSkidVolume", {value = rigidSkidVolume, min = 0, max = 1})
    -- streams.drawGraph(wd.name.." rigidSkidPitch", {value = rigidSkidPitch, min = 0, max = 1})
    -- streams.drawGraph(wd.name.." rigidSkidSlip + 0.5", {value = rigidSkidSlip + 0.5, min = 0, max = 1})
    -- streams.drawGraph(wd.name.." slip * 0.01", {value = slip * 0.01, min = 0, max = 1})

    -- prints for roll/skid RTPC's for an individual wheel
    -- if wd.name == "RR" then
    -- streams.drawGraph(wd.name.." rigidSkidVolume (slip)", {value = rigidSkidVolume, min = 0, max = 1})
    -- streams.drawGraph(wd.name.." rigidSkidPitch (pressure on surface)", {value = rigidSkidPitch, min = 0, max = 1})
    -- streams.drawGraph(wd.name.." rigidSkidSlip + 0.5 (Slip difference)", {value = rigidSkidSlip + 0.5, min = 0, max = 1})
    -- streams.drawGraph(wd.name.." sideSlip", {value = sideSlip * 0.0125, min = 0, max = 1})
    -- end

    if wd.tireSoundVolumeCoef > 0 then
      wheelSound.rigidRoll:setVolumePitch(rigidRollVolume * wd.tireSoundVolumeCoef, rigidRollPitch, rigidRollColor, rigidSurfaceType)
      wheelSound.rigidSkid:setVolumePitch(rigidSkidVolume * wd.tireSoundVolumeCoef, rigidSkidPitch, rigidSkidSlip + 0.5, rigidSurfaceType)
    end

    maxContact = maxContactBase --I don't think this is required twuce

    -- LOOSE dirt
    local dirtContactSmooth = wheelSound.dirtContactSmoother:getUncapped(boolToNum[mat == 15 and isRubberTire], dt)
    if dirtContactSmooth > maxContact then
      -- if wd.name == "RL" and wd.contactDepth > 0 then print (string.format("  DIRT depth=%.2f", wd.contactDepth).." "..wd.name);end
      maxContact = dirtContactSmooth
      looseSurfaceType = 0.025
      looseRollVolume = min(1, absWheelSpeed * 0.015) * dirtContactSmooth
      looseRollPitch = tirePressure
      looseRollDepth = wd.contactDepth + 0.1
      looseSkidVolume = slip * wheelSound.tirePropertiesSlip * 0.030 * dirtContactSmooth
      looseSkidPitch = tirePressure
      looseSkidDepth = wd.contactDepth
      -- if looseRollVolume > 0.001 then print (string.format("  DIRT ROLL Vol=%.2f : Pitch=%.2f : Depth=%.2f : Texture=%.2f", looseRollVolume, looseRollPitch, looseRollDepth, looseSurfaceType).." "..wd.name); end
      -- if looseSkidVolume > 0.001 then print (string.format("  DIRT SKID Vol=%.2f : Pitch=%.2f : Depth=%.2f : Texture=%.2f", looseSkidVolume, looseSkidPitch, looseSkidDepth, looseSurfaceType).." "..wd.name); end
      -- streams.drawGraph(wd.name.." looseSkidVolume", {value = looseSkidVolume, min = 0, max = 1})

      -- LOOSE dirt kickup
      local wheelPeripherySpeedKickup = max(slip * wheelSound.tirePropertiesKickup * 10, absWheelSpeed * wheelSound.tirePropertiesKickup * 4)
      wheelSound.looseSurfaceKickupLimit = wheelSound.looseSurfaceKickupLimit - dt * wheelPeripherySpeedKickup
      if wheelSound.looseSurfaceKickupLimit <= 0 and wheelPeripherySpeedKickup > 2 and wd.tireSoundVolumeCoef > 0 then
        local kickupVolume = min(1, wheelPeripherySpeedKickup * 0.002 * wheelSound.tirePropertiesKickup)
        playSoundOnceAtNode("event:>Surfaces>kickup_dirt", wd.node1, kickupVolume * wd.tireSoundVolumeCoef, wheelSound.tirePropertiesVolRoll, wheelSound.tirePropertiesPitch, 1)
        wheelSound.looseSurfaceKickupLimit = randomGauss3() * 8 / wheelSound.tirePropertiesKickup
      -- print(string.format("KICKUP DIRT Vol=%.2f : Pitch=%.2f : Color=%.2f : tirePropertiesKickup=%.2f", kickupVolume, wheelSound.tirePropertiesVolRoll, wheelSound.tirePropertiesPitch, wheelSound.tirePropertiesKickup) .. " " .. wd.name)
      -- streams.drawGraph(wd.name.." kickupVolume", {value = kickupVolume, min = 0, max = 1})
      end
    end

    -- LOOSE dirtDusty
    local dirtdustyContactSmooth = wheelSound.dirtDustyContactSmoother:getUncapped(boolToNum[mat == 14 and isRubberTire], dt)
    if dirtdustyContactSmooth > maxContact then
      -- if wd.name == "RL" and wd.contactDepth > 0 then print (string.format("  DIRT depth=%.2f", wd.contactDepth).." "..wd.name);end
      maxContact = dirtdustyContactSmooth
      looseSurfaceType = 0.075
      looseRollVolume = min(1, absWheelSpeed * 0.015) * dirtdustyContactSmooth
      looseRollPitch = tirePressure
      looseRollDepth = wd.contactDepth
      looseSkidVolume = slip * wheelSound.tirePropertiesSlip * 0.030 * dirtdustyContactSmooth
      looseSkidPitch = tirePressure
      looseSkidDepth = wd.contactDepth
      -- if looseRollVolume > 0.001 then print (string.format(" DUSTY ROLL Vol=%.2f : Pitch=%.2f : Depth=%.2f : Texture=%.2f", looseRollVolume, looseRollPitch, looseRollDepth, looseSurfaceType).." "..wd.name); end
      -- if looseSkidVolume > 0.001 then print (string.format(" DUSTY SKID Vol=%.2f : Pitch=%.2f : Depth=%.2f : Texture=%.2f", looseSkidVolume, looseSkidPitch, looseSkidDepth, looseSurfaceType).." "..wd.name); end
      -- streams.drawGraph(wd.name.." looseSkidVolume", {value = looseSkidVolume, min = 0, max = 1})

      -- LOOSE dirt kickup
      local wheelPeripherySpeedKickup = max(slip * wheelSound.tirePropertiesKickup * 10, absWheelSpeed * wheelSound.tirePropertiesKickup * 4)
      wheelSound.looseSurfaceKickupLimit = wheelSound.looseSurfaceKickupLimit - dt * wheelPeripherySpeedKickup
      if wheelSound.looseSurfaceKickupLimit <= 0 and wheelPeripherySpeedKickup > 2 and wd.tireSoundVolumeCoef > 0 then
        local kickupVolume = min(1, wheelPeripherySpeedKickup * 0.002 * wheelSound.tirePropertiesKickup)
        playSoundOnceAtNode("event:>Surfaces>kickup_dirtDusty", wd.node1, kickupVolume * wd.tireSoundVolumeCoef, wheelSound.tirePropertiesVolRoll, wheelSound.tirePropertiesPitch, 1)
        wheelSound.looseSurfaceKickupLimit = wheelSound.looseSurfaceKickupLimit + randomGauss3() * 20 / wheelSound.tirePropertiesKickup
      -- print(string.format("KICKUP DUST Vol=%.2f : Pitch=%.2f : Color=%.2f : tirePropertiesKickup=%.2f", kickupVolume, wheelSound.tirePropertiesVolRoll, wheelSound.tirePropertiesPitch, wheelSound.tirePropertiesKickup) .. " " .. wd.name)
      -- streams.drawGraph(wd.name.." kickupVolume", {value = kickupVolume, min = 0, max = 1})
      end
    end

    -- LOOSE grass
    local grassContactSmooth = wheelSound.grassContactSmoother:getUncapped(boolToNum[mat == 20 and isRubberTire], dt)
    if grassContactSmooth > maxContact then
      -- if wd.name == "RL" and wd.contactDepth > 0 then print (string.format(" GRASS depth=%.2f", wd.contactDepth).." "..wd.name);end
      maxContact = grassContactSmooth
      looseSurfaceType = 0.125
      looseRollVolume = min(1, absWheelSpeed * 0.015) * grassContactSmooth
      looseRollPitch = tirePressure
      looseRollDepth = wd.contactDepth
      looseSkidVolume = slip * wheelSound.tirePropertiesSlip * 0.030 * grassContactSmooth
      looseSkidPitch = tirePressure
      looseSkidDepth = wd.contactDepth
      -- if looseRollVolume > 0.001 then print (string.format(" GRASS ROLL Vol=%.2f : Pitch=%.2f : Depth=%.2f : Texture=%.2f", looseRollVolume, looseRollPitch, looseRollDepth, looseSurfaceType).." "..wd.name); end
      -- if looseSkidVolume > 0.001 then print (string.format(" GRASS SKID Vol=%.2f : Pitch=%.2f : Depth=%.2f : Texture=%.2f", looseSkidVolume, looseSkidPitch, looseSkidDepth, looseSurfaceType).." "..wd.name); end
      -- streams.drawGraph(wd.name.." looseSkidVolume", {value = looseSkidVolume, min = 0, max = 1})

      -- LOOSE grass kickup
      local wheelPeripherySpeedKickup = max(slip * wheelSound.tirePropertiesKickup * 12, absWheelSpeed * wheelSound.tirePropertiesKickup * 8)
      wheelSound.looseSurfaceKickupLimit = wheelSound.looseSurfaceKickupLimit - dt * wheelPeripherySpeedKickup
      if wheelSound.looseSurfaceKickupLimit <= 0 and wheelPeripherySpeedKickup > 2 and wd.tireSoundVolumeCoef > 0 then
        local kickupVolume = min(1, wheelPeripherySpeedKickup * 0.0024 * wheelSound.tirePropertiesKickup)
        playSoundOnceAtNode("event:>Surfaces>kickup_grass", wd.node1, kickupVolume * wd.tireSoundVolumeCoef, wheelSound.tirePropertiesVolRoll, wheelSound.tirePropertiesPitch, 1)
        wheelSound.looseSurfaceKickupLimit = randomGauss3() * 12 / wheelSound.tirePropertiesKickup
      -- print (string.format("KICKUP GRASS Vol=%.2f : Pitch=%.2f : Color=%.2f : tirePropertiesKickup=%.2f", kickupVolume, wheelSound.tirePropertiesVolRoll, wheelSound.tirePropertiesPitch, wheelSound.tirePropertiesKickup).." "..wd.name)
      -- streams.drawGraph(wd.name.." kickupVolume", {value = kickupVolume, min = 0, max = 1})
      end
    end

    -- LOOSE gravel
    local gravelContactSmooth = wheelSound.gravelContactSmoother:getUncapped(boolToNum[mat == 19 and isRubberTire], dt)
    if gravelContactSmooth > maxContact then
      -- if wd.name == "RL" and wd.contactDepth > 0 then print (string.format("GRAVEL depth=%.2f", wd.contactDepth).." "..wd.name);end
      maxContact = gravelContactSmooth
      looseSurfaceType = 0.175
      looseRollVolume = min(1, absWheelSpeed * 0.015) * gravelContactSmooth
      looseRollPitch = tirePressure
      looseRollDepth = wd.contactDepth + 0.4
      looseSkidVolume = slip * wheelSound.tirePropertiesSlip * 0.030 * gravelContactSmooth
      looseSkidPitch = tirePressure
      looseSkidDepth = wd.contactDepth
      -- if looseRollVolume > 0.001 then print (string.format("GRAVEL ROLL Vol=%.2f : Pitch=%.2f : Depth=%.2f : Texture=%.2f", looseRollVolume, looseRollPitch, looseRollDepth, looseSurfaceType).." "..wd.name); end
      -- if looseSkidVolume > 0.001 then print (string.format("GRAVEL SKID Vol=%.2f : Pitch=%.2f : Depth=%.2f : Texture=%.2f : tirePropertiesSlip=%.2f", looseSkidVolume, looseSkidPitch, looseSkidDepth, looseSurfaceType, wheelSound.tirePropertiesSlip).." "..wd.name); end
      -- streams.drawGraph(wd.name.." looseSkidVolume", {value = looseSkidVolume, min = 0, max = 1})

      local wheelPeripherySpeedKickup = max(slip * wheelSound.tirePropertiesKickup * 60, absWheelSpeed * wheelSound.tirePropertiesKickup * 20)
      wheelSound.looseSurfaceKickupLimit = wheelSound.looseSurfaceKickupLimit - dt * wheelPeripherySpeedKickup
      if wheelSound.looseSurfaceKickupLimit <= 0 and wheelPeripherySpeedKickup > 2 and wd.tireSoundVolumeCoef > 0 then
        local kickupVolume = min(1, wheelPeripherySpeedKickup * 0.0003 * wheelSound.tirePropertiesKickup)
        playSoundOnceAtNode("event:>Surfaces>kickup_gravel", wd.node1, kickupVolume * wd.tireSoundVolumeCoef, wheelSound.tirePropertiesVolRoll, wheelSound.tirePropertiesPitch, 1)
        wheelSound.looseSurfaceKickupLimit = wheelSound.looseSurfaceKickupLimit + randomGauss3() * 16 / wheelSound.tirePropertiesKickup
      -- print (string.format("KICKUP Slip=%.2f : Slip*Properties=%.2f", slip * 12, slip * wheelSound.tirePropertiesKickup * 12).." "..wd.name)
      -- print (string.format("KICKUP GRAVEL Vol=%.2f : Pitch=%.2f : Color=%.2f : tirePropertiesKickup=%.2f", kickupVolume, wheelSound.tirePropertiesVolRoll, wheelSound.tirePropertiesPitch, wheelSound.tirePropertiesKickup).." "..wd.name)
      -- streams.drawGraph(wd.name.." kickupVolume", {value = kickupVolume, min = 0, max = 1})
      end
    end

    -- LOOSE mud
    local mudContactSmooth = wheelSound.mudContactSmoother:getUncapped(boolToNum[mat == 18 and isRubberTire], dt)
    if mudContactSmooth > maxContact then
      -- if wd.name == "RL" and wd.contactDepth > 0 then print (string.format("  MUD depth=%.2f", wd.contactDepth).." "..wd.name);end
      maxContact = mudContactSmooth
      looseSurfaceType = 0.225
      looseRollVolume = min(1, absWheelSpeed * 0.015) * mudContactSmooth
      looseRollPitch = tirePressure
      looseRollDepth = wd.contactDepth * 2
      looseSkidVolume = slip * wheelSound.tirePropertiesSlip * 0.030 * mudContactSmooth
      looseSkidPitch = tirePressure
      looseSkidDepth = wd.contactDepth
      -- if looseRollVolume > 0.001 then print (string.format("   MUD ROLL Vol=%.2f : Pitch=%.2f : Depth=%.2f : Texture=%.2f", looseRollVolume, looseRollPitch, looseRollDepth, looseSurfaceType).." "..wd.name); end
      -- if looseSkidVolume > 0.001 then print (string.format("   MUD SKID Vol=%.2f : Pitch=%.2f : Depth=%.2f : Texture=%.2f", looseSkidVolume, looseSkidPitch, looseSkidDepth, looseSurfaceType).." "..wd.name); end
      -- streams.drawGraph(wd.name.." looseSkidVolume", {value = looseSkidVolume, min = 0, max = 1})
      -- streams.drawGraph(wd.name.."  mud looseRollDepth", {value = looseRollDepth, min = 0, max = 0.5})

      local wheelPeripherySpeedKickup = max(slip * wheelSound.tirePropertiesKickup * 20, absWheelSpeed * wheelSound.tirePropertiesKickup * 4)
      wheelSound.looseSurfaceKickupLimit = wheelSound.looseSurfaceKickupLimit - dt * wheelPeripherySpeedKickup
      if wheelSound.looseSurfaceKickupLimit <= 0 and wheelPeripherySpeedKickup > 2 and wd.tireSoundVolumeCoef > 0 then
        local kickupVolume = min(1, wheelPeripherySpeedKickup * 0.0024 * wheelSound.tirePropertiesKickup)
        playSoundOnceAtNode("event:>Surfaces>kickup_mud", wd.node1, kickupVolume * wd.tireSoundVolumeCoef, wheelSound.tirePropertiesVolRoll, wheelSound.tirePropertiesPitch, 1)
        wheelSound.looseSurfaceKickupLimit = randomGauss3() * 8 / wheelSound.tirePropertiesKickup
      -- print (string.format("KICKUP MUD Vol=%.2f : Pitch=%.2f : Color=%.2f : tirePropertiesKickup=%.2f", kickupVolume, wheelSound.tirePropertiesVolRoll, wheelSound.tirePropertiesPitch, wheelSound.tirePropertiesKickup).." "..wd.name)
      -- streams.drawGraph(wd.name.." kickupVolume", {value = kickupVolume, min = 0, max = 1})
      end
    end

    -- LOOSE rock
    local rockContactSmooth = wheelSound.rockContactSmoother:getUncapped(boolToNum[mat == 13 and isRubberTire], dt)
    if rockContactSmooth > maxContact then
      maxContact = rockContactSmooth
      looseSurfaceType = 0.275
      looseRollVolume = min(1, absWheelSpeed * 0.015) * rockContactSmooth
      looseRollPitch = tirePressure
      looseRollDepth = wd.contactDepth
      looseSkidVolume = slip * wheelSound.tirePropertiesSlip * 0.030 * rockContactSmooth
      looseSkidPitch = tirePressure
      looseSkidDepth = wd.contactDepth
      -- if looseRollVolume > 0.001 then print (string.format("  ROCK ROLL Vol=%.2f : Pitch=%.2f : Depth=%.2f : Texture=%.2f", looseRollVolume, looseRollPitch, looseRollDepth, looseSurfaceType).." "..wd.name); end
      -- if looseSkidVolume > 0.001 then print (string.format("  ROCK SKID Vol=%.2f : Pitch=%.2f : Depth=%.2f : Texture=%.2f", looseSkidVolume, looseSkidPitch, looseSkidDepth, looseSurfaceType).." "..wd.name); end
      -- streams.drawGraph(wd.name.." looseSkidVolume", {value = looseSkidVolume, min = 0, max = 1})

      local wheelPeripherySpeedKickup = max(slip * wheelSound.tirePropertiesKickup * 20, absWheelSpeed * wheelSound.tirePropertiesKickup * 5)
      wheelSound.looseSurfaceKickupLimit = wheelSound.looseSurfaceKickupLimit - dt * wheelPeripherySpeedKickup
      if wheelSound.looseSurfaceKickupLimit <= 0 and wheelPeripherySpeedKickup > 2 and wd.tireSoundVolumeCoef > 0 then
        local kickupVolume = min(1, wheelPeripherySpeedKickup * 0.0024 * wheelSound.tirePropertiesKickup)
        playSoundOnceAtNode("event:>Surfaces>kickup_rock", wd.node1, kickupVolume * wd.tireSoundVolumeCoef, wheelSound.tirePropertiesVolRoll, wheelSound.tirePropertiesPitch, 1)
        wheelSound.looseSurfaceKickupLimit = randomGauss3() * 8 / wheelSound.tirePropertiesKickup
      -- print (string.format("KICKUP ROCK Vol=%.2f : Pitch=%.2f : Color=%.2f : tirePropertiesKickup=%.2f", kickupVolume, wheelSound.tirePropertiesVolRoll, wheelSound.tirePropertiesPitch, wheelSound.tirePropertiesKickup).." "..wd.name)
      -- streams.drawGraph(wd.name.." kickupVolume", {value = kickupVolume, min = 0, max = 1})
      end
    end

    -- LOOSE sand
    local sandContactSmooth = wheelSound.sandContactSmoother:getUncapped(boolToNum[mat == 16 and isRubberTire], dt)
    if sandContactSmooth > maxContact then
      -- if wd.name == "RL" and wd.contactDepth > 0 then print (string.format(" SAND depth=%.2f", wd.contactDepth).." "..wd.name);end
      maxContact = sandContactSmooth
      looseSurfaceType = 0.325
      looseRollVolume = min(1, absWheelSpeed * 0.015) * sandContactSmooth
      looseRollPitch = tirePressure
      looseRollDepth = wd.contactDepth * 1
      looseSkidVolume = slip * wheelSound.tirePropertiesSlip * 0.020 * sandContactSmooth
      looseSkidPitch = tirePressure
      looseSkidDepth = wd.contactDepth
      -- if looseRollVolume > 0.001 then print (string.format("  SAND ROLL Vol=%.2f : Pitch=%.2f : Depth=%.2f : Texture=%.2f", looseRollVolume, looseRollPitch, looseRollDepth, looseSurfaceType).." "..wd.name); end
      -- if looseSkidVolume > 0.001 then print (string.format("  SAND SKID Vol=%.2f : Pitch=%.2f : Depth=%.2f : Texture=%.2f", looseSkidVolume, looseSkidPitch, looseSkidDepth, looseSurfaceType).." "..wd.name); end
      -- if wd.name == "FL" then streams.drawGraph(wd.name.." looseSkidVolume", {value = looseSkidVolume}); end
      -- streams.drawGraph(wd.name.." sand looseRollDepth", {value = looseRollDepth, min = 0, max = 0.5})

      local wheelPeripherySpeedKickup = max(slip * wheelSound.tirePropertiesKickup * 20, absWheelSpeed * wheelSound.tirePropertiesKickup * 4)
      wheelSound.looseSurfaceKickupLimit = wheelSound.looseSurfaceKickupLimit - dt * wheelPeripherySpeedKickup
      if wheelSound.looseSurfaceKickupLimit <= 0 and wheelPeripherySpeedKickup > 2 and wd.tireSoundVolumeCoef > 0 then
        local kickupVolume = min(1, wheelPeripherySpeedKickup * 0.0010 * wheelSound.tirePropertiesKickup)
        playSoundOnceAtNode("event:>Surfaces>kickup_sand", wd.node1, kickupVolume * wd.tireSoundVolumeCoef, wheelSound.tirePropertiesVolRoll, wheelSound.tirePropertiesPitch, 1)
        wheelSound.looseSurfaceKickupLimit = randomGauss3() * 4 / wheelSound.tirePropertiesKickup
      -- print (string.format("KICKUP SAND Vol=%.2f : Pitch=%.2f : Color=%.2f : tirePropertiesKickup=%.2f", kickupVolume, wheelSound.tirePropertiesVolRoll, wheelSound.tirePropertiesPitch, wheelSound.tirePropertiesKickup).." "..wd.name)
      -- streams.drawGraph(wd.name.." kickupVolume", {value = kickupVolume, min = 0, max = 1})
      end
    end

    looseSurfaceType = wheelSound.loosenessSmoother:getUncapped(looseSurfaceType, dt)

    -- streams.drawGraph(wd.name.." looseRollVolume", {value = looseRollVolume, min = 0, max = 1})
    -- streams.drawGraph(wd.name.." looseRollPitch", {value = looseRollPitch, min = 0, max = 1})
    -- streams.drawGraph(wd.name.." looseRollDepth", {value = looseRollDepth, min = 0, max = 0.3})
    -- streams.drawGraph(wd.name.." looseSkidVolume", {value = looseSkidVolume, min = 0, max = 1})
    -- streams.drawGraph(wd.name.." looseSkidPitch", {value = looseSkidPitch, min = 0, max = 1})
    -- streams.drawGraph(wd.name.." looseSkidDepth", {value = looseSkidDepth, min = 0, max = 0.3})
    -- if looseRollDepth > 0 then print (string.format("looseSurfaceType = %0.3f / looseRollDepth = %0.2f", looseSurfaceType, looseRollDepth)); end

    if wd.tireSoundVolumeCoef > 0 then
      wheelSound.looseRoll:setVolumePitch(looseRollVolume * wd.tireSoundVolumeCoef, looseRollPitch, looseRollDepth, looseSurfaceType)
      wheelSound.looseSkid:setVolumePitch(looseSkidVolume * wd.tireSoundVolumeCoef, looseSkidPitch, looseSkidDepth, looseSurfaceType)
    end

    -- MISC flat tire
    if wd.isTireDeflated and not wd.isBroken and mat >= 0 then
      wd.deflatedTireAngle = wd.deflatedTireAngle + clamp(wd.angularVelocity, -100, 100) * dt
      if abs(wd.deflatedTireAngle) > twoPi then
        local downForceVolume = clamp(wd.downForce / 5000, 0, 1)
        local speedPitch = clamp(abs(wd.angularVelocity) / 200, 0, 1)
        obj:playSFXOnceCT(wd.flatTireSound, wd.node1, downForceVolume, speedPitch, wd.tireVolume * 5, max(M.scrapeLoosenessMap[mat] or 0, M.scrapeLoosenessMap[mat2] or 0))
        wd.deflatedTireAngle = 0
      end
    end

    -- MISC rumblestrip
    local rumbleStripContactSmooth = wheelSound.rumbleStripContactSmoother:getUncapped(boolToNum[mat == 29 and isRubberTire], dt)
    if rumbleStripContactSmooth > 0 then
      local vehicleSpeed = groundSpeed
      local peakForce = wd.peakForce
      -- if vehicleSpeed < 15 then
      if peakForce > 0 and wd.obj:getPeakPeriod() * vehicleSpeed > 0.0 and wd.tireSoundVolumeCoef > 0 then
        rigidSurfaceType = 0.35
        -- if wd.name == "FL" then print (string.format("peakForce=%6.0f / wd.obj:getPeakPeriod()=%0.2f / vehicleSpeed=%0.3f", peakForce, wd.obj:getPeakPeriod(), vehicleSpeed)); end
        local volume = min(1, peakForce * vehicleSpeed * 0.000002 / wheelSound.tireContactPatchCoef - 0.01)
        local pitch = vehicleSpeed * 0.0175
        playSoundOnceAtNode("event:>Surfaces>roll_rumblestrip", wd.node1, volume * wd.tireSoundVolumeCoef, pitch, wheelSound.tirePropertiesVolRoll)
      -- if wd.name == "FL" then playSoundOnceAtNode("event:>Surfaces>roll_rumblestrip", wd.node1, volume, pitch, wheelSound.tirePropertiesVolRoll);end
      end
    -- end
    end
  end

  if M.uiDebugging and playerInfo.firstPlayerSeated then
    guihooks.trigger("AudioDebug", soundBank)
  end
end

local function addWheelSounds(wheelID, wd)
  if wheelsSounds[wheelID] ~= nil then
    return
  end

  local tireContactPatchCoef = linearScale((wd.radius * wd.tireWidth), 0, 0.17, 0, 1) -- max contact patch so far is 0.165
  local tireContactPatchReverseNormalCoef = linearScale((wd.radius * wd.tireWidth), 0, 0.17, 1.5, 0.5) -- max contact patch so far is 0.165
  --local tireLinearScaleRadius = linearScale(wd.radius, 0, 0.5, 0, 1) -- used to help bring in the tread noise of large rugged tires
  --local tireVolWidthCoef = linearScale(wd.tireWidth, 0.05, 0.35, 0, 1)
  local tireVolProfileCoef = linearScale(wd.radius - wd.hubRadius, 0, 0.3, 1.4, 0.6)
  local tireVolTreadCoef = linearScale(wd.treadCoef, 0, 1, 0.75, 1.25)
  --local tireVolRollSoftCoef = linearScale(wd.softnessCoef, 0, 1, 1.4, 0.6) -- changes the THRESHOLD of when the squeal starts....
  local tireVolSkidSoftCoef = linearScale(wd.softnessCoef, 0, 1, 1.5, 1) -- changes the THRESHOLD of when the squeal starts....
  --local tireVolAirCoefOld = linearScale(wd.tireVolume, 0, 0.2, 0.7, 1.3) -- the larger the volume of air, the higher the volume
  local tireVolAirCoef = linearScale(wd.tireVolume, 0, 0.2, 0.8, 1.2) -- the larger the volume of air, the higher the volume
  local tirePitchWidthCoef = linearScale(wd.tireWidth, 0.05, 0.35, 0, 1)
  local tirePitchRadiusCoef = linearScale(wd.radius - wd.hubRadius, 0, 0.3, 0, 1)
  local tirePitchTreadCoef = linearScale(wd.treadCoef, 0, 1, 0.8, 1.2)
  local tirePitchSoftCoef = linearScale(wd.softnessCoef, 0, 1, 1.3, 0.7) -- changes the THRESHOLD of when the squeal starts....
  local tirePitchSoftCoef2 = linearScale(wd.softnessCoef, 0, 1, 0.7, 1.3) -- changes the THRESHOLD of when the squeal starts....
  local tirePitchAirCoef = linearScale(wd.tireVolume, 0.01, 0.25, 0, 1) -- the larger volume of air, the lower the pitch
  local tireSlipTreadCoef = linearScale(wd.treadCoef, 0, 1, 0.8, 1.2) -- changed from 0.75/1.25 just to help race tyres get a bit more slip
  local tireSlipSoftCoef = linearScale(wd.softnessCoef, 0, 1, 1.25, 0.75)
  local tireKickSoftCoef = linearScale(wd.softnessCoef, 0, 1, 0.75, 1.25)
  local wh = {
    asphaltContactSmoother = newTemporalSmoothing(4, 4),
    asphaltwetContactSmoother = newTemporalSmoothing(4, 4),
    cobbleStoneContactSmoother = newTemporalSmoothing(4, 4),
    dirtContactSmoother = newTemporalSmoothing(2, 4),
    dirtDustyContactSmoother = newTemporalSmoothing(2, 4),
    grassContactSmoother = newTemporalSmoothing(2, 4),
    gravelContactSmoother = newTemporalSmoothing(2, 4),
    metalContactSmoother = newTemporalSmoothing(4, 4),
    mudContactSmoother = newTemporalSmoothing(2, 4),
    iceContactSmoother = newTemporalSmoothing(4, 4),
    rockContactSmoother = newTemporalSmoothing(2, 4),
    sandContactSmoother = newTemporalSmoothing(2, 4),
    woodContactSmoother = newTemporalSmoothing(4, 4),
    loosenessSmoother = newTemporalSmoothing(5, 5),
    rumbleStripContactSmoother = newTemporalSmoothing(3, 10000),
    rumbleStripHighSpeedContactSmoother = newTemporalSmoothing(1, 1),
    pressureSmoother = newTemporalSmoothingNonLinear(1, 1),
    looseSurfaceKickupLimit = 0,
    tireVolumePitch = (0.5 - 2) * (wd.tireVolume - 0.010) / (0.2 - 0.01) + 2,
    tireContactPatchCoef = tireContactPatchCoef,
    tirePropertiesVolRoll = tireContactPatchCoef * tireVolTreadCoef * tireVolAirCoef * tireVolProfileCoef,
    tirePropertiesVolSkid = tireContactPatchCoef * tireVolSkidSoftCoef * tireVolAirCoef * tireVolProfileCoef,
    tirePropertiesSlip = tireContactPatchReverseNormalCoef * tireSlipSoftCoef * tireSlipTreadCoef,
    tirePropertiesPitch_old = 0.5 * tirePitchSoftCoef * tirePitchWidthCoef * tirePitchTreadCoef + (tirePitchRadiusCoef * tirePitchAirCoef),
    tirePropertiesPitch = 0.5 * tirePitchSoftCoef2 * tireContactPatchCoef * tirePitchTreadCoef + (tirePitchRadiusCoef * tirePitchAirCoef),
    -- tirePropertiesSizeTread = wd.treadCoef * tireLinearScaleRadius * tireVolWidthCoef,
    tirePropertiesSizeTread = wd.treadCoef * tireContactPatchCoef,
    tirePropertiesKickup = (tireContactPatchCoef + 0.5) * tireKickSoftCoef
  }

  -- print (string.format("(Tread) %.2f * (LinearRadius) %.2f * (tireLinearWidth) = %.2f = Total = %.2f", wd.treadCoef, tireLinearScaleRadius, tireVolWidthCoef, wd.treadCoef * tireLinearScaleRadius * tireVolWidthCoef))
  -- print ((tireContactPatchCoef + 0.5) * tireKickSoftCoef)
  -- print(wd.radius * wd.tireWidth);print(((wd.radius - wd.hubRadius) * 2) * wd.tireWidth)

  -- Print wheel sizes
  -- print (string.format("Size = %.0f/%.0f R%.0f : Actual Values - wd.radius=%.3f - wd.hubRadius=%.3f - wd.tireWidth=%.3f - contactPatch=%.3f : Tread=%.2f : Softness=%.2f", (wd.radius *2) * 100, ((wd.hubRadius * 2) / 2.54) * 100, wd.tireWidth * 1000, wd.radius, wd.hubRadius, wd.tireWidth, (wd.radius * wd.radius * wd.tireWidth), wd.treadCoef, wd.softnessCoef).." "..wd.name)

  -- tireProperties
  --print ((string.format("TRUE TIRE PHYSICS - ContactPatch=%0.3f = Width=%0.3f * wd.radius=%0.3f : wd.hubRadius=%0.3f : tireRadius=%0.3f : AirVolume=%0.3f : Soft=%0.3f : wd.treadCoef=%0.2f : tirePropertiesSizeTread=%0.2f", (wd.radius * wd.tireWidth), wd.tireWidth, wd.radius, wd.hubRadius, (wd.radius - wd.hubRadius), wd.tireVolume, wd.softnessCoef, wd.treadCoef, wh.tirePropertiesSizeTread)).." "..wd.name)

  -- TIRE SLIP PROPERTIES
  -- print ((string.format("Tire Slip TRUE/NORMALISED - ContactPatch=%0.3f/%0.3f * Soft=%0.3f/%0.3f * Tread=%0.3f/%0.3f  = PropSlip=%0.3f  ", (wd.radius * wd.tireWidth), tireContactPatchReverseNormalCoef, wd.softnessCoef, tireSlipSoftCoef, wd.treadCoef, tireSlipTreadCoef, tireContactPatchReverseNormalCoef * tireSlipSoftCoef * tireSlipTreadCoef  )).." "..wd.name); print (" ")

  -- TIRE ROLL VOLUME PROPERTIES (left value = true value, right value = normalised Coef value)
  -- print ((string.format("ROLL Vol=wd.radi=%0.2f : wd.hubR=%0.2f : VolProfile=%0.2f/%0.2f : Width=%0.2f/%0.2f : ContPat=%0.2f/%0.2f : Tread=%0.2f/%0.2f : Soft=%0.2f/%0.2f : AirVol=%0.2f/%0.2f : PropVolRoll=%0.2f : PropVolSkid=%0.2f : PropPitch=%0.2f", wd.radius, wd.hubRadius, (wd.radius - wd.hubRadius), tireVolProfileCoef, wd.tireWidth, tireVolWidthCoef, (wd.radius * wd.radius * wd.tireWidth), tireContactPatchCoef, wd.treadCoef, tireVolTreadCoef, wd.softnessCoef, tireVolRollSoftCoef, wd.tireVolume, tireVolAirCoef, wh.tirePropertiesVolRoll, wh.tirePropertiesVolSkid, wh.tirePropertiesPitch)).." "..wd.name); print (" ")

  -- TIRE SKID VOLUME PROPERTIES (left value = true value, right value = normalised game value)
  -- print ((string.format("SKID Vol=wd.radi=%0.2f : wd.hubR=%0.2f : VolProfile=%0.2f/%0.2f : Width=%0.2f/%0.2f : ContPat=%0.2f/%0.2f : Tread=%0.2f/%0.2f : Soft=%0.2f/%0.2f : AirVol=%0.2f/%0.2f : PropVolRoll=%0.2f : PropSlip=%0.2f : PropVolSkid=%0.2f : PropPitch=%0.2f", wd.radius, wd.hubRadius, (wd.radius - wd.hubRadius), tireVolProfileCoef, wd.tireWidth, tireVolWidthCoef, (wd.radius * wd.radius * wd.tireWidth), tireContactPatchCoef, wd.treadCoef, tireVolTreadCoef, wd.softnessCoef, tireVolSkidSoftCoef, wd.tireVolume, tireVolAirCoef, wh.tirePropertiesVolRoll, wh.tirePropertiesSlip, wh.tirePropertiesVolSkid, wh.tirePropertiesPitch)).." "..wd.name);  print (" ")

  -- TIRE PITCH PROPERTIES (left value = true value, right value = normalised game value)
  -- print ((string.format("SKID PITCH=RubberRadius=%0.2f/%0.2f : Width=%0.2f/%0.2f : ContPat=%0.3f/%0.2f : Tread=%0.2f/%0.2f : Soft=%0.2f/%0.2f : AirVol=%0.2f/%0.2f : PropVolRoll=%0.2f : PropVolSkid=%0.2f : PropPitch=%0.2f : PropPitchOld=%0.2f", (wd.radius - wd.hubRadius), tirePitchRadiusCoef, wd.tireWidth, tirePitchWidthCoef, (wd.radius * wd.radius * wd.tireWidth), tireContactPatchCoef, wd.treadCoef, tirePitchTreadCoef, wd.softnessCoef, tirePitchSoftCoef, wd.tireVolume, tirePitchAirCoef, wh.tirePropertiesVolRoll, wh.tirePropertiesVolSkid, wh.tirePropertiesPitch, wh.tirePropertiesPitch_old)).." "..wd.name); print (" ")

  -- TIRE KICKUP PROPERTIES
  -- print ((string.format("KICKUP tireContactPatchCoef+0.5=%.2f : tireKickSoftCoef=%.2f : tirePropertiesKickup=%.2f",tireContactPatchCoef + 0.5, tireKickSoftCoef, wh.tirePropertiesKickup)).." "..wd.name)

  -- tirePropertiesVolRoll
  -- print ((string.format("tirePropertiesVolRoll - (tireContactPatchCoef=%0.3f * tireVolTreadCoef=%0.3f) * (tireVolAirCoef=%0.3f * tireVolProfileCoef%0.3f) = %0.3f", tireContactPatchCoef, tireVolTreadCoef, tireVolAirCoef, tireVolProfileCoef, (tireContactPatchCoef * tireVolTreadCoef) * (tireVolAirCoef * tireVolProfileCoef))).." "..wd.name)

  -- tirePropertiesVolSkid
  -- print ((string.format("tirePropertiesVolSkid - (tireContactPatchCoef=%0.3f * tireVolTreadCoef=%0.3f) * (tireVolAirCoef=%0.3f * tireVolProfileCoef%0.3f) = %0.3f", tireContactPatchCoef, tireVolTreadCoef, tireVolAirCoef, tireVolProfileCoef, (tireContactPatchCoef * tireVolTreadCoef) * (tireVolAirCoef * tireVolProfileCoef))).." "..wd.name)

  --Static RTPC'st
  -- print ((string.format("wh.tirePropertiesVolRoll = %0.3f / wh.tirePropertiesPitch = %0.3f / wh.tirePropertiesSizeTread = %0.3f / wh.tirePropertiesVolSkid = %0.3f / wh.softnessCoef = %0.3f", wh.tirePropertiesVolRoll, wh.tirePropertiesPitch, wh.tirePropertiesSizeTread, wh.tirePropertiesVolSkid, wd.softnessCoef)).." "..wd.name)

  -- const char *filename, const char *descriptionName, const char* sfxProfileName, bool preload
  if wd.tireSoundVolumeCoef > 0 then
    wh.rigidRoll = createSoundObj("event:>Surfaces>roll_rigid_v2", "AudioDefaultLoop3D", "rigidRoll", wd.node1)
    wh.rigidRoll:setParameter("c_tirPrpVolRol", wh.tirePropertiesVolRoll)
    wh.rigidRoll:setParameter("c_tirPrpPitch", wh.tirePropertiesPitch)
    wh.rigidRoll:setParameter("c_tirPrpSizTrd", wh.tirePropertiesSizeTread)
    wh.rigidSkid = createSoundObj("event:>Surfaces>skid_rigid_v2", "AudioDefaultLoop3D", "rigidSkid", wd.node1)
    wh.rigidSkid:setParameter("c_tirPrpVolSkd", wh.tirePropertiesVolSkid)
    wh.rigidSkid:setParameter("c_tirPrpPitch", wh.tirePropertiesPitch)
    wh.rigidSkid:setParameter("c_tirPrpSoft", wd.softnessCoef)
    wh.looseRoll = createSoundObj("event:>Surfaces>roll_loose_v2", "AudioDefaultLoop3D", "looseRoll", wd.node1)
    wh.looseRoll:setParameter("c_tirPrpVolRol", wh.tirePropertiesVolRoll)
    wh.looseRoll:setParameter("c_tirPrpPitch", wh.tirePropertiesPitch)
    wh.looseRoll:setParameter("c_tirPrpSizTrd", wh.tirePropertiesSizeTread)
    wh.looseSkid = createSoundObj("event:>Surfaces>skid_loose_v2", "AudioDefaultLoop3D", "looseSkid", wd.node1)
    wh.looseSkid:setParameter("c_tirPrpVolSkd", wh.tirePropertiesVolSkid)
    wh.looseSkid:setParameter("c_tirPrpPitch", wh.tirePropertiesPitch)
    wh.looseSkid:setParameter("c_tirPrpSoft", wd.softnessCoef)
  end

  wheelsSounds[wheelID] = wh
end

local function loadSoundFiles(directory)
  --log('D', "sounds.loadSoundFiles", "loading sound files from: "..directory)
  local files = FS:findFiles(directory, "*.sbeam", -1, true, false)
  if not files or #files == 0 then
    --log('D', 'sounds.loadSoundFiles', 'unable to open directory for reading: ' .. directory)
    return
  end

  -- first: figure out all the filenames. TODO: recursive?
  local sbeamFiles = {}
  for _, file in ipairs(files) do
    table.insert(sbeamFiles, file)
  end

  --load and merge
  local soundBank = {}
  for _, sbfn in pairs(sbeamFiles) do
    local tmp = readDictJSONTable(sbfn)
    if tmp then
      for _, v in pairs(tmp.sounds) do
        v.minVolume = v.minVolume * sbeamVolumeFactor
        v.maxVolume = v.maxVolume * sbeamVolumeFactor
      end

      tableMergeRecursive(soundBank, tmp)
    else
      log("E", "sounds.lua", "sbeam file empty or unable to parse: " .. sbfn)
    end
  end

  -- fallback if no sounds were loaded
  if not soundBank.sounds then
    soundBank.sounds = {}
  end

  -- create lookup table
  if soundBank.modifiers then
    soundBank.modifiersNamed = {}
    for _, sbm in pairs(soundBank.modifiers) do
      soundBank.modifiersNamed[sbm.name] = sbm
    end
  end

  if type(soundBank.sounds) == "table" then
    --log('D', "sounds.loadSoundFiles", 'loaded '.. #soundBank.sounds .. ' sounds from directory ' .. directory)
  else
    log("D", "sounds.loadSoundFiles", "no sounds loaded from directory " .. directory)
    return nil
  end

  return soundBank
end

local function checkLocalFile(folder, file)
  if not FS:fileExists(file) then
    local testfn = folder .. file
    if FS:fileExists(testfn) then
      return testfn
    end
  end
  return file
end

local function getNextProfile()
  sfxprofilecounter = sfxprofilecounter + 1
  return "LuaSoundProfile" .. sfxprofilecounter .. "_" .. os.time()
end

local function bodyCollision(p)
  -- print((p.slipVel * p.normalForce * 0.001)..','..(p.slipVel * 0.01)..','..(p.normalForce * 0.01))
  local slipVel = p.slipVel
  if slipVel < 0.01 then
    return
  end
  local matid1, matid2 = p.materialID1, p.materialID2
  local event = scrapeMap[max(matid1, matid2)] or scrapeMap[min(matid1, matid2)]
  if not event then
    return
  end
  if scrapeAbsorbing[matid2] or scrapeAbsorbing[matid1] then
    if scrapeAbsorbing[matid2] == "wheel" or scrapeAbsorbing[matid1] == "wheel" then
      if v.data.nodes[p.id1].wheelID then
        return
      end
    else
      return
    end
  end
  local normalForce = p.normalForce
  obj:setNodeVolumePitchCT(event, p.id1, slipVel * normalForce / (normalForce + 6) * 8000, slipVel * 0.01, normalForce, max(M.scrapeLoosenessMap[matid1] or 0, M.scrapeLoosenessMap[matid2] or 0))
end

local function updateCabinFilter()
  obj:queueGameEngineLua(string.format("core_sounds.setCabinFilterStrength(%d, %f)", obj:getID(), clamp(cabinFilterCoef, 0, 1)))
  -- print(string.format("%d: Setting Cabin Filter -> %f", objectId, clamp(cabinFilterCoef, 0, 1)))
end

local function init()
  if not v.data.nodes then
    return
  end
  obj:deleteSFXSources()
  local cameraNode = 0
  if v.data.camerasInternal ~= nil then
    local _, c = next(v.data.camerasInternal)
    if c ~= nil then
      cameraNode = c.camNodeID
    end
  end

  --replace constants
  local maxrpm = 1
  M.refNode = 0
  M.engineNode = 0
  if v.data.refNodes and v.data.refNodes[0] then
    M.refNode = v.data.refNodes[0].ref or v.data.refNodes[0].leftCorner
    M.engineNode = v.data.refNodes[0].ref or v.data.refNodes[0].leftCorner
  end

  if #powertrain.engineData > 0 then
    for _, v in pairs(powertrain.engineData) do
      maxrpm = max(maxrpm, v.maxSoundRPM or 1)
    end

    -- Try to get a node on the engine. There is currently a libbeamng bug that
    -- causes sound sources at the camera to cause problems for multichannel
    -- setups, as small variations in position can cause sound to dither back
    -- and forth between the channels when in cockpit view.
    if powertrain.engineData[1].torqueReactionNodes then
      local t = powertrain.engineData[1].torqueReactionNodes
      if #t > 0 and v.data.nodes[t[1]] ~= nil then
        M.engineNode = t[1]
      end
    end
  end

  if not v.data.vehicleDirectory then
    return
  end

  local loadedFolder = v.data.vehicleDirectory .. "sounds/"

  --load sbeam files
  local sounds = loadSoundFiles(loadedFolder)
  M.usesOldCustomSounds = not tableIsEmpty(sounds)

  --no sbeam files on current vehicle, load defaults
  if not sounds then
    loadedFolder = "vehicles/common/sounds/"
    sounds = loadSoundFiles(loadedFolder)
  end

  if sounds then
    --store in module
    soundBank = sounds
    if usingNewEngineSounds then
      soundBank.sounds = {}
    end

    -- build node name index
    local nodeNameIdx = {}
    for _, node in pairs(v.data.nodes) do
      if node.name then
        nodeNameIdx[node.name] = node.cid
      end
    end

    --check and postprocess them
    for skey, s in pairs(soundBank.sounds) do
      -- set default values
      if s.volumeModifiers == nil then
        s.volumeModifiers = {}
      end
      if s.pitchModifiers == nil then
        s.pitchModifiers = {}
      end
      if s.profile == nil then
        s.profile = "AudioDefaultLoop3D"
      end

      -- create the sfxprofiles dynamically when filename and profile are specified
      if not s.sfxProfile and s.filename and s.profile then
        -- figure out if the filename was specified relative to the current folder
        s.filename = checkLocalFile(loadedFolder, s.filename)

        -- create the SFXProfile on the T3D - at least the supposed SFXprofilename
        s.sfxProfile = getNextProfile()
        s.waitforloading = 1 -- wait one frame before trying to load the sfxprofile
        s.autocreatedSFXProfile = true
      end

      --try to find our node, default to camera
      s.node = nodeNameIdx[s.nodeName]
      if s.nodeName == "CAMERA" then
        s.node = cameraNode
      end
      if s.nodeName == "ENGINE" then
        s.node = M.engineNode
      end
      s.node = s.node or cameraNode -- fall back to camera node

      s.clip = createSoundObj(s.filename, s.profile, s.sfxProfile, s.node)
      --log('D', 'sounds.update', 'createSFXSource('..s.sfxProfile..','..s.node..') = '..tostring(s.clip))
      if not s.clip then
        log("W", "sounds.update", "unable to create sound, removing it: " .. s.sfxProfile)
        soundBank.sounds[skey] = nil
      end
    end

    for _, snd in pairs(soundBank.sounds) do
      for k2, v2 in pairs(snd) do
        if v2 == "MAXRPM" then
          snd[k2] = maxrpm
        end
      end
    end

    --initialize groups
    local soundGroup = v.data.engine and v.data.engine.soundGroup
    for _, vl in pairs(soundBank.sounds) do
      vl.active = (vl.group == "default" or vl.group == soundGroup)
    end

    beamResetTimer = 0

    --initialize per beam suspension sounds
    if v.data.beams then
      for _, bm in pairs(v.data.beams) do
        if bm.soundFile ~= nil then
          local soundTable = {}

          --loop
          local soundProfileType = "AudioDefaultLoop3D"

          --setup our table
          local soundFile = checkLocalFile(v.data.vehicleDirectory, bm.soundFile)
          soundTable.soundType = bm.soundType
          soundTable.sfxProfile = getNextProfile()
          soundTable.clip = createSoundObj(soundFile, soundProfileType, soundTable.sfxProfile, bm.id1)
          if soundTable.clip then
            soundTable.volumeFactor = bm.volumeFactor or 1
            soundTable.pitchFactor = bm.pitchFactor or 0
            soundTable.maxStress = bm.maxStress or 35000
            soundTable.colorFactor = bm.colorFactor or 0.5

            soundTable.beam = bm.cid

            local attackFactor = bm.attackFactor or 10
            local decayFactor = bm.decayFactor or 10

            soundTable.smoothing = newTemporalSmoothingNonLinear(decayFactor, attackFactor) --first value = decay smoothing, second value = attack smoothing. The smaller the number, the more the smoothing

            soundTable.resonance = 0
            soundTable.clip:setVolumePitch(0, 0)
            soundTable.noiseFactor = bm.noiseFactor or 0
            soundTable.noiseTrail = 0

            -- Audio debug - display the current suspension setup from jbeam
            -- print (string.format("Suspension : %s color=%.2f  attack=%.0f  volume=%.2f  decay=%.0f  pitch=%.2f  maxStress=%.0f", bm.soundFile, bm.colorFactor, bm.attackFactor, bm.volumeFactor, bm.decayFactor, bm.pitchFactor, bm.maxStress))
            -- print (string.format("Suspension : %s color=%s  attack=%s  volume=%s  decay=%s  pitch=%s  maxStress=%s", dumps(bm.soundFile), dumps(bm.colorFactor), dumps(bm.attackFactor), dumps(bm.volumeFactor), dumps(bm.decayFactor), dumps(bm.pitchFactor), dumps(bm.maxStress)))

            -- finally, insert it for graphing
            table.insert(beamSounds, soundTable)
          else
            log("E", "sounds.init", "unable to load sound: " .. tostring(soundFile))
          end
        end
      end
    else
      log("E", "sounds.init", "unable to load any sound bank (*.sbeam), that is quite bad :/")
    end
  end

  cabinFilterCoef = 1
  windSoundEventNode = getSoundscapeNode("horn") or M.refNode or 0
  if v.data.sounds then
    --impacts
    impactGenericEvent = v.data.sounds.impactGeneric == nil and impactGenericEvent or v.data.sounds.impactGeneric
    impactMetalEvent = v.data.sounds.impactMetal == nil and impactMetalEvent or v.data.sounds.impactMetal
    impactPlasticEvent = v.data.sounds.impactPlastic == nil and impactPlasticEvent or v.data.sounds.impactPlastic
    impactSoundVolumeCoef = v.data.sounds.impactSoundVolumeCoef == nil and impactSoundVolumeCoef or v.data.sounds.impactSoundVolumeCoef
    --break
    breakGenericEvent = v.data.sounds.breakGeneric == nil and breakGenericEvent or v.data.sounds.breakGeneric
    --aero
    windSoundEvent = v.data.sounds.wind == nil and windSoundEvent or v.data.sounds.wind
    windSoundEventNode = (v.data.sounds.windNode and beamstate.nodeNameMap[v.data.sounds.windNode]) or windSoundEventNode
    --scrapes
    scrapeMap[2] = v.data.sounds.scrapeMetal == nil and scrapeMap[2] or v.data.sounds.scrapeMetal
    scrapeMap[3] = v.data.sounds.scrapePlastic == nil and scrapeMap[3] or v.data.sounds.scrapePlastic

    cabinFilterCoef = v.data.sounds.cabinFilterCoef == nil and 1 or v.data.sounds.cabinFilterCoef
  end

  for matId, event in pairs(scrapeMap) do
    if type(event) == "string" then --this can be nil (if nothing exists for a given material) or a boolean/false (if disabled via jbeam)
      if scrapeSounds[event] == nil then
        scrapeSounds[event] = obj:createSFXSource(event, "AudioDefaultLoop3D", "Scrape", -2)
      end
      scrapeMap[matId] = scrapeSounds[event]
    end
  end

  if wheelsSounds == nil then
    wheelsSounds = {}

    for wi, wd in pairs(wheels.wheels) do
      addWheelSounds(wi, wd)
    end
  end

  local soundscapes = v.data.soundscape
  if soundscapes then
    for name, soundscape in pairs(soundscapes) do
      bdebug.setNodeDebugText("Soundscape", type(soundscape.node) == "number" and soundscape.node or M.refNode, name .. ": " .. soundscape.src)
    end
  end

  M.updateObjType()
end

-- this function enables or disables the data reporting to the UI. It is quite performance heavy and should be only used for debugging
local function setUIDebug(enabled, data)
  M.uiDebugging = enabled
  -- todo: save data
end

local function onDeserialized()
  M.objType = -1
end

local function disableOldEngineSounds()
  soundBank.sounds = {}
  usingNewEngineSounds = true
end

local fmodtable = {20.0, 40.0, 80.0, 160.0, 330.0, 660.0, 1300.0, 2700.0, 5400.0, 11000.0, 22000.0} --Hz values
-- IMPORTANT: code also present in engine/audio/backends/fmod/trueforceDSP.cpp. Make sure you sync LUA changes to the C++ port
local function hzToFMODHz(hzValue)
  local range = #fmodtable - 1
  hzValue = max(fmodtable[1], min(hzValue, fmodtable[#fmodtable])) --clamp hzValue to min/max possible values
  for i = range, 1, -1 do --iterate all fmod hz entries starting at the top
    if fmodtable[i] <= hzValue then --if we found an fmod hz value smaller than our target, set that as our range
      range = i
      break
    end
  end
  return 100 * ((range - 1) + ((hzValue - fmodtable[range]) / (fmodtable[range + 1] - fmodtable[range])))
end

local reverseFmodTable = {0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000}
-- IMPORTANT: code also present in engine/audio/backends/fmod/trueforceDSP.cpp. Make sure you sync LUA changes to the C++ port
local function FMODHzToHz(fmodHzValue)
  local range = #reverseFmodTable - 1
  fmodHzValue = max(reverseFmodTable[1], min(fmodHzValue, reverseFmodTable[#reverseFmodTable]))
  for i = range, 1, -1 do
    if reverseFmodTable[i] <= fmodHzValue then
      range = i
      break
    end
  end
  return (((fmodHzValue / 100) - (range - 1)) * (fmodtable[range + 1] - fmodtable[range])) + fmodtable[range]
end

local function reset()
  beamResetTimer = 0

  if windSound then
    windSound:setVolumePitch(0, 0)
  end

  for _, snd in ipairs(beamSounds) do
    snd.resonance = 0
    snd.smoothing:reset()
    snd.clip:setVolumePitch(0, 0)
    snd.noiseTrail = 0
  end

  --resend all static tire properties upon reset because the GE param cache is cleared upon vehicle reset
  for wi, wh in pairs(wheelsSounds) do
    local wd = wheels.wheels[wi]
    if wd.tireSoundVolumeCoef > 0 then
      wh.rigidRoll:setParameter("c_tirPrpVolRol", wh.tirePropertiesVolRoll)
      wh.rigidRoll:setParameter("c_tirPrpPitch", wh.tirePropertiesPitch)
      wh.rigidRoll:setParameter("c_tirPrpSizTrd", wh.tirePropertiesSizeTread)

      wh.rigidSkid:setParameter("c_tirPrpVolSkd", wh.tirePropertiesVolSkid)
      wh.rigidSkid:setParameter("c_tirPrpPitch", wh.tirePropertiesPitch)
      wh.rigidSkid:setParameter("c_tirPrpSoft", wd.softnessCoef)

      wh.looseRoll:setParameter("c_tirPrpVolRol", wh.tirePropertiesVolRoll)
      wh.looseRoll:setParameter("c_tirPrpPitch", wh.tirePropertiesPitch)
      wh.looseRoll:setParameter("c_tirPrpSizTrd", wh.tirePropertiesSizeTread)

      wh.looseSkid:setParameter("c_tirPrpVolSkd", wh.tirePropertiesVolSkid)
      wh.looseSkid:setParameter("c_tirPrpPitch", wh.tirePropertiesPitch)
      wh.looseSkid:setParameter("c_tirPrpSoft", wd.softnessCoef)
    end
  end

  M.updateObjType()
end

local function updateObjType()
  local newObjType = playerInfo.anyPlayerSeated and 0 or (ai.mode == "traffic" and 2 or 1)
  if newObjType == M.objType then
    return
  end
  M.objType = newObjType

  for wi, wh in pairs(wheelsSounds or {}) do
    if wh.rigidRoll then
      wh.rigidRoll:setParameter("c_objType", newObjType)
    end
    if wh.rigidSkid then
      wh.rigidSkid:setParameter("c_objType", newObjType)
    end
    if wh.looseRoll then
      wh.looseRoll:setParameter("c_objType", newObjType)
    end
    if wh.looseSkid then
      wh.looseSkid:setParameter("c_objType", newObjType)
    end
  end
end

-- Debug
local function getBeamSounds()
  return beamSounds
end

local function getEngineSoundData()
  --windowOpen[0] = true
  local engineSoundData = {}
  local engineNameStrings = {}

  local engines = powertrain.getDevicesByCategory("engine")
  local count = 0
  for index, engine in ipairs(engines) do
    if engine.getSoundConfiguration then
      local soundConfig = engine:getSoundConfiguration()
      if soundConfig then
        for name, data in pairs(soundConfig) do
          local blendFileName = data.blendFile:match("^.+/(.+)$")
          local configName = string.format("%s (%s) -> %s", name, engine.name, blendFileName)
          engineSoundData[count] = {
            reference = name,
            data = data,
            fundamentalFrequencyRPMCoef = engine.fundamentalFrequencyRPMCoef,
            engineIndex = index
          }
          table.insert(engineNameStrings, configName)
          count = count + 1
        end
      end
    end
  end
  return engineSoundData, engineNameStrings
end

-- public interface
M.updateGFX = updateGFX
M.playSoundOnceAtNode = playSoundOnceAtNode
M.playSoundOnceFollowNode = playSoundOnceFollowNode
M.init = init
M.reset = reset
M.setUIDebug = setUIDebug
M.createSFXSource = createSFXSource
M.onDeserialized = onDeserialized --this enables serialization of all M. values for the module so they survive reloads
M.disableOldEngineSounds = disableOldEngineSounds
M.hzToFMODHz = hzToFMODHz
M.FMODHzToHz = FMODHzToHz
M.createSoundscapeSound = createSoundscapeSound
M.playSoundSkipAI = playSoundSkipAI
M.bodyCollision = bodyCollision
M.createSoundObj = createSoundObj
M.updateObjType = updateObjType

-- Debug
M.getBeamSounds = getBeamSounds
M.getEngineSoundData = getEngineSoundData

M.updateCabinFilter = updateCabinFilter

return M
