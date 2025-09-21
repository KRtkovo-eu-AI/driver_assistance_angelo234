-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local abs = math.abs

local hasSegments
local hasEndstops
local hasPlayedStartSound
local hasPlayedEndSound

local movementSound
local soundNode
local segmentIncSoundEvent
local segmentDecSoundEvent
local minPositionSoundEvent
local maxPositionSoundEvent
local positionSensorBeamCid
local positionSensorBeamMinPosition
local positionSensorBeamMaxPosition
local movementLength
local segmentLength
local lastPositionSensorBeamLength
local movementSoundVolume
local isPlaying = false
local movementVelocitySmoother
local absMovementVelocitySmoother
local lastSegmentMod

local positionCalibrationOffset = -0.3
local movementSoundPitchMinVelocity = 0
local movementSoundPitchMaxVelocity = 2
local movementSoundVolumeCoefMinVelocity = 0.05
local movementSoundVolumeCoefMaxVelocity = 0.1
local movementSoundMinPitch = 0
local movementSoundMaxPitch = 1
local movementSoundPlayingThresholdVelocity = 0.05
local movementSoundPlayingThresholdVolume = 0.05

local printDebug

local updateMovementSoundFunction
local updateSegmentSoundFunction
local updateEndstopsSoundFunction

local function updateMovementSound(positionSensorBeamLength, movementVelocity, movementDirection, volume, pitch, dt)
  obj:setVolumePitchCT(movementSound, volume, pitch, movementDirection, 0)

  --handle starting and stopping the sound, we don't want to start it if the velocity is really low or the volume is barely audible
  if movementVelocity >= movementSoundPlayingThresholdVelocity and volume > movementSoundPlayingThresholdVolume then
    if not isPlaying then
      obj:playSFX(movementSound)
      isPlaying = true
    end
  else
    if isPlaying then
      obj:stopSFX(movementSound)
      isPlaying = false
    end
  end

  lastPositionSensorBeamLength = positionSensorBeamLength
end

local function updateSegmentSound(positionSensorBeamLength, volume, pitch, dt)
  --if we have a segmented part to this sound, calculate it
  --calculate a position starting at 0 and consider the calibration offset
  local adjustedPosition = positionSensorBeamLength - positionSensorBeamMinPosition + positionCalibrationOffset
  --we want to calculate the modulus to detect whenever we went past one segment length of travel
  local segmentMod = adjustedPosition % segmentLength
  --see how much the mod changed since the last frame
  local segmentModDelta = lastSegmentMod - segmentMod
  --adjust the segment event based on movement direction
  local segmentSoundEvent = sign(segmentModDelta) > 0 and segmentIncSoundEvent or segmentDecSoundEvent
  --if our delta is suddenly very large, we jumped to the next segment and want to emit a sound
  --usually it's very small frame-to-frame, so we just check against half the segment length as the trigger
  if segmentSoundEvent and abs(segmentModDelta) > segmentLength * 0.5 then
    --color == 1 is the trigger for fmod to play the segment part on top of the noise part
    obj:playSFXOnceCT(segmentSoundEvent, soundNode, volume, pitch, 0, 0)
  end
  --remember the last modulus for the next frame
  lastSegmentMod = segmentMod
end

local function updateEndstopsSound(positionSensorBeamLength, volume, pitch, dt)
  --only handle endstop sounds if we need them
  --only play the start sound once and at the start position
  if not hasPlayedStartSound and positionSensorBeamLength <= positionSensorBeamMinPosition and minPositionSoundEvent then
    --trigger the start sound
    obj:playSFXOnceCT(minPositionSoundEvent, soundNode, volume, pitch, 0, 0)
    --prevent further start sounds from playing
    hasPlayedStartSound = true
  end

  --only play the end sound once and at the end position
  if not hasPlayedEndSound and positionSensorBeamLength >= positionSensorBeamMaxPosition and maxPositionSoundEvent then
    --trigger the end sound
    obj:playSFXOnceCT(maxPositionSoundEvent, soundNode, volume, pitch, 0, 0)
    --prevent further end sounds from playing
    hasPlayedEndSound = true
  end

  --if we played the start sound and moved far enough away from the start position, reset it
  if hasPlayedStartSound and positionSensorBeamLength > positionSensorBeamMinPosition + movementLength * 0.05 then
    hasPlayedStartSound = false
  end
  --if we played the end sound and moved far enough away from the end position, reset it
  if hasPlayedEndSound and positionSensorBeamLength < positionSensorBeamMaxPosition - movementLength * 0.05 then
    hasPlayedEndSound = false
  end
end

local function updateGFX(dt)
  --get the current length of our sensor beam
  local positionSensorBeamLength = obj:getBeamLength(positionSensorBeamCid)
  --calculate a smoothed velocity of that beam length
  local smoothDirectionalMovementVelocity = movementVelocitySmoother:get((positionSensorBeamLength - lastPositionSensorBeamLength) / dt, dt)
  local movementVelocity = absMovementVelocitySmoother:get(abs(smoothDirectionalMovementVelocity), dt)
  local movementDirection = sign(smoothDirectionalMovementVelocity)
  --calculate the desird pitch based on the beam length velocity
  local pitch = linearScale(movementVelocity, movementSoundPitchMinVelocity, movementSoundPitchMaxVelocity, movementSoundMinPitch, movementSoundMaxPitch)
  --calculate our volume based on the base volume from jbeam and a little fade coef based on the beam length velocity
  local volume = movementSoundVolume * linearScale(movementVelocity, movementSoundVolumeCoefMinVelocity, movementSoundVolumeCoefMaxVelocity, 0, 1)

  updateMovementSoundFunction(positionSensorBeamLength, movementVelocity, movementDirection, volume, pitch, dt)
  updateSegmentSoundFunction(positionSensorBeamLength, volume, pitch, dt)
  updateEndstopsSoundFunction(positionSensorBeamLength, volume, pitch, dt)

  if printDebug then
    print(string.format("%s: position: %.2f, velocity: %.2f, volume: %.2f, pitch: %.2f, isPlaying: %s", M.name, positionSensorBeamLength, movementVelocity, volume, pitch, isPlaying))
  end
end

local function initEndstops()
  --if we use endstops, init the endstop sound play variables according to the current position
  if hasEndstops then
    local positionSensorBeamLength = obj:getBeamLength(positionSensorBeamCid)
    if positionSensorBeamLength <= positionSensorBeamMinPosition then
      --prevent further start sounds from playing
      hasPlayedStartSound = true
    end

    --only play the end sound once and at the end position
    if positionSensorBeamLength >= positionSensorBeamMaxPosition then
      --prevent further end sounds from playing
      hasPlayedEndSound = true
    end
  end
end

local function updateSoundMethods()
  --check which functionality we actually need and update our methods accordingly
  updateMovementSoundFunction = movementSound and updateMovementSound or nop
  updateSegmentSoundFunction = hasSegments and updateSegmentSound or nop
  updateEndstopsSoundFunction = hasEndstops and updateEndstopsSound or nop
end

local function resetSounds(jbeamData)
  if movementSound then
    obj:stopSFX(movementSound)
  end
  initEndstops()
end

local function reset(jbeamData)
  lastPositionSensorBeamLength = obj:getBeamLength(positionSensorBeamCid)
  lastSegmentMod = 0
  movementVelocitySmoother:reset()
  absMovementVelocitySmoother:reset()
end

local function initSounds(jbeamData)
  --base volume for both the noise part and the segment/endstop sounds
  movementSoundVolume = jbeamData.movementSoundVolume or 1
  --event for the overall movement sound, this includes the constant part, the segment part and the endstops
  local movementSoundEvent = jbeamData.movementSoundEvent
  segmentIncSoundEvent = jbeamData.segmentIncSoundEvent or jbeamData.segmentSoundEvent
  segmentDecSoundEvent = jbeamData.segmentDecSoundEvent or jbeamData.segmentSoundEvent
  minPositionSoundEvent = jbeamData.minPositionSoundEvent
  maxPositionSoundEvent = jbeamData.maxPositionSoundEvent
  --a single node where the sound is emitted
  soundNode = jbeamData.movementSoundNode_nodes and jbeamData.movementSoundNode_nodes[1] or 0
  if movementSoundEvent then
    movementSound = obj:createSFXSource2(movementSoundEvent, "AudioDefaultLoop3D", "segmentedMovement." .. M.name, soundNode, 0)
  end

  --the velocity at which the minimum pitch applies
  movementSoundPitchMinVelocity = jbeamData.movementSoundPitchMinVelocity or 0
  --the velocity at which the maximum pitch applies
  movementSoundPitchMaxVelocity = jbeamData.movementSoundPitchMaxVelocity or 2

  --the pitch at minimum velocity
  movementSoundMinPitch = jbeamData.movementSoundMinPitch or 0
  --the pitch at maximum velocity
  movementSoundMaxPitch = jbeamData.movementSoundMaxPitch or 1

  --movement velocity that scale between 0 and full volume
  movementSoundVolumeCoefMinVelocity = jbeamData.movementSoundVolumeCoefMinVelocity or 0.05
  movementSoundVolumeCoefMaxVelocity = jbeamData.movementSoundVolumeCoefMaxVelocity or 0.1

  --threshold for velocity and volume to start/stop the sound
  movementSoundPlayingThresholdVelocity = jbeamData.movementSoundPlayingThresholdVelocity or 0.05
  movementSoundPlayingThresholdVolume = jbeamData.movementSoundPlayingThresholdVolume or 0.05

  if movementSoundVolumeCoefMinVelocity <= 0 then
    log("E", "linearMovement.initSounds", "'movementSoundVolumeCoefMinVelocity' can't be lower or equal to 0, disabling system... Actual value: " .. movementSoundVolumeCoefMinVelocity)
    M.updateGFX = nop
  end

  if movementSoundPlayingThresholdVelocity <= 0 then
    log("E", "linearMovement.initSounds", "'movementSoundPlayingThresholdVelocity' can't be lower or equal to 0, disabling system... Actual value: " .. movementSoundPlayingThresholdVelocity)
    M.updateGFX = nop
  end

  if movementSoundPlayingThresholdVolume <= 0 then
    log("E", "linearMovement.initSounds", "'movementSoundPlayingThresholdVolume' can't be lower or equal to 0, disabling system... Actual value: " .. movementSoundPlayingThresholdVolume)
    M.updateGFX = nop
  end

  bdebug.setNodeDebugText("Linear Movement", soundNode, M.name .. " - Loop: " .. (movementSoundEvent or "no event"))
  bdebug.setNodeDebugText("Linear Movement", soundNode, M.name .. " - Segment Inc: " .. (segmentIncSoundEvent or "no event"))
  bdebug.setNodeDebugText("Linear Movement", soundNode, M.name .. " - Segment Dec: " .. (segmentDecSoundEvent or "no event"))
  bdebug.setNodeDebugText("Linear Movement", soundNode, M.name .. " - Position Min: " .. (minPositionSoundEvent or "no event"))
  bdebug.setNodeDebugText("Linear Movement", soundNode, M.name .. " - Position Max: " .. (maxPositionSoundEvent or "no event"))

  initEndstops()
  updateSoundMethods()
end

local function init(jbeamData)
  --used when the sound has a segment sound in addition to the base noise
  --color value for segment trigger: 0.5
  hasSegments = jbeamData.hasSegments or false
  --used when the sound has endstop sounds in addition to the base noise, the event itself can define  if one or both exist
  --color values for endstop triggers: start: 0.1, end: 0.9
  hasEndstops = jbeamData.hasEndstops or false
  local positionSensorBeamTag = jbeamData.positionSensorBeamTag or ""
  local positionSensorBeamCids = beamstate.tagBeamMap[positionSensorBeamTag]

  if not positionSensorBeamCids or not positionSensorBeamCids[1] then
    log("E", "linearMovement.init", "Can't find position sensor beam with tag: " .. positionSensorBeamTag)
    return
  end
  positionSensorBeamCid = positionSensorBeamCids[1]

  segmentLength = jbeamData.segmentLength
  positionSensorBeamMinPosition = jbeamData.minPosition or 0
  positionSensorBeamMaxPosition = jbeamData.maxPosition or 0
  movementLength = abs(positionSensorBeamMaxPosition - positionSensorBeamMinPosition)

  positionCalibrationOffset = jbeamData.positionCalibration or 0

  lastPositionSensorBeamLength = obj:getBeamLength(positionSensorBeamCid)
  local velocitySmoothing = jbeamData.velocitySmoothing or 5
  movementVelocitySmoother = newTemporalSmoothing(velocitySmoothing, velocitySmoothing)

  local absVelocitySmoothing = jbeamData.absVelocitySmoothing or 5
  local absVelocitySmoothingIn = jbeamData.absVelocitySmoothingIn or absVelocitySmoothing
  local absVelocitySmoothingOut = jbeamData.absVelocitySmoothingOut or absVelocitySmoothing
  absMovementVelocitySmoother = newTemporalSmoothing(absVelocitySmoothingIn, absVelocitySmoothingOut)
  lastSegmentMod = 0

  printDebug = jbeamData.debugMode or false

  M.updateGFX = updateGFX
end

M.init = init
M.initSounds = initSounds

M.reset = reset
M.resetSounds = resetSounds

M.updateGFX = nop

return M
