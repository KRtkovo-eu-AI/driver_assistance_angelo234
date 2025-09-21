-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local forwardSound
local reverseSound

local maxSpeed = 50 / 3.6
local warningVolume

local forwardModes = {D = true, S = true}
local reverseModes = {R = true}
local currentState

local function updateGFX(dt)
  local gear = electrics.values.gear
  local speed = electrics.values.wheelspeed or 0

  local desiredState = "stopped"
  if forwardModes[gear] then
    desiredState = "playingForward"
  elseif reverseModes[gear] then
    desiredState = "playingReverse"
  end

  if (desiredState == "playingForward" or desiredState == "playingReverse") and speed > (maxSpeed + 1) then
    desiredState = "stopped"
  end

  if desiredState ~= currentState then
    if desiredState == "playingForward" then
      obj:playSFX(forwardSound)
      obj:stopSFX(reverseSound)
    elseif desiredState == "playingReverse" then
      obj:playSFX(reverseSound)
      obj:stopSFX(forwardSound)
    elseif desiredState == "stopped" then
      obj:stopSFX(forwardSound)
      obj:stopSFX(reverseSound)
    end
    currentState = desiredState
  end

  local speedCoef = linearScale(speed, 0, maxSpeed, 0, 1)
  obj:setVolumePitchCT(forwardSound, warningVolume, 1, speedCoef, 0)
  obj:setVolumePitchCT(reverseSound, warningVolume, 1, speedCoef, 0)
end

local function reset()
  obj:stopSFX(forwardSound)
  obj:stopSFX(reverseSound)
  currentState = "stopped"
end

local function init(jbeamData)
end

local function initSounds(jbeamData)
  local forwardEvent = jbeamData.forwardSoundEvent or "event:>Engine>Pedestrian Warning>Forward_01"
  local reverseEvent = jbeamData.reverseSoundEvent or "event:>Engine>Pedestrian Warning>Reverse_01"
  local forwardSoundNode = jbeamData.forwardSoundNode_nodes and jbeamData.forwardSoundNode_nodes[1] or 0
  local reverseSoundNode = jbeamData.reverseSoundNode_nodes and jbeamData.reverseSoundNode_nodes[1] or 0
  forwardSound = obj:createSFXSource2(forwardEvent, "AudioDefaultLoop3D", "avasForward", forwardSoundNode, 0)
  reverseSound = obj:createSFXSource2(reverseEvent, "AudioDefaultLoop3D", "avasReverse", reverseSoundNode, 0)

  maxSpeed = jbeamData.warningMaxSpeed or (50 / 3.6)
  warningVolume = jbeamData.warningVolume or 0.5
end

M.init = init
M.initSounds = initSounds
M.reset = reset
M.updateGFX = updateGFX

return M
