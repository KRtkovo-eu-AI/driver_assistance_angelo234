-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

local im = ui_imgui

local continuousDriftSoundId

local firstOnUpdate = true

local isSoundPlaying = {}

local driftDebugInfo = {
  default = false,
  canBeChanged = true
}

local soundAssets = {
  continuousDrift = "event:>UI>Career>Drift_Counting",
  newTier = "event:>UI>Career>Drift_Tier",
  newCombo1 = "event:>UI>Career>Drift_Combo_1x",
  newCombo5 = "event:>UI>Career>Drift_Combo_5x",
  driftCanceled = "event:>UI>Career>Drift_Canceled",
  pointsConfirmed = "event:>UI>Career>Drift_PointsReceived",
}


-- audio debug test stuff, simulate a drift
local simulateDriftPtr = im.BoolPtr(false)
local driftSpeedPtr = im.FloatPtr(20)
local driftAnglePtr = im.FloatPtr(40)
local wallDistancePtr = im.FloatPtr(3)
local currentlyDoingATransition = false
local driftTransitionTime = 0.2
local driftTransitionTimer = 0
local dtSim
local isCrashing = false
local function setupContinuousDriftSound()
  continuousDriftSoundId = continuousDriftSoundId or Engine.Audio.createSource('AudioGui', soundAssets.continuousDrift)
end

local function updateContinuousDriftSoundParameters()
  local sound = scenetree.findObjectById(continuousDriftSoundId)
  if sound then
    sound:setParameter("pitch", gameplay_drift_scoring.getDriftPerformanceFactor())
    sound:setTransform(getCameraTransform())
  end
end

local function activateContinuousDriftSound(soundId, active)
  if not soundId then return end

  local sound = scenetree.findObjectById(soundId)
  if sound and (isSoundPlaying[soundId] == not active or not isSoundPlaying[soundId]) then
    if active then
      sound:play(-1)
    else
      sound:stop(-1)
    end
    sound:setTransform(getCameraTransform())
    isSoundPlaying[soundId] = active
  end
end


local function getDriftDebugInfo()
  return driftDebugInfo
end

local function imguiDebug()
  if gameplay_drift_general.getExtensionDebug("gameplay_drift_sounds") then
    -- simulate a transition from left to right or vice versa
    if currentlyDoingATransition then
      driftTransitionTimer = driftTransitionTimer + dtSim
      if driftTransitionTimer > driftTransitionTime then
        currentlyDoingATransition = false
        driftTransitionTimer = 0
      end
    end

    -- simulate a crash
    if isCrashing then
      gameplay_drift_drift.simulateADrift()
      isCrashing = false
      simulateDriftPtr[0] = false
    end

    if im.Begin("Drift audio") then
      im.PushStyleColor2(im.Col_Text, im.ImVec4(1, 0.1, 0.1, 1))
        im.TextWrapped("-This is *not* a 1 to 1 simulation, many things are wrong.")
      im.PopStyleColor()
      im.Dummy(im.ImVec2(1, 10))

      im.Checkbox("Simulate drift", simulateDriftPtr)

      if simulateDriftPtr[0] and not currentlyDoingATransition then
        im.SliderFloat("Drift speed", driftSpeedPtr, 10, 200, "%.2f")
        im.SliderFloat("Drift angle", driftAnglePtr, 10, 130, "%.2f")
        im.SliderFloat("Wall distance", wallDistancePtr, 1, 3, "%.2f")

        if im.Button("Do a drift transition") then
          currentlyDoingATransition = true
        end

        if im.Button("Simulate a crash") then
          isCrashing = true
        end

        if im.Button("Trigger a new tier") then
          extensions.hook("onNewDriftTierReached", { minScore =     0, continuousScore = 10, id = "drift", order = 1, name = "A test tier!" })
        end

        gameplay_drift_drift.simulateADrift({
          currDegAngle = driftAnglePtr[0],
          airSpeed = driftSpeedPtr[0],
          wallDistance = wallDistancePtr[0],
          isCrashing = isCrashing
        })
      else
        gameplay_drift_drift.simulateADrift()
      end
    end
  end
end

local function playContinuousDriftSound()
  activateContinuousDriftSound(continuousDriftSoundId, gameplay_drift_drift.getIsDrifting())
  if isSoundPlaying[continuousDriftSoundId] then
    updateContinuousDriftSoundParameters()
  end
end

local function onUpdate(dtReal, _dtSim, dtRaw)
  dtSim = _dtSim

  if firstOnUpdate then
    setupContinuousDriftSound()
    firstOnUpdate = false
  end
  imguiDebug()

  -- disable sound if drift system if paused or no UI app
  if gameplay_drift_general.getPaused() or gameplay_drift_general.getFrozen() then
    activateContinuousDriftSound(continuousDriftSoundId, false)
  else
    playContinuousDriftSound()
  end

end

local function onNewDriftTierReached(tierData)
  Engine.Audio.playOnce('AudioGui', soundAssets.newTier, {volume = tierData.order - 1})
end



local function onDriftNewCombo(data)
  if data.comboChange <= 0.2 then
    Engine.Audio.playOnce('AudioGui', soundAssets.newCombo1)
  else
    Engine.Audio.playOnce('AudioGui', soundAssets.newCombo5)
  end
end


local function playDriftCanceledSound()
  Engine.Audio.playOnce('AudioGui', soundAssets.driftCanceled)
end

local function onDriftCrash()
  playDriftCanceledSound()
end

local function onDriftSpinout()
  playDriftCanceledSound()
end

local function onDriftDebugChanged(value)
  if not value then
    gameplay_drift_drift.simulateADrift() -- stop simulating a drift
  end
end

local function onDriftCompletedScored(data)
  Engine.Audio.playOnce('AudioGui', soundAssets.pointsConfirmed)
end

local function reset()
  activateContinuousDriftSound(continuousDriftSoundId, false)
  isSoundPlaying = {}
end

local function onExtensionUnloaded()
  activateContinuousDriftSound(continuousDriftSoundId, false)
end

M.onUpdate = onUpdate
M.onExtensionUnloaded = onExtensionUnloaded

M.getDriftDebugInfo = getDriftDebugInfo

M.onNewDriftTierReached = onNewDriftTierReached
M.onDriftNewCombo = onDriftNewCombo
M.onDriftCrash = onDriftCrash
M.onDriftSpinout = onDriftSpinout
M.onDriftDebugChanged = onDriftDebugChanged
M.onDriftCompletedScored = onDriftCompletedScored

return M