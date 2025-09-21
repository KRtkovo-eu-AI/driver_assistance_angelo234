-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

M.dependencies = {"gameplay_drift_general", "gameplay_drift_drift", "gameplay_drift_scoring", "ui_apps_genericMissionData"}
local im = ui_imgui
local driftDebugInfo = {
  default = false,
  canBeChanged = true
}

local flashTime = 1.5
local msgData = {}
local score

local wrongWayFlag = false
local outOfBoundsFlag = false
local creepSmoother = newTemporalSmoothingNonLinear(50,10, 0)

local driftDebugUILayout = false

local function rtMessage(msg)
  msgData.msg = msg
  msgData.context = "drift"
  guihooks.trigger('ScenarioRealtimeDisplay', msgData)
end

local function clearRt()
  table.clear(msgData)
  msgData.msg = ""
  guihooks.trigger('ScenarioRealtimeDisplay', msgData)
end

local function flashMessage(msg, duration)
  duration = duration or flashTime

  guihooks.trigger('DriftFlashMessage', {{msg, flashTime, 0, false}} )
end

local function onDriftCompletedScored(data)
  guihooks.trigger("setDriftRemainingComboTime", 0)
  guihooks.trigger("setDriftRealtimeCreep", 0)
  creepSmoother:set(0)

  flashMessage(string.format("+ %i points", data.addedScore))
  guihooks.trigger("setDriftPersistentDriftScored", data.addedScore, data.combo)
  clearRt()
end

local function onDriftCrash()
  guihooks.trigger("setDriftRealtimeFail", "Crashed!")
  clearRt()
end

local function onDriftSpinout()
  guihooks.trigger("setDriftRealtimeFail", "Spinout!")
  clearRt()
end

local function onDonutDriftScored(score)
  flashMessage(string.format("Donut! + %i points", score))
  guihooks.trigger("stuntZoneScored",{type = "donut", score = score})
end

local function onNearPoleScored(score)
  flashMessage(string.format("Near pole drift! + %i points", score))
  guihooks.trigger("stuntZoneScored",{type = "nearPole", score = score})
end

local function onTightDriftScored(score)
  flashMessage(string.format("Drift through! + %i points", score))
  guihooks.trigger("stuntZoneScored",{type = "tightDrift", score = score})
end

local function onHitPoleScored(score)
  flashMessage(string.format("Pole hit! + %i points", score))
  guihooks.trigger("stuntZoneScored",{type = "hitPole", score = score})
end


local function updateApps(dtReal)
  if not gameplay_drift_general.getMainDriftUIAppLoaded() then return end

  score = gameplay_drift_scoring.getScore()
  local scoreOptions = gameplay_drift_scoring.getScoreOptions()

  guihooks.trigger("setDriftPermanentAndPotentialScore", score.score, score.potentialScore)

  if score.cachedScore > 0 then
    guihooks.trigger("setDriftRealtimeScore", math.floor(score.cachedScore), score.combo)
    guihooks.trigger("setDriftRemainingComboTime", gameplay_drift_drift.getCurrDriftCompletedTime())
    local creepPercent = score.comboCreepup / 100
    if score.combo >= scoreOptions.comboOptions.comboSoftCap then
      creepPercent = 1
    end

    guihooks.trigger("setDriftRealtimeCreep", creepSmoother:get(creepPercent, dtReal))
  end
  guihooks.trigger("setDriftPerformanceFactor", gameplay_drift_scoring.getSteppedDriftPerformanceFactor())

  local airspeed =  gameplay_drift_drift.getAirSpeed() or 0
  local angle = gameplay_drift_drift.getCurrDegAngleSigned() or 0
  local isDrifting = gameplay_drift_drift.getIsDrifting()


  if math.abs(angle) < gameplay_drift_drift.getDriftOptions().maxAngle and gameplay_drift_drift.getIsOverSteering() and gameplay_drift_drift.getIsOverMinSpeedForDrift() and not gameplay_drift_drift.getIsInTheAir() and not gameplay_walk.isWalking() then
    guihooks.trigger("setDriftRealtimeAngle", angle )
  else
    guihooks.trigger("setDriftRealtimeAngle", 0)
  end

  if isDrifting then
    if angle == math.huge or angle == -math.huge or airspeed < 2 then angle = 0 end
    guihooks.trigger("setDriftRealtimeAirSpeed",airspeed)
  else
    guihooks.trigger("setDriftRealtimeAirSpeed", 0)
  end
end

local function displayRemainingDist()
  if gameplay_drift_destination and not gameplay_drift_destination.getDisableWrongWayAndDist() then
    local remainingDist = gameplay_drift_destination.getRemainingDist() or 0
    local data = {
      title = "missions.missions.general.distRemaining",
      txt = string.format("%d m", remainingDist),
      meters = remainingDist,
      category = "drift",
      style = "text",
      order = 10,
    }

    ui_apps_genericMissionData.setData(data)
  end
end

local function displayWrongWay()
  if not gameplay_drift_destination then return end

  if gameplay_drift_freeroam_driftSpots and gameplay_drift_freeroam_driftSpots.getIsInFreeroamChallenge() then
    return
  end

  if gameplay_drift_destination.getGoingWrongWay() and not outOfBoundsFlag then
    rtMessage(translateLanguage("missions.drift.general.wrongWayFirst", "missions.drift.general.wrongWayFirst"))
    wrongWayFlag = true
  elseif wrongWayFlag then
    clearRt()
    wrongWayFlag = false
  end
end

-- "Out of bounds" message has higher priority than the "Wrong way" message
local function displayOutOfBounds()
  if not gameplay_drift_bounds then return end

  local isInConcludingPhase = gameplay_drift_freeroam_driftSpots and gameplay_drift_freeroam_driftSpots.getIsInTheConcludingPhase()

  if gameplay_drift_bounds.getIsOutOfBounds() and not isInConcludingPhase then
    rtMessage(translateLanguage("missions.crawl.general.outOfBounds", "missions.crawl.general.outOfBounds"))
    outOfBoundsFlag = true
  elseif outOfBoundsFlag then
    clearRt()
    outOfBoundsFlag = false
  end
end

local function SetDriftUILayout(value)
  if value then
    core_gamestate.setGameState('freeroam', 'driftMission', 'freeroam')
  else
    core_gamestate.setGameState('freeroam', 'freeroam', 'freeroam')
  end

  driftDebugUILayout = value
end

local function imguiDebug()
  if gameplay_drift_general.getExtensionDebug("gameplay_drift_display") then
    if im.Begin("Drift display") then
      if im.Button("Toggle drift ui layout") then
        SetDriftUILayout(not driftDebugUILayout)
      end
    end
  end
end

local function onUpdate(dtReal, dtSim, dtRaw)
  imguiDebug()
  updateApps(dtReal)

  score = gameplay_drift_scoring.getScore()

  displayWrongWay()
  displayOutOfBounds()
  displayRemainingDist()
end

local function onFreeroamChallengeCompleted(data)
  core_jobsystem.create(function(job)
    rtMessage(string.format(data.newRecord and "New record! Score: %i" or "Drift zone finished! Score: %i", data.score))
    job.sleep(data.duration)
    clearRt()
  end
  )
end

local function onDriftQuickMessageDisplay(data)
  guihooks.trigger('displayDriftScoreModifier', data.msg )
end

local function onDriftPlVehReset()
  guihooks.trigger("setDriftRealtimeScore", 0, 0)
  guihooks.trigger("setDriftRemainingComboTime", 0)
  guihooks.trigger("setDriftRealtimeCreep", 0)
  clearRt()
end

local function onDriftCachedScoreReset()
  guihooks.trigger("setDriftRealtimeScore", 0, 0)
  clearRt()
end

local function onExtensionUnloaded()
  clearRt()
end

local function onFreeroamDriftZoneNewHighscore()
  flashMessage("New Highscore!")
end

local function onFreeroamChallengeTerminated(reason, msgDisplayTime)
  core_jobsystem.create(function(job)
    rtMessage(reason.msg)
    job.sleep(msgDisplayTime)
    clearRt()
    end
  )
end

local function onDriftScoreWrappedUp(score)
  flashMessage(string.format("+ %i points for current drift", math.floor(score)))
end

local function onNewDriftTierReached(tierData)
  flashMessage(string.format("%s", tierData.name))
end


local function onSerialize()
  SetDriftUILayout(false)

  return {
    driftDebugUILayout = driftDebugUILayout
  }
end

local function onDeserialized(data)
  driftDebugUILayout = data.driftDebugUILayout
end

local function onDriftDebugChanged(value)
  if not value then
    SetDriftUILayout(false)
  end
end

local function reset()
  ui_apps_genericMissionData.clearData()
end

local function getDriftDebugInfo()
  return driftDebugInfo
end

M.reset = reset

M.onUpdate = onUpdate
M.onSerialize = onSerialize
M.onDeserialized = onDeserialized
M.onDriftDebugChanged = onDriftDebugChanged

M.onDriftPlVehReset = onDriftPlVehReset
M.onDriftCompletedScored = onDriftCompletedScored
M.onDriftCrash = onDriftCrash
M.onDriftSpinout = onDriftSpinout
M.onDriftQuickMessageDisplay = onDriftQuickMessageDisplay
M.onFreeroamChallengeCompleted = onFreeroamChallengeCompleted
M.onDriftScoreWrappedUp = onDriftScoreWrappedUp
M.onNewDriftTierReached = onNewDriftTierReached

M.onDonutDriftScored = onDonutDriftScored
M.onTightDriftScored = onTightDriftScored
M.onHitPoleScored = onHitPoleScored
M.onNearPoleScored = onNearPoleScored

M.onNearStuntZoneFirst = onNearStuntZoneFirst

M.onDriftCachedScoreReset = onDriftCachedScoreReset
M.onExtensionUnloaded = onExtensionUnloaded
M.onFreeroamDriftZoneNewHighscore = onFreeroamDriftZoneNewHighscore
M.onFreeroamChallengeTerminated = onFreeroamChallengeTerminated

M.getDriftDebugInfo = getDriftDebugInfo
return M