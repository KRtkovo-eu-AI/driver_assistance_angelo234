-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

local im = ui_imgui

local driftDebugInfo = {
  default = false,
  canBeChanged = true
}

-- stats to track during a challenge and to display at the end of challenge
local tiersAndScore = {}
local rawPerformanceStats = {}
local quickMessages = {}

local quickMessagesToBeConfirmed = {}

local firstUpdateFrame = true

local function imguiDebug()
  if gameplay_drift_general.getExtensionDebug("gameplay_drift_scoreboard") then
    if im.Begin("Drift scoreboard") then
      if im.Button("Reset scoreboard") then
        M.reset()
      end
      im.Text(dumpsz(tiersAndScore, 3))
      im.SameLine()
      im.Text(dumpsz(M.getPerformanceStats(), 3))
      im.SameLine()
      im.Text(dumpsz(quickMessages, 3))
    end
  end
end

local function populateTierNames()
  for _, tierData in pairs(gameplay_drift_scoring.getDriftTiers()) do
    tiersAndScore[tierData.id] = {totalScore = 0, count = 0, name = tierData.name, order = tierData.order}
  end
end

local function onUpdate()
  if firstUpdateFrame then
    firstUpdateFrame = false
    M.reset()
  end

  imguiDebug()
end

local function onDriftQuickMessageReached(data)
  if not quickMessagesToBeConfirmed[data.quickMessageId] then
    quickMessagesToBeConfirmed[data.quickMessageId] = {count = 0, totalScoreEarned = 0}
  end

  quickMessagesToBeConfirmed[data.quickMessageId].count = quickMessagesToBeConfirmed[data.quickMessageId].count + 1
  quickMessagesToBeConfirmed[data.quickMessageId].totalScoreEarned = quickMessagesToBeConfirmed[data.quickMessageId].totalScoreEarned + data.reward
end

local function populateQuickMessagesNames()
  for quickMessageId, quickMessageData in pairs(gameplay_drift_quickMessages.getQuickMessages()) do
    quickMessages[quickMessageId] = {count = 0, totalScoreEarned = 0, msg = quickMessageData.msg}
  end
end

local function onDriftCompletedScored(data)
  if gameplay_drift_general.getPaused() or gameplay_drift_general.getFrozen() then return end
  local scoreToSubstract = 0 -- do not count the quick messages scores as part of the score earned by the tiers

  if data.combo > rawPerformanceStats.maxDriftCombo then rawPerformanceStats.maxDriftCombo = data.combo end

  for quickMessageId, quickMessageData in pairs(quickMessagesToBeConfirmed) do
    local qmScoreEarnedWithMulti = quickMessageData.totalScoreEarned * data.combo
    scoreToSubstract = scoreToSubstract + qmScoreEarnedWithMulti
    quickMessages[quickMessageId].count = quickMessages[quickMessageId].count + quickMessageData.count
    quickMessages[quickMessageId].totalScoreEarned = quickMessages[quickMessageId].totalScoreEarned + qmScoreEarnedWithMulti
  end
  quickMessagesToBeConfirmed = {}

  tiersAndScore[data.tier.id].totalScore = tiersAndScore[data.tier.id].totalScore + (data.addedScore - scoreToSubstract)
  tiersAndScore[data.tier.id].count = tiersAndScore[data.tier.id].count + 1
end

-- when single drift done
local function onDriftActiveDataFinished(data)
  if gameplay_drift_general.getPaused() or gameplay_drift_general.getFrozen() then return end

  rawPerformanceStats.driftSpeeds.total = rawPerformanceStats.driftSpeeds.total + data.totalSpeeds
  rawPerformanceStats.driftSpeeds.count = rawPerformanceStats.driftSpeeds.count + data.totalSteps

  rawPerformanceStats.driftPerformanceFactors.total = rawPerformanceStats.driftPerformanceFactors.total + data.totalPerformanceFactors
  rawPerformanceStats.driftPerformanceFactors.count = rawPerformanceStats.driftPerformanceFactors.count + data.totalSteps

  rawPerformanceStats.driftAngles.total = rawPerformanceStats.driftAngles.total + data.totalAngles
  rawPerformanceStats.driftAngles.count = rawPerformanceStats.driftAngles.count + data.totalSteps

  rawPerformanceStats.totalIndividualDrifts = rawPerformanceStats.totalIndividualDrifts + 1
  rawPerformanceStats.totalDriftDuration = rawPerformanceStats.totalDriftDuration + data.totalDriftTime
  rawPerformanceStats.totalDriftDist = rawPerformanceStats.totalDriftDist + data.driftDistance
end

local function reset()
  populateTierNames()
  populateQuickMessagesNames()
  rawPerformanceStats = {
    driftSpeeds = {total = 0, count = 0},
    driftAngles = {total = 0, count = 0},
    driftPerformanceFactors = {total = 0, count = 0},
    maxDriftAngle = 0,
    maxDriftCombo = 0,
    totalDriftDist = 0,
    totalIndividualDrifts = 0,
    totalDriftDuration = 0
  }
end

local function getDriftDebugInfo()
  return driftDebugInfo
end

local function formatTime(seconds)
  seconds = math.ceil(seconds)

  local hours = math.floor(seconds / 3600)

  seconds = seconds % 3600
  local minutes = math.floor(seconds / 60)
  seconds = seconds % 60

  local parts = {}

  if hours > 0 then
      table.insert(parts, hours .. (hours == 1 and " "..translateLanguage("ui.timespan.hour", "ui.timespan.hour", true) or " "..translateLanguage("ui.timespan.hour.plural", "ui.timespan.hour.plural", true)))
  end
  if minutes > 0 then
      table.insert(parts, minutes .. (minutes == 1 and " " ..translateLanguage("ui.timespan.minute", "ui.timespan.minute", true) or " " ..translateLanguage("ui.timespan.minute.plural", "ui.timespan.minute.plural", true)))
  end
  if seconds > 0 or #parts == 0 then
      table.insert(parts, seconds .. (seconds == 1 and " "..translateLanguage("ui.timespan.second", "ui.timespan.second", true) or " "..translateLanguage("ui.timespan.second.plural", "ui.timespan.second.plural", true)))
  end

  return table.concat(parts, " ")
end

-- getting the performance stats is a function  as opposed to as a table because we don't want to calculate avrgs every frame i guess
local function getPerformanceStats()
  local compiledStats = {}

  compiledStats.avrgDriftPerformanceFactor = {
    order = 1,
    value = string.format("%.3f", rawPerformanceStats.driftPerformanceFactors.total / rawPerformanceStats.driftPerformanceFactors.count)
  }
  compiledStats.avrgDriftSpeed = {
    order = 2,
    value = string.format("%i %s", translateVelocity(rawPerformanceStats.driftSpeeds.total / rawPerformanceStats.driftSpeeds.count / 3.6, true))
  }
  compiledStats.avrgDriftAngle = {
    order = 3,
    value = string.format("%i °", rawPerformanceStats.driftAngles.total / rawPerformanceStats.driftAngles.count)
  }
  compiledStats.maxDriftAngle = {
    order = 4,
    value = string.format("%i °", rawPerformanceStats.maxDriftAngle)
  }
  compiledStats.maxDriftCombo = {
    order = 5,
    value = rawPerformanceStats.maxDriftCombo
  }
  compiledStats.totalDriftDist = {
    order = 6,
    value = string.format("%i %s", translateDistance(rawPerformanceStats.totalDriftDist, false))
  }
  compiledStats.totalIndividualDrifts = {
    order = 7,
    value = rawPerformanceStats.totalIndividualDrifts
  }
  compiledStats.totalDriftDuration = {
    order = 8,
    value = string.format("%s", formatTime(rawPerformanceStats.totalDriftDuration))
  }

  -- translate all at once
  for name, data in pairs(compiledStats) do
    data.name = translateLanguage("missions.drift.stats."..name, "missions.drift.stats."..name, true)
  end

  table.sort(compiledStats, function(a, b) return a.order < b.order end)

  return compiledStats
end

local function getTiersStats()
  return tiersAndScore
end

local function getDriftEventStats()
  return quickMessages
end

local function driftFailed()
  quickMessagesToBeConfirmed = {}
end

local function onDriftCrash()
  driftFailed()
end

local function onDriftSpinout()
  driftFailed()
end

M.reset = reset

M.onUpdate = onUpdate

M.onDriftActiveDataFinished = onDriftActiveDataFinished
M.onDriftQuickMessageReached = onDriftQuickMessageReached
M.onDriftCompletedScored = onDriftCompletedScored

M.onDriftCrash = onDriftCrash
M.onDriftSpinout = onDriftSpinout

M.getDriftDebugInfo = getDriftDebugInfo
M.getPerformanceStats = getPerformanceStats
M.getTiersStats = getTiersStats
M.getDriftEventStats = getDriftEventStats

return M