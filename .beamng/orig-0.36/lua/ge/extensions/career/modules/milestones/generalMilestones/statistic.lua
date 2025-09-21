-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}
local milestones
  -- statistic milestones
local statisticMilestonesConfig
M.onGeneralMilestonesCollect = function(milestonesList)
  milestones = career_modules_milestones_milestones
  statisticMilestonesConfig = {
    {
      id = "stat_distance",
      sortIndex = 0,
      watchStatisticKey = "vehicle/total_odometer.length",
      filter = {statistic=true, general=true},
      type = "stat",
      icon = "odometer",
      color = milestones.colorGeneralGray,
      maxStep = 8,
      getValue = function() return (gameplay_statistic.metricGet("vehicle/total_odometer.length", true) or {value=0}).value end,
      getLabel = function(step, current, target) return "Long Distance Driver" end,
      getDescription = function(step, current, target) return string.format("Drive a total distance of %0.1fkm.", target/1000) end,
      getProgressLabel = function(step, current, target) return string.format("%0.1fkm / %0.1fkm", current/1000, target/1000) end,
      getTarget = function(step) return step == 0 and 0 or ({10,20,35,60,90,145,215,300})[step]*1000 end,
      getRewards = milestones.majorLinear,
    },
    {
      id = "stat_playtime",
      sortIndex = 1,
      watchStatisticKey = "general/mode/career.time",
      filter = {statistic=true, general=true},
      type = "stat",
      icon = "timer",
      color = milestones.colorGeneralGray,
      maxStep = 8,
      getValue = function() return (gameplay_statistic.metricGet("general/mode/career.time", true) or {value=0}).value end,
      getLabel = function(step, current, target) return "Play the Game" end,
      getDescription = function(step, current, target) return string.format("Play the game for %dh.", target/3600) end,
      getProgressLabel = function(step, current, target) return string.format("%dh %dm / %dh", math.floor(current / 3600),math.floor(((current % 3600) or 0) / 60),target/3600) end,
      getTarget = function(step) return step == 0 and 0 or math.max(1,(step)*5)*3600 end,
      getRewards = milestones.majorLinear,
    },
    {
      id = "stat_rollover",
      sortIndex = 2,
      watchStatisticKey = "vehicle/rollover",
      filter = {statistic=true, general=true},
      type = "stat",
      icon = "carToWheels",
      color = milestones.colorGeneralGray,
      maxStep = 8,
      getValue = function() return (gameplay_statistic.metricGet("vehicle/rollover", true)  or {value=0}).value end,
      getLabel = function(step, current, target) return string.format('Rollovers') end,
      getDescription = function(step, current, target) return string.format("Do %d rollovers in your vehicles.", target) end,
      getProgressLabel = function(step, current, target) return string.format("%d / %d", current, target) end,
      getTarget = function(step) return step == 0 and 0 or math.max(1,math.floor(math.pow(step-1,1.5))*5) end,
      getRewards = milestones.majorLinear,
    },
    {
      id = "jump",
      sortIndex = 3,
      watchStatisticKey = "vehicle/airtime.time",
      filter = {statistic=true, general=true},
      type = "stat",
      icon = "jump",
      color = milestones.colorGeneralGray,
      maxStep = 8,
      getValue = function() return (gameplay_statistic.metricGet("vehicle/airtime.time", true) or {value=0}).value  end,
      getLabel = function(step, current, target) return string.format('Airtime') end,
      getDescription = function(step, current, target) return string.format("Be airborne with your vehicle for %dm %ds.", target/60, target%60) end,
      getProgressLabel = function(step, current, target) return string.format("%dm %ds / %dm %ds", current/60, current%60, target/60, target%60) end,
      getTarget = function(step) return step == 0 and 0 or math.max(1,math.floor(math.pow(step,1.5)))*30 end,
      getRewards = milestones.majorLinear,
    },
  }
  for _, milestone in ipairs(statisticMilestonesConfig) do
    table.insert(milestonesList, milestone)
  end
end

M.onGeneralMilestonesSetupCallbacks = function(milestone)
  for _, milestone in ipairs(statisticMilestonesConfig) do
    M.registerStastisticCallback(milestone)
  end
end

-- statistic callback functions
local function clearStatisticCallback(milestone)
  if not milestone._statCallback then return end
  gameplay_statistic.callbackRemove(milestone.watchStatisticKey, milestone._callbackTrigger, milestone._statCallback, true)
  milestone._statCallback = nil
  milestone._callbackTrigger = nil
end

local function registerStastisticCallback(milestone)
  if not milestone.watchStatisticKey then return end

  M.clearStatisticCallback(milestone)

  local step = milestones.saveData.general[milestone.id].claimedStep + 1
  if step < milestone.maxStep then
    local statCallback = function()
      log("I","",string.format("Milestone Reached: %s %0.2f!", milestone.getLabel(step), milestone.getTarget(step)))
      milestones.milestoneReached(milestone.getLabel(step))
      milestone._statCallback = nil
    end
    milestone._statCallback = statCallback
    milestone._callbackTrigger = milestone.getTarget(step)
    gameplay_statistic.callbackRegister(milestone.watchStatisticKey, milestone.getTarget(step), statCallback, true)
  end
end

-- after a milestone is claimed, also re-register the watch
M.onGeneralMilestoneClaimed = registerStastisticCallback
M.registerStastisticCallback = registerStastisticCallback
M.clearStatisticCallback = clearStatisticCallback





return M