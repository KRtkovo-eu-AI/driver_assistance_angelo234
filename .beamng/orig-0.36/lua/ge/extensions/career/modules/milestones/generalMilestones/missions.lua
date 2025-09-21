-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

M.dependencies = {"gameplay_missions_missions"}
local missionIdToMilestonesList = {}
local milestoneConfigs = {}
local milestones

M.onGeneralMilestonesCollect = function(milestonesList)
  milestones = career_modules_milestones_milestones
  -- get all career missions.
  local careerMissions = {}
  local missionsByMissionType = {}
  local missionsByBranch = {}
  for i, mission in ipairs(gameplay_missions_missions.get()) do
    if mission.careerSetup.showInCareer then
      table.insert(careerMissions, mission)
      for branchKey, _ in pairs(mission.unlocks.branchTags or {}) do
        missionsByBranch[branchKey] = missionsByBranch[branchKey] or {}
        table.insert(missionsByBranch[branchKey], mission)
      end
    end
  end

  M.makeAllMissionStarMilestones(careerMissions, milestonesList)
  for _, branchKey in ipairs(tableKeysSorted(missionsByBranch)) do
    M.makeBranchMissionStarMilestones(missionsByBranch[branchKey], branchKey , milestonesList)
  end



  --M.makeUnlockMissionsMilestones(careerMissions, milestonesList)

end

local stepPercent = {0.10,0.18,0.3,0.54,0.74,1.0}
M.makeAllMissionStarMilestones = function(missions, milestonesList)
  local defaultStarCount, bonusStarCount = 0, 0
  local missionCount = 0
  for _, m in ipairs(missions) do
    defaultStarCount = defaultStarCount + m.careerSetup._activeStarCache.defaultStarCount
    bonusStarCount = bonusStarCount + m.careerSetup._activeStarCache.bonusStarCount
    missionCount = missionCount + 1
  end
  local totalStarCount = defaultStarCount + bonusStarCount


  -- all stars
  local allStarMilestone = {
      id = "mission_totalStarCount",
      filter = {mission=true, general=true},
      type = "mission",
      hooks = {onAnyMissionChanged = true},
      maxStep = #stepPercent,
      icon = "star",
      color=career_modules_milestones_milestones.colorMissionBlue,
      getValue = function()
        local count = 0
        for _, m in ipairs(missions) do
          count = count + tableSize(m.saveData.unlockedStars or {})
        end
        return count
      end,
      getLabel = function(step, displayValue, target) return string.format("Star Collector") end,
      getDescription = function(step, displayValue, target) return string.format("Collect %d stars from any challenge.", target) end,
      getProgressLabel = function(step, current, target) return string.format("%d Stars / %d Stars", current, target) end,
      getTarget = function(step) return step == 0 and 0 or math.ceil(stepPercent[step]*totalStarCount) end,
      getRewards = milestones.majorLinear,
    }
  milestones.saveData.general[allStarMilestone.id] = milestones.saveData.general[allStarMilestone.id] or {claimedStep = 0, notificationStep = 0}

  for _, m in ipairs(missions) do
    missionIdToMilestonesList[m.id] = missionIdToMilestonesList[m.id] or {}
    table.insert(missionIdToMilestonesList[m.id], allStarMilestone)
  end
  table.insert(milestoneConfigs, allStarMilestone)
  table.insert(milestonesList, allStarMilestone)

  -- pass all missions
  local passAllMissionMilestone = {
      id = "mission_passMissions",
      filter = {mission=true},
      type = "mission",
      hooks = {onAnyMissionChanged = true},
      maxStep = #stepPercent,
      icon = "star",
      color=career_modules_milestones_milestones.colorMissionBlue,
      getValue = function()
        local count = 0
        for _, m in ipairs(missions) do
          count = count + (m.saveData.unlockedStars.defaultUnlockedStarCount and 1 or 0)
        end
        return count
      end,
      getLabel = function(step, displayValue, target) return string.format("Challenge Passer") end,
      getDescription = function(step, displayValue, target) return string.format("To pass a challenge, you need to get at least one default star.", target) end,
      getProgressLabel = function(step, current, target) return string.format("%d Challenges / %d Challenges", current, target) end,
      getTarget = function(step) return step == 0 and 0 or math.ceil(stepPercent[step]*missionCount) end,
      getRewards = milestones.minorLinear,
    }
  milestones.saveData.general[passAllMissionMilestone.id] = milestones.saveData.general[passAllMissionMilestone.id] or {claimedStep = 0, notificationStep = 0}

  for _, m in ipairs(missions) do
    missionIdToMilestonesList[m.id] = missionIdToMilestonesList[m.id] or {}
    table.insert(missionIdToMilestonesList[m.id], passAllMissionMilestone)
  end
  table.insert(milestoneConfigs, passAllMissionMilestone)
  table.insert(milestonesList, passAllMissionMilestone)

    -- completing all missions
  local completeAllMissionsMilestone = {
      id = "mission_completeMissions",
      filter = {mission=true},
      type = "mission",
      hooks = {onAnyMissionChanged = true},
      maxStep = #stepPercent,
      icon = "star",
      color=career_modules_milestones_milestones.colorMissionBlue,
      getValue = function()
        local count = 0
        for _, m in ipairs(missions) do
          count = count +
          ((m.saveData.unlockedStars.totalUnlockedStarCount == m.careerSetup._activeStarCache.defaultStarCount + m.careerSetup._activeStarCache.bonusStarCount) and 1 or 0)
        end
        return count
      end,
      getLabel = function(step, displayValue, target) return string.format("Challenge Completionist", step) end,
      getDescription = function(step, displayValue, target) return string.format("To complete a challenge, you need to get at all default and all bonus stars.", target) end,
      getProgressLabel = function(step, current, target) return string.format("%d Challenges / %d Challenges", current, target) end,
      getTarget = function(step) return step == 0 and 0 or math.ceil(stepPercent[step]*missionCount) end,
      getRewards = milestones.minorLinear,
    }
  milestones.saveData.general[completeAllMissionsMilestone.id] = milestones.saveData.general[completeAllMissionsMilestone.id] or {claimedStep = 0, notificationStep = 0}

  for _, m in ipairs(missions) do
    missionIdToMilestonesList[m.id] = missionIdToMilestonesList[m.id] or {}
    table.insert(missionIdToMilestonesList[m.id], completeAllMissionsMilestone)
  end
  table.insert(milestoneConfigs, completeAllMissionsMilestone)
  table.insert(milestonesList, completeAllMissionsMilestone)
end

M.makeBranchMissionStarMilestones = function(missions, branchKey, milestonesList)
  local defaultStarCount, bonusStarCount = 0, 0
  local missionCount = 0
  for _, m in ipairs(missions) do
    defaultStarCount = defaultStarCount + m.careerSetup._activeStarCache.defaultStarCount
    bonusStarCount = bonusStarCount + m.careerSetup._activeStarCache.bonusStarCount
    missionCount = missionCount + 1
  end
  local totalStarCount = defaultStarCount + bonusStarCount
  local branchName = career_branches.getBranchById(branchKey).name
  branchName = translateLanguage(branchName, branchName, true)
  -- all stars
  local milestoneConfig = {
      id = "mission_totalStar_branch_"..branchKey,
      filter = {mission=true, ['branch_'..branchKey] = true},
      type = "mission",
      hooks = {onAnyMissionChanged = true},
      maxStep = #stepPercent,
      icon="star",
      color=career_modules_milestones_milestones.colorMissionBlue,
      getValue = function()
        local count = 0
        for _, m in ipairs(missions) do
          count = count + tableSize(m.saveData.unlockedStars or {})
        end
        return count
      end,
      getLabel = function(step, displayValue, target) return string.format("%s Star Collector", branchName) end,
      getDescription = function(step, displayValue, target) return string.format("Collect %d stars from challenges in the %s %s.", target, branchName, career_branches.getBranchById(branchKey).isSkill and "Skill" or "Branch" ) end,
      getProgressLabel = function(step, current, target) return string.format("%d Stars / %d Stars", current, target) end,
      getTarget = function(step) return step == 0 and 0 or math.ceil(stepPercent[step]*totalStarCount) end,
      getRewards = milestones.minorLinear,
    }
  milestones.saveData.general[milestoneConfig.id] = milestones.saveData.general[milestoneConfig.id] or {claimedStep = 0, notificationStep = 0}

  for _, m in ipairs(missions) do
    missionIdToMilestonesList[m.id] = missionIdToMilestonesList[m.id] or {}
    table.insert(missionIdToMilestonesList[m.id], milestoneConfig)
  end
  table.insert(milestoneConfigs, milestoneConfig)
  table.insert(milestonesList, milestoneConfig)

  -- pass all missions
  local passAllMissionMilestone = {
      id = "mission_passMissions_"..branchKey,
      filter = {mission=true, [''..branchKey] = true},
      type = "mission",
      hooks = {onAnyMissionChanged = true},
      maxStep = #stepPercent,
      icon="star",
      color=career_modules_milestones_milestones.colorMissionBlue,
      getValue = function()
        local count = 0
        for _, m in ipairs(missions) do
          count = count + (m.saveData.unlockedStars.defaultUnlockedStarCount and 1 or 0)
        end
        return count
      end,
      getLabel = function(step, displayValue, target) return string.format("%s Challenge Passer", branchName) end,
      getDescription = function(step, displayValue, target) return string.format("To pass a challenge, you need to get at least one default star.", target) end,
      getProgressLabel = function(step, current, target) return string.format("%d Challenges / %d Challenges", current, target) end,
      getTarget = function(step) return step == 0 and 0 or math.ceil(stepPercent[step]*missionCount) end,
      getRewards = milestones.minorLinear,
    }
  milestones.saveData.general[passAllMissionMilestone.id] = milestones.saveData.general[passAllMissionMilestone.id] or {claimedStep = 0, notificationStep = 0}

  for _, m in ipairs(missions) do
    missionIdToMilestonesList[m.id] = missionIdToMilestonesList[m.id] or {}
    table.insert(missionIdToMilestonesList[m.id], passAllMissionMilestone)
  end
  table.insert(milestoneConfigs, passAllMissionMilestone)
  table.insert(milestonesList, passAllMissionMilestone)

    -- completing all missions
  local completeAllMissionsMilestone = {
      id = "mission_completeMissions_"..branchKey,
      filter = {mission=true, [''..branchKey] = true},
      type = "mission",
      hooks = {onAnyMissionChanged = true},
      maxStep = #stepPercent,
      icon="star",
      color=career_modules_milestones_milestones.colorMissionBlue,
      getValue = function()
        local count = 0
        for _, m in ipairs(missions) do
          count = count +
          ((m.saveData.unlockedStars.totalUnlockedStarCount == m.careerSetup._activeStarCache.defaultStarCount + m.careerSetup._activeStarCache.bonusStarCount) and 1 or 0)
        end
        return count
      end,
      getLabel = function(step, displayValue, target) return string.format("%s Challenge Completionist", branchName) end,
      getDescription = function(step, displayValue, target) return string.format("To complete a challenge, you need to get at all default and all bonus stars.", target) end,
      getProgressLabel = function(step, current, target) return string.format("%d Challenges / %d Challenges", current, target) end,
      getTarget = function(step) return step == 0 and 0 or math.ceil(stepPercent[step]*missionCount) end,
      getRewards = milestones.minorLinear,
    }
  milestones.saveData.general[completeAllMissionsMilestone.id] = milestones.saveData.general[completeAllMissionsMilestone.id] or {claimedStep = 0, notificationStep = 0}

  for _, m in ipairs(missions) do
    missionIdToMilestonesList[m.id] = missionIdToMilestonesList[m.id] or {}
    table.insert(missionIdToMilestonesList[m.id], completeAllMissionsMilestone)
  end
  table.insert(milestoneConfigs, completeAllMissionsMilestone)
  table.insert(milestonesList, completeAllMissionsMilestone)
end

M.makeUnlockMissionsMilestones = function(missions, milestonesList)

  local milestoneConfig = {
      id = "mission_unlockMissions",
      filter = {mission=true},
      type = "mission",
      hooks = {onMissionUnlocked = true},
      getValue = function() return milestones.saveData.general['mission_unlockMissions'].unlockedCount end,
      getLabel = function(step, displayValue, target) return string.format("Challenge Unlocker %d", step) end,
      getDescription = function(step, displayValue, target) return string.format("Unlock new challenges by completing challenges and gaining branch levels.", target) end,
      getProgressLabel = function(step, current, target) return string.format("%d Challenges / %d Challenges", current, target) end,
      getTarget = function(step) return (step+1)*1 end,
      getRewards = milestones.minorLinear,
    }
  milestones.saveData.general[milestoneConfig.id] = milestones.saveData.general[milestoneConfig.id] or {claimedStep = 0, notificationStep = 0, unlockedCount = 0}

  for _, m in ipairs(missions) do
    missionIdToMilestonesList[m.id] = missionIdToMilestonesList[m.id] or {}
    table.insert(missionIdToMilestonesList[m.id], milestoneConfig)
  end
  table.insert(milestoneConfigs, milestoneConfig)
  table.insert(milestonesList, milestoneConfig)
end



M.onGeneralMilestonesSetupCallbacks = function()
  for _, milestone in ipairs(milestoneConfigs) do
    M.setNotificationTarget(milestone)
  end
end


-- branch related updates
local function setNotificationTarget(milestone)
  local step = milestones.saveData.general[milestone.id].notificationStep +1
  if milestone.maxStep and step > milestone.maxStep then return end
  local target = milestone.getTarget(step)
  -- check if completed
  if target then
    milestone._target = target
  end
end

local function onAnyMissionChanged(state, mission)
  if state == "stopped" then
    for _, milestone in ipairs(missionIdToMilestonesList[mission.id] or {}) do
      if milestone.hooks.onAnyMissionChanged then
        local step = milestones.saveData.general[milestone.id].notificationStep +1
        if milestone._target and milestone.getValue() >= milestone._target then
          milestones.milestoneReached(milestone.getLabel(step))
          milestone._target = nil
          milestones.saveData.general[milestone.id].notificationStep = step
          M.setNotificationTarget(milestone)
        end
      end
    end
  end
end

local function onMissionUnlocked(id)
  for _, milestone in ipairs(milestoneConfigs) do
    if milestone.hooks.onMissionUnlocked then
      -- count the unlock manually
      milestones.saveData.general[milestone.id].unlockedCount = milestones.saveData.general[milestone.id].unlockedCount + 1
      local step = milestones.saveData.general[milestone.id].notificationStep +1
      if milestone._target and milestone.getValue() >= milestone._target then
        milestones.milestoneReached(milestone.getLabel(step))
        milestone._target = nil
        milestones.saveData.general[milestone.id].notificationStep = step
        M.setNotificationTarget(milestone)
      end
    end
  end

end

M.setNotificationTarget = setNotificationTarget
M.onAnyMissionChanged = onAnyMissionChanged

return M