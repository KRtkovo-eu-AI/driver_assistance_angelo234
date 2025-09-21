-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local leaguesById = nil
M.getLeagueById = function(id) return leaguesById[id] end


-- career utility
local missionsBySkill = {}
local starsBySkillCache = {}
M.clearLeagueUnlockCache = function()
  for _, league in pairs(leaguesById) do league._unlocked = nil end
  starsBySkillCache = {}
end
M.getSimpleUnlockedStatus = function()
  M.clearLeagueUnlockCache()
  local ret = {}
  for id, _ in pairs(leaguesById) do
    ret[id] = M.isLeagueUnlocked(id)
  end
  return ret
end

local function isLeagueUnlocked(id)
  local league = M.getLeagueById(id)
  if not league then return false end
  if league._unlocked ~= nil then return league._unlocked end
  -- no condition = unlocked by default
  if not league.unlock or not next(league.unlock) then
    league._unlocked = true
    return true
  end

  local allConditionsMet = true
  for _, condition in ipairs(league.unlock) do
    if condition.type == "leagueStars" then
      local count = 0
      local otherLeague = M.getLeagueById(condition.leagueId)
      for _, mId in ipairs(otherLeague.missions) do
        local all, def, bon = gameplay_missions_progress.getUnlockedStarCountsForMissionById(mId)
        -- todo: decide if all or only default stars count
        count = count + all
      end

      for i, dsId in ipairs(otherLeague.driftSpots or {}) do
        local spot = gameplay_drift_saveLoad.getDriftSpotById(dsId)
        local defaultCount = 0
        for _, obj in ipairs(spot.info.objectives) do
          defaultCount = defaultCount + (spot.saveData.objectivesCompleted[obj.id] and 1 or 0)
        end
        count = count + defaultCount
      end

      condition.met = (count >= condition.stars)
      condition.progress = {
        min = 0,
        max = condition.stars,
        cur = count,
        label = string.format("%d Stars / %d Stars", count, condition.stars)

      }
      condition.label = string.format("Get %d stars from '%s'", condition.stars, translateLanguage(otherLeague.name, otherLeague.name, true))

    elseif condition.type == "branchLevel" then
      local level = career_branches.getBranchLevel(condition.skillId)
      local neededForLevel = career_branches.getXPNeededForLevel(condition.skillId, condition.level)
      local xp = career_branches.getBranchXP(condition.skillId)
      condition.met = level >= condition.level
      condition.progress = {
        min = 0,
        max = neededForLevel,
        cur = xp,
        label = string.format("%d XP / %d XP", xp, neededForLevel)
      }
      local branch = career_branches.getBranchById(condition.skillId)
      condition.label = string.format("Reach level %d of '%s'", condition.level,  translateLanguage(branch.name, branch.name, true))
    elseif condition.type == "skillStars" then
      if not starsBySkillCache[condition.skillId] then
        starsBySkillCache[condition.skillId] = {}
        starsBySkillCache[condition.skillId].total, starsBySkillCache[condition.skillId].unlocked = M.getStarsForSkills({[condition.skillId] = true})
      end
      condition.met = starsBySkillCache[condition.skillId].unlocked >= condition.stars
      condition.progress = {
        min = 0, max = condition.stars, cur = starsBySkillCache[condition.skillId].unlocked, label = string.format("%d / %d", starsBySkillCache[condition.skillId].unlocked, condition.stars)
      }
      local branch = career_branches.getBranchById(condition.skillId)
      condition.label = string.format("Get %d stars in '%s'", condition.stars,  translateLanguage(branch.name, branch.name, true))
    elseif condition.type == "branchStars" then
      if not starsBySkillCache[condition.branchId] then
        starsBySkillCache[condition.branchId] = {}

        local validSkills = {}
        for _, skill in pairs(career_branches.getBranches()) do
          if skill.parentBranch == condition.branchId then
            validSkills[skill.id] = true
          end
        end
        starsBySkillCache[condition.branchId].total, starsBySkillCache[condition.branchId].unlocked = M.getStarsForSkills(validSkills)
      end
      condition.met = starsBySkillCache[condition.branchId].unlocked >= condition.stars
      condition.progress = {
        min = 0, max = condition.stars, cur = starsBySkillCache[condition.branchId].unlocked, label = string.format("%d / %d", starsBySkillCache[condition.branchId].unlocked, condition.stars)
      }
      local branch = career_branches.getBranchById(condition.branchId)
      condition.label = string.format("Get %d stars in '%s'", condition.stars,  translateLanguage(branch.name, branch.name, true))
    elseif condition.type == "inDevelopment" then
      condition.met = false
      condition.hidden = true
      --condition.label = translateLanguage("ui.career.inDevelopment", "ui.career.inDevelopment", true)
      --condition.progress = {}
    end
    allConditionsMet = allConditionsMet and condition.met
  end
  league._unlocked = allConditionsMet
  return allConditionsMet
end
M.isLeagueUnlocked = isLeagueUnlocked


local function formatLeague(l)
  isLeagueUnlocked(l.id)
  local league = deepcopy(l)
  league.totalStarsAvailable, league.totalStarsObtained = 0, 0
  for _, mId in ipairs(league.missions) do
    local mission = gameplay_missions_missions.getMissionById(mId)
    if mission then
      league.totalStarsAvailable = league.totalStarsAvailable + mission.careerSetup._activeStarCache.defaultStarCount
      local all, def, bon = gameplay_missions_progress.getUnlockedStarCountsForMissionById(mId)
      league.totalStarsObtained = league.totalStarsObtained + def
    end
  end

  for i, dsId in ipairs(league.driftSpots or {}) do
    local spot = gameplay_drift_saveLoad.getDriftSpotById(dsId)
    league.totalStarsAvailable = league.totalStarsAvailable + #spot.info.objectives
    local defaults = {}
    local defaultCount = 0
    for _, obj in ipairs(spot.info.objectives) do
      table.insert(defaults, spot.saveData.objectivesCompleted[obj.id] or false)
      defaultCount = defaultCount + (spot.saveData.objectivesCompleted[obj.id] and 1 or 0)
    end
    league.totalStarsObtained = league.totalStarsObtained + defaultCount
  end

  local skill = career_branches.getBranchById(league.skillId)
  if skill then
    league.icon = skill.icon
    league.accentColor = skill.accentColor
  end
  if league.isCertification then
    league.icon = "badgeRoundStar"
  end
  --league.milestones = career_modules_milestones_milestones.getMilestones({"branch_apexRacing"}).list
  return league
end
M.formatLeague = formatLeague

local function getLeaguesForProgressBranchPage(branchId)
  M.clearLeagueUnlockCache()
  local ret = {}
  for id, l in pairs(leaguesById) do
    local league = formatLeague(l)
    local skill = career_branches.getBranchById(league.skillId)
    if not branchId or branchId == skill.id or skill.parentId == branchId then
      table.insert(ret, league)
    end
  end
  M.addLeagueSortOrder()
  table.sort(ret, function(a,b) return a._order < b._order end)
  return ret
end
M.getLeaguesForProgressBranchPage = getLeaguesForProgressBranchPage

local function getLeaguesForUnlockChange(before, after)
  local ret = {}
  for id, l in pairs(leaguesById) do
    if not before[id] and after[id] then
      local league = formatLeague(l)
      table.insert(ret, league)
    end
  end
  M.addLeagueSortOrder()
  table.sort(ret, function(a,b) return a._order < b._order end)
  return ret
end
M.getLeaguesForUnlockChange = getLeaguesForUnlockChange

local function getNoLeague(skill, missions, driftSpots)
  log("E","","Deprecated")
  -- todo: remove
end
M.getNoLeague = getNoLeague

local function getLeaguesForMission(missionId)
  M.clearLeagueUnlockCache()
  local ret = {}
  for id, l in pairs(leaguesById) do
    if tableContains(l.missions, missionId) then
      local league = formatLeague(l)
      table.insert(ret, league)
    end
  end
  M.addLeagueSortOrder()
  table.sort(ret, function(a,b) return a._order < b._order end)
  return ret
end
M.getLeaguesForMission = getLeaguesForMission

-- save/load

local function addLeagueSortOrder()
  -- Group leagues by skill
  local leaguesBySkill = {}
  for id, league in pairs(leaguesById) do
    if not leaguesBySkill[league.skillId] then
      leaguesBySkill[league.skillId] = {}
    end
    table.insert(leaguesBySkill[league.skillId], id)
  end

  -- Sort leagues within each skill group
  for skillId, skillLeagues in pairs(leaguesBySkill) do
    -- First sort by skill stars and file order
    table.sort(skillLeagues, function(a, b)
      local leagueA = leaguesById[a]
      local leagueB = leaguesById[b]
      if leagueA._skillStars ~= leagueB._skillStars then
        return leagueA._skillStars < leagueB._skillStars
      end
      return leagueA._orderByFile < leagueB._orderByFile
    end)

    -- Move highlighted and unlocked leagues to front
    local highlighted = {}
    local regular = {}
    for _, id in ipairs(skillLeagues) do
      local league = leaguesById[id]
      if league.highlightIfUnlocked and isLeagueUnlocked(id) then
        table.insert(highlighted, id)
        --log("I", "", "Highlighted league: " .. league.id)
      else
        table.insert(regular, id)
      end
    end

    -- Combine highlighted and regular leagues
    leaguesBySkill[skillId] = {}
    for _, id in ipairs(highlighted) do
      table.insert(leaguesBySkill[skillId], id)
    end
    for _, id in ipairs(regular) do
      table.insert(leaguesBySkill[skillId], id)
    end
  end

  -- Assign final order based on skill order and position within skill group
  local orderIndex = 1
  local skillIds = tableKeys(leaguesBySkill)
  table.sort(skillIds, function(a, b)
    return career_branches.getOrder(a) < career_branches.getOrder(b)
  end)

  for _, skillId in ipairs(skillIds) do
    for _, leagueId in ipairs(leaguesBySkill[skillId]) do
      leaguesById[leagueId]._order = orderIndex
      orderIndex = orderIndex + 1
    end
  end
end

local function loadLeagues()
  -- todo: load from file
  if not leaguesById then
    leaguesById = {}
    local files = FS:findFiles("/gameplay/", "*.leagues.json", -1, true, false)
    table.sort(files)
    local i = 1
    for _, file in ipairs(files) do
      local data = jsonReadFile(file)
      if data then
        for _, league in ipairs(data) do
          -- todo sanitize
          if leaguesById[league.id] then
            log("E","","League already exists: " .. league.id .. " (from file " .. dumps(file)..") Ignored.")
          else
            league._skillStars = 0
            league._orderByFile = i
            i = i+1
            for _, prog in ipairs(league.unlock or {}) do
              if prog.type == "skillStars" then
                league._skillStars = prog.stars
              end
            end
            league.missions = league.missions or {}
            league._missionOrderByMissionId = {}
            for i, id in ipairs(league.missions) do
              league._missionOrderByMissionId[id] = i
            end
            league.driftSpots = league.driftSpots or {}

            league.skillPathId = career_branches.extractBranchPathIdFromFilePath(file)
            local skill = career_branches.getBranchById(league.skillPathId)
            league.skillId = skill.pathId
            if skill then
              league.branchId = skill.parentId
              if skill.isInDevelopment then
                league.isInDevelopment = true
              end
            end
            if league.isInDevelopment then
              league.unlock = {{type = "inDevelopment"}}
              league.comingSoon = {{icon = "roadblockL", label = "In Development"}}
              league.missions = {}
            end
            if not league.branchId then
              log("E","","League " .. league.id .. " has no branchId but has parentBranch " .. league.skillId)
            end



            leaguesById[league.id] = league
          end
        end
      end
    end
    addLeagueSortOrder()
  end
end

local function getFallbackLeague(skillId, missions)
  local league  = {
    id = "fallback_"..skillId,
    name = "",
    description = "",
    missions = {},
    driftSpots = {},
    unlock = {},
    skillId = skillId,
    branchId = skillId,
    _unlocked = true,
    _isNoLeague = true,
    _orderByFile = -1,
    _skillStars = 0,
  }
  local branch = career_branches.getBranchById(skillId)
  league.name = string.format("%s Challenges", translateLanguage(branch.name, branch.name, true))
  league.description = string.format("Fallback for %s Skill", translateLanguage(branch.name, branch.name, true))
  league.missions = missions
  league._missionOrderByMissionId = {}
  for i, id in ipairs(league.missions) do
    league._missionOrderByMissionId[id] = i
  end
  local skill = career_branches.getBranchById(league.skillId)
  if skill and skill.parentBranch then
    league.branchId = skill.parentBranch
  else
    skill.branchId = league.skillId
  end
  return league
end

local function onCareerModulesActivated()
  local saveSlot, savePath = career_saveSystem.getCurrentSaveSlot()

  -- load leagues
  loadLeagues()
end
M.onCareerModulesActivated = onCareerModulesActivated

local function getStarsForSkills(skillIds)
  local total, unlocked = 0, 0

  for _, m in ipairs(gameplay_missions_missions.get()) do
    if m.careerSetup and skillIds[m.careerSetup.skill] then
      total = total + m.careerSetup._activeStarCache.defaultStarCount
      local _, def, _ = gameplay_missions_progress.getUnlockedStarCountsForMissionById(m.id)
      unlocked = unlocked + def
    end
  end
  return total, unlocked
end
M.getStarsForSkills = getStarsForSkills
M.getStarsForSkill = function(skillId) return getStarsForSkills({[skillId] = true}) end

local function startConditionIncludesLeague(cond, leagueId)
  if cond.type == "league" and (cond.leagueId == leagueId or not leagueId) then
    return true
  else
    for _, n in ipairs(cond.nested or {}) do
      if M.startConditionIncludesLeague(n, leagueId) then
        return true
      end
    end
  end
end
M.startConditionIncludesLeague = startConditionIncludesLeague

local function getStartConditionLeagueId(cond, map)
  if cond.type == "league" then
    map[cond.leagueId] = true
  else
    for _, n in ipairs(cond.nested or {}) do
      M.getStartConditionLeagueId(n, map)
    end
  end
end
M.getStartConditionLeagueId = getStartConditionLeagueId

local function onMissionsLoadedFromFiles(missionsById)
  loadLeagues()
  missionsBySkill = {}
  local missionIdsBySkill = {}
  local dirty = 0
  local missionIdsBySkillWithoutLeague = {}
  local missionsUsedInLeagues = {}

  local filesDataById = {}
  for _, mission in ipairs(gameplay_missions_missions.getFilesData() or {}) do
    filesDataById[mission.id] = mission
  end
  local dirtyFilesById = {}
  for id, league in pairs(leaguesById) do
    local validMissionIds = {}
    for _, mId in ipairs(league.missions) do
      local m = gameplay_missions_missions.getMissionById(mId)
      if not m then
        log("E","","Mission does not exist: " .. dumps(mId) .. " in league " .. dumps(id)..". Removed from league.")
      end

      if m and m.careerSetup.showInCareer then
        table.insert(validMissionIds, mId)
        if not M.startConditionIncludesLeague(m.startCondition, id) then

          local filesData = filesDataById[mId]
          if filesData then
            if filesData.startCondition.type == "always" or filesData.startCondition.type == "automatic" or filesData.startCondition.type == "branchLevel" then
              filesData.startCondition = {type = "league", leagueId = id}
            else
              filesData.startCondition = {type = "multiAnd", nested = {filesData.startCondition}}
              table.insert(filesData.startCondition.nested, {type = "league", leagueId = id})
            end
            m.startCondition = filesData.startCondition
            filesData._dirty = true
            dirty = dirty + 1
            dirtyFilesById[mId] = (dirtyFilesById[mId] or 0) + 1
          end
        end
        if not missionsUsedInLeagues[mId] then
          missionsUsedInLeagues[mId] = id
        else
          log("E","","Mission is used in more than one league: " .. dumps(mId) .. " in league " .. dumps(missionsUsedInLeagues[mId]) .. " and " .. dumps(id))
        end
      end

    end
    leaguesById[id].missions = validMissionIds
  end
  for mId, m in pairs(missionsById) do
    local lMap = {}
    M.getStartConditionLeagueId(m.startCondition, lMap)
    if table.getn(lMap) > 1 then
      log("W","","Mission has more than one league in starting condition... " .. mId)
    end
    local lId = next(lMap)
    local mSkill = m.careerSetup.skill
    local league
    if lId then
      league = M.getLeagueById(lId)
      if not league then
        log("E","","League doesnt exist! " .. lId .. " in " .. mId)
      end
    end
    if not league then
      -- if no league, make sure to use the fallback league.
      local fallbackLeagueId = "fallback_".. mSkill
      m._leagueId = fallbackLeagueId
      missionIdsBySkillWithoutLeague[mSkill] = missionIdsBySkillWithoutLeague[mSkill] or {}
      table.insert(missionIdsBySkillWithoutLeague[mSkill], mId)
    else
      -- make sure the mission setup matches the league setup.
      m._leagueId = league.id
      if m.careerSetup.skill ~= league.skillId then
        local filesData = filesDataById[mId]
        if filesData then
          filesData._dirty = true
          dirty = dirty + 1
          filesData.careerSetup.skill = league.skillId
          m.careerSetup.skill = league.skillId
          dirtyFilesById[mId] = (dirtyFilesById[mId] or 0) + 1
        end
      end
    end

    missionIdsBySkill[m.careerSetup.skill] = missionIdsBySkill[m.careerSetup.skill] or {}
    missionsBySkill[m.careerSetup.skill] = missionsBySkill[m.careerSetup.skill] or {}
    if not missionIdsBySkill[m.careerSetup.skill][mId] then
      table.insert(missionsBySkill[m.careerSetup.skill], m)
    end
    missionIdsBySkill[m.careerSetup.skill][mId] = true
  end

  -- set up the leagues for missions with fallback league.

  for skillId, list in pairs(missionIdsBySkillWithoutLeague) do
    local fallbackLeagueId = "fallback_"..skillId
    table.sort(list, gameplay_missions_unlocks.depthIdSortUsingIds)
    local fallbackLeague = getFallbackLeague(skillId, list)
    --log("I", "", string.format("Creating fallback league for skill '%s' with missions: %s", skillId, dumps(list)))
    leaguesById[fallbackLeagueId] = fallbackLeague
  end
  if next(missionIdsBySkillWithoutLeague) then
    addLeagueSortOrder()
  end

  if dirty > 0  then
    log("W","","Some ("..tostring(dirty)..") missions were not set up properly for leagues.")
    for mId, count in pairs(dirtyFilesById) do
      log("W","","Mission " .. mId .. " was not set up properly for leagues " .. count .. " times.")
    end
  end
end
M.onMissionsLoadedFromFiles = onMissionsLoadedFromFiles
M.addLeagueSortOrder = addLeagueSortOrder
--[[
M.onAfterDriftSpotsLoaded = function(spotsById)
  loadLeagues()
  for _, spot in pairs(spotsById) do
    spot._isInLeague = nil
    spot.unlocked = true
  end
  for id, league in pairs(leaguesById) do
    for _, dsId in ipairs(league.driftSpots) do
      local spot = spotsById[dsId]
      if not spot then
        log("E","","Drift spot " .. dumps(dsId) .. " listed in league " .. dumps(id).. " does not exist.")
      else
        if spot._isInLeague then
          log("E","","Drift spot " .. dumps(dsId) .. " is listed in more than one league! " .. dumps(spot._isInLeague) ..  " and " .. dumps(id))
        end
        spot._isInLeague = id
        spot.unlocked = M.isLeagueUnlocked(id)
      end
    end
  end
end
]]
return M