-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.dependencies = {'gameplay_missions_missions', 'gameplay_markerInteraction'}
local preselectedMissionId = nil
local openMenuWithCustomMissionList = nil

local repairCostMoney = 1000
local repairCostVoucher = 1
M.getRepairCostForStartingRepairType = function(type)
  if type == "moneyRepair" then return { money = repairCostMoney} end
  if type == "voucherRepair" then return { vouchers = repairCostVoucher} end
end

-- formats a single mission.
local function formatMission(m)
  local previewFile = m.previewFile
  if previewFile:sub(1, 1) == "/" then
      previewFile = previewFile:sub(2)
  end
  local info = {
    id = m.id,
    name = m.name,
    icon = m.iconFontIcon,
    description = m.description,
    previews = {previewFile},
    missionTypeLabel = m.missionTypeLabel or mission.missionType,
    userSettings = m:getUserSettingsData() or {},
    defaultUserSettings = m.defaultUserSettings or {},
    lastUserSettings = m.lastUserSettings,
    activeStars = M.getActiveStarsForUserSettings(m.id, m.defaultUserSettings),
    additionalAttributes = {},
    progress = m.saveData.progress,
    currentProgressKey = m.currentProgressKey or m.defaultProgressKey,
    unlocks = m.unlocks,
    hasUserSettingsUnlocked = gameplay_missions_progress.missionHasUserSettingsUnlocked(m.id),
    devMission = m.devMission,
    hasRules = m:hasRules(),
    official = m.official,
    author = m.author,
    date = m.date,
  }

  if gameplay_missions_missionManager.getForegroundMissionId() == m.id then
    --dump("default user settings from ",info.defaultUserSettings)
    --info.defaultUserSettings = deepcopy(m.lastUserSettings or {})
    --dump(info.defaultUserSettings)
    --info.activeStars = M.getActiveStarsForUserSettings(m.id, info.defaultUserSettings)
    for _, s in ipairs(info.userSettings) do
      if (m.lastUserSettings and m.lastUserSettings[s.key] ~= nil) then
        s.value = m.lastUserSettings[s.key]
      end
    end
  end


  info.hasUserSettings = #info.userSettings > 0
  local additionalAttributes, additionalAttributesSortedKeys = gameplay_missions_missions.getAdditionalAttributes()

  for _, attKey in ipairs(additionalAttributesSortedKeys) do
    local att = additionalAttributes[attKey]
    local mAttKey = m.additionalAttributes[attKey]
    local val
    if type(mAttKey) == 'string' then
      val = att.valuesByKey[m.additionalAttributes[attKey]]
    elseif type(mAttKey) == 'table' then
      val = m.additionalAttributes[attKey]
    end
    if val then
      table.insert(info.additionalAttributes, {
        icon = att.icon or "",
        labelKey = att.translationKey,
        valueKey = val.translationKey
      })
    end
  end
  for _, customAtt in ipairs(m.customAdditionalAttributes or {}) do
    table.insert(info.additionalAttributes, customAtt)
  end
  info.formattedProgress =  gameplay_missions_progress.formatSaveDataForUi(m.id)

  -- pre-format aggregates for the UI. This formatting might be the default later and then be moved to gameplay_missions_progress
  for key, prog in pairs(info.formattedProgress.formattedProgressByKey) do
    --dump(key)
    --dumpz(prog, 5)
    local ownAggregate = {}
    for i, label in ipairs(prog.ownAggregate.labels) do
      table.insert(ownAggregate, {
        label = label,
        value = prog.ownAggregate.rows[1][i]
      })
    end
    prog.ownAggregate = ownAggregate
  end

  info.leaderboardKey = m.defaultLeaderboardKey or 'recent'

  --info.gameContextUiButtons = {}
  info.gameContextUiButtons = m.getMissionScreenDataUiButtons and m:getMissionScreenDataUiButtons()

  --some career info like leagues
  if career_modules_branches_leagues then
    info.leagues = career_modules_branches_leagues.getLeaguesForMission(m.id)
  end
  return info
end

-- gets all the missions at the current location (mission marker), and returns them in a list already formatted.
local function getMissionsAtCurrentLocationFormatted()
  if not M.isStateFreeroam() then return nil end
  local dataToSend = {}
  local currentInteractableElements = gameplay_markerInteraction.getCurrentInteractableElements()
  if not currentInteractableElements then return end

  for _, m in ipairs(currentInteractableElements) do
    if m.missionId then
      table.insert(dataToSend, M.formatMission(gameplay_missions_missions.getMissionById(m.missionId)))
    end
  end
  for _, m in ipairs(gameplay_missions_missions.get()) do
    --table.insert(dataToSend, M.formatMission(gameplay_missions_missions.getMissionById(m.id)))
  end
  table.sort(dataToSend, gameplay_missions_unlocks.depthIdSort)
  return dataToSend
end

local function formatOngoingMission()
 local activeMission = nil
  for _, m in ipairs(gameplay_missions_missions.get()) do
    if m.id == gameplay_missions_missionManager.getForegroundMissionId() then
      activeMission = m
    end
  end
  return {context = 'ongoingMission', missions = {M.formatMission(activeMission)}, preselectedMissionId = gameplay_missions_missionManager.getForegroundMissionId()}
end

local function getMissionScreenData()
  if gameplay_missions_missionManager.getForegroundMissionId() ~= nil then
    -- case when there is currently a mission going on.
    return formatOngoingMission()
  else
    -- case when there are no ongoing missions.
    local missions = {}
    local hasCustomMissionList = openMenuWithCustomMissionList ~= nil
    if hasCustomMissionList then
      for _, id in ipairs(openMenuWithCustomMissionList) do
        table.insert(missions, M.formatMission(gameplay_missions_missions.getMissionById(id)))
      end
    else
      missions = M.getMissionsAtCurrentLocationFormatted()
    end
    openMenuWithCustomMissionList = nil
    if M.isStateFreeroam() and missions and next(missions) then
      extensions.hook("onAvailableMissionsSentToUi", context) -- for tutorial

      -- get ids for the tiles
      local ids = {}
      for _, m in ipairs(missions) do ids[m.id] = true end

      local ret = {
        context = 'availableMissions',
        missions = missions,
        missionCards = M.getMissionTiles(ids), -- for some us test
        --isWalking = gameplay_walk.isWalking(),
        --isCareerActive = career_career.isActive(),
        showMissionCards = true, --hasCustomMissionList,
        selectedMissionId = preselectedMissionId,
        isTutorialEnabled = (career_modules_linearTutorial and career_modules_linearTutorial.isLinearTutorialActive())
      }

      if career_career.isActive() then
        if career_modules_permissions then
          local reason = career_modules_permissions.getStatusForTag("interactMission")
          if reason.label then
            ret.startWarning = {label = reason.label, title ="Delivery in progress!" }
          end
        end
      end
      --[[ TODO: preselected mission refactor
      if fromMissionMenu then
        preselectedMissionId = nil
      end
      ]]
      return ret
    else
      if fromMissionMenu then
        preselectedMissionId = nil
      end
      return {context = 'empty' }
    end
  end
end

local defaultStartingOptions = {{ enabled = true, label = "ui.scenarios.start.start", type = "defaultStart" }}
local cantStartWalkingOptions = {{ enabled = false, label = "Cannot start this challenge on foot with current settings." }}
local cannotPayFeeOptions = {{enabled = false, label = "Cannot pay fee to start this challenge"}}
local function getUncertifiedVehicleOptions(certificationName)
  return {{enabled = false, label = string.format("Vehicle needs %s certification to start this challenge", certificationName)}}
end

local function sendStartingOptions(id, options, fee)
  local ret = {
    missionId = id,
    options = options,
    entryFee = fee,
  }
  guihooks.trigger("missionStartingOptionsForUserSettingsReady", ret)
  return
end

local function requestStartingOptionsForUserSettings(id, userSettings)
  local m = gameplay_missions_missions.getMissionById(id)
  if m then
    if not career_career.isActive() then
      -- outside of career, a mission can always be started
      sendStartingOptions(id, defaultStartingOptions)
      return
    end

    if (career_modules_linearTutorial and career_modules_linearTutorial.isLinearTutorialActive()) then
      -- during the tutorial, mission can be started without repair.
      sendStartingOptions(id, defaultStartingOptions)
      return
    end

    local missionUserSettings = m:getUserSettingsData() or {}
    local missionUserSettingByKey = {}
    for _, setting in ipairs(missionUserSettings) do
      missionUserSettingByKey[setting.key] = setting
    end

    -- figure out if the user intends to use their own vehicle.
    local usesOwnVehicle = false
    for _, setting in ipairs(userSettings) do
      if setting.key == "setupModuleVehicles" and missionUserSettingByKey[setting.key] then
        local val = missionUserSettingByKey[setting.key].values[setting.value]
        if val and val.type == "player" then
          usesOwnVehicle = true
        end
      end
    end

    -- entry fee
    local entryFee = m:getEntryFee(userSettings) or {}
    local hasEntryFee = false
    local canPayFee = true
    for key, value in pairs(entryFee) do
      hasEntryFee = hasEntryFee or value > 0
      if career_modules_playerAttributes.getAttributeValue(key) < value then
        canPayFee = false
      end
    end
    -- as list for the UI
    local entryFeeAsList = {}
    local attributesSorted = tableKeys(entryFee)
    table.sort(attributesSorted, career_branches.sortAttributes)
    for _, key in ipairs(attributesSorted) do
      if entryFee[key] > 0 then
        table.insert(entryFeeAsList, {rewardAmount = entryFee[key], icon = career_branches.getBranchIcon(key), attributeKey = key})
      end
    end
    if not next(entryFeeAsList) then
      entryFeeAsList = nil
    end

    if not canPayFee then
      sendStartingOptions(id, cannotPayFeeOptions, entryFeeAsList)
    end

    if not usesOwnVehicle then
      -- if not using own vehicle, mission can always be started normally.
      sendStartingOptions(id, defaultStartingOptions, entryFeeAsList)
      return
    end

    -- Check certification if mission requires it
    if usesOwnVehicle then
      local currentVehicle = career_modules_inventory.getCurrentVehicle()
      if not currentVehicle then
        log("W","","Player has no vehicle, but none of the previous starting options triggered. Something wrong?")
        sendStartingOptions(id, {{enabled=false, label="Something wrong..."}})
        return
      end

      local classInfo = career_modules_vehiclePerformance.getVehicleClass(currentVehicle)
      if not classInfo then
        log("W","","Player vehicle has no class info.")
        --sendStartingOptions(id, {{enabled=false, label="Something wrong..."}})
        --return
      end

      log("I","","Player has vehicle: " .. dumps(classInfo))

    end

    -- if the player uses own vehicle and walks, disable starting.
    if gameplay_walk.isWalking() then
      sendStartingOptions(id, cantStartWalkingOptions, entryFeeAsList)
      return
    end

    -- if we reached this, it means we need to check the repair status of the car and send options accordingly.
    -- getting repair status is async though.
    local currentVehicle = career_modules_inventory.getCurrentVehicle()
    if not currentVehicle then
      -- this shouldnt happen tho... just to be sure.
      log("W","","Player has no vehicle, but none of the previous starting options triggered. Something wrong?")
      sendStartingOptions(id, {{enabled=false, label="Something wrong..."}})
      return
    end

    career_modules_inventory.updatePartConditions(career_modules_inventory.getVehicleIdFromInventoryId(currentVehicle), currentVehicle,
    function()
      local needsRepair = career_modules_insurance.inventoryVehNeedsRepair(currentVehicle)
      if not needsRepair then
        -- all good! vehicle not damaged, can start normally.
        sendStartingOptions(id, defaultStartingOptions, entryFeeAsList)
        return
      end

      -- build repair options based on player currency.
      local vouchers = career_modules_playerAttributes.getAttributeValue('vouchers') - (hasEntryFee and entryFee.vouchers or 0)
      local money = career_modules_playerAttributes.getAttributeValue('money') - (hasEntryFee and entryFee.money or 0)
      local repairOptions = {
        {
          enabled = false,
          label = "Vehicle needs to be repaired to start",
          optionLabel = "Don't repair",
        }, {
          enabled = vouchers >= repairCostVoucher,
          label = vouchers >= repairCostVoucher and "Pay Repair and Start" or "Not enough vouchers for repair",
          optionsLabel = string.format("Repair for %d vouchers", repairCostVoucher),
          type = "voucherRepair"
        }, {
          enabled = money >= repairCostMoney,
          label = money >= repairCostMoney and "Pay Repair and Start" or "Not enough money to repair",
          optionsLabel = string.format("Repair for %d$",repairCostMoney),
          type = "moneyRepair",
        }
      }
      sendStartingOptions(id, repairOptions, entryFeeAsList)
    end)
  end
end
M.requestStartingOptionsForUserSettings = requestStartingOptionsForUserSettings



local function getActiveStarsForUserSettings(id, userSettings)
  local m = gameplay_missions_missions.getMissionById(id)
  if m then
    local defaultUserSettings = m.defaultUserSettings
    local flatSettings = {}
    if userSettings[1] then
      for _, setting in ipairs(userSettings) do
        flatSettings[setting.key] = setting.value
      end
    else
      flatSettings = userSettings
    end

    gameplay_missions_progress.setDynamicStarRewards(m, flatSettings)

    if m.ignoreUserSettingsKeyForActiveStars then
      for key, _ in pairs(m.ignoreUserSettingsKeyForActiveStars) do
        flatSettings[key] = nil
        defaultUserSettings[key] = nil
      end
    end

    -- check if settings are actually equal
    local same = true
    for k, v in pairs(defaultUserSettings) do
      same = same and flatSettings[k] == v
    end
    for k, v in pairs(flatSettings) do
      same = same and defaultUserSettings[k] == v
    end
    if not same then
      --dump(defaultUserSettings)
      --dump(flatSettings)
    end

    -- if same, enable all stars. if false, enable only bonus stars.
    -- TODO: make this a mission base class function. this way, each mission can handle this on its own.
    -- for example, some bonus stars could only be active with specific user settings (traffic on etc)

    local starInfo = {}
    local message = nil
    local starKeys, defaultCache = m.careerSetup._activeStarCache.sortedStars, m.careerSetup._activeStarCache.defaultStarKeysByKey
    if m.getActiveStarsForUserSettings then
      starInfo, message = m:getActiveStarsForUserSettings(flatSettings, starKeys, defaultCache, same)
    else
      -- fallback: default stars need same, bonus stars always on
      for _, key in ipairs(starKeys) do
        local info = {
          visible = true,
          message = nil,
          label = m.starLabels[key],
        }

        if defaultCache[key] then
          info.enabled = same
        else
          info.enabled = true
        end


        starInfo[key] = info
      end
      -- message if stars are disabled...
      if not same then
        message = "Main Objectives are only available with default settings."
      end
    end

    for key, info in pairs(starInfo) do
      if info.visible then
        info.label = m.starLabels[key]
        if type(m.starLabels[key]) == "function" then
          info.label = m.starLabels[key](m, flatSettings)
        elseif type(m.starLabels[key]) == "string" then
          info.label = {
            txt = info.label,
            context = gameplay_missions_progress.tryBuildContext(info.label, m.missionTypeData)
          }
        end
        info.rewards = m.careerSetup._activeStarCache.sortedStarRewardsByKey[key] or {}
      end
    end



    return {
      message = message,
      starInfo = starInfo,
    }
  end
  return {}
end
M.getActiveStarsForUserSettings = getActiveStarsForUserSettings


local function getLeaderboardsForUserSettings(id, userSettings)


end


local function startMissionById(id, userSettings, startingOptions)
  local m = gameplay_missions_missions.getMissionById(id)
  userSettings = userSettings or {}
  startingOptions = startingOptions or {}
  if m then
    if m.unlocks.startable or startingOptions.skipUnlockCheck then
      local flatSettings = {}
      for _, setting in ipairs(userSettings) do
        flatSettings[setting.key] = setting.value
      end
      gameplay_missions_missionManager.startWithFade(m, flatSettings, startingOptions or {})
      return
    else
      log("E","","Trying to start mission that is not startable due to unlocks: " .. dumps(id))
    end
  else
    log("E","","Trying to start mission with invalid id: " .. dumps(id))
  end

end

local function stopMissionById(id, force)
  id = id or gameplay_missions_missionManager.getForegroundMissionId()
  for _, m in ipairs(gameplay_missions_missions.get()) do
    if m.id == id then
      gameplay_missions_missionManager.attemptAbandonMissionWithFade(m, force)
      return
    end
  end
end
M.abandonCurrentMission = function() stopMissionById(nil, true) end

local function showMissionRules(id)
  id = id or gameplay_missions_missionManager.getForegroundMissionId()
  local mission = gameplay_missions_missions.getMissionById(id)
  if not mission then return end

  mission:showRulesAsPopups()
end
M.showMissionRules = showMissionRules

local function changeUserSettings(id, settings)
  local mission = gameplay_missions_missions.getMissionById(id)
  if not mission then return end

  -- flatten user settings first
  local flatSettings = {}
  for _, setting in ipairs(settings) do
    flatSettings[setting.key] = setting.value
  end

  local progressKey = "default"
  if mission.getProgressKeyForUserSetting then
    progressKey = mission:getProgressKeyForUserSetting(flatSettings) or progressKey
  end

  local translation = progressKey
  if mission.getProgressKeyTranslation then
    translation = mission:getProgressKeyTranslation(progressKey) or translation
  end


  -- always stock up to 5 elements at least
  local formattedProgress =  gameplay_missions_progress.formatSaveDataForUi(id, progressKey, nil, 5)
   -- pre-format aggregates for the UI. This formatting might be the default later and then be moved to gameplay_missions_progress
  for key, prog in pairs(formattedProgress.formattedProgressByKey) do
    --dump(key)
    --dumpz(prog, 5)
    local ownAggregate = {}
    for i, label in ipairs(prog.ownAggregate.labels) do
      table.insert(ownAggregate, {
        label = label,
        value = prog.ownAggregate.rows[1][i]
      })
    end
    prog.ownAggregate = ownAggregate
  end
  --dump(formattedProgress.formattedProgressByKey[progressKey])
  --dump("missionProgressKeyChanged", progressKey, translation)
  guihooks.trigger('missionProgressKeyChanged', {
    progressKey = progressKey,
    translation = translation,
    progress = formattedProgress.formattedProgressByKey[progressKey],

    })
end


local function setPreselectedMissionId(mId)
  preselectedMissionId = mId
end

local function isStateFreeroam()
  if core_gamestate.state and (core_gamestate.state.state == "freeroam" or core_gamestate.state.state == 'career') then
    return true
  end
  return false
end

-- Sound stuff
--local soundId = nil
local function setupSounds()
  --soundId = soundId or Engine.Audio.createSource('AudioGui', 'event:>UI>Career>EndScreen_Snapshot')
end

local function activateSoundBlur(active)
  --[[
  setupSounds()
  local sound = scenetree.findObjectById(soundId)
  if sound then
    if active then
      sound:play(-1)
      log("I","","Activated Sound Blur for Mission-Control")
    else
      sound:stop(-1)
      log("I","","Deactivated Sound Blur for Mission-Control")
    end
    sound:setTransform(getCameraTransform())
  end
  ]]
  core_sounds.setAudioBlur(active and 1 or 0)
end

local function tryDeleteSoundObject()
  --[[
  local sound = scenetree.findObjectById(soundId)
  if sound then
    sound:stop(-1)
    sound:delete()
  end
  soundId = nil
  ]]
end

local function onExtensionUnloaded()
  --tryDeleteSoundObject()
end

local function onClientEndMission(levelPath)
  --tryDeleteSoundObject()

end

M.onExtensionUnloaded = onExtensionUnloaded
M.activateSoundBlur = activateSoundBlur
M.onClientEndMission = onClientEndMission


local function openAPMChallenges(branch, skill)
  if branch == "undefined" then branch = nil end
  if skill == "undefined" or skill == "all" then skill = nil end
  -- set APM missions
  openMenuWithCustomMissionList = {
    mode = "leagues"
  }
  for _, m in ipairs(gameplay_missions_missions.get()) do
    if m.careerSetup.showInCareer and m.startTrigger.level == getCurrentLevelIdentifier() and m.startTrigger.type == "league" then
      local add = false
      if branch == nil and skill == nil then
        add = m.unlocks.startable
      end
      if branch and not skill then
        add = m.careerSetup.branch == branch
      end
      if skill then
        add = m.careerSetup.skill == skill
      end
      if add then
        table.insert(openMenuWithCustomMissionList, m.id)
      end
    end
  end

  guihooks.trigger('MenuOpenModule','mission-details')
end
M.openAPMChallenges = openAPMChallenges


local navigateToMissionSettings = nil
local function navigateToMission(id)
  local m = gameplay_missions_missions.getMissionById(id)
  if not m then return end

  freeroam_bigMapMode.enterBigMap({instant = true})

  navigateToMissionSettings = {
    missionId = id,
    timestamp = os.time(),
  }
end



local function onBigMapActivated()
  if not navigateToMissionSettings then return end
  if navigateToMissionSettings and navigateToMissionSettings.timestamp > os.time() - 3 then
    return
  end

  freeroam_bigMapMode.selectPoi(navigateToMissionSettings.missionId)
  freeroam_bigMapMode.navigateToMission(navigateToMissionSettings.missionId)
  navigateToMissionSettings = nil
end
M.onBigMapActivated = onBigMapActivated
M.navigateToMission = navigateToMission


local soundObjectIds = {}
local soundNames = {
  money = 'event:>UI>Career>EndScreen_Counting_Money',
  xp = 'event:>UI>Career>EndScreen_Counting_XP',
  vouchers = 'event:>UI>Career>EndScreen_Counting_Voucher',
}

local function activateSound(soundLabel, active,  pitch)
  soundObjectIds["money"] = soundObjectIds["money"] or Engine.Audio.createSource('AudioGui', soundNames["money"])
  soundObjectIds["xp"] = soundObjectIds["xp"] or Engine.Audio.createSource('AudioGui', soundNames["xp"])
  soundObjectIds["vouchers"] = soundObjectIds["vouchers"] or Engine.Audio.createSource('AudioGui', soundNames["vouchers"])
  pitch = pitch or 1
  if active then
    local sound = scenetree.findObjectById(soundObjectIds[soundLabel])
    if sound then
      log("I","","Activating sound: " .. soundLabel .. " with pitch: " .. pitch)
      sound:play(-1)
      sound:setParameter("pitch", pitch)
    end
  else
    for label, _ in pairs(soundNames) do
      local sound = scenetree.findObjectById(soundObjectIds[label])
      if sound then
        sound:stop(-1)
        sound:setParameter("pitch", 0)
      end
    end
  end
end
M.activateSound = activateSound

--------------------------------
-- Missions Grid Screen (WIP) --------

local makeMissionTile = function(m)
  return {
    skill = {m.careerSetup.skill},
    id = m.id,
    icon = m.iconFontIcon,
    label = m.name,
    formattedProgress =  gameplay_missions_progress.formatSaveDataForUi(m.id),
    startable = m.unlocks.startable,
    preview = m.previewFile,
    thumbnail = m.thumbnailFile,
    locked = not m.unlocks.visible,
  }
end


local difficultyValues = {veryLow=0, low=1, medium=2, high=3, veryHigh=4}
local function getMissionTiles(ids)
  local tilesById = {}

  local groupsByKey = {}
  groupsByKey["all"] = {meta = {type = "all"}, sortingKey = "date", sortingDirection = "desc"}
  for _, diff in pairs(gameplay_missions_missions.getAdditionalAttributes().difficulty.valuesByKey) do
    groupsByKey["difficulty_"..diff.key] = {label = "Difficulty: " ..diff.translationKey, meta = {type = "difficulty"}}
  end



  for _, m in ipairs(gameplay_missions_missions.get()) do
    if not ids then
      if not m.careerSetup or not m.careerSetup.showInFreeroam  or not m.unlocks.visible  then goto continue end
    else
      if not ids[m.id] then goto continue end
    end

    local filterData = {
      groupTags = {},
      sortingValues = {},
    }
    tilesById[m.id] = makeMissionTile(m)

    -- all
    filterData.groupTags['all'] = true
    filterData.sortingValues['date'] = tonumber(m.date) or 0

    -- missionType
    filterData.groupTags['missionType_'..m.missionTypeLabel] = true
    if not groupsByKey['missionType_'..m.missionTypeLabel] then
      groupsByKey['missionType_'..m.missionTypeLabel] = {label = m.missionTypeLabel, meta = {type = "missionType"}}
    end

    -- branch/skill
    local branchSkill = "branchSkill_"..string.format("%s_%s", m.careerSetup.branch or "No Branch", m.careerSetup.skill or "No Skill")
    filterData.groupTags[branchSkill] = true
    if not groupsByKey[branchSkill] then
      groupsByKey[branchSkill] = {label = string.format("%s / %s", m.careerSetup.branch or "No Branch", m.careerSetup.skill or "No Skill"), meta = {type = "branchSkill"}}
    end

    -- difficulty
    if m.additionalAttributes.difficulty then
      filterData.groupTags['difficulty_'..m.additionalAttributes.difficulty] = true
      filterData.sortingValues['difficulty'] = difficultyValues[m.additionalAttributes.difficulty]
    end

    -- level
    local levelId = m.startTrigger and m.startTrigger.level
    local level = core_levels.getLevelByName(levelId)
    if not groupsByKey['level_'..levelId] then
      if level then
        groupsByKey['level_'..levelId] = {label = level.title, meta = {type = "level"}}
      end
    end
    filterData.groupTags['level_'..levelId] = true

   -- league
    if m._leagueId then
      local league = career_modules_branches_leagues.getLeagueById(m._leagueId)
      local key = "league_"..league.id
      filterData.groupTags[key] = true
      if league._missionOrderByMissionId then
        filterData.sortingValues['_missionOrderByMissionId'] = league._missionOrderByMissionId[m.id]
      else
        filterData.sortingValues['_missionOrderByMissionId'] = m.id
      end
      if not groupsByKey[key] then
        groupsByKey[key] = {
          label = league.name,
          order = league._order,
          sortingKey = '_missionOrderByMissionId',
          meta = {type = "league", leagueId = league.id},
        }
      end

    end

    tilesById[m.id].filterData = filterData

    ::continue::
  end

  -- TODO: scenarios and other non-missions

  -- build group lists (new groups might have been added)
  for key, group in pairs(groupsByKey) do
    group.tileIdsUnsorted = {}
    group.meta = group.meta or {type=group.type}
  end

  -- add tiles to groups
  for id, tile in pairs(tilesById) do
    for groupKey, _ in pairs(tile.filterData.groupTags) do
      if groupsByKey[groupKey] then
        table.insert(groupsByKey[groupKey].tileIdsUnsorted, id)
        if groupsByKey[groupKey].sortingKey then
          local sKey = groupsByKey[groupKey].sortingKey
          table.sort(groupsByKey[groupKey].tileIdsUnsorted, function(a, b) return tilesById[a].filterData.sortingValues[sKey] < tilesById[b].filterData.sortingValues[sKey] end)
        else
          table.sort(groupsByKey[groupKey].tileIdsUnsorted)
        end
        if groupsByKey[groupKey].sortingDirection and groupsByKey[groupKey].sortingDirection == "desc" then
          arrayReverse(groupsByKey[groupKey].tileIdsUnsorted)
        end
      end
    end
  end

  -- groupSets
  local groupSetsByKey = {}
  if career_career and career_career.isActive() then
    -- Group by leagues in career mode
    career_modules_branches_leagues.clearLeagueUnlockCache()
    for groupKey, group in pairs(groupsByKey) do
      if group.meta.type == "league" then
        local league = career_modules_branches_leagues.getLeagueById(group.meta.leagueId)
        local formatted = career_modules_branches_leagues.formatLeague(league)
        group.meta.formattedLeague = formatted
        table.insert(groupSetsByKey, groupKey)
      end
    end
  else
    -- Group by mission type outside career
    for groupKey, group in pairs(groupsByKey) do
      if group.meta.type == "all" then
        table.insert(groupSetsByKey, groupKey)
      end
    end
  end

  table.sort(groupSetsByKey, function(a,b)
    local ag, bg = groupsByKey[a], groupsByKey[b]
    return (ag.order or a) < (bg.order or b)
  end)

  return {
    tilesById = tilesById,
    groupsByKey = groupsByKey,
    groupKeys = groupSetsByKey,
  }

end
M.getMissionTiles = getMissionTiles

M.isStateFreeroam = isStateFreeroam

M.formatMission = formatMission
M.getMissionsAtCurrentLocationFormatted = getMissionsAtCurrentLocationFormatted
M.startMissionById = startMissionById
M.stopMissionById = stopMissionById
M.startFromWithinMission = function(id, settings)
  local flatSettings = {}
  for _, setting in ipairs(settings or {}) do
    flatSettings[setting.key] = setting.value
  end
 gameplay_missions_missionManager.startFromWithinMission(gameplay_missions_missions.getMissionById(id), flatSettings)
end
M.changeUserSettings = changeUserSettings
M.setPreselectedMissionId = setPreselectedMissionId
M.getMissionScreenData = getMissionScreenData



M.uiLayoutHistory = {}



M.savedLayouts = nil
M.discoverLayouts = function()
  M.savedLayouts = {}
  for _, file in ipairs(FS:findFiles("/gameplay/testing", "*.json", 1, false, true)) do
    local layout = jsonReadFile(file)
    layout.filePath = file
    local dir, fn, ext = path.split(file)
    layout.fileName = fn
    layout.isSaved = true
    table.insert(M.savedLayouts, layout)
  end
end
M.testLayout = nil
M.enableDebug = function(enable)
  if enable then
    M.onUpdate = M.debugOnUpdate
  else
    M.onUpdate = nil
  end
  extensions.hookUpdate("onUpdate")
end

local im = nil
M.debugOnUpdate = function(dt)
  im = im or ui_imgui
  if im then
    if not M.savedLayouts then
      M.discoverLayouts()
    end
    im.Begin("Mission Start Screen History")
    local layouts = {}
    arrayConcat(layouts, M.uiLayoutHistory)
    arrayConcat(layouts, M.savedLayouts)
    if im.Button("Discover Layouts") then
      M.discoverLayouts()
    end
    if im.Button("Exit Screen") then
      guihooks.trigger('ChangeState', 'freeroam')
    end
    if im.Button("Clear History") then
      M.uiLayoutHistory = {}
    end
    im.Separator()
    -- Table header
    if im.BeginTable("LayoutsTable", 4, bit.bor(im.TableFlags_Resizable, im.TableFlags_ScrollY)) then
      im.TableSetupColumn("Date")
      im.TableSetupColumn("file")
      im.TableSetupColumn("Header Name")
      im.TableSetupColumn("Actions")
      im.TableHeadersRow()
      local layoutsChanged = false
      -- Table rows
      for i, layout in ipairs(layouts) do
        im.PushID1("layout:"..i)
        im.TableNextRow()

        -- View Layout column
        im.TableNextColumn()
        local header = "# " .. i
        header = header .. " (" .. (layout.recordedAtFormatted or "Unknown") .. ")"
        if im.Button(header) then
          M.testLayout = layout
          guihooks.trigger('ChangeState', 'freeroam')
          guihooks.trigger('ChangeState', {state = 'mission-control', params = { mode = "test"}})
        end

        -- File column
        im.TableNextColumn()
        im.Text(layout.fileName or "(Not saved as file)")

        -- Header Name column
        im.TableNextColumn()
        local headerName = "N/A"
        if layout.header and layout.header.header then
          headerName = translateLanguage(layout.header.header, layout.header.header, true)
        end
        headerName = headerName .. " (" .. (layout.mode or "N/A") .. ")"
        im.Text(headerName)

        -- Actions column
        im.TableNextColumn()
        -- Show either Save or Delete button based on whether the layout is saved
        if layout.isSaved then
          if im.Button("Delete") then
            -- Find the file path
            local filePath = layout.filePath
            -- Delete the file
            if FS:fileExists(filePath) then
              FS:removeFile(filePath)
              layoutsChanged = true

            end
          end
        else
          if im.Button("Save") then
            jsonWriteFile("/gameplay/testing/missionScreen_"..layout.recordedAt..".json", layout, true)
            layoutsChanged = true
          end
        end
        im.PopID()
      end

      if layoutsChanged then
        M.discoverLayouts()
      end
      im.EndTable()
    end
    im.End()
  end
end

M.onMissionStartScreenReady = function(uiLayout)
  -- Store the last 20 UI layouts in memory
  table.insert(M.uiLayoutHistory, uiLayout)
  uiLayout.recordedAt = os.time()
  uiLayout.recordedAtFormatted = os.date("%Y-%m-%d %H:%M:%S")

  -- Keep only the last 20 layouts
  if #M.uiLayoutHistory > 20 then
    table.remove(M.uiLayoutHistory, 1)
  end
end

-- Testing: Todo: Remove
M.onRequestMissionScreenData = function(mode)
  if string.find(mode, "test") then
    local layout = deepcopy(M.testLayout)
    guihooks.trigger("onRequestMissionScreenDataReady", layout, {replaySupport = false} )
  end
end





M.isAnyMissionActive = function() return gameplay_missions_missionManager.getForegroundMissionId() ~= nil end
M.isMissionStartOrEndScreenActive = function()
  local screensActive = {}
  extensions.hook("onGetIsMissionStartOrEndScreenActive", screensActive)
  return screensActive[1] or false
end


return M