-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local logTag = 'scenariosLoader'

local M = {}
M.dependencies = {'core_levels'}
M.scenarioModules   = {'scenario_scenarios', 'scenario_waypoints', 'statistics_statistics', 'scenario_raceUI', 'scenario_raceGoals'}
M.prefabCache = {}

local displayedRestrictMessage = nil

local function buildPrefabCache(levelPath)
  if levelPath and not M.prefabCache[levelPath] then
    local cacheTable = {}
    local foundFiles = FS:findFiles(levelPath, "*.prefab\t*.prefab.json", -1, true, false)
    local dir
    local pathScenarioName
    local ext
    for _, prefabPath in ipairs(foundFiles) do
      if string.endswith(prefabPath, ".json") then
        dir, pathScenarioName, ext = path.splitWithoutExt(prefabPath, ".prefab.json")
      else
        dir, pathScenarioName, ext = path.splitWithoutExt(prefabPath, ".prefab")
      end
      pathScenarioName = string.lower(pathScenarioName)
      cacheTable[dir] = cacheTable[dir] or {}
      local internal_dic = cacheTable[dir]
      internal_dic[pathScenarioName] = prefabPath
    end
    M.prefabCache[levelPath] = cacheTable
  end
end

M.processedScenarioCache = {}

local function processScenarioData(scenarioKey, scenarioData, scenarioFilename)
    scenarioData.scenarioKey = scenarioKey

    if scenarioFilename then
      scenarioData.sourceFile = scenarioFilename
      scenarioData.official = isOfficialContentVPath(string.sub(scenarioFilename, 0))
      scenarioData.levelName = string.gsub(scenarioFilename, "(.*/)(.*)/scenarios/(.*)%.json", "%2")
      scenarioData.map = "ui.common.unknown"
      if scenarioFilename ~= 'flowgraphEditor' and scenarioData.levelName ~= 'flowgraphEditor' then
        -- improve the data a little bit
        scenarioData.mission = path.getPathLevelMain(scenarioData.levelName)

        if not FS:fileExists(scenarioData.mission) then
          -- Fallback to old MIS file
          scenarioData.mission = '/levels/'..scenarioData.levelName..'/'..scenarioData.levelName..'.mis'
        end

        if not FS:fileExists(scenarioData.mission) then
          -- Fallback to level directory
          scenarioData.mission = '/levels/'..scenarioData.levelName..'/'
          if not FS:directoryExists(scenarioData.mission) then log('E', logTag, scenarioData.levelName.." scenario file not found") end
        end
      end
      scenarioData.scenarioName = scenarioData.scenarioName or string.gsub(scenarioFilename, "(.*/)(.*)%.json", "%2")
      scenarioData.directory = string.gsub(scenarioFilename, "(.*)/(.*)%.json", "%1")
    end

    local tmp = '/levels/' .. scenarioData.levelName .. '/info.json'
    if FS:fileExists(tmp) then
      local infoJson = jsonReadFile(tmp)
      if infoJson and infoJson.title then
        scenarioData.map = infoJson.title
      end
    end

    -- below are the defaults for a scenario including automatic file guessing for some fields
    if not scenarioData.vehicles then
      scenarioData.vehicles = {scenario_player0 = {playerUsable = true, startFocus = true}, ['*'] = {playerUsable = false}}
    end

    scenarioData.aiControlledVehiclesById = {}

    if not scenarioData.difficulty then
      scenarioData.difficulty = 'easy'
    end
    scenarioData.extensions = scenarioData.extensions or {}
    table.insert(scenarioData.extensions, {name=scenarioData.scenarioName, optional=true}) -- try to load an extension with the scenarioname by default

    -- figure out if a html start file is existing
    local htmldiscovered = false
    if not scenarioData.startHTML then
      scenarioData.startHTML = scenarioData.scenarioName .. '.html'
      htmldiscovered = true
    end
    if scenarioData.directory and not FS:fileExists(scenarioData.directory.."/"..scenarioData.startHTML) then
      if not htmldiscovered then
        log('W', logTag, 'start html not found, disabled: ' .. scenarioData.startHTML)
      end
      scenarioData.startHTML = nil
    end

    if not scenarioData.introType then
        scenarioData.introType = 'htmlOnly'
    end

    -- figure out the prefabs: add default and check them
    if not scenarioData.prefabs then
      scenarioData.prefabs = {}
    end

    -- try to load some defaults
    local levelPath = scenarioData.mission
    buildPrefabCache(levelPath)

    if scenarioData.directory then
      local foundMatchingPrefabs = {}
      local prefabsToFind = {}
      prefabsToFind[string.lower(scenarioData.scenarioName)] = true
      for _, prefabPath in ipairs(scenarioData.prefabs) do
        if not string.find(prefabPath,".prefab") then -- checking if this is a short name entry in the table. Checking for ".prefab" also cover ".prefab.json"
          prefabsToFind[prefabPath] = true
        else
          table.insert(foundMatchingPrefabs, prefabPath)
        end
      end

      local prefabCache = M.prefabCache[levelPath] or {}

      -- check the local directory of the scenario first
      local directory_cache = prefabCache[scenarioData.directory.."/"] or {}
      for prefabName, prefabPath in pairs(directory_cache) do
        if prefabsToFind[prefabName] or string.find(prefabName, scenarioData.scenarioName) then
          table.insert(foundMatchingPrefabs, prefabPath)
          prefabsToFind[prefabName] = false
        end
      end

      -- then check the level's directory
      local levelPath_cache = prefabCache[levelPath] or {}
      for prefabName, prefabPath in pairs(levelPath_cache) do
        if prefabsToFind[prefabName] or string.find(prefabName, scenarioData.scenarioName) then
          table.insert(foundMatchingPrefabs, prefabPath)
          prefabsToFind[prefabName] = false
        end
      end
      scenarioData.prefabs = foundMatchingPrefabs
    end

    -- figure out the previews automatically and check for errors
    if not scenarioData.previews then
      local tmp = FS:findFiles(scenarioData.directory.."/", scenarioData.scenarioName..'*.jpg', 0, true, false)
      local matchedScenarios = FS:findFiles(scenarioData.directory.."/", scenarioData.scenarioName..'*.json', 0, true, false)
      local otherScenarios = {}
      for i,v in ipairs(matchedScenarios) do
        local otherScenarioName = string.gsub(v, "(.*/)(.*)%.json", "%2")
        if otherScenarioName ~= scenarioData.scenarioName then
          table.insert(otherScenarios, otherScenarioName)
        end
      end

      scenarioData.previews = {}
      for _, p in pairs(tmp) do
        if string.startswith(p, scenarioData.directory) then
          local imageFilename = string.sub(p, string.len(scenarioData.directory) + 2, string.len(p) - 4)
          local foundClash = false
          for i,otherScenarioName in ipairs(otherScenarios) do
            if imageFilename == otherScenarioName then
              foundClash = true
            end
          end
          if not foundClash then
            table.insert(scenarioData.previews, imageFilename..'.jpg')
          end
        end
      end
    end

    local np = {}
    if scenarioData.directory then
      for _,p in pairs(scenarioData.previews) do
          table.insert(np, imageExistsDefault(scenarioData.directory.."/"..p))
      end
      if tableIsEmpty(np) then
         table.insert(np, imageExistsDefault('/'))
      end
      scenarioData.previews = np
    end
    if #scenarioData.previews == 0 then
      log('W', logTag, 'scenario has no previews: ' .. tostring(scenarioData.scenarioName))
    end

    if not scenarioData.playersCountRange then scenarioData.playersCountRange = {} end
    if not scenarioData.playersCountRange.min then scenarioData.playersCountRange.min = 1 end
    scenarioData.playersCountRange.min = math.max( 1, scenarioData.playersCountRange.min )
    if not scenarioData.playersCountRange.max then scenarioData.playersCountRange.max = scenarioData.playersCountRange.min end
    scenarioData.playersCountRange.max = math.max( scenarioData.playersCountRange.min, scenarioData.playersCountRange.max )

    scenarioData.extraTime = 0

    -- set defaults if keys are missing

    scenarioData.lapCount = scenarioData.lapCount or 1
    scenarioData.whiteListActions = scenarioData.whiteListActions or {}
    table.insert(scenarioData.whiteListActions, "default_whitelist_scenario")

    scenarioData.blackListActions = scenarioData.blackListActions or {}
    table.insert(scenarioData.blackListActions, "default_blacklist_scenario")

    scenarioData.radiusMultiplierAI = scenarioData.radiusMultiplierAI or 1

    local restrictScenarios = settings.getValue("restrictScenarios")
    if restrictScenarios == nil then restrictScenarios = true end
    if (shipping_build and campaign_campaigns and campaign_campaigns.getCampaignActive()) then restrictScenarios = true end

    if not restrictScenarios then
      if not displayedRestrictMessage then
        displayedRestrictMessage = true
        log('W', logTag, '**** Restrictions on Scenario Turned off in game settings. Removing restrictions. ****')
      end
      scenarioData.whiteListActions = {}
      scenarioData.blackListActions = core_input_actionFilter.createActionTemplate({"vehicleTeleporting"})
    end

    -- process lapConfig
    scenarioData.BranchLapConfig = scenarioData.BranchLapConfig or scenarioData.lapConfig or {}
    scenarioData.lapConfig = {}
    for i, v in ipairs(scenarioData.BranchLapConfig) do
      if type(v) == 'string' then
        table.insert(scenarioData.lapConfig, v)
      end
    end
    scenarioData.initialLapConfig = deepcopy(scenarioData.lapConfig)

    if scenarioData.attemptsInfo then
      scenarioData.attemptsInfo.allowedAttempts = scenarioData.attemptsInfo.allowedAttempts or 0
      scenarioData.attemptsInfo.delayPerAttempt = scenarioData.attemptsInfo.delayPerAttempt or 1
      scenarioData.attemptsInfo.allowVehicleSelectPerAttempt = scenarioData.attemptsInfo.allowVehicleSelectPerAttempt or false
      scenarioData.attemptsInfo.failAttempts = scenarioData.attemptsInfo.failAttempts or {}
      scenarioData.attemptsInfo.completeAttempt = scenarioData.attemptsInfo.completeAttempt or {}
      scenarioData.attemptsInfo.attemptNumber = 0
      scenarioData.attemptsInfo.waitTimerStart = false
      scenarioData.attemptsInfo.waitTimer = 0
      scenarioData.attemptsInfo.waitTimerActive = false
      scenarioData.attemptsInfo.currentAttemptReported = false
    end
    return scenarioData
end

local function loadScenario(scenarioPath, key)
  -- log('D', logTag, 'Load scenario - '..scenarioPath)
  local processedScenario = nil
  if scenarioPath then
    local scenarioData = jsonReadFile(scenarioPath)
    if scenarioData then
      -- jsonReadFile for valid scenarios returns a table with 1 entry
      if type(scenarioData) == 'table' and #scenarioData == 1 then
        processedScenario = processScenarioData(key, scenarioData[1], scenarioPath)
        --TODO: this is converted to string to avoid data loss when json en-/decoding.
        processedScenario.date = processedScenario.date and (processedScenario.date .. "")
      end
    else
      log('E', logTag, 'Could not find scenario '..scenarioPath)
    end
  end

  return processedScenario
end

local loadMissionAsScenarioFlowgraph = 'lua/ge/extensions/scenario/loadMissionAsScenario.flow.json'

local function getScenarioDataForMission(m)
  local additionalAttributes, additionalAttributesSortedKeys = gameplay_missions_missions.getAdditionalAttributes()
  local diffString = additionalAttributes.difficulty.valuesByKey[m.additionalAttributes.difficulty] and additionalAttributes.difficulty.valuesByKey[m.additionalAttributes.difficulty].translationKey
  local scenarioData = {
    name = m.name,
    description = m.description,
    authors = m.author or "Anonymous",
    difficultyLabel = diffString or nil,
    difficulty = m.additionalAttributes.difficulty,
    date = ((m.date ~= 0 and m.date) or "") .. "",
    flowgraph = loadMissionAsScenarioFlowgraph,
    variables = {
      level = m.startTrigger.level,
      missionId = m.id,
    },
    isMissionAsScenario = true,
    scenarioName = m.id,
    levelName = m.startTrigger.level,
    previews = {m.previewFile},
    official = isOfficialContentVPath(m.missionFolder .. "/info.json") and m.author == "BeamNG",
    customOrderKey = m.missionType,
    additionalAttributes = {{
      labelKey = "bigMap.missionLabels.missionType",
      valueKey = m.missionTypeLabel,
      icon = 'bubble_chart'}
    },
  }
  return processScenarioData(nil, scenarioData, nil)
end
M.getScenarioDataForMission = getScenarioDataForMission
-- this function is used by the UI to display the list of scenarios, and by techCore to get the list of available scenarios
local function getList(subdirectory, includeAll)
  displayedRestrictMessage = false
  local levelPaths = core_levels.getLevelPaths()
  local scenarios = {}
  local paths = {}
  for _, path in ipairs(levelPaths) do
    if subdirectory ~= nil then
      path = path .. 'scenarios/' .. subdirectory .. '/'
    else
      path = path ..'scenarios/'
    end
    table.insert(paths, path)
  end
  table.insert(paths, "flowgraphEditor/scenarios/")
  -- find all normal scenarios.
  for _, path in ipairs(paths) do
    local subfiles = FS:findFiles(path, '*.json', -1, true, false)
    for _, scenarioFilename in ipairs(subfiles) do
      local newScenario = loadScenario(scenarioFilename)
      if newScenario then
        if (not shipping_build or includeAll)  or  (shipping_build and not newScenario.restrictToCampaign) then
          table.insert(scenarios, newScenario)
        end
      end
    end
  end
  -- find all Scenario-enabled flowgraphs.
  for _, p in ipairs(paths) do
    local fgFiles = FS:findFiles(p, '*.flow.json', -1, true, false)
    for _, fgPath in ipairs(fgFiles) do
      local fgData = jsonReadFile(fgPath)

      local dir, fn, ext = path.splitWithoutExt(fgPath, true)
      if fgData.isScenario then
        local scenarioData = {
          name = fgData.name or "New Flowgraph Scenario",
          description =  string.gsub(fgData.description or "No Description", "\\n", "\n"),
          authors = fgData.authors or "Anonymous",
          difficulty = fgData.difficulty or 40,
          date = (fgData.date or os.time()).."",
          flowgraph = fgPath,
          scenarioName = fn
        }
        local newScenario = processScenarioData(nil, scenarioData, fgPath)
        if newScenario then
          if (not shipping_build or includeAll)  or  (shipping_build and not newScenario.restrictToCampaign) then
            table.insert(scenarios, newScenario)
          end
        end
      end
    end
  end

  for _, m in ipairs(gameplay_missions_missions.get()) do
    if m.startTrigger.level and core_levels.getLevelByName(m.startTrigger.level) and m.isAvailableAsScenario then
      local scenarioData = M.getScenarioDataForMission(m)
      if scenarioData then
        if (not shipping_build or includeAll)  or  (shipping_build and not scenarioData.restrictToCampaign) then
          table.insert(scenarios, scenarioData)
        end
      end
    end
  end

  return scenarios
end

-- this function is called when the user selects a scenario to play from the UI
local function start(sc)
  if campaign_campaigns then
    campaign_campaigns.stop()
  end

  if scenetree.MissionGroup then
    log('D', logTag, 'Delaying start of scenario until current level is unloaded...')

    M.triggerDelayedStart = function()
      log('D', logTag, 'Triggering a delayed start of scenario...')
      M.triggerDelayedStart = nil
      start(sc)
    end

   endActiveGameMode(M.triggerDelayedStart)
  else
    if sc.flowgraph then
      local fgPath = sc.flowgraph
      if type(sc.flowgraph) ~= 'string' then
        fgPath = sc.scenarioName .. '.flow.json'
      end
      local relativePath = (sc.directory or "").."/"..fgPath
      local absolutePath = fgPath
      local path = FS:fileExists(relativePath) and relativePath or (FS:fileExists(absolutePath) and absolutePath or nil)
      if not path then
        log("E", "", "Unable to locate flowgraph for scenario "..dumps(sc.name)..", neither as relative nor absolute dir: "..dumps(sc.flowgraph))
        return true
      end

      local mgr = core_flowgraphManager.loadManager(path)
      for name, value in pairs(sc.variables or {}) do
        mgr.variables:changeBase(name, value)
      end
      unloadAutoExtensions()
      loadPresetExtensions()
      extensions.hook("startTracking", {Name = "ScenarioRunning", ScenarioName = sc.name, File = sc.sourceFile})
      mgr:setRunning(true)
      mgr.stopRunningOnClientEndMission = true -- make mgr self-destruct when level is ended.
    else
      unloadAutoExtensions()
      loadPresetExtensions()
      extensions.load(M.scenarioModules)
      displayedRestrictMessage = nil
      scenario_scenarios.executeScenario(sc)
    end
  end
end

local function startByPath(path)
  if not string.find(path, ".json") then
    path = path..".json"
  end
  if not FS:fileExists(path) then
    log('E', logTag, path .." does not exist")
    return false
  end
  --TODO check whether correct level is loaded <- really necessary?
  local newScenario = loadScenario(path)
  start(newScenario)
  return true
end

-- function that reloads the current scenario when its sources have changed
local function reloadScenarioSourcefile()
  local scenario = scenarios and scenario_scenarios.getScenario()
  if not scenario then return end

  -- saves the values to look for
  local levelName = scenario.levelName
  local scenarioName = scenario.scenarioName
  local sourceFile = scenario.sourceFile
  -- refresh the list
  local scenarios = getList()
  -- select the scenario again
  for k,v in pairs(scenarios) do
    if v.levelName == levelName and v.scenarioName == scenarioName and v.sourceFile == sourceFile then
      start(v)
      break
    end
  end
  log('E', logTag, 'Unable to reload scenario: scenario not found anymore. Please check for typos in the JSON')
end

-- called when a file is modified, deleted, etc
local function onFileChanged(filename, type)
  local scenario = scenarios and scenario_scenarios.getScenario()
  if not scenario then return end

  if scenario.sourceFile == filename then
    reloadScenarioSourcefile()
  end
end

local function load(name)
  local list = getList()
  for _, v in ipairs(list) do
    if v.name == name then
      start(v)
    end
  end
end

local function  customPreviewLoader(levelInfo,  levelName)
  -- figure out the previews automatically and check for errors
  local directory = '/levels/'..levelName
  local previews = {}

  if levelInfo and levelInfo.levelInfo and type(levelInfo.levelInfo.previews) == 'table' and #levelInfo.levelInfo.previews > 0 then
    -- add prefix
    local newPreviews = {}
    for _, img in pairs(levelInfo.levelInfo.previews) do
      table.insert(newPreviews, directory..'/' .. img)
    end
    previews = newPreviews
  else
    local tmp = FS:findFiles("/levels/"..levelName.."/",levelName..'_preview*.png', 0, true, false)
    for _, p in pairs(tmp) do
      table.insert(previews, p)
    end
    tmp = FS:findFiles("/levels/"..levelName.."/",levelName..'_preview*.jpg', 0, true, false)
    for _, p in pairs(tmp) do
      table.insert(previews, p)
    end
  end
  -- if #previews == 0 then
  --   log('W', 'scenarios', 'scenario has no previews: ' .. tostring(scenarioData.scenarioName))
  -- end
  return previews
end

local function getLevels(subdirectory)
  local levelList = core_levels.getLevelNames()
  local levels = {}

  for _, levelName in ipairs(levelList) do
    local path = '/levels/' .. levelName .. '/scenarios/' .. subdirectory
    local foundFiles =  FS:findFiles(path, '*.json', -1, true, false)

    local busScenarios = {}
    for i,filepath in ipairs(foundFiles) do
      local ext = string.match(filepath, "(%.%w+)")
      if ext == '.json' then
        table.insert(busScenarios, filepath)
      end
    end

    -- TODO: make this more generic. Perhaps think about how it can be applied to different "Job" types
    if (#busScenarios > 0) then
      local newLevel = {}
      newLevel.levelName = levelName
      newLevel.levelInfo = jsonReadFile('/levels/'..levelName..'/info.json') -- this contains the level info for the UI!
      newLevel.official = isOfficialContentVPath('/levels/'..levelName..'/info.json')
      newLevel.previews = customPreviewLoader(newLevel, levelName)

      newLevel.scenarios = {}

      -- hardcoded for now...
      local busLineFiles = FS:findFiles('/levels/'.. levelName .. '/buslines/', '*.buslines.json', -1, true, false)
      local routes = {}
      for _, file in pairs(busLineFiles) do
        local busLine = jsonReadFile(file)
        for _, route in pairs(busLine.routes) do
          -- For now we assume there is only one bus scenario therefore
          -- we just use this as a 'template' for each route.
          local scenario = loadScenario(busScenarios[1])
          -- assign scenario name to route direction
          scenario.name = route.routeID .. ' ' .. route.direction
          -- check if starting position for the line exists
          if route.spawnLocation then
            scenario.spawnLocation = route.spawnLocation
          end

          if route.previews then
            scenario.previews ="/levels/".. levelName .."/buslines/" .. route.previews[1]
          end

          if route.vehicle then
            scenario.userSelectedVehicle = route.vehicle
          end

          if route.tasklist then
            scenario.stopCount = 0;
            for _, task in pairs(route.tasklist) do
              scenario.stopCount = scenario.stopCount + 1
            end
          end

          -- scenario.busdriver.simulatePassengers = true
          scenario.busdriver.strictStop = true
          scenario.busdriver.traffic = false
          scenario.busdriver.routeID = route.routeID
          scenario.busdriver.variance = route.variance
          table.insert(newLevel.scenarios, scenario)
        end
      end
      table.insert(levels, newLevel)
    end
  end

  return levels
end


-- public interface
M.getLevels                       = getLevels
M.getList                         = getList
M.loadScenario                    = loadScenario
M.processScenarioData             = processScenarioData
M.start                           = start
M.startByPath                     = startByPath
M.load                            = load
M.onFileChanged                   = onFileChanged

return M
