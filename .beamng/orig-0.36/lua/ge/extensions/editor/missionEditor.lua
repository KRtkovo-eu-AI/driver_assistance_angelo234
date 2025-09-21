-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local ffi = require("ffi")
local im = ui_imgui
local toolWindowName = "mission_editor"
M.dependencies = {'gameplay_missions_missions'}

-- form backend
local generalInfo
local issuesWindow
local startCondition
local devEditor
local visibleCondition
local startTrigger
local prefabs
local missionTypeData
local additionalAttributes
local setupModules
local careerSetup
local objectives
local progressSingle
local progressMulti
local playbookUtils
local unsavedColor = im.ImVec4(1, 0.3, 0.1, 1.0)
local colorLightRed = im.ImVec4(1, 0.8, 0.7, 1.0)
local colorLightGreen = im.ImVec4(0.5, 1.0, 0.8, 1.0)
local windows = {}
local tabs = {}
local showWindows = {
  issuesWindow = false,
  generalWindows = true,
  additionalWindows = true,
  setupModulesWindows = true,
  missionTypeWindows = true
}
local generalWindows, additionalWindows, setupModulesWindows, missionTypeWindows = {}, {}, {}, {}
local missionTypeWindow = {}
local oldMissionTypeData = {}

-- Add mission start position editor
local missionStartPositionEditor = require('/lua/ge/extensions/editor/missionStartPositionEditor')

local filter = {
  onlyCurrentLevel = false,
  hideProcedural = true,
  showCareerMissions = false,
  showFreeroamMissions = false,
}
local filterNamesSorted = {
  {name = "Current Level Only", key = "onlyCurrentLevel"},
  {name = "Hide Procedural Missions", key = "hideProcedural"},
  {name = "Hide non-Career Missions", key = "showCareerMissions"},
  {name = "Hide non-Freeroam Missions", key = "showFreeroamMissions"}
}

local grouping = {
  mode = "none",
  missions = {}
}
local groupingNamesSorted = {
  {name = "None", key = "none"},
  {name = "Level", key = "level"},
  {name = "Mission Type", key = "type"},
  {name = "Date", key = "date"},
  {name = "Branch", key = "branch"},
}


local sort

local hoveredMission
local clickedMission
local newMissionData = {
  name = im.ArrayChar(1024, "Name"),
  id = im.ArrayChar(1024, "Id"),
  level = "gridmap",
  type = "flowgraph",
  copy = false,
  autoId = true
}
local missionList = nil
local filterInput = im.ArrayChar(1024,'')

local missionSearch = require('/lua/ge/extensions/editor/util/searchUtil')()
local missionSearchTxt = im.ArrayChar(256, "")
local missionSearchDisplayResult = false
local missionSearchResults = {}

local function getShortId(id)
  local p, fn, _ = path.split(id)
  return fn
end


local function openRaceEditor(shownMission)
  if editor_raceEditor then
    if not editor.active then
      editor.setEditorActive(true)
    end
    editor_raceEditor.show()

    local folder = shownMission.missionFolder
    local raceFname = folder .. '/race.race.json'
    log('D', logTag, 'opening RaceEditor with raceFname='..raceFname)

    if not FS:fileExists(raceFname) then
      jsonWriteFile(raceFname, {}, true)
    end

    editor_raceEditor.loadRace(raceFname)
  end
end

local function openRallyEditor(shownMission)
  if editor_rallyEditor then
    if not editor.active then
      editor.setEditorActive(true)
    end
    editor_rallyEditor.loadForMissionEditor(shownMission.missionFolder)
    editor_rallyEditor.showRallyTool()
  end
end

local lastShownMission = nil -- always force an update on first call
local function displayHeader(clickedMission, hoveredMission, shownMission)
  if shownMission then
    if shownMission._dirty then
      if editor.uiIconImageButton(editor.icons.save, im.ImVec2(40, 40), unsavedColor) then
        gameplay_missions_missions.saveMission(shownMission,shownMission.missionFolder)
        shownMission._dirty = false
      end
      ui_flowgraph_editor.tooltip("Save unsaved changes for this mission")
    else
      if editor.uiIconImageButton(editor.icons.save, im.ImVec2(40, 40)) then end
      ui_flowgraph_editor.tooltip("No unsaved changes for this mission")
    end
    im.SameLine()
    local playerVehicle = getPlayerVehicle(0)
    local instance = gameplay_missions_missions.getMissionById(shownMission.id)
    local canTeleport = (getCurrentLevelIdentifier() == shownMission.startTrigger.level) and playerVehicle and shownMission.startTrigger
    if not canTeleport then im.BeginDisabled() end
    if editor.uiIconImageButton(editor.icons.flag, im.ImVec2(40, 40)) then
      -- check if map loaded and player has vehicle
      -- TODO: load map and player vehicle if not loaded

      -- teleport vehicle
      spawn.safeTeleport(playerVehicle,vec3(shownMission.startTrigger.pos), quat(shownMission.startTrigger.rot))

      instance.unlocks.startable = true
      instance.unlocks.visible = true
      gameplay_rawPois.clear()

      -- deactivate editor
      editor.setEditorActive(false)

      -- exitBigMap
      freeroam_bigMapMode.exitBigMap(true,true)

      gameplay_missions_missionScreen.setPreselectedMissionId(shownMission.id)
      guihooks.trigger('MenuOpenModule','mission-details')

      -- TODO: open mission popup
      -- TODO: switch to this mission in cluster
    end
    if not canTeleport then im.EndDisabled() end
    if canTeleport then
      im.tooltip("Teleports you to the mission start trigger, make mission visible and startable and closes the editor.")
    else
      im.tooltip("Either wrong level loaded, no vehicle for player, or no start trigger for mission.")
    end

    im.SameLine()
    if editor.uiIconImageButton(instance.unlocks.startable and editor.icons.assignment_turned_in or editor.icons.assignment_late, im.ImVec2(40, 40), instance.unlocks.startable and colorLightGreen or colorLightRed) then
      instance.unlocks.startable = not instance.unlocks.startable
      gameplay_rawPois.clear()
    end
    im.tooltip("Mission is currently ".. (instance.unlocks.startable and "startable" or "not startable (locked)".. " (click to toggle)"))

    im.SameLine()
    if editor.uiIconImageButton(instance.unlocks.visible and editor.icons.visibility or editor.icons.visibility_off, im.ImVec2(40, 40), instance.unlocks.visible and colorLightGreen or colorLightRed) then
      instance.unlocks.visible = not instance.unlocks.visible
      gameplay_rawPois.clear()
    end
    im.tooltip("Mission is currently ".. (instance.unlocks.visible and "visible" or "not visible") .. " (click to toggle)")

    im.SameLine()
    -- Add button to edit start position
    if editor.uiIconImageButton(editor.icons.edit_location, im.ImVec2(40, 40)) then
      missionStartPositionEditor.setSelectedMission(shownMission)
      editor.selectEditMode(editor.editModes.missionStartPositionEditMode)
    end
    im.tooltip("Edit mission start position")

    im.SameLine()
    if shownMission.missionType == 'rallyStage' then
      if editor.uiIconImageButton(editor.icons.simobject_bng_waypoint, im.ImVec2(40, 40)) then
        openRaceEditor(shownMission)
      end
      im.tooltip("Open Race Editor")
      im.SameLine()

      if editor.uiIconImageButton(editor.icons.import_contacts, im.ImVec2(40, 40)) then
        openRallyEditor(shownMission)
      end
      im.tooltip("Open Rally Editor")
      im.SameLine()
    end
  end
  if shownMission then
    im.HeaderText(translateLanguage(shownMission.name, shownMission.name, true))
    im.SameLine()
    im.SetCursorPosY(im.GetCursorPosY() + 6)
    im.Text(shownMission.id)
  else
    im.BeginDisabled()
    im.Text("(no mission selected)")
    im.EndDisabled()
  end
end
local initializeColumnWidth = true
local function loadWindows()
  -- general info
  generalInfo = require('/lua/ge/extensions/editor/missionEditor/general')(M)
  table.insert(windows, generalInfo)
  table.insert(generalWindows, generalInfo)
  --devEditor = require('/lua/ge/extensions/editor/missionEditor/dev')(M)

  issuesWindow = require('/lua/ge/extensions/editor/missionEditor/issues')(M)
  table.insert(windows, issuesWindow)

  --prefabs = require('/lua/ge/extensions/editor/missionEditor/prefabs')(M)
  local preview = require('/lua/ge/extensions/editor/missionEditor/previewChecker')(M)
  table.insert(windows, preview)
  table.insert(generalWindows, preview)

  startTrigger = require('/lua/ge/extensions/editor/missionEditor/startTrigger')(M)
  table.insert(windows, startTrigger)
  table.insert(generalWindows, startTrigger)

  careerSetup = require('/lua/ge/extensions/editor/missionEditor/careerSetup')(M)
  table.insert(windows, careerSetup)
  table.insert(generalWindows, careerSetup)

  -- additional Info
  startCondition = require('/lua/ge/extensions/editor/missionEditor/conditions')(M, 'startCondition','Start Condition')
  table.insert(windows, startCondition)
  table.insert(generalWindows, startCondition)

  visibleCondition = require('/lua/ge/extensions/editor/missionEditor/conditions')(M, 'visibleCondition','Visibility Conditions')
  table.insert(windows, visibleCondition)
  table.insert(generalWindows, visibleCondition)

  additionalAttributes = require('/lua/ge/extensions/editor/missionEditor/additionalAttributes')(M)
  table.insert(windows, additionalAttributes)
  table.insert(additionalWindows, additionalAttributes)

  local layers = require('/lua/ge/extensions/editor/missionEditor/layers')(M)
  table.insert(windows, layers)
  table.insert(additionalWindows, layers)

  -- setup modules
  setupModules = require('/lua/ge/extensions/editor/missionEditor/setupModules')(M)
  table.insert(windows, setupModules)
  table.insert(setupModulesWindows, setupModules)

  -- missionType Info
  missionTypeWindow = require('/lua/ge/extensions/editor/missionEditor/genericTypeData')(M)
  table.insert(windows, missionTypeWindow)
  table.insert(missionTypeWindows, missionTypeWindow)


  objectives = require('/lua/ge/extensions/editor/missionEditor/objectives')(M)
  table.insert(windows, objectives)
  table.insert(tabs, objectives)

  progressSingle = require('/lua/ge/extensions/editor/missionEditor/progressSingle')(M)
  table.insert(windows, progressSingle)
  table.insert(tabs, progressSingle)

  progressMulti = require('/lua/ge/extensions/editor/missionEditor/progressMulti')(M)
  table.insert(windows, progressMulti)
  table.insert(tabs, progressMulti)

  playbookUtils = require('/lua/ge/extensions/editor/missionEditor/playbookUtils')(M)
  table.insert(windows, playbookUtils)
  table.insert(tabs, playbookUtils)
  --table.insert(windows, devEditor)
  --table.insert(windows, visibleCondition)
  --table.insert(windows, prefabs)
end
local infoSize = im.ImVec2(18, 18)
local infoColors = {
  check = im.ImVec4(0, 1, 0, 1.0),
  warning = im.ImVec4(1, 1, 0, 1.0),
  error = im.ImVec4(1, 0, 0, 1.0)
}



local function loadGroupFilter()
  local groupFilter = editor.getPreference("missionEditor.general.groupFilter") or {}
  missionList = nil
  if groupFilter.filter then
    for _, f in ipairs(filterNamesSorted) do
      if groupFilter.filter[f.key] ~= nil then
        filter[f.key] = groupFilter.filter[f.key]
      end
    end
  end
  if groupFilter.grouping ~= nil then
    grouping.mode = groupFilter.grouping
    editor.savePreferences()
  end
end

local function saveGroupFilter()
  local groupFilter = { filter = {}}
  for _, f in ipairs(filterNamesSorted) do
    groupFilter.filter[f.key] = filter[f.key]
  end
  groupFilter.grouping = grouping.mode
  editor.setPreference("missionEditor.general.groupFilter", groupFilter)
end


local function displayMissionSelector(missionData)
  if missionData._issueList then
    local icon = missionData._issueList.icon
    if missionData._issueList.importantCount < 10 and missionData._issueList.importantCount > 0 then
      icon = "filter_"..tostring(missionData._issueList.importantCount)
    end
    editor.uiIconImageButton(editor.icons[icon], infoSize, missionData._issueList.color)
    if im.IsItemHovered() then
      im.BeginTooltip()
      if missionData._issueList.importantCount == 0 then
        im.Text("No Issues!")
      end

      for _, issue in ipairs(missionData._issueList) do
        im.BulletText(issue.label)
      end
      im.EndTooltip()
    end

  else
    im.Dummy(infoSize)
  end
  im.SameLine()


  local name = missionData.id

  if editor.getPreference('missionEditor.general.shortIds') then
    local p, fn, _ = path.split(name)
    name = fn
    --[[
    if string.find(p or '', '/') then
      name = name .. ' (' .. p..')'
    end
    ]]
  end

  if missionData.devMission then
    name = name .. " [DEV]"
  end


  if missionData._dirty then
    name = "*** " .. name .. " ***"
    im.PushStyleColor2(im.Col_Text, unsavedColor)
  end
  if missionSearchDisplayResult then
    im.HighlightSelectable(name, ffi.string(missionSearchTxt), clickedMission == missionData)
  else
    im.Selectable1(name, clickedMission == missionData)
  end
  if im.IsItemClicked() then
    if clickedMission == missionData then
      clickedMission = nil
    else
      clickedMission = missionData
    end
  end
  if missionData._dirty then im.PopStyleColor() end
  if im.IsItemHovered() then
    hoveredMission = missionData
  end
  im.tooltip(missionData.id)
end


local filterFunctions = {
  onlyCurrentLevel = function(mission, f) return not f or mission.startTrigger.level == getCurrentLevelIdentifier() end,
  hideProcedural = function(mission, f) return not f or not mission.procedural end,
  showCareerMissions = function(mission, f) return not f or f and mission.careerSetup.showInCareer end,
  showFreeroamMissions = function(mission, f) return not f or f and mission.careerSetup.showInFreeroam end,
}
local function applyFilter()
  missionList = {}

  for _, mission in ipairs(gameplay_missions_missions.getFilesData() or {}) do
    local passed = true
    for key, fun in pairs(filterFunctions) do
      passed = passed and ( filterFunctions[key](mission, filter[key]))
    end
    if passed then
      table.insert(missionList, mission)
    end
  end

end

local function idSort(a,b) return a.id < b.id end
local function getMissionType(mission) return mission.missionType end
local function getMissionLevelOrNone(mission) return mission.startTrigger.level or "No Level" end
local function getMissionDateOrNone(mission) return mission.date and os.date('%Y-%m-%d', mission.date)  or "No Date Set!" end
local function getMissionBranchOrNone(mission) return string.format("%s", mission.careerSetup.skill or "No Skill") end
local function groupMissionsByFunction(missions, propertyFunction)
  local result = {missions = {}, sortedKeys = {}}
  for _, mission in ipairs(missions) do
    local key = propertyFunction(mission)
    if not result.missions[key] then
      result.missions[key] = {}
      table.insert(result.sortedKeys, key)
    end
    table.insert(result.missions[key], mission)
  end
  table.sort(result.sortedKeys)
  for key, list in pairs(result.missions) do
    table.sort(list, idSort)
  end
return result
end


local function applyGrouping()
  if grouping.mode == "none" then
    grouping.missions = missionList
    table.sort(grouping.missions, idSort)
  elseif grouping.mode == "type" then
    grouping = groupMissionsByFunction(missionList, getMissionType)
    grouping.mode = "type"
  elseif grouping.mode == "level" then
    grouping = groupMissionsByFunction(missionList, getMissionLevelOrNone)
    grouping.mode = "level"
  elseif grouping.mode == "date" then
    grouping = groupMissionsByFunction(missionList, getMissionDateOrNone)
    grouping.mode = "date"
  elseif grouping.mode == "branch" then
    grouping = groupMissionsByFunction(missionList, getMissionBranchOrNone)
    grouping.mode = "branch"
  end
end

local function applySearch()
  local s = ffi.string(missionSearchTxt)
  missionSearchDisplayResult = s:len() > 0
  if missionSearchDisplayResult then
    missionSearch:startSearch(s)
    for k,missionData in ipairs(missionList) do
      missionSearch:queryElement({
        name = missionData.id,
        score = 1,
        data = missionData
      })
    end
    missionSearchResults = missionSearch:finishSearch()
  end
end

local function newMissionPopup()
  if im.BeginPopup("NewMission") then
    --print("ok?")
    local scale = editor.getPreference("ui.general.scale")
    im.BeginChild1("createNewMissionPopupChild", im.ImVec2(350*scale,180*scale), 0)
    im.HeaderText("Create New Mission")
    im.PushStyleVar2(im.StyleVar_FramePadding, im.ImVec2(0,0))
    im.Columns(2)
    im.PopStyleVar()
    im.SetColumnWidth(0, im.CalcTextSize("MissionType").x +5)
    im.Text("Name: ")
    im.NextColumn()
    im.PushItemWidth(im.GetContentRegionAvailWidth())
    if editor.uiInputText("##NameNewMission", newMissionData.name, 1024) and newMissionData.autoId then
      newMissionData.recalcId = true
    end
    im.PopItemWidth()
    im.NextColumn()
    im.Text("Id: ")
    im.NextColumn()
    im.PushItemWidth(im.GetContentRegionAvailWidth()-26*scale)
    if editor.uiInputText("##idNewMission", newMissionData.id, 1024) then
      newMissionData.autoId = false
    end
    im.PopItemWidth()
    im.SameLine()
    local sameTypeLevelCount = 1
    local level = getCurrentLevelIdentifier() or "gridmap"

    local id = ffi.string(newMissionData.id)
    -- check if the id is already taken
    local taken = false
    for _, mission in ipairs(missionList or {}) do
      if mission.id == id then
        taken = true
      end

      if mission.missionType == newMissionData.type and mission.startTrigger.type == 'coordinates' and mission.startTrigger.level == level then
        sameTypeLevelCount = sameTypeLevelCount + 1
      end
    end

    if not taken then
      editor.uiIconImage(editor.icons.check, im.ImVec2(24,24), im.ImVec4(0,1,0,1))
      im.tooltip("This ID is available.")
    else
      editor.uiIconImage(editor.icons.error_outline, im.ImVec2(24,24), im.ImVec4(1,0,0,1))
      im.tooltip("This ID is already taken!")
    end
    im.NextColumn()
    im.Text("Auto Id")
    im.NextColumn()
    if im.Checkbox("##Automatic Id", im.BoolPtr(newMissionData.autoId)) then
      newMissionData.autoId = not newMissionData.autoId
    end

    if newMissionData.recalcId then
      local name = ffi.string(newMissionData.name)
      name = name:match("(.-) ") or name
      name = name:gsub('%W','') or name

      local id = string.format("%s/%s/%03d-%s", level, newMissionData.type, sameTypeLevelCount, name)
      newMissionData.id = im.ArrayChar(1024, id)
    end
    newMissionData.recalcId = false

    im.NextColumn()
    im.Text("Missiontype")
    im.NextColumn()
    im.PushItemWidth(im.GetContentRegionAvailWidth())
    if im.BeginCombo('##MissionType',newMissionData.type) then
      for _, mType in ipairs(missionTypeWindow.missionTypes) do
        if im.Selectable1(mType, mType == newMissionData.type) then
          newMissionData.type = mType
          newMissionData.recalcId = true
        end
        if im.IsItemHovered() then
          im.BeginTooltip()
          im.PushTextWrapPos(200 * editor.getPreference("ui.general.scale"))
          im.TextWrapped(gameplay_missions_missions.getMissionStaticData(mType)["description"] or "No Description")
          im.PopTextWrapPos()
          im.EndTooltip()
        end
      end
      im.EndCombo()
    end
    im.PopItemWidth()

    local cameraPosition = core_camera.getPosition()
    local position = core_camera.getQuat() * vec3(0, 15, 0)
    local beforeCam = position + cameraPosition

    --debugDrawer:drawSphere(beforeCam, 3, ColorF(1,0,0,0.5))
    im.Columns(1)

    if taken then
      im.BeginDisabled()
    end
    if im.Button("Create Mission", im.ImVec2(-1, -1)) then
      -- create mission and add into mission list
      local data = {
        name = ffi.string(newMissionData.name),
        id = ffi.string(newMissionData.id),
        missionType = newMissionData.type,
        startTrigger = {
          type = 'coordinates',
          level = getCurrentLevelIdentifier() or "gridmap",
          pos = (beforeCam):toTable(),
          radius = 3,
          rot = quat(0,0,0,1)
          },
        careerSetup = {
          showInFreeroam = true,
          }

        }
      local newMis

      if newMissionData.copy then
        newMis = gameplay_missions_missions.createMission(data.id, deepcopy(lastShownMission))
      else
        newMis = gameplay_missions_missions.createMission(data.id, data)
        missionTypeWindow:fillGeneric(newMis)
      end

      gameplay_missions_missions.saveMission(newMis)
      M.reloadMissionSystem()

      M.setMissionById(data.id)
      M.forceOpenTree = true
      im.CloseCurrentPopup()
    end
    if taken then
      im.EndDisabled()
    end
    im.EndChild()
    im.EndPopup()
  end
end
local openNewMissionPopup = false

local translationData = nil
local function makeTranslation()
  local translation = {}
  local prefix = ffi.string(translationData.translationKeyPtr)
  local instance = gameplay_missions_missions.getMissionById(clickedMission.id)
  table.insert(translation,{ key = prefix..".title",       value = clickedMission.name,       source = 'name' })
  table.insert(translation,{ key = prefix..".description", value = clickedMission.description, source = 'description' })
  local editHelper = missionTypeWindow:getCurrentEditorHelper()
  if editHelper then
    for _, elem in ipairs(editHelper.elements) do
      if elem.displayOptions and elem.displayOptions.isTranslation then
        if elem.type == 'string' then
          table.insert(translation,{key = prefix.."."..elem.fieldName, value = ffi.string(elem.ac), source = elem})
        end
      end
    end
  end
  local keys = deepcopy(instance.sortedStarKeys)
  table.insert(keys,"noStarUnlocked")
  for _, key in ipairs(keys) do
    local t = clickedMission.careerSetup.starOutroTexts[key]
    if t and t ~= "" then
      table.insert(translation,{ key = prefix..".starOutroTexts."..key,       value = t,       source = key, starText = true})
    end
  end
  for _, elem in ipairs(translation) do
    if translateLanguage(elem.value, "NoTranslation!") ~= "NoTranslation!" then
      elem.value = translateLanguage(elem.value, elem.value)
    end
  end

  local translationStrings = {}
  for _, elem in ipairs(translation) do
    table.insert(translationStrings,string.format('    "%s": "%s"', elem.key, elem.value))
  end

  translationData.copyPastaPtr = im.ArrayChar(100000, table.concat( translationStrings, ",\n")..",\n")
  translationData.translation = translation
end

local function exportBranchInfo()
  local csvdata = require('csvlib').newCSV("Branch ID", "Branch Name", "Branch Domain", "Branch Parent ID", "Branch Parent Domain")
end
local function exportMissionsWithStars()
  local branchNames = {}
  for _, branch in ipairs(career_branches.getSortedBranches()) do
    if not branch.showProgressAsStars then
      table.insert(branchNames, branch.attributeKey)
    end
  end
  if not career_modules_branches_leagues then
    extensions.load("career_modules_branches_leagues")

  end
  local csvdata = require('csvlib').newCSV("Mission ID", "Mission Name", "Mission Type", "Branch", "Leagues", "Tier", "Star Id", "Star Label", "Star Type", "money", "vouchers",unpack(branchNames))

  for _, mission in ipairs(missionList) do
    local instance = gameplay_missions_missions.getMissionById(mission.id)
    local translatedName = translateLanguage(mission.name, mission.name, true)
    local missionType = mission.missionType
    local firstBranch = mission.careerSetup.skill
    local leagues = career_modules_branches_leagues.getLeaguesForMission(mission.id)
    local leagueNames = {}
    for _, league in ipairs(leagues) do
      table.insert(leagueNames, league.id)
    end
    leagueNames = table.concat(leagueNames, ",")

    for _, key in ipairs(instance.careerSetup._activeStarCache.sortedStars) do
      local translatedStarLabel = instance.starLabels[key]
      if type(translatedStarLabel) == "function" then
        translatedStarLabel = instance.starLabels[key](instance, {})
      end

      if type(translatedStarLabel) == "string" then
        translatedStarLabel = translateLanguage(translatedStarLabel, translatedStarLabel, true)
      elseif type(translatedStarLabel) == "table" and translatedStarLabel.txt then
        translatedStarLabel = translateLanguage(translatedStarLabel.txt, translatedStarLabel.txt, true)
      else
        translatedStarLabel = "Star..."
      end
      local rewards = {}
      for _, r in ipairs(instance.careerSetup._activeStarCache.sortedStarRewardsByKey[key] or {}) do
        rewards[r.attributeKey] = r.rewardAmount
      end
      local branchRewards = {}
      for _, branch in ipairs(branchNames) do
        table.insert(branchRewards, rewards[branch] or 0)
      end
      local starType = instance.careerSetup._activeStarCache.defaultStarKeysByKey[key] and "default" or "bonus"
      csvdata:add(mission.id, translatedName, missionType, firstBranch, leagueNames, instance.unlocks.maxBranchlevel, key, translatedStarLabel, starType, rewards.money or 0,rewards.vouchers or 0,unpack(branchRewards))
    end
  end
  csvdata:write("missionOverview.csv")
  Engine.Platform.exploreFolder("missionOverview.csv")
end


local function exportMissionsSimple()
  local csvdata = require('csvlib').newCSV("Mission ID", "Mission Name", "Mission Type", "Level", "Gamemode")
  local levelNameById = {}

  for _, mission in ipairs(missionList) do
    local instance = gameplay_missions_missions.getMissionById(mission.id)
    local translatedName = translateLanguage(mission.name, mission.name, true)
    local missionType = mission.missionType
    local level = mission.id:match("^([^/]+)")
    local gamemode = instance.careerSetup.showInCareer and "Career" or "Freeroam"
    csvdata:add(mission.id, translatedName, missionType, level, gamemode)
  end
  csvdata:write("missionOverview.csv")
  Engine.Platform.exploreFolder("missionOverview.csv")
end

local function exportContentOverview()
  local csvdata = require('csvlib').newCSV("Folder", "Name","Date","Origin","Map","Type 1","Type 2", "Type 3")

  for _, mission in ipairs(missionList) do
    local instance = gameplay_missions_missions.getMissionById(mission.id)
    local translatedName = translateLanguage(mission.name, mission.name, true)
    local type2 = ""
    local origin = "Mission"
    if mission.procedural and mission.missionType == "busMode" then origin = "Bus Route" end
    if mission.procedural and mission.missionType == "generatedTimeTrial" then origin = "Time Trials" end

    -- Convert timestamp to human-readable date (YYYY-MM-DD format)
    local dateStr = "-"
    if mission.date and mission.date > 0 then
      dateStr = os.date("%Y-%m-%d", mission.date)
    end

    local folder = mission.missionFolder or "None"
    csvdata:add(folder, translatedName, dateStr, origin, mission.startTrigger and mission.startTrigger.level or "None", mission.missionType, mission.careerSetup.showInFreeroam and "Freeroam" or "Career")
  end

  for _, scenario in ipairs(scenario_scenariosLoader.getList()) do
    local origin = "Scenario"
    if scenario.restrictToCampaign then origin = "Campaign" end

    local type2 = ""
    if scenario.isCreatedFromFlowgraph or scenario.flowgraph then type2 = "Flowgraph" end

    -- Convert timestamp to human-readable date (YYYY-MM-DD format)
    local dateStr = "-"
    if scenario.date and tonumber(scenario.date) and tonumber(scenario.date) > 0 then
      dateStr = os.date("%Y-%m-%d", tonumber(scenario.date))
    end

    if not scenario.isMissionAsScenario then
      csvdata:add(scenario.sourceFile, translateLanguage(scenario.name, scenario.name, true), dateStr, origin, string.lower(scenario.levelName), type2)
    end
  end

--  for _, qr in ipairs(scenario_quickRaceLoader.getQuickraceList()) do
--    csvdata:add(translateLanguage(qr.scenarioName, qr.scenarioName, true), tonumber(qr.date), "Time Trial")
--  end

  csvdata:write("content.csv")
  Engine.Platform.exploreFolder("missionOverview.csv")
end


local function fixStartTriggerForLeagues()

  local validSkills = {}
  for _, branch in ipairs(career_branches.getSortedBranches()) do
    if branch.parentDomain == "apm" then
      validSkills[branch.id] = true
    end
    if branch.parentId then
      local parent = career_branches.getBranchById(branch.parentId)
      if parent.parentDomain == "apm" or parent.id == "apm" then
        validSkills[branch.id] = true
      end
    end
  end

  for _, mission in ipairs(missionList) do
    local instance = gameplay_missions_missions.getMissionById(mission.id)
    local missionType = mission.missionType
    dump(string.format("%s - %s", mission.careerSetup.skill, mission.id))
    if mission.careerSetup.showInCareer and validSkills[mission.careerSetup.skill] then
      dump(string.format("%s - %s", mission.name, mission.id))
      mission.startTrigger = {}
      mission.startTrigger.type = "league"
      mission.startTrigger.level = "west_coast_usa"
      mission._dirty = true
    end
  end
end

local ignoreAttributeKeys = {
  money = true,
  vouchers = true,
}
local function removeNonAttributeStarRewards()
  print("Removing non-attribute star rewards...")
  for _, mission in ipairs(missionList) do
    local shortId = getShortId(mission.id)
    local validXP = mission.careerSetup.skill
    local branch = career_branches.getBranchById(validXP)
    while branch and not branch.missing and branch.showProgressAsStars do
      validXP = branch.parentId
      branch = career_branches.getBranchById(validXP)
    end
    if validXP == "(none)" then
      print(string.format(" > Skipped mission %s: invalid XP via %s", shortId, mission.careerSetup.skill))
    else
      for key, list in pairs(mission.careerSetup.starRewards) do
        local newRewards = {}
        local containedAttributes = {}
        local hasAddedValidXP = false
        local invalidXP = nil
        shortId = getShortId(mission.id) .. "-" .. key
        for _, reward in ipairs(list) do
          if ignoreAttributeKeys[reward.attributeKey] then
            table.insert(newRewards, reward)
          else
            if validXP ~= reward.attributeKey then
              print(string.format("For mission %s: removed %s (only allowed XP is %s via %s)", shortId, reward.attributeKey, validXP, mission.careerSetup.skill))
              mission._dirty = true
              invalidXP = reward
            else
              local branch = career_branches.getBranchById(reward.attributeKey)
              if not branch.missing and not branch.showProgressAsStars then
                if not containedAttributes[reward.attributeKey] then
                  table.insert(newRewards, reward)
                  containedAttributes[reward.attributeKey] = true
                else
                  print(string.format("For mission %s: removed %s (duplicate)", shortId, reward.attributeKey))
                  mission._dirty = true
                end
              else
                print(string.format("For mission %s: removed %s", shortId, reward.attributeKey))
                mission._dirty = true
              end
            end
          end
        end
        if not hasAddedValidXP and invalidXP then
          print(string.format(" ! For mission %s: Fixing To %s (from %s)", shortId, validXP, invalidXP.attributeKey))
          invalidXP.attributeKey = validXP
          table.insert(newRewards, invalidXP)
          hasAddedValidXP = true
        end
        mission.careerSetup.starRewards[key] = newRewards
      end
    end
  end
end

local function escapeCSV(s)
  if string.find(s, '[,"]') then
    s = '"' .. string.gsub(s, '"', '""') .. '"'
  end
  return s
end
local function loadMissionCSV()
  local f = io.open('missionImport.csv')
  local header = f:read()
  local data = {}
  local keys = {}
  for x in header:gmatch("([^',']+)") do table.insert(keys, x) end
  dump(keys)
  local row = f:read()
  while row do
    local values, idx = {}, 1
    for x in row:gmatch("([^,]+)") do
      dump(x)
      --print(idx, x)
      local val = tonumber(x) or x
      values[keys[idx]] = val
      idx = idx + 1
    end
    table.insert(data, values)
    row = f:read()
  end
  f:close()
  return data
end

local function importMissionOverview()
  local data = loadMissionCSV()

  local branchNames = {}
  for _, branch in ipairs(career_branches.getSortedBranches()) do
    if not branch.showProgressAsStars then
      table.insert(branchNames, branch.attributeKey)
    end
  end

  local fDataById = {}
  for _, m in ipairs(gameplay_missions_missions.getFilesData()) do
    fDataById[m.id] = m
  end
  for _, row in ipairs(data) do
    local mission = fDataById[row['Mission ID']]
    if mission then
      local starKey = row['Star Id']
      local rewards = {}
      for _, key in ipairs({"money", "vouchers",unpack(branchNames)}) do
        if row[key] ~= 0 then
          table.insert(rewards,{attributeKey = key, rewardAmount = row[key]})
        end
      end
      mission.careerSetup.starRewards[starKey] = rewards
      mission._dirty = true
    end
  end

end

local function getTranslatableStrings()

  --[[
  for _, mission in ipairs(missionList) do
    mission.careerSetup.showInFreeroam = false
    mission.careerSetup.showInCareer = true
    mission.date = 1663575239
    mission.author = "BeamNG"
    mission._dirty = true

  end
  local addedKeys = {}
  local lastCount = 0
  local finalList = {}
  for _, mission in ipairs(missionList) do

    local translation = {}
    local instance = gameplay_missions_missions.getMissionById(mission.id)
    table.insert(translation,mission.name)
    table.insert(translation,mission.description)
    local editHelper = missionTypeWindow:getCurrentEditorHelper()
    if editHelper then
      for _, elem in ipairs(editHelper.elements) do
        if elem.displayOptions and elem.displayOptions.isTranslation then
          if elem.type == 'string' then
            table.insert(translation,ffi.string(elem.ac))
          end
        end
      end
    end
    local keys = deepcopy(instance.sortedStarKeys or {})
    table.insert(keys,"noStarUnlocked")
    for _, key in ipairs(keys) do
      local t = mission.careerSetup.starOutroTexts[key]
      if t and t ~= "" then
        table.insert(translation,t)
      end
      local d = instance.defaultStarOutroTexts[key]
      if d and d ~= "" then
        table.insert(translation,d)
      end
      local s = instance.starLabels[key]
      if s and s ~= "" then
        table.insert(translation,s)
      end
    end
    local count = 0
    for _, elem in ipairs(translation) do
      if elem ~= "" and translateLanguage(elem, "NoTranslation!") ~= "NoTranslation!" then
        if not addedKeys[elem] then
          table.insert(finalList,{key = elem, value = translateLanguage(elem, elem)})
          count = count +1
        end
        addedKeys[elem] = true
      end
    end
    print(mission.id .. " -> " ..importantCount)
  end



  local wordCount = 0
  local translationStrings = {}
  local translationJson = {}
  for _, elem in ipairs(finalList) do
    table.insert(translationStrings,string.format('"%s": "%s"', elem.key, elem.value))
    translationJson[elem.key] = elem.value
    local _,n = elem.value:gsub("%S+","")
    wordCount = wordCount + n
  end
  dump("Words: " .. wordCount)
  writeFile("translationText.json",table.concat( translationStrings, ",\n")..",\n")
  jsonWriteFile("translationJson.json",translationJson, true)
  ]]
end

local function applyTranslation()
  for _, elem in ipairs(translationData.translation) do
    if elem.starText then
      clickedMission.careerSetup.starOutroTexts[elem.source] = elem.key
    elseif elem.source == 'name' then
      clickedMission.name = elem.key
    elseif elem.source == 'description' then
      clickedMission.description = elem.key
    else
      if elem.source.type == 'string' then
        elem.source.ac = im.ArrayChar(elem.source.len, elem.key)
        clickedMission.missionTypeData[elem.source.fieldName] = elem.key
      end
    end
  end

  for _, win in ipairs(windows) do
    win:setMission(clickedMission)
  end
  clickedMission._dirty = true
end




local function missionTranslationHelperPopup()
  if im.BeginPopup("missionTranslationHelper") then
    if translationData == nil then
      local shortId = getShortId(clickedMission.id)
      local level = clickedMission.startTrigger and clickedMission.startTrigger.level or "noLevel"
      translationData = {
        copyPastaPtr = im.ArrayChar(100000),
        copyPastaLength = 100000,
        translationKeyPtr = im.ArrayChar(1000,string.format("missions.%s.%s.%s",clickedMission.missionType, level, shortId)),
        translationKeyLength = 1000
      }
      makeTranslation()
    end
    if im.Button("Change Auto Key",im.ImVec2(-1,0)) then
      local level = clickedMission.startTrigger and clickedMission.startTrigger.level or "noLevel"
      local shortId = nil
      local f,t =  string.find(clickedMission.id, "-[^-]*$")
      if not f or not t then
        local p, fn, _ = path.split(clickedMission.id)
        shortId = fn
      else
        shortId = clickedMission.id:sub(f+1,t)
      end
      translationData.translationKeyPtr = im.ArrayChar(1000,string.format("missions.%s.%s.%s",level, clickedMission.missionType, shortId))
    end
    im.InputText("Key:##translationKey", translationData.translationKeyPtr, translationData.translationKeyLength)
    if im.Button("Make Translation",im.ImVec2(-1,0)) then
      makeTranslation()
    end
    im.InputTextMultiline("##translationCopyPasta", translationData.copyPastaPtr, translationData.copyPastaLength, im.ImVec2(500,500))
    if im.Button("Replace Mission Data with Keys",im.ImVec2(-1,0)) then
      applyTranslation()
    end
    im.EndPopup()
  else
    translationData = nil
  end
end


-- display window
local function onEditorGui()
  if editor.beginWindow(toolWindowName, "Mission Editor",  im.WindowFlags_MenuBar) then

    if not generalInfo then
      loadWindows()
    end
    if im.BeginMenuBar() then
      if im.BeginMenu("File") then
        if im.MenuItem1("New Mission...") then
          openNewMissionPopup = true
          newMissionData.name = im.ArrayChar(1024, "Name")
          newMissionData.id = im.ArrayChar(1024, "Id")
          newMissionData.type = "flowgraph"
          newMissionData.copy = false
          newMissionData.autoId = true
          newMissionData.recalcId = true
        end
        local saveable = lastShownMission ~= nil
        if not saveable then im.BeginDisabled() end
        if im.MenuItem1("Save") then
          gameplay_missions_missions.saveMission(lastShownMission,lastShownMission.missionFolder)
          lastShownMission._dirty = false
        end
        if not saveable then im.EndDisabled() end
        if saveable then
          im.tooltip(dumps(lastShownMission.id) .. " to " .. lastShownMission.missionFolder .. "/info.json")
        end
        local changedMissioncount = 0
        for _, m in ipairs(missionList or {}) do
          if m._dirty then changedMissioncount = changedMissioncount + 1 end
        end
        if im.MenuItem1("Save All ("..changedMissioncount..")") then
          for _, m in ipairs(missionList) do
            if m._dirty then
              gameplay_missions_missions.saveMission(m,m.missionFolder)
              m._dirty = false
            end
          end
        end
        im.Separator()
        if im.BeginMenu("Delete") then
          if not saveable then im.BeginDisabled() end
          local files = {}
          local dirs = {}
          if lastShownMission then
            for _, f in ipairs(FS:findFiles(lastShownMission.missionFolder, "*", -1, true, false)) do
              table.insert(files, f)
              local dir, filename, ext =path.split(string.sub(f, 2+string.len(lastShownMission.missionFolder)))
              if dir then
                table.insert(dirs, dir)
              end
            end
          end
          im.Text("Contains " .. #files.." files and " .. #dirs .. " directories:")
          table.sort(files)
          table.sort(dirs)
          for _, f in ipairs(files) do
            im.Text(f)
          end
          im.Separator()
          for _, d in ipairs(dirs) do
            im.Text(d)
          end
          im.Separator()
          if im.MenuItem1("PERMANENTLY delete mission") then
            for _, f in ipairs(files) do
              FS:removeFile(f)
            end
            table.sort(dirs, function(a,b) return a>b end)
            for _, d in ipairs(dirs) do
              FS:directoryRemove(lastShownMission.missionFolder .. '/'..d)
            end
            FS:directoryRemove(lastShownMission.missionFolder)
            local newMissions = {}
            for _, m in ipairs(missionList) do
              if m.id ~= lastShownMission.id then
                table.insert(newMissions, m)
              end
            end
            missionList = newMissions
            lastShownMission = nil
            hoveredMission = nil
            clickedMission = nil

          end
          if lastShownMission then im.tooltip("Deletes the folder " .. lastShownMission.missionFolder.."!") end
          if not saveable then im.EndDisabled() end
          im.EndMenu()

        end
        if im.MenuItem1("Reload Mission System " .. (changedMissioncount > 0 and ("(" .. changedMissioncount .. " unsaved missions!)") or "")) then
          M.reloadMissionSystem()
        end
        if im.MenuItem1("(Debug) Print Mission Dependency Tree") then
          local currentMission = lastShownMission
        end
        im.EndMenu()
      end
      local variableClicked, translationClicked, timeUpdaterClicked = false, false, false
      if im.BeginMenu("Tools") then
        if im.MenuItem1("Calculate Mission Issues") then
          issuesWindow:calculateMissionIssues(missionList, windows, missionTypeWindow)
          issuesWindow:showIssuesWindow()
        end
        im.tooltip("Can take a few seconds.")
        if im.MenuItem1("Show All Mission Issues") then
          issuesWindow:showIssuesWindow()
        end
        im.Separator()
        if not clickedMission then im.BeginDisabled() end
        if im.BeginMenu("Custom Editor Functions...") then
          local helper = M.getCurrentEditorHelperWhenActive()
          if helper then
            local sortedFunctions = tableKeysSorted(helper)
            for _, funName in ipairs(sortedFunctions) do
              if string.startswith(funName, "customFunction") then
                if im.MenuItem1(funName) then
                  helper[funName](helper, clickedMission, M)
                  for _, win in ipairs(windows) do
                    win:setMission(clickedMission)
                  end
                end
                if im.MenuItem1("All Mission of same type: " ..funName) then
                  for _, m in ipairs(missionList) do
                    if m.missionType == clickedMission.missionType then
                      helper[funName](helper, m, M)
                    end
                  end
                  for _, win in ipairs(windows) do
                    win:setMission(clickedMission)
                  end
                end
              end
            end

          end
          im.EndMenu()
        end
        im.Separator()

        if im.MenuItem1("Flowgraph Variable Check") then
          variableClicked = true
        end
        if im.MenuItem1("Translation Helper") then
          translationClicked = true
        end
        if im.MenuItem1("Translation List (Debug)") then
          getTranslatableStrings()
        end
        if not clickedMission then im.EndDisabled() end
        im.Separator()
        if clickedMission and im.BeginMenu("Generate Attempts") then
          if not clickedMission then im.BeginDisabled() end
          if im.MenuItem1("Generate 25 Attempts") then
            gameplay_missions_progress.generateAttempts(clickedMission.id, 25, true)
          end
          local instance = gameplay_missions_missions.getMissionById(clickedMission.id)
          for _, key in ipairs(instance.sortedStarKeys or {}) do
            if im.MenuItem1("Generate one attempt with star: " .. key) then
              gameplay_missions_progress.generateAttempt(clickedMission.id, {unlockedStars = {[key..''] = true}})
            end
          end

          if not clickedMission then im.EndDisabled() end
          if im.MenuItem1("Generate 1-5 Attempts for all Missions") then
            gameplay_missions_progress.startBatchMode()
            for _, mission in ipairs(missionList) do
              gameplay_missions_progress.generateAttempts(mission.id, 1 + math.floor(math.random()*4))
            end
            gameplay_missions_progress.endBatchMode()
            M.reloadMissionSystem()
          end
          im.tooltip("This will a few seconds, check the log for progress!")

          if im.MenuItem1("Generate 1-5 Attempts for random half of all Missions") then
            gameplay_missions_progress.startBatchMode()
            for _, mission in ipairs(missionList) do
              if math.random() < 0.5 then
                gameplay_missions_progress.generateAttempts(mission.id, 1 + math.floor(math.random()*4))
              end
            end
            gameplay_missions_progress.endBatchMode()
            M.reloadMissionSystem()
          end
          im.tooltip("This will a few seconds, check the log for progress!")
          im.EndMenu()
        end

        if im.MenuItem1("Export Missions with Stars") then
          exportMissionsWithStars()
        end
        if im.MenuItem1("Export Missions Simple") then
          exportMissionsSimple()
        end
        if im.MenuItem1("Import Mission Overview") then
          importMissionOverview()
        end
        if im.MenuItem1("Export Content Overview") then
          exportContentOverview()
        end
        if im.MenuItem1("Fix Start Trigger For Leagues") then
          fixStartTriggerForLeagues()
        end
        if im.MenuItem1("Remove non-attribute star rewards") then
          removeNonAttributeStarRewards()
        end
        im.Separator()
        if im.MenuItem1("Time Updater") then
          timeUpdaterClicked = true
        end
        im.EndMenu()
      end
      if variableClicked then missionTypeWindow:openPopup() end
      if translationClicked then im.OpenPopup("missionTranslationHelper") end
      if timeUpdaterClicked then additionalAttributes:openTimeUpdater() end

      if openNewMissionPopup then
        im.OpenPopup("NewMission")
        openNewMissionPopup = nil
      end
      newMissionPopup()
      if missionTypeWindow and missionTypeWindow.variablesHelperPopup then
        missionTypeWindow:variablesHelperPopup()
      end
      missionTranslationHelperPopup()
      if additionalAttributes and additionalAttributes.timeUpdaterPopup then
        additionalAttributes:timeUpdaterPopup()
      end
      im.EndMenuBar()
    end
    if missionList == nil then
      applyFilter()
      applyGrouping()
    end
    -- no missions indicator


    if missionTypeWindow.drawViews then
      missionTypeWindow:drawViews()
    end

    im.Columns(2)
    if initializeColumnWidth then
      im.SetColumnWidth(0, 420)
      initializeColumnWidth = nil
    end
    local missionListWidth = im.GetColumnWidth(0) -15
    local filterHeight = im.GetTextLineHeightWithSpacing()
    local winHeight = im.GetContentRegionAvail().y - filterHeight
    local yOrigin = im.GetCursorPos().y
    -- mission list ----
    local areaWidth = missionListWidth
    im.BeginChild1("Search",im.ImVec2(areaWidth, filterHeight+2), false)

    if editor.uiInputSearch(nil, missionSearchTxt, (areaWidth-40) ) then
      applyFilter()
      applySearch()
    end
    im.SameLine()

    local clr = nil
    local groupFilterText = {}
    if grouping.mode ~= "none" then
      table.insert(groupFilterText, "Grouped by: " .. grouping.mode)
    end
    for _, fl in ipairs(filterNamesSorted) do
      if filter[fl.key] then
        table.insert(groupFilterText, fl.name)
      end
    end

    if editor.uiIconImageButton(editor.icons.ab_filter_default, im.ImVec2(21,21), next(groupFilterText) and im.ImVec4(0,1,0,1)) then
      im.OpenPopup("MissionEditorGroupFilter")
      --grouping.mode = "type"
      --applyGrouping()
    end
    if im.IsItemHovered() then
      im.BeginTooltip()
      for _, elem in ipairs(groupFilterText) do
        im.BulletText(elem)
      end
      if not next(groupFilterText) then
        im.BulletText("No Filter active!")
      end
      im.EndTooltip()
    end


    if im.BeginPopup("MissionEditorGroupFilter") then
      im.BeginChild1("MissionEditorGroupFilterChild", im.ImVec2(300,200))
      im.PushItemWidth(im.GetContentRegionAvailWidth())
      if im.BeginCombo("##GroupMissions", "Grouping: " .. grouping.mode) then
        for _, gr in ipairs(groupingNamesSorted) do
          if im.Selectable1(gr.name, gr.key == grouping.mode) then
            grouping.mode = gr.key
            applyFilter()
            applyGrouping()
            saveGroupFilter()
            im.CloseCurrentPopup()
          end
        end
        im.EndCombo()
      end
      im.PopItemWidth()
      for _, fl in ipairs(filterNamesSorted) do
        local bl = im.BoolPtr(filter[fl.key])
        if im.Checkbox(fl.name, bl) then
          filter[fl.key] = not filter[fl.key]
          applyFilter()
          applySearch()
          applyGrouping()
          saveGroupFilter()
        end
      end
      if im.Checkbox('Short IDs', im.BoolPtr(editor.getPreference('missionEditor.general.shortIds'))) then
        editor.setPreference('missionEditor.general.shortIds', not editor.getPreference('missionEditor.general.shortIds'))
        editor.savePreferences()
      end
      im.EndChild()
      im.EndPopup()
    end


    im.EndChild()
    im.BeginChild1("missionList", im.ImVec2(areaWidth, -1), true)
      -- header
    local displayed = false
    if missionSearchDisplayResult then
      for _, item in ipairs(missionSearchResults) do
        displayMissionSelector(item.data)
        displayed = true
      end
    else
      if grouping.mode == "none" then
        for k,missionData in ipairs(missionList) do
          displayMissionSelector(missionData)
          displayed = true
        end
      else
        for _, key in ipairs(grouping.sortedKeys) do
          displayed = true
          local prevOther = nil
          if M.forceOpenTree then
            for k,missionData in ipairs(grouping.missions[key]) do
              if missionData.id == (hoveredMission or clickedMission or {}).id then
                im.SetNextItemOpen(true, im.Cond_Once)
              end
            end
          end
          if im.TreeNode1(key.." ("..#grouping.missions[key]..")"..'##'..key.."missionEditorGrouping") then
            for k,missionData in ipairs(grouping.missions[key]) do
              local other = nil
              if grouping.mode == 'type' then
                other = getMissionLevelOrNone(missionData)
              elseif grouping.mode == 'level' then
                other = getMissionType(missionData)
              end
              if other and other ~= prevOther then
                if prevOther then
                  im.Separator()
                end
                if editor.getPreference('missionEditor.general.shortIds') or grouping.mode == 'level' or grouping.mode == 'type' then
                  im.BeginDisabled()
                  im.Text("  "..other)
                  im.EndDisabled()
                end
              end
              displayMissionSelector(missionData)
              prevOther = other
            end
            im.TreePop()
          end
        end
        M.forceOpenTree = nil
      end
    end
    if not displayed then
      im.Text("No missions were found!")
    end


    if im.IsItemHovered() then
      hoveredMission = nil
    end
    if not editor.keyModifiers.alt then
      hoveredMission = nil
    end

    local shownMission = hoveredMission or clickedMission
    if shownMission ~= lastShownMission then
      lastShownMission = shownMission
      shownMission = shownMission or nil -- default fill
      if shownMission then
        for _, win in ipairs(windows) do
          win:setMission(shownMission)
        end
      end
    end
    im.EndChild()

    im.NextColumn()
    displayHeader(clickedMission, hoveredMission, shownMission)
    if im.BeginTabBar('MissionEditorTabBar##') then
      local selectTab = nil
      if M.forceOpenTab then
        if M.forceOpenTab == 'Mission Properties' then
          selectTab = im.TabItemFlags_SetSelected
        end
      end
      if im.BeginTabItem('Mission Properties', nil, selectTab) then
        M.lastTabItemShown = 'Mission Properties'
        -- mission details ----
        local areaWidth = im.GetWindowContentRegionWidth() - im.GetCursorPos().x + 15
        im.BeginChild1("missionDetails", im.ImVec2(areaWidth, winHeight-45), false, 0)

        -- details
        if shownMission then
          if shownMission.procedural then
            --im.BeginDisabled()
          end
          if shownMission._issueList then
            im.HeaderText("Issues: "..shownMission._issueList.importantCount)
            if im.IsItemClicked() then
              showWindows.issuesWindow = not showWindows.issuesWindow
              --editor.setPreference("missionEditor.general.showWindows", showWindows)
              --editor.savePreferences()
            end
            if showWindows.issuesWindow then
              im.PushID1("issues" ..shownMission.id)
              issuesWindow:draw()
              im.PopID()
              im.Separator()
            else
              im.SameLine()
              im.HeaderText("(...)")
              im.Separator()
            end
          end

          im.HeaderText("General Info")
          if im.IsItemClicked() then
            showWindows.generalWindows = not showWindows.generalWindows
            editor.setPreference("missionEditor.general.showWindows", showWindows)
            editor.savePreferences()
          end
          if showWindows.generalWindows then
            for idx, win in ipairs(generalWindows) do
              im.PushID1("generalWindows" .. idx.."_"..shownMission.id)
              win:draw()
              im.PopID()
              im.Separator()
            end
          else
            im.SameLine()
            im.HeaderText("(...)")
            im.Separator()
          end

          im.HeaderText("Additional Info")
          if im.IsItemClicked() then
            showWindows.additionalWindows = not showWindows.additionalWindows
            editor.setPreference("missionEditor.general.showWindows", showWindows)
            editor.savePreferences()
          end
          if showWindows.additionalWindows then
            for idx, win in ipairs(additionalWindows) do
              im.PushID1("additionalWindows" .. idx.."_"..shownMission.id)
              win:draw()
              im.PopID()
              im.Separator()
            end
          else
            im.SameLine()
            im.HeaderText("(...)")
            im.Separator()
          end



        else
          im.Text("No Mission Selected.")
        end
        im.EndChild()
        im.EndTabItem()
      end
      selectTab = nil
      if M.forceOpenTab then
        if M.forceOpenTab == 'Setup Modules' then
          selectTab = im.TabItemFlags_SetSelected
        end
      end
      if im.BeginTabItem('Setup Modules', nil, selectTab) then
        M.lastTabItemShown = 'Setup Modules'
        local areaWidth = im.GetWindowContentRegionWidth() - im.GetCursorPos().x + 15
        im.BeginChild1("missionDetails", im.ImVec2(areaWidth, winHeight-45), false, 0)
        if shownMission then

          im.HeaderText("Setup Modules")
          if im.IsItemClicked() then
            showWindows.setupModulesWindows = not showWindows.setupModulesWindows
            editor.setPreference("missionEditor.general.showWindows", showWindows)
            editor.savePreferences()
          end
          if showWindows.setupModulesWindows then
            for idx, win in ipairs(setupModulesWindows) do
              im.PushID1("setupModulesWindows" .. idx.."_"..shownMission.id)
              win:draw()
              im.PopID()
              im.Separator()
            end
          else
            im.SameLine()
            im.HeaderText("(...)")
            im.Separator()
          end
        else
          im.Text("No Mission Selected.")
        end
        im.EndChild()
        im.EndTabItem()
      end
      selectTab = nil
      if M.forceOpenTab then
        if M.forceOpenTab == 'Mission Type Data' then
          selectTab = im.TabItemFlags_SetSelected
        end
      end
      if im.BeginTabItem('Mission Type Data', nil, selectTab) then
        M.lastTabItemShown = 'Mission Type Data'
        local areaWidth = im.GetWindowContentRegionWidth() - im.GetCursorPos().x + 15
        im.BeginChild1("missionDetails", im.ImVec2(areaWidth, winHeight-45), false, 0)
        if shownMission then
          im.HeaderText("Mission Type Data")
          if im.IsItemClicked() then
            showWindows.missionTypeWindows = not showWindows.missionTypeWindows
            editor.setPreference("missionEditor.general.showWindows", showWindows)
            editor.savePreferences()
          end
          if showWindows.missionTypeWindows then
            for idx, win in ipairs(missionTypeWindows) do
              im.PushID1("missionTypeWindows" .. idx.."_"..shownMission.id)
              win:draw()
              im.PopID()
              im.Separator()
            end
          else
            im.SameLine()
            im.HeaderText("(...)")
          end
          if shownMission.procedural then
            --im.EndDisabled()
          end
        else
          im.Text("No Mission Selected.")
        end
        im.EndChild()
        im.EndTabItem()
      end
      for _, tab in ipairs(tabs) do
        selectTab = nil
        if M.forceOpenTab then
          if M.forceOpenTab == tab.tabName then
            selectTab = im.TabItemFlags_SetSelected
          end
        end
        if im.BeginTabItem(tab.tabName, nil, selectTab) then
          M.lastTabItemShown = tab.tabName
          local areaWidth = im.GetWindowContentRegionWidth() - im.GetCursorPos().x
          im.BeginChild1(tab.tabName.."windowhild", im.ImVec2(areaWidth, winHeight-45), false, 0)
          if shownMission then
            local mission = gameplay_missions_missions.getMissionById(shownMission.id)
            if mission then
              tab:draw()
            end
          else
            im.Text("No Mission Selected.")
          end
          im.EndChild()
          im.EndTabItem()
        end
      end
      M.forceOpenTab = nil
      im.EndTabBar()
    end
    im.Columns(1)
  end
  editor.endWindow()
  if issuesWindow then
    issuesWindow:drawIssuesWindow()
  end
  hoveredMission = nil
end


local function onWindowMenuItem()
  editor.showWindow(toolWindowName)
  loadGroupFilter()
end

local function onEditorInitialized()
  editor.registerWindow(toolWindowName, im.ImVec2(1500,700))
  editor.registerWindow("mission_issues", im.ImVec2(700,700))
  editor.addWindowMenuItem("Mission Editor", onWindowMenuItem, {groupMenuName="Missions"})
  loadGroupFilter()

  local _showWindows = editor.getPreference("missionEditor.general.showWindows")
  -- if the value didn't exist in the preferences, use the default bool by skipping the item
  for k, v in pairs(_showWindows) do
    if v ~= nil then
      showWindows[k] = v
    end
  end
end


local function onEditorRegisterPreferences(prefsRegistry)
  prefsRegistry:registerCategory("missionEditor")
  prefsRegistry:registerSubCategory("missionEditor", "general", "General",
  {
    -- {name = {type, default value, desc, label (nil for auto Sentence Case), min, max, hidden, advanced, customUiFunc, enumLabels}}
    {groupFilter = {"table", {}, "", nil, nil, nil, true}},
    {showWindows = {"table", deepcopy(showWindows), "", nil, nil, nil, true}},
    {shortIds = {"bool", false, "Use Short Ids", nil, nil, nil, false}},
    {alwaysShowScreenshots = {'bool', false, "", nil, nil, nil, true}},

})
end


local function onSerialize()
  local data = {
    lastSelectedMissionId = lastShownMission and lastShownMission.id,
    lastTabItemShown = M.lastTabItemShown
  }
  return data
end

local function onDeserialized(data)
  if data then
    if data.lastSelectedMissionId then
      M.setMissionById(data.lastSelectedMissionId)
      M.forceOpenTree = true
    end
    if data.lastTabItemShown then
      M.forceOpenTab = data.lastTabItemShown
    end
  end
end

M.onSerialize = onSerialize
M.onDeserialized = onDeserialized

M.onEditorInitialized = onEditorInitialized
M.onEditorRegisterPreferences = onEditorRegisterPreferences
M.onEditorGui = onEditorGui
M.getMissionList = function() return missionList end
M.clickMission = function(m) clickedMission = m end
M.show = onWindowMenuItem
M.getStartTriggerWindow = function() return startTrigger end
M.getMissionTypeWindow = function() return missionTypeWindow end
M.setMissionById = function(id, instant)
  if missionList == nil then
    missionList = gameplay_missions_missions.getFilesData() or {}
  end
  for k,missionData in ipairs(missionList) do
    if missionData.id == id then
      clickedMission = missionData
      M.forceOpenTree = true
      if instant then
        for _, win in ipairs(windows) do
          win:setMission(clickedMission)
        end
      end
      return
    end
  end
end

M.getCurrentEditorHelperWhenActive = function()
  if not editor.isWindowVisible(toolWindowName) then return nil end
  if not missionTypeWindow or not missionTypeWindow.mission then return nil end
  return missionTypeWindow:getCurrentEditorHelper()
end

M.reloadMissionSystem = function()
  gameplay_missions_missions.reloadCompleteMissionSystem()
  missionList = gameplay_missions_missions.getFilesData()

  local oldId = lastShownMission and lastShownMission.id
  lastShownMission = nil
  hoveredMission = nil
  clickedMission = nil
  if oldId then
    for _,m in ipairs(missionList) do
      if m.id == oldId then
        clickedMission = m
        for _, win in ipairs(windows) do
          win:setMission(m)
        end
      end
    end
  end
  applyFilter()
  applySearch()
  applyGrouping()
  saveGroupFilter()
end

M.onConsoleLog = function(timer, lvl, origin, line)
  print(timer)
end

M.getSelectedMissionId = function()
  return clickedMission
end

return M
