-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- extensions.gameplay_statistic.setDebug(true)

local M = {}
M.dependencies = {"ui_imgui","core_gamestate"}
local im = ui_imgui

local realtimer = 0
local simtimer = 0

local fileName = "/settings/cloud/gameplay_stat.json"
local fileData = nil

local fileDataCareer = nil
local currentActivity = nil
local currentLevel = nil
local currentMod = nil
local timers = {}
local windowOpen = im.BoolPtr(false)
local initialWindowSize = im.ImVec2(300, 100)
local max = math.max
local min = math.min
local callbacks = {}
local callbacksCareer = {}
local currentPlayerVehicleId = nil
local currentPlayerVehicleObj = nil
local statSchedule = {}
local lenstatSchedule=0
local currentStatSchedule=1

local function _loadData()
  -- log("E","load","load!!!!!!")
  local l = jsonReadFile(fileName)
  if l and l.entries and type(l.entries)=="table" and l.version then
    fileData = l
  else
    fileData = {version=1, entries={}}
  end
end

local function _saveData()
  -- log("E","save","!!!!!!!!!!!!!!!!!!!!")
  if jsonWriteFile(fileName..".tmp",fileData) then
    FS:removeFile(fileName)
    FS:renameFile(fileName..".tmp", fileName )
  else
    log("E","save","failed to write json!")
  end
  -- career saving has been moved to onSaveCurrentSaveSlot
end

local function onInit()
  _loadData()
end

local function _save()
  --log("E","_save","!!!!!!!!!!!!!!!!!!!!")
  M.forceTimerUpdate()
  _saveData()
end

local function onExit()
  if currentActivity then
    M.onClientEndMission()
  end
  _save()
end

local function onExtensionUnloaded()
  _save()
end

local function onSerialize()
  _save()
  local data = {}
  data.currentActivity = currentActivity
  data.currentLevel = currentLevel
  data.currentMod = currentMod
  data.windowOpen = windowOpen[0]
  data.realtimer = realtimer
  data.simtimer = simtimer
  return data
end

local function onDeserialized(data)
  if data.currentActivity then currentActivity = data.currentActivity end
  if data.currentLevel then currentLevel = data.currentLevel end
  if data.currentMod then currentMod = data.currentMod end
  if data.realtimer then realtimer = data.realtimer end
  if data.simtimer then simtimer = data.simtimer end
  if data.windowOpen ~= nil then windowOpen[0] = data.windowOpen end
  if data.currentLevel then M.onClientStartMission() end
  if fileDataCareer then
    M.onGameStateUpdate()
  end
end

local function addSchedule(fn)
  statSchedule[#statSchedule +1] = fn
  lenstatSchedule = #statSchedule
end

local function removeSchedule(fn)
  for i in ipairs(statSchedule) do
    if statSchedule[i] == fn then
      table.remove(statSchedule, i)
      lenstatSchedule = #statSchedule
      return true
    end
  end
  return false
end

local function loadSubmodules(data)
  statSchedule = {}
  currentPlayerVehicleObj = getPlayerVehicle(0) or nil
  if currentPlayerVehicleObj then
    currentPlayerVehicleId = currentPlayerVehicleObj:getId()
  end

  --iterate over all files within subdir: gameplay/statisticModules
  --load each of them, wait for registerModule call, then unload them
  local moduleDir = "lua/ge/extensions/gameplay/statisticModules"
  local moduleFiles = FS:findFiles(moduleDir, "*.lua", -1, true, false)
  if moduleFiles then
    for _, filePath in ipairs(moduleFiles) do
      local _, file, _ = path.split(filePath)
      local fileName = file:sub(1, -5)
      local extensionPath = "gameplay/statisticModules/" .. fileName
      extensions.load(extensionPath)
      local extensionName = "gameplay_statisticModules_" .. fileName
      if extensions[extensionName] then
        -- log("E","ld",dumps(extensions[extensionName]));
        addSchedule(extensions[extensionName].workload)
      end
    end
  end
end


local function _runCallback(name, oldentry, newentry, career)
  local cbs = nil
  if career then
    cbs = callbacksCareer[name]
  else
    cbs = callbacks[name]
  end
  -- log("E", "_runCallback "..name, dumps(cbs).."\t career="..dumps(callbacksCareer[name]).."\t gen="..dumps(callbacks[name]))
  if not cbs then return end
  local nval = deepcopy(newentry)
  nval.career = career

  for cbindex in pairs(cbs) do
    if not cbs[cbindex].trigger or newentry.value >= cbs[cbindex].trigger then
      cbs[cbindex].func(name, oldentry, nval)
      if cbs[cbindex] and cbs[cbindex].trigger then --recheck because can call remove
        cbs[cbindex] = nil
      end
    end
  end
end

local function _metricAdd(entry,name,value,aggregate)
  if not entry then
    entry = {value=value}
    if aggregate then
      entry.last=value
      entry.count=1
      entry.min=value
      entry.max=value
    end
    return entry
  else
    --greg had some odo corupted
    entry.value = (value or 0) + (entry.value or 0)
  end
  if aggregate then
    entry.last = value
    entry.count = (entry.count or 0) + 1
    entry.min= min(entry.min or math.huge, value)
    entry.max= max(entry.max or -math.huge, value)
  end
  return entry
end

local function metricAdd(name,value,aggregate)
  if name==nil then log("E", "metricAdd", "invalid metric name") return end
  if value==nil then log("E", "metricAdd", "invalid value metric="..dumps(name)) return end
  local oldEntry = deepcopy(fileData.entries[name])
  local r = _metricAdd(fileData.entries[name],name,value,aggregate)
  if not oldEntry then
    fileData.entries[name] = r
  end
  _runCallback(name,oldEntry,r,false)

  if fileDataCareer then
    oldEntry = deepcopy(fileDataCareer.entries[name])
    r = _metricAdd(fileDataCareer.entries[name],name,value,aggregate)
    if not oldEntry then
      fileDataCareer.entries[name] = r
    end
    _runCallback(name,oldEntry,r,true)
  end
end

local function _metricSet(entry,name,value,aggregate)
  if not entry then
    entry = {value=value}
    if aggregate then
      entry.count=1
      entry.min=value
      entry.max=value
    end
    return entry
  else
    entry.value = value
  end
  if aggregate then
    entry.count = (entry.count or 0) + 1
    entry.min= min(entry.min or math.huge, value)
    entry.max= max(entry.max or -math.huge, value)
  end
  return entry
end

local function metricSet(name,value,aggregate)
  if name==nil then log("E", "metricSet", "invalid metric name") return end
  if value==nil then log("E", "metricSet", "invalid value") return end
  local oldEntry = deepcopy(fileData.entries[name])
  fileData.entries[name] = _metricSet(fileData.entries[name],name,value,aggregate)
  _runCallback(name,oldEntry,fileData.entries[name],false)

  if fileDataCareer then
    oldEntry = deepcopy(fileDataCareer.entries[name])
    local r = _metricSet(fileDataCareer.entries[name],name,value,aggregate)
    if not oldEntry then
      fileDataCareer.entries[name] = r
    end
    _runCallback(name,oldEntry,r,true)
  end
end

local function metricGet(name, carreer)
  local entry = nil
  if fileDataCareer and carreer then
    entry = fileDataCareer.entries[name]
  else
    entry = fileData.entries[name]
  end
  if not entry then
    return nil
  else
    return deepcopy(entry)
  end
end

local function timerStart(name, increment, aggregate, useSimTime)
  if name==nil then log("E", "timerStart", "invalid timer name") return end
  if useSimTime==nil then useSimTime=false end
  if increment==nil then increment=true end
  if aggregate==nil then aggregate=false end
  if timers[name] then
    log("W","timerStart", "Timer "..dumps(name).." already started. will be ignored")
    return
  end
  timers[name] = {sim=useSimTime,increment=increment,start= (useSimTime and simtimer or realtimer),aggregate=aggregate}
end

local function _timerSave(name)
  local value = (timers[name].sim and simtimer or realtimer) - timers[name].start
  if timers[name].increment then
    metricAdd(name, value, timers[name].aggregate)
  else
    metricSet(name, value, timers[name].aggregate)
  end
  timers[name].start = (timers[name].sim and simtimer or realtimer)
  return value
end

local function timerStop(name)
  if not timers[name] then
    log("W","timerStop", "Timer "..dumps(name).." not started. will be ignored")
    return 0
  end
  local value = _timerSave(name)
  timers[name] = nil
  return value
end

local function callbackRegister(name, trigger, callbackFunction, career)
  local entry = {}
  if career then
    if not callbacksCareer[name]then
      callbacksCareer[name]= {}
    end
  else
    if not callbacks[name] then
      callbacks[name]= {}
    end
  end

  entry.trigger = trigger
  entry.func = callbackFunction
  if career then
    callbacksCareer[name][tostring(trigger)..tostring(callbackFunction)] = entry
    log("I","",string.format("Registered callback for %s at %0.2f", name, trigger))
  else
    callbacks[name][tostring(trigger)..tostring(callbackFunction)] = entry
  end
end

local function callbackRemove(name, trigger, callbackFunction, career)
  local target
  if career then
    target = callbacksCareer[name]
  else
    target = callbacks[name]
  end
  if not target then
    log("E", "callbackRemove", "no callback registered for `"..dumps(name).."`")
    return false
  end

  for e in pairs( target ) do
    if target[e].trigger == trigger and target[e].func == callbackFunction then
      target[e] = nil
      return true
    end
  end
  log("E", "callbackRemove", "callback not found for `"..dumps(name).."`")
  return false
end

-- ############################

local function _levelPath(fullpath)
  local dir, filename, ext = path.split(fullpath)
  if dir ~= nil then
    return string.lower(string.gsub(dir, "/levels/(.*)/", "%1"))
  end
end

local function endActivity()
  if not currentActivity then return end--log("E","endAct","no activity"); return end
  if timers[currentActivity] then
    timerStop(currentActivity)
  end
  currentActivity = nil
end

local function startActivity(levelPath)
  local levelFolder = getCurrentLevelIdentifier()
  -- if not levelPath then
  --   levelFolder =
  --   log("E","1", "levelFolder="..dumps(levelFolder))
  --   log("E","2", "getMissionFilename="..dumps(getMissionFilename()))
  -- else
  --   levelFolder = _levelPath(levelPath)
  -- end
  if currentMod and timers["general/mode/"..currentMod..".time"] then
    timerStop("general/mode/"..currentMod..".time")
    currentMod = nil
  end
  if currentLevel and timers["general/map/"..currentLevel..".time"] then
    timerStop("general/map/"..currentLevel..".time")
    currentLevel = nil
  end
  if not levelFolder then
    -- log("E","startActivity", "levelFolder is null!!!!!")
    if currentActivity then
      endActivity()
    end

    return
  end
  local missionId
  if gameplay_missions_missionManager then
    missionId = gameplay_missions_missionManager.getForegroundMissionId()
    -- log("I","startAct", "missionId="..dumps(missionId))
    --small_island/crawl/002-CTrialsA
  end
  local activityType
  local activityDetail = ""
  if missionId then
    activityType = "scenario"
    activityDetail = "/fg"..missionId:sub(levelFolder:len()+1):gsub("/","@")
  elseif scenario_scenarios and scenario_scenarios.getScenario() then
    local scenario = scenario_scenarios.getScenario()
    if scenario.scenarioName then
      activityType = "scenario"
      activityDetail = "/"..scenario.scenarioName
    elseif scenario.isQuickRace then
      activityType = "timetrial"
    else
      activityType = "scenario"
      activityDetail = "/unknown"
    end
  elseif career_career and career_career.isActive() then
    activityType = "career"
  else
    activityType = extensions.core_gamestate.state.state
  end

  if currentActivity then
    endActivity()
  end
  if not activityType then
    if currentMod then
      activityType = currentMod
      log("W","startActivity","Assumed `activityType` didn't change")
    else
      log("E","startActivity","Could not determine activityType")
      return
    end
  end
  currentActivity = activityType .."/".. levelFolder .. activityDetail ..".time"
  timerStart(currentActivity)

  if activityType~= currentMod or not timers["general/mode/"..currentMod..".time"] then
    if currentMod and timers["general/mode/"..currentMod..".time"] then
      timerStop("general/mode/"..currentMod..".time")
    end
    currentMod = activityType
    timerStart("general/mode/"..currentMod..".time")
  end

  if levelFolder~= currentLevel or not timers["general/map/"..currentLevel..".time"]then
    if currentLevel and timers["general/mapmap/"..currentLevel..".time"]then
      timerStop("general/map/"..currentLevel..".time")
    end
    currentLevel = levelFolder
    timerStart("general/map/"..currentLevel..".time")
  end

  -- log("E","startAct", dump(currentActivity))
end

local function _callbackTest(name,old,new)
  log("E","_callbackTest", dumps(name).."\t"..dumps(old.value).."\t"..dumps(new.value).."\t"..dumps(new.career))
  callbackRegister("vehicle/burnout.time", new.value+2 ,_callbackTest, new.career)
end

local function onClientStartMission(levelPath)
  startActivity(levelPath)
end

local function onClientEndMission(levelPath)
  endActivity()
end

local function onClientPostStartMission(levelPath)
  startActivity(levelPath)

  loadSubmodules()
  -- callbackRegister("vehicle/burnout.time", 0,_callbackTest, true)
  -- callbackRegister("vehicle/burnout.time", 0,_callbackTest, false)
  -- print(callbackRemove("vehicle/burnout", 0,_callbackTest))
end

local function onWorldReadyState(worldReadyState)
  -- log("E","onWorldReadyState", "fgAct="..dumps(missionId))
end

local function onEditorActivated()
  timerStart("general/editor.time")
end

local function onEditorDeactivated()
  timerStop("general/editor.time")
end

local function onUiChangedState(toState, fromState)
  -- log("E","onUiChangedState", dumps(toState))
  if "scenario-start" == toState then
    -- startActivity(currentActivity.levelPath)
  end
end

local function onScenarioLoaded(scenario)
  -- local data = deepcopy(scenario)
  -- data.nodes = nil
  -- log("E","onScenarioLoaded", dumps(data))
  startActivity()
end

local function onScenarioFinished(scenario)
  -- local data = deepcopy(scenario)
  -- data.nodes = nil
  -- log("E","onScenarioLoaded", dumps(data))
  metricAdd("scenario/finished",1)
  startActivity()
end

local function onScenarioRestarted(scenario)
  -- local data = deepcopy(scenario)
  -- data.nodes = nil
  -- log("E","onScenarioLoaded", dumps(data))
  startActivity()
  metricAdd("scenario/restarted",1)
end

local function onGameStateUpdate(newState)
  if realtimer == 0 and fileDataCareer then
    return--fix start before onSerialize only in career
  end
  startActivity()
end

local function onUpdate(dtReal, dtSim, dtRaw)

  realtimer = realtimer + dtReal
  simtimer = simtimer + dtSim

  if currentPlayerVehicleId ~= -1 and lenstatSchedule>0 and currentPlayerVehicleObj and currentPlayerVehicleObj.playerUsable and currentPlayerVehicleObj:isPlayerControlled() and dtSim~=0 then
    statSchedule[currentStatSchedule](currentPlayerVehicleObj, currentPlayerVehicleId, dtSim)
    currentStatSchedule=currentStatSchedule +1
    if currentStatSchedule > lenstatSchedule then currentStatSchedule = 1 end
  end

  if windowOpen[0] ~= true then return end
  im.SetNextWindowSize(initialWindowSize, im.Cond_FirstUseEver)
  im.SetNextWindowPos(initialWindowSize, im.Cond_FirstUseEver)
  if( im.Begin("gameplay_statistic Debugger", windowOpen) ) then
    if im.BeginTabBar("shapeeditor##") then
      if im.BeginTabItem("module") then
        im.TextUnformatted("activity: "..dumps(currentActivity))
        im.TextUnformatted("mod: "..dumps(currentMod))
        im.TextUnformatted("level: "..dumps(currentLevel))
        if im.TreeNode1("Timers") then
          for k,v in pairs(timers) do
            im.TextUnformatted(k..": "..dumps(v))
          end
          im.TreePop()
        end
        im.EndTabItem()
      end
      if im.BeginTabItem("entries") then
        for k,v in pairs(fileData.entries) do
          if v.max then
            im.TextUnformatted(k..": "..dumps(v))
          else
            im.TextUnformatted(k..": v="..dumps(v.value))
          end
        end
        im.EndTabItem()
      end
      if im.BeginTabItem("entries career") then
        --im.TextUnformatted("save : "..dumps(type(fileNameCareer))..dumps(string.len(fileNameCareer or "") )..dumps(fileNameCareer))
        if fileDataCareer then
          for k,v in pairs(fileDataCareer.entries) do
            if v.max then
              im.TextUnformatted(k..": "..dumps(v))
            else
              im.TextUnformatted(k..": v="..dumps(v.value))
            end
          end
        end
        im.EndTabItem()
      end
      if im.BeginTabItem("callback") then
        if im.TreeNode1("callbacks general") then
          for k,v in pairs(callbacks) do
            im.TextUnformatted(k..": v="..dumps(v))
          end
          im.TreePop()
        end
        if im.TreeNode1("callbacks career") then
          for k,v in pairs(callbacksCareer) do
            im.TextUnformatted(k..": v="..dumps(v))
          end
          im.TreePop()
        end
        im.EndTabItem()
      end
      im.EndTabBar()
    end
  end
  im.End()
end

local function onVehicleSpawned(vid, v)
end

local function onVehicleSwitched(oldid, newid, player)
  if player==0 then
    currentPlayerVehicleId = newid
    if newid == -1 or not newid then
      currentPlayerVehicleObj = nil
    else
      currentPlayerVehicleObj = getObjectByID(newid)
    end
    metricAdd("vehicle/switch",1)
  end
end

local function onVehicleResetted(vid)
  if currentPlayerVehicleId==vid then metricAdd("vehicle/reset",1) end
end

local function onVehicleDestroyed(vid)
  if currentPlayerVehicleId==vid then
    currentPlayerVehicleId=-1
    currentPlayerVehicleObj = nil
  end
end

local function setDebug(newValue)
  if newValue then
    windowOpen[0] = true
  else
    windowOpen[0] = false
  end
end

local function sendGUIState()
  local data = {general= fileData.entries}
  if fileDataCareer then
    data.career = fileDataCareer.entries
  end
  guihooks.trigger('StatisticData', data )

end

local function onSaveCurrentSaveSlot(currentSavePath)
  -- save local career data into the saveslot folder
  M.forceTimerUpdate()
  local careerSaveFilePath = currentSavePath .. "/career/gameplay_stat.json"
  if not career_saveSystem.jsonWriteFileSafe(careerSaveFilePath,fileDataCareer) then
    log("E","save","failed to write career json!")
  end
end


local function onCareerActive(active)
  if active then
    -- load current saveslot stats
    local saveSlot, savePath = career_saveSystem.getCurrentSaveSlot()
    if not saveSlot then return end
    fileDataCareer = jsonReadFile(savePath.. "/career/gameplay_stat.json")
    -- if no stats are found, create empty data.
    if not fileDataCareer or not fileDataCareer.entries or type(fileDataCareer.entries)~="table" or not fileDataCareer.version then
      fileDataCareer = {version=1, entries={}}
    end
  else
    if fileDataCareer then
      _save()
    end
    -- if career is deactivated, remove local career data.
    fileDataCareer = nil
  end
end

local function onExtensionLoaded()
  loadSubmodules()
end

local function forceTimerUpdate()
  for k in pairs(timers) do
    _timerSave(k)
  end
end


M.onInit = onInit
M.onExit = onExit
M.onExtensionUnloaded = onExtensionUnloaded
M.onSerialize = onSerialize
M.onDeserialized = onDeserialized
M.onExtensionLoaded = onExtensionLoaded

M.onSaveCurrentSaveSlot = onSaveCurrentSaveSlot
M.onCareerActive = onCareerActive

M.onClientStartMission = onClientStartMission
M.onClientEndMission = onClientEndMission
M.onClientPostStartMission = onClientPostStartMission
M.onWorldReadyState = onWorldReadyState
M.onEditorActivated = onEditorActivated
M.onEditorDeactivated = onEditorDeactivated
-- M.onUiChangedState = onUiChangedState
M.onScenarioLoaded = onScenarioLoaded
M.onScenarioFinished = onScenarioFinished
M.onScenarioRestarted = onScenarioRestarted
M.onUpdate = onUpdate
-- M.onVehicleSpawned = onVehicleSpawned
M.onVehicleSwitched = onVehicleSwitched
M.onVehicleResetted = onVehicleResetted
M.onVehicleDestroyed = onVehicleDestroyed
M.onGameStateUpdate = onGameStateUpdate

M.sendGUIState = sendGUIState
M.forceTimerUpdate = forceTimerUpdate

M.metricAdd = metricAdd
M.metricSet = metricSet
M.metricGet = metricGet
M.timerStart = timerStart
M.timerStop = timerStop
M.callbackRegister = callbackRegister
M.callbackRemove = callbackRemove

M.addSchedule = addSchedule
M.removeSchedule = removeSchedule

M.setDebug = setDebug

return M