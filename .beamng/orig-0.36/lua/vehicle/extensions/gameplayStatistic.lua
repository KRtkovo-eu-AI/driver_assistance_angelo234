-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- extensions.gameplay_statistic.setDebug(true)

local M = {}

local simtimer = 0
local timers = {}
local statSchedule = {}
local lenstatSchedule=0
local currentStatSchedule=1

local function metricAdd(name,value,aggregate)
  if name==nil then log("E", "metricAdd", "invalid metric name") return end
  if value==nil then value=1 end
  obj:queueGameEngineLua("gameplay_statistic.metricAdd("..dumps(name)..","..tostring(value)..","..tostring(aggregate)..")")
end

local function metricSet(name,value,aggregate)
  if name==nil then log("E", "metricSet", "invalid metric name") return end
  obj:queueGameEngineLua("gameplay_statistic.metricSet("..dumps(name)..","..tostring(value)..","..tostring(aggregate)..")")
end

local function timerStart(name, increment, aggregate)
  if name==nil then log("E", "timerStart", "invalid timer name") return end
  if increment==nil then increment=true end
  if timers[name] then
    log("W","timerStart", "Timer "..dumps(name).." already started. will be ignored")
    return
  end
  timers[name] = {increment=increment,start=simtimer,aggregate=aggregate}
end

local function timerStop(name)
  local entry = timers[name]
  if not entry then
    log("W","timerStart", "Timer "..dumps(name).." not started. will be ignored")
    return 0
  end
  local value = simtimer - entry.start
  if entry.increment then
    metricAdd(name, value, entry.aggregate)
  else
    metricSet(name, value, entry.aggregate)
  end
  timers[name] = nil
  return value
end

local function timerGet(name)
  if not timers[name] then
    log("W","timerStart", "Timer "..dumps(name).." not started. will be ignored")
    return 0
  end
  return simtimer - timers[name].start
end

local function timerDelete(name)
  timers[name] = nil
end

local function updateGFX(dtsim)
  if not playerInfo.anyPlayerSeated and ai.mode ~= "disabled" then return end
  simtimer = simtimer + dtsim
  if lenstatSchedule>0 then
    statSchedule[currentStatSchedule]()
    currentStatSchedule=currentStatSchedule +1
    if currentStatSchedule > lenstatSchedule then currentStatSchedule = 1 end
  end
end

local function refreshTimer(timerName, value, minTime, aggregate)
  if timers[timerName] == nil then
    if value then
      timerStart(timerName,true, aggregate)
    end
  else
    if not value then
      if minTime then
        if minTime > timerGet(timerName) then timerDelete(timerName) return end
      end
      local val = timerStop(timerName)
    end
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

local function onExtensionLoaded()
  if wheels.wheelCount == 0 and not hydros.isPhysicsStepUsed() and not powertrain.isPhysicsStepUsed() then
    return false --unload
  end

  --iterate over all files within subdir: gameplayStatisticModules
  --load each of them, wait for registerModule call, then unload them
  local moduleDir = "lua/vehicle/extensions/gameplayStatisticModules"
  local moduleFiles = FS:findFiles(moduleDir, "*.lua", -1, true, false)
  if moduleFiles then
    for _, filePath in ipairs(moduleFiles) do
      local _, file, _ = path.split(filePath)
      local fileName = file:sub(1, -5)
      local extensionPath = "gameplayStatisticModules/" .. fileName
      extensions.load(extensionPath)
      local extensionName = "gameplayStatisticModules_" .. fileName
      if extensions[extensionName] then
        addSchedule(extensions[extensionName].workload)
      end
    end
  end
end


M.onExtensionLoaded = onExtensionLoaded
--M.onExtensionUnloaded = onExtensionUnloaded

M.metricAdd = metricAdd
M.metricSet = metricSet
M.timerStart = timerStart
M.timerStop = timerStop
M.timerGet = timerGet
M.timerDelete = timerDelete
M.refreshTimer = refreshTimer

M.addSchedule = addSchedule
M.removeSchedule = removeSchedule

M.updateGFX = updateGFX

return M