-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = 'specialTriggers'

M.active = true
M.debugMode = false

local triggers = {} -- main triggers table
local originalStates = {} -- original states of objects
local tickTimer = 0
local tickTime = 0.25

-- triggers can be BeamNGTriggers or Site Zones
-- triggers can refer to SimGroups containing statics, lights, and other objects
-- triggers can also execute Lua commands
-- when trigger state changes to true, toggles original state of all linked objects
-- if there are nested SimGroups, process them all in parallel (useful for statics that go with lights at the same time)

-- auto loads triggers.json if it exists in the level root directory
-- for an example, see west_coast_usa

local function setState(id, state) -- updates the state of a single object
  local obj = scenetree.findObjectById(id)
  if not obj then return end

  local field = (obj:getClassName() == 'PointLight' or obj:getClassName() == 'SpotLight') and 'isEnabled' or 'hidden' -- depends on class type
  local isLight = field == 'isEnabled'

  if not originalStates[id] then -- sets original states of objects, so that they can be reset
    if isLight then
      originalStates[id] = obj:getLightEnabled() and 1 or 0
    else
      originalStates[id] = obj.hidden and 1 or 0
    end
  end

  if state == nil then -- if state is undefined, assumes that the state should be reset
    if originalStates[id] then
      obj[field] = originalStates[id] == 1 and true or false
    end
  else
    if originalStates[id] == 1 then
      obj[field] = not state
    else
      obj[field] = state
    end
  end
end

local function setGroupState(id, state) -- sets the visible state of a single object or SimGroup
  local obj = scenetree.findObjectById(id or 0)
  if obj then
    if obj:getClassName() == 'SimGroup' then -- if object is a SimGroup, process all objects within it
      for _, o in ipairs(obj:getObjects()) do
        setState(tonumber(o) or scenetree.findObject(o):getID(), state)
      end
    else
      setState(id, state)
    end
  end
end

local function setTriggerObjects(key, state) -- instantly updates the visible states of linked objects
  if not key or not triggers[key] or not triggers[key].objects then return end

  for k, _ in pairs(triggers[key].objects) do
    if scenetree.findObject(k) then
      setGroupState(scenetree.findObject(k):getID(), state)
    end
  end
end

local function processTriggerObjects(key) -- processes objects and SimGroups for the trigger
  if not key or not triggers[key] or not triggers[key].objects then return end

  for k, v in pairs(triggers[key].objects) do
    local obj = scenetree.findObject(k)
    if obj then
      if obj:getClassName() == 'SimGroup' then
        v.isNested = true -- if nested SimGroup structure exists, process the children of the SimGroups, in parallel
        for _, o in ipairs(obj:getObjects()) do
          local obj = scenetree.findObjectById(tonumber(o) or scenetree.findObject(o):getID())
          if obj:getGroup():getName() == k and obj:getClassName() ~= 'SimGroup' then
            v.isNested = false
          end
        end

        if v.isNested then -- get the inner SimGroup with the most children, and use that value as the maxIdx
          v.maxIdx = 0
          for _, o in ipairs(obj:getObjects()) do
            local obj = scenetree.findObjectById(tonumber(o) or scenetree.findObject(o):getID())
            if obj:getGroup():getName() == k and obj:getClassName() == 'SimGroup' then
              v.maxIdx = math.max(v.maxIdx, obj:getCount())
            end
          end
        else
          v.maxIdx = obj:getCount()
        end
      else
        v.maxIdx = 0
      end
    end
  end
end

local function addTrigger(key, data) -- creates data for a trigger; "key" should be the trigger or zone name
  data = data or {}
  triggers[key] = {
    type = data.type or 'trigger', -- "trigger", "zone"
    triggers = data.triggers, -- optional array of multiple trigger names
    enabled = true,
    active = false,
    stack = 0,
    vehIds = {}, -- table of occupying vehicle ids
    subjectType = data.subjectType or 'player', -- "player", "notPlayer", "all"
    subjectIds = data.subjectIds, -- optional table of valid vehicle ids to check for
    enterCommand = data.enterCommand, -- GELua
    exitCommand = data.exitCommand,
    enterVehCommand = data.enterVehCommand, -- vLua
    exitVehCommand = data.exitVehCommand
  }

  -- table of linked scenetree objects or SimGroups
  if type(data.objects) == 'table' then
    triggers[key].objects = {}
    for k, v in pairs(data.objects) do
      triggers[key].objects[k] = {
        timer = -1,
        stepTimer = -1,
        currIdx = 0,
        maxIdx = 0,
        randomOrder = v.randomOrder and true or false,
        enterDelay = tonumber(v.enterDelay) or 0,
        exitDelay = tonumber(v.exitDelay) or 0,
        enterRandomMin = tonumber(v.enterRandomMin) or 0,
        enterRandomMax = tonumber(v.enterRandomMax) or 0,
        exitRandomMin = tonumber(v.exitRandomMin) or 0,
        exitRandomMax = tonumber(v.exitRandomMax) or 0
      }
    end
  end

  if data.triggers then -- creates aliases if there are multiple triggers assigned
    for _, tName in ipairs(data.triggers) do
      triggers[tName] = {alias = key, active = false}
    end
  end

  processTriggerObjects(key)
end

local function removeTrigger(key, useOrigState) -- removes data of a trigger
  if not key or not triggers[key] then return end
  if useOrigState then
    setTriggerObjects(key)
  end

  if triggers[key].triggers then
    for _, tName in ipairs(triggers[key].triggers) do
      triggers[tName] = nil
    end
  end

  triggers[key] = nil
end

local function reset(useOrigState) -- resets everything
  if useOrigState then
    for k, v in pairs(triggers) do
      removeTrigger(k, useOrigState)
    end
  end

  if next(triggers) then
    log('D', logTag, 'Special triggers resetted')
  end

  table.clear(originalStates)
end

local function setupTriggers(data) -- setup for the triggers table
  if not be or not data then return end
  reset(true) -- clears tables and resets states

  for k, v in pairs(data) do
    addTrigger(k, v)
  end

  log('D', logTag, 'Created '..tableSize(data)..' special triggers')
end

local function loadTriggers(filePath) -- loads triggers data from a file path
  if not filePath then
    local levelDir = path.split(getMissionFilename())
    if levelDir then filePath = levelDir..'triggers.json' end
  end

  if filePath then
    setupTriggers(jsonReadFile(filePath))
  end
end

local function getTriggers() -- returns table of special triggers
  return triggers
end

local function isVehicleValid(key, vehId) -- checks if vehicle is allowed for this trigger
  if not triggers[key] then return false end
  local trigger = triggers[key]
  local valid = false
  if scenetree.objectExistsById(vehId) and be:getObjectActive(vehId) then
    if (trigger.subjectIds and arrayFindValueIndex(trigger.subjectIds, vehId)) or
    (not trigger.subjectType or trigger.subjectType == 'all') or
    (trigger.subjectType == 'player' and vehId == be:getPlayerVehicleID(0)) or
    (trigger.subjectType == 'notPlayer' and vehId ~= be:getPlayerVehicleID(0)) then
      valid = true
    end
  end

  return valid
end

local function useTrigger(data) -- called whenever a trigger or zone detects an event
  local name = data.triggerName
  data.vehId = data.vehId or data.subjectID

  local tempTrigger
  if triggers[name] and triggers[name].alias then
    tempTrigger = triggers[name]
    name = triggers[name].alias
  end

  local trigger = triggers[name]
  if not trigger or not trigger.enabled then return end
  local active = data.event == 'enter' or data.event == 'tick'
  local valid = false

  if data.valid then
    valid = true
  else
    valid = isVehicleValid(name, data.vehId)
  end

  if valid and (trigger.vehIds[data.vehId] == nil or trigger.vehIds[data.vehId] ~= active) then -- checks if a state change occurred for this vehicle
    local obj = getObjectByID(data.vehId)
    if obj then
      if active and trigger.enterVehCommand then
        getObjectByID(data.vehId):queueLuaCommand(trigger.enterVehCommand)
      elseif not active and trigger.exitVehCommand then
        getObjectByID(data.vehId):queueLuaCommand(trigger.exitVehCommand)
      end
    end
    trigger.vehIds[data.vehId] = active
  end

  if not active then
    for _, vState in pairs(trigger.vehIds) do
      if vState then
        valid = false -- state change is not valid if other filtered vehicles are still within the trigger
      end
    end

    if valid and trigger.triggers then
      if tempTrigger then
        tempTrigger.active = false -- the active state of the calling trigger gets set to false here, but this is not a clean solution
      end

      for _, v in ipairs(trigger.triggers) do
        if triggers[v] and triggers[v].active then -- state change is not valid if other grouped triggers are active
          valid = false
        end
      end
    end
  end

  if valid and trigger.active ~= active then -- only updates if the active state changed
    trigger.active = active
    trigger.stack = 0

    if tempTrigger then
      tempTrigger.active = active -- this sets the active state of the calling trigger, if applicable
    end

    if M.debugMode then
      log('D', logTag, 'Updated state of '..name..': '..tostring(active))
    end

    if trigger.objects then
      for _, objData in pairs(trigger.objects) do
        objData.timer = 0
        objData.stepTimer = -1
        trigger.stack = trigger.stack + 1
      end
    end

    if data.event == 'enter' then
      if trigger.enterCommand then
        local fx = load(trigger.enterCommand)
        if fx then pcall(fx) end
      end
    elseif data.event == 'exit' then
      if trigger.exitCommand then
        local fx = load(trigger.exitCommand)
        if fx then pcall(fx) end
      end
    end
  end
end

local function setTriggerActive(tName, active, instant) -- manually sets a trigger active state
  -- the instant bool forces the timer to be ignored
  if not tName or not triggers[tName] then return end

  local data = triggers[tName]
  if active == nil then active = not data.active print(tostring(active)) end
  data.triggerName = tName
  data.event = active and 'enter' or 'exit'
  data.vehId = 0 -- untested, but this forces the trigger to stay active
  data.valid = true

  useTrigger(data)
  if instant then
    setTriggerObjects(tName, active) -- instant activation
    data.timer = -1
    data.stepTimer = -1
    data.stack = 0
  end
end

local function onTick() -- tick to check for zones, if applicable
  if not M.active or not gameplay_city then return end
  if true then return end

  local zones = gameplay_city.getSites() and gameplay_city.getSites().zones
  if not zones or not zones.sorted[1] then return end

  for k, v in pairs(triggers) do
    if v.type == 'zone' and zones.byName[k] and not v.vehIds[0] then -- veh id of 0 blocks detection
      local valid = false
      local data = {}
      for _, veh in ipairs(getAllVehiclesByType()) do
        local vehId = veh:getID()
        if isVehicleValid(k, vehId) then
          local vehInZone = zones.byName[k]:containsVehicle(veh, true)
          if v.vehIds[vehId] == nil then v.vehIds[vehId] = false end
          if v.vehIds[vehId] ~= vehInZone then -- checks if a state change occurred for this vehicle
            data.triggerName = k
            data.vehId = vehId
            data.valid = true
            data.event = vehInZone and 'enter' or 'exit'
            useTrigger(data)
          end
        end
      end
    end
  end
end

local function onVehicleSwitched(oldId, newId) -- temporarily sets the state to false if the new vehicle meets the requirements
  if not M.active then return end

  for k, v in pairs(triggers) do
    if v.vehIds and isVehicleValid(k, newId) then
      local data = {}
      data.triggerName = k
      data.vehId = newId
      data.valid = true
      data.event = 'exit'
      useTrigger(data)
    end
  end
end

local function onVehicleDestroyed(vehId) -- clears vehicle id from tables
  if not M.active then return end

  for k, v in pairs(triggers) do
    if v.vehIds then
      v.vehIds[vehId] = nil
    end
  end
end

local function onVehicleActiveChanged(vehId, active)
  if not M.active then return end

  if not active then
    onVehicleDestroyed(vehId)
  end
end

local function onClientStartMission()
  loadTriggers()
end

local function onClientEndMission()
  reset(true)
end

local function onUpdate(dt, dtSim)
  if not next(triggers) then return end

  local zoneExists = false
  for k, v in pairs(triggers) do
    if v.type == 'zone' then
      zoneExists = true
    end

    if v.objects then
      for name, data in pairs(v.objects) do
        local obj = scenetree.objectExists(name)
        if obj and v.stack > 0 then
          obj = scenetree.findObject(name)
          local delay = v.active and data.enterDelay or data.exitDelay
          local delayRandomMin = v.active and data.enterRandomMin or data.exitRandomMin
          local delayRandomMax = v.active and data.enterRandomMax or data.exitRandomMax

          if data.timer >= 0 then
            if data.timer >= delay then
              if delayRandomMax == 0 or data.maxIdx < 1 then
                setGroupState(obj:getID(), v.active)
                v.stack = v.stack - 1
              else
                data.currIdx = 0
                data.stepTimer = 0
                if not data.idxList then
                  data.idxList = {}
                  for i = 1, data.maxIdx do
                    table.insert(data.idxList, i)
                  end
                end
                if data.randomOrder then
                  data.idxList = arrayShuffle(data.idxList) -- random index order to use for target scenetree group
                end
              end

              if M.debugMode then
                log('D', logTag, '['..k..']['..name..']: delay done ('..delay..' s)')
              end

              data.timer = -1
            else
              data.timer = data.timer + dtSim
            end
          else
            if data.stepTimer >= 0 then
              if not data.delayRandom then
                data.delayRandom = lerp(delayRandomMin, delayRandomMax, math.random()) -- random delay until next index step
              end
              if data.stepTimer >= data.delayRandom then
                data.currIdx = data.currIdx + 1

                if obj:getClassName() == 'SimGroup' then
                  local idx = data.idxList[data.currIdx]
                  local innerObjects = obj:getObjects()

                  if data.isNested then -- processes all nested objects, in parallel
                    for _, o1 in ipairs(innerObjects) do
                      local obj1 = scenetree.findObjectById(tonumber(o1) or scenetree.findObject(o1):getID())
                      if obj1 and obj1:getGroup():getName() == name and obj1:getClassName() == 'SimGroup' then
                        local o2 = obj1:getObjects()[idx]
                        if o2 then
                          setState(tonumber(o2) or scenetree.findObject(o2):getID(), v.active)
                        end
                      end
                    end
                  else
                    if innerObjects[idx] then -- processes the next object
                      setState(tonumber(innerObjects[idx]) or scenetree.findObject(innerObjects[idx]):getID(), v.active)
                    end
                  end
                else
                  setState(obj:getID(), v.active)
                end

                if M.debugMode then
                  log('D', logTag, '['..k..']['..name..']: step delay done ('..data.delayRandom..' s)')
                end

                data.stepTimer = 0
                data.delayRandom = nil -- reset random delay

                if not data.idxList[data.currIdx + 1] then
                  v.stack = v.stack - 1
                  data.stepTimer = -1
                end
              else
                data.stepTimer = data.stepTimer + dtSim
              end
            end
          end
        else
          data.timer = -1
          data.stepTimer = -1
        end
      end
    end
  end

  if not zoneExists then return end
  tickTimer = tickTimer + dtSim

  if tickTimer > tickTime then
    tickTimer = tickTimer - tickTime
    onTick()
  end
end

local function onBeamNGTrigger(data)
  if not M.active or not data.triggerName then return end

  useTrigger(data)
end

local function onSerialize()
  for k, _ in ipairs(triggers) do
    setTriggerObjects(k)
  end
  return {triggers = triggers}
end

local function onDeserialized(data)
  triggers = data.triggers
end

M.reset = reset
M.loadTriggers = loadTriggers
M.setupTriggers = setupTriggers
M.addTrigger = addTrigger
M.removeTrigger = removeTrigger
M.getTriggers = getTriggers
M.setTriggerActive = setTriggerActive

M.onVehicleSwitched = onVehicleSwitched
M.onVehicleDestroyed = onVehicleDestroyed
M.onVehicleActiveChanged = onVehicleActiveChanged
M.onClientStartMission = onClientStartMission
M.onClientEndMission = onClientEndMission
M.onUpdate = onUpdate
M.onBeamNGTrigger = onBeamNGTrigger
M.onSerialize = onSerialize
M.onDeserialized = onDeserialized

return M