-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local im = ui_imgui
local ffi = require('ffi')
local logTag = ""
--M.dependencies = {"gameplay_drag_dragRace", "gameplay_drag_times", "gameplay_drag_display"}
local currentFileDir = "/gameplay/temp/"
local debugMenu = false
local dragData
local gameplayContext = "freeroam"
local ext

local levelDir = ""
local defaultSaveSlot = 'default'
local currentSaveSlotName = defaultSaveSlot
local saveRoot = 'settings/cloud/'
local currentSavePath = saveRoot .. defaultSaveSlot .. "/"
local allDragTimesData

local selectedVehicle = -1
local search = require('/lua/ge/extensions/editor/util/searchUtil')()
local aviableLanes = {}

local needsMapReload = false
local needsCollisionRebuild = false
local currentLevel = ""
local initFlagCounter = 0

local careerRewards = 5 --beamXP

----------------------------
-- Clearing and unloading --
----------------------------

local function unloadAllExtensions()
  extensions.hook("onBeforeDragUnloadAllExtensions")
  extensions.unload('gameplay_drag_display')
  extensions.unload('gameplay_drag_times')
  extensions.unload('gameplay_drag_dragTypes_headsUpDrag')
  extensions.unload('gameplay_drag_dragTypes_bracketRace')
  extensions.unload('gameplay_drag_dragTypes_dragPracticeRace')
end

local function clear()
  dragData = nil
  selectedVehicle = -1
  aviableLanes = {}
  needsMapReload = false
  needsCollisionRebuild = false
  ext = nil
  unloadAllExtensions()
  gameplayContext = "freeroam"
  initFlagCounter = 0
  if ui_gameplayAppContainers then
    ui_gameplayAppContainers.resetContainerContext('gameplayApps')
  end
  guihooks.trigger('updateTreeLightStaging', false)
end

-----------------------------
-- Loading data from files --
-----------------------------

local function setSavePath(path)
  currentSavePath = path and path or (saveRoot .. defaultSaveSlot .. "/")
end
local function setCurrentSaveSlot()
  local saveSlot, savePath = career_saveSystem.getCurrentSaveSlot()
  if not savePath then return end
  setSavePath(savePath .. "/career/")
end

-- this should only be loaded when the career is active
-- local function onSaveCurrentSaveSlot(currentSavePath, oldSaveDate)
--   setSavePath(currentSavePath .. "/career/")
--   for id, dirtyDate in pairs(allDragTimesData) do
--     if dirtyDate > oldSaveDate then
--       if gameplay_missions_progress.saveMissionSaveData(id, dirtyDate) == false then
--         career_saveSystem.saveFailed()
--       end
--     end
--   end
-- end

local function loadTransform(t)
  for key, data in pairs(t) do
    if key == "rot" then
      t[key] = quat(data.x, data.y, data.z, data.w)
    else
      t[key] = vec3(data.x, data.y, data.z)
    end
  end
  -- compute local unit vectors
  t.x, t.y, t.z = t.rot * vec3(t.scl.x,0,0), t.rot * vec3(0,t.scl.y,0), t.rot * vec3(0,0,t.scl.z)
end

local function loadDragStripData(filepath)
  if not filepath then
    log("E", logTag, "No filepath given for loading drag strip")
    return
  end
  local data = jsonReadFile(filepath)
  --Comprobe that the data is valid and has all the necessary fields
  if not data or not data.context or not data.strip or not data.phases or not next(data.strip.lanes)  then
    log("E", logTag, "Failed to read file: " .. filepath)
    return
  end

  for k,lane in ipairs(data.strip.lanes) do
    -- load all waypoint transforms
    for _, waypoint in pairs(lane.waypoints) do
      loadTransform(waypoint.transform)
    end
    -- boundary
    loadTransform(lane.boundary.transform)

    --
    local stageToEnd = lane.waypoints.endLine.transform.pos - lane.waypoints.stage.transform.pos
    lane.stageToEnd = stageToEnd
    lane.stageToEndNormalized = stageToEnd:normalized()
  end

  --Convert the endCamera transform to vec3 and quat if there is any camera.
  if data.strip.endCamera then
    for key, data in pairs(data.strip.endCamera.transform) do
      if key == "rot" then
        data.strip.endCamera.transform[key] = quat(data.x, data.y, data.z, data.w)
      else
        data.strip.endCamera.transform[key] = vec3(data.x, data.y, data.z)
      end
    end
  end

  data.isCompleted = false
  data.isStarted = false

  data.racers = {}
  --log('I', logTag, 'Loaded drag strip from: '.. filepath)

  local dir, filename, ext = path.split(filepath, true)
  filename = filename:gsub('.'..ext, "")
  data._file = dir..data.stripInfo.id
  data.saveFile = data._file .. "/history.json"
  --dumpz(data, 1)
  return data
end
M.loadDragStripData = loadDragStripData

local function loadPrefabs(data)
  if not data then return end

  --log("I", logTag, 'Loading Waypoints...')
  for laneNum, lane in ipairs(data.strip.lanes) do
    for key, waypoint in pairs(lane.waypoints) do
      if waypoint.waypoint  ~= nil then
        local wp = scenetree.findObject(waypoint.name)
        if not wp then
          --log("I", logTag, 'Creating waypoint named "'..waypoint.name..'"')
          wp = createObject('BeamNGWaypoint')
          wp:setPosition(waypoint.transform.pos)
          local scl = waypoint.transform.scl or {x = 3, y = 3, z = 3}
          wp:setField('scale', 0, scl.x .. ' ' ..scl.y..' '..scl.z)
          wp:setField('rotation', 0, waypoint.transform.rot.x .. ' ' ..waypoint.transform.rot.y..' '..waypoint.transform.rot.z..' '..waypoint.transform.rot.w)
          wp:registerObject(waypoint.name)
          scenetree.MissionGroup:addObject(wp)
          needsMapReload = true
        else
          log("W", logTag, "Waypoint already exists in the scene: " .. waypoint.name)
        end
      end
    end
  end

  --Spawn all prefabs aviable in the file
  --log("I", logTag, 'Loading Prefabs...')
  for prefabName, prefabData in pairs(data.prefabs) do
    if prefabData.path and prefabData.isUsed then
      local existingPrefab = scenetree.findObject(prefabName)
      if not existingPrefab then
        --log("I", logTag, 'Spawning Prefab: '..prefabData.path)
        local scenetreeObject = spawnPrefab(Sim.getUniqueName(prefabName) , prefabData.path, 0 .. " " .. 0 .. " " .. 0, "0 0 1 0", "1 1 1", false)
        scenetreeObject.canSave = false
        if scenetree.MissionGroup then
          scenetree.MissionGroup:add(scenetreeObject)
          prefabData.prefabId = scenetreeObject:getID()
          needsCollisionRebuild = true
          --log("I", logTag, "Prefab ".. prefabName .." added to MissionGroup")
        else
          log("E","","No missiongroup found! MissionGroup = " .. scenetree.MissionGroup)
        end
      else
        log("W",logTag, 'Prefab already spawned: '..prefabName)
      end
    end
  end
  --Resets the collision to avoid issues with old data
  if needsCollisionRebuild then
    be:reloadCollision()
  end
  --Resets the Navgrapgh to avoid issues with old data, this has a callback
  if needsMapReload then
    map.reset()
  end
end

local function unloadPrefabs()
  if not dragData or gameplayContext == "freeroam" then return end
  if needsMapReload then
    for laneNum, lane in ipairs(dragData.strip.lanes) do
      for pointType, point in pairs(lane) do
        point.waypoint.wp:delete()
      end
    end
    map.reset()
  end
  if needsCollisionRebuild then
    for prefabName, prefabData in pairs(dragData.prefabs) do
      if prefabData.path and prefabData.isUsed then
        local obj = scenetree.findObjectById(prefabData.prefabId)
        if obj then
          if editor and editor.onRemoveSceneTreeObjects then
            editor.onRemoveSceneTreeObjects({prefabData.prefabId})
          end
          obj:delete()
        end
      end
    end
    be:reloadCollision()
  end
end

local function setupRacer(vehId, lane)
  if not dragData then return end

  if not vehId then
    log('E', logTag, 'No vehicle id')
    return
  end

  local veh = scenetree.findObjectById(vehId)
  if not veh or veh.className ~= "BeamNGVehicle" then
    log('E', logTag, 'Object with ID: ' .. vehId .. ' is not a vehicle')
    return
  end

  local oldData = jsonReadFile(currentSavePath .. "dragTimes.json") or {}

  local timesKey =  M.generateHashFromFile(vehId)

  local dial = 10
  if oldData[timesKey] then
    dial = oldData[timesKey].time_1_4
  else
    --dumpz(core_vehicles.getVehicleDetails(vehId), 2)
    if core_vehicles.getVehicleDetails(vehId).configs["Drag Times"] then
      dial = core_vehicles.getVehicleDetails(vehId).configs["Drag Times"].time_1_4 or 10
    end
  end
  --Create a table for this vehicle and add to racers
  local racer = {
    vehId = vehId,
    phases = {}, --list of phases that are active
    currentPhase = 1, --current phase that is active
    isPlayable = true, --determine if it's controlled by the AI or the player
    lane = lane, --lane number of this vehicle in the race
    isDesqualified = false, --if the vehicle is desqualified
    desqualifiedReason = "None", --reason for desqualification if applicable
    isFinished = false, --if the vehicle has finished the race
    wheelsOffsets = {}, --table for the wheels offsets
    currentCorners = {}, --table for the current corners offsets (For now this is not used)
    canBeTeleported = dragData.canBeTeleported, --if the vehicle can be teleported
    canBeReseted = dragData.canBeReseted, --if the vehicle can be reseted when teleported, if not, if the player breaks the vehicle it will not be reseted and the player will have to restart the race or keep with a broken vehicle
    treeStarted = false,
    timersStarted = false,
    damage = 0,
    timers = {
      dial = {type = "dialTimer", value = dial, isSet = true},
      timer = {type = "timer", value = 0},
      reactionTime = {type = "distanceTimer", value = 0, distance = 0.4, isSet = false, label = "Reaction Time"},
      time_60 = {type = "distanceTimer", value = 0, distance = 18.288, isSet = false, label = "Distance: 60ft / 18.28m"},
      time_330 = {type = "distanceTimer", value = 0, distance = 100.584, isSet = false, label = "Distance: 330ft / 100.58m"},
      time_1_8 = {type = "distanceTimer", value = 0, distance = 201.168, isSet = false, label = "Distance: 1/8th mile / 201.16m"},
      time_1000 = {type = "distanceTimer", value = 0, distance = 304.8, isSet = false, label = "Distance: 1000ft / 304.8m"},
      time_1_4 = {type = "distanceTimer", value = 0, distance = 402.336, isSet = false, label = "Distance: 1/4th mile / 402.34m"},
      velAt_1_8 = {type = "velocity", value = 0, distance = 201.168, isSet = false, label = "Distance: 1/8th mile / 201.16m"},
      velAt_1_4 = {type = "velocity", value = 0, distance = 402.336, isSet = false, label = "Distance: 1/4th mile / 402.34m"},
      time_0_60 = {type = "timeToVelocity", value = 0, velocity = 26.8224, isSet = false},
      brakingG = {type = "brakingG", value = 0, isSet = false, deltaTime = 1},
    },
  }

  -- add working vector3 fields
  racer.vehPos = vec3()
  racer.vehDirectionVector = vec3()
  racer.vehDirectionVectorUp = vec3()
  racer.vehRot = quat()
  racer.vehVelocity = vec3()
  racer.prevSpeed = 0
  racer.vehSpeed = 0
  racer.vehObj = nil

  --Save the wheels offsets in local transform, this way we can update it and calculate distances only with the vehicle position and rotation.
  local wCount = veh:getWheelCount()-1
  local wheelsByFrontness = {}
  local maxFrontness = -math.huge
  if wCount > 0 then
    local vehPos = veh:getPosition()
    local forward = veh:getDirectionVector()
    local up = veh:getDirectionVectorUp()
    local vRot = quatFromDir(forward, up)
    local x,y,z = vRot * vec3(1,0,0),vRot * vec3(0,1,0),vRot * vec3(0,0,1)
    local center = veh:getSpawnWorldOOBB():getCenter()
    for i=0,wCount do
      local axisNodes = veh:getWheelAxisNodes(i)
      local nodePos = vec3(veh:getNodePosition(axisNodes[1]))
      local wheelNodePos = vehPos + nodePos

      local frontness = forward:dot(wheelNodePos - center)

      -- check if theres already a frontness thats less than 0.2m away
      for key, _ in pairs(wheelsByFrontness) do
        if math.abs(tonumber(key) - frontness) < 0.2 then
          frontness = key
        end
      end

      local pos = vec3(nodePos:dot(x), nodePos:dot(y), nodePos:dot(z))
      wheelsByFrontness[frontness] = wheelsByFrontness[frontness] or {}
      table.insert(wheelsByFrontness[frontness], pos)

      maxFrontness = math.max(frontness, maxFrontness)
    end
  end


  if not next(wheelsByFrontness) then
    log('E', logTag, 'Couldnt find front wheels for ' .. vehId .. '! will use OOBB as wheel offsets')

    local vehPos = veh:getPosition()
    local forward = veh:getDirectionVector()
    local up = veh:getDirectionVectorUp()
    local vRot = quatFromDir(forward, up)
    local x,y,z = vRot * vec3(1,0,0),vRot * vec3(0,1,0),vRot * vec3(0,0,1)
    local frontLeft, frontRight = veh:getSpawnWorldOOBB():getPoint(0) - vehPos, veh:getSpawnWorldOOBB():getPoint(3) - vehPos

    local posL = vec3(frontLeft:dot(x),  frontLeft:dot(y),  frontLeft:dot(z))
    local posR = vec3(frontRight:dot(x), frontRight:dot(y), frontRight:dot(z))
    maxFrontness = "oobb"
    wheelsByFrontness[maxFrontness] = {posL, posR}

  end
  racer.allWheelsOffsets = wheelsByFrontness
  racer.wheelsCenter = {}
  racer.beamState = {}
  racer.frontWheelId = maxFrontness
  for k, v in pairs(racer.allWheelsOffsets) do
    racer.wheelsCenter[k] = {pos = vec3(), wheelCountInv = 1 / #racer.allWheelsOffsets[k]}
    racer.beamState[k] = {preStage = false, stage = false}
  end

  --Initialize phases for this vehicle
  for _, p in ipairs(dragData.phases) do
    table.insert(racer.phases, {
      name = p.name,
      started = false, --true if the phase has been started
      completed = false, --true if the phase is completed
      dependency = p.dependency, --true if this phase depends on another to be completed or started
      timerOffset = 0, --seconds
      startedOffset = p.startedOffset, --seconds constant
    })
  end


  local details = core_vehicles.getVehicleDetails(vehId)
  if details then
    racer.niceName = (details.model.Brand or "") .. " " .. (details.configs.Name or "Unknown")
  end
  local status, ret = xpcall(function() return type(deserialize(veh.partConfig)) end, nop)
  if not ret then
    racer.stock = true
  else
    racer.stock = false
  end
  racer.licenseText = core_vehicles.getVehicleLicenseText(veh)

  --DEBUG
  if debugMenu then
    M.selectElement(vehId) --select the vehicle in the editor
  end

  --log('I', logTag, "Loaded vehicle " .. vehId .. " at lane: " .. lane)
  dragData.racers[vehId] = racer
end
M.setupRacer = setupRacer

-----------------------------
-- Mission Setup Interface --
-----------------------------

M.loadDragDataForMission = function (filepath)
  clear()
  local data = loadDragStripData(filepath)
  if not data then
    log("E", logTag, "Failed to load drag data from file: " .. filepath)
    return
  end

  --Load the prefabs and waypoints
  loadPrefabs(data)
  gameplayContext = data.context
  if data.dragType == "headsUpRace" then
    extensions.load('gameplay_drag_dragTypes_headsUpDrag')
    ext = gameplay_drag_dragTypes_headsUpDrag
  elseif data.dragType == "bracketRace" then
    extensions.load('gameplay_drag_dragTypes_bracketRace')
    ext = gameplay_drag_dragTypes_bracketRace
  end
  dragData = data
  --log('I', logTag, 'Loaded data from file: ' .. filepath)
end


M.setVehicles = function (vehIds)
  for _, data in ipairs(vehIds) do
    setupRacer(data.id, data.lane)
    if not dragData.racers[data.id] then
      log("E", logTag, "There is a problem with the vehicle setting, vehicle has not been set correctly.")
      return
    end
    dragData.racers[data.id].isPlayable = data.isPlayable
    if data.dial and data.dial > 0 then
      dragData.racers[data.id].timers.dial.value = data.dial
    end
  end
end

------------------------------
-- Freeroam Setup Interface --
------------------------------

local function init()
  --return loadDragStripData(levelDir .. "/dragstrips/dragStripData.dragData.json")
end

local function getPropertyValue(obj, path)
  local current = obj
  for _, key in ipairs(path) do
    if current == nil then
      log("E", logTag, "Property path not found: " .. dump(path))
      return
    end
    current = current[key]
  end
  return current
end

local function checkVehiclePermission(model, rules)
  for _, rule in ipairs(rules) do
    local propertyValue = getPropertyValue(model, rule.path)

    if rule.allowedValues then
      local found = false
      for _, allowedValue in ipairs(rule.allowedValues) do
        if propertyValue == allowedValue then
          found = true
          break
        end
      end
      if not found then
        log("E", logTag, "Vehicle " .. model.model .. " does not match rule: " .. dump(rule.path))
        return false
      end
    elseif rule.value ~= nil then
      if propertyValue ~= rule.value then
        log("E", logTag, "Vehicle " .. model.model .. " does not match rule: " .. dump(rule.path))
        return false
      end
    end
  end
  return true
end


-- Example of a vehicle permission rules
-- local vehiclePermissionRules = {
--   {
--     ruleName = "vehicleType",
--     path = {"Type"},
--     allowedValues = {"Car", "Truck"}
--   },
--   {
--     ruleName = "notAuxiliary",
--     path = {"isAuxiliary"},
--     allowedValues = false
--   }
-- }
-- Generate opponents group based on the player vehicle.
----
-- -- If no dial is provided, it will use the player vehicle's dial.
-- -- If no vehiclePermissionRules are provided, it will allow all vehicles and configs.
-- -- If no offset is provided, it will use the default value: 0.5.
-- -- If no amount is provided, it will use the default value: 1.
local function generateOpponentsGroup(vehId, dial, vehiclePermissionRules, amount, offset)
  if not vehId then
    log("E", logTag, "Invalid input parameters")
    return
  end

  if not amount then amount = 1 end
  if not offset then offset = 0.5 end

  local configs = core_vehicles.getConfigList()
  local vehicleDetails = core_vehicles.getVehicleDetails(vehId)
  if not vehicleDetails then
    log("E", logTag, "Could not find vehicle details for ID: " .. tostring(vehId))
    return
  end

  if not vehicleDetails.configs["Drag Times"] then
    log("E", logTag, "Vehicle has no drag times data")
    return
  end

  local quarterMileScore = dial or vehicleDetails.configs["Drag Times"].time_1_4 or 10
  local minTime = quarterMileScore - offset
  local maxTime = quarterMileScore + 0.1

  local eligibleVehicles = {}
  local eligibleCount = 0

  for i, c in pairs(configs.configs) do
    if c["Drag Times"] and c["Drag Times"].time_1_4 and c["Drag Times"].time_1_4 >= minTime and c["Drag Times"].time_1_4 < maxTime then

      local model = core_vehicles.getModel(c.model_key).model
      -- simple_traffic configs are not allowed to be used as opponents in any case
      if checkVehiclePermission(model, vehiclePermissionRules) and not string.match(c.key, 'simple_traffic') then
        eligibleCount = eligibleCount + 1
        eligibleVehicles[eligibleCount] = c
      end
    end
  end

  if eligibleCount == 0 then
    log("W", logTag, "No eligible vehicles found, using player vehicle as fallback")
    eligibleVehicles[1] = vehicleDetails
    eligibleCount = 1
  end

  math.randomseed(os.time())
  local opponentsGroup = {}
  for i = 1, amount do
    local selectedConfig = eligibleVehicles[math.random(eligibleCount)]
    local paints = tableKeys(tableValuesAsLookupDict(core_vehicles.getModel(selectedConfig.model_key).model.paints or {}))
    local paintCount = #paints

    table.insert(opponentsGroup, {
      model = selectedConfig.model_key,
      config = selectedConfig.key,
      paint = paintCount > 0 and paints[math.random(paintCount)] or nil,
    })
  end
  return opponentsGroup
end
M.generateOpponentsGroup = generateOpponentsGroup

local function setDragRaceData(data)
  if dragData then return end
  dragData = data
end
M.setDragRaceData = setDragRaceData

M.resetDragRace = function ()
  ext.resetDragRace()
end

M.clearRacers = function ()
  dragData.racers = {}
end

M.unloadRace = function ()
  clear()
end

M.setPlayableVehicle = function (vehId)
  if not vehId then return end
  dragData.racers[vehId].isPlayable = true
end

M.getTimers = function (vehId)
  if not dragData then return end
  return dragData.racers[vehId].timers or {}
end

M.getRacerData = function (vehId)
  if not dragData or not dragData.racers[vehId] then return end
  return dragData.racers[vehId] or {}
end

local waitForUIContextResetCallback = false
local function trySetUIContainerContextToDrag()
  if ui_gameplayAppContainers and ui_gameplayAppContainers.getContainerContext('gameplayApps') == nil then -- the container is free
    ui_gameplayAppContainers.setContainerContext('gameplayApps', 'drag')
  else
    waitForUIContextResetCallback = true
  end
end

M.startDragRaceActivity = function (lane)
  if not dragData or not dragData.racers then
    log("E", logTag, "Data not found to start the Drag Race")
    return
  end
  if lane ~= nil and gameplayContext == "freeroam" then
    -- load the racer (player vehicle)
    dragData.racers = {}
    if lane == 1 then
      dragData.prefabs.christmasTree.treeType = ".500"
    else
      dragData.prefabs.christmasTree.treeType = ".400"
    end
    M.setupRacer(be:getPlayerVehicleID(0), lane)

    -- load the practice extension and
    extensions.load('gameplay_drag_dragTypes_' .. dragData.dragType)
    ext = gameplay_drag_dragTypes_dragPracticeRace
    gameplayContext = dragData.context or 'freeroam'

    --log("I",logTag,"Starting Freeroam Dragrace on lane " .. lane)
  end

  trySetUIContainerContextToDrag()

  guihooks.trigger('updateTreeLightStaging', true)
  ext.startActivity()
end

M.onUIContainerContextReset = function()
  if waitForUIContextResetCallback then
    waitForUIContextResetCallback = false
    trySetUIContainerContextToDrag()
  end
end

M.getWinnersData = function()
  return gameplay_drag_utils.generateWinData()
end

M.getData = function ()
  return dragData
end

M.getDragIsStarted = function ()
  if not dragData then return false end
  return dragData.isStarted or false
end

-------------------------
-- Exit/Breakout hooks --
-------------------------

local function onVehicleResetted(vid)
  if be:getPlayerVehicleID(0) == vid then
    M.clearTimeslip()
  end
  if gameplayContext == "freeroam" and dragData and dragData.isStarted then
    if dragData.racers[vid] then
      clear()
    end
  end
end
M.onVehicleResetted = onVehicleResetted

local function onVehicleSwitched(oldId, newId)
  if gameplayContext == "freeroam" and dragData and dragData.isStarted then
    if dragData.racers[oldId] or dragData.racers[newId] then
      clear()
    end
  end
end
M.onVehicleSwitched = onVehicleSwitched

local function onVehicleDestroyed(vid)
  if gameplayContext == "freeroam" and dragData and dragData.isStarted then
    if dragData.racers[vid] then
      clear()
    end
  end
end
M.onVehicleDestroyed = onVehicleDestroyed

local function onExtensionLoaded()
  clear()
end
M.onExtensionLoaded = onExtensionLoaded

local function onAnyMissionChanged(status, id)
  clear()
  --check if its stopped to load the freeroam data again
  if status == "stopped" then
    dragData = init()
  end
end
M.onAnyMissionChanged = onAnyMissionChanged



-- Save / Load stuff
local savePathFreeroam = 'settings/cloud/drag/'
local savePathCareer = '/career/drag/'
M.getCurrentSavePath = function()
  local saveFolder = savePathFreeroam
  if career_career.isActive() then
    local saveSlot, savePath = career_saveSystem.getCurrentSaveSlot()
    saveFolder = savePath .. savePathCareer
  end
  return saveFolder
end

-- dials
local dialData = nil
M.saveDialTimes = function()
  if not dialData then
    dialData = jsonReadFile(M.getCurrentSavePath() .. "dialTimes.json") or {
      dials = {},
    }
  end

  for _, racer in pairs(dragData.racers) do
    if racer.isPlayable then
      local hash = M.generateHashFromFile()
      local prevTime = (dialData.dials[hash] or {})._timestamp or os.time()
      local doUpdate = prevTime < os.time() - (24*60*60)
      -- force update if odler than 24h, otherwise only update if time1/4 is better

      local _dirt = {}
      local timerKeys = {"time_60", "time_330", "time_1_8", "time_1000", "time_1_4", "velAt_1_4", "velAt_1_8", "time_0_60", "brakingG" }
      for _, key in ipairs(timerKeys) do
        _dirt[key] = racer.timers[key].value
      end
      _dirt._timestamp = os.time()
      dialData.dials[hash] = _dirt
      dialData._dirty = true
    end
  end

  if not career_career.isActive() then
    M.saveDialFile(savePathFreeroam)
  end
end

M.saveDialFile = function(dir)
  if dialData and dialData._dirty then
    dialData._dirty = nil
    jsonWriteFile(dir .. "dragTimes.json", dialData, true)
    --log("I","Wrote drag dial times to " .. dir .. "dragTimes.json")
    dialData = nil
  end
end

M.getDialTimes = function()
  if dialData then
    return dialData.dials
  else
    dialData = jsonReadFile(M.getCurrentSavePath() .. "dialTimes.json") or {
      dials = {},
    }
    return dialData.dials
  end
end

-- history
local historyData = {}
M.saveHistory = function(timeslip)
  local file = historyData[dragData.saveFile]
  if not file then
    file = jsonReadFile(M.getCurrentSavePath() .. dragData.saveFile) or {
      history = {}
    }
    historyData[dragData.saveFile] = file
  end

  table.insert(file.history, timeslip)
  file._dirty = true

  if not career_career.isActive() then
    M.saveHistoryFile(savePathFreeroam)
  end
end

M.saveHistoryFile = function(dir)
  for file, data in pairs(historyData) do
    if data._dirty then
      data._dirty = false
      jsonWriteFile(dir..file, data, true)
      --log("I","Wrote drag history to " .. dir..file)
    end
  end
  historyData = {}
end

local function dateToTimestamp(dateStr)
  return os.time({
    year = tonumber(dateStr:sub(11, 14)),
    month = tonumber(dateStr:sub(5, 6)),
    day = tonumber(dateStr:sub(8, 9)),
    hour = tonumber(dateStr:sub(16, 17)),
    min = tonumber(dateStr:sub(19, 20)),
    sec = tonumber(dateStr:sub(22, 23)),
    isdst = false
  })
end



M.getHistory = function(id)
  local filePath = M.getCurrentSavePath() .. "levels/" .. getCurrentLevelIdentifier() .. "/dragstrips/" .. id .. "/history.json"
  if not historyData[filePath] then
    local file = jsonReadFile(filePath) or {
      history = {},
    }
    historyData[filePath] = file
  end
  table.sort(historyData[filePath].history, function(a, b) return dateToTimestamp(a.stripInfo[3]) > dateToTimestamp(b.stripInfo[3]) end)
  return historyData[filePath]
end

M.setCareerRewrads = function ()
  if not career_career.isActive() or dragData.context == "activity" then return end
  -- todo: remove beamXP
  local rewards = {bmra = math.ceil(careerRewards)}
  --  return { beamXP = beamXP }
  --log("I","logTag", "Set Carrer Reward for drag race to "..serialize(rewards) .. " BeamXP")
  career_modules_playerAttributes.addAttributes(rewards,{label="Rewards for Drag Race", tags={"gameplay"}})
  return rewards
end

local function onSaveCurrentSaveSlot(currentSavePath)
  M.saveDialFile(currentSavePath .. savePathCareer)
  M.saveHistoryFile(currentSavePath .. savePathCareer)
end
M.onSaveCurrentSaveSlot = onSaveCurrentSaveSlot


local function onCareerActive()
  dialData = nil
  historyData = {}
end
M.onCareerActive = onCareerActive

--TIMESLIP interface

local timerKeys = {"reactionTime", "time_60", "time_330", "time_1_8",  "time_1000", "time_1_4" }
local velocityKeys = {"velAt_1_4", "velAt_1_8"}
local rowsInfo = {
  { key = "laneName", label = "Lane" },
  --{ key = "tree", label = "Tree" },
  { key = nil, label = "" },  -- Fixed empty key
  { key = "reactionTime", label = "R/T" },
  { key = "time_60", label = "60'" },
  { key = "time_330", label = "330'" },
  { key = "time_1_8", label = "660'" },
  { key = "velAt_1_8_kmh", label = "km/h" },
  { key = "velAt_1_8_mph", label = "mph" },
  { key = "time_1000", label = "1000'" },
  { key = "time_1_4", label = "1/4 mile" },
  { key = "velAt_1_4_kmh", label = "km/h" },
  { key = "velAt_1_4_mph", label = "mph" },

}
local racerRowsInfo = {
  { key = "lane", label = "Lane" },
  { key = "licenseText", label = "License" },
  { key = "name", label = "Vehicle" },
  { key = "stock", label = "" },
}

local longestRowLabel = -math.huge
for _, r in ipairs(rowsInfo) do
  longestRowLabel = math.max(longestRowLabel, #r.label+2)
end
for _, r in ipairs(racerRowsInfo) do
  longestRowLabel = math.max(longestRowLabel, #r.label+2)
end
for _, r in ipairs(rowsInfo) do
  r.label = r.label .. string.rep(".", longestRowLabel - #r.label)
end
  for _, r in ipairs(racerRowsInfo) do
  r.label = r.label .. string.rep(".", longestRowLabel - #r.label)
end

local treeNames = {[".400"] = "Pro Tree", [".500"] = "Sportsman Tree"}
M.clearTimeslip = function()
  guihooks.trigger("onDragRaceTimeslipData", nil)
end

M.createTimeslipData = function ()
  if not dragData or not next(dragData) then
    return
  end
  local slipData = {}

  -- info about the strip itself
  local stripInfo = {}
  table.insert(stripInfo, dragData.stripInfo and dragData.stripInfo.stripName or "Drag Strip")
  table.insert(stripInfo, core_levels.getLevelByName(getCurrentLevelIdentifier()).title)
  table.insert(stripInfo, os.date(dragData.stripInfo and dragData.stripInfo.dateFormat or "%a %m/%d/%Y %I:%M:%S %p"))
  slipData.stripInfo = stripInfo
  slipData.tree = treeNames[dragData.prefabs.christmasTree.treeType]
  slipData.env = {
    tempK = core_environment.getTemperatureK(),
    tempC = core_environment.getTemperatureK() - 273.15,
    tempF = (core_environment.getTemperatureK() - 273.15) * (9/5) + 32,
    customGrav = math.abs(core_environment.getGravity() - 9.81) > 0.01,
    gravity = string.format("%0.2f m/s²", 100*core_environment.getGravity() / 9.81),
  }

  -- initialize every lane as empty
  local dataByLane = {}
  local laneNumsOrdered = {}
  for laneNum, lane in ipairs(dragData.strip.lanes) do
    dataByLane[laneNum] = {laneName = lane.shortName}
    table.insert(laneNumsOrdered, laneNum)
  end
  table.sort(laneNumsOrdered, function(a,b) return ((dragData.strip.lanes[a].laneOrder) or a) > ((dragData.strip.lanes[b].laneOrder) or b) end)



  local racerInfos = {}
  for vehId, racer in pairs(dragData.racers) do
    for _, key in ipairs(timerKeys) do
      dataByLane[racer.lane][key] = string.format("%0.3f",racer.timers[key].value)
    end
    for _, key in ipairs(velocityKeys) do
      dataByLane[racer.lane][key..'_kmh'] = string.format("%0.3f",racer.timers[key].value * 3.6)
      dataByLane[racer.lane][key..'_mph'] = string.format("%0.3f",racer.timers[key].value * 2.23694)
    end
    --if racer.timers['dial'] then
      --dataByLane[racer.lane]['dial'] = string.format("%0.3f",racer.timers['dial'].value)
      --dataByLane[racer.lane]['dial_diff'] = string.format("%0.3f",racer.timers['time_1_4'] - racer.timers['dial'].value)
    --end

    local currentVeh = core_vehicles.getVehicleDetails(vehId)
    local vehicleConfig = currentVeh.configs

    local racerInfo = {
      name = racer.niceName,
      stock = racer.stock and "Stock" or "Modified",
      licenseText = racer.licenseText,
      lane = dragData.strip.lanes[racer.lane].longName,
      laneOrder = dragData.strip.lanes[racer.lane].laneOrder,
      laneNum = racer.lane,
      finalTime = racer.timers.time_1_4.value,
      rewards = M.setCareerRewrads() or {},
      dialDiff = racer.timers.time_1_4.value - racer.timers.dial.value,
      dial = racer.timers.dial.value,
      disqualification = racer.isDesqualified and "DQ" or "Not DQ",

      -- Add Info for sorting the tickets at the Drag History Screen
      brand = currentVeh.model.Brand or "Unknown",
      country = currentVeh.model.Country or "Unknown",
      drivetrain = vehicleConfig.Drivetrain or "Unknown",
      fuelType = vehicleConfig["Fuel Type"] or "Unknown",
      transmission = vehicleConfig.Transmission or "Unknown",
      configType = vehicleConfig["Config Type"] or "Unknown",
      inductionType = vehicleConfig["Induction Type"] or "Unknown",
    }
    table.insert(racerInfos, racerInfo)
  end
  table.sort(racerInfos, function(a,b) return a.laneOrder > b.laneOrder end)
  slipData.racerInfos = racerInfos


  -- build final table for UI
  local tab = {}
  for _, r in ipairs(rowsInfo) do
    local row = {}
    table.insert(row, r.label)
    for _, laneNum in ipairs(laneNumsOrdered) do
      if r.key then
        local col = dataByLane[laneNum]
        if col then
          table.insert(row, col[r.key] or " - ")
        else
          table.insert(row, '-')
        end
      else
        table.insert(row, ' ')
      end
    end
    table.insert(tab, row)
  end
  --dump(racerInfos)

  if #racerInfos > 1 then
    -- [1] is lane 1, right lane
    -- [2] is lane 2, left lane

    if dragData.dragType == "bracketRace" then
      table.insert(tab, {'Dial',string.format("%0.3f", racerInfos[1].dial), string.format("%0.3f", racerInfos[2].dial)})
      table.insert(tab, {'Dial Difference',string.format("%s%0.3f", racerInfos[1].dialDiff > 0 and "+" or "",racerInfos[1].dialDiff), string.format("%s%0.3f",racerInfos[2].dialDiff > 0 and "+" or "", racerInfos[2].dialDiff)})

      -- Check for disqualifications first
      if racerInfos[1].disqualification == "DQ" and racerInfos[2].disqualification == "DQ" then
        table.insert(tab, {'',"DQ", "DQ"})
      elseif racerInfos[1].disqualification == "DQ" then
        table.insert(tab, {'',"DQ", "WINNER"})
      elseif racerInfos[2].disqualification == "DQ" then
        table.insert(tab, {'',"WINNER", "DQ"})
      else
        -- Original logic for non-disqualified racers
        if racerInfos[1].dialDiff == racerInfos[2].dialDiff then
          table.insert(tab, {'',"TIE", "TIE"})
        elseif racerInfos[1].dialDiff > 0 and racerInfos[2].dialDiff > 0 then
          -- both got their dial, lower (closer to 0) value wins
          if racerInfos[1].dialDiff < racerInfos[2].dialDiff then
            table.insert(tab, {'',"WINNER", " "})
          else
            table.insert(tab, {''," ", "WINNER"})
          end
        else
          if racerInfos[1].dialDiff > racerInfos[2].dialDiff then
            table.insert(tab, {'',"WINNER", "Break Out"})
          else
            table.insert(tab, {'',"Break Out", "WINNER"})
          end
        end
      end
    else
      -- Check for disqualifications in non-bracket races
      if racerInfos[2].disqualification == "DQ" and racerInfos[1].disqualification == "DQ" then
        table.insert(tab, {'',"DQ", "DQ"})
      elseif racerInfos[1].disqualification == "DQ" then
        table.insert(tab, {'',"DQ", "WINNER"})
      elseif racerInfos[2].disqualification == "DQ" then
        table.insert(tab, {'',"WINNER", "DQ"})
      else
        -- Original logic for non-disqualified racers
        if racerInfos[1].finalTime > racerInfos[2].finalTime then
          table.insert(tab, {'',string.format("+%0.3f",racerInfos[1].finalTime - racerInfos[2].finalTime),'WINNER'})
        else
          table.insert(tab, {'','WINNER',string.format("+%0.3f",racerInfos[2].finalTime - racerInfos[1].finalTime)})
        end
      end
    end

  end

  slipData.timesTable = tab
  return slipData
end


M.createTimeslipPanelData = function()
  local slip = M.createTimeslipData()
  local ret = {}
  for _, key in ipairs({"stripInfo","tree","env","racerInfos"}) do
    ret[key] = slip[key]
  end

  -- ui grid needs to be in its own function (upgrade timeslip data for vue...)
  local grid = {
    labels = {},
    rows = {}
  }

  local tab = slip.timesTable
  for _, l in ipairs(tab[1]) do
    table.insert(grid.labels, l)
  end
  for i = 3, #tab do
    local row = {}
    table.insert(row, {
      text = (tab[i][1]):gsub("%.+$", "")
    })

    for j = 2, #tab[i] do
      local txt = tab[i][j]
      local num = tonumber(txt)

      table.insert(row, {text = txt, mono=true})
    end
    table.insert(grid.rows, row)
  end

  ret.grid = grid

  return ret
end

M.sendTimeslipDataToUi = function()
  --log("I","","Requesting Timeslip Data...")
  -- main data
  local slipData = M.createTimeslipData()


  if not slipData or not next(slipData) then
    --log("I","","Timeslip cleared.")
    guihooks.trigger("onDragRaceTimeslipData", nil)
    return
  end

  M.saveHistory(slipData)
  --log("I","","Timeslip sent.")
  guihooks.trigger("onDragRaceTimeslipData", slipData)
end

M.generateHashFromFile = function(vehId)
  local currentVeh = vehId and core_vehicles.getVehicleDetails(vehId) or core_vehicles.getCurrentVehicleDetails()

  if string.find(currentVeh.current.pc_file, ".pc") then
    return hashStringSHA256(serialize(jsonReadFile(currentVeh.current.pc_file)))
  end

  return hashStringSHA256(currentVeh.current.pc_file)
end

M.screenshotTimeslip = function()
  local dir = "screenshots/timeslips/"..getScreenShotDateTimeString()
  screenshot.doScreenshot(nil, nil, dir,'jpg')
  ui_message("Timeslip saved: " .. dir .. ".jpg", nil, nil, "save")
end


-- DEBUG FUNCTIONALITY


local function getSelection(classNames)
  local id
  if editor.selection and editor.selection.object and editor.selection.object[1] then
    local currId = editor.selection.object[1]
    if not classNames or arrayFindValueIndex(classNames, scenetree.findObjectById(currId).className) then
      id = currId
    end
  end
  return id
end

local function selectElement(index)
  selectedVehicle = index
end

local function getLastElement()
  for vehId,_ in pairs(dragData.racers) do
    selectedVehicle = vehId
  end
end

local red, yellow, green = im.ImVec4(1,0.5,0.5,0.75), im.ImVec4(1,1,0.5,0.75), im.ImVec4(0.5,1,0.5,0.75)
local function drawDebugMenu()
  if debugMenu then
    if im.Begin("Drag Race General Debug") then
      --[[
      if not editor_fileDialog then im.BeginDisabled() end
      if im.Button("Load Save Data ##loadDataFromFile") then
        editor_fileDialog.openFile(function(data) loadDataFromFile(data.filepath) end, {{"dragData Files",".dragData.json"}}, false, currentFileDir)
      end
      if not editor_fileDialog then im.EndDisabled() end
      ]]
      im.SameLine()
      if im.Button("Clear Save Data ##clearData") then
        dragData = nil
      end
      if dragData then
        im.Columns(2,'mainDrag')
        im.Text("Drag Data")
        im.Text("Context: ")
        im.SameLine()
        im.Text(dragData.context)

        im.Text("dragtype extension: ")
        im.SameLine()
        im.TextColored(ext and green or red, ext and ext.__extensionName__ or "No Extension")

        im.Text("Is Started:")
        im.SameLine()
        im.TextColored(dragData.isStarted and green or red, dragData.isStarted and "Started" or "Stopped")

        im.NewLine()
        im.Text("Phases: ")
        for index, value in ipairs(dragData.phases or {}) do
          im.SameLine()
          if im.Button("Play " .. value.name) then
            ext.startDebugPhase(index, dragData)
          end
        end
        im.NewLine()

        if im.Button("Start Drag Race") then
          M.startDragRaceActivity()
        end

        if im.Button("Reset Drag Race") then
          ext.resetDragRace()
        end


        im.NextColumn()
        im.Text("Strip Data")
        im.NewLine()
        if dragData.strip.endCamera then
          im.Text("End Camera: ")
          im.SameLine()
          im.Text("Position: {" .. dragData.strip.endCamera.transform.pos.x .. ", " .. dragData.strip.endCamera.transform.pos.y .. ", " .. dragData.strip.endCamera.transform.pos.z .. "}")
          im.SameLine()
          im.Text("Rotation: {" .. dragData.strip.endCamera.transform.rot.x .. ", " .. dragData.strip.endCamera.transform.rot.y .. ", " .. dragData.strip.endCamera.transform.rot.z .. ", " .. dragData.strip.endCamera.transform.rot.w .. "}")
          im.SameLine()
          im.Text("Scale: {" .. dragData.strip.endCamera.transform.scl.x .. ", " .. dragData.strip.endCamera.transform.scl.y .. ", " .. dragData.strip.endCamera.transform.scl.z .. "}")
        end

        for nameType, p in pairs(dragData.prefabs) do
          im.Text("Prefab: " .. nameType)
          im.SameLine()
          im.Text(" | Is Used: " .. tostring(p.isUsed))
          if p.isUsed then
            im.SameLine()
            im.Text(" |  " .. (p.path or "No path founded"))
          end
        end
        im.NextColumn()

        im.Columns(2, 'vehicles')

        im.BeginChild1("vehicle select", im.GetContentRegionAvail(), 1)
        im.Text("Vehicle Settings")
        for k,v in ipairs(aviableLanes) do
          if v then
            if im.Selectable1("Empty Lane - " ..k.. "##" .. k) then
              local vehId = getSelection()

              setupRacer(vehId, k)
              if not dragData.racers[vehId] then
                aviableLanes[k] = true
              else
                aviableLanes[k] = false
              end
            end
            if im.IsItemHovered() then
              im.tooltip("Add selected vehicle from scenetree to Lane: " ..k)
            end
          end
        end
        for vehId, _ in pairs(dragData.racers or {}) do
          if im.Selectable1(string.format("Racer ID: %d Lane: %d", vehId, dragData.racers[vehId].lane), vehId == selectedVehicle) then
            selectElement(vehId)
          end
        end
        im.EndChild()
        im.NextColumn()

        im.BeginChild1("vehicle detail", im.GetContentRegionAvail(), 1)
        if selectedVehicle and dragData.racers[selectedVehicle] then
          if im.Button("Remove Vehicle" .. "##"..selectedVehicle) then
            aviableLanes[dragData.racers[selectedVehicle].lane] = true
            dragData.racers[selectedVehicle] = nil
            selectedVehicle = -1
            getLastElement()
          end

          im.NewLine()
          im.Text("Lane ".. dragData.racers[selectedVehicle].lane .. " Data :")
          if selectedVehicle ~= -1 then
            im.Text("(Click to dump, hover to preview)")
            for key, laneData in pairs(dragData.strip.lanes[dragData.racers[selectedVehicle].lane]) do
              if im.Button("Lanedata: " .. key) then
                dump(laneData.transform)
              end
              if im.IsItemHovered() and editor_dragRaceEditor then
                local rot = quat(laneData.transform.rot)
                local x, y, z = laneData.transform.x, laneData.transform.y, laneData.transform.z
                local scl = (x+y+z)/2
                editor_dragRaceEditor.drawAxisBox(((-scl*2)+vec3(laneData.transform.pos)),x*2,y*2,z*2,color(255,255,255,0.2*255))
                local pos = vec3(laneData.transform.pos)
                debugDrawer:drawLine(pos, pos + x, ColorF(1,0,0,0.8))
                debugDrawer:drawLine(pos, pos + y, ColorF(0,1,0,0.8))
                debugDrawer:drawLine(pos, pos + z, ColorF(0,0,1,0.8))
              end
              --[[
              im.Text("-" .. key .. ": ")
              im.Text("Position: {" .. laneData.transform.pos.x .. ", " .. laneData.transform.pos.y .. ", " .. laneData.transform.pos.z .. "}")
              im.SameLine()
              im.Text("Rotation: {" .. laneData.transform.rot.x .. ", " .. laneData.transform.rot.y .. ", " .. laneData.transform.rot.z .. "," .. laneData.transform.rot.w .. "}")
              im.SameLine()
              im.Text("Scale: {" .. laneData.transform.scl.x .. ", " .. laneData.transform.scl.y .. ", " .. laneData.transform.scl.z .. "}")
              ]]
            end
          end
          im.Text("Vehicle Data: ")
          local vehicleData = dragData.racers[selectedVehicle]
          if not vehicleData then
            im.Text("No vehicle data yet")
          else
            if im.Button("Dump Vehicle data") then
              dump(dragData.racers[selectedVehicle])
            end
            local isP = im.BoolPtr(vehicleData.isPlayable)
            im.Checkbox("Is Playable", isP)
            vehicleData.isPlayable = isP[0]
            im.SameLine()
            im.Text(vehicleData.isPlayable and "Is Playable" or "Not playable")
            im.Text("Lane: " .. vehicleData.lane)
            im.Text(vehicleData.isDesqualified and "Desqualified" or "Not desqualified")
            im.Text("Desqualification Reason: " ..vehicleData.desqualifiedReason)
            im.Separator()
            im.Text("Phases")

            for _, phase in ipairs(vehicleData.phases) do
              im.Text(phase.name .. " - ")
              im.SameLine()
              im.TextColored(phase.started and green or red, "Started")
              im.SameLine()
              im.TextColored(phase.completed and green or red, "Completed")
              im.Text(dumps(phase))
              im.Separator()
            end
          end
        end
        im.EndChild()
        im.NextColumn()
      end
      im.Columns(0)
    end
  end
end


return M