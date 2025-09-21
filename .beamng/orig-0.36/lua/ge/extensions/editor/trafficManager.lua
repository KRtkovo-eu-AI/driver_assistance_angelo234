-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local im = ui_imgui

local logTag = "editor_trafficManager"
local editModeName = "trafficEditMode"
local realName = "Traffic Manager"
local simGroupName = "TrafficSession"
local version = 1

local windows = { -- editor windows names, layouts, and states
  main = {
    key = "trafficManager",
    name = realName,
    size = im.ImVec2(360, 80)
  },
  vehicles = {
    key = "trafficManager_vehicles",
    name = "Traffic Vehicles & Props",
    icon = "car",
    size = im.ImVec2(600, 560),
    active = false
  },
  lights = {
    key = "trafficManager_lights",
    name = "Traffic Lights",
    icon = "traffic",
    size = im.ImVec2(500, 560),
    active = false
  },
  signs = {
    key = "trafficManager_signs",
    name = "Traffic Signs",
    icon = "directions",
    size = im.ImVec2(500, 560),
    active = false
  },
  options = {
    key = "trafficManager_options",
    name = "Options",
    icon = "settings",
    size = im.ImVec2(240, 240),
    popup = true,
    active = false
  }
}
local windowsSorted = {"vehicles", "lights", "signs", "options"} -- array of window order

local imColors = {
  active = im.ImVec4(1, 1, 1, 1),
  inactive = im.ImVec4(1, 1, 1, 0.5),
  activeLive = im.ImVec4(1, 1, 0.5, 1),
  inactiveLive = im.ImVec4(1, 1, 0.5, 0.5),
  accept = im.ImVec4(0, 0.5, 0, 1),
  warning = im.ImVec4(1, 1, 0, 1),
  error = im.ImVec4(1, 0, 0, 1)
}
local imSizes = {
  dummy = im.ImVec2(5, 5),
  small = im.ImVec2(20, 20),
  medium = im.ImVec2(30, 30),
  large = im.ImVec2(40, 40)
}
local debugColors = {
  main = ColorF(1, 1, 1, 0.5),
  selected = ColorF(1, 1, 0.25, 0.5),
  guide = ColorF(0.25, 1, 0.25, 0.5),
  warning = ColorF(1, 0.7, 0.2, 0.5),
  error = ColorF(1, 0.25, 0.25, 0.5),
  background = ColorI(0, 0, 0, 200)
}

local mousePos, anchorPos, tempVec, tempVecAlt = vec3(), vec3(), vec3(), vec3()
local confirmData = {}
local speedUnits = {"km/h", "mph", "m/s"}
local distanceUnits = {"m", "km", "ft", "mi"}
local validVehTypes = {Car = 1, Truck = 1, Automation = 1, Traffic = 1}
local vecUp = vec3(0, 0, 1)
local vecY = vec3(0, 1, 0)
local imDefaultPos = im.ImVec2(60, 180) -- places new windows below the main window
local prevFilePath = "/"
local isDragging = false
local isUserTransform = false
local clickLock = false
local shiftLock = false
local altLock = false
local spawnDelayFrames = -1
local tickTimer = 0

local vehSelector, signSelector, session, options, currTransform, currSelection, activeWindow, aiModes, inputWidth
local mouseMode, tempEditMode

M.debugMode = false

local function convertDistance(val, unit) -- converts distance units for vehicle stats
  if unit == "km" then
    val = val / 1000
  elseif unit == "ft" then
    val = val * 3.2808
  elseif unit == "mi" then
    val = val / 1609.344
  end

  return val
end

local function convertSpeed(val, unit) -- converts speed units for vehicle stats
  if unit == "km/h" then
    val = val * 3.6
  elseif unit == "mph" then
    val = val * 2.2369
  end

  return val
end

local function staticRayCast()
  local rayCastHit
  if core_forest.getForestObject() then core_forest.getForestObject():disableCollision() end
  local rayCast = cameraMouseRayCast()
  if core_forest.getForestObject() then core_forest.getForestObject():enableCollision() end

  if rayCast then rayCastHit = vec3(rayCast.pos) end
  return rayCastHit
end

local function mouseHandler(tip) -- helper function for mouse actions for certain mouse modes
  if clickLock then return end -- prevents mouse input

  debugDrawer:drawSphere(mousePos, 0.25, debugColors.guide)
  if not isDragging then
    debugDrawer:drawTextAdvanced(mousePos, String(tip), debugColors.main, true, false, debugColors.background)
  end

  if not isDragging and im.IsMouseClicked(0) then
    anchorPos:set(mousePos)
    isDragging = true
  end

  if isDragging then -- drag direction out from anchor position
    tempVec:setSub2(mousePos, anchorPos)
    if tempVec:squaredLength() > 1 then
      tempVec:normalize()
    end
    debugDrawer:drawSquarePrism(anchorPos, anchorPos + tempVec, Point2F(0.3, 0.5), Point2F(0.3, 0), debugColors.guide)
    if im.IsMouseReleased(0) then
      isDragging = false
      if anchorPos:squaredDistance(mousePos) <= 0.1 then
        mousePos:setAdd2(anchorPos, core_camera.getForward())
      end

      return anchorPos, mousePos
    end
  end
end

local function mouseGizmoHandler()
  -- TODO: copy from other modules
end

local function checkSimGroup() -- checks the traffic session SimGroup and updates the session data if changes were found
  if not scenetree.objectExists(simGroupName) then return end

  local group = scenetree.findObject(simGroupName)
  table.clear(session.vehiclesSorted)
  table.clear(session.propsSorted)

  for _, objName in ipairs(group:getObjects()) do -- automatically registers objects as session data, depending on class and dynamic fields
    local obj = scenetree.findObject(objName)
    local className = obj:getClassName()

    if className == "BeamNGVehicle" then
      obj:setInternalName("vehicle") -- internal name helps with classification within this editor

      local isVehicle = false
      local modelData = core_vehicles.getModel(obj.jbeam)
      if modelData and next(modelData) and modelData.model.Type then
        isVehicle = validVehTypes[modelData.model.Type] and true or false
      end

      -- maybe signs should be excluded from the props table
      table.insert(isVehicle and session.vehiclesSorted or session.propsSorted, obj:getName())
    elseif className == "TSStatic" then
      if obj.signalInstance then -- traffic light object
        obj:setInternalName("signal") -- object is linked to a traffic light
      else
        obj:setInternalName("sign") -- assumes that object is a sign (?)
      end
    end
  end

  table.sort(session.vehiclesSorted)
  table.sort(session.propsSorted)
  session.objectCount = group:getCount() -- object count is used to check if the SimGroup got updated
end

local function deleteSimGroup() -- deletes the existing traffic session SimGroup, and everything in it
  if scenetree.objectExists(simGroupName) then
    scenetree.findObject(simGroupName):delete()
    be:reloadCollision()
  end
end

local function createSimGroup() -- creates the traffic session SimGroup
  if not scenetree.objectExists(simGroupName) then
    local trafficSessionGroup = createObject("SimGroup")
    trafficSessionGroup:registerObject(simGroupName)
    scenetree.MissionGroup:addObject(trafficSessionGroup)
  end
end

local function resetSession(fullReset) -- resets all session data; optionally deletes the SimGroup
  session = {
    name = "Traffic Session",
    author = "",
    description = "Traffic session description.",
    active = false,
    play = false,
    lightsActive = false,
    lightsPreloaded = false,
    playerId = be:getPlayerVehicleID(0),
    objectCount = 0,
    vehicles = {},
    lights = {},
    signs = {},
    options = {},
    vehiclesSorted = {},
    propsSorted = {},
    lightsSorted = {},
    signsSorted = {}
  }
  options = {
    sessionName = im.ArrayChar(256, session.name),
    sessionAuthor = im.ArrayChar(256, session.author),
    sessionDescription = im.ArrayChar(4096, session.description),
    vehicleType = im.IntPtr(1),
    vehicleName = im.ArrayChar(128, ""),
    vehicleClass = im.ArrayChar(128, ""),
    vehicleGroupEnabled = im.BoolPtr(false),
    vehicleGroupMode = im.IntPtr(1),
    vehicleGroupFile = im.ArrayChar(1024, ""),
    vehicleGroupRandomPaint = im.BoolPtr(false),
    vehicleMulti = im.IntPtr(1),
    speedUnits = im.IntPtr(1),
    distanceUnits = im.IntPtr(1),
    fullStats = im.BoolPtr(false),
    objPos = im.ArrayFloat(3),
    objRot = im.ArrayFloat(3),
    aiData = {},
    includePlayerVehicle = im.BoolPtr(false),
    signalsKeepOriginal = im.BoolPtr(true),
    signalsSaveApart = im.BoolPtr(false),
    debugMode = im.BoolPtr(false)
  }
  aiModes = {
    basic = {traffic = "Traffic", random = "Random", flee = "Flee", chase = "Chase", follow = "Follow", stop = "Stop"},
    script = {script = "Script"},
    target = {target = "Drive to Target"},
    user = {flowgraph = "Flowgraph"}
  }

  imColors.basicLights = {
    im.ImVec4(0, 0.75, 0.25, 1), -- green
    im.ImVec4(0.75, 0.75, 0, 1), -- yellow
    im.ImVec4(0.75, 0, 0, 1),    -- red
    im.ImVec4(0, 0, 0, 1)        -- black
  }
  imColors.controllers = {
    im.ImVec4(0.6, 1, 0.6, 1),
    im.ImVec4(0.8, 1, 0.8, 1),
    im.ImVec4(0.6, 1, 1, 1),
    im.ImVec4(0.8, 1, 1, 1),
    im.ImVec4(0.6, 0.6, 1, 1),
    im.ImVec4(0.8, 0.8, 1, 1)
  }
  debugColors.controllers = {
    ColorF(0.2, 1, 0.2, 0.6),
    ColorF(0.6, 1, 0.6, 0.6),
    ColorF(0.2, 1, 1, 0.6),
    ColorF(0.6, 1, 1, 0.6),
    ColorF(0.2, 0.2, 1, 0.6),
    ColorF(0.6, 0.6, 1, 0.6)
  }

  currTransform = {pos = vec3(), rot = quat(), scl = 0}
  currSelection = {vehicle = 0, light = 0, sign = 0}
  activeWindow = "none"
  mouseMode = "none"

  for _, key in ipairs(windowsSorted) do
    windows[key].active = false
  end

  if fullReset then
    core_trafficSignals.loadSignals() -- loads original signals of map
    core_trafficSignals.setActive(true, true)
    core_trafficSignals.debugLevel = 0
    deleteSimGroup()
  end
end

local function resetAll() -- resets the session and deletes the traffic session SimGroup
  resetSession(true)
end

local function setWindowConfirm(txt, yesFunc, noFunc) -- helper function to set info for popup confirmation window
  confirmData.txt = txt or "Are you sure?"
  confirmData.yesFunc = yesFunc or nop
  confirmData.noFunc = noFunc or nop
  confirmData.ready = true
end

local function enableVehicleAi(name) -- activates or updates the stored AI mode for this vehicle
  local sessionData = session.vehicles[name] or {id = 0}
  local veh = getObjectByID(sessionData.id)
  if not sessionData or not veh then return end

  sessionData.aiActive = true

  local aiData = sessionData.aiData or {}
  local trafficData = gameplay_traffic.getTrafficData()
  if not trafficData[sessionData.id] and aiData.enableTraffic then
    gameplay_traffic.insertTraffic(sessionData.id, false, true)
    local traffic = gameplay_traffic.getTrafficData()
    if traffic[sessionData.id] then
      traffic[sessionData.id].isAi = true
    end
  elseif trafficData[sessionData.id] and not aiData.enableTraffic then
    gameplay_traffic.removeTraffic(sessionData.id)
  end

  if sessionData.aiType == "basic" then
    veh:queueLuaCommand('ai.setMode("'..sessionData.aiMode..'")')
  elseif sessionData.aiType == "target" then
    veh:queueLuaCommand('ai.setMode("manual")')
  elseif sessionData.aiType == "script" then
    local script = jsonReadFile(aiData.scriptFile or "")
    if script then
      -- annoyingly, there are two ScriptAI formats that we need to differentiate between here
      -- so we need to check if script.recording exists, and if not, then just use the script as is
      script = script.recording or script
      veh:queueLuaCommand('ai.startFollowing('..serialize(script)..')')
    end
  elseif sessionData.aiType == "user" then
    sessionData._tempData.flowgraph = core_flowgraphManager.loadManager(aiData.flowgraphFile)
    if sessionData._tempData.flowgraph then
      sessionData._tempData.flowgraph.transient = true -- prevents flowgraph from restarting after ctrl+L
      if sessionData._tempData.flowgraph.variables:variableExists("currentId") then
        sessionData._tempData.flowgraph.variables:changeBase("currentId", sessionData.id) -- sets the flowgraph variable as this vehicle id
      end
      if sessionData._tempData.flowgraph.variables:variableExists("playerId") then
        sessionData._tempData.flowgraph.variables:changeBase("playerId", be:getPlayerVehicleID(0) or 0) -- sets the flowgraph variable as the active player vehicle id
      end
      sessionData._tempData.flowgraph:setRunning(true)
      sessionData._tempData.flowgraph.modules.traffic.keepTrafficState = true
    end
  end

  if aiData.targetName ~= nil then -- target vehicle for AI mode
    local targetVeh = scenetree.findObject(aiData.targetName)
    if targetVeh then
      veh:queueLuaCommand('ai.setTargetObjectID('..targetVeh:getID()..')')
    end
  end
  if aiData.targetPos ~= nil then -- target position for AI mode
    local n1, n2 = map.findClosestRoad(aiData.targetPos)
    if n1 and n2 then
      local p1, p2 = map.getMap().nodes[n1].pos, map.getMap().nodes[n2].pos
      if p2:squaredDistance(aiData.targetPos) < p1:squaredDistance(aiData.targetPos) then
        n1, n2 = n2, n1
      end
      veh:queueLuaCommand('ai.setTarget("'..n1..'")')
    end
  end

  if aiData.aggression ~= nil then
    veh:queueLuaCommand('ai.setAggression('..aiData.aggression..')')
  end
  if aiData.driveInLane ~= nil then
    veh:queueLuaCommand('ai.driveInLane("'..(aiData.driveInLane and 'on' or 'off')..'")')
  end
  if aiData.speed ~= nil then
    veh:queueLuaCommand('ai.setSpeed('..aiData.speed..')')
  end
  if aiData.useSpeedLimit ~= nil then
    veh:queueLuaCommand('ai.setSpeedMode("'..(aiData.useSpeedLimit and 'legal' or 'limit')..'")')
  end
  if aiData.avoidCars ~= nil then
    veh:queueLuaCommand('ai.setAvoidCars("'..(aiData.avoidCars and 'on' or 'off')..'")')
  end
end

local function disableVehicleAi(name) -- stops the AI mode for this vehicle
  local sessionData = session.vehicles[name] or {id = 0}
  local veh = getObjectByID(sessionData.id)
  if not sessionData or not veh then return end

  if sessionData._tempData.flowgraph then
    sessionData._tempData.flowgraph:setRunning(false)
    core_flowgraphManager.removeManager(sessionData._tempData.flowgraph)
    sessionData._tempData.flowgraph = nil
  end

  sessionData.aiActive = false

  veh:queueLuaCommand('ai.setMode("stop")')
  gameplay_traffic.removeTraffic(sessionData.id)
end

local function deleteVehicles()
  for id, data in pairs(session.vehicles) do
    if scenetree.objectExists(id) and not data.locked then
      getObjectByID(data.id):delete()
    end
  end
end

local function getDefaultAiData(aiType) -- gets default AI parameters
  local aiData = {}
  if aiType == "basic" then
    aiData = {aggression = 0.35, speed = 16.7, useSpeedLimit = false, driveInLane = true, avoidCars = true, enableTraffic = false}
  elseif aiType == "script" then
    aiData = {scriptFile = ""}
  elseif aiType == "target" then
    aiData = {aggression = 0.35, speed = 16.7, useSpeedLimit = false, driveInLane = true, avoidCars = true, targetPos = vec3(), directRadius = 0} -- directRadius is unused
  elseif aiType == "user" then
    aiData = {flowgraphFile = ""}
  end
  table.clear(options.aiData)

  return aiData
end

local function createVehicleData(vehId, vehData) -- creates or updates vehicle data
  local veh = getObjectByID(vehId)
  if not veh then return end

  local name = veh:getName()

  vehData = vehData or session.vehicles[name]
  if not vehData then vehData = {} end

  vehData.id = vehId
  vehData.name = name
  vehData.class = vehData.class or "default" -- maybe could use model info
  vehData.model = veh.jbeam
  vehData.config = veh.partConfig
  vehData.vehType = core_vehicles.getModel(vehData.model).model.Type
  vehData.home = vehData.home or {pos = veh:getPosition(), rot = quatFromDir(veh:getDirectionVector(), veh:getDirectionVectorUp())} -- reset transform
  vehData.aiType = vehData.aiType or "basic" -- basic, script, target, user
  vehData.aiMode = vehData.aiMode or "traffic"
  vehData.aiData = vehData.aiData or getDefaultAiData(vehData.aiType) -- various ai parameters
  vehData.aiActive = vehData.aiActive and true or false
  if vehData.locked == nil then vehData.locked = false end

  vehData._tempData = vehData._tempData or {} -- temporary data, does not get saved

  vehData.stats = {timer = 0, prevPos = vec3(), prevVel = vec3(), gForce = 0, distance = 0, avgSpeed = 0, maxSpeed = 0} -- reset stats

  veh:queueLuaCommand("mapmgr.enableTracking()") -- always enable tracking for vehicles in the traffic manager

  session.vehicles[name] = vehData
end

local function enableSignals() -- activates the traffic lights
  local tempLights = {}
  for _, id in ipairs(session.lightsSorted) do
    table.insert(tempLights, session.lights[id])
  end
  local data = {instances = tempLights, controllers = session.signalControllers, sequences = session.signalSequences}

  if data.instances[1] then
    core_trafficSignals.setupSignals(data, options.signalsKeepOriginal[0])
    core_trafficSignals.setActive(true, true)
    core_trafficSignals.debugLevel = M.debugMode and 1 or 0
    map.reset()
    session.lightsActive = true
  end
end

local function disableSignals()
  if session.lightsActive then
    core_trafficSignals.setActive(false)
    core_trafficSignals.debugLevel = 0
    session.lightsActive = false
  end
end

local function createSignalControllersAndSequences() -- creates default controllers and sequences for this editor
  session.signalElements, session.signalControllers, session.signalSequences = {}, {}, {}
  session._elementId = 10000 -- prevents id conflicts with existing signals of map

  local controllerDesc = {"(Primary)", "(Secondary)"}
  local sequenceDesc = {"(Short)", "(Medium)", "(Long)"}
  local defaultDurations = {12, 4, 1}

  for i = 1, 6 do -- creates 6 signal controllers (2 for each of the 3 sequences)
    local cName = "autoLightController"..i
    session._elementId = session._elementId + 1
    local controller = core_trafficSignals.newController({name = cName, id = session._elementId})
    controller.savedIndex = i
    controller.description = controllerDesc[i % 2 == 1 and 1 or 2]
    controller:applyDefinition("lightsBasic")

    for j, state in ipairs(controller.states) do
      state.duration = defaultDurations[j]
      if j == 1 then
        state.duration = state.duration + (math.ceil(i / 2) - 1) * 6 -- auto adjusts the green light duration
      end
    end

    table.insert(session.signalControllers, controller)
    session.signalElements[controller.id] = controller
  end

  for i = 1, 3 do -- creates 3 signal sequences
    local sName = "autoLightSequence"..i

    session._elementId = session._elementId + 1
    local sequence = core_trafficSignals.newSequence({name = sName, id = session._elementId})
    sequence.savedIndex = i
    sequence.description = sequenceDesc[i]

    sequence:createPhase({controllerData = {{id = session.signalControllers[i * 2 - 1].id, required = true}}})
    sequence:createPhase({controllerData = {{id = session.signalControllers[i * 2].id, required = true}}})

    table.insert(session.signalSequences, sequence)
    session.signalElements[sequence.id] = sequence
  end
end

local function createSignalData(name) -- creates a new signal instance
  if not name then
    session._lightId = session._lightId or #session.lightsSorted
    session._lightId = session._lightId + 1
    name = "autoLightInstance"..session._lightId
  end

  local pos = core_camera.getPosition() - core_camera.getForward():z0()
  local z = be:getSurfaceHeightBelow(pos) -- snap to ground below camera view
  if z > 1e-6 then
    pos.z = z
  end

  session._elementId = session._elementId + 1
  local maxId = 0
  for id, _ in pairs(session.signalElements) do -- quick element id duplicate check (maybe do this after loading signals)
    if session._elementId == id then
      maxId = math.max(maxId, id)
    end
  end
  if maxId > 0 then session._elementId = maxId + 1 end

  local signal = core_trafficSignals.newSignal({name = name, id = session._elementId, pos = pos})
  signal:setController(session.signalControllers[1].id)
  signal:setSequence(session.signalSequences[1].id)
  signal.choiceIndex = 1

  local ctrl = session.signalElements[signal.controllerId]
  local seq = session.signalElements[signal.sequenceId]

  session.lights[name] = signal
  session.signalElements[signal.id] = signal
  session.lightsSorted = tableKeysSorted(session.lights)
end

local function deleteLights()
  for _, id in ipairs(session.lightsSorted) do
    if scenetree.objectExistsById(session.lights[id].spawnedObjectId or 0) then
      scenetree.findObjectById(session.lights[id].spawnedObjectId):delete()
    end
  end

  table.clear(session.lights)
  table.clear(session.lightsSorted)
end

local function enableSimulation() -- starts the vehicle and traffic lights simulations, returns true if successful
  if not session then return false end

  for id, data in pairs(session.vehicles) do
    if not data.locked then
      enableVehicleAi(id)
    end
  end
  enableSignals()
  if editor.dirty then be:reloadCollision() end

  session.play = true

  log("I", logTag, "Traffic manager simulation started.")
  return true
end

local function disableSimulation() -- stops the vehicle and traffic lights simulations, returns true if successful
  if not session then return false end

  for id, data in pairs(session.vehicles) do
    if not data.locked then
      disableVehicleAi(id)
    end
  end
  disableSignals()

  session.play = false

  log("I", logTag, "Traffic manager simulation stopped.")
  return true
end

local function tabVehicleSelector()
  local availWidth = im.GetContentRegionAvailWidth()

  if not vehSelector then -- initialize vehicle selector util here, with custom settings
    vehSelector = require("/lua/ge/extensions/editor/util/vehicleSelectUtil")("Vehicle##trafficManager")
    vehSelector.enablePaints = true
    vehSelector.paintLayers = 1
    vehSelector.allowedTypes = {"Car", "Truck", "Automation"}
    vehSelector.allowedSubtypes = {"PropTraffic"}
    vehSelector:resetSelections()
  end

  im.TextUnformatted("Filter by Type:")
  -- Vehicle types: Standard, Simple, Parked, Props, Any

  if im.RadioButton2("Standard Vehicles##trafficManager", options.vehicleType, im.Int(1)) then
    vehSelector.allowedTypes = {"Car", "Truck", "Automation"}
    vehSelector.allowedSubtypes = {"PropTraffic"}
    vehSelector:resetSelections()
  end
  im.SameLine()
  im.Dummy(imSizes.dummy)
  im.SameLine()
  if im.RadioButton2("Simple Vehicles##trafficManager", options.vehicleType, im.Int(2)) then
    vehSelector.allowedTypes = {"Traffic"}
    vehSelector.allowedSubtypes = {"PropTraffic"}
    vehSelector:resetSelections()
  end
  im.SameLine()
  im.Dummy(imSizes.dummy)
  im.SameLine()
  if im.RadioButton2("Parked Vehicles##trafficManager", options.vehicleType, im.Int(3)) then
    vehSelector.allowedTypes = {"Traffic"}
    vehSelector.allowedSubtypes = {"PropParked"}
    vehSelector:resetSelections()
  end

  if availWidth >= 560 then
    im.SameLine()
    im.Dummy(imSizes.dummy)
    im.SameLine()
  end

  if im.RadioButton2("Props##trafficManager", options.vehicleType, im.Int(4)) then
    vehSelector.allowedTypes = {"Prop", "Trailer", "Utility"}
    vehSelector.allowedSubtypes = {"PropTraffic", "PropParked"}
    vehSelector:resetSelections()
  end
  im.SameLine()
  im.Dummy(imSizes.dummy)
  im.SameLine()
  if im.RadioButton2("Any##trafficManager", options.vehicleType, im.Int(5)) then
    vehSelector.allowedTypes = {"Car", "Truck", "Automation", "Trailer", "Prop", "Utility", "Traffic", "Unknown", "Any"}
    vehSelector.allowedSubtypes = {"PropTraffic", "PropParked"}
    vehSelector:resetSelections()
  end

  vehSelector:widget() -- renders the vehicle selector here

  im.Checkbox("Enable Advanced Selection", options.vehicleGroupEnabled)

  if options.vehicleGroupEnabled[0] then
    im.TextUnformatted("Advanced Mode:")

    if im.RadioButton2("Multiple Vehicles##trafficManager", options.vehicleGroupMode, im.Int(1)) then
      options.vehicleMulti[0] = 1
      ffi.copy(options.vehicleGroupFile, "")
    end
    im.SameLine()
    im.Dummy(imSizes.dummy)
    im.SameLine()
    if im.RadioButton2("Vehicle Group##trafficManager", options.vehicleGroupMode, im.Int(2)) then
      ffi.copy(options.vehicleGroupFile, "")
    end

    if options.vehicleGroupMode[0] == 2 then
      if editor.uiInputFile("Vehicle Group##trafficManager", options.vehicleGroupFile, nil, nil, {{"Vehicle group files", ".vehGroup.json"}}, im.InputTextFlags_EnterReturnsTrue) then
        local tempVehGroup = jsonReadFile(ffi.string(options.vehicleGroupFile))
        if tempVehGroup and tempVehGroup.data then
          options.vehicleMulti[0] = #tempVehGroup.data -- smartly updates the spawn amount as the group size
        end
      end
      if im.Button("Use Vehicle Groups Manager...##trafficManager") then
        if editor_multiSpawnManager then
          editor_multiSpawnManager.onWindowMenuItem()
        end
      end
      im.tooltip("Launches the Vehicle Groups tool, where you can create, edit, and save vehicle groups.")
    end

    im.PushItemWidth(inputWidth)
    if im.InputInt("Spawn Amount##trafficManagerVehicles", options.vehicleMulti, 1) then
      options.vehicleMulti[0] = clamp(options.vehicleMulti[0], 1, 100)
    end
    im.PopItemWidth()
    local tip = options.vehicleGroupMode[0] == 2 and "Vehicle group amount." or "Multiplier of the currently selected vehicle to spawn."
    im.tooltip(tip)

    im.Checkbox("Randomize Vehicle Paints##trafficManager", options.vehicleGroupRandomPaint)
  end

  im.Dummy(imSizes.dummy)

  im.PushStyleColor2(im.Col_Button, imColors.accept)
  im.PushStyleColor2(im.Col_ButtonHovered, imColors.accept)
  if im.Button("Spawn Here##trafficManagerVehicles", im.ImVec2(140, im.GetFrameHeight())) then
    spawnDelayFrames = 3
    if editor.keyModifiers.alt then altLock = true end
    mouseMode = "none"
    isUserTransform = false
    isDragging = false
  end
  im.PopStyleColor(2)
  if spawnDelayFrames == -1 then
    local tip = options.vehicleGroupEnabled[0] and "Spawns the vehicle group under the current camera view." or "Spawns the vehicle under the current camera view."
    im.tooltip(tip)
  end

  im.SameLine()

  local spawnOnClickActive = mouseMode == "spawn"
  if spawnOnClickActive then
    im.PushStyleColor2(im.Col_Button, im.GetStyleColorVec4(im.Col_ButtonActive))
    im.PushStyleColor2(im.Col_ButtonHovered, im.GetStyleColorVec4(im.Col_ButtonActive))
  end
  if im.Button("Spawn on Click##trafficManagerVehicles", im.ImVec2(140, im.GetFrameHeight())) then
    if mouseMode ~= "spawn" then
      mouseMode = "spawn"
      isUserTransform = true
      clickLock = true
    else
      mouseMode = "none"
      isUserTransform = false
    end
    isDragging = false
  end
  if spawnOnClickActive then
    im.PopStyleColor(2)
  end
  if spawnDelayFrames == -1 then
    local tip = options.vehicleGroupEnabled[0] and "Enables clicking in the world to spawn the vehicle group." or "Enables clicking in the world to spawn the vehicle."
    im.tooltip(tip)
  end

  if activeWindow == "vehicles" and mouseMode == "spawn" and not im.IsWindowHovered(im.HoveredFlags_AnyWindow) then
    if mouseHandler("Click and drag to spawn here") then
      spawnDelayFrames = 3
      if editor.keyModifiers.alt then altLock = true end
      mouseMode = "none"
    end
  end

  if spawnDelayFrames >= 0 then
    im.SameLine()
    im.TextColored(imColors.warning, "Please wait...")
  end

  if spawnDelayFrames == 0 then
    if not vehSelector.model then
      vehSelector.model = "simple_traffic"
    end
    if options.vehicleType[0] == 3 and not vehSelector.config then -- ensures that config is a parked vehicle if it was not defined
      vehSelector.model = "simple_traffic"
      vehSelector.config = "bastion_base_parked"
    end

    if not options.vehicleGroupEnabled[0] then -- single vehicle spawn
      local spawnOptions = {config = vehSelector.config, paintName = vehSelector.paintName}
      spawnOptions = fillVehicleSpawnOptionDefaults(vehSelector.model, spawnOptions)
      spawnOptions.autoEnterVehicle = false
      spawnOptions.centeredPosition = true
      if isUserTransform then
        spawnOptions.pos = anchorPos
        spawnOptions.rot = quatFromDir(vecY:rotated(quatFromDir((mousePos - anchorPos):normalized(), vecUp)), vecUp)
      else
        spawnOptions.pos = core_camera.getPosition()
        local z = be:getSurfaceHeightBelow(spawnOptions.pos) -- snap to ground below camera view
        if z > 1e-6 then
          spawnOptions.pos.z = z
        end
      end

      core_vehicles.spawnNewVehicle(vehSelector.model, spawnOptions)
    else -- multiple vehicle spawn
      local vehGroup
      local vehGroupFile = ffi.string(options.vehicleGroupFile)
      if vehGroupFile:len() > 0 then
        vehGroup = jsonReadFile(vehGroupFile)
        if vehGroup and vehGroup.data then
          vehGroup = vehGroup.data
        else
          vehGroup = nil
        end
      end

      if not vehGroup then
        vehGroup = {{model = vehSelector.model or "pickup", config = vehSelector.config, paintName = vehSelector.paintName}} -- creates a minimal vehicle group
      end

      if options.vehicleGroupRandomPaint[0] then
        for _, group in ipairs(vehGroup) do
          group.paintName = "(Random)"
        end
      end

      local spawnOptions = {startIndex = 0, gap = 12}
      if isUserTransform then
        spawnOptions.pos = anchorPos
        spawnOptions.dir = (mousePos - anchorPos):normalized()
      end

      spawnOptions.mode = altLock and "lineAhead" or "roadAhead" -- hold down Alt to ignore road snapping
      altLock = false

      core_multiSpawn.spawnGroup(vehGroup, options.vehicleMulti[0], spawnOptions)
    end
  end
  spawnDelayFrames = math.max(-1, spawnDelayFrames - 1)
end

local vehiclesPropsSorted = {"vehiclesSorted", "propsSorted"}
local function tabVehicleManager()
  im.BeginChild1("vehicleManagerList##trafficManager", im.ImVec2(240 * im.uiscale[0], 440 * im.uiscale[0]), im.WindowFlags_ChildWindow)

  local deleteId -- if this gets set, the vehicle will be queued to be deleted

  for i, key in ipairs(vehiclesPropsSorted) do
    local label = string.lower(key:gsub("Sorted", ""))
    if im.CollapsingHeader1(label.."##trafficManagerHeader", im.TreeNodeFlags_DefaultOpen) then
      im.Columns(2, key.."Columns##trafficManager", false)
      im.SetColumnWidth(0, 190 * im.uiscale[0])
      im.SetColumnWidth(1, 30 * im.uiscale[0])

      for _, nameId in ipairs(session[key]) do
        local veh = scenetree.findObject(nameId)
        if veh then
          local vehId = veh:getID()
          if not session.vehicles[nameId] then
            createVehicleData(vehId)
          end

          if session.vehicles[nameId] then
            local str = "["..tostring(vehId).."] "..veh.jbeam
            if key == "vehiclesSorted" and not map.objects[vehId] then
              str = str.." *"
            end

            local textColor = imColors.active
            if session.vehicles[nameId].locked then
              textColor = session.vehicles[nameId].aiActive and imColors.inactiveLive or imColors.inactive
            else
              textColor = session.vehicles[nameId].aiActive and imColors.activeLive or imColors.active
            end

            im.PushStyleColor2(im.Col_Text, textColor)
            if im.Selectable1(str, nameId == currSelection.vehicle, bit.bor(im.SelectableFlags_SpanAllColumns, im.SelectableFlags_AllowItemOverlap)) then
              currSelection.vehicle = nameId
              table.clear(options.aiData)
              ffi.copy(options.vehicleName, session.vehicles[nameId].name)
              ffi.copy(options.vehicleClass, session.vehicles[nameId].class)
              if not commands.isFreeCamera() then
                if scenetree.objectExists(session.playerId or 0) then
                  be:enterVehicle(0, getObjectByID(session.playerId))
                end
                commands.setFreeCamera()
              end
            end
            im.PopStyleColor()
            im.NextColumn()
            if editor.uiIconImageButton(session.vehicles[nameId].locked and editor.icons.lock or editor.icons.lock_open, imSizes.small) then
              session.vehicles[nameId].locked = not session.vehicles[nameId].locked
            end
            im.tooltip(session.vehicles[nameId].locked and "Unlock Object" or "Lock Object")
            im.NextColumn()
          end
        else
          session.vehicles[nameId] = nil
          session.objectCount = 0
        end
      end

      im.Columns(1)
    end
  end
  im.EndChild()

  if options.includePlayerVehicle[0] then
    local playerId = be:getPlayerVehicleID(0)
    if playerId and playerId > 0 then
      local playerVeh = getObjectByID(playerId)
      if playerVeh then
        local name = playerVeh:getName() or "playerVeh"
        if not session.vehicles[name] then -- TODO: simplify this
          if not scenetree.objectExists(simGroupName) then
            createSimGroup()
          end
          local innerGroup = scenetree.findObject(simGroupName)
          innerGroup:addObject(playerVeh)

          createVehicleData(playerId, {aiMode = "disabled"})
        end
      end
    end
  end

  im.SameLine()

  im.BeginChild1("vehicleManagerData##trafficManager", im.ImVec2(0, 440 * im.uiscale[0]), im.WindowFlags_None)
  local sessionData = session.vehicles[currSelection.vehicle]
  local vehId = sessionData and sessionData.id or 0
  local currVeh = getObjectByID(vehId)
  if not currVeh then
    if not next(session.vehicles) then
      im.TextUnformatted("Spawn an object via the Vehicle Selector to begin.")
    else
      im.TextUnformatted("Select an object from the list to view data.")
    end
  else
    local mapVehData = map.objects[vehId]
    local isDrivable = validVehTypes[sessionData.vehType] and true or false -- assumed drivable from vehicle type
    -- currVeh.partConfig seems to cause a crash even though vehicle got deleted
    --if isDrivable and string.find(currVeh.partConfig, "_parked") then -- parked vehicles can not be drivable
      --isDrivable = false
    --end

    mapVehData = mapVehData or {}

    im.Columns(3, "trafficManagerMainButtons")
    im.SetColumnWidth(0, 40)
    im.SetColumnWidth(1, 155)

    if editor.uiIconImageButton(editor.icons.portrait, imSizes.medium) then
      currVeh.playerUsable = true
      be:enterVehicle(0, currVeh)
      commands.setGameCamera()
    end
    im.tooltip("Enter Vehicle")
    im.NextColumn()

    if not isDrivable then im.BeginDisabled() end
    if editor.uiIconImageButton(editor.icons.play_arrow, imSizes.medium, sessionData.aiActive and im.GetStyleColorVec4(im.Col_ButtonActive)) then
      enableVehicleAi(currSelection.vehicle)
    end
    im.tooltip("Start AI")
    im.SameLine()
    if editor.uiIconImageButton(editor.icons.stop, imSizes.medium) then
      disableVehicleAi(currSelection.vehicle)
    end
    im.tooltip("Stop AI")
    if not isDrivable then im.EndDisabled() end
    im.SameLine()
    if editor.uiIconImageButton(editor.icons.refresh, imSizes.medium) then
      if sessionData._loaded then
        spawn.safeTeleport(currVeh, sessionData.home.pos, sessionData.home.rot, nil, nil, false, true)
        currVeh:queueLuaCommand('recovery.saveHome()')
        sessionData._loaded = nil
      else
        currVeh:queueLuaCommand('recovery.loadHome()')
      end
    end
    im.tooltip("Reset")
    im.SameLine()
    if editor.uiIconImageButton(editor.icons.delete_forever, imSizes.medium) then
      deleteId = vehId
    end
    im.tooltip("Delete")

    im.NextColumn()

    if editor.uiIconImageButton(editor.icons.home, imSizes.medium) then
      currVeh:queueLuaCommand('recovery.saveHome()')
      sessionData.home = {pos = currVeh:getPosition(), rot = quatFromDir(currVeh:getDirectionVector(), currVeh:getDirectionVectorUp())}
      editor.showNotification("Updated home position of current vehicle.")
    end
    im.tooltip("Save Home Position")
    im.SameLine()
    if editor.uiIconImageButton(editor.icons.cars, imSizes.medium) then
      spawnDelayFrames = 3
    end
    im.tooltip("Duplicate")

    if spawnDelayFrames >= 0 then
      im.SameLine()
      im.TextColored(imColors.warning, "Please wait...")
    end

    if spawnDelayFrames == 0 then
      local metallicPaintData = currVeh:getMetallicPaintData()
      local spawnOptions = {
        pos = currVeh:getPosition(),
        rot = quatFromDir(currVeh:getDirectionVector(), currVeh:getDirectionVectorUp()),
        model  = currVeh.jbeam,
        config = currVeh.partConfig,
        paint  = createVehiclePaint(currVeh.color, metallicPaintData[1]),
        paint2 = createVehiclePaint(currVeh.colorPalette0, metallicPaintData[2]),
        paint3 = createVehiclePaint(currVeh.colorPalette1, metallicPaintData[3]),
        autoEnterVehicle = false,
        centeredPosition = true
      }

      core_vehicles.spawnNewVehicle(currVeh.jbeam, spawnOptions)
    end
    spawnDelayFrames = math.max(-1, spawnDelayFrames - 1)

    im.NextColumn()
    im.Columns(1)

    im.Separator()

    im.TextUnformatted("Properties")
    local niceName = core_vehicle_manager.getVehicleData(vehId).vdata.information.name or currVeh.jbeam
    if niceName == "Simple Traffic Vehicle" then
      -- get config nice name instead (ask about having an alias in the data for this)
    end
    im.Selectable1(niceName.."##trafficManager", true)

    if editor.uiInputText("Name##trafficManagerVehicle", options.vehicleName, nil, im.InputTextFlags_EnterReturnsTrue) then
      local tempName = ffi.string(options.vehicleName)
      if not scenetree.findObject(tempName) then -- transfer vehicle data to new table
        currVeh:setName(tempName)
        createVehicleData(vehId, sessionData)
        session.vehicles[currSelection.vehicle] = nil
        currSelection.vehicle = tempName
        session.objectCount = 0
      else -- rename rejected, revert to previous name
        ffi.copy(options.vehicleName, currSelection.vehicle)
      end
    end
    if editor.uiInputText("Class##trafficManagerVehicle", options.vehicleClass, nil, im.InputTextFlags_EnterReturnsTrue) then
      sessionData.class = ffi.string(options.vehicleClass)
    end

    if not isDrivable then
      im.TextColored(imColors.warning, "This object is currently undrivable.")
    else
      local prevType = sessionData.aiType
      local prevMode = sessionData.aiMode

      local label = aiModes[sessionData.aiType] and aiModes[sessionData.aiType][sessionData.aiMode]
      local edited = false

      if im.BeginCombo("AI Mode##trafficManager", label or "(None)") then
        for _, t in ipairs(tableKeysSorted(aiModes)) do
          for _, mode in ipairs(tableKeysSorted(aiModes[t])) do
            if im.Selectable1(aiModes[t][mode].."##trafficManagerAiMode", sessionData.aiMode == mode) then
              sessionData.aiType = t
              sessionData.aiMode = mode
            end
          end
          im.Separator()
        end
        im.EndCombo()
      end

      if sessionData.aiType ~= prevType then -- resets the ai data tables if the ai type changed
        sessionData.aiData = getDefaultAiData(sessionData.aiType)
      end

      if sessionData.aiMode ~= prevMode and sessionData.aiActive then
        enableVehicleAi(currSelection.vehicle)
      end

      if sessionData.aiData.scriptFile ~= nil then
        options.aiData.scriptFile = options.aiData.scriptFile or im.ArrayChar(1024, sessionData.aiData.scriptFile)
        if editor.uiInputFile("ScriptAi File##trafficManagerAi", options.aiData.scriptFile, nil, nil, {{"ScriptAI Recordings", {".track.json", ".json"}}}, im.InputTextFlags_EnterReturnsTrue) then
          sessionData.aiData.scriptFile = ffi.string(options.aiData.scriptFile)
          edited = true
        end
      end

      if sessionData.aiData.targetPos ~= nil then
        -- maybe there should be a way to click and set the target position
        if im.Button("Set Target Position Here##trafficManagerAi") then
          sessionData.aiData.targetPos = core_camera.getPosition()
          local z = be:getSurfaceHeightBelow(sessionData.aiData.targetPos) -- snap to ground below camera view
          if z > 1e-6 then
            sessionData.aiData.targetPos.z = z
          end
          edited = true
        end
      end

      if sessionData.aiData.flowgraphFile ~= nil then
        options.aiData.flowgraphFile = options.aiData.flowgraphFile or im.ArrayChar(1024, sessionData.aiData.flowgraphFile)
        if editor.uiInputFile("Flowgraph File##trafficManagerAi", options.aiData.flowgraphFile, nil, nil, {{"Flowgraphs", {".flow.json"}}}, im.InputTextFlags_EnterReturnsTrue) then
          sessionData.aiData.flowgraphFile = ffi.string(options.aiData.flowgraphFile)
          sessionData._tempData.varCurrentId = nil
          sessionData._tempData.varPlayerId = nil
          edited = true
        end

        if im.Button("Check Variables") then
          local tempFg = jsonReadFile(sessionData.aiData.flowgraphFile)
          if tempFg then
            sessionData._tempData.varCurrentId = false
            sessionData._tempData.varPlayerId = false
            local varArray = tempFg.variables.customVariableOrder -- assuming that this exists
            if varArray then
              for _, var in ipairs(varArray) do
                if var == "currentId" then
                  sessionData._tempData.varCurrentId = true
                elseif var == "playerId" then
                  sessionData._tempData.varPlayerId = true
                end
              end
            end
          end
        end

        if sessionData._tempData.varCurrentId ~= nil then
          im.SameLine()
          editor.uiIconImage(sessionData._tempData.varCurrentId and editor.icons.check or editor.icons.close, imSizes.small)
          im.tooltip("Variable: currentId")
          im.SameLine()
          editor.uiIconImage(sessionData._tempData.varPlayerId and editor.icons.check or editor.icons.close, imSizes.small)
          im.tooltip("Variable: playerId")

          im.TextWrapped('Tip: If you want, you can create a number variable named "currentId" in the flowgraph to use this vehicle as input. Same with "playerId", for the ego vehicle. Ensure that you link the variables to nodes.')
        end
      end

      if im.Button("AI Parameters...") then
        im.OpenPopup("AI Parameters##trafficManager")
      end
      if im.BeginPopup("AI Parameters##trafficManager") then
        local aiData = sessionData.aiData

        if sessionData.aiMode == "chase" or sessionData.aiMode == "follow" or sessionData.aiMode == "flee" then
          if not aiData.targetName then
            aiData.targetName = getPlayerVehicle(0) and getPlayerVehicle(0):getName() or ""
          end

          im.PushItemWidth(160)
          if im.BeginCombo("Target Vehicle##trafficManagerAiMode", aiData.targetName) then
            for _, veh in ipairs(getAllVehiclesByType()) do
              local vehName = veh:getName()
              if vehName ~= currVeh:getName() then
                if im.Selectable1(vehName.."##trafficManagerAiMode", aiData.targetName == vehName) then
                  aiData.targetName = vehName
                  edited = true
                end
              end
            end
            im.EndCombo()
          end
          im.PopItemWidth()
        else
          aiData.targetName = nil
        end

        if aiData.aggression ~= nil then
          im.PushItemWidth(160)
          options.aiData.aggression = options.aiData.aggression or im.FloatPtr(aiData.aggression)
          if editor.uiSliderFloat("Risk##trafficManagerAi", options.aiData.aggression, 0.1, 1.25, "%.2f") then
            aiData.aggression = options.aiData.aggression[0]
            edited = true
          end
          im.PopItemWidth()
        end

        if aiData.speed ~= nil then
          if aiData.useSpeedLimit then
            im.BeginDisabled()
          end
          im.PushItemWidth(160)
          options.aiData.speed = options.aiData.speed or im.FloatPtr(aiData.speed)
          if editor.uiSliderFloat("Speed (m/s)##trafficManagerAi", options.aiData.speed, 0, 80, "%.1f") then
            aiData.speed = options.aiData.speed[0]
            edited = true
          end
          im.PopItemWidth()
          if aiData.useSpeedLimit then
            im.EndDisabled()
          end

          im.TextColored(imColors.inactive, string.format("%0.2f %s", convertSpeed(aiData.speed, speedUnits[1]), speedUnits[1]))
          im.SameLine()
          im.Dummy(imSizes.dummy)
          im.SameLine()
          im.TextColored(imColors.inactive, string.format("%0.2f %s", convertSpeed(aiData.speed, speedUnits[2]), speedUnits[2]))
        end

        if aiData.useSpeedLimit ~= nil then
          options.aiData.useSpeedLimit = options.aiData.useSpeedLimit or im.BoolPtr(aiData.useSpeedLimit)
          if im.Checkbox("Use Road Speed Limit##trafficManagerAi", options.aiData.useSpeedLimit) then
            aiData.useSpeedLimit = options.aiData.useSpeedLimit[0]
            edited = true
          end
        end

        if aiData.driveInLane ~= nil then
          options.aiData.driveInLane = options.aiData.driveInLane or im.BoolPtr(aiData.driveInLane)
          if im.Checkbox("Use Road Lanes##trafficManagerAi", options.aiData.driveInLane) then
            aiData.driveInLane = options.aiData.driveInLane[0]
            edited = true
          end
        end

        if aiData.avoidCars ~= nil then
          options.aiData.avoidCars = options.aiData.avoidCars or im.BoolPtr(aiData.avoidCars)
          if im.Checkbox("Avoid Collisions##trafficManagerAi", options.aiData.avoidCars) then
            aiData.avoidCars = options.aiData.avoidCars[0]
            edited = true
          end
        end

        if aiData.enableTraffic ~= nil then
          im.Separator()

          options.aiData.enableTraffic = options.aiData.enableTraffic or im.BoolPtr(aiData.enableTraffic)
          if im.Checkbox("Use as Dynamic Traffic##trafficManagerAi", options.aiData.enableTraffic) then
            aiData.enableTraffic = options.aiData.enableTraffic[0]
            edited = true
          end
          im.tooltip("If true, this vehicle will automatically respawn if it drives away from the camera view.")
        end

        im.EndPopup()
      end

      if edited and sessionData.aiActive then -- instantly updates the vehicle ai if it is already running
        enableVehicleAi(currSelection.vehicle)
      end
    end

    im.Dummy(imSizes.dummy)

    local floatFormat = "%0."..editor.getPreference("ui.general.floatDigitCount").."f"

    local pos = mapVehData.pos or currVeh:getPosition()
    options.objPos[0], options.objPos[1], options.objPos[2] = pos.x, pos.y, pos.z
    if im.InputFloat3("Position##trafficManagerVehPos", options.objPos, floatFormat, im.InputTextFlags_EnterReturnsTrue) then
      pos.x, pos.y, pos.z = options.objPos[0], options.objPos[1], options.objPos[2]
      spawn.safeTeleport(currVeh, pos, quatFromDir(currVeh:getDirectionVector(), currVeh:getDirectionVectorUp()), nil, nil, false, true)
    end

    local dirVec = mapVehData.dirVec or currVeh:getDirectionVector()
    options.objRot[0], options.objRot[1], options.objRot[2] = dirVec.x, dirVec.y, dirVec.z
    if im.InputFloat3("Direction##trafficManagerVehDirVec", options.objRot, floatFormat, im.InputTextFlags_EnterReturnsTrue) then
      dirVec.x, dirVec.y, dirVec.z = options.objRot[0], options.objRot[1], options.objRot[2]
      spawn.safeTeleport(currVeh, currVeh:getPosition(), quatFromDir(dirVec, vecUp), nil, nil, false, true)
    end

    if editor.uiIconImageButton(editor.icons.videocam, imSizes.medium) then
      spawn.safeTeleport(currVeh, core_camera.getPosition(), core_camera.getQuat(), nil, nil, false, true)
    end
    im.tooltip("Set Transform from Camera")
    im.SameLine()
    if editor.uiIconImageButton(editor.icons.mouse, imSizes.medium, mouseMode == "move" and im.GetStyleColorVec4(im.Col_ButtonActive)) then
      if mouseMode ~= "move" then
        mouseMode = "move"
        shiftLock = true
        clickLock = true
      else
        mouseMode = "none"
        shiftLock = false
      end
    end
    im.tooltip("Set Transform with Mouse (Shift key also works)")

    if not im.IsWindowHovered(im.HoveredFlags_AnyWindow) and editor.keyModifiers.shift then
      mouseMode = "move"
    elseif not editor.keyModifiers.shift and not shiftLock then
      mouseMode = "none"
    end

    if activeWindow == "vehicles" and mouseMode == "move" and not im.IsWindowHovered(im.HoveredFlags_AnyWindow) then
      if mouseHandler("Click and drag to move object to here") then
        spawn.safeTeleport(currVeh, anchorPos, quatFromDir((mousePos - anchorPos):normalized(), vecUp), nil, nil, false, true)
        shiftLock = false
        mouseMode = "none"
      end
    end

    if isDrivable then
      im.Separator()

      im.TextUnformatted("Stats")

      im.Columns(2, "trafficManagerVehStats1")
      im.SetColumnWidth(0, 120)

      im.TextUnformatted("Speed")
      im.NextColumn()
      local speed = mapVehData.vel and mapVehData.vel:length() or 0
      im.TextUnformatted(string.format("%0.2f", convertSpeed(speed, speedUnits[options.speedUnits[0]])))
      im.SameLine()
      if im.Button(speedUnits[options.speedUnits[0]]) then
        options.speedUnits[0] = options.speedUnits[0] + 1
        if options.speedUnits[0] > 3 then options.speedUnits[0] = 1 end
      end
      im.NextColumn()

      im.TextUnformatted("Distance")
      im.NextColumn()
      im.TextUnformatted(string.format("%0.2f", convertDistance(sessionData.stats.distance, distanceUnits[options.distanceUnits[0]])))
      im.SameLine()
      if im.Button(distanceUnits[options.distanceUnits[0]]) then
        options.distanceUnits[0] = options.distanceUnits[0] + 1
        if options.distanceUnits[0] > 4 then options.distanceUnits[0] = 1 end
      end
      im.NextColumn()

      im.TextUnformatted("Acceleration")
      im.NextColumn()
      im.TextUnformatted(string.format("%0.2f G", sessionData.stats.gForce))
      im.NextColumn()

      im.Columns(1)

      if not options.fullStats[0] then
        if im.Button("Show More Stats") then
          options.fullStats[0] = true
        end
      else
        im.Columns(2, "trafficManagerVehStats2")
        im.SetColumnWidth(0, 120)

        im.TextUnformatted("Timer")
        im.NextColumn()
        im.TextUnformatted(string.format("%0.1f", sessionData.stats.timer))
        im.NextColumn()

        im.TextUnformatted("Avg Speed (10 s)")
        im.NextColumn()
        im.TextUnformatted(string.format("%0.2f %s", convertSpeed(sessionData.stats.avgSpeed, speedUnits[options.speedUnits[0]]), speedUnits[options.speedUnits[0]]))
        im.NextColumn()

        im.TextUnformatted("Max Speed")
        im.NextColumn()
        im.TextUnformatted(string.format("%0.2f %s", convertSpeed(sessionData.stats.maxSpeed, speedUnits[options.speedUnits[0]]), speedUnits[options.speedUnits[0]]))
        im.NextColumn()

        im.Columns(1)
      end
    end
  end
  im.EndChild()

  im.Separator()

  im.Columns(2)
  im.SetColumnWidth(0, 240 * im.uiscale[0])

  im.TextUnformatted("Actions (All Unlocked Objects)")

  local allAiActive = session.vehiclesSorted[1] and true or false
  for _, name in ipairs(session.vehiclesSorted) do
    if not session.vehicles[name] or not session.vehicles[name].aiActive then
      allAiActive = false
      break
    end
  end

  if editor.uiIconImageButton(editor.icons.play_arrow, imSizes.medium, allAiActive and im.GetStyleColorVec4(im.Col_ButtonActive)) then
    enableSimulation()
  end
  im.tooltip("Start AI")
  im.SameLine()
  if editor.uiIconImageButton(editor.icons.stop, imSizes.medium) then
    disableSimulation()
  end
  im.tooltip("Stop AI")
  im.SameLine()
  if editor.uiIconImageButton(editor.icons.refresh, imSizes.medium) then
    for id, data in pairs(session.vehicles) do
      if scenetree.objectExists(id) and not data.locked then
        getObjectByID(data.id):queueLuaCommand('recovery.loadHome()')
        createVehicleData(data.id)
      end
    end
  end
  im.tooltip("Reset Objects")
  im.SameLine()
  if editor.uiIconImageButton(editor.icons.delete_forever, imSizes.medium) then
    if session.vehiclesSorted[1] or session.propsSorted[1] then
      setWindowConfirm("Are you sure you want to delete all unlocked objects?", deleteVehicles)
    end
  end
  im.tooltip("Delete Objects")

  im.NextColumn()

  if im.Button("More Options...##trafficManagerAllVehicles") then -- dropdown with special functions
    im.OpenPopup("Advanced Functions##trafficManagerAllVehicles")
  end
  if im.BeginPopup("Advanced Functions##trafficManagerAllVehicles") then
    if im.Selectable1("Include All Vehicles From Scene##trafficManagerAllVehicles") then -- adds the player vehicle and all other vehicles to the session data
      for _, veh in ipairs(getAllVehiclesByType()) do
        if not scenetree.objectExists(simGroupName) then
          createSimGroup()
        end
        local innerGroup = scenetree.findObject(simGroupName)
        innerGroup:addObject(veh)
      end
    end
    if im.Selectable1("Unlock All Vehicles##trafficManagerAllVehicles") then
      for id, data in pairs(session.vehicles) do
        data.locked = false
      end
    end
    im.EndPopup()
  end

  im.NextColumn()
  im.Columns(1)

  if deleteId then
    getObjectByID(deleteId):delete()
  end
end

local function windowVehicles() -- manage traffic vehicles
  if not windows.vehicles.active then return end

  if editor.beginWindow(windows.vehicles.key, windows.vehicles.name) then
    if im.IsWindowFocused(im.FocusedFlags_RootAndChildWindows) then activeWindow = "vehicles" end

    if im.BeginTabBar("Vehicles Tab##trafficManager") then
      if im.BeginTabItem("Vehicle Selector##trafficManager") then
        tabVehicleSelector()
        im.EndTabItem()
      end
      if im.BeginTabItem("Vehicle Manager##trafficManager") then
        tabVehicleManager()
        im.EndTabItem()
      end
    end
  else
    windows.vehicles.active = false
  end
  editor.endWindow()
end

local function windowTrafficLights() -- simplified traffic lights
  if not windows.lights.active then return end

  if editor.beginWindow(windows.lights.key, windows.lights.name) then
    if im.IsWindowFocused(im.FocusedFlags_RootAndChildWindows) then activeWindow = "lights" end

    if not session.lightsPreloaded then -- initialize signal controller and sequence data
      createSignalControllersAndSequences()
      session.lightsPreloaded = true
    end

    im.BeginChild1("trafficLightsList##trafficManager", im.ImVec2(160 * im.uiscale[0], 440 * im.uiscale[0]), im.WindowFlags_ChildWindow)
    for _, id in ipairs(session.lightsSorted) do
      if session.lights[id].choiceIndex then
        im.PushStyleColor2(im.Col_Text, imColors.controllers[session.lights[id].choiceIndex])
      end
      if im.Selectable1(id, id == currSelection.light) then
        currSelection.light = id
      end
      if session.lights[id].choiceIndex then
        im.PopStyleColor()
      end
    end
    im.Separator()
    if im.Selectable1("New...##trafficManagerLights", false) then
      createSignalData()
    end
    im.tooltip("Places a new traffic light marker here.")
    im.EndChild()

    im.SameLine()

    im.BeginChild1("trafficLightsData##trafficManager", im.ImVec2(0, 440 * im.uiscale[0]), im.WindowFlags_None)

    local currInstance = session.lights[currSelection.light]
    if not currInstance then
      if not next(session.lights) then
        im.Text("Create a traffic light via the left panel to begin.")
      else
        im.Text("Select an item from the list to edit details.")
      end
    else
      local queueDelete

      if editor.uiIconImageButton(editor.icons.videocam, imSizes.medium) then
        currInstance.pos = core_camera.getPosition()
        currInstance.road = currInstance:getBestRoad()
        if currInstance.road then
          currInstance.dir = vec3(currInstance.road.dir)
        end
      end
      im.tooltip("Set Transform from Camera")
      im.SameLine()
      if editor.uiIconImageButton(editor.icons.mouse, imSizes.medium, mouseMode == "move" and im.GetStyleColorVec4(im.Col_ButtonActive)) then
        if mouseMode ~= "move" then
          mouseMode = "move"
          shiftLock = true
          clickLock = true
        else
          mouseMode = "none"
          shiftLock = false
        end
      end
      im.tooltip("Set Transform with Mouse (Shift key also works)")
      im.SameLine()
      if editor.uiIconImageButton(editor.icons.delete_forever, imSizes.medium) then
        queueDelete = true
      end
      im.tooltip("Delete")

      if not im.IsWindowHovered(im.HoveredFlags_AnyWindow) and editor.keyModifiers.shift then
        mouseMode = "move"
      elseif not shiftLock and not editor.keyModifiers.shift then
        mouseMode = "none"
      end

      if activeWindow == "lights" and mouseMode == "move" and not im.IsWindowHovered(im.HoveredFlags_AnyWindow) then
        if mouseHandler("Click and drag to move light to here") then
          currInstance.pos = vec3(anchorPos)
          currInstance.road = currInstance:getBestRoad(currInstance.pos, vec3())
          if currInstance.road then
            currInstance.dir = vec3(currInstance.road.dir)
          end
          shiftLock = false
          mouseMode = "none"
        end
      end

      im.Separator()

      im.TextUnformatted("Properties")

      if not currInstance.description then
        local ctrl = session.signalElements[currInstance.controllerId] or {}
        local seq = session.signalElements[currInstance.sequenceId] or {}

        if ctrl.savedIndex then
          ctrl.innerIndex = ctrl.savedIndex % 2 == 1 and 1 or 2
        end
        local ctrlStr = ctrl.innerIndex or ctrl.name
        local ctrlDesc = ctrl.description or "(Custom)"
        local seqStr = seq.savedIndex or seq.name
        local seqDesc = seq.description or "(Custom)"
        currInstance.description = string.format("Sequence %s %s / Phase %s %s", seqStr, seqDesc, ctrlStr, ctrlDesc)
      end

      im.PushItemWidth(im.GetContentRegionAvailWidth())
      if im.BeginCombo("##trafficManagerSignalController", currInstance.description or "(None)") then
        for i, ctrl in ipairs(session.signalControllers) do
          local j = math.ceil(i * 0.5)

          if ctrl.savedIndex then
            ctrl.innerIndex = i % 2 == 1 and 1 or 2
          end
          local ctrlStr = ctrl.innerIndex or ctrl.name
          local ctrlDesc = ctrl.description or "(Custom)"
          local seq = session.signalSequences[j]
          local seqStr = seq.savedIndex or seq.name
          local seqDesc = seq.description or "(Custom)"
          local description = string.format("Sequence %s %s / Phase %s %s", seqStr, seqDesc, ctrlStr, ctrlDesc)

          if ctrl.savedIndex then
            im.PushStyleColor2(im.Col_Text, imColors.controllers[ctrl.savedIndex]) -- special color for preset sequence & controller pairs
          end
          if im.Selectable1(description, currInstance.controllerId == ctrl.id and currInstance.sequenceId == seq.id) then -- prevents duplicates by limiting controller selection
            currInstance:setController(ctrl.id)
            currInstance:setSequence(seq.id)
            currInstance.description = description
            currInstance.choiceIndex = ctrl.savedIndex
          end
          if ctrl.savedIndex then
            im.PopStyleColor()
          end

          if seq.savedIndex and i % 2 == 0 then
            im.Separator()
          end
        end
        im.EndCombo()
      end
      im.PopItemWidth()

      im.Dummy(imSizes.dummy)

      local currController = session.signalElements[currInstance.controllerId]
      local currSequence = session.signalElements[currInstance.sequenceId]

      if currController and currSequence then
        if not currController.isSimple and currController.states[1] then -- checks if state durations are enabled
          if not currController.totalDuration then
            currController.totalDuration = 0
            for _, state in ipairs(currController.states) do
              currController.totalDuration = currController.totalDuration + state.duration
            end
          end

          currSequence.totalDuration = 0
          for _, phase in ipairs(currSequence.phases) do
            for _, state in ipairs(session.signalElements[phase.controllerData[1].id].states) do
              currSequence.totalDuration = currSequence.totalDuration + state.duration
            end
          end

          im.BeginTable("lightStateDurations", #currController.states, bit.bor(im.TableFlags_RowBg, im.TableFlags_Borders))

          for i, state in ipairs(currController.states) do
            im.TableSetupColumn("state"..i, nil, clamp(state.duration, 0.01, 1e6))
          end
          for i, state in ipairs(currController.states) do
            im.TableNextColumn()
            im.TableSetBgColor(im.TableBgTarget_CellBg, im.GetColorU322(imColors.basicLights[i] or imColors.basicLights[4]), i - 1)
            im.TextUnformatted(" ")
          end
          im.EndTable()

          im.TextColored(imColors.inactive, string.format("Phase Duration: %0.2f s", currController.totalDuration))
          im.TextColored(imColors.inactive, string.format("Sequence Duration: %0.2f s", currSequence.totalDuration))

          im.Dummy(imSizes.dummy)

          -- controller state durations
          for i, state in ipairs(currController.states) do
            local stateData = currController:getStateData(state.state)
            if stateData then
              local var = im.FloatPtr(state.duration)
              im.PushItemWidth(inputWidth)
              if im.InputFloat(stateData.name.."##trafficManagerControllerState"..i, var, 0.1, 0.1, "%.2f", im.InputTextFlags_EnterReturnsTrue) then
                state.duration = math.max(0, var[0])
                currController.totalDuration = nil
              end
              im.PopItemWidth()
              if state.state == "redTrafficLight" then
                im.tooltip("This is usually the delay time until the next signal phase starts.")
              end
            end
          end
        end

        im.Dummy(imSizes.dummy)

        local var = im.FloatPtr(currSequence.startTime)
        im.PushItemWidth(inputWidth)
        if im.InputFloat("Initial Delay##trafficManagerControllerState", var, 0.01, 0.1, "%.2f", im.InputTextFlags_EnterReturnsTrue) then
          currSequence.startTime = var[0]
        end
        im.PopItemWidth()
        im.tooltip("Set to positive to delay start, or negative to skip ahead of start.")

        im.Separator()

        if not currInstance.tempSignalObjects then -- creates a temporary table of linked signal object ids
          currInstance.tempSignalObjects = currInstance:getSignalObjects(true)
        end

        im.Text("Traffic Light Objects")

        if not scenetree.objectExistsById(currInstance.spawnedObjectId or 0) then
          im.SameLine()
          if im.Button("Use Default") then -- NOTE: only works for right side of road
            local pos = vec3(currInstance.road and currInstance.road.pos or currInstance.pos)
            local offset = currInstance.road and currInstance.road.radius or 5
            pos = pos + currInstance.dir:cross(vecUp) * (offset + 1)
            local rot = quatFromDir(currInstance.dir:z0(), vecUp)
            local obj = currInstance:createSignalObject("art/shapes/objects/s_trafficlight_boom_sn.dae", pos, rot)

            if obj then
              if not scenetree.objectExists(simGroupName) then
                createSimGroup()
              end
              currInstance.spawnedObjectId = obj:getID()
              scenetree[simGroupName]:addObject(obj.obj)
              currInstance:linkSignalObject(currInstance.spawnedObjectId)
              table.insert(currInstance.tempSignalObjects, currInstance.spawnedObjectId)
            end
          end
          im.tooltip("Places and links a standard traffic light object.")
        end

        if editor.uiIconImageButton(editor.icons.add_circle, imSizes.medium, tempEditMode == "objectSelect" and im.GetStyleColorVec4(im.Col_ButtonActive)) then
          if tempEditMode ~= "objectSelect" then
            tempEditMode = "objectSelect"
          else
            tempEditMode = nil
          end
        end
        im.tooltip("Add Object to List")

        im.SameLine()
        if editor.uiIconImageButton(editor.icons.remove_circle, imSizes.medium) then
          local objId = table.remove(currInstance.tempSignalObjects)
          if objId then
            currInstance:unlinkSignalObject(objId)
          end
        end
        im.tooltip("Remove Object from List")

        if tempEditMode == "objectSelect" then
          local rayCastInfo = cameraMouseRayCast(true, bit.bor(SOTStaticShape, SOTStaticObject))
          if rayCastInfo and im.IsMouseClicked(0) and editor.isViewportHovered() and not editor.isAxisGizmoHovered() then
            local hoveredObject = rayCastInfo.object
            if hoveredObject and hoveredObject:getClassName() == "TSStatic" and editor.isObjectSelectable(hoveredObject) then
              local objId = hoveredObject:getID()
              currInstance:linkSignalObject(objId)
              if not arrayFindValueIndex(currInstance.tempSignalObjects, objId) then
                table.insert(currInstance.tempSignalObjects, objId)
              end

              tempEditMode = nil
            end
          end
        end

        if tempEditMode == "objectSelect" then
          im.SameLine()
          im.TextColored(imColors.warning, "Object selection mode active.")
        end

        if currInstance.tempSignalObjects[1] then
          local columnWidth = im.GetContentRegionAvailWidth() * 0.25
          im.Columns(4, "signalObjects", false)
          im.SetColumnWidth(0, columnWidth)
          im.SetColumnWidth(1, columnWidth)
          im.SetColumnWidth(2, columnWidth)

          for _, id in ipairs(currInstance.tempSignalObjects) do
            if im.Button(tostring(id).."##signalObject", im.ImVec2(columnWidth - im.GetStyle().ItemSpacing.x, 20 * im.uiscale[0])) then
              editor.clearObjectSelection()
              editor.selectObjects({id})
              editor.fitViewToSelection()
            end
            im.tooltip("View")
            im.NextColumn()
          end

          im.Columns(1)
        else
          im.TextWrapped("Object list is empty; click the Add button and select a traffic signal object in the world.")
        end

        if queueDelete then
          if scenetree.objectExistsById(session.lights[currSelection.light].spawnedObjectId or 0) then
            scenetree.findObjectById(session.lights[currSelection.light].spawnedObjectId):delete()
          end

          session.lights[currSelection.light] = nil
          session.lightsSorted = tableKeysSorted(session.lights)
        end
      else
        im.TextColored(imColors.error, "Signal phase or sequence errors!")
      end
    end

    im.EndChild()

    im.Separator()

    im.Columns(2)
    im.SetColumnWidth(0, 160 * im.uiscale[0])

    im.TextUnformatted("Controls")

    if editor.uiIconImageButton(editor.icons.play_arrow, imSizes.medium, session.lightsActive and im.GetStyleColorVec4(im.Col_ButtonActive)) then
      enableSignals()
    end
    im.tooltip("Play")
    im.SameLine()
    if editor.uiIconImageButton(editor.icons.stop, imSizes.medium) then
      disableSignals()
    end
    im.tooltip("Stop")
    im.SameLine()
    if editor.uiIconImageButton(editor.icons.delete_forever, imSizes.medium) then
      if session.lightsSorted[1] then
        setWindowConfirm("Are you sure you want to delete all traffic lights?", deleteLights)
      end
    end
    im.tooltip("Delete Lights")

    im.NextColumn()

    if im.Button("Use Advanced Editor...##trafficManager") then
      if editor_trafficSignalsEditor then
        editor_trafficSignalsEditor.onWindowMenuItem()
      end
    end
    im.tooltip("Launches the Traffic Signals Editor, for advanced setup of signals.")

    im.Checkbox("Keep Original Traffic Lights from Map##trafficManager", options.signalsKeepOriginal)
    im.tooltip("If true, the traffic lights from the map stay the same while the custom ones are used.")

    im.NextColumn()
    im.Columns(1)
  else
    windows.lights.active = false
  end
  editor.endWindow()
end

local function windowTrafficSigns() -- place traffic signs

  --[[ NOTE: Road sign properties for vehicle AI are not a feature yet.
  Therefore, these signs are visual only and do not affect vehicle AI.
  Also, it would be better to use TSStatic signs instead of full objects.
  These TSStatic signs need to be available in a common folder, for all levels to use.
  ]]--

  if not windows.signs.active then return end

  if editor.beginWindow(windows.signs.key, windows.signs.name) then
    if im.IsWindowFocused(im.FocusedFlags_RootAndChildWindows) then activeWindow = "signs" end

    if not signSelector then
      signSelector = {model = "roadsigns", config = "stop", models = {}}
      local modelKeys = tableKeysSorted(core_vehicles.getModelList().models)
      for _, model in ipairs(modelKeys) do
        if string.find(model, "signs") and core_vehicles.getModel(model).model.Type == "Prop" then
          table.insert(signSelector.models, model)
        end
      end
    end

    local label
    if signSelector.model and signSelector.config then
      local modelData = core_vehicles.getModel(signSelector.model)
      if modelData then
        label = (modelData and modelData.configs[signSelector.config]) and modelData.configs[signSelector.config].Configuration.." ["..signSelector.config.."]"
      end
    end
    if im.BeginCombo("Road Signs##trafficManagerSigns", label or "(None)") then
      for _, m in ipairs(signSelector.models) do
        for _, c in ipairs(tableKeysSorted(core_vehicles.getModel(signSelector.model).configs)) do
          label = core_vehicles.getModel(m).configs[c].Configuration.." ["..c.."]"
          if im.Selectable1(label.."##trafficManagerSignConfig", signSelector.model == m and signSelector.config == c) then
            signSelector.model = m
            signSelector.config = c
          end
        end
        im.Separator()
      end
      im.EndCombo()
    end

    im.Dummy(imSizes.dummy)

    local spawnReady = false

    im.PushStyleColor2(im.Col_Button, imColors.accept)
    im.PushStyleColor2(im.Col_ButtonHovered, imColors.accept)
    if im.Button("Spawn Here##trafficManagerSigns", im.ImVec2(140, im.GetFrameHeight())) then
      spawnReady = true
      mouseMode = "none"
      isUserTransform = false
      isDragging = false
    end
    im.PopStyleColor(2)
    im.tooltip("Spawns the sign under the current camera view.")

    im.SameLine()

    local spawnOnClickActive = mouseMode == "spawn"
    if spawnOnClickActive then
      im.PushStyleColor2(im.Col_Button, im.GetStyleColorVec4(im.Col_ButtonActive))
      im.PushStyleColor2(im.Col_ButtonHovered, im.GetStyleColorVec4(im.Col_ButtonActive))
    end
    if im.Button("Spawn on Click##trafficManagerSigns", im.ImVec2(140, im.GetFrameHeight())) then
      if mouseMode ~= "spawn" then
        mouseMode = "spawn"
        isUserTransform = true
        clickLock = true
      else
        mouseMode = "none"
        isUserTransform = false
      end
      isDragging = false
    end
    if spawnOnClickActive then
      im.PopStyleColor(2)
    end
    im.tooltip("Enables clicking in the world to spawn the sign.")

    im.Dummy(imSizes.dummy)

    im.TextColored(imColors.inactive, "WIP! Sign actions not available yet.")

    if activeWindow == "signs" and mouseMode == "spawn" and not im.IsWindowHovered(im.HoveredFlags_AnyWindow) then
      if mouseHandler("Click and drag to spawn here") then
        spawnReady = true
        mouseMode = "none"
      end
    end

    if spawnReady then
      local spawnOptions = {config = signSelector.config}
      spawnOptions = fillVehicleSpawnOptionDefaults(signSelector.model, spawnOptions)
      spawnOptions.autoEnterVehicle = false
      spawnOptions.centeredPosition = true
      if isUserTransform then
        spawnOptions.pos = anchorPos
        spawnOptions.rot = quatFromDir(vecY:rotated(quatFromDir((mousePos - anchorPos):normalized(), vecUp)), vecUp)
      else
        spawnOptions.pos = core_camera.getPosition()
        local z = be:getSurfaceHeightBelow(spawnOptions.pos) -- snap to ground below camera view
        if z > 1e-6 then
          spawnOptions.pos.z = z
        end
      end

      core_vehicles.spawnNewVehicle(signSelector.model, spawnOptions)
    end
  else
    windows.signs.active = false
  end
  editor.endWindow()
end

local function windowOptions()
  if im.BeginPopup(windows.options.key) then
    im.PushItemWidth(300 * im.uiscale[0])
    if editor.uiInputText("Session Name##trafficManager", options.sessionName, nil, im.InputTextFlags_EnterReturnsTrue) then
      session.name = ffi.string(options.sessionName)
    end
    im.PopItemWidth()

    im.PushItemWidth(300 * im.uiscale[0])
    if editor.uiInputText("Author##trafficManager", options.sessionAuthor, nil, im.InputTextFlags_EnterReturnsTrue) then
      session.author = ffi.string(options.sessionAuthor)
    end
    im.PopItemWidth()

    im.PushItemWidth(300 * im.uiscale[0])
    if editor.uiInputTextMultiline("Description##trafficManager", options.sessionDescription, 4096, im.ImVec2(0, 80 * im.uiscale[0])) then
      session.description = ffi.string(options.sessionDescription)
    end
    im.PopItemWidth()

    im.Separator()

    if not shipping_build and im.Button("Dump Data (Debug)") then
      dump(session)
    end

    im.Checkbox("Always Include Player Vehicle##trafficManager", options.includePlayerVehicle)

    im.Checkbox("Save Traffic Lights Separately##trafficManager", options.signalsSaveApart)

    if im.Checkbox("Enable Debug Mode##trafficManager", options.debugMode) then
      M.debugMode = options.debugMode[0]
      core_trafficSignals.debugLevel = M.debugMode and 1 or 0
    end

    if im.Selectable1("Reset Session##trafficManager") then
      setWindowConfirm("Are you sure you want to reset everything?", resetAll)
    end
    im.EndPopup()
  else
    windows.options.active = false
  end
end

local function loadSession(filePath)
  if not filePath then return end

  if string.endswith(filePath, ".prefab.json") then -- tries to load the actual data file instead of the prefab
    filePath = filePath:gsub(".prefab.json", ".json")
  end

  local data = jsonReadFile(filePath)
  if data then
    resetSession(true)

    ffi.copy(options.sessionName, data.name)
    ffi.copy(options.sessionAuthor, data.author)
    ffi.copy(options.sessionDescription, data.description)
    options.includePlayerVehicle[0] = data.includePlayerVehicle
    options.signalsKeepOriginal[0] = data.signalsKeepOriginal
    options.signalsSaveApart[0] = data.signalsSaveApart
    options.debugMode[0] = data.debugMode
    session.playerId = be:getPlayerVehicleID(0)

    M.debugMode = data.debugMode
    core_trafficSignals.debugLevel = M.debugMode and 1 or 0

    local fp, fn = path.splitWithoutExt(filePath)
    local prefabFilePath = fp..fn..".prefab.json"
    if FS:fileExists(prefabFilePath) then
      local name = generateObjectNameForClass("Prefab", fn)
      local prefab = spawnPrefab(name, prefabFilePath, "0 0 0", "0 0 1", "1 1 1", true)

      if prefab then
        editor.selectObjects({prefab:getID()})
        local groups = editor.explodeSelectedPrefab() -- reverts prefab back into SimGroup
        groups[1]:setName(simGroupName)
        be:reloadCollision()
      end
    end

    if data.vehicles then
      for _, data in ipairs(data.vehicles) do
        local veh = scenetree.findObject(data.name) -- vehicles need to be referenced by name instead of id when loading from prefab
        if veh then
          createVehicleData(veh:getID(), data)
          local sessionData = session.vehicles[data.name]
          if sessionData then
            sessionData.home.pos = vec3(sessionData.home.pos)
            sessionData.home.rot = quat(sessionData.home.rot)
            sessionData._loaded = true -- temporary flag, gets unset when home transform is processed
          end
        end
      end
    end

    if data.signals then
      core_trafficSignals.loadSignals() -- load original signals, if applicable
      core_trafficSignals.setActive(true, true) -- this is required to actually run the signals simulation
      -- see comment in core_trafficSignals.setActive for more details

      local signalsData = data.signals
      if data.signalsFile then
        signalsData = jsonReadFile(data.signalsFile) or data.signals
      end

      session.signalControllers, session.signalSequences, session.signalElements = {}, {}, {}

      for _, instance in ipairs(signalsData.instances or {}) do
        instance.pos = vec3(instance.pos)
        instance.dir = vec3(instance.dir)
        local obj = core_trafficSignals.newSignal(instance)
        obj.choiceIndex = instance.choiceIndex
        session.lights[obj.name] = obj
        session.signalElements[obj.id] = obj
      end
      for _, ctrl in ipairs(signalsData.controllers or {}) do
        local obj = core_trafficSignals.newController(ctrl)
        obj.description = ctrl.description
        obj.savedIndex = ctrl.savedIndex
        table.insert(session.signalControllers, obj)
        session.signalElements[obj.id] = obj
      end
      for _, sequence in ipairs(signalsData.sequences or {}) do
        local obj = core_trafficSignals.newSequence(sequence)
        obj.description = sequence.description
        obj.savedIndex = sequence.savedIndex
        table.insert(session.signalSequences, obj)
        session.signalElements[obj.id] = obj
      end

      session.lightsSorted = tableKeysSorted(session.lights)
    else
      core_trafficSignals.loadSignals() -- attempt to load original traffic signals
    end

    if getObjectByID(session.playerId) then
      be:enterVehicle(0, getObjectByID(session.playerId))
    end

    prevFilePath = path.split(filePath)

    if data.level ~= getCurrentLevelIdentifier() then -- checks if the level from the data matches the current level
      editor.showNotification("Warning, level mismatch from data ("..tostring(data.level)..").", nil, nil, 10)
      log("W", logTag, "Wrong level! Expected: "..tostring(data.level)..", Actual: "..tostring(getCurrentLevelIdentifier()))
    else
      log("I", logTag, "Traffic manager session loaded: "..filePath)
      editor.showNotification("Session data loaded.", nil, nil, 5)
    end
  else
    editor.showNotification("Failed to load session data!", nil, nil, 5)
    log("E", logTag, "Failed to load session data!")
  end
end

local function saveSession(filePath)
  if not filePath then return end

  if not next(session.vehicles) and not next(session.lights) and not next(session.signs) then -- checks if there is any data to save
    editor.showNotification("Missing session data!", nil, nil, 5)
    log("E", logTag, "Failed to save session data!")
    return
  end

  if string.endswith(filePath, ".prefab.json") then -- tries to save the actual data file instead of the prefab
    filePath = filePath:gsub(".prefab.json", ".json")
  end

  local fp, fn = path.splitWithoutExt(filePath)
  local prefabFilePath = fp..fn..".prefab.json"

  local saveData = {}
  saveData.name = session.name
  saveData.author = session.author
  saveData.description = session.description
  saveData.level = getCurrentLevelIdentifier()
  saveData.version = version

  saveData.includePlayerVehicle = options.includePlayerVehicle[0]
  saveData.signalsKeepOriginal = options.signalsKeepOriginal[0]
  saveData.signalsSaveApart = options.signalsSaveApart[0]
  saveData.debugMode = options.debugMode[0]

  saveData.vehicles = {}
  saveData.signals = {instances = {}, controllers = {}, sequences = {}}

  for _, key in ipairs(vehiclesPropsSorted) do
    for i, nameId in ipairs(session[key]) do
      if scenetree.objectExists(nameId) then
        local obj = scenetree.findObject(nameId)
        spawn.safeTeleport(obj, obj:getPosition(), quatFromDir(obj:getDirectionVector(), obj:getDirectionVectorUp()), nil, nil, false, true)

        local vehData = deepcopy(session.vehicles[nameId])
        vehData.home.pos = session.vehicles[nameId].home.pos:toTable()
        vehData.home.rot = session.vehicles[nameId].home.rot:toTable()

        vehData.aiActive, vehData.stats, vehData._tempData = nil, nil, nil
        table.insert(saveData.vehicles, vehData)
      end
    end
  end

  if session.signalControllers then
    for i, id in ipairs(session.lightsSorted) do
      local processed = session.lights[id]:onSerialize()
      processed.choiceIndex = session.lights[id].choiceIndex
      table.insert(saveData.signals.instances, processed)
    end
    for i, v in ipairs(session.signalControllers) do
      local processed = v:onSerialize()
      processed.description = v.description
      processed.savedIndex = v.savedIndex
      table.insert(saveData.signals.controllers, processed)
    end
    for i, v in ipairs(session.signalSequences) do
      local processed = v:onSerialize()
      processed.description = v.description
      processed.savedIndex = v.savedIndex
      table.insert(saveData.signals.sequences, processed)
    end
  end

  if options.signalsSaveApart[0] then
    saveData.signalsFile = fp..fn.."_signals.json"
    local signalsData = {instances = saveData.signals.instances, controllers = saveData.signals.controllers, sequences = saveData.signals.sequences}
    jsonWriteFile(saveData.signalsFile, signalsData, true)

    table.clear(saveData.signals.instances)
    table.clear(saveData.signals.controllers)
    table.clear(saveData.signals.sequences)
  end

  if scenetree.objectExists(simGroupName) then
    editor.selectObjects({scenetree.findObject(simGroupName):getID()})
    local prefab = editor.createPrefabFromObjectSelection(prefabFilePath, nil, "auto") -- saves prefab file
    editor.selectObjects({prefab:getID()})
    local groups = editor.explodeSelectedPrefab() -- reverts prefab back into SimGroup
    groups[1]:setName(simGroupName)
  end

  jsonWriteFile(filePath, saveData, true)

  prevFilePath = path.split(filePath)

  log("I", logTag, "Traffic manager session saved: "..filePath)
  editor.showNotification("Session data saved.", nil, nil, 5)
end

local function getSessionData()
  return session
end

local function debugDraw()
  if not session then return end

  if windows.vehicles.active then
    local sessionData = session.vehicles[currSelection.vehicle]
    if sessionData and (M.debugMode or not sessionData.aiActive) then
      if sessionData.aiData.targetPos then
        debugDrawer:drawCylinder(sessionData.aiData.targetPos, sessionData.aiData.targetPos + vec3(0, 0, 25), 0.2, debugColors.main)
      end
      if sessionData.home then
        if not sessionData.home.p1 then
          local veh = scenetree.findObject(currSelection.vehicle)
          if veh then
            local oobb = veh:getSpawnWorldOOBB()
            sessionData.home.p1 = linePointFromXnorm(oobb:getPoint(0), oobb:getPoint(3), 0.5)
            sessionData.home.p2 = linePointFromXnorm(oobb:getPoint(4), oobb:getPoint(7), 0.5)
          end
        end
        if sessionData.home.p1 then
          debugDrawer:drawSquarePrism(sessionData.home.p1, sessionData.home.p2, Point2F(0.2, 2.2), Point2F(0.2, 2.2), debugColors.main) -- draws a box under the vehicle
        end
      end
    end
  end

  if windows.lights.active and not session.lightsActive then
    for _, id in ipairs(session.lightsSorted) do
      session.lights[id]:drawDebug(true, id == currSelection.light, false, session.lights[id].choiceIndex and debugColors.controllers[session.lights[id].choiceIndex], 5, 2)
    end
  end
end

local function onEditorGui()
  if not editor.editMode or editor.editMode.displayName ~= realName then return end

  if editor.beginWindow(editModeName, windows.main.name) then
    inputWidth = 100 * im.uiscale[0]

    im.Columns(2)
    im.SetColumnWidth(0, 240)
    for _, key in ipairs(windowsSorted) do
      if editor.uiIconImageButton(editor.icons[windows[key].icon], imSizes.large, windows[key].active and imColors.inactive or imColors.active) then
        if not windows[key].active then
          if not windows[key].popup then
            editor.showWindow(windows[key].key)
          else
            im.OpenPopup(windows[key].key)
          end
          windows[key].active = true
        else
          if not windows[key].popup then
            editor.hideWindow(windows[key].key)
          end
          windows[key].active = false
        end
      end
      im.tooltip(windows[key].name)
      im.SameLine()
      im.Dummy(imSizes.dummy)
      im.SameLine()
    end
    im.NextColumn()

    if editor.uiIconImageButton(editor.icons.save, imSizes.large) then
      editor_fileDialog.saveFile(function(data) saveSession(data.filepath) end, {{"Session Data Files", ".json"}}, false, prevFilePath)
    end
    im.tooltip("Save Session")
    im.SameLine()
    im.Dummy(imSizes.dummy)
    im.SameLine()
    if editor.uiIconImageButton(editor.icons.folder, imSizes.large) then
      editor_fileDialog.openFile(function(data) loadSession(data.filepath) end, {{"Session Data Files", ".json"}}, false, prevFilePath)
    end
    im.tooltip("Load Session")

    im.NextColumn()
    im.Columns(0)

    windowVehicles()
    windowTrafficLights()
    windowTrafficSigns()
    windowOptions()

    if confirmData.ready then
      im.OpenPopup("Confirm##trafficManager")
      confirmData.ready = nil
    end

    if im.BeginPopupModal("Confirm##trafficManager", nil, im.WindowFlags_AlwaysAutoResize) then
      im.TextUnformatted(confirmData.txt)
      im.Dummy(imSizes.dummy)

      if im.Button("YES", im.ImVec2(inputWidth, 20 * im.uiscale[0])) then
        confirmData.yesFunc()
        im.CloseCurrentPopup()
      end
      im.SameLine()
      if im.Button("NO", im.ImVec2(inputWidth, 20 * im.uiscale[0])) then
        confirmData.noFunc()
        im.CloseCurrentPopup()
      end
    end

    mousePos:set(staticRayCast() or vecUp)

    debugDraw()

    clickLock = false
  end

  editor.endWindow()
end

local function onUpdate(dt, dtSim)
  if not session then return end

  if not scenetree.objectExists(simGroupName) then
    if session.active then
      resetSession()
    end
    return
  end

  local group = scenetree.findObject(simGroupName)

  if session.vehiclesTemp then -- adds newly spawned vehicles to the traffic session SimGroup
    for _, id in ipairs(session.vehiclesTemp) do
      local prefix = "obj"
      local obj = getObjectByID(id)
      local modelData = core_vehicles.getModel(obj.jbeam)
      if modelData then
        prefix = string.lower(modelData.model.Type or "unknown")
      end

      obj:setName(prefix..tostring(id)) -- try to make this a unique name
      group:addObject(obj)
    end
    session.vehiclesTemp = nil
  end

  if session.objectCount ~= group:getCount() then -- updates session data whenever a change is detected in the traffic session SimGroup
    checkSimGroup()
  end

  tickTimer = tickTimer + dtSim

  for _, data in pairs(session.vehicles) do
    local mapVehData = map.objects[data.id] -- ensures only drivable vehicles get stat updates
    if mapVehData then
      data.isDrivable = true
      data._tempData.frameDelay = nil

      local distanceStep = mapVehData.pos:distance(data.stats.prevPos)
      if distanceStep >= 10 then distanceStep = 0 end

      if data.stats.timer == 0 then
        data.stats.avgSpeed = 0
        data.stats.tempSpeedTbl = {}
      else
        local speed = mapVehData.vel:length()
        data.stats.gForce = (mapVehData.vel:distance(data.stats.prevVel) / math.max(1e-12, dtSim)) / 9.81
        data.stats.distance = data.stats.distance + distanceStep
        data.stats.maxSpeed = math.max(speed, data.stats.maxSpeed)

        if tickTimer >= 0.25 then
          table.insert(data.stats.tempSpeedTbl, speed)
          if data.stats.tempSpeedTbl[41] then -- 40 table entries = 10 seconds
            table.remove(data.stats.tempSpeedTbl, 1)
          end

          data.stats.avgSpeed = 0
          for _, v in ipairs(data.stats.tempSpeedTbl) do
            data.stats.avgSpeed = data.stats.avgSpeed + v
          end
          data.stats.avgSpeed = data.stats.avgSpeed / math.max(1, #data.stats.tempSpeedTbl)
        end
      end

      data.stats.prevVel:set(mapVehData.vel)
      data.stats.prevPos:set(mapVehData.pos)

      if data.aiActive then
        data.stats.timer = data.stats.timer + dtSim
      end
    else
      data.isDrivable = false
      if be:getEnabled() then
        if data._tempData.frameDelay then
          data.aiType = "basic"
          data.aiMode = "stop"
          data.aiData = {}
        else
          data._tempData.frameDelay = true -- just in case map object data is not ready on this frame
        end
      end
    end
  end

  if tickTimer >= 0.25 then
    tickTimer = tickTimer - 0.25
  end
end

local function onVehicleSpawned(vehId) -- whenever a vehicle gets spawned while the corresponding window is active, move it to the traffic session SimGroup
  if not session then return end

  if windows.vehicles.active or windows.signs.active then
    if not scenetree.objectExists(simGroupName) then
      createSimGroup()
    end

    session.vehiclesTemp = session.vehiclesTemp or {} -- uses existing table if multiple vehicles were spawned on the same frame
    table.insert(session.vehiclesTemp, vehId) -- the resulting table gets processed on the next frame, intentionally
  end
end

local function onVehicleResetted(vehId) -- whenever a vehicle gets resetted, reset its stats
  if not session then return end

  for _, data in pairs(session.vehicles) do
    if vehId == data.id then
      createVehicleData(vehId)
    end
  end
end

local function onActivate()
  if not session then
    resetSession()
    checkSimGroup()
  end
  editor.clearObjectSelection()
  editor.showWindow(editModeName)
end

local function onDeactivate()
  editor.hideWindow(editModeName)
end

local function onSerialize()
  local data = {}
  return data
end

local function onDeserialized(data)
end

local function onWindowMenuItem()
  if not session then
    resetSession()
    checkSimGroup()
  end
  editor.clearObjectSelection()
  editor.showWindow(editModeName)
end

local function onEditorInitialized()
  editor.editModes[editModeName] = {
    displayName = windows.main.name,
    onActivate = onActivate,
    onDeactivate = onDeactivate,
    auxShortcuts = {},
    icon = editor.icons.traffic,
    iconTooltip = windows.main.name,
    hideObjectIcons = true
  }
  editor.editModes[editModeName].auxShortcuts[editor.AuxControl_LMB] = "Select Object"
  editor.editModes[editModeName].auxShortcuts[bit.bor(editor.AuxControl_LMB, editor.AuxControl_Shift)] = "Move Object"
  editor.editModes[editModeName].auxShortcuts[editor.AuxControl_Alt] = "Alt"

  editor.registerWindow(editModeName, windows.main.size)
  for _, key in ipairs(windowsSorted) do
    editor.registerWindow(windows[key].key, windows[key].size, imDefaultPos)
  end
end

M.loadSession = loadSession
M.saveSession = saveSession
M.getSessionData = getSessionData
M.enableSimulation = enableSimulation
M.disableSimulation = disableSimulation

M.onVehicleSpawned = onVehicleSpawned
M.onVehicleResetted = onVehicleResetted
M.onEditorInitialized = onEditorInitialized
M.onWindowMenuItem = onWindowMenuItem
M.onEditorGui = onEditorGui
M.onUpdate = onUpdate
M.onSerialize = onSerialize
M.onDeserialized = onDeserialized

return M