-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local logTag = 'TechGE'
local M = {}
M.dependencies = {"ui_imgui"}

local im = ui_imgui

local tcom = require('tech/techCommunication')
local scenariosLoader = require('scenario/scenariosLoader')
local procPrimitives = require('util/trackBuilder/proceduralPrimitives')
local jbeamIO = require('jbeam/io')
local techUtils = require('tech/techUtils')

local tcomParams = {
  ip = '127.0.0.1',
  port = 25252,
  debug = nil
}

local quitRequested = false

local conSleep = 1

local blocking = {
  reason = nil,
  data = nil,
  socket = nil,
}

local frameDelayFuncQueue = {}

local sensors = {}

-- Containers for each sensor type, where the keys are the unique sensor name (given in beamNGpy), and the values are the unique sensor Id
-- in the simulator. Any interaction with the sensors in the simulator must be done through this unique sensor Id number.
local cameras = {}
local lidars = {}
local ultrasonics = {}
local radars = {}
local advancedIMUs = {}
local GPSs = {}
local powertrains = {}
local meshes = {}
local idealRADARs = {}
local roadsSensors = {}
local vehicleFeeders = {}

local objectCount = 1

local server = nil
local clients = nil

local missingLicenseFeature = 'MISSING_LICENSE'

local debugObjects = { spheres = {},
                       dynamicSpheres = {},
                       polylines = {},
                       cylinders = {},
                       triangles = {},
                       rectangles ={},
                       text = {},
                       squarePrisms = {}
                      }
local debugObjectCounter = {sphereNum = 0,
                            dynamicSphereNum = 0,
                            lineNum = 0,
                            cylinderNum = 0,
                            triangleNum = 0,
                            rectangleNum = 0,
                            textNum = 0,
                            prismNum = 0
                          }

local scenarios = nil
local currentMissionState = nil

local showServerGUI = false
local openServerGuiData = {
  ip = im.ArrayChar(40, '127.0.0.1'),
  allInterfaces = im.BoolPtr(false),
  port = im.IntPtr(tcomParams.port),
}

-- Helper functions

local function addFrameDelayFunc(func, delay)
  table.insert(frameDelayFuncQueue, {callback=func, frameCountDown=delay})
end

local function block(reason, request, data)
  if blocking.reason ~= nil then
    request:sendBNGError('Cannot fullfill this request. It needs blocking, but BeamNG.tech is already blocked (\'' .. blocking.reason .. '\' <- \'' .. reason .. '\').')
    return false
  end
  blocking.reason = reason
  blocking.socket = request
  blocking.data = data

  return true
end

local function isBlocking(reason)
  if reason == nil then
    return blocking.reason ~= nil
  end
  return blocking.reason == reason
end

local function stopBlocking()
  local tmp, tmpData = blocking.socket, blocking.data
  blocking.reason = nil
  blocking.socket = nil
  blocking.data = nil
  return tmp, tmpData
end

local function checkVehicleInfoPending()
  if isBlocking('vehicleInfo') then
    local vehicleInfoPending = blocking.data.vehicleInfoPending
    if next(vehicleInfoPending) == nil then
      local vehicleInfo = blocking.data.vehicleInfo
      local resp = {}
      for _, v in pairs(vehicleInfo) do
        resp[v.name] = v
      end
      resp = {type = 'GetCurrentVehicles', result = resp}
      local waiting = stopBlocking()
      waiting:sendResponse(resp)
    end
  end
end


local function getRunningFlowgraphManager()
  local mgrs = extensions.core_flowgraphManager.getAllManagers()
  for i = 1, #mgrs do
    if mgrs[i].runningState == 'running' then
      return mgrs[i]
    end
  end
end

local function getCurrentMissionMainButton()
  local mgrs = extensions.core_flowgraphManager.getAllManagers()
  local button = nil
  for i = 1, #mgrs do
    if mgrs[i].runningState ~= 'running' then goto continue end
    local ui = mgrs[i].modules.ui
    if not ui then goto continue end
    local uiLayout = ui.uiLayout
    if not uiLayout then goto continue end
    local buttons = uiLayout.buttons
    if not buttons then goto continue end

    for i = 1, #buttons do
      if buttons[i].main then
        button = buttons[i]
        goto foundbutton
      end
    end
    ::continue::
  end

  ::foundbutton::
  return button
end

local function translateNested(name)
  if name == nil then return nil end
  name = translateLanguage(name, name, true)
  local result = name
  for match, translationString in name:gmatch('(%{%{\'(.+)\' | translate%}%})') do
    result = result:gsub(match, translateLanguage(translationString, translationString, true))
  end

  return result
end

local function missionPathFromId(missionId)
  local missionInfoFile = 'info.json'
  local missionsDir = '/gameplay/missions/'

  return missionsDir .. missionId .. '/' .. missionInfoFile
end

local function refreshScenarioList()
  scenarios = {}
  local scenarioList = scenariosLoader.getList(nil, true)
  for _, v in ipairs(scenarioList) do
    local scenarioPath = nil
    if v.sourceFile then
      scenarioPath = v.sourceFile
      v.isMission = false
    else -- it's a mission
      scenarioPath = missionPathFromId(v.scenarioName)
      v.isMission = true
    end
    v.name = translateNested(v.name)
    v.description = translateNested(v.description)
    if v.levelName == nil then
      _, v.levelName = v.map:gmatch('([^%.]+)')
    end
    scenarios[scenarioPath:lower()] = v
  end
end

local function reportMissingLicenseFeature(request)
  local msg = 'This feature requires a BeamNG.tech license.'
  log('E', logTag, msg)
  request:sendBNGValueError(msg)
end

local function reportRendererNotAvailableFeature(request)
  local msg = 'This feature requires a rendering backend to be available. Please ensure you are not running with the \'-gfx null\' argument.'
  log('E', logTag, msg)
  request:sendBNGValueError(msg)
end

local function reportMissingLinuxFeature(request)
  local msg = 'This feature is not yet supported on Linux hosts.'
  log('E', logTag, msg)
  request:sendBNGValueError(msg)
end

local function setup()
  settings.setValue('uiUnits', 'metric')
  settings.setValue('uiUnitLength', 'metric')
  settings.setValue('uiUnitTemperature', 'c')
  settings.setValue('uiUnitWeight', 'kg')
  settings.setValue('uiUnitTorque', 'metric')
  settings.setValue('uiUnitConsumptionRate', 'metric')
  settings.setValue('uiUnitEnergy', 'metric')
  settings.setValue('uiUnitDate', 'iso')
  settings.setValue('uiUnitPower', 'hp')
  settings.setValue('uiUnitVolume', 'l')
  settings.setValue('uiUnitPressure', 'bar')

  extensions.load('tech/partAnnotations')
end

local function getSensorData(request, callback)
  local response, sensor_type, handler
  sensor_type = request['type']
  handler = sensors[sensor_type]
  if handler ~= nil then
    handler(request, callback)
  else
    callback(nil)
  end
end

local function getNextSensorData(requests, response, callback)
  local key = next(requests)
  if key == nil then
    callback(response)
    return
  end

  local request = requests[key]
  requests[key] = nil

  local cb = function(data)
    if data == missingLicenseFeature then
      response = missingLicenseFeature
      callback(response)
      return
    end

    response[key] = data
    getNextSensorData(requests, response, callback)
  end

  getSensorData(request, cb)
end

local function placeObject(name, mesh, pos, rot)
  if name == nil then
    name = 'procObj' .. tostring(objectCount)
    objectCount = objectCount + 1
  end

  pos = vec3(pos)
  rot = quat(rot):toTorqueQuat()

  local proc = createObject('ProceduralMesh')
  if proc == nil then return nil end
  proc:registerObject(name)
  proc.canSave = false
  scenetree.MissionGroup:add(proc.obj)
  proc:createMesh({{mesh}})
  proc:setPosition(pos)
  -- proc:setPosition(pos:toPoint3F())
  proc:setField('rotation', 0, rot.x .. ' ' .. rot.y .. ' ' .. rot.z .. ' ' .. rot.w)
  proc.scale = vec3(1, 1, 1)
  -- proc.scale = Point3F(1, 1, 1)

  be:reloadCollision()

  return proc
end

local function tableToVec3(point, cling, offset)
  local point = vec3(point[1], point[2], point[3])
  if cling then
    local z = techUtils.getSurfaceHeight(point)
    point = vec3(point.x, point.y, z+offset)
  end
  return point
end

-- Sensors

sensors.Timer = function(req, callback)
  local time
  if scenario_scenarios then
    time = scenario_scenarios.getScenario().timer
  else
    local fgMgr = getRunningFlowgraphManager()
    time = Engine.Platform.getRuntime() - fgMgr.startTime
  end
  callback({time = time})
end

-- Exported functions
M.notifyUI = function()
  local state = {}
  if server ~= nil then
    state.running = true
    state.port = tcomParams.port
  else
    state.running = false
  end
  guihooks.trigger('BeamNGpyExtensionReady', state)
end

-- Hooks

M.onAnyMissionChanged = function(state)
  if state == 'started' then
    if isBlocking('loadMission') then
      local waiting = stopBlocking()
      waiting:sendACK('MapLoaded')
    end
  end
end

M.onDrawDebug = function(dtReal, lastFocus)
  for _, sphere in pairs(debugObjects.spheres) do
    debugDrawer:drawSphere(sphere.coo, sphere.radius, sphere.color)
  end
  for _, dSphere in pairs(debugObjects.dynamicSpheres) do
    local spec = dSphere.getSpec()
    debugDrawer:drawSphere(spec.coo, spec.radius, spec.color)
  end
  for _, polyline in pairs(debugObjects.polylines) do
    for _, segment in pairs(polyline.segments) do
      debugDrawer:drawLine(segment.origin, segment.target, polyline.color)
    end
  end
  for _, cylinder in pairs(debugObjects.cylinders) do
    debugDrawer:drawCylinder(cylinder.circleAPos, cylinder.circleBPos, cylinder.radius, cylinder.color)
  end
  for _, triangle in pairs(debugObjects.triangles) do
    if type(triangle.color) == "number" then
      debugDrawer:drawTriSolid(triangle.a, triangle.b, triangle.c, triangle.color)
    else
      debugDrawer:drawTriSolid(triangle.a, triangle.b, triangle.c, color(triangle.color.r,triangle.color.g,triangle.color.b,triangle.color.a))
    end
  end
  for _, rectangle in pairs(debugObjects.rectangles) do
    debugDrawer:drawQuadSolid(rectangle.a, rectangle.b, rectangle.c, rectangle.d, rectangle.color)
  end
  for _, line in pairs(debugObjects.text) do
    debugDrawer:drawText(line.origin, line.content, line.color)
  end
  for _, prism in pairs(debugObjects.squarePrisms) do
    debugDrawer:drawSquarePrism(prism.sideA, prism.sideB, prism.sideADims, prism.sideBDims, prism.color)
  end
end

M.onFilesChanged = function()
  refreshScenarioList()
end

M.onSerialize = function()
  local data = {}
  data.tcomParams = tcomParams
  data.quitRequested = quitRequested
  data.cameras = cameras
  data.lidars = lidars
  data.ultrasonics = ultrasonics
  data.radars = radars
  data.advancedIMUs = advancedIMUs
  data.GPSs = GPSs
  data.powertrains = powertrains
  data.meshes = meshes
  data.idealRADARs = idealRADARs
  data.roadsSensors = roadsSensors
  data.vehicleFeeders = vehicleFeeders

  if server ~= nil then
    local _, serverSocket = next(server)
    _, data.runningPort = serverSocket:getsockname()
  end

  data.showServerGUI = showServerGUI
  return data
end

M.onDeserialized = function(data)
  tcomParams = data.tcomParams
  quitRequested = data.quitRequested
  cameras = data.cameras
  lidars = data.lidars
  ultrasonics = data.ultrasonics
  radars = data.radars
  advancedIMUs = data.advancedIMUs
  GPSs = data.GPSs
  powertrains = data.powertrains
  meshes = data.meshes
  idealRADARs = data.idealRADARs
  roadsSensors = data.roadsSensors
  vehicleFeeders = data.vehicleFeeders

  if data.runningPort ~= nil then
    M.openServer(data.runningPort)
  end
  showServerGUI = data.showServerGUI
end

M.isServerRunning = function()
  return server ~= nil
end

M.openServer = function(port)
  if server ~= nil then
    local _, serverSocket = next(server)
    local runningIP, runningPort = serverSocket:getsockname()
    log('E', logTag, 'Server already running at ' .. runningIP .. '/' .. tostring(runningPort) .. '.')
    return
  end

  if port == nil then
    port = tcomParams.port
  end
  local ip = tcomParams.ip
  local debug = tcomParams.debug

  if server == nil then
    local serverSocket = tcom.openServer(port, ip)
    if serverSocket == nil then
      log('E', logTag, 'Could not create server socket')
      return
    end
    server = tcom.newSet()
    if port == 0 then
      local _, acquiredPort = serverSocket:getsockname()
      tcomParams.port = tonumber(acquiredPort)
    end
    server:insert(serverSocket)
    clients = tcom.newSet()
  end

  if debug == true then
    tcom.enableDebug()
    extensions.load('tech/techCapture')
    tech_techCapture.enableRequestCapture()
  elseif debug == false then
    tcom.disableDebug()
  end
end

M.closeServer = function()
  if server == nil then
    log('E', logTag, 'Cannot close, no server runnning!')
  end

  local clientCount = #clients

  for _, client in pairs(clients) do
    client:close()
  end

  local _, serverSocket = next(server)
  local ip, port = serverSocket:getsockname()
  serverSocket:close()
  log('I', logTag, 'Stopped listening on ' .. ip .. '/' .. tostring(port) .. ' (disconnected ' .. tostring(clientCount) .. ' clients) .')

  clients = nil
  server = nil
end

M.onInit = function()
  setExtensionUnloadMode(M, 'manual')

  local cmdArgs = Engine.getStartingArgs()
  local legacyInit = false -- new versions of beamngpy use custom command to open the server, but we need to keep compatibility
  for i, v in ipairs(cmdArgs) do
    if v == '-rport' then
      -- new version of beamngpy does not send this command-line option
      -- the port is set in the openServer function or in the -tport option
      legacyInit = true
      tcomParams.port = tonumber(cmdArgs[i + 1]) or tcomParams.port
    elseif v == '-tport' then
      legacyInit = false
      tcomParams.port = tonumber(cmdArgs[i + 1]) or tcomParams.port
    elseif v == '-tcom-debug' then
      tcomParams.debug = true
    elseif v == '-no-tcom-debug' then
      tcomParams.debug = false
    elseif v == '-tcom-listen-ip' then
      tcomParams.ip = cmdArgs[i + 1]
    end
  end

  setup()
  if legacyInit then
    M.openServer()
  end
  refreshScenarioList()
end

M.onLoadingScreenFadeout = function()
  if isBlocking('loadScenarioFG') then
    local waiting = stopBlocking()
    waiting:sendACK('MapLoaded')
  elseif isBlocking('restartScenarioFG') then
    guihooks.trigger('ScenarioPlay')
    local waiting = stopBlocking()
    waiting:sendACK('ScenarioRestarted')
  end
end

M.onRequestMissionScreenData = function(mode)
  currentMissionState = mode
end

M.openServerGUI = function()
  showServerGUI = true
end

local function drawOpenServerGUI()
  if M.isServerRunning() then
    ffi.copy(openServerGuiData.ip, tcomParams.ip)
    openServerGuiData.port[0] = tcomParams.port
    openServerGuiData.allInterfaces[0] = false
  end

  im.SetNextWindowSizeConstraints(im.ImVec2(500, 150), im.ImVec2(500, 150))
  im.Begin('Launch BeamNGpy Server', nil, im.WindowFlags_Move)
  if M.isServerRunning() then im.BeginDisabled() end
  if not openServerGuiData.allInterfaces[0] then
    im.InputText("Listen IP address", openServerGuiData.ip)
  end
  im.Checkbox('Listen on all interfaces', openServerGuiData.allInterfaces)
  im.InputInt('Port', openServerGuiData.port)
  if M.isServerRunning() then
    im.EndDisabled()
    if im.Button('Stop server') then
      M.closeServer()
    end
  else
    if im.Button('Start server') then
      if openServerGuiData.allInterfaces[0] then
        tcomParams.ip = '*'
      else
        tcomParams.ip = ffi.string(openServerGuiData.ip)
      end
      tcomParams.port = openServerGuiData.port[0]
      M.openServer(tcomParams.port)
    end
  end

  im.SameLine()
  if im.Button('Exit') then
    showServerGUI = false
  end
  im.End()
end

M.onPreRender = function(dt)
  if showServerGUI then
    drawOpenServerGUI()
  end

  if quitRequested then
    shutdown(0)
  end

  local obsoleteFuncIndices = {}
  if frameDelayFuncQueue then
    for idx, item in pairs(frameDelayFuncQueue) do
      item.frameCountDown = item.frameCountDown-1
      if item.frameCountDown == 0 then
        item.callback()
        table.insert(obsoleteFuncIndices, idx)
      end
    end
  end

  if obsoleteFuncIndices then
    for k, idx in pairs(obsoleteFuncIndices) do
      table.remove(obsoleteFuncIndices, idx)
    end
  end

  if isBlocking() then
    if isBlocking('returnMainMenu') then
      if next(core_gamestate.state) == nil then
        local waiting = stopBlocking()
        waiting:sendACK('ScenarioStopped')
        goto continue
      end
    end

    if isBlocking('returnMainMenuCreateScenario') then
      if next(core_gamestate.state) == nil then
        local waiting, response = stopBlocking()
        waiting:sendResponse(response)
        goto continue
      end
    end

    if isBlocking('step') then
      -- stepsLeft = blocking.data
      blocking.data = blocking.data - 1
      if blocking.data == 0 then
        local waiting = stopBlocking()
        waiting:sendACK('Stepped')
        goto continue
      end
    end
    return
  end

  ::continue::

  if server ~= nil then
    if conSleep <= 0 then
      conSleep = 1
      local newClients = tcom.checkForClients(server)
      for i = 1, #newClients do
        clients:insert(newClients[i])
        local ip, clientPort = newClients[i]:getpeername()
        log('I', logTag, 'Accepted new client: ' .. tostring(ip) .. '/' .. tostring(clientPort))
      end
    else
      conSleep = conSleep - dt
    end
  else
    return
  end

  while tcom.checkMessages(M, clients) do end
end

M.onScenarioLoaded = function()
  if isBlocking('loadScenario') then
    local waiting = stopBlocking()
    waiting:sendACK('MapLoaded')
  end
end

M.onScenarioRestarted = function(scenario)
  if isBlocking('restartScenario') then
    scenario_scenarios.changeState('running')
    scenario.showCountdown = false
    scenario.countDownTime = 0

    guihooks.trigger('ScenarioPlay')

    local restrictActions = blocking.data
    if not restrictActions then -- allow freeroam-like controls of the scenario
      core_input_actionFilter.clear(0)
      core_gamestate.setGameState('exploration', nil, 'freeroam', 'freeroam')
    end

    local waiting = stopBlocking()
    waiting:sendACK('ScenarioRestarted')
  end
end

M.onVehicleConnectionReady = function(vehicleID, port)
  log('I', logTag, 'New vehicle connection: ' .. tostring(vehicleID) .. ', ' .. tostring(port))
  if isBlocking('vehicleConnection') then
    local name = ''
    local veh = scenetree.findObjectById(vehicleID)
    if veh ~= nil then
      name = veh:getName()
    end
    if name == '' then
      name = tostring(vehicleID)
    end
    local resp = {type = 'StartVehicleConnection', vid = name, result = port}
    local waiting = stopBlocking()
    waiting:sendResponse(resp)
  end
end

M.onVehicleInfoReady = function(vehicleID, info)
  if isBlocking('vehicleInfo') then
    local current = blocking.data.vehicleInfo[vehicleID]
    current['port'] = info.port
    blocking.data.vehicleInfoPending[vehicleID] = nil

    checkVehicleInfoPending()
  end
end

M.onVehicleSpawned = function(vID)
  if isBlocking('spawnVehicle') then
    local spawnPending = blocking.data
    if spawnPending ~= nil then
      local obj = scenetree.findObject(spawnPending)
      log('I', logTag, 'Vehicle spawned: ' .. tostring(vID))
      if obj ~= nil and obj:getID() == vID then
        local resp = {type = 'VehicleSpawned', name = spawnPending, success = true}
        local waiting = stopBlocking()
        waiting:sendResponse(resp)
      end
    end
  end
end

-- Handlers

M.handleHello = function(request)
  local resp = {type = 'Hello', protocolVersion = tcom.protocolVersion}
  request:sendResponse(resp)
end

M.handleEcho = function(request)
  local delay = request['delay']
  local func = function ()
    local resp = {type = 'Echo', data = request['data']}
    request:sendResponse(resp)
  end

  if delay == nil or delay == 0 then
    func()
  else
    addFrameDelayFunc(func, delay)
  end
end

M.handleQuit = function(request)
  request:sendACK('Quit')
  quitRequested = true
  block('quit')
end

M.handleLoadScenario = function(request)
  local scenarioPath = request['path']
  if not scenarioPath then
    log('I', logTag, 'Scenario path empty...')
    request:sendBNGValueError(
      'Scenario not found, empty path was provided. ' ..
      'Run scenario.make() before starting the scenario.')
    return false
  end

  if request['precompileShaders'] then
    Engine.Render.setAsyncShaderCompilation(false) -- to avoid problems with the camera sensor
  end
  FS:updateDirectoryWatchers() -- late prefab file notification could cause a bug, update explicitly

  log('I', logTag, 'Loading scenario: ' .. scenarioPath)
  local sc = scenarios[scenarioPath:lower()]
  if not sc then
    -- try to load it anyways, it may not be in one of the default folders
    sc = scenario_scenariosLoader.loadScenario(scenarioPath:lower())
    if not sc then
      log('I', logTag, 'Scenario not found...')
      request:sendBNGValueError('Scenario not found: "' .. scenarioPath .. '"')
      return false
    end
  end

  if sc.levelName then
    local infoPath = '/levels/' .. tostring(sc.levelName) .. '/info.json'
    if not FS:fileExists(infoPath) then
      local msg = 'Level not found: "' .. tostring(sc.levelName) .. '"'
      log('E', logTag, msg)
      request:sendBNGValueError(msg)
      return false
    end
  end

  log('I', logTag, 'Scenario found...')
  if sc.isMission then
    if not block('loadMission', request) then return false end
  elseif sc.flowgraph then
    if not block('loadScenarioFG', request) then return false end
  else
    if not block('loadScenario', request) then return false end
  end
  sc.forceNoCountDown = true
  scenariosLoader.start(sc)

  return false -- keep false here -> do not process any more commands in this frame after loadscenario to avoid bugs
end

M.handleStartScenario = function(request)
  local scenario = scenario_scenarios and scenario_scenarios.getScenario()

  if not scenario and not gameplay_missions_missionManager.getForegroundMissionId() and not getRunningFlowgraphManager() then
    request:sendBNGError('No scenario is running.')
    return false
  end

  if scenario then -- 'normal' scenario loading (not flowgraph)
    scenario_scenarios.changeState('running')
  end

  guihooks.trigger('ScenarioPlay')
  if currentMissionState == 'startScreen' then -- starting a mission
    currentMissionState = 'startedFromTech'
    local startButton = getCurrentMissionMainButton()
    if startButton then
      extensions.hook('onMissionScreenButtonClicked', startButton)
    end
  end

  if not request['restrict_actions'] then -- allow freeroam-like controls of the scenario
    core_input_actionFilter.clear(0)
    core_gamestate.setGameState('exploration', nil, 'freeroam', 'freeroam')
  end
  request:sendACK('ScenarioStarted')
  return true
end

M.handleRestartScenario = function(request)
  local scenario = scenario_scenarios and scenario_scenarios.getScenario()

  if scenario then
    if not block('restartScenario', request, request['restrict_actions']) then return false end
    scenario_scenarios.restartScenario()
  else
    local fgMgr = getRunningFlowgraphManager()
    if not block('restartScenarioFG', request) then return false end
    fgMgr:queueForRestart()
  end
  return true
end

M.handleStopScenario = function(request)
  if not block('returnMainMenu', request) then return false end
  returnToMainMenu()
end

M.handleGetScenarioName = function(request)
  local name
  if scenario_scenarios then
    name = scenario_scenarios.getscenarioName()
  else
    local fgMgr = getRunningFlowgraphManager()
    name = fgMgr.name
  end
  local resp = {type = 'ScenarioName', name = name}
  request:sendResponse(resp)
end

M.handleHideHUD = function(request)
  be:executeJS('document.body.style.opacity = "0.0";')
end

M.handleShowHUD = function(request)
  be:executeJS('document.body.style.opacity = "1.0";')
end

M.handleSetPhysicsDeterministic = function(request)
  be:setPhysicsSpeedFactor(-1)
  request:sendACK('SetPhysicsDeterministic')
end

M.handleSetPhysicsNonDeterministic = function(request)
  be:setPhysicsSpeedFactor(0)
  request:sendACK('SetPhysicsNonDeterministic')
end

M.handleFPSLimit = function(request)
  settings.setValue('fpsLimit', request['fps'])
  settings.setValue('fpsLimitEnabled', true)
  settings.setValue('fpsLimitBackgroundEnabled', false)
  request:sendACK('SetFPSLimit')
end

M.handleRemoveFPSLimit = function(request)
  settings.setValue('fpsLimitEnabled', false)
  request:sendACK('RemovedFPSLimit')
end

M.handlePause = function(request)
  be:setPhysicsRunning(false)
  request:sendACK('Paused')
end

M.handleResume = function(request)
  be:setPhysicsRunning(true)
  request:sendACK('Resumed')
end

M.handleStep = function(request)
  local count = request["count"]
  if request['ack'] then
    if not block('step', request, count) then return false end
  end
  be:physicsStep(count)
  return true
end

M.handleTeleport = function(request)
  local vID = request['vehicle']
  local veh = scenetree.findObject(vID)
  local resp = {type = 'Teleported', success = false}
  if veh == nil then
    request:sendResponse(resp)
    return
  end

  local reset = request['reset'] == nil or request['reset']

  local rot = nil
  if request['rot'] ~= nil then
    rot = quat(request['rot'][1], request['rot'][2], request['rot'][3], request['rot'][4])
  end
  if reset and rot ~= nil then
    veh:setPositionRotation(request['pos'][1], request['pos'][2], request['pos'][3], rot.x, rot.y, rot.z, rot.w)
  elseif reset and rot == nil then
    local vehRot = quat(veh:getClusterRotationSlow(veh:getRefNodeId()))
    veh:setPositionRotation(request['pos'][1], request['pos'][2], request['pos'][3], vehRot.x, vehRot.y, vehRot.z, vehRot.w)
  elseif not reset and rot ~= nil then
    local vehRot = quat(veh:getClusterRotationSlow(veh:getRefNodeId()))
    local diffRot = vehRot:inversed() * rot
    veh:setClusterPosRelRot(veh:getRefNodeId(), request['pos'][1], request['pos'][2], request['pos'][3], diffRot.x, diffRot.y, diffRot.z, diffRot.w)
  else -- not reset and rot == nil
    veh:setClusterPosRelRot(veh:getRefNodeId(), request['pos'][1], request['pos'][2], request['pos'][3], 0, 0, 0, 1)
  end
  resp.success = true
  request:sendResponse(resp)
end

M.handleTeleportScenarioObject = function(request)
  local sobj = scenetree.findObject(request['id'])
  if request['rot'] ~= nil then
    local quat = quat(request['rot'][1], request['rot'][2], request['rot'][3], request['rot'][4])
    sobj:setPosRot(request['pos'][1], request['pos'][2], request['pos'][3], quat.x, quat.y, quat.z, quat.w)
  else
    sobj:setPosition(vec3(request['pos'][1], request['pos'][2], request['pos'][3]))
    -- sobj:setPosition(Point3F(request['pos'][1], request['pos'][2], request['pos'][3]))
  end
  request:sendACK('ScenarioObjectTeleported')
end

M.handleStartVehicleConnection = function(request)
  local vid, veh, command

  vid = request['vid']

  command = 'extensions.load("tech/techCore")'
  veh = scenetree.findObject(vid)
  veh:queueLuaCommand(command)

  local exts = request['exts']
  if exts then
    for idx, ext in pairs(exts) do
      command = 'extensions.load("' .. ext .. '")'
      veh:queueLuaCommand(command)
    end
  end

  if tech_techCapture then
    command = [[
      extensions.load("tech/techCapture")
      tech_techCapture.import(lpack.decode(%q))
    ]]
    veh:queueLuaCommand(string.format(command, lpack.encode(tech_techCapture.export())))
  end

  if not block('vehicleConnection', request) then return false end

  local skipServer = server == nil
  command = string.format('tech_techCore.startConnection(\'%s\', %s)', tcomParams.ip, tostring(skipServer))
  veh:queueLuaCommand(command)
  return true
end

M.handleWaitForSpawn = function(request)
  local name = request['name']
  return block('spawnVehicle', request, name)
end

M.handleSpawnVehicle = function(request)
  local replace = request['replace']

  local alreadyExists = scenetree.findObject(request['name'])
  if alreadyExists and not replace then
    local resp = {type = 'VehicleSpawned', name = request['name'], success = false}
    request:sendResponse(resp)
    return false
  end

  local name = request['name']
  local model = request['model']
  local pos = request['pos']
  local rot = request['rot']
  local cling = request['cling']

  if not core_vehicles.getModel(model).model then
    request:sendBNGError('Model not found: ' .. tostring(model))
    return false
  end

  if not replace then
    pos = vec3(pos[1], pos[2], pos[3])
    rot = quat(rot)
    rot = quat(-rot.y, rot.x, rot.w, -rot.z) -- vehicles' forward is inverted
  end

  local partConfig = request['partConfig']

  local options = {}
  options.config = partConfig
  options.pos = pos
  options.rot = rot
  options.cling = cling
  options.vehicleName = name
  options.color = request['color']
  options.color2 = request['color2']
  options.color3 = request['color3']
  options.licenseText = request['licenseText']

  if not block('spawnVehicle', request, name) then return false end

  local veh = nil
  if replace then
    local replaceVid = request['replace_vid']
    if replaceVid then
      local cur = getPlayerVehicle(0)
      veh = scenetree.findObject(replaceVid)
      if not veh then
        request:sendBNGError('Vehicle \'' .. replaceVid .. '\' to be replaced was not found.')
        stopBlocking()
        return false
      end

      be:enterVehicle(0, veh)
      veh:setField('name', '', options.vehicleName)
      veh = core_vehicles.replaceVehicle(model, options)
      be:enterVehicle(0, cur)
    else
      local cur = getPlayerVehicle(0)
      cur:setField('name', '', options.vehicleName)
      veh = core_vehicles.replaceVehicle(model, options)
    end
  else
    veh = core_vehicles.spawnNewVehicle(model, options)
  end
  if veh ~= nil and options.licenseText ~= nil then
    core_vehicles.setPlateText(options.licenseText, veh:getID()) -- BUG: licenseText not respected in spawnNewVehicle
  end
end

M.handleDespawnVehicle = function(request)
  local name = request['vid']
  local veh = scenetree.findObject(name)
  if veh ~= nil then
    veh:delete()
  end
  request:sendACK('VehicleDespawned')
end

M.handleSensorRequest = function(request)
  local requests

  local cb = function(response)
    if response == missingLicenseFeature then
      reportMissingLicenseFeature(request)
      return false
    end
    response = {type = 'SensorData', data = response}
    request:sendResponse(response)
  end

  requests = request['sensors']

  getNextSensorData(requests, {}, cb)
  return true
end

M.handleGetDecalRoadVertices = function(request)
  local response = Sim.getDecalRoadVertices()
  response = {type = 'DecalRoadVertices', vertices = response}
  request:sendResponse(response)
end

-- TODO: DEPRECATED, USE handleGetRoadNetwork
M.handleGetDecalRoadData = function(request)
  local resp = {type = 'DecalRoadData'}
  local data = {}
  local roads = scenetree.findClassObjects('DecalRoad')
  for idx, roadName in ipairs(roads) do
    local road = scenetree.findObject(roadName)
    local roadID = road:getID()
    local roadData = {}
    for fieldName, _ in pairs(road:getFields()) do
      roadData[fieldName] = road:getField(fieldName, '')
    end
    data[roadID] = roadData
  end
  resp['data'] = data
  request:sendResponse(resp)
end

M.handleGetRoadNetwork = function(request)
  local resp = {type = 'RoadNetwork'}
  local data = {}
  local drivableOnly = request.drivableOnly
  local includeEdges = request.includeEdges

  local roads = scenetree.findClassObjects('DecalRoad')
  for idx, roadName in ipairs(roads) do
    local road = scenetree.findObject(roadName)
    local roadID = road:getID()
    local roadData = {}
    for fieldName, _ in pairs(road:getFields()) do
      roadData[fieldName] = road:getField(fieldName, '')
    end
    if drivableOnly and roadData.drivability == '-1' then
      goto continue
    end
    if includeEdges then
      roadData.edges = {}
      for i, e in ipairs(road:getEdgesTable()) do
        local edge = {
          left = { e[1].x, e[1].y, e[1].z },
          middle = { e[2].x, e[2].y, e[2].z },
          right = { e[3].x, e[3].y, e[3].z }
        }
        table.insert(roadData.edges, edge)
      end
    end
    data[roadID] = roadData
    ::continue::
  end
  resp['data'] = data
  request:sendResponse(resp)
end

M.handleGetDecalRoadEdges = function(request)
  local roadID = request['road']
  local response = {type = 'DecalRoadEdges'}
  local road = scenetree.findObject(roadID)
  local edges = {}
  for i, e in ipairs(road:getEdgesTable()) do
    local edge = {
      left = {
        e[1].x,
        e[1].y,
        e[1].z
      },
      middle = {
        e[2].x,
        e[2].y,
        e[2].z
      },
      right = {
        e[3].x,
        e[3].y,
        e[3].z
      }
    }
    table.insert(edges, edge)
  end
  response['edges'] = edges
  request:sendResponse(response)
end

M.handleGetTimeOfDay = function(request)
  local timeOfDay = core_environment.getTimeOfDay()
  timeOfDay.timeStr = getTimeOfDay(true)
  request:sendResponse({type='TimeOfDay', data=timeOfDay})
end

M.handleTimeOfDayChange = function(request)
  local timeOfDay = core_environment.getTimeOfDay()
  local todIsNumber = type(request['time']) == 'number'

  if todIsNumber then
    timeOfDay['time'] = request['time']
  end
  if request['play'] ~= nil then
    timeOfDay['play'] = request['play']
  end
  if request['dayScale'] ~= nil then
    timeOfDay['dayScale'] = request['dayScale']
  end
  if request['nightScale'] ~= nil then
    timeOfDay['nightScale'] = request['nightScale']
  end
  if request['dayLength'] ~= nil then
    timeOfDay['dayLength'] = request['dayLength']
  end
  if request['azimuthOverride'] ~= nil then
    timeOfDay['azimuthOverride'] = request['azimuthOverride']
  end

  core_environment.setTimeOfDay(timeOfDay)
  if not todIsNumber and request['time'] ~= nil then
    setTimeOfDay(request['time'])
  end

  request:sendACK('TimeOfDayChanged')
end

M.handleGetAdvancedImuId = function(request)
  local sensorId = advancedIMUs[request['name']]
  local resp = {type = 'getAdvancedImuId', data = sensorId}
  request:sendResponse(resp)
end

M.handleGetGPSId = function(request)
  local sensorId = GPSs[request['name']]
  local resp = {type = 'getGPSId', data = sensorId}
  request:sendResponse(resp)
end

M.handleGetPowertrainId = function(request)
  local sensorId = powertrains[request['name']]
  local resp = {type = 'getPowertrainId', data = sensorId}
  request:sendResponse(resp)
end

M.handleGetMeshId = function(request)
  local sensorId = meshes[request['name']]
  local resp = {type = 'getMeshId', data = sensorId}
  request:sendResponse(resp)
end

M.handleGetIdealRADARId = function(request)
  local sensorId = idealRADARs[request['name']]
  local resp = {type = 'getIdealRADARId', data = sensorId}
  request:sendResponse(resp)
end

M.handleGetRoadsSensorId = function(request)
  local sensorId = roadsSensors[request['name']]
  local resp = {type = 'getRoadsSensorId', data = sensorId}
  request:sendResponse(resp)
end

M.handleOpenCamera = function(request)
  if not ResearchVerifier.isTechLicenseVerified() then
    reportMissingLicenseFeature(request)
    return false
  end
  if not Engine.Render.isAvailable() then
    reportRendererNotAvailableFeature(request)
    return false
  end

  local args = {}
  args.requestedUpdateTime = request['updateTime']
  args.updatePriority = request['priority']
  args.size = request['size']
  args.fovY = request['fovY']
  args.nearFarPlanes = request['nearFarPlanes']
  args.pos = vec3(request['pos'][1], request['pos'][2], request['pos'][3])
  args.dir = vec3(request['dir'][1], request['dir'][2], request['dir'][3])
  args.up = vec3(request['up'][1], request['up'][2], request['up'][3])
  args.colourShmemHandle = request['colourShmemName']
  args.colourShmemSize = request['colourShmemSize']
  args.annotationShmemHandle = request['annotationShmemName']
  args.annotationShmemSize = request['annotationShmemSize']
  args.depthShmemHandle= request['depthShmemName']
  args.depthShmemSize = request['depthShmemSize']
  args.renderColours = request['renderColours']
  args.renderAnnotations = request['renderAnnotations']
  args.renderDepth = request['renderDepth']
  args.renderInstance = request['renderInstance']
  args.isVisualised = request['isVisualised']
  args.isStreaming = request['isStreaming']
  args.isStatic = request['isStatic']
  args.isSnappingDesired = request['isSnappingDesired']
  args.isForceInsideTriangle = request['isForceInsideTriangle']
  args.isDirWorldSpace = request['isDirWorldSpace']
  args.integerDepth = request['integerDepth']

  -- If annotations are required, we need to enable this in the engine.
  if request['renderAnnotations'] == true then
    Engine.Annotation.enable(true)
    log('I', logTag, 'Camera sensor - annotaton rendering enabled')
  end

  local name = request['name']
  local vid = 0
  if request['vid'] ~= 0 then
    vid = scenetree.findObject(request['vid']):getID();
  end
  if request['useSharedMemory'] == true then
    cameras[name] = extensions.tech_sensors.createCameraWithSharedMemory(vid, args)
    log('I', logTag, 'Opened camera sensor (with shared memory)')
  else
    cameras[name] = extensions.tech_sensors.createCamera(vid, args)
    log('I', logTag, 'Opened camera sensor (without shared memory)')
  end

  request:sendACK('OpenedCamera')
end

M.handleCloseCamera = function(request)
  local name = request['name']
  local sensorId = cameras[name]
  if sensorId ~= nil then
    extensions.tech_sensors.removeSensor(sensorId)
    cameras[name] = nil
    log('I', logTag, 'Closed camera sensor')
  end

  request:sendACK('ClosedCamera')
end

M.handlePollCamera = function(request)
  local name = request['name']
  local isUsingSharedMemory = request['isUsingSharedMemory']

  local sensorId = cameras[name]
  if sensorId ~= nil then
    if isUsingSharedMemory then
      -- Shared memory is being used, so the memory sizes are the response.
      local cameraSizes = Research.Camera.getLastCameraDataShmem(sensorId)
      local resp = {type = 'PollCamera', data = cameraSizes}
      request:sendResponse(resp)
    else
      -- Shared memory is not being used, so the data is the response.
      local cameraData = extensions.tech_sensors.getCameraData(sensorId)
      local resp = {type = 'PollCamera', data = {
        colour = cameraData.colour,
        annotation = cameraData.annotation,
        depth = cameraData.depth } }
      request:sendResponse(resp)
    end
  else
    -- The sensor was not found, so send an empty response.
    local resp = {type = 'PollCamera', data = nil}
    log('I', logTag, 'WARNING: Camera sensor not found')
    request:sendResponse(resp)
  end
end

M.handleSendAdHocRequestCamera = function(request)
  local requestId = extensions.tech_sensors.sendCameraRequest(cameras[request['name']])
  local resp = {type = 'requestId', data = requestId}
  request:sendResponse(resp)
end

M.handleIsAdHocPollRequestReadyCamera = function(request)
  local isRequestComplete = extensions.tech_sensors.isRequestComplete(request['requestId'])
  local resp = {type = 'isRequestComplete', data = isRequestComplete}
  request:sendResponse(resp)
end

M.handleCollectAdHocPollRequestCamera = function(request)
  local cameraData = extensions.tech_sensors.collectCameraRequest(request['requestId'])
  local resp = {type = 'AdHocPollRequest', data = cameraData}
  request:sendResponse(resp)
end

-- TODO Should be replaced when GE-2170 is complete.
M.handleGetFullCameraRequest = function(request)
  local camera = cameras[request['name']]
  if camera == nil then
    -- The sensor was not found, so send an empty response.
    local resp = {type = 'FullCameraRequest', data = nil}
    log('I', logTag, 'WARNING: Camera sensor not found')
    request:sendResponse(resp)
    return true
  end

  local cameraData = extensions.tech_sensors.getFullCameraRequest(camera)
  local data = {}
  data['colour'] = cameraData['colour']
  data['annotation'] = cameraData['annotation']
  data['instance'] = cameraData['instance']
  data['depth'] = cameraData['depth']
  local resp = {type = 'FullCameraRequest', data = data}
  request:sendResponse(resp)
end

M.handleCameraWorldPointToPixel = function(request)
  local point = vec3(request['pointX'], request['pointY'], request['pointZ'])
  local pixel = extensions.tech_sensors.convertWorldPointToPixel(cameras[request['name']], point)
  local resp = {type = 'CameraWorldPointToPixel', data = { x = pixel.x, y = pixel.y }}
  request:sendResponse(resp)
end

M.handleGetCameraSensorPosition = function(request)
  local pos = extensions.tech_sensors.getCameraSensorPosition(cameras[request['name']])
  local resp = {type = 'GetCameraSensorPosition', data = { x = pos.x, y = pos.y, z = pos.z}}
  request:sendResponse(resp)
end

M.handleGetCameraSensorDirection = function(request)
  local dir = extensions.tech_sensors.getCameraSensorDirection(cameras[request['name']])
  local resp = {type = 'dir', data = { x = dir.x, y = dir.y, z = dir.z}}
  request:sendResponse(resp)
end

M.handleGetCameraSensorUp = function(request)
  local up = extensions.tech_sensors.getCameraSensorUp(cameras[request['name']])
  local resp = {type = 'up', data = { x = up.x, y = up.y, z = up.z}}
  request:sendResponse(resp)
end

M.handleGetCameraMaxPendingGpuRequests = function(request)
  local maxRequests = extensions.tech_sensors.getCameraMaxPendingGpuRequests(cameras[request['name']])
  local resp = {type = 'maxPendingGpuRequests', data = maxRequests}
  request:sendResponse(resp)
end

M.handleGetCameraRequestedUpdateTime = function(request)
  local updateTime = extensions.tech_sensors.getCameraRequestedUpdateTime(cameras[request['name']])
  local resp = {type = 'updateTime', data = updateTime}
  request:sendResponse(resp)
end

M.handleGetCameraUpdatePriority = function(request)
  local priority = extensions.tech_sensors.getCameraUpdatePriority(cameras[request['name']])
  local resp = {type = 'updatePriority', data = priority}
  request:sendResponse(resp)
end

M.handleSetCameraSensorPosition = function(request)
  extensions.tech_sensors.setCameraSensorPosition(cameras[request['name']], vec3(request['posX'], request['posY'], request['posZ']))
  request:sendACK('CompletedSetCameraSensorPosition')
end

M.handleSetCameraSensorDirection = function(request)
  extensions.tech_sensors.setCameraSensorDirection(cameras[request['name']], vec3(request['dirX'], request['dirY'], request['dirZ']))
  request:sendACK('CompletedSetCameraSensorDirection')
end

M.handleSetCameraMaxPendingGpuRequests = function(request)
  extensions.tech_sensors.setCameraMaxPendingGpuRequests(cameras[request['name']], request['maxPendingGpuRequests'])
  request:sendACK('CompletedSetCameraMaxPendingGpuRequests')
end

M.handleSetCameraRequestedUpdateTime = function(request)
  extensions.tech_sensors.setCameraRequestedUpdateTime(cameras[request['name']], request['updateTime'])
  request:sendACK('CompletedSetCameraRequestedUpdateTime')
end

M.handleSetCameraUpdatePriority = function(request)
  extensions.tech_sensors.setCameraUpdatePriority(cameras[request['name']], request['updatePriority'])
  request:sendACK('CompletedSetCameraUpdatePriority')
end

M.handleOpenLidar = function(request)
  if not ResearchVerifier.isTechLicenseVerified() then
    reportMissingLicenseFeature(request)
    return false
  end
  if not Engine.Render.isAvailable() then
    reportRendererNotAvailableFeature(request)
    return false
  end

  local args = {}
  args.pointCloudShmemName = request['pointCloudShmemHandle']
  args.pointCloudShmemSize = request['pointCloudShmemSize']
  args.colourShmemName = request['colourShmemHandle']
  args.colourShmemSize = request['colourShmemSize']
  args.requestedUpdateTime = request['updateTime']
  args.updatePriority = request['priority']
  args.pos = vec3(request['pos'][1], request['pos'][2], request['pos'][3])
  args.dir = vec3(request['dir'][1], request['dir'][2], request['dir'][3])
  args.up = vec3(request['up'][1], request['up'][2], request['up'][3])
  args.verticalResolution = request['vRes']
  args.verticalAngle = request['vAngle']
  args.frequency = request['hz']
  args.horizontalAngle = request['hAngle']
  args.maxDistance = request['maxDist']
  args.density = request['density']
  args.isRotate = request['isRotate']
  args.is360 = request['is360']
  args.isVisualised = request['isVisualised']
  args.isStreaming = request['isStreaming']
  args.isAnnotated = request['isAnnotated']
  args.isStatic = request['isStatic']
  args.isSnappingDesired = request['isSnappingDesired']
  args.isForceInsideTriangle = request['isForceInsideTriangle']
  args.isDirWorldSpace = request['isDirWorldSpace']

  local name = request['name']
  local vid = 0
  if request['vid'] ~= 0 then
    vid = scenetree.findObject(request['vid']):getID();
  end
  if request['useSharedMemory'] == true then
    lidars[name] = extensions.tech_sensors.createLidarWithSharedMemory(vid, args)
    log('I', logTag, 'Opened LiDAR sensor (with shared memory)')
  else
    lidars[name] = extensions.tech_sensors.createLidar(vid, args)
    log('I', logTag, 'Opened LiDAR sensor (without shared memory)')
  end
  request:sendACK('OpenedLidar')
end

M.handleCloseLidar = function(request)
  local name = request['name']
  local sensorId = lidars[name]
  if sensorId ~= nil then
    extensions.tech_sensors.removeSensor(sensorId)
    lidars[name] = nil
    log('I', logTag, 'Closed LiDAR sensor')
  end
  request:sendACK('ClosedLidar')
end

M.handlePollLidar = function(request)
  local name = request['name']
  local isUsingSharedMemory = request['isUsingSharedMemory']
  local sensorId = lidars[name]
  if sensorId ~= nil then
    if isUsingSharedMemory then
      -- Shared memory is being used, so the memory sizes goes in the response.
      local pointCloudSize = Research.Lidar.getLastPointCloudDataShmem(sensorId)
      local colourSize = Research.Lidar.getLastColourDataShmem(sensorId)
      local resp = {type = 'PollLidar', data = { points = pointCloudSize, colours = colourSize }}
      request:sendResponse(resp)
    else
      -- Shared memory is not being used, so the point cloud and colour data goes in the response.
      local pointCloud = extensions.tech_sensors.getLidarPointCloud(sensorId)           -- get the LiDAR point cloud data.
      local colours = extensions.tech_sensors.getLidarColourData(sensorId)              -- get the LiDAR colour data.
      local resp = {type = 'PollLidar', data = { pointCloud = pointCloud, colours = colours } }
      request:sendResponse(resp)
    end
  else
    -- The sensor was not found, so send an empty response.
    local resp = {type = 'PollLidar', data = nil}
    log('I', logTag, 'WARNING: LiDAR sensor not found')
    request:sendResponse(resp)
  end
end

M.handleSendAdHocRequestLidar = function(request)
  local requestId = extensions.tech_sensors.sendLidarRequest(lidars[request['name']])
  local resp = {type = 'requestId', data = requestId}
  request:sendResponse(resp)
end

M.handleIsAdHocPollRequestReadyLidar = function(request)
  local isRequestComplete = extensions.tech_sensors.isRequestComplete(request['requestId'])
  local resp = {type = 'isRequestComplete', data = isRequestComplete}
  request:sendResponse(resp)
end

M.handleCollectAdHocPollRequestLidar = function(request)
  local data = extensions.tech_sensors.collectLidarRequest(request['requestId'])
  local resp = {type = 'AdHocPollRequestData', data = data }
  request:sendResponse(resp)
end

M.handleGetLidarSensorPosition = function(request)
  local pos = extensions.tech_sensors.getLidarSensorPosition(lidars[request['name']])
  local resp = {type = 'pos', data = { x = pos.x, y = pos.y, z = pos.z}}
  request:sendResponse(resp)
end

M.handleGetLidarSensorDirection = function(request)
  local dir = extensions.tech_sensors.getLidarSensorDirection(lidars[request['name']])
  local resp = {type = 'dir', data = { x = dir.x, y = dir.y, z = dir.z}}
  request:sendResponse(resp)
end

M.handleGetLidarMaxPendingGpuRequests = function(request)
  local maxRequests = extensions.tech_sensors.getLidarMaxPendingGpuRequests(lidars[request['name']])
  local resp = {type = 'maxPendingGpuRequests', data = maxRequests}
  request:sendResponse(resp)
end

M.handleGetLidarRequestedUpdateTime = function(request)
  local updateTime = extensions.tech_sensors.getLidarRequestedUpdateTime(lidars[request['name']])
  local resp = {type = 'updateTime', data = updateTime}
  request:sendResponse(resp)
end

M.handleGetLidarUpdatePriority = function(request)
  local priority = extensions.tech_sensors.getLidarUpdatePriority(lidars[request['name']])
  local resp = {type = 'updatePriority', data = priority}
  request:sendResponse(resp)
end

M.handleGetLidarVerticalResolution = function(request)
  local vRes = extensions.tech_sensors.getLidarVerticalResolution(lidars[request['name']])
  local resp = {type = 'verticalResolution', data = vRes}
  request:sendResponse(resp)
end

M.handleGetLidarFrequency = function(request)
  local freq = extensions.tech_sensors.getLidarFrequency(lidars[request['name']])
  local resp = {type = 'frequency', data = freq}
  request:sendResponse(resp)
end

M.handleGetLidarMaxDistance = function(request)
  local maxDist = extensions.tech_sensors.getLidarMaxDistance(lidars[request['name']])
  local resp = {type = 'maxDistance', data = maxDist}
  request:sendResponse(resp)
end

M.handleGetLidarIsVisualised = function(request)
  local isVisualised = extensions.tech_sensors.getLidarIsVisualised(lidars[request['name']])
  local resp = {type = 'isVisualised', data = isVisualised}
  request:sendResponse(resp)
end

M.handleGetLidarIsAnnotated = function(request)
  local isAnnotated = extensions.tech_sensors.getLidarIsAnnotated(lidars[request['name']])
  local resp = {type = 'isAnnotated', data = isAnnotated}
  request:sendResponse(resp)
end

M.handleSetLidarVerticalResolution = function(request)
  extensions.tech_sensors.setLidarVerticalResolution(lidars[request['name']], request['verticalResolution'])
  request:sendACK('CompletedSetLidarVerticalResolution')
end

M.handleSetLidarFrequency = function(request)
  extensions.tech_sensors.setLidarFrequency(lidars[request['name']], request['frequency'])
  request:sendACK('CompletedSetLidarFrequency')
end

M.handleSetLidarMaxDistance = function(request)
  extensions.tech_sensors.setLidarMaxDistance(lidars[request['name']], request['maxDistance'])
  request:sendACK('CompletedSetLidarMaxDistance')
end

M.handleSetLidarIsVisualised = function(request)
  extensions.tech_sensors.setLidarIsVisualised(lidars[request['name']], request['isVisualised'])
  request:sendACK('CompletedSetLidarIsVisualised')
end

M.handleSetLidarIsAnnotated = function(request)
  extensions.tech_sensors.setLidarIsAnnotated(lidars[request['name']], request['isAnnotated'])
  request:sendACK('CompletedSetLidarIsAnnotated')
end

M.handleSetLidarMaxPendingGpuRequests = function(request)
  extensions.tech_sensors.setLidarMaxPendingGpuRequests(lidars[request['name']], request['maxPendingGpuRequests'])
  request:sendACK('CompletedSetLidarMaxPendingGpuRequests')
end

M.handleSetLidarRequestedUpdateTime = function(request)
  extensions.tech_sensors.setLidarRequestedUpdateTime(lidars[request['name']], request['updateTime'])
  request:sendACK('CompletedSetLidarRequestedUpdateTime')
end

M.handleSetLidarUpdatePriority = function(request)
  extensions.tech_sensors.setLidarUpdatePriority(lidars[request['name']], request['updatePriority'])
  request:sendACK('CompletedSetLidarUpdatePriority')
end

M.handleOpenUltrasonic = function(request)
  if not ResearchVerifier.isTechLicenseVerified() then
    reportMissingLicenseFeature(request)
    return false
  end
  if not Engine.Render.isAvailable() then
    reportRendererNotAvailableFeature(request)
    return false
  end

  local args = {}
  args.shmemName = request['shmemHandle']
  args.shmemSize = request['shmemSize']
  args.requestedUpdateTime = request['updateTime']
  args.updatePriority = request['priority']
  args.size = request['size']
  args.fovY = request['fovY']
  args.nearFarPlanes = request['near_far_planes']
  args.rangeRoundness = request['range_roundness']
  args.rangeCutoffSensitivity = request['range_cutoff_sensitivity']
  args.rangeShape = request['range_shape']
  args.rangeFocus = request['range_focus']
  args.rangeMinCutoff = request['range_min_cutoff']
  args.rangeDirectMaxCutoff = request['range_direct_max_cutoff']
  args.sensitivity = request['sensitivity']
  args.fixedWindowSize = request['fixed_window_size']
  args.pos = vec3(request['pos'][1], request['pos'][2], request['pos'][3])
  args.dir = vec3(request['dir'][1], request['dir'][2], request['dir'][3])
  args.up = vec3(request['up'][1], request['up'][2], request['up'][3])
  args.isVisualised = request['isVisualised']
  args.isStreaming = request['isStreaming']
  args.isStatic = request['isStatic']
  args.isSnappingDesired = request['isSnappingDesired']
  args.isForceInsideTriangle = request['isForceInsideTriangle']
  args.isDirWorldSpace = request['isDirWorldSpace']

  local name = request['name']
  local vid = 0
  if request['vid'] ~= 0 then
    vid = scenetree.findObject(request['vid']):getID();
  end
  ultrasonics[name] = extensions.tech_sensors.createUltrasonic(vid, args)
  log('I', logTag, 'Opened ultrasonic sensor')
  request:sendACK('OpenedUltrasonic')
end

M.handleCloseUltrasonic = function(request)
  local name = request['name']
  local sensorId = ultrasonics[name]
  if sensorId ~= nil then
    extensions.tech_sensors.removeSensor(sensorId)
    ultrasonics[name] = nil
    log('I', logTag, 'Closed ultrasonic sensor')
  end

  request:sendACK('ClosedUltrasonic')
end

M.handlePollUltrasonic = function(request)
  local name = request['name']
  local sensorId = ultrasonics[name]
  if sensorId ~= nil then
    local readings = Research.Ultrasonic.getLastReadings(sensorId)
    local resp = {type = 'PollUltrasonic', data = readings}
    request:sendResponse(resp)
  else
    -- The sensor was not found, so send an empty response.
    local resp = {type = 'PollUltrasonic', data = nil}
    log('I', logTag, 'WARNING: Ultrasonic sensor not found')
    request:sendResponse(resp)
  end
end

M.handleSendAdHocRequestUltrasonic = function(request)
  local requestId = extensions.tech_sensors.sendUltrasonicRequest(ultrasonics[request['name']])
  local resp = {type = 'requestId', data = requestId}
  request:sendResponse(resp)
end

M.handleIsAdHocPollRequestReadyUltrasonic = function(request)
  local isRequestComplete = extensions.tech_sensors.isRequestComplete(request['requestId'])
  local resp = {type = 'isRequestComplete', data = isRequestComplete}
  request:sendResponse(resp)
end

M.handleCollectAdHocPollRequestUltrasonic = function(request)
  local ultrasonicData = extensions.tech_sensors.collectUltrasonicRequest(request['requestId'])
  local resp = {type = 'AdHocPollRequestData', data = ultrasonicData}
  request:sendResponse(resp)
end

M.handleGetUltrasonicSensorPosition = function(request)
  local pos = extensions.tech_sensors.getUltrasonicSensorPosition(ultrasonics[request['name']])
  local resp = {type = 'pos', data = { x = pos.x, y = pos.y, z = pos.z}}
  request:sendResponse(resp)
end

M.handleGetUltrasonicSensorDirection = function(request)
  local dir = extensions.tech_sensors.getUltrasonicSensorDirection(ultrasonics[request['name']])
  local resp = {type = 'dir', data = { x = dir.x, y = dir.y, z = dir.z}}
  request:sendResponse(resp)
end

M.handleGetUltrasonicMaxPendingGpuRequests = function(request)
  local maxRequests = extensions.tech_sensors.getUltrasonicMaxPendingGpuRequests(ultrasonics[request['name']])
  local resp = {type = 'maxPendingGpuRequests', data = maxRequests}
  request:sendResponse(resp)
end

M.handleGetUltrasonicRequestedUpdateTime = function(request)
  local updateTime = extensions.tech_sensors.getUltrasonicRequestedUpdateTime(ultrasonics[request['name']])
  local resp = {type = 'updateTime', data = updateTime}
  request:sendResponse(resp)
end

M.handleGetUltrasonicUpdatePriority = function(request)
  local priority = extensions.tech_sensors.getUltrasonicUpdatePriority(ultrasonics[request['name']])
  local resp = {type = 'updatePriority', data = priority}
  request:sendResponse(resp)
end

M.handleGetUltrasonicIsVisualised = function(request)
  local isVisualised = extensions.tech_sensors.getUltrasonicIsVisualised(ultrasonics[request['name']])
  local resp = {type = 'isVisualised', data = isVisualised}
  request:sendResponse(resp)
end

M.handleSetUltrasonicMaxPendingGpuRequests = function(request)
  extensions.tech_sensors.setUltrasonicMaxPendingGpuRequests(ultrasonics[request['name']], request['maxPendingGpuRequests'])
  request:sendACK('CompletedSetUltrasonicMaxPendingGpuRequests')
end

M.handleSetUltrasonicRequestedUpdateTime = function(request)
  extensions.tech_sensors.setUltrasonicRequestedUpdateTime(ultrasonics[request['name']], request['updateTime'])
  request:sendACK('CompletedSetUltrasonicRequestedUpdateTime')
end

M.handleSetUltrasonicUpdatePriority = function(request)
  extensions.tech_sensors.setUltrasonicUpdatePriority(ultrasonics[request['name']], request['updatePriority'])
  request:sendACK('CompletedSetUltrasonicUpdatePriority')
end

M.handleSetUltrasonicIsVisualised = function(request)
  extensions.tech_sensors.setUltrasonicIsVisualised(ultrasonics[request['name']], request['isVisualised'])
  request:sendACK('CompletedSetUltrasonicIsVisualised')
end

M.handleOpenRadar = function(request)
  if not ResearchVerifier.isTechLicenseVerified() then
    reportMissingLicenseFeature(request)
    return false
  end
  if not Engine.Render.isAvailable() then
    reportRendererNotAvailableFeature(request)
    return false
  end

  local args = {}
  args.shmemName = request['shmemHandle']
  args.shmemName2 = request['shmemHandle2']
  args.shmemSize = request['shmemSize']
  args.requestedUpdateTime = request['updateTime']
  args.updatePriority = request['priority']
  args.size = request['size']
  args.fovY = request['fovY']
  args.nearFarPlanes = request['near_far_planes']
  args.rangeRoundness = request['range_roundness']
  args.rangeCutoffSensitivity = request['range_cutoff_sensitivity']
  args.rangeShape = request['range_shape']
  args.rangeFocus = request['range_focus']
  args.rangeMinCutoff = request['range_min_cutoff']
  args.rangeDirectMaxCutoff = request['range_direct_max_cutoff']
  args.pos = vec3(request['pos'][1], request['pos'][2], request['pos'][3])
  args.dir = vec3(request['dir'][1], request['dir'][2], request['dir'][3])
  args.up = vec3(request['up'][1], request['up'][2], request['up'][3])
  args.rangeBins = request['range_bins']
  args.azimuthBins = request['azimuth_bins']
  args.velBins = request['vel_bins']
  args.rangeMin = request['range_min']
  args.rangeMax = request['range_max']
  args.velMin = request['vel_min']
  args.velMax = request['vel_max']
  args.halfAngleDeg = request['half_angle_deg']
  args.isVisualised = request['isVisualised']
  args.isStreaming = request['isStreaming']
  args.isStatic = request['isStatic']
  args.isSnappingDesired = request['isSnappingDesired']
  args.isForceInsideTriangle = request['isForceInsideTriangle']
  args.isDirWorldSpace = request['isDirWorldSpace']

  local name = request['name']
  local vid = 0
  if request['vid'] ~= 0 then
    vid = scenetree.findObject(request['vid']):getID();
  end
  radars[name] = extensions.tech_sensors.createRadar(vid, args)
  log('I', logTag, 'Opened Radar sensor')
  request:sendACK('OpenedRadar')
end

M.handleCloseRadar = function(request)
  local name = request['name']
  local sensorId = radars[name]
  if sensorId ~= nil then
    extensions.tech_sensors.removeSensor(sensorId)
    radars[name] = nil
    log('I', logTag, 'Closed Radar sensor')
  end

  request:sendACK('ClosedRadar')
end

M.handlePollRadar = function(request)
  local name = request['name']
  local sensorId = radars[name]
  if sensorId ~= nil then
    local readings = extensions.tech_sensors.getRadarReadings(sensorId)
    local resp = {type = 'PollRadar', data = readings}
    request:sendResponse(resp)
  else
    -- The sensor was not found, so send an empty response.
    local resp = {type = 'PollRadar', data = nil}
    log('I', logTag, 'WARNING: Radar sensor not found')
    request:sendResponse(resp)
  end
end

M.handleGetPPIRadar = function(request)
  local name = request['name']
  local sensorId = radars[name]
  if sensorId ~= nil then
    local dataSize = extensions.tech_sensors.getRadarPPIData(sensorId)
    local resp = {type = 'GetPPIRadar', data = dataSize}
    request:sendResponse(resp)
  else
    -- The sensor was not found, so send an empty response.
    local resp = {type = 'GetPPIRadar', data = nil}
    log('I', logTag, 'WARNING: Radar sensor not found')
    request:sendResponse(resp)
  end
end

M.handleGetRangeDopplerRadar = function(request)
  local name = request['name']
  local sensorId = radars[name]
  if sensorId ~= nil then
    local dataSize = extensions.tech_sensors.getRadarRangeDopplerData(sensorId)
    local resp = {type = 'GetRangeDopplerRadar', data = dataSize}
    request:sendResponse(resp)
  else
    -- The sensor was not found, so send an empty response.
    local resp = {type = 'GetRangeDopplerRadar', data = nil}
    log('I', logTag, 'WARNING: Radar sensor not found')
    request:sendResponse(resp)
  end
end

M.handleSendAdHocRequestRadar = function(request)
  local requestId = extensions.tech_sensors.sendRadarRequest(radars[request['name']])
  local resp = {type = 'requestId', data = requestId}
  request:sendResponse(resp)
end

M.handleIsAdHocPollRequestReadyRadar = function(request)
  local isRequestComplete = extensions.tech_sensors.isRequestComplete(request['requestId'])
  local resp = {type = 'isRequestComplete', data = isRequestComplete}
  request:sendResponse(resp)
end

M.handleCollectAdHocPollRequestRadar = function(request)
  local radarData = extensions.tech_sensors.collectRadarRequest(request['requestId'])
  local resp = {type = 'AdHocPollRequestData', data = radarData}
  request:sendResponse(resp)
end

M.handleGetRadarSensorPosition = function(request)
  local pos = extensions.tech_sensors.getRadarSensorPosition(radars[request['name']])
  local resp = {type = 'pos', data = { x = pos.x, y = pos.y, z = pos.z}}
  request:sendResponse(resp)
end

M.handleGetRadarSensorDirection = function(request)
  local dir = extensions.tech_sensors.getRadarSensorDirection(radars[request['name']])
  local resp = {type = 'dir', data = { x = dir.x, y = dir.y, z = dir.z}}
  request:sendResponse(resp)
end

M.handleGetRadarMaxPendingGpuRequests = function(request)
  local maxRequests = extensions.tech_sensors.getRadarMaxPendingGpuRequests(radars[request['name']])
  local resp = {type = 'maxPendingGpuRequests', data = maxRequests}
  request:sendResponse(resp)
end

M.handleGetRadarRequestedUpdateTime = function(request)
  local updateTime = extensions.tech_sensors.getRadarRequestedUpdateTime(radars[request['name']])
  local resp = {type = 'updateTime', data = updateTime}
  request:sendResponse(resp)
end

M.handleGetRadarUpdatePriority = function(request)
  local priority = extensions.tech_sensors.getRadarUpdatePriority(radars[request['name']])
  local resp = {type = 'updatePriority', data = priority}
  request:sendResponse(resp)
end

M.handleSetRadarMaxPendingGpuRequests = function(request)
  extensions.tech_sensors.setRadarMaxPendingGpuRequests(radars[request['name']], request['maxPendingGpuRequests'])
  request:sendACK('CompletedSetRadarMaxPendingGpuRequests')
end

M.handleSetRadarRequestedUpdateTime = function(request)
  extensions.tech_sensors.setRadarRequestedUpdateTime(radars[request['name']], request['updateTime'])
  request:sendACK('CompletedSetRadarRequestedUpdateTime')
end

M.handleSetRadarUpdatePriority = function(request)
  extensions.tech_sensors.setRadarUpdatePriority(radars[request['name']], request['updatePriority'])
  request:sendACK('CompletedSetRadarUpdatePriority')
end

M.handleOpenAdvancedIMU = function(request)
  if not ResearchVerifier.isTechLicenseVerified() then
    reportMissingLicenseFeature(request)
    return false
  end

  local args = {}
  args.GFXUpdateTime = request['GFXUpdateTime']
  args.physicsUpdateTime = request['physicsUpdateTime']
  args.pos = vec3(request['pos'][1], request['pos'][2], request['pos'][3])
  args.dir = vec3(request['dir'][1], request['dir'][2], request['dir'][3])
  args.up = vec3(request['up'][1], request['up'][2], request['up'][3])
  args.accelWindowWidth = request['accelWindowWidth']
  args.accelFrequencyCutoff = request['accelFrequencyCutoff']
  args.gyroWindowWidth = request['gyroWindowWidth']
  args.gyroFrequencyCutoff = request['gyroFrequencyCutoff']
  args.isSendImmediately = request['isSendImmediately']
  args.isVisualised = request['isVisualised']
  args.isUsingGravity= request['isUsingGravity']
  args.isSnappingDesired = request['isSnappingDesired']
  args.isForceInsideTriangle = request['isForceInsideTriangle']
  args.isDirWorldSpace = request['isDirWorldSpace']

  local name = request['name']
  local vid = scenetree.findObject(request['vid']):getID();

  advancedIMUs[name] = extensions.tech_sensors.createAdvancedIMU(vid, args)
  log('I', logTag, 'Opened AdvancedIMU sensor')

  request:sendACK('OpenedAdvancedIMU')
end

M.handleCloseAdvancedIMU = function(request)
  local name = request['name']
  local vid = request['vid']
  local sensorId = advancedIMUs[name]
  if sensorId ~= nil then
    advancedIMUs[name] = nil                                    -- remove from ge lua
    extensions.tech_sensors.removeAdvancedIMU(vid, sensorId)    -- remove from vlua.
    log('I', logTag, 'Closed Advanced IMU sensor')
  end

  request:sendACK('ClosedAdvancedIMU')
end

M.handlePollAdvancedImuGE = function(request)
  local name = request['name']
  local sensorId = advancedIMUs[name]
  if sensorId ~= nil then
    local readings = extensions.tech_sensors.getAdvancedIMUReadings(sensorId)
    if readings ~= nil then
      local resp = { type = 'PollAdvancedImuGE', data = readings }
      request:sendResponse(resp)
      return true
    end
  end

  -- The sensor was not found, or the readings did not exist, so send an empty response.
  local resp = {type = 'PollAdvancedImuGE', data = {} }
  log('I', logTag, 'WARNING: Advanced IMU sensor not found')
  request:sendResponse(resp)
end

M.handleSendAdHocRequestAdvancedIMU = function(request)
  local requestId = extensions.tech_sensors.sendAdvancedIMURequest(advancedIMUs[request['name']], request['vid'])
  local resp = {type = 'requestId', data = requestId}
  request:sendResponse(resp)
end

M.handleIsAdHocPollRequestReadyAdvancedIMU = function(request)
  local isRequestComplete = extensions.tech_sensors.isVluaRequestComplete(request['requestId'])
  local resp = {type = 'isRequestComplete', data = isRequestComplete}
  request:sendResponse(resp)
end

M.handleCollectAdHocPollRequestAdvancedIMU = function(request)
  local reading = extensions.tech_sensors.collectAdvancedIMURequest(request['requestId'])
  local resp = {type = 'AdHocPollRequestData', data = reading}
  request:sendResponse(resp)
end

M.handleSetAdvancedIMURequestedUpdateTime = function(request)
  extensions.tech_sensors.setAdvancedIMUUpdateTime(advancedIMUs[request['name']], request['vid'], request['updateTime'])
  request:sendACK('CompletedSetAdvancedIMURequestedUpdateTime')
end

M.handleSetAdvancedIMUIsUsingGravity = function(request)
  extensions.tech_sensors.setAdvancedIMUIsUsingGravity(advancedIMUs[request['name']], request['vid'], request['isUsingGravity'])
  request:sendACK('CompletedSetAdvancedIMUIsUsingGravity')
end

M.handleSetAdvancedIMUIsVisualised = function(request)
  extensions.tech_sensors.setAdvancedIMUIsVisualised(advancedIMUs[request['name']], request['vid'], request['isVisualised'])
  request:sendACK('CompletedSetAdvancedIMUIsVisualised')
end

M.handleOpenGPS = function(request)
  if not ResearchVerifier.isTechLicenseVerified() then
    reportMissingLicenseFeature(request)
    return false
  end

  local args = {}
  args.GFXUpdateTime = request['GFXUpdateTime']
  args.physicsUpdateTime = request['physicsUpdateTime']
  args.pos = vec3(request['pos'][1], request['pos'][2], request['pos'][3])
  args.refLon = request['refLon']
  args.refLat = request['refLat']
  args.isSendImmediately = request['isSendImmediately']
  args.isVisualised = request['isVisualised']
  args.isSnappingDesired = request['isSnappingDesired']
  args.isForceInsideTriangle = request['isForceInsideTriangle']
  args.isDirWorldSpace = request['isDirWorldSpace']

  local name = request['name']
  local vid = scenetree.findObject(request['vid']):getID();

  GPSs[name] = extensions.tech_sensors.createGPS(vid, args)
  log('I', logTag, 'Opened GPS sensor')

  request:sendACK('OpenedGPS')
end

M.handleCloseGPS = function(request)
  local name = request['name']
  local vid = request['vid']
  local sensorId = GPSs[name]
  if sensorId ~= nil then
    GPSs[name] = nil                                    -- remove from ge lua
    extensions.tech_sensors.removeGPS(vid, sensorId)    -- remove from vlua.
    log('I', logTag, 'Closed GPS sensor')
  end

  request:sendACK('ClosedGPS')
end

M.handlePollGPSGE = function(request)
  local name = request['name']
  local sensorId = GPSs[name]
  if sensorId ~= nil then
    local readings = extensions.tech_sensors.getGPSReadings(sensorId)
    if readings ~= nil then
      local resp = { type = 'PollGPSGE', data = readings }
      request:sendResponse(resp)
      return true
    end
  end

  -- The sensor was not found, or the readings did not exist, so send an empty response.
  local resp = {type = 'PollGPSGE', data = {} }
  log('I', logTag, 'WARNING: GPS sensor not found')
  request:sendResponse(resp)
end

M.handleSendAdHocRequestGPS = function(request)
  local requestId = extensions.tech_sensors.sendGPSRequest(GPSs[request['name']], request['vid'])
  local resp = {type = 'requestId', data = requestId}
  request:sendResponse(resp)
end

M.handleIsAdHocPollRequestReadyGPS = function(request)
  local isRequestComplete = extensions.tech_sensors.isVluaRequestComplete(request['requestId'])
  local resp = {type = 'isRequestComplete', data = isRequestComplete}
  request:sendResponse(resp)
end

M.handleCollectAdHocPollRequestGPS = function(request)
  local reading = extensions.tech_sensors.collectGPSRequest(request['requestId'])
  local resp = {type = 'AdHocPollRequestData', data = reading}
  request:sendResponse(resp)
end

M.handleSetGPSRequestedUpdateTime = function(request)
  extensions.tech_sensors.setGPSUpdateTime(GPSs[request['name']], request['vid'], request['updateTime'])
  request:sendACK('CompletedSetGPSRequestedUpdateTime')
end

M.handleSetGPSIsVisualised = function(request)
  extensions.tech_sensors.setGPSIsVisualised(GPSs[request['name']], request['vid'], request['isVisualised'])
  request:sendACK('CompletedSetGPSIsVisualised')
end

M.handleOpenPowertrain = function(request)
  if not ResearchVerifier.isTechLicenseVerified() then
    reportMissingLicenseFeature(request)
    return false
  end

  local args = {}
  args.GFXUpdateTime = request['GFXUpdateTime']
  args.physicsUpdateTime = request['physicsUpdateTime']
  args.isSendImmediately = request['isSendImmediately']

  local name = request['name']
  local vid = scenetree.findObject(request['vid']):getID();

  powertrains[name] = extensions.tech_sensors.createPowertrainSensor(vid, args)
  log('I', logTag, 'Opened Powertrain sensor')

  request:sendACK('OpenedPowertrain')
end

M.handleClosePowertrain = function(request)
  local name = request['name']
  local vid = request['vid']
  local sensorId = powertrains[name]
  if sensorId ~= nil then
    powertrains[name] = nil                                    -- remove from ge lua
    extensions.tech_sensors.removePowertrainSensor(vid, sensorId)    -- remove from vlua.
    log('I', logTag, 'Closed Powertrain sensor')
  end

  request:sendACK('ClosedPowertrain')
end

M.handlePollPowertrainGE = function(request)
  local name = request['name']
  local sensorId = powertrains[name]
  if sensorId ~= nil then
    local readings = extensions.tech_sensors.getPowertrainReadings(sensorId)
    if readings ~= nil then
      local resp = { type = 'PollPowertrainGE', data = readings }
      request:sendResponse(resp)
      return true
    end
  end

  -- The sensor was not found, or the readings did not exist, so send an empty response.
  local resp = {type = 'PollPowertrainGE', data = {} }
  log('I', logTag, 'WARNING: Powertrain sensor not found')
  request:sendResponse(resp)
end

M.handleSendAdHocRequestPowertrain = function(request)
  local requestId = extensions.tech_sensors.sendPowertrainRequest(powertrains[request['name']], request['vid'])
  local resp = {type = 'requestId', data = requestId}
  request:sendResponse(resp)
end

M.handleIsAdHocPollRequestReadyPowertrain = function(request)
  local isRequestComplete = extensions.tech_sensors.isVluaRequestComplete(request['requestId'])
  local resp = {type = 'isRequestComplete', data = isRequestComplete}
  request:sendResponse(resp)
end

M.handleCollectAdHocPollRequestPowertrain = function(request)
  local reading = extensions.tech_sensors.collectPowertrainRequest(request['requestId'])
  local resp = {type = 'AdHocPollRequestData', data = reading}
  request:sendResponse(resp)
end

M.handleSetPowertrainRequestedUpdateTime = function(request)
  extensions.tech_sensors.setPowertrainUpdateTime(powertrains[request['name']], request['vid'], request['updateTime'])
  request:sendACK('CompletedSetPowertrainRequestedUpdateTime')
end

M.handleOpenMesh = function(request)
  if not ResearchVerifier.isTechLicenseVerified() then
    reportMissingLicenseFeature(request)
    return false
  end

  local args = {}
  args.GFXUpdateTime = request['GFXUpdateTime']
  args.physicsUpdateTime = request['physicsUpdateTime']

  local name = request['name']
  local vid = scenetree.findObject(request['vid']):getID();

  meshes[name] = extensions.tech_sensors.createMeshSensor(vid, args)
  log('I', logTag, 'Opened Mesh sensor')

  request:sendACK('OpenedMesh')
end

M.handleCloseMesh = function(request)
  local name = request['name']
  local vid = request['vid']
  local sensorId = meshes[name]
  if sensorId ~= nil then
    meshes[name] = nil                                                -- remove from ge lua
    extensions.tech_sensors.removeMeshSensor(vid, sensorId)           -- remove from vlua.
    log('I', logTag, 'Closed Mesh sensor')
  end

  request:sendACK('ClosedMesh')
end

M.handleSendAdHocRequestMesh = function(request)
  local requestId = extensions.tech_sensors.sendMeshRequest(meshes[request['name']], request['vid'])
  local resp = {type = 'requestId', data = requestId}
  request:sendResponse(resp)
end

M.handleIsAdHocPollRequestReadyMesh = function(request)
  local isRequestComplete = extensions.tech_sensors.isVluaRequestComplete(request['requestId'])
  local resp = {type = 'isRequestComplete', data = isRequestComplete}
  request:sendResponse(resp)
end

M.handleCollectAdHocPollRequestMesh = function(request)
  local reading = extensions.tech_sensors.collectMeshRequest(request['requestId'])
  local resp = {type = 'AdHocPollRequestData', data = reading}
  request:sendResponse(resp)
end

M.handleSetMeshRequestedUpdateTime = function(request)
  extensions.tech_sensors.setMeshUpdateTime(powertrains[request['name']], request['vid'], request['updateTime'])
  request:sendACK('CompletedSetMeshRequestedUpdateTime')
end

M.handleOpenIdealRADAR = function(request)
  if not ResearchVerifier.isTechLicenseVerified() then
    reportMissingLicenseFeature(request)
    return false
  end

  local args = {}
  args.GFXUpdateTime = request['GFXUpdateTime']
  args.physicsUpdateTime = request['physicsUpdateTime']

  local name = request['name']
  local vid = scenetree.findObject(request['vid']):getID();

  idealRADARs[name] = extensions.tech_sensors.createIdealRADARSensor(vid, args)
  log('I', 'Opened Ideal RADAR sensor')

  request:sendACK('OpenedIdealRADAR')
end

M.handleCloseIdealRADAR = function(request)
  local name = request['name']
  local vid = request['vid']
  local sensorId = idealRADARs[name]
  if sensorId ~= nil then
    idealRADARs[name] = nil                                                -- remove from ge lua
    extensions.tech_sensors.removeIdealRADARSensor(vid, sensorId)           -- remove from vlua.
    log('I', 'Closed Ideal RADAR sensor')
  end

  request:sendACK('ClosedIdealRADAR')
end

M.handlePollIdealRADARGE = function(request)
  local name = request['name']
  local sensorId = idealRADARs[name]
  if sensorId ~= nil then
    local readings = extensions.tech_sensors.getIdealRADARReadings(sensorId)
    if readings ~= nil then
      local resp = { type = 'PollIdealRADARGE', data = readings }
      request:sendResponse(resp)
      return true
    end
  end

  -- The sensor was not found, or the readings did not exist, so send an empty response.
  local resp = {type = 'PollIdealRADARGE', data = {} }
  log('I', 'WARNING: Ideal RADAR sensor not found')
  request:sendResponse(resp)
end

M.handleSendAdHocRequestIdealRADAR = function(request)
  local requestId = extensions.tech_sensors.sendIdealRADARRequest(idealRADARs[request['name']], request['vid'])
  local resp = {type = 'requestId', data = requestId}
  request:sendResponse(resp)
end

M.handleIsAdHocPollRequestReadyIdealRADAR = function(request)
  local isRequestComplete = extensions.tech_sensors.isVluaRequestComplete(request['requestId'])
  local resp = {type = 'isRequestComplete', data = isRequestComplete}
  request:sendResponse(resp)
end

M.handleCollectAdHocPollRequestIdealRADAR = function(request)
  local reading = extensions.tech_sensors.collectIdealRADARRequest(request['requestId'])
  local resp = {type = 'AdHocPollRequestData', data = reading}
  request:sendResponse(resp)
end

M.handleSetIdealRADARRequestedUpdateTime = function(request)
  extensions.tech_sensors.setIdealRADARUpdateTime(idealRADARs[request['name']], request['vid'], request['updateTime'])
  request:sendACK('CompletedSetIdealRADARRequestedUpdateTime')
end

M.handleOpenRoadsSensor = function(request)
  if not ResearchVerifier.isTechLicenseVerified() then
    reportMissingLicenseFeature(request)
    return false
  end

  local args = {}
  args.GFXUpdateTime = request['GFXUpdateTime']
  args.physicsUpdateTime = request['physicsUpdateTime']
  args.isSendImmediately = request['isSendImmediately']

  local name = request['name']
  local vid = scenetree.findObject(request['vid']):getID();

  roadsSensors[name] = extensions.tech_sensors.createRoadsSensor(vid, args)
  log('I', logTag, 'Opened Roads sensor')

  request:sendACK('OpenedRoadsSensor')
end

M.handleCloseRoadsSensor = function(request)
  local name = request['name']
  local vid = request['vid']
  local sensorId = roadsSensors[name]
  if sensorId ~= nil then
    roadsSensors[name] = nil                                    -- remove from ge lua
    extensions.tech_sensors.removeRoadsSensor(vid, sensorId)    -- remove from vlua.
    log('I', logTag, 'Closed Roads sensor')
  end

  request:sendACK('ClosedRoadsSensor')
end

M.handlePollRoadsSensorGE = function(request)
  local name = request['name']
  local sensorId = roadsSensors[name]
  if sensorId ~= nil then
    local readings = extensions.tech_sensors.getRoadsSensorReadings(sensorId)
    if readings ~= nil then
      local resp = { type = 'PollRoadsSensorGE', data = readings }
      request:sendResponse(resp)
      return true
    end
  end

  -- The sensor was not found, or the readings did not exist, so send an empty response.
  local resp = {type = 'PollRoadsSensorGE', data = {} }
  log('I', logTag, 'WARNING: Roads sensor not found')
  request:sendResponse(resp)
end

M.handleSendAdHocRequestRoadsSensor = function(request)
  local requestId = extensions.tech_sensors.sendRoadsSensorRequest(roadsSensors[request['name']], request['vid'])
  local resp = {type = 'requestId', data = requestId}
  request:sendResponse(resp)
end

M.handleIsAdHocPollRequestReadyRoadsSensor = function(request)
  local isRequestComplete = extensions.tech_sensors.isVluaRequestComplete(request['requestId'])
  local resp = {type = 'isRequestComplete', data = isRequestComplete}
  request:sendResponse(resp)
end

M.handleCollectAdHocPollRequestRoadsSensor = function(request)
  local reading = extensions.tech_sensors.collectRoadsSensorRequest(request['requestId'])
  local resp = {type = 'AdHocPollRequestData', data = reading}
  request:sendResponse(resp)
end

M.handleSetRoadsSensorRequestedUpdateTime = function(request)
  extensions.tech_sensors.setRoadsSensorUpdateTime(roadsSensors[request['name']], request['vid'], request['updateTime'])
  request:sendACK('CompletedSetRoadsSensorRequestedUpdateTime')
end

M.handleOpenVehicleFeeder = function(request)
  if not ResearchVerifier.isTechLicenseVerified() then
    reportMissingLicenseFeature(request)
    return false
  end

  local name = request['name']
  local vid = scenetree.findObject(request['vid']):getID()

  vehicleFeeders[name] = extensions.tech_sensors.createValidation(vid, request['testId'])
  log('I', logTag, 'Opened Vehicle Feeder')

  request:sendACK('OpenedVehicleFeeder')
end

M.handleCloseVehicleFeeder = function(request)
  local name = request['name']
  local vid = request['vid']
  local sensorId = vehicleFeeders[name]
  if sensorId ~= nil then
    vehicleFeeders[name] = nil                                      -- remove from ge lua
    extensions.tech_sensors.removeValidation(vid, sensorId)         -- remove from vlua.
    log('I', logTag, 'Closed Vehicle Feeder')
  end

  request:sendACK('ClosedVehicleFeeder')
end

M.handleIsTimeEvolutionComplete = function(request)
  local vid = scenetree.findObject(request['vid']):getID()
  local data = extensions.tech_sensors.isTimeEvolutionComplete(vid)
  local resp = {type = 'IsTimeEvolutionComplete', data = data}
  request:sendResponse(resp)
end

M.handleGetRoadGraph = function(request)
  local reading = extensions.tech_sensors.getRoadGraph()
  local resp = {type = 'GetRoadGraph', data = reading}
  request:sendResponse(resp)
end

M.handleExportOpenDrive = function(request)
  local filename = request['filename']
  local reading = extensions.tech_openDriveExporter.export(filename)
  local resp = {type = 'ExportOpenDrive', data = nil}
  request:sendResponse(resp)
end

M.handleExportOpenStreetMap = function(request)
  local filename = request['filename']
  local reading = extensions.tech_openStreetMapExporter.export(filename)
  local resp = {type = 'ExportOpenStreetMap', data = nil}
  request:sendResponse(resp)
end
M.handleExportSumo = function(request)
  local filename = request['filename']
  local reading = extensions.tech_sumoExporter.export(filename)
  local resp = {type = 'ExportSumo', data = nil}
  request:sendResponse(resp)
end


M.handleResetNavgraph = function(request)
  extensions.tech_sensors.resetNavgraph()
  local resp = {type = 'ResetNavgraph', data = nil}
  request:sendResponse(resp)
end

M.handleGetBeamData = function(request)
  local data = extensions.tech_sensors.getBeamData(request['vid'])
  local resp = {type = 'GetBeamData', data = data}
  request:sendResponse(resp)
end

M.handleGetFullTriangleData = function(request)
  local data = extensions.tech_sensors.getFullTriangleData(request['vid'])
  local resp = {type = 'GetFullTriangleData', data = data}
  request:sendResponse(resp)
end

M.handleGetWheelTriangleData = function(request)
  local data = extensions.tech_sensors.getWheelTriangleData(request['vid'], request['wheelIndex'])
  local resp = {type = 'GetWheelTriangleData', data = data}
  request:sendResponse(resp)
end

M.handleGetNodePositions = function(request)
  local data = extensions.tech_sensors.getNodePositions(request['vid'])
  local resp = {type = 'GetNodePositions', data = data}
  request:sendResponse(resp)
end

M.handleGetClosestMeshPointToGivenPoint = function(request)
  local data = extensions.tech_sensors.getClosestMeshPointToGivenPoint(request['vid'], request['point'])
  local resp = {type = 'GetClosestMeshPointToGivenPoint', data = {x = data.x, y = data.y, z = data.z}}
  request:sendResponse(resp)
end

M.handleGetClosestTriangle = function(request)
  local data = extensions.tech_sensors.getClosestTriangle(request['vid'], request['point'], request['includeWheelNodes'])
  local resp = {type = 'GetClosestTriangle', data = data}
  request:sendResponse(resp)
end

M.handleSetWeatherPreset = function(request)
  local preset = request['preset']
  local time = request['time']
  core_weather.switchWeather(preset, time)
  request:sendACK('WeatherPresetChanged')
end

M.handleDisplayGuiMessage = function(request)
  local message = request['message']
  guihooks.message(message)
  request:sendACK('GuiMessageDisplayed')
end

M.handleSwitchVehicle = function(request)
  local vID = request['vid']
  local vehicle = scenetree.findObject(vID)
  be:enterVehicle(0, vehicle)
  request:sendACK('VehicleSwitched')
end

M.handleSetFreeCamera = function(request)
  local pos = request['pos']
  local direction = request['dir']
  local rot = quatFromDir(vec3(direction[1], direction[2], direction[3]))

  commands.setFreeCamera()
  core_camera.setPosRot(0, pos[1], pos[2], pos[3], rot.x, rot.y, rot.z, rot.w)
  request:sendACK('FreeCameraSet')
end

M.handleParticlesEnabled = function(request)
  local enabled = request['enabled']
  Engine.Render.ParticleMgr.setEnabled(enabled)
  request:sendACK('ParticlesSet')
end

M.handleAnnotateParts = function(request)
  local vehicle = scenetree.findObject(request['vid'])
  tech_partAnnotations.annotateParts(vehicle:getID())
  request:sendACK('PartsAnnotated')
end

M.handleRevertAnnotations = function(request)
  local vehicle = scenetree.findObject(request['vid'])
  tech_partAnnotations.revertAnnotations(vehicle:getID())
  request:sendACK('AnnotationsReverted')
end

M.handleGetPartAnnotations = function(request)
  local vehicle = scenetree.findObject(request['vid'])
  local colors = tech_partAnnotations.getPartAnnotations(vehicle:getID())
  local converted = {}
  for key, val in pairs(colors) do
    converted[key] = {val.r, val.g, val.b}
  end
  request:sendResponse({type = 'PartAnnotations', colors = converted})
end

M.handleGetPartAnnotation = function(request)
  local part = request['part']
  local color = tech_partAnnotations.getPartAnnotation(part)
  if color ~= nil then
    color = {color.r, color.g, color.b}
  end
  request:sendResponse({type = 'PartAnnotation', color = color})
end

M.handleGetAnnotations = function(request)
  if not ResearchVerifier.isTechLicenseVerified() then
    reportMissingLicenseFeature(request)
    return false
  end

  local annotations = AnnotationManager.getAnnotations()
  for k, v in pairs(annotations) do
    annotations[k] = {v.r, v.g, v.b}
  end
  local ret = {type = 'Annotations', annotations = annotations}
  request:sendResponse(ret)
end

M.handleFindObjectsClass = function(request)
  local clazz = request['class']
  local objects = scenetree.findClassObjects(clazz)
  local resp = {type='ClassObjects'}
  local list = {}
  for idx, object in ipairs(objects) do
    object = scenetree.findObject(object)

    local obj = {type=clazz, id=object:getID(), name=object:getName()}

    local scl = object:getScale()
    local pos = object:getPosition()
    local rot = object:getRotation()
    if clazz == 'BeamNGVehicle' then
      local vehicleData = map.objects[obj.id]
      rot = quatFromDir(vehicleData.dirVec, vehicleData.dirVecUp)
    end

    pos = {pos.x, pos.y, pos.z}
    rot ={rot.x, rot.y, rot.z, rot.w}

    scl = {scl.x, scl.y, scl.z}

    obj['position'] = pos
    obj['rotation'] = rot
    obj['scale'] = scl

    obj['options'] = {}
    for fld, nfo in pairs(object:getFieldList()) do
      if fld ~= 'position' and fld ~= 'rotation' and fld ~= 'scale' and fld ~= 'id' and fld ~= 'type' and fld ~= 'name' then
        local val = object:getField(fld, '')
        obj['options'][fld] = val
      end
    end

    table.insert(list, obj)
  end
  resp['objects'] = list
  request:sendResponse(resp)
end

M.handleCreateCylinder = function(request)
  local name = request['name']
  local radius = request['radius']
  local height = request['height']
  local material = request['material']
  local pos = request['pos']
  local rot = request['rot']

  local cylinder = procPrimitives.createCylinder(radius, height, material)
  placeObject(name, cylinder, pos, rot)

  request:sendACK('CreatedCylinder')
end

M.handleCreateBump = function(request)
  local name = request['name']
  local length = request['length']
  local width = request['width']
  local height = request['height']
  local upperLength = request['upperLength']
  local upperWidth = request['upperWidth']
  local material = request['material']
  local pos = request['pos']
  local rot = request['rot']

  local bump = procPrimitives.createBump(length, width, height, upperLength, upperWidth, material)
  placeObject(name, bump, pos, rot)

  request:sendACK('CreatedBump')
end

M.handleCreateCone = function(request)
  local name = request['name']
  local radius = request['radius']
  local height = request['height']
  local material = request['material']
  local pos = request['pos']
  local rot = request['rot']

  local cone = procPrimitives.createCone(radius, height, material)
  placeObject(name, cone, pos, rot)

  request:sendACK('CreatedCone')
end

M.handleCreateCube = function(request)
  local name = request['name']
  local size = vec3(request['size'])
  local material = request['material']
  local pos = request['pos']
  local rot = request['rot']

  local cube = procPrimitives.createCube(size, material)
  placeObject(name, cube, pos, rot)

  request:sendACK('CreatedCube')
end

M.handleCreateRing = function(request)
  local name = request['name']
  local radius = request['radius']
  local thickness = request['thickness']
  local material = request['material']
  local pos = request['pos']
  local rot = request['rot']

  local ring = procPrimitives.createRing(radius, thickness, material)
  placeObject(name, ring, pos, rot)

  request:sendACK('CreatedRing')
end

M.handleRemoveObject = function(request)
  local name = request['name']
  local obj = scenetree.findObject(name)
  if obj == nil then
    request:sendBNGError('Object not found in scenetree: \'' .. name .. '\'')
    return false
  end

  obj:delete()
  request:sendACK('RemovedObject')
end

M.handleGetBBoxCorners = function(request)
  local veh = scenetree.findObject(request['vid'])
  local resp = {type = 'BBoxCorners'}
  local points = {}
  local bbox = veh:getSpawnWorldOOBB()
  for i = 0, 7 do
    local point = bbox:getPoint(i)
    point = {tonumber(point.x), tonumber(point.y), tonumber(point.z)}
    table.insert(points, point)
  end
  resp['points'] = points
  request:sendResponse(resp)
end

M.handleGetGravity = function(request)
  request:sendResponse({type = 'Gravity', gravity = core_environment.getGravity()})
end

M.handleSetGravity = function(request)
  local gravity = request['gravity']
  core_environment.setGravity(gravity)
  request:sendACK('GravitySet')
end

M.handleGetAvailableVehicles = function(request)
  local resp = {type = 'AvailableVehicles', vehicles = {}}

  local models = core_vehicles.getModelList().models
  local configs = core_vehicles.getConfigList().configs

  for model, modelData in pairs(models) do
    local data = {
      author = modelData.Author,
      name = modelData.Name,
      type = modelData.Type,
      key = modelData.key,
    }
    data.configurations = {}
    for key, config in pairs(configs) do
      if config.model_key == model then
        data.configurations[config.key] = {
          author = config.Author,
          model_key = config.model_key,
          key = config.key,
          name = config.Name,
          type = config.Type
        }
      end
    end
    resp.vehicles[model] = data
  end

  request:sendResponse(resp)
end

M.handleSpawnTraffic = function(request)
  local maxAmount = request['max_amount']
  local policeRatio = request['police_ratio']
  local extraAmount = request['extra_amount']
  local parkedAmount = request['parked_amount']

  gameplay_parking.setupVehicles(parkedAmount)
  gameplay_traffic.setupTraffic(maxAmount, policeRatio)
  request:sendACK('TrafficSpawned')
end

M.handleStartTraffic = function(request)
  local participants = request.participants
  local ids = {}
  for idx, participant in ipairs(participants) do
    local veh = scenetree.findObject(participant)

    if veh == nil then
      request:sendBNGValueError('Vehicle not present for traffic: ' .. tostring(participant))
      return
    end

    table.insert(ids, veh:getID())
  end

  gameplay_traffic.activate(ids)
  request:sendACK('TrafficStarted')
end

M.handleResetTraffic = function(request)
  gameplay_traffic.forceTeleportAll()
  request:sendACK('TrafficReset')
end

M.handleStopTraffic = function(request)
  local stop = request.stop
  gameplay_traffic.deactivate(stop)
  request:sendACK('TrafficStopped')
end

M.handleChangeSetting = function(request)
  local key = request['key']
  local value = request['value']
  settings.setValue(key, value, true)
  request:sendACK('SettingsChanged')
end

M.handleApplyGraphicsSetting = function(request)
  core_settings_graphic.applyGraphicsState()
  request:sendACK('GraphicsSettingApplied')
end

M.handleSetRelativeCam = function(request)
  core_camera.setByName(0, 'relative', false, {})

  local vid = getPlayerVehicle(0):getID()
  local pos = request['pos']
  local dir = request['dir']
  local func = function()
    core_camera.setOffset(vid, vec3(pos[1], pos[2], pos[3]))

    if dir ~= nil then
      local up = request['up']
      local vDir, vUp = vec3(dir[1], dir[2], dir[3]), vec3(up[1], up[2], -up[3])
      local rot = quatFromDir(vDir, vUp):toEulerYXZ() * 57.29578 -- rad to deg.
      core_camera.setRotation(vid, rot)
    end

    request:sendACK('RelativeCamSet')
  end
  addFrameDelayFunc(func, 3)
  return false
end

M.handleAddDebugSpheres = function(request)
  local sphereIDs = {}
  for idx = 1,#request.radii do
    local coo = tableToVec3(request.coordinates[idx], request.cling, request.offset)
    local color = request.colors[idx]
    color = ColorF(color[1], color[2], color[3], color[4])
    local sphere = {coo = coo, radius = request.radii[idx], color = color}
    debugObjectCounter.sphereNum = debugObjectCounter.sphereNum + 1
    debugObjects.spheres[debugObjectCounter.sphereNum] = sphere
    table.insert(sphereIDs, debugObjectCounter.sphereNum)
  end
  local resp = {type = 'DebugSphereAdded', sphereIDs = sphereIDs}
  request:sendResponse(resp)
end

M.handleRemoveDebugObjects = function(request)
  for _, idx in pairs(request.objIDs) do
    debugObjects[request.objType][idx] = nil
  end
  request:sendACK('DebugObjectsRemoved')
end

M.handleAddDebugPolyline = function(request)
  local polyline = {segments = {}}
  polyline.color = ColorF(request.color[1], request.color[2], request.color[3], request.color[4])
  local origin = tableToVec3(request.coordinates[1], request.cling, request.offset)
  for i = 2, #request.coordinates do
    local target = tableToVec3(request.coordinates[i], request.cling, request.offset)
    local segment = {origin = origin, target = target}
    table.insert(polyline.segments, segment)
    origin = target
  end
  debugObjectCounter.lineNum = debugObjectCounter.lineNum + 1
  table.insert(debugObjects.polylines, debugObjectCounter.lineNum, polyline)
  local resp = {type = 'DebugPolylineAdded', lineID = debugObjectCounter.lineNum}
  request:sendResponse(resp)
end

M.handleAddDebugCylinder = function(request)
  local circleAPos = tableToVec3(request.circlePositions[1], false, 0)
  local circleBPos = tableToVec3(request.circlePositions[2], false, 0)
  local color = ColorF(request.color[1], request.color[2], request.color[3], request.color[4])
  local cylinder = {circleAPos=circleAPos, circleBPos=circleBPos, radius=request.radius, color=color}
  debugObjectCounter.cylinderNum = debugObjectCounter.cylinderNum + 1
  table.insert(debugObjects.cylinders, debugObjectCounter.cylinderNum, cylinder)
  local resp = {type='DebugCylinderAdded', cylinderID=debugObjectCounter.cylinderNum}
  request:sendResponse(resp)
end

M.handleAddDebugTriangle = function(request)
  local color = ColorF(request.color[1], request.color[2], request.color[3], request.color[4])
  local pointA = tableToVec3(request.vertices[1], request.cling, request.offset)
  local pointB = tableToVec3(request.vertices[2], request.cling, request.offset)
  local pointC = tableToVec3(request.vertices[3], request.cling, request.offset)
  local triangle = {a=pointA, b=pointB, c=pointC, color=color}
  debugObjectCounter.triangleNum = debugObjectCounter.triangleNum + 1
  table.insert(debugObjects.triangles, debugObjectCounter.triangleNum, triangle)
  local resp = {type ='DebugTriangleAdded', triangleID = debugObjectCounter.triangleNum}
  request:sendResponse(resp)
end

M.handleAddDebugRectangle = function(request)
  local color = ColorF(request.color[1], request.color[2], request.color[3], request.color[4])
  local pointA = tableToVec3(request.vertices[1], request.cling, request.offset)
  local pointB = tableToVec3(request.vertices[2], request.cling, request.offset)
  local pointC = tableToVec3(request.vertices[3], request.cling, request.offset)
  local pointD = tableToVec3(request.vertices[4], request.cling, request.offset)
  local rectangle = {a=pointA, b=pointB, c=pointC, d=pointD, color=color}
  debugObjectCounter.rectangleNum = debugObjectCounter.rectangleNum + 1
  table.insert(debugObjects.rectangles, debugObjectCounter.rectangleNum, rectangle)
  local resp = {type ='DebugRectangleAdded', rectangleID = debugObjectCounter.rectangleNum}
  request:sendResponse(resp)
end

M.handleAddDebugText = function(request)
  local color = ColorF(request.color[1], request.color[2], request.color[3], request.color[4])
  local origin = tableToVec3(request.origin, request.cling, request.offset)
  local content = String(request.content)
  local text = {origin = origin, content = content, color = color}
  debugObjectCounter.textNum = debugObjectCounter.textNum + 1
  table.insert(debugObjects.text, debugObjectCounter.textNum, text)
  local resp = {type ='DebugTextAdded', textID = debugObjectCounter.textNum}
  request:sendResponse(resp)
end

M.handleAddDebugSquarePrism = function(request)
  local color = ColorF(request.color[1], request.color[2], request.color[3], request.color[4])
  local az, bz = request.endPoints[1][3], request.endPoints[2][3]
  local sideA = tableToVec3(request.endPoints[1], false, 0)
  local sideB = tableToVec3(request.endPoints[2], false, 0)
  local sideADims = Point2F(request.dims[1][1], request.dims[1][2])
  local sideBDims = Point2F(request.dims[2][1], request.dims[2][2])
  local prism = {sideA=sideA, sideB=sideB, sideADims=sideADims, sideBDims=sideBDims, color = color}
  debugObjectCounter.prismNum = debugObjectCounter.prismNum + 1
  table.insert(debugObjects.squarePrisms, debugObjectCounter.prismNum, prism)
  local resp = {type ='DebugSquarePrismAdded', prismID = debugObjectCounter.prismNum}
  request:sendResponse(resp)
end

M.handleQueueLuaCommandGE = function(request)
  local func, loading_err = load(request.chunk)
  local status, err
  if func then
    status, err = pcall(func)
    if not status then
      log('E', logTag, 'execution error: "' .. err .. '"')
    end
  else
    log('E', logTag, 'compilation error in: "' .. request.chunk .. '"')
  end
  if request.resp then
    if not status then
      request:sendBNGError(err)
    else
      request:sendResponse({type ='ExecutedLuaChunkGE', resp = tostring(err)})
    end
  else
    request:sendACK('ExecutedLuaChunkGE')
  end
end

M.handleGetLevels = function(request)
  local list = core_levels.getList()
  local resp = {type = 'GetLevels', result = list}
  request:sendResponse(resp)
end

M.handleGetScenarios = function(request)
  local levels = request['levels']

  local response
  if levels == nil then
    response = scenarios
  elseif next(levels) == nil then
    request:sendBNGValueError('The levels field cannot be empty!')
    return false
  else
    local levelSet = {}
    for _, level in ipairs(levels) do
      levelSet[level] = true
    end
    response = {}
    for i, scenario in pairs(scenarios) do
      if levelSet[scenario.levelName] then
        response[i] = scenario
      end
    end
  end

  local resp = {type = 'GetScenarios', result = response}
  request:sendResponse(resp)
end

M.handleGetCurrentScenario = function(request)
  if not scenario_scenarios and not getRunningFlowgraphManager() then
    request:sendBNGValueError('No scenario loaded.')
    return false
  end

  local sourceFile
  local missionId = gameplay_missions_missionManager.getForegroundMissionId()
  if missionId ~= nil then
    sourceFile = missionPathFromId(missionId)
  elseif scenario_scenarios then
    sourceFile = scenario_scenarios.getScenario().sourceFile
  else
    local fgMgr = getRunningFlowgraphManager()
    local name = string.gsub(fgMgr.savedFilename, "(.*)%.flow.json", "%1")
    sourceFile = fgMgr.savedDir .. name .. '.json'
    if string.sub(sourceFile, 1, 1) ~= '/' then
      sourceFile = '/' .. sourceFile
    end
  end

  local scenario = scenarios[sourceFile:lower()]
  local level = core_levels.getLevelByName(scenario.levelName)
  scenario.level = level

  local resp = {type = 'GetCurrentScenario', result = scenario}
  request:sendResponse(resp)
end

M.handleCreateScenario = function(request)
  local name = request['name']
  local level = request['level']
  local prefab = request['prefab']
  local info = request['info']
  local jsonPrefab = request['json']

  if name == nil then
    request:sendBNGValueError('Scenario needs a name.')
    return false
  end

  if level == nil then
    request:sendBNGValueError('Scenario needs an associated level.')
    return false
  end

  if info == nil then
    request:sendBNGValueError('Scenario needs an info file definition.')
    return false
  end

  local path = '/levels/' .. level .. '/scenarios/'
  local infoPath = path .. name .. '.json'

  local writePrefab = true
  if prefab ~= nil then
    local extension = jsonPrefab and '.prefab.json' or '.prefab'

    local otherExtension = jsonPrefab and '.prefab' or '.prefab.json'
    local otherPrefabPath = path .. name .. otherExtension
    if FS:fileExists(FS:expandFilename(otherPrefabPath)) then
      log('I', logTag, 'Deleting previous scenario prefab file \'' .. otherPrefabPath .. '\', as it is in a different format.')
      FS:remove(otherPrefabPath)
    end

    local prefabPath = path .. name .. extension
    local existingFile = io.open(prefabPath, 'r')
    if existingFile then
      local content = existingFile:read('a')
      -- when a prefab file is rewritten, the game reloads the vehicles in it, causing a bug
      -- this check is to limit this buggy behaviour until it is fixed in the game engine
      if content == prefab then
        writePrefab = false
      end
      existingFile:close()
    end

    if writePrefab then
      local outFile = io.open(prefabPath, 'w')
      if not outFile then
        request:sendBNGValueError('Could not write scenario prefab file.')
        return false
      end
      outFile:write(prefab)
      outFile:flush()
      outFile:close()

      local scenario = scenario_scenarios and scenario_scenarios.getScenario()
      if scenario and scenario.sourceFile == infoPath then
        log('W', logTag, 'Overwritten currently loaded scenario\'s prefab file. The scenario has to be stopped.')
        block('returnMainMenuCreateScenario', request)
        returnToMainMenu()
      end
    end
  end

  local outFile = io.open(infoPath, 'w')
  if not outFile then
    request:sendBNGValueError('Could not write scenario info file.')
    return false
  end
  outFile:write(jsonEncode({info}))
  outFile:flush()
  outFile:close()

  scenarios[infoPath:lower()] = scenariosLoader.loadScenario(infoPath)

  FS:updateDirectoryWatchers() -- late prefab file notification could cause a bug, update explicitly
  local resp = {type = 'CreateScenario', result = infoPath}

  if isBlocking('returnMainMenuCreateScenario') then
    blocking.data = resp
    request:markHandled() -- will be handled after we get back to main menu
  else
    request:sendResponse(resp)
  end

  return false
end

M.handleDeleteScenario = function(request)
  local infoPath = request['path']
  local scenarioDir, infoFile, _ = path.splitWithoutExt(infoPath)
  local prefabPath = scenarioDir .. infoFile .. '.prefab.json'

  FS:removeFile(infoPath)
  FS:removeFile(prefabPath)

  request:sendACK('DeleteScenario')
end



M.handleGameStateRequest = function(request)
  local state = core_gamestate.state.state
  local resp = {type = 'GameState'}
  if state == 'scenario' then
    resp['state'] = 'scenario'
    resp['scenario_state'] = scenario_scenarios.getScenario().state
    resp['level'] = getCurrentLevelIdentifier()
  elseif  state == 'freeroam' then
    resp['state'] = 'freeroam'
  elseif  state == 'garage' then
    resp['state'] = 'garage'
  else
    resp['state'] = 'menu'
  end
  request:sendResponse(resp)
end

M.handleGetPlayerVehicleID = function(request)
  local vehicle = getPlayerVehicle(0)
  if not vehicle then
    request:sendBNGError('There is no vehicle')
  else
    local vehicle = getPlayerVehicle(0)
    local id = vehicle:getID()
    local vid = vehicle:getName()
    local resp = {type = 'getPlayerVehicleID', vid= vid, id=id}
    request:sendResponse(resp)
  end
end


M.handleGetCurrentVehicles = function(request)
  local vehicleInfo = {}
  local vehicleInfoPending = {}

  for k, veh in pairs(getAllVehicles()) do
    if not veh:getActive() then goto continue end

    local info = {}
    local id = veh:getId()
    info['id'] = id
    info['model'] = veh:getJBeamFilename()
    info['name'] = veh:getName()
    if info['name'] == nil then
      info['name'] = tostring(id)
    end

    if request['include_config'] then
      local playerVeh = getPlayerVehicle(0)
      local currentId = playerVeh and playerVeh:getID() or nil
      be:enterVehicle(0, veh)
      info['config'] = core_vehicle_partmgmt.getConfig()
      if playerVeh then
        be:enterVehicle(0, scenetree.findObjectById(currentId))
      else
        be:exitVehicle(0)
      end
    end

    info['options'] = jsonReadFile('/vehicles/' .. info['model'] .. '.json')
    vehicleInfo[id] = info

    vehicleInfoPending[id] = true
    veh:queueLuaCommand('extensions.load("tech/techCore")')
    veh:queueLuaCommand('tech_techCore.requestVehicleInfo()')

    ::continue::
  end

  if next(vehicleInfo) == nil then
    request:sendResponse({type = 'GetCurrentVehicles', result = vehicleInfo})
  else
    return block('vehicleInfo', request, {
      vehicleInfo = vehicleInfo,
      vehicleInfoPending = vehicleInfoPending
    })
  end
end

-- TODO: DEPRECATED, USE getSceneTreeNodeFull
local function getSceneTreeNode(obj)
  local node = {}
  node.class = obj:getClassName()
  node.name = obj:getName()
  node.id = obj:getID()
  if obj.getObject ~= nil and obj.getCount ~= nil then
    node.children = {}
    local count = obj:getCount()
    for i=0, count - 1 do
      local child = getSceneTreeNode(Sim.upcast(obj:getObject(i)))
      table.insert(node.children, child)
    end
  end
  return node
end

-- TODO: DEPRECATED, USE handleSyncScene
M.handleGetSceneTree = function(request)
  local rootGrp = Sim.upcast(Sim.findObject('MissionGroup'))
  local tree = getSceneTreeNode(rootGrp)
  local resp = {type = 'GetSceneTree', result = tree}
  request:sendResponse(resp)
end

local typeConverters = {}
typeConverters['MatrixPosition'] = function(t)
  return string.split(t)
end
typeConverters['MatrixRotation'] = function(t)
  return string.split(t)
end
typeConverters['vec3'] = function(t)
-- typeConverters['Point3F'] = function(t)
  return string.split(t)
end

local function serializeGenericObject(obj)
  local ignoreNames = {
    id = true,
    name = true,
    internalName = true,
    isSelectionEnabled = true,
    isRenderEnabled = true,
    hidden = true,
    canSaveDynamicFields = true,
    canSave = true,
    parentGroup = true,
    persistentId = true,
    rotationMatrix = true,
    class = true,
    superClass = true,
    edge = true,
    plane = true,
    point = true,
  }

  local okayTypes = {
    int = true,
    string = true,
    filename = true,
    float = true,
    MatrixPosition = true,
    MatrixRotation = true,
    annotation = true,
    bool = true,
    vec3 = true,
    -- Point3F = true,
    ColorF = true,
    TSMeshType = true,
  }

  local position = nil
  if obj.getPosition ~= nil then
    position = obj:getPosition()
    position = {position.x, position.y, position.z}
  else
    position = {0, 0, 0}
  end

  local rotation = nil
  if obj.getRotation ~= nil then
    rotation = obj:getRotation()
    rotation = {rotation.x, rotation.y, rotation.z, rotation.w}
  else
    rotation = {0, 0, 0, 0}
  end

  local scale = nil
  if obj.getScale ~= nil then
    scale = obj:getScale()
    scale = {scale.x, scale.y, scale.z}
  else
    scale = {0, 0, 0}
  end

  local ret = {
    id = obj:getID(),
    name = obj:getName(),
    class = obj:getClassName(),
    position = position,
    rotation = rotation,
    scale = scale
  }

  local fields = obj:getFieldList()
  for field, props in pairs(fields) do
    if ignoreNames[field] == nil then
      local type = props['type']
      if okayTypes[type] then
        local converter = typeConverters[type]
        if converter ~= nil then
          ret[field] = converter(obj:getField(field, ''))
        else
          ret[field] = obj:getField(field, '')
        end
      end
    end
  end

  return ret
end

local objectSerializers = {}
objectSerializers['DecalRoad'] = function(obj)
  local ret = serializeGenericObject(obj)

  local position = obj:getPosition()
  position = {position.x, position.y, position.z}
  local rotation = obj:getRotation()
  rotation = {rotation.x, rotation.y, rotation.z, rotation.w}
  local scale = obj:getScale()
  scale = {scale.x, scale.y, scale.z}

  local annotation = obj:getField('annotation', '')
  local detail = obj:getField('Detail', '')
  local material = obj:getField('Material', '')
  local breakAngle = obj:getField('breakAngle', '')
  local drivability = obj:getField('drivability', '')
  local flipDirection = obj:getField('flipDirection', '')
  local improvedSpline = obj:getField('improvedSpline', '')
  local lanesLeft = obj:getField('lanesLeft', '')
  local lanesRight = obj:getField('lanesRight', '')
  local oneWay = obj:getField('oneWay', '')
  local overObjects = obj:getField('overObjects', '')

  local lines = {}
  local edges = obj:getEdgesTable()
  for i = 1, #edges do
    local edge = edges[i]
    table.insert(lines, {
      left = {
        edge[1].x,
        edge[1].y,
        edge[1].z
      },
      middle = {
        edge[2].x,
        edge[2].y,
        edge[2].z
      },
      right = {
        edge[3].x,
        edge[3].y,
        edge[3].z
      }
    })
  end

  ret.lines = lines

  return ret
end

local function getSerializedObject(obj)
  obj = Sim.upcast(obj)
  local class = obj:getClassName()
  local serializer = objectSerializers[class]
  if serializer ~= nil then
    obj = serializer(obj)
  else
    obj = serializeGenericObject(obj)
  end
  return obj
end

local function getSceneTreeNodeFull(root)
  local unpack = unpack or table.unpack
  local rootNode = getSerializedObject(root)
  local objects = {{rootNode, root}}

  while #objects > 0 do
    local node, obj = unpack(table.remove(objects))

    if obj.getObject ~= nil and obj.getCount ~= nil then
      node.children = {}
      local count = obj:getCount()
      for i = 0, count - 1 do
        local childObj = Sim.upcast(obj:getObject(i))
        local childNode = getSerializedObject(childObj)
        table.insert(node.children, childNode)
        table.insert(objects, {childNode, childObj})
      end
    end
  end
  return rootNode
end

M.handleSyncScene = function(request)
  local rootGrp = Sim.upcast(Sim.findObject('MissionGroup'))
  local tree = getSceneTreeNodeFull(rootGrp)
  local resp = {type = 'SyncScene', result = tree}
  request:sendResponse(resp)
end

M.handleGetObject = function(request)
  local id = request['id']
  local obj = Sim.findObjectById(id)
  if obj ~= nil then
    obj = getSerializedObject(obj)
    local resp = {type = 'GetObject', result = obj}
    request:sendResponse(resp)
  else
    request:sendBNGValueError('Unknown object ID: ' .. tostring(id))
  end
end

M.handleGetPartConfig = function(request)
  local vid = request['vid']
  local veh = scenetree.findObject(vid)
  local cur = getPlayerVehicle(0)
  if cur ~= nil then
    cur = cur:getID()
  end

  be:enterVehicle(0, veh)
  local cfg = core_vehicle_partmgmt.getConfig()
  local resp = {type = 'PartConfig', config = cfg}

  if cur ~= nil then
    veh = scenetree.findObjectById(cur)
    be:enterVehicle(0, veh)
  end

  request:sendResponse(resp)
end

-- TODO: DEPRECATED, USE GetPartConfig
M.handleGetPartOptions = function(request)
  local vid = request['vid']
  local veh = scenetree.findObject(vid)
  local cur = getPlayerVehicle(0):getID()
  be:enterVehicle(0, veh)
  local data = core_vehicle_manager.getPlayerVehicleData()
  --local cfg = core_vehicle_partmgmt.getConfig()
  --jbeamIO.getCompatiblePartNamesForSlot(data.ioCtx, slotDef)
  local slotMap = jbeamIO.getAvailableSlotNameMap(data.ioCtx)
  local resp = {type = 'PartOptions', options = slotMap}
  veh = scenetree.findObjectById(cur)
  be:enterVehicle(0, veh)
  request:sendResponse(resp)
end

M.handleSetPartConfig = function(request)
  local vid = request['vid']
  local veh = scenetree.findObject(vid)
  local cur = getPlayerVehicle(0):getID()
  be:enterVehicle(0, veh)
  local cfg = request['config']
  core_vehicle_partmgmt.setConfig(cfg)
  veh = scenetree.findObjectById(cur)
  be:enterVehicle(0, veh)
end

M.handleSetPlayerCameraMode = function(request)
  local vid = request['vid']
  local mode = request['mode']
  local config = request['config']
  local customData = request['customData']

  local veh = scenetree.findObject(vid)
  local id = veh:getID()
  core_camera.setVehicleCameraByNameWithId(id, mode, nil, customData)

  for k, v in pairs(config) do
    if k == 'rotation' then
      local rotation = vec3(v[1], v[2], v[3])
      core_camera.setRotation(id, rotation)
    end

    if k == 'fov' then
      core_camera.setFOV(id, v)
    end

    if k == 'offset' then
      local offset = vec3(v[1], v[2], v[3])
      core_camera.setOffset(id, offset)
    end

    if k == 'distance' then
      core_camera.setDistance(id, v)
    end
  end

  request:sendACK('PlayerCameraModeSet')
end

M.handleGetPlayerCameraMode = function(request)
  local vid = request['vid']
  local veh = scenetree.findObject(vid)
  -- Serialize & deserialize to get rid of data MessagePack can't serialize
  local cameraData = deserialize(serialize(core_camera.getCameraDataById(veh:getID())))
  cameraData['unicycle'] = nil
  cameraData = tcom.sanitizeTable(cameraData)
  local resp = {type = 'PlayerCameraMode', cameraData = cameraData}
  request:sendResponse(resp)
end

M.handleLoadTrackBuilderTrack = function(request)
  local trackPath = request['path']
  if not FS:fileExists(trackPath) then
    request:sendBNGValueError('The track file does not exist.')
    return false
  end

  local _, filename, _ = path.split(sanitizePath(trackPath))

  local tb = extensions['util_trackBuilder_splineTrack']
  tb.load(jsonReadFile(trackPath), true, nil, nil, true, false)

  -- rename the created meshes so multiple trackbuilder mesh names don't collide
  for _, objName in ipairs(scenetree.findClassObjects("ProceduralMesh")) do
    if string.find(objName, "procMesh") and not string.find(objName, ".json") then
      scenetree.findObject(objName):setName(filename..'-'..objName)
    end
  end
  tb.unloadAll()

  request:sendACK('TrackBuilderTrackLoaded')
end

M.handleSetLicensePlate = function(request)
  if settings.getValue('SkipGenerateLicencePlate') then
    local err = 'Dynamic license plates are disabled, check you are not running on \'Low\' graphic settings and that the \'Skip generation of License Plates\' option is unchecked.'
    request:sendBNGError(err)
    return false
  end
  local veh = scenetree.findObject(request['vid'])
  if not veh then
    request:sendBNGValueError('Vehicle not found: "' .. tostring(request['vid']) .. '"')
    return false
  end
  core_vehicles.setPlateText(request['text'], veh:getID())
  request:sendACK('SetLicensePlate')
end

M.handleGetSystemInfo = function(request)
  local response = {
    type = 'GetSystemInfo',
    tech = ResearchVerifier.isTechLicenseVerified()
  }
  if request['os'] then
    response['os'] = Engine.Platform.getOSInfo()
  end
  if request['power'] then
    response['power'] = Engine.Platform.getPowerInfo()
  end
  if request['cpu'] then
    response['cpu'] = Engine.Platform.getCPUInfo()
  end
  if request['gpu'] then
    response['gpu'] = Engine.Platform.getGPUInfo()
  end

  request:sendResponse(response)
end

-- Imports a heightmap from beamngpy (the old way).  THIS IS DEPRECATED AS OF 08/08/2024.
M.handleImportHeightmap = function(request)
  extensions.tech_terrainImporter.importHeightmap(request.data, request.w, request.h, request.scale, request.zMin, request.zMax, request.isYFlipped)
  log('I', logTag, 'Heightmap imported.')
  request:sendACK('CompletedImportHeightmap')
end

-- Imports a terrain and lays a collection of roads on it along with some terraforming.
-- The path of the terrain .png is provided. Roads are provided as a table/dictionary.
M.handleTerrainAndRoadImport = function(request)
  extensions.tech_terrainImporter.terrainAndRoadImport(request.pngPath, request.roads, request.DOI, request.margin, request.zMax)
  log('I', logTag, 'Terrain and roads - imported.')
  request:sendACK('CompletedTerrainAndRoadImport')
end

-- Creates a a terrain from a collection of peaks and troughs on a grid, and lays a collection of roads on them, along with some terraforming.
-- The peaks and roads are provided as table/dictionaries.
M.handlePeaksAndRoadImport = function(request)
  extensions.tech_terrainImporter.peaksAndRoadImport(request.peaks, request.roads, request.DOI, request.margin)
  log('I', logTag, 'Peaks and roads - imported.')
  request:sendACK('CompletedPeaksAndRoadImport')
end

-- Resets the terrain and roads created by a call to the terrain+roads or peaks+roads importer functions (see above).
M.handleResetTerrain = function(request)
  extensions.tech_terrainImporter.reset()
  log('I', logTag, 'Terrain and roads - reset.')
  request:sendACK('CompletedResetTerrain')
end

-- Opens/closes the world editor, by call.
M.handleOpenCloseWorldEditor = function(request)
  extensions.tech_terrainImporter.toggleWorldEditor(request.isOpen)
  log('I', logTag, 'Terrain and roads - toggle world editor open/close.')
  request:sendACK('CompletedOpenCloseWorldEditor')
end

-- Compute the vehicle space position of a sensor, given the local reference frame coefficients.
-- [This is used with the ADAS Sensor Configuration tool].
local function coeffs2PosVS(c, veh)
  local fwd, up = veh:getDirectionVector(), veh:getDirectionVectorUp()
  fwd:normalize()
  up:normalize()
  local right = fwd:cross(up)
  return c.x * fwd + c.y * right + c.z * up
end

-- Compute the vehicle space/world space frame of a sensor, given the local reference frame.
-- [This is used with the ADAS Sensor Configuration tool].
local function sensor2VS(dirLoc, upLoc, veh)
  local fwd, up = veh:getDirectionVector(), veh:getDirectionVectorUp()
  fwd:normalize()
  up:normalize()
  local right = fwd:cross(up)
  return vec3(fwd:dot(dirLoc), right:dot(dirLoc), up:dot(dirLoc)), vec3(fwd:dot(upLoc), right:dot(upLoc), up:dot(upLoc))
end

M.handleUnpackVehicleSensorConfiguration = function(request)
  local filepath, vid = request.filepath, request.vid
  local veh = scenetree.findObject(vid)
  if not veh then
    request:sendBNGError('Vehicle ' .. vid .. ' not found.')
    return false
  end

  local loadedJson = jsonReadFile(filepath)
  if not loadedJson then
    request:sendBNGError('File ' .. filepath .. ' not found.')
    return false
  end
  local sData = lpack.decode(loadedJson.data).sensors
  local numSensors = #sData
  for i = 1, numSensors do
    if sData[i].pos then
      sData[i].pos = coeffs2PosVS(sData[i].pos, veh)
      sData[i].dir, sData[i].up = sensor2VS(sData[i].dir, sData[i].up, veh)
    end
  end
  request:sendResponse({ type = 'UnpackVehicleSensorConfiguration', data = sData })
end

M.handleUnpackMapSensorConfiguration = function(request)
  local filepath = request.filepath
  local loadedJson = jsonReadFile(filepath)
  if not loadedJson then
    request:sendBNGError('File ' .. filepath .. ' not found.')
    return false
  end
  local sData = lpack.decode(loadedJson.data)
  request:sendResponse({ type = 'UnpackMapSensorConfiguration', data = sData })
end

M.handleGetEnvironmentPaths = function(request)
  request:sendResponse({
    type = 'GetEnvironmentPaths',
    home = FS:getGamePath(),
    user = FS:getUserPath()
  })
end

--platoon handler
M.handleLoadPlatoon = function(request)
  print("loading in techCore gelua")
  log('I', 'techCore', ' Received LoadPlatoon request')
  -- local leader = request.leaderID
  -- local vid = request.vid
  speed = request.speed
  
  
  for _, v in ipairs(getAllVehicles()) do
    if v:getName() == request.leaderID then
      leader = v:getID()
      print("leader: "..leader)
    elseif v:getName() == request.follower1ID then
      follower1 = v:getID()
      print("follower 1: "..follower1)
    elseif v:getName() == request.follower2ID then
      follower2 = v:getID()
      print("follower2: "..follower2)
    elseif v:getName() == request.follower3ID then
      follower3 = v:getID()
      print("follower3: "..follower3)
    end 

    print("name: "..v:getName())
    print("ID: "..v:getID())
  end
  mode = 1

  print("leader: "..leader.." follower1: "..follower1.."follower2: "..follower2.." speed: "..speed)
  extensions.tech_platoonFunctions.loadWithID(leader, follower1, speed)
  extensions.tech_platoonFunctions.joinWithID(leader, follower2, speed)
  if follower3 ~= nil then
    -- print("3rd vehicle")
    extensions.tech_platoonFunctions.joinWithID(leader, follower3, speed)
  else 
    -- print("no follower3: ")
  end
  extensions.tech_platoonFunctions.launchPlatoon(leader, mode, speed)
  request:sendACK('platoonLoaded')

end



M.handlePlatoonSpeed = function(request)
  print("loading in techCore gelua")
  log('I', 'techCore', ' Received PlatoonSpeed request')
  speed = request.speed
  
  
  for _, v in ipairs(getAllVehicles()) do
    if v:getName() == request.leaderID then
      leader = v:getID()
      print("leader: "..leader)
    end 

    print("name: "..v:getName())
    print("ID: "..v:getID())
  end
  mode = 1

  print("leader: "..leader.." speed: "..speed)
  extensions.tech_platoonFunctions.launchPlatoon(leader, mode, speed)
  request:sendACK('changedSpeed')

end

M.handleJoinPlatoon = function(request)
  print("joining in techCore gelua")
  log('I', 'techCore', ' Received joinPlatoon request')
  local speed = request.speed
  
  
  for _, v in ipairs(getAllVehicles()) do
    if v:getName() == request.leaderID then
      leader = v:getID()
    elseif v:getName() == request.vid then
      follower = v:getID()
    end 

    print("name: "..v:getName())
    print("ID: "..v:getID())
  end

  print("leader: "..leader.." follower: "..follower.." speed: "..speed)
  extensions.tech_platoonFunctions.joinWithID(leader, follower, speed)
  request:sendACK('platoonJoined')

end

M.handleJoinMiddlePlatoon = function(request)
  print("joining in techCore gelua")
  log('I', 'techCore', ' Received joinPlatoon request')
  local speed = request.speed
  
  
  for _, v in ipairs(getAllVehicles()) do
    if v:getName() == request.leaderID then
      leader = v:getID()
    elseif v:getName() == request.vPid then 
      relay = v:getID()
    elseif v:getName() == request.vid then
      follower = v:getID()
    end 

    print("name: "..v:getName())
    print("ID: "..v:getID())
  end

  print("leader: "..leader.." follower: "..follower.." speed: "..speed)
  extensions.tech_platoonFunctions.joinInBetween(leader, relay, follower, speed)
  request:sendACK('platoonJoined')

end

M.handleLeavePlatoon = function(request)
  print("leaving in techCore gelua")
  log('I', 'techCore', ' Received joinPlatoon request')
  
  
  for _, v in ipairs(getAllVehicles()) do
    if v:getName() == request.leaderID then
      leader = v:getID()
    elseif v:getName() == request.vid then
      follower = v:getID()
    end 

    print("name: "..v:getName())
    print("ID: "..v:getID())
  end

  print("leader: "..leader.." follower: "..follower)
  extensions.tech_platoonFunctions.leavePlatoon(leader, follower)
  request:sendACK('platoonLeft')

end


return M
