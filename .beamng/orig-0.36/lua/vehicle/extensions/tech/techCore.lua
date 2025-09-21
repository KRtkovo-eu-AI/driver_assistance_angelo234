-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local logTag = 'TechVE'
local M = {}

local tcom = require('tech/techCommunication')
local scriptai = require('scriptai')
local techUtils = require('tech/techUtils')

local sensorHandlers = {}

local server = nil
local clients = nil

local port = nil

local conSleep = 60

-- Helper functions

local function getVehicleState()
  local vehicleState = {
    time = obj:getSimTime(),
    pos = obj:getPosition(),
    dir = obj:getDirectionVector(),
    up = obj:getDirectionVectorUp(),
    vel = obj:getVelocity(),
    front = obj:getFrontPosition(),
    rotation = quat(obj:getRotation())
  }
  vehicleState['pos'] = {
    vehicleState['pos'].x,
    vehicleState['pos'].y,
    vehicleState['pos'].z
  }

  vehicleState['dir'] = {
    vehicleState['dir'].x,
    vehicleState['dir'].y,
    vehicleState['dir'].z
  }

  vehicleState['up'] = {
    vehicleState['up'].x,
    vehicleState['up'].y,
    vehicleState['up'].z
  }

  vehicleState['vel'] = {
    vehicleState['vel'].x,
    vehicleState['vel'].y,
    vehicleState['vel'].z
  }

  vehicleState['front'] = {
    vehicleState['front'].x,
    vehicleState['front'].y,
    vehicleState['front'].z
  }

  vehicleState['rotation'] = {
    vehicleState['rotation'].x,
    vehicleState['rotation'].y,
    vehicleState['rotation'].z,
    vehicleState['rotation'].w
  }

  return vehicleState
end

local function submitInput(inputs, key)
  local val = inputs[key]
  if val ~= nil then
    input.event(key, val, 1)
  end
end

local function getSensorData(request)
  local response, sensor_type, handler

  sensor_type = request['type']
  handler = sensorHandlers[sensor_type]
  if handler ~= nil then
    response = handler(request)
    return response
  end

  return nil
end

-- Sensors

sensorHandlers.Damage = function(request)
  local resp = {type = 'Damage'}
  resp['damage_ext'] = beamstate.damageExt
  resp['deform_group_damage'] = beamstate.deformGroupDamage
  resp['lowpressure'] = beamstate.lowpressure
  resp['damage'] = beamstate.damage
  resp['part_damage'] = beamstate.getPartDamageData()
  return resp
end

sensorHandlers.Electrics = function(request)
  local resp = {type = 'Electrics'}
  resp['values'] = electrics.values
  return resp
end

sensorHandlers.GForces = function(request)
  local resp = {type='GForces'}

  resp['gx'] = sensors.gx
  resp['gx2'] = sensors.gx2
  resp['gy'] = sensors.gy
  resp['gy2'] = sensors.gy2
  resp['gz'] = sensors.gz
  resp['gz2'] = sensors.gz2

  return resp
end

sensorHandlers.IMU = function(request)
  local name = request['name']
  local imu = imu.getIMU(name)
  return {
    name = imu.name,
    aX = imu.aX,
    aY = imu.aY,
    aZ = imu.aZ,
    gX = imu.gX,
    gY = imu.gY,
    gZ = imu.gZ
  }
end

sensorHandlers.State = function(request)
  local resp = {type = 'VehicleUpdate'}
  local vehicleState = getVehicleState()
  resp['state'] = vehicleState
  return resp
end

-- Exported functions

M.requestVehicleInfo = function()
  local info = {}
  info['port'] = port
  info['id'] = obj:getID()

  local cmd = 'extensions.hook("onVehicleInfoReady", ' .. tostring(obj:getID()) .. ', ' .. serialize(info) .. ')'
  obj:queueGameEngineLua(cmd)
end

local function startConnectionReload(ip, port)
  server = tcom.openServer(port, ip)
  local _
  _, port = server:getsockname()
  local set = tcom.newSet()
  set:insert(server)
  server = set
  clients = tcom.newSet()
end

M.startConnection = function(ip, skipServer)
  if skipServer then
    port = -1
  elseif server == nil then
    server = tcom.openServer(0, ip)
    local _
    _, port = server:getsockname()
    local set = tcom.newSet()
    set:insert(server)
    server = set
    clients = tcom.newSet()
  end
  local cmd = 'extensions.hook("onVehicleConnectionReady", ' .. tostring(obj:getID()) .. ', ' .. tostring(port) .. ')'
  obj:queueGameEngineLua(cmd)
end

-- Hooks

M.onDebugDraw = function()
  if server ~= nil then
    if conSleep <= 0 then
      conSleep = 60
      local newClients = tcom.checkForClients(server)
      for i = 1, #newClients do
        clients:insert(newClients[i])
        local ip, clientPort = newClients[i]:getpeername()
        log('I', logTag, 'Accepted new vehicle client: ' .. tostring(ip) .. '/' .. tostring(clientPort))
      end
    else
      conSleep = conSleep - 1
    end
  else
    return
  end

  while tcom.checkMessages(M, clients) do end
end

M.onSerialize = function()
  local data = {}
  if server ~= nil then
    local _, serverSocket = next(server)
    data.runningIP, data.runningPort = serverSocket:getsockname()
  end

  return data
end

M.onDeserialized = function(data)
  if data.runningIP ~= nil then
    startConnectionReload(data.runningIP, data.runningPort)
  end
end

-- Handlers

M.handleHello = function(request)
  local resp = {type = 'Hello', protocolVersion = tcom.protocolVersion}
  request:sendResponse(resp)
end

M.handleControl = function(request)
  submitInput(request, 'throttle')
  submitInput(request, 'steering')
  submitInput(request, 'brake')
  submitInput(request, 'parkingbrake')
  submitInput(request, 'clutch')

  local gear = request['gear']
  if gear ~= nil then
    drivetrain.shiftToGear(gear)
  end

  request:sendACK('Controlled')
end

M.handleSetShiftMode = function(request)
  drivetrain.setShifterMode(request['mode'])
  request:sendACK('ShiftModeSet')
end

M.handleSensorRequest = function(request)
  local sensorRequest, sensorData, data
  sensorData = {}
  sensorRequest = request['sensors']
  for k, v in pairs(sensorRequest) do
    data = getSensorData(v)
    if data == nil then
      log('E', logTag, 'Could not get data for sensor: ' .. k)
    end
    sensorData[k] = data
  end

  local response = {type = 'SensorData', data = sensorData}
  request:sendResponse(response)
end

M.handleSetColor = function(request)
  local cmd = 'Point4F(' .. request['r'] .. ', ' .. request['g'] .. ', ' .. request['b'] .. ', ' .. request['a'] .. ')'
  cmd = 'getObjectByID(' .. obj:getID() .. '):setColor(' .. cmd .. ')'
  obj:queueGameEngineLua(cmd)
  request:sendACK('ColorSet')
end

M.handleSetVelocity = function(request)
  thrusters.applyVelocity(obj:getDirectionVector() * request['velocity'], request['dt'])
  request:sendACK('VelocitySet')
end

M.handleSetAiMode = function(request)
  ai.setMode(request['mode'])
  ai.stateChanged()
  request:sendACK('AiModeSet')
end

M.handleSetAiLine = function(request)
  local nodes = request['line']
  local fauxPath = {}
  local cling = request['cling']
  local z = 0
  local speedList = {}
  for idx, n in ipairs(nodes) do
    local pos = vec3(n['pos'][1], n['pos'][2], 10000)
    if cling then
      z = techUtils.getSurfaceHeight(pos)
    else
      z = n['pos'][3]
    end
    pos.z = z
    local fauxNode = {x=pos.x, y=pos.y, z=pos.z, v=n['speed'], radius=0, radiusOrig=0}
    table.insert(speedList, n['speed'])
    table.insert(fauxPath, fauxNode)
  end

  local arg = {
    script = fauxPath,
    wpSpeeds = speedList
  }
  ai.driveUsingPath(arg)
  ai.stateChanged()
  request:sendACK('AiLineSet')
end

M.handleSetAiScript = function(request)
  local script = request['script']
  local cling = request['cling']

  if cling then
    for i, v in ipairs(script) do
      v.z = techUtils.getSurfaceHeight(v)
    end
  end

  ai.startFollowing(script, 0, 0, 'never')

  ai.stateChanged()
  request:sendACK('AiScriptSet')
end

M.handleExecuteScript = function(request)
  local script = request['script']
  local cling = request['cling']
  local noReset = request['noReset']
  if cling then
    for i, v in ipairs(script) do
      v.z = techUtils.getSurfaceHeight(v)
    end
  end

  script.startDelay = request['startDelay']
  if noReset == true then
    ai.startFollowing(script, 0, 1, "noReset")
  else
    ai.startFollowing(script)
  end
  ai.stateChanged()
  request:sendACK('CompletedExecuteScript')
end

M.handleGetInitialSpawnPositionOrientation = function(request)
  local pos, rot = scriptai.getInitialSpawnPositionOrientation(request['script'])
  local pose = { pos = { x = pos.x, y = pos.y, z = pos.z }, rot = { x = rot.x, y = rot.y, z = rot.z, w = rot.w } }
  local resp = { type = 'GetInitialSpawnPositionOrientation', data = pose }
  request:sendResponse(resp)
end

M.handleSetAiSpeed = function(request)
  ai.setSpeedMode(request['mode'])
  ai.setSpeed(request['speed'])
  ai.stateChanged()
  request:sendACK('AiSpeedSet')
end

M.handleSetAiTarget = function(request)
  local targetName = request['target']
  obj:queueGameEngineLua('scenetree.findObjectById(' .. obj:getID() .. '):queueLuaCommand("ai.setTargetObjectID(" .. scenetree.findObject(\'' .. targetName .. '\'):getID() .. ")")')
  request:sendACK('AiTargetSet')
end

M.handleSetAiWaypoint = function(request)
  local targetName = request['target']
  ai.setTarget(targetName)
  ai.stateChanged()
  request:sendACK('AiWaypointSet')
end

M.handleSetAiSpan = function(request)
  if request['span'] then
    ai.spanMap(0)
  else
    ai.setMode('disabled')
  end
  ai.stateChanged()
  request:sendACK('AiSpanSet')
end

M.handleSetAiAggression = function(request)
  local aggr = request['aggression']
  ai.setAggression(aggr)
  ai.stateChanged()
  request:sendACK('AiAggressionSet')
end

M.handleStartRecording = function(request)
  ai.startRecording()
  request:sendACK('CompletedStartRecording')
end

M.handleStopRecording = function(request)
  local script = ai.stopRecording()
  jsonWriteFile(request['filename'], script, true)
  request:sendACK('CompletedStopRecording')
end

M.handleSetDriveInLane = function(request)
  ai.driveInLane(request['lane'])
  ai.stateChanged()
  request:sendACK('AiDriveInLaneSet')
end

M.handleSetLights = function(request)
  local leftSignal = request['leftSignal']
  local rightSignal = request['rightSignal']
  local hazardSignal = request['hazardSignal']
  local fogLights = request['fogLights']
  local headLights = request['headLights']
  local lightBar = request['lightBar']

  local state = electrics.values

  if headLights ~= nil and state.lights_state ~= headLights then
    electrics.setLightsState(headLights)
  end

  if hazardSignal ~= nil then
    if hazardSignal == true then
      hazardSignal = 1
    end
    if hazardSignal == false then
      hazardSignal = 0
    end
    if state.hazard_enabled ~= hazardSignal then
      leftSignal = nil
      rightSignal = nil
      electrics.toggle_warn_signal()
    end
  end

  if leftSignal ~= nil then
    if leftSignal == true then
      leftSignal = 1
    end
    if leftSignal == false then
      leftSignal = 0
    end
    if state.signal_left_input ~= leftSignal then
      electrics.toggle_left_signal()
    end
  end

  if rightSignal ~= nil then
    if rightSignal == true then
      rightSignal = 1
    end
    if rightSignal == false then
      rightSignal = 0
    end
    if state.signal_right_input ~= rightSignal then
      electrics.toggle_right_signal()
    end
  end

  if fogLights ~= nil and state.fog ~= fogLights then
    electrics.set_fog_lights(fogLights)
  end

  if lightBar ~= nil then
    if state.lightbar ~= lightBar then
      electrics.set_lightbar_signal(lightBar)
    end
  end

  request:sendACK('LightsSet')
end

M.handleQueueLuaCommandVE = function(request)
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
      request:sendResponse({type ='ExecutedLuaChunkVE', resp = tostring(err)})
    end
  else
    request:sendACK('ExecutedLuaChunkVE')
  end
end

M.handleAddIMUPosition = function(request)
  local name = request['name']
  local pos = request['pos']
  pos = vec3(pos[1], pos[2], pos[3])
  local debug = request['debug']

  if imu == nil then
    extensions.load('imu')
  end

  imu.addIMU(name, pos, debug)
  request:sendACK('IMUPositionAdded')
end

M.handleAddIMUNode = function(request)
  local name = request['name']
  local node = request['node']
  local debug = request['debug']

  if imu == nil then
    extensions.load('imu')
  end

  imu.addIMUAtNode(name, node, debug)
  request:sendACK('IMUNodeAdded')
end

M.handleRemoveIMU = function(request)
  local imu = imu.removeIMU(request['name'])
  if imu ~= nil then
    request:sendACK('IMURemoved')
  else
    request:sendBNGValueError('Unknown IMU: ' .. tostring(request['name']))
  end
end

M.handleApplyVSLSettingsFromJSON = function(request)
  extensions.vehicleStatsLogger.applySettingsFromJSON(request['fileName'])
  request:sendACK('AppliedVSLSettings')
end

M.handleWriteVSLSettingsToJSON = function(request)
  extensions.vehicleStatsLogger.writeSettingsToJSON(request['fileName'])
  request:sendACK('WroteVSLSettingsToJSON')
end

M.handleStartVSLLogging = function(request)
  extensions.vehicleStatsLogger.settings.outputDir = request['outputDir']
  extensions.vehicleStatsLogger.startLogging()
  request:sendACK('StartedVSLLogging')
end

M.handleStopVSLLogging = function(request)
  extensions.vehicleStatsLogger.stopLogging()
  request:sendACK('StoppedVSLLogging')
end

M.handlePollAdvancedImuVE = function(request)
  local name = request['name']
  local sensorId = request['sensorId']
  if sensorId ~= nil then
    local readings = extensions.tech_advancedIMU.getAdvancedIMUReading(sensorId)
    if readings ~= nil then
      local resp = { type = 'PollAdvancedImuVE', data = readings }
      request:sendResponse(resp)
      return true
    end
  end

  -- The sensor was not found, or the readings did not exist, so send an empty response.
  local resp = {type = 'PollAdvancedImuVE', data = {} }
  log('I', logTag, 'WARNING: Advanced IMU sensor not found')
  request:sendResponse(resp)
end

M.handlePollGPSVE = function(request)
  local name = request['name']
  local sensorId = request['sensorId']
  if sensorId ~= nil then
    local readings = extensions.tech_GPS.getGPSReading(sensorId)
    if readings ~= nil then
      local resp = { type = 'PollGPSVE', data = readings }
      request:sendResponse(resp)
      return true
    end
  end

  -- The sensor was not found, or the readings did not exist, so send an empty response.
  local resp = {type = 'PollGPSVE', data = {} }
  log('I', logTag, 'WARNING: GPS sensor not found')
  request:sendResponse(resp)
end

M.handlePollPowertrainVE = function(request)
  local name = request['name']
  local sensorId = request['sensorId']
  if sensorId ~= nil then
    local readings = extensions.tech_powertrainSensor.getPowertrainReading(sensorId)
    if readings ~= nil then
      local resp = { type = 'PollPowertrainVE', data = readings }
      request:sendResponse(resp)
      return true
    end
  end

  -- The sensor was not found, or the readings did not exist, so send an empty response.
  local resp = {type = 'PollPowertrainVE', data = {} }
  log('I', logTag, 'WARNING: Powertrain sensor not found')
  request:sendResponse(resp)
end

M.handlePollMeshVE = function(request)
  local name = request['name']
  local sensorId = request['sensorId']
  if sensorId ~= nil then
    local readings = extensions.tech_mesh.getMeshReading(sensorId)
    if readings ~= nil then
      local resp = { type = 'PollMeshVE', data = readings }
      request:sendResponse(resp)
      return true
    end
  end

  -- The sensor was not found, or the readings did not exist, so send an empty response.
  local resp = {type = 'PollMeshVE', data = {} }
  log('I', logTag, 'WARNING: Mesh sensor not found')
  request:sendResponse(resp)
end

M.handlePollIdealRADARVE = function(request)
  local name = request['name']
  local sensorId = request['sensorId']
  if sensorId ~= nil then
    local readings = extensions.tech_idealRADARSensor.getIdealRADARReading(sensorId)
    if readings ~= nil then
      local resp = { type = 'PollIdealRADARVE', data = readings }
      request:sendResponse(resp)
      return true
    end
  end

  -- The sensor was not found, or the readings did not exist, so send an empty response.
  local resp = {type = 'PollIdealRADARVE', data = {} }
  log('I', logTag, 'WARNING: Ideal RADAR sensor not found')
  request:sendResponse(resp)
end

M.handlePollRoadsSensorVE = function(request)
  local name = request['name']
  local sensorId = request['sensorId']
  if sensorId ~= nil then
    local readings = extensions.tech_roadsSensor.getRoadsSensorReading(sensorId)
    if readings ~= nil then
      local resp = { type = 'PollRoadsSensorVE', data = readings }
      request:sendResponse(resp)
      return true
    end
  end

  -- The sensor was not found, or the readings did not exist, so send an empty response.
  local resp = {type = 'PollRoadsSensorVE', data = {} }
  log('I', logTag, 'WARNING: Roads sensor not found')
  request:sendResponse(resp)
end

M.handleRecover = function(request)
  recovery.startRecovering()
  recovery.stopRecovering()

  request:sendACK('Recovered')
end

M.handleGetCenterOfGravity = function(request)
  local withoutWheels = request['withoutWheels'] or false
  local cog = obj:calcCenterOfGravity(withoutWheels)
  local response = { data = { cog.x, cog.y, cog.z } }
  request:sendResponse(response)
end

M.handleDeflateTire = function(request)
  beamstate.deflateTires(request['wheelId'])
  request:sendACK('CompletedDeflateTire')
end

--- ACC handler
M.handleLoadACC = function(request)
  extensions.tech_ACC.loadAccWithID()
  request:sendACK('ACCloaded')
end

M.handleUnloadACC = function(request)
  extensions.tech_ACC.unloadACC()
  request:sendACK('ACCunloaded')
end

--- vehicle platoon handler
M.handleLoadPlatoon = function(request)
  extensions.tech_platooning.loadWithIDPlatoon()
  request:sendACK('Platoonloaded')
end

M.handleFormPlatoon = function(request)
  extensions.tech_platooning.formPlatoon()
  request:sendACK('Platoonformed')
end

M.handleJoinPlatoon = function(request)
  extensions.tech_platooning.joinPlatoon()
  request:sendACK('Platoonjoined')
end

M.handleEndPlatoon = function(request)
  extensions.tech_platooning.endPlatoon()
  request:sendACK('Platoonended')
end

M.handleLeavePlatoon = function(request)
  extensions.tech_platooning.leavePlatoon()
  request:sendACK('Platoonleft')
end

M.handleReassignLeaderNFSA = function(request)
  extensions.tech_platooning.reassignLeaderNFSA()
  request:sendACK('LeaderReassigned')
end

M.handleLeaderExitPlatoon = function(request)
  extensions.tech_platooning.leaderExitPlatoon()
  request:sendACK('leaderExitPlatoon')
end

M.handlUpdateLeaderToFollow = function(request)
  extensions.tech_platooning.updateLeaderToFollow()
  request:sendACK('updatedLeader')
end

M.handleStartCosimulation = function(request)
  local cData = {{
    signalsTo = request.signalsTo, signalsFrom = request.signalsFrom,
    sensorMap = request.sensorMap,
    time3rdParty = request.time3rdParty, pingTime = request.pingTime,
    udpSendPort = request.udpSendPort, udpReceivePort = request.udpReceivePort,
    udpSendIP = request.udpSendIP, udpReceiveIP = request.udpReceiveIP
  }}
  controller.loadControllerExternal('tech/cosimulationCoupling', 'cosimulationCoupling', lpack.encode(cData))

  request:sendACK('CosimulationStarted')
end

M.handleStopCosimulation = function(request)
  controller.getController('cosimulationCoupling').stop()
  controller.unloadControllerExternal('cosimulationCoupling')
  request:sendACK('CosimulationStopped')
end

M.handleAttachCouplers = function(request)
  beamstate.attachCouplers(request.tag)
end

M.handleDetachCouplers = function(request)
  beamstate.detachCouplers(request.tag, request.forceLocked, request.forceWelded)
end

M.handleToggleCouplers = function(request)
  beamstate.toggleCouplers(request.tag, request.forceLocked, request.forceWelded, request.forceAutoCoupling)
end

return M
