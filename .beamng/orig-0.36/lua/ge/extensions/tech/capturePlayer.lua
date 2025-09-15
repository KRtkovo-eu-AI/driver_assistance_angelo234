-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}


local logTag = 'CapturePlayer'

local BLOCKING_CALLS = {
  ['LoadScenario'] = 'MapLoaded',
  ['RestartScenario'] = 'ScenarioRestarted',
  ['StartScenario'] = 'ScenarioStarted',
  ['StopScenario'] = 'ScenarioStopped',
  ['StartVehicleConnection'] = 'StartVehicleConnection',
  ['GetCurrentVehicles'] = 'GetCurrentVehicles',
  ['SpawnVehicle'] = 'VehicleSpawned',
  ['Step'] = 'Stepped'
}

-- Some BeamNG requests/responses are needed for us to properly play the capture (the vehicle IDs, for example).
-- These store the handlers. The handlers are defined below in the file.
local RequestSync = {}
local ResponseSync = {}

-- For some responses (camera data for example), we want to convert the data into a nice format.
-- These are the handlers which convert the data to a better readable format (defined below).
local ResponseCallbacks = {}

local responsesFile = nil
local captureState = {
  lastVid = nil,
  portToVid = {},
  sensors = {}
}

local ffi = require('ffi')
local captureBuffer = require('string.buffer').new()
local shmemManager = Research.SharedMemoryManager.getInstance()
local pcdLib = require('tech/pcdLib')

local cos = math.cos
local sin = math.sin

M.dependencies = {'tech_techCore', 'tech_techCapture', 'core_jobsystem'}

local function getAttachmentFilename(name, id)
  if not responsesFile then return end

  local responsesDir, filename, _ = path.split(responsesFile)
  return responsesDir .. filename .. '_' .. name .. '_' .. tostring(id)
end

local function parseIntermediate(file, output)
  -- state machine
  local HEADER, TIMESTAMP, CONTEXT, PAYLOAD = -1, 0, 1, 2
  local NUM_STATES = 3
  local state = HEADER
  local ctx, timestamp

  local line = file:read()
  while line do
    if #line == 0 then
      return
    end
    if state == CONTEXT then
      ctx = line
    elseif state == TIMESTAMP then
      timestamp = tonumber(line)
    elseif state == PAYLOAD then
      table.insert(output, {ctx = ctx, timestamp = timestamp, payload = line})
    end

    state = (state + 1) % NUM_STATES
    line = file:read()
  end
end

local function mergeCaptures(captureName, captureType, removeIntermediates)
  if captureType == nil then captureType = 'REQUEST' end
  if removeIntermediates == nil then removeIntermediates = true end

  local files = tech_techCapture.getAllRelatedFiles(captureName, false, true)
  files = tech_techCapture.filterFilesByHeader(files, captureType, 'INTERMEDIATE')

  if #files == 0 then
    log('E', logTag, 'No files found for input filename ' .. captureName .. '.')
    return
  end

  local messages = {}
  for _, currFilename in ipairs(files) do
    log('D', logTag, 'Merging ' .. currFilename .. '.')
    local currFile, err = io.open(currFilename, 'r')
    if currFile == nil then
      log('E', logTag, 'Couldn\'t open ' .. currFile .. ' for reading. Original error: ' .. err)
    end
    parseIntermediate(currFile, messages)
  end

  local function compare(a, b)
    if a.timestamp ~= b.timestamp then
      return a.timestamp < b.timestamp
    end
    return a.ctx < b.ctx
  end
  table.sort(messages, compare) -- can be optimized using k-way merge, we're sorting sorted arrays

  local dirname, baseFilename, _ = path.splitWithoutExt(captureName)
  if dirname == nil then dirname = '' end
  baseFilename = baseFilename:gmatch("([^%.]+)")()
  local outputFilename = dirname .. '/' .. baseFilename .. '.log'
  local outputFile, err = io.open(outputFilename, 'w')
  if outputFile == nil then
    log('E', logTag, 'Couldn\'t open ' .. outputFilename .. ' for writing. Original error: ' .. err)
    return
  else
    if captureType == 'REQUEST' then
      outputFile:write('TECH CAPTURE v1 COMPLETE\n')
    elseif captureType == 'RESPONSE' then
      outputFile:write('TECH RESPONSE v1 COMPLETE\n')
    end
  end

  for _, message in ipairs(messages) do
    local line = message.timestamp .. '\n' .. message.ctx .. '\n' .. message.payload .. '\n'
    outputFile:write(line)
  end
  outputFile:flush()
  outputFile:close()

  if removeIntermediates then
    for _, file in ipairs(files) do
      FS:removeFile(file)
    end
  end

  log('I', logTag, 'Successfully merged into ' .. outputFilename .. '.')
  return outputFilename
end

local function waitForResponse(job, request, type)
  while true do
    local response = request.response
    if response == nil then
      job.yield()
    elseif type ~= nil and response.type ~= type then
      job.yield()
    else
      return response
    end
  end
end

local function syncRequest(request)
  local func = RequestSync[request.type]
  if func then
    func(request)
  end
end

local function syncResponse(response)
  local func = ResponseSync[response.type]
  if func then
    func(response)
  end
end

local function processRequest(job, ctx, payload, forceWait)
  log('D', logTag, 'Processing ' .. ctx .. ' [' .. payload.type .. ']')
  if ctx == 'GE' then
    syncRequest(payload)
    local callback = ResponseCallbacks[payload.type]
    local request, processMore = tech_techCapture.injectMessage(payload, callback)
    if not processMore then
      job.yield()
    end
    local waitFor = BLOCKING_CALLS[request.type]
    if not waitFor and not forceWait then
      return
    end
    local response = waitForResponse(job, request, waitFor)
    syncResponse(response)
  else
    local port = tonumber(ctx)
    if not captureState.portToVid[port] then
      captureState.portToVid[port] = captureState.lastVid
    end
    local vid = captureState.portToVid[port]
    local serializedData = string.format("tech_techCapture.injectMessage(lpack.decode(%q))", lpack.encode(payload))
    be:queueObjectLua(vid, serializedData)
  end
end

local function techCaptureJob(job, args)
  local err
  local inputFile = io.open(args.inputFilename, 'r')
  if inputFile == nil then
    log('E', logTag, 'Couldn\'t open ' .. completeInputFilename .. ' for reading. Original error: ' .. err)
    return
  end

  local dtBetweenRequests = args.dtBetweenRequests -- -1 = use timestamps diff from the file; nil = don't wait (except blocking requests); >0 = wait for n seconds

  -- state machine
  local HEADER, TIMESTAMP, CONTEXT, PAYLOAD = -1, 0, 1, 2
  local NUM_STATES = 3
  local state = HEADER
  local ctx, captureTimestamp
  local lastRealTimestamp = os.clockhp()
  local lastCaptureTimestamp = nil

  local line = inputFile:read()
  while line do
    local payload = nil
    if #line == 0 then
      return
    end
    if state == HEADER and line ~= 'TECH CAPTURE v1 COMPLETE' then
      log('E', logTag, 'Header mismatch, got ' .. line .. '.')
      return
    end
    if state == CONTEXT then
      ctx = line
    elseif state == TIMESTAMP then
      captureTimestamp = tonumber(line)
      local realTimestamp = os.clockhp()

      if dtBetweenRequests == -1 and lastCaptureTimestamp ~= nil then
        local actualWait = realTimestamp - lastRealTimestamp
        local expectedWait = captureTimestamp - lastCaptureTimestamp
        local remainingWait = expectedWait - actualWait
        if remainingWait > 0 then
          log('D', logTag, 'Sleeping for ' .. tostring(remainingWait) .. 's.')
          job.sleep(remainingWait)
        end
      end
      lastRealTimestamp = realTimestamp
      lastCaptureTimestamp = captureTimestamp
    elseif state == PAYLOAD then
      payload = jsonDecode(line)
    end

    line = inputFile:read()
    if state == PAYLOAD then
      local eof = line == nil or #line == 0

      processRequest(job, ctx, payload, eof)
      if dtBetweenRequests and dtBetweenRequests > 0 then
        job.sleep(dtBetweenRequests)
      end
    end

    state = (state + 1) % NUM_STATES
  end

  if state ~= TIMESTAMP then
    log('W', logTag, 'Incomplete capture detected, expected state ' .. tostring(state))
  end

  job.yield()
  -- last request is forcefully waited for, so we know we can cleanup
  log('I', logTag, 'Finished playing ' .. args.inputFilename .. '.')
  tech_techCapture.disableResponseCapture()
  if args.mergeResponses and responsesFile then
    mergeCaptures(responsesFile .. '.log', 'RESPONSE', true)
  end
  if args.quitOnEnd then
    quit()
  end
end

local function checkCaptureRequestFile(inputFilename)
  local captureType, captureMerged = tech_techCapture.getCaptureTypeFromFile(inputFilename)
  if captureType == nil then return nil end
  if captureType ~= 'REQUEST' then
    log('E', logTag, inputFilename .. ' is not a request file but was supplied to function that loads requests.')
    return nil
  end

  if captureMerged == 'COMPLETE' then
    return inputFilename
  end
  if captureMerged == 'INTERMEDIATE' then -- needs to be merged
    return mergeCaptures(inputFilename, 'REQUEST', true)
  end

  return nil
end

local function playCapture(inputFilename, outputPrefix, dtBetweenRequests, mergeResponses, quitOnEnd)
  if dtBetweenRequests == nil then
    dtBetweenRequests = -1 -- by default, emulate timestamps from the request file
  end
  if mergeResponses == nil then
    mergeResponses = true
  end
  if quitOnEnd == nil then
    quitOnEnd = false
  end

  local completeInputFilename = checkCaptureRequestFile(inputFilename)
  if completeInputFilename == nil then
    log('E', logTag, 'Cannot parse ' .. inputFilename .. '. Check if it exists and is a valid tech capture file.')
    return
  end

  log('I', logTag, 'Playing capture ' .. completeInputFilename .. '.')
  captureState = {
    lastVid = nil,
    portToVid = {},
    sensors = {}
  }

  if outputPrefix then
    if outputPrefix == true then -- automatic name generation if name not provided (but we want to record)
      outputPrefix = inputFilename:gsub("%.log$", "") .. '_response'
    end
    outputPrefix = outputPrefix:gsub("%.log$", "") -- if user included the extension, remove it
    responsesFile = outputPrefix
    tech_techCapture.enableResponseCapture(outputPrefix)
  end
  local args = {
    inputFilename = completeInputFilename,
    dtBetweenRequests = dtBetweenRequests,
    mergeResponses = mergeResponses,
    quitOnEnd = quitOnEnd
  }
  core_jobsystem.create(techCaptureJob, 0.001, args)
end

local function onInit()
  setExtensionUnloadMode(M, 'manual')

  ffi.cdef([[
  struct radar_return_t {
    float range;
    float dopplerVelocity;
    float azimuth;
    float elevation;
    float radarCrossSection;
    float signalToNoiseRatio;
    float facingFactor;
  };
  ]])
end

RequestSync.OpenLidar = function(request)
  captureState.sensors[request.name] = {
    isStreaming = request.isStreaming,
  }

  if request.useSharedMemory then
    captureState.sensors[request.name].colourShmemSize = request.colourShmemSize
    captureState.sensors[request.name].colourShmemHandle = request.colourShmemHandle
    captureState.sensors[request.name].pointCloudShmemSize = request.pointCloudShmemSize
    captureState.sensors[request.name].pointCloudShmemHandle = request.pointCloudShmemHandle
  end

  if request.vid then
    local veh = scenetree.findObject(request.vid)
    captureState.sensors[request.name].vid = veh:getID()
  end
end

RequestSync.OpenCamera = function(request)
  captureState.sensors[request.name] = {
    isStreaming = request.isStreaming,
    size = request.size,
    renderColours = request.renderColours,
    renderAnnotations = request.renderAnnotations,
    renderDepth = request.renderDepth,
  }
  if request.useSharedMemory then
    if request.renderColours then
      captureState.sensors[request.name].colourShmemName = request.colourShmemName
      captureState.sensors[request.name].colourShmemSize = request.colourShmemSize
    end
    if request.renderAnnotations then
      captureState.sensors[request.name].annotationShmemName = request.annotationShmemName
      captureState.sensors[request.name].annotationShmemSize = request.annotationShmemSize
    end
    if request.renderDepth then
      captureState.sensors[request.name].depthShmemName = request.depthShmemName
      captureState.sensors[request.name].depthShmemSize = request.depthShmemSize
    end
  end
end

RequestSync.OpenUltrasonic = function(request)
  captureState.sensors[request.name] = {
    shmemHandle = request.shmemHandle,
    shmemSize = request.shmemSize,
    isStreaming = request.isStreaming,
  }

  if request.vid then
    local veh = scenetree.findObject(request.vid)
    captureState.sensors[request.name].vid = veh:getID()
  end
end

RequestSync.OpenRadar = function(request)
  captureState.sensors[request.name] = {
    shmemName = request.shmemHandle,
    shmemName2 = request.shmemHandle2,
    shmemSize = request.shmemSize,
    isStreaming = request.isStreaming
  }

  if request.vid then
    local veh = scenetree.findObject(request.vid)
    captureState.sensors[request.name].vid = veh:getID()
  end
end

ResponseSync.StartVehicleConnection = function(response)
  local veh = scenetree.findObject(response.vid)
  captureState.lastVid = veh:getID()
end

local function saveBitmap(size, data, filename)
  local bitmap = GBitmap()
  bitmap:init(size[1], size[2], true)
  bitmap:fromBuffer(data)
  bitmap:saveFile(filename)
  log('I', logTag, 'Saved bitmap to ' .. filename .. '.')
end

local function depthToRGBA(sensorData)
  local BYTES_IN_POINT = 4
  local bytes = #sensorData
  local points = bytes / BYTES_IN_POINT

  local depthFloat = ffi.new('float[' .. tostring(points) .. ']')
  ffi.copy(depthFloat, sensorData, bytes)

  sensorData:reset()
  local alpha = string.char(255)
  for i = 0, points - 1 do
    local value = round(clamp(depthFloat[i] * 255.0, 0, 255))
    local char = string.char(value)
    sensorData:put(string.rep(char, 3))
    sensorData:put(alpha)
  end
  return sensorData
end

ResponseCallbacks.PollCamera = function(request, response)
  local name = request.name
  local cam = captureState.sensors[name]

  if cam.renderColours then
    local binary = response.data.colour
    if cam.colourShmemName then
      shmemManager:readSharedMemory(cam.colourShmemName, captureBuffer)
      binary = captureBuffer
    end
    local filename = getAttachmentFilename(name, request._id) .. '_colour.png'
    saveBitmap(cam.size, binary, filename)
    response.data.colour = filename
  end
  if cam.renderAnnotations then
    local binary = response.data.annotation
    if cam.annotationShmemName then
      shmemManager:readSharedMemory(cam.annotationShmemName, captureBuffer)
      binary = captureBuffer
    end
    local filename = getAttachmentFilename(name, request._id) .. '_annotation.png'
    saveBitmap(cam.size, binary, filename)
    response.data.annotation = filename
  end
  if cam.renderDepth then
    local binary = response.data.depth
    if cam.depthShmemName then
      shmemManager:readSharedMemory(cam.depthShmemName, captureBuffer)
      binary = captureBuffer
    end
    local depthRGB = depthToRGBA(binary)
    local filename = getAttachmentFilename(name, request._id) .. '_depth.png'
    saveBitmap(cam.size, depthRGB, filename)
    response.data.depth = filename
  end
end

ResponseCallbacks.PollLidar = function(request, response)
  local name = request.name
  local lidar = captureState.sensors[name]

  local binary = response.data.pointCloud
  if lidar.pointCloudShmemHandle then
    local numBytes = response.data.points
    shmemManager:readSharedMemory(lidar.pointCloudShmemHandle, captureBuffer, numBytes)
    binary = captureBuffer
  end

  local pcd = pcdLib.newPcd()
  if lidar.vid then
    local veh = getObjectByID(lidar.vid)
    pcd:setViewpoint(veh:getPosition(), veh:getRefNodeRotation())
  end

  pcd:addField('x', 4, 'float')
  pcd:addField('y', 4, 'float')
  pcd:addField('z', 4, 'float')
  pcd:setData(binary, #binary)

  local filename = getAttachmentFilename(name, request._id) .. '_points.pcd'
  pcd:save(filename)
  response.data.pointCloud = filename
end

local function radarReturnsToPointcloud(sensorData)
  local returnSize = ffi.sizeof('struct radar_return_t')
  local points = #sensorData / returnSize
  if points ~= math.floor(points) then
    log('E', logTag, 'Number of points ' .. tostring(points) .. ' is not an integer!')
    return nil
  end
  local sizeStr = '[' .. tostring(points) .. ']'
  local returnsFloat = ffi.new('struct radar_return_t' .. sizeStr)
  ffi.copy(returnsFloat, sensorData, #sensorData)

  local pointcloud = ffi.new('struct __luaVec3_t' .. sizeStr)
  for i = 0, points - 1 do
    local point = pointcloud[i]
    local radarRet = returnsFloat[i]

    point.x = radarRet.range * cos(radarRet.azimuth) * cos(radarRet.elevation)
    point.y = radarRet.range * sin(radarRet.azimuth) * cos(radarRet.elevation)
    point.z = radarRet.range * sin(radarRet.elevation)
  end
  return pointcloud
end

ResponseCallbacks.PollRadar = function(request, response)
  local name = request.name
  if response.data == nil or #response.data == 0 then
    log('E', logTag, 'Empty radar data received.')
    return
  end

  local radar = captureState.sensors[name]
  local pointcloud = radarReturnsToPointcloud(response.data)
  local pcd = pcdLib.newPcd()
  if radar.vid then
    local veh = getObjectByID(radar.vid)
    pcd:setViewpoint(veh:getPosition(), veh:getRefNodeRotation())
  end

  pcd:addField('x', 4, 'float')
  pcd:addField('y', 4, 'float')
  pcd:addField('z', 4, 'float')
  local points = #response.data / ffi.sizeof('struct radar_return_t')
  pcd:setData(pointcloud, points * ffi.sizeof('struct __luaVec3_t'))

  local filename = getAttachmentFilename(name, request._id) .. '_points.pcd'
  pcd:save(filename)
  response.data = filename
end

M.onInit = onInit
M.onReset = onInit
M.mergeCaptures = mergeCaptures
M.playCapture = playCapture

return M