-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}


local buffer = require('string.buffer')

-- Vlua ad-hoc request data.
local requestId = -1                            -- The counter for unique vlua ad-hoc request Id numbers.
local adHocVluaRequests = {}                    -- The collection of pending ad-hoc requests for vlua sensors.

-- Vlua sensor readings data.
local advancedIMULastRawReadings = {}           -- The most-recently-read Advanced IMU data (this is a table).
local GPSLastRawReadings = {}                   -- The most-recently-read GPS data (this is a table).
local powertrainLastRawReadings = {}            -- The most-recently-read Powertrain data (this is a table).
local idealRADARLastRawReadings = {}            -- The most-recently-read ideal RADAR data (this is a table).
local roadsSensorLastRawReadings = {}           -- The most-recently-read roads data (this is a table).
local isVehicleFeedingComplete = false          -- Marks when the vehicle feeding has finished.

-- Buffers used for fast sensor polling.
local cameraBufferColors = {}                   -- Camera sensor.
local cameraBufferAnnotations = {}
local cameraBufferDepth = {}
local lidarBufferPoints = {}                    -- LiDAR sensor.
local lidarBufferColors = {}
local radarBufferPoints = {}                    -- RADAR sensor.
local radarBufferPPI = {}
local radarBufferRangeDoppler = {}

-- Raw LiDAR sensor structures.
local rawLidarData = {}                         -- Stores the data for the RAW Lidar sensors (indexed by unique sensor Id).

-- Ultrasonic sensor visualisation data/parameters.
local visualisedUltrasonicSensors = {}
local usParameters = {
  pulseWidthDispersion = 0.1,                   -- The rate of longitudinal growth of the pulse (width). Used for the ultrasonic sensor visualisation.
  minPulseWidth = 0.1,                          -- The minimum possible displayed pulse width. Used for the ultrasonic sensor visualisation.
  maxPulseWidth = 0.25,                         -- The maximum possible displayed pulse width. Used for the ultrasonic sensor visualisation.
  minAlpha = 0.02,                              -- The smallest possible displayed alpha channel value. Used for the ultrasonic sensor visualisation.
  animationPeriod = 1.50,                       -- The animation wave period length (in seconds). Used for the ultrasonic sensor visualisation.
  animationSpeed = 6.0                          -- The animation wave speed (in m/s). Used for the ultrasonic sensor visualisation.
}

local function unpack_float(b4, b3, b2, b1)
  local sign = b1 > 0x7F and -1 or 1
  local expo = (b1 % 0x80) * 0x2 + math.floor(b2 / 0x80)
  local mant = ((b2 % 0x80) * 0x100 + b3) * 0x100 + b4
  if mant == 0 and expo == 0 then
    return sign * 0
  elseif expo == 0xFF then
    return mant == 0 and sign * math.huge or 0/0
  else
    return sign * math.ldexp(1 + mant / 0x800000, expo - 0x7F)
  end
end

-- Rotates vector v around unit axis k, by angle theta (in radians).
-- [This function uses the standard Rodrigues formula].
local function rotateVecAroundAxis(v, k, theta)
  local c = math.cos(theta)
  return v * c + k:cross(v) * math.sin(theta) + k * k:dot(v) * (1.0 - c)
end

-- Computes the (small) angle between two unit vectors, in radians.
local function angleBetweenVecsNorm(a, b) return math.acos(a:dot(b)) end

-- Computes the (small) angle between two vectors of arbitrary length, in radians.
local function angleBetweenVecs(a, b) return angleBetweenVecsNorm(a:normalized(), b:normalized()) end

local function getUniqueRequestId()
  requestId = requestId + 1
  return requestId
end

local function doesSensorExist(sensorId)
  return Research.SensorManager.doesSensorExist(sensorId)
end

local function removeSensor(sensorId)
  if rawLidarData[sensorId] then
    rawLidarData[sensorId].fileRef.onRemove()                                                       -- If a RAW LiDAR sensor, call the users remove function, if implemented.
  end
  Research.SensorManager.removeSensor(sensorId)
end

local function removeAllSensorsFromVehicle(vid)
  Research.SensorManager.removeSensorByVid(vid)
end

local function getAverageUpdateTime(sensorId)
  return Research.GpuRequestManager.getAverageUpdateTime(sensorId)
end

local function getMaxLoadPerFrame()
  return Research.GpuRequestManager.getMaxLoadPerFrame()
end

local function setMaxLoadPerFrame(maxLoadPerFrame)
  Research.GpuRequestManager.setMaxLoadPerFrame(maxLoadPerFrame)
end

-- Ad-hoc polling functions.
local function sendCameraRequest(sensorId) return Research.GpuRequestManager.sendAdHocCameraGpuRequest(sensorId) end
local function sendLidarRequest(sensorId) return Research.GpuRequestManager.sendAdHocLidarGpuRequest(sensorId) end
local function sendUltrasonicRequest(sensorId) return Research.GpuRequestManager.sendAdHocUltrasonicGpuRequest(sensorId) end
local function sendRadarRequest(sensorId) return Research.GpuRequestManager.sendAdHocRadarGpuRequest(sensorId) end

local function collectCameraRequest(requestId) return Research.GpuRequestManager.collectAdHocCameraGpuRequest(requestId) end

local function collectLidarRequest(requestId) return Research.GpuRequestManager.collectAdHocLidarGpuRequest(requestId) end
local function collectUltrasonicRequest(requestId) return Research.GpuRequestManager.collectAdHocUltrasonicGpuRequest(requestId) end
local function collectRadarRequest(requestId) return Research.GpuRequestManager.collectAdHocRadarGpuRequest(requestId) end

local function isRequestComplete(requestId) return Research.GpuRequestManager.isAdHocGpuRequestComplete(requestId) end

-- TODO Should be replaced when GE-2170 is complete.
local function getFullCameraRequest(sensorId)
  Engine.Annotation.enable(true)
  AnnotationManager.setInstanceAnnotations(false)
  local semanticData = Research.GpuRequestManager.sendBlockingCameraGpuRequest(sensorId)
  AnnotationManager.setInstanceAnnotations(true)
  local instanceData = Research.GpuRequestManager.sendBlockingCameraGpuRequest(sensorId)
  AnnotationManager.setInstanceAnnotations(false)
  Engine.Annotation.enable(false)
  local out = {}
  out['colour'] = instanceData['colour']
  out['annotation'] = semanticData['annotation']
  out['depth'] = instanceData['depth']
  out['instance'] = instanceData['annotation']
  return out
end

local function sendAdvancedIMURequest(sensorId, vid)
  local requestId = getUniqueRequestId()
  local vehicleId = scenetree.findObject(vid):getID();
  be:queueObjectLua(vehicleId, "extensions.tech_advancedIMU.adHocRequest(" .. sensorId .. ", " .. requestId .. ")")
  return requestId
end

local function collectAdvancedIMURequest(requestId)
  if adHocVluaRequests[requestId] ~= nil then
    local data = adHocVluaRequests[requestId]
    adHocVluaRequests[requestId] = nil
    return data
  end
  return false
end

local function sendGPSRequest(sensorId, vid)
  local requestId = getUniqueRequestId()
  local vehicleId = scenetree.findObject(vid):getID();
  be:queueObjectLua(vehicleId, "extensions.tech_GPS.adHocRequest(" .. sensorId .. ", " .. requestId .. ")")
  return requestId
end

local function collectGPSRequest(requestId)
  if adHocVluaRequests[requestId] ~= nil then
    local data = adHocVluaRequests[requestId]
    adHocVluaRequests[requestId] = nil
    return data
  end
  return false
end

local function sendPowertrainRequest(sensorId, vid)
  local requestId = getUniqueRequestId()
  local vehicleId = scenetree.findObject(vid):getID();
  be:queueObjectLua(vehicleId, "extensions.tech_powertrainSensor.adHocRequest(" .. sensorId .. ", " .. requestId .. ")")
  return requestId
end

local function collectPowertrainRequest(requestId)
  if adHocVluaRequests[requestId] ~= nil then
    local data = adHocVluaRequests[requestId]
    adHocVluaRequests[requestId] = nil
    return data
  end
  return false
end

local function sendIdealRADARRequest(sensorId, vid)
  local requestId = getUniqueRequestId()
  local vehicleId = scenetree.findObject(vid):getID();
  be:queueObjectLua(vehicleId, "extensions.tech_idealRADARSensor.adHocRequest(" .. sensorId .. ", " .. requestId .. ")")
  return requestId
end

local function collectIdealRADARRequest(requestId)
  if adHocVluaRequests[requestId] ~= nil then
    local data = adHocVluaRequests[requestId]
    adHocVluaRequests[requestId] = nil
    return data
  end
  return false
end

local function sendRoadsSensorRequest(sensorId, vid)
  local requestId = getUniqueRequestId()
  local vehicleId = scenetree.findObject(vid):getID();
  be:queueObjectLua(vehicleId, "extensions.tech_roadsSensor.adHocRequest(" .. sensorId .. ", " .. requestId .. ")")
  return requestId
end

local function collectRoadsSensorRequest(requestId)
  if adHocVluaRequests[requestId] ~= nil then
    local data = adHocVluaRequests[requestId]
    adHocVluaRequests[requestId] = nil
    return data
  end
  return false
end

local function sendMeshRequest(sensorId, vid)
  local requestId = getUniqueRequestId()
  local vehicleId = scenetree.findObject(vid):getID();
  be:queueObjectLua(vehicleId, "extensions.tech_meshSensor.adHocRequest(" .. sensorId .. ", " .. requestId .. ")")
  return requestId
end

local function collectMeshRequest(requestId)
  if adHocVluaRequests[requestId] ~= nil then
    local data = adHocVluaRequests[requestId]
    adHocVluaRequests[requestId] = nil
    return data
  end
  return false
end

local function isVluaRequestComplete(requestId)
  if adHocVluaRequests[requestId] ~= nil then
    return true
  end
  return true
end

local function attachSensor(sensorId, pos, dir, up, vid, isSensorStatic, isSnappingDesired, forceInsideTriangle, isAllowWheelNodes, isDirWorldSpace)
  Research.SensorMatrixManager.attachSensor(sensorId, pos, dir, up, vid, isSensorStatic, isSnappingDesired, forceInsideTriangle, isAllowWheelNodes, isDirWorldSpace)
end

local function getSensorMatrix(sensorId)
  return Research.SensorMatrixManager.getSensorMatrixExternal(sensorId)
end

local function getWorldFrame(sensorId)
  return Research.SensorMatrixManager.getWorldFrameVectors(sensorId)
end

local function getLocalFrame(sensorId)
  return Research.SensorMatrixManager.getLocalFrameVectors(sensorId)
end

local function getBeamData(vid)
  local vehicleId = scenetree.findObject(vid):getID();
  local d = Research.SensorMatrixManager.getBeamData(vehicleId)
  local beams = {}
  local ctr = 0
  for i=0, #d, 4 do
    beams[ctr] = {d[i], d[i + 1], d[i + 2], d[i + 3]}
    ctr = ctr + 1
  end
  return beams
end

local function getFullTriangleData(vid)
  local vehicleId = scenetree.findObject(vid):getID();
  local d = Research.SensorMatrixManager.getFullTriangleData(vehicleId)
  local triangles = {}
  local ctr = 0
  for i=0, #d, 3 do
    triangles[ctr] = {d[i], d[i + 1], d[i + 2]}
    ctr = ctr + 1
  end
  return triangles
end

local function getWheelTriangleData(vid, wheelIndex)
  local vehicleId = scenetree.findObject(vid):getID();
  local d = Research.SensorMatrixManager.getWheelTriangleData(vehicleId, wheelIndex)
  local triangles = {}
  local ctr = 0
  for i=0, #d, 3 do
    triangles[ctr] = {d[i], d[i + 1], d[i + 2]}
    ctr = ctr + 1
  end
  return triangles
end

local function getNodePositions(vid, nodeId)
  local vehicleId = scenetree.findObject(vid):getID();
  return Research.SensorMatrixManager.getNodePositions(vehicleId)
end

local function getClosestMeshPointToGivenPoint(vid, point)
  local vehicleId = scenetree.findObject(vid):getID();
  return Research.SensorMatrixManager.getClosestMeshPointToGivenPoint(vehicleId, point)
end

local function getClosestTriangle(vid, point, includeWheelNodes)
  local vehicleId = scenetree.findObject(vid):getID();
  return Research.SensorMatrixManager.getClosestTriangle(vehicleId, point, includeWheelNodes)
end

local function createCamera(vid, args)
  return Research.SensorManager.createCameraSensorWithoutSharedMemory(vid, args)
end

local function createCameraWithSharedMemory(vid, args)
  return Research.SensorManager.createCameraSensorWithSharedMemory(vid, args)
end

local function getCameraImage(sensorId)
  cameraBufferColors[sensorId] = cameraBufferColors[sensorId] or buffer.new()
  Research.Camera.getLastCameraColorBuffer(sensorId, cameraBufferColors[sensorId])
  return cameraBufferColors[sensorId]
end

local function getCameraAnnotations(sensorId)
  cameraBufferAnnotations[sensorId] = cameraBufferAnnotations[sensorId] or buffer.new()
  Research.Camera.getLastCameraAnnotationsBuffer(sensorId, cameraBufferAnnotations[sensorId])
  return cameraBufferAnnotations[sensorId]
end

local function getCameraDepth(sensorId)
  cameraBufferDepth[sensorId] = cameraBufferDepth[sensorId] or buffer.new()
  Research.Camera.getLastCameraDepthBuffer(sensorId, cameraBufferDepth[sensorId])
  return cameraBufferDepth[sensorId]
end

local function getCameraData(sensorId)
  return {
    colour = getCameraImage(sensorId),
    annotation = getCameraAnnotations(sensorId),
    depth = getCameraDepth(sensorId) }
end

local function getCameraDataShmem(sensorId)
  return Research.Camera.getLastCameraDataShmem(sensorId)
end

local function processCameraData(sensorId)
  local binary = getCameraData(sensorId)
  local colourData, cData = {}, tostring(binary.colour)
  for i = 1, #cData do
    table.insert(colourData, cData:byte(i))
  end
  local annotationData, aData = {}, tostring(binary.annotation)
  for i = 1, #aData do
    table.insert(annotationData, aData:byte(i))
  end
  local depthData, dData = {}, tostring(binary.depth)
  for i = 1, #dData, 4 do
    table.insert(depthData, unpack_float(dData:byte(i), dData:byte(i + 1), dData:byte(i + 2), dData:byte(i + 3)))
  end
  return { colour = colourData, annotation = annotationData, depth = depthData}
end

local function getCameraSensorPosition(sensorId)
  return Research.Camera.getSensorPosition(sensorId)
end

local function getCameraSensorDirection(sensorId)
  return Research.Camera.getSensorDirection(sensorId)
end

local function getCameraMaxPendingGpuRequests(sensorId)
  return Research.Camera.getMaxPendingGpuRequests(sensorId)
end

local function getCameraRequestedUpdateTime(sensorId)
  return Research.Camera.getRequestedUpdateTime(sensorId)
end

local function getCameraUpdatePriority(sensorId)
  return Research.Camera.getUpdatePriority(sensorId)
end

-- Functions to set the fustrum properties of an existing camera sensor.
local function setCameraSensorPosition(sensorId, pos) Research.Camera.setSensorPosition(sensorId, pos) end
local function setCameraSensorDirection(sensorId, dir) Research.Camera.setSensorDirection(sensorId, dir) end
local function setCameraSensorUp(sensorId, dir) Research.Camera.setSensorUp(sensorId, dir) end
local function setCameraSensorResolution(sensorId, x, y) Research.Camera.setSensorResolution(sensorId, x, y) end
local function setCameraSensorFOVX(sensorId, x) Research.Camera.setSensorFOVX(sensorId, x) end
local function setCameraSensorNearFarPlanes(sensorId, pNear, pFar) Research.Camera.setSensorNearFarPlanes(sensorId, pNear, pFar) end

local function setCameraMaxPendingGpuRequests(sensorId, maxPendingGpuRequests)
  Research.Camera.setMaxPendingGpuRequests(sensorId, maxPendingGpuRequests)
end

local function setCameraRequestedUpdateTime(sensorId, requestedUpdateTime)
  Research.Camera.setRequestedUpdateTime(sensorId, requestedUpdateTime)
end

local function setCameraUpdatePriority(sensorId, priority)
  Research.Camera.setUpdatePriority(sensorId, priority)
end

local function convertWorldPointToPixel(sensorId, point)
  return Research.Camera.convertWorldPointToPixel(sensorId, point)
end

-- Creates a new Raw LiDAR sensor.
-- [Note: these are managed in a special way. We return only the sector of data, based on the position of the sensor due to its rotation, since the last frame, and poll it ad-hoc.]
local function createRawLidar(vid, filename, args)

  -- Get a reference to the user's lua file.
  local fileRef = require(filename)
  if not fileRef then
    log('E', 'sensors', 'ERROR - user .lua file not found at the given path: ' .. tostring(filename))
  end

  -- Call the user's lua initialisation callback.
  local initData = fileRef.onInit(args)
  if not initData then
    log('E', 'sensors', 'Initialization of user .lua file failed.')
    return
  end

  local args = {}
  args.pos = vec3(initData.posX, initData.posY, initData.posZ)
  args.up = vec3(initData.upX, initData.upY, initData.upZ)
  args.size = { initData.resX, initData.resY }
  args.fovY = initData.fovY
  args.nearFarPlanes = { initData.pNear, initData.pFar }
  args.updateTime = -1.0                                                                            -- We will only use ad-hoc polling with this sensor type.
  args.renderColours = true
  args.renderAnnotations = true
  args.renderInstance = false
  args.renderDepth = true

  -- Create the four sensors (one per each quadrant, with direction set to the center of each quadrant, with the required amount of overlap)
  local aperture = math.pi * 0.5 + initData.overlap
  local dir = vec3(initData.dirX, initData.dirY, initData.dirZ)
  args.dir = rotateVecAroundAxis(dir, args.up, -math.pi * 0.25)                                     -- The first quadrant sensor is centered at -45 degrees.
  local sensorId1 = Research.SensorManager.createCameraSensorWithoutSharedMemory(vid, args)
  setCameraSensorFOVX(sensorId1, aperture)
  args.dir = rotateVecAroundAxis(dir, args.up, math.pi * 0.25)                                      -- The second quadrant sensor is centered at 45 degrees.
  local sensorId2 = Research.SensorManager.createCameraSensorWithoutSharedMemory(vid, args)
  setCameraSensorFOVX(sensorId2, aperture)
  args.dir = rotateVecAroundAxis(dir, args.up, math.pi * 0.75)                                      -- The third quadrant sensor is centered at 135 degrees.
  local sensorId3 = Research.SensorManager.createCameraSensorWithoutSharedMemory(vid, args)
  setCameraSensorFOVX(sensorId3, aperture)
  args.dir = rotateVecAroundAxis(dir, args.up, -math.pi * 0.75)                                     -- The third quadrant sensor is centered at -135 degrees.
  local sensorId4 = Research.SensorManager.createCameraSensorWithoutSharedMemory(vid, args)
  setCameraSensorFOVX(sensorId4, aperture)

  Engine.Annotation.enable(true)                                                                    -- Ensure that annotations are switched on in the engine.

  -- Cache the sensor set's data. [Note: the sensors are stored using the unique Id of the first sensor].
  rawLidarData[sensorId1] = {
    fileRef = fileRef,
    sensorId1 = sensorId1, sensorId2 = sensorId2, sensorId3 = sensorId3, sensorId4 = sensorId4,
    pos = args.pos, dir = dir, up = args.up, resX = args.size.x, resY = args.size.y, fovY = args.fovY, pNear = args.nearFarPlanes[1], pFar = args.nearFarPlanes[2],
    requests1 = {}, requests2 = {}, requests3 = {}, requests4 = {} }

  return { sensorId1, sensorId2, sensorId3, sensorId4 }
end

local function createLidar(vid, args)
  return Research.SensorManager.createLidarSensorWithoutSharedMemory(vid, args)
end

local function createLidarWithSharedMemory(vid, args)
  return Research.SensorManager.createLidarSensorWithSharedMemory(vid, args)
end

local function getLidarPointCloud(sensorId)
  lidarBufferPoints[sensorId] = lidarBufferPoints[sensorId] or buffer.new()
  Research.Lidar.getLastPointCloudBuffer(sensorId,  lidarBufferPoints[sensorId])
  return  lidarBufferPoints[sensorId]
end

local function getLidarColourData(sensorId)
  lidarBufferColors[sensorId] = lidarBufferColors[sensorId] or buffer.new()
  Research.Lidar.getLastColourBuffer(sensorId,  lidarBufferColors[sensorId])
  return  lidarBufferColors[sensorId]
end

local function getLidarDataPositions(sensorId)
  local pts = tostring(getLidarPointCloud(sensorId))
  local pointsData = {}
  local numPts = #pts
  for i = 1, numPts, 12 do
    local x = unpack_float(pts:byte(i), pts:byte(i + 1), pts:byte(i + 2), pts:byte(i + 3))
    local y = unpack_float(pts:byte(i + 4), pts:byte(i + 5), pts:byte(i + 6), pts:byte(i + 7))
    local z = unpack_float(pts:byte(i + 8), pts:byte(i + 9), pts:byte(i + 10), pts:byte(i + 11))
    table.insert(pointsData, vec3(x, y, z))
  end
  local colourBinary = tostring(getLidarColourData(sensorId))
  local colourData = {}
  local numColour = #colourBinary
  for i = 1, numColour do
    table.insert(colourData, colourBinary:byte(i))
  end
  return { pointCloud = pointsData, colour = colourData }
end

local function getLidarPointCloudShmem(sensorId)
  return Research.Lidar.getLastPointCloudDataShmem(sensorId)
end

local function getLidarColourDataShmem(sensorId)
  return Research.Lidar.getLastColourDataShmem(sensorId)
end

local function getActiveLidarSensors()
  return Research.Lidar.getActiveLidarSensors()
end

local function getLidarSensorPosition(sensorId)
  return Research.Lidar.getSensorPosition(sensorId)
end

local function getLidarSensorDirection(sensorId)
  return Research.Lidar.getSensorDirection(sensorId)
end

local function getLidarVerticalResolution(sensorId)
  return Research.Lidar.getVerticalRes(sensorId)
end

local function getLidarFrequency(sensorId)
  return Research.Lidar.getFrequency(sensorId)
end

local function getLidarMaxDistance(sensorId)
  return Research.Lidar.getMaxDistance(sensorId)
end

local function getLidarIsVisualised(sensorId)
  return Research.Lidar.getIsVisualised(sensorId)
end

local function getLidarIsAnnotated(sensorId)
  return Research.Lidar.getIsAnnotated(sensorId)
end

local function getLidarMaxPendingGpuRequests(sensorId)
  return Research.Lidar.getMaxPendingGpuRequests(sensorId)
end

local function getLidarRequestedUpdateTime(sensorId)
  return Research.Lidar.getRequestedUpdateTime(sensorId)
end

local function getLidarUpdatePriority(sensorId)
  return Research.Lidar.getUpdatePriority(sensorId)
end

local function setLidarVerticalResolution(sensorId, verticalResolution)
  Research.Lidar.setVerticalRes(sensorId, verticalResolution)
end

local function setLidarFrequency(sensorId, frequency)
  Research.Lidar.setFrequency(sensorId, frequency)
end

local function setLidarMaxDistance(sensorId, maxDistance)
  Research.Lidar.setMaxDistance(sensorId, maxDistance)
end

local function setLidarIsVisualised(sensorId, isVisualised)
  Research.Lidar.setIsVisualised(sensorId, isVisualised)
end

local function setLidarIsAnnotated(sensorId, isAnnotated)
  Research.Lidar.setIsAnnotated(sensorId, isAnnotated)
end

local function setLidarMaxPendingGpuRequests(sensorId, maxPendingGpuRequests)
  Research.Lidar.setMaxPendingGpuRequests(sensorId, maxPendingGpuRequests)
end

local function setLidarRequestedUpdateTime(sensorId, requestedUpdateTime)
  Research.Lidar.setRequestedUpdateTime(sensorId, requestedUpdateTime)
end

local function setLidarUpdatePriority(sensorId, updatePriority)
  Research.Lidar.setUpdatePriority(sensorId, updatePriority)
end

local function createUltrasonic(vid, args)
  local sensorId = Research.SensorManager.createUltrasonicSensor(vid, args)
  if args.isVisualised or args.isVisualised == nil  then
    visualisedUltrasonicSensors[sensorId] = { animationTime = 0.0 }
  end
  return sensorId
end

local function getUltrasonicReadings(sensorId)
  return Research.Ultrasonic.getLastReadings(sensorId)
end

local function getActiveUltrasonicSensors()
  return Research.Ultrasonic.getActiveUltrasonicSensors()
end

local function getUltrasonicIsVisualised(sensorId)
  return visualisedUltrasonicSensors[sensorId] ~= nil
end

local function getUltrasonicMaxPendingGpuRequests(sensorId)
  return Research.Ultrasonic.getMaxPendingGpuRequests(sensorId)
end

local function getUltrasonicRequestedUpdateTime(sensorId)
  return Research.Ultrasonic.getRequestedUpdateTime(sensorId)
end

local function getUltrasonicUpdatePriority(sensorId)
  return Research.Ultrasonic.getUpdatePriority(sensorId)
end

local function setUltrasonicIsVisualised(sensorId, isVisualised)
  if isVisualised then
    visualisedUltrasonicSensors[sensorId] = { animationTime = 0.0 }
  else
    visualisedUltrasonicSensors[sensorId] = nil
  end
end

local function getUltrasonicSensorPosition(sensorId)
  return Research.Ultrasonic.getSensorPosition(sensorId)
end

local function getUltrasonicSensorDirection(sensorId)
  return Research.Ultrasonic.getSensorDirection(sensorId)
end

local function getUltrasonicSensorRadius(sensorId, distanceFromSensor)
  return Research.Ultrasonic.getSensorRadius(sensorId, distanceFromSensor)
end

local function setUltrasonicMaxPendingGpuRequests(sensorId, maxPendingGpuRequests)
  Research.Ultrasonic.setMaxPendingGpuRequests(sensorId, maxPendingGpuRequests)
end

local function setUltrasonicRequestedUpdateTime(sensorId, requestedUpdateTime)
  Research.Ultrasonic.setRequestedUpdateTime(sensorId, requestedUpdateTime)
end

local function setUltrasonicUpdatePriority(sensorId, updatePriority)
  Research.Ultrasonic.setUpdatePriority(sensorId, updatePriority)
end

local function visualiseUltrasonicSensor(sensorId, dtSim)
  -- If this sensor no longer exists, remove from visualisation array and leave early.
  if not doesSensorExist(sensorId) then
    visualisedUltrasonicSensors[sensorId] = nil
    return
  end

  -- Get the world space position and direction of this ultrasonic sensor.
  local pos = getUltrasonicSensorPosition(sensorId)
  local dir = getUltrasonicSensorDirection(sensorId):normalized()

  -- Draw the ultrasonic sensor at its current position in world space, in green.
  debugDrawer:drawSphere(pos, 0.05, ColorF(0, 1, 0, 1))

  -- Cycle the animation phase based on the simDt value and the wave parameters (period, speed).
  local animationTime = visualisedUltrasonicSensors[sensorId].animationTime + dtSim
  if animationTime >= usParameters.animationPeriod then
    animationTime = animationTime - usParameters.animationPeriod
  end

  visualisedUltrasonicSensors[sensorId].animationTime = animationTime

  -- Get the latest measurements computed by this ultrasonic sensor.
  local lastReadings = getUltrasonicReadings(sensorId)
  local lastDistance = lastReadings['distance']
  local lastWindowMin = lastReadings['windowMin']

  -- Compute the physical distance travelled by the outgoing pulse, at the current animation phase.
  local pulseDistance = usParameters.animationSpeed * animationTime

  -- If we are in the transmission phase, draw the red transmission pulse (heading outward from the sensor).
  if pulseDistance <= lastDistance then

    -- Compute the centre point of the outward-travelling pulse.
    local pulseCentre = pos + pulseDistance * dir

    -- Compute the half width of the pulse. The pulse slowly disperses longitudinally as the distance increases.
    local halfPulseWidth = math.min(
      usParameters.maxPulseWidth,
      math.max(usParameters.minPulseWidth, lastDistance - lastWindowMin) + pulseDistance * usParameters.pulseWidthDispersion)

    -- Compute the top and bottom cylinder points for the pulse. We use the measurement window width as the height.
    local halfCylinderVector = halfPulseWidth * dir
    local firstPoint = pulseCentre - halfCylinderVector
    local secondPoint = pulseCentre + halfCylinderVector

    -- Compute the radius and alpha channel value for the pulse at this distance.
    local radius = getUltrasonicSensorRadius(sensorId, pulseDistance)
    local alpha = math.max(usParameters.minAlpha, 1.0 - pulseDistance)

    -- Draw a red cylinder to represent the outgoing pulse.
    debugDrawer:drawCylinder(firstPoint, secondPoint, radius, ColorF(1, 0, 0, alpha))
  else

    local bounceDistance = pulseDistance - lastDistance

    -- Compute the physical distance travelled by the returning pulse.
    local pulseDistance = lastDistance - bounceDistance

    -- If we have reached the sensor position, stop the returning pulse animation.
    if pulseDistance < 0 then
      return
    end

    -- Compute the centre point of the returning pulse.
    local pulseCentre = pos + pulseDistance * dir

    -- Compute the half width of the pulse. The pulse slowly disperses longitudinally as the distance increases.
    local halfPulseWidth = math.max(usParameters.minPulseWidth, lastDistance - lastWindowMin) + bounceDistance * usParameters.pulseWidthDispersion

    -- Compute the top and bottom cylinder points for the pulse. We use the measurement window width as the height.
    local halfCylinderVector = halfPulseWidth * dir
    local firstPoint = pulseCentre - halfCylinderVector
    local secondPoint = pulseCentre + halfCylinderVector

    -- Compute the radius and alpha channel value for the pulse at this distance.
    local radius = getUltrasonicSensorRadius(sensorId, lastDistance) + bounceDistance * 0.1
    local alpha = math.max(usParameters.minAlpha, 1 - bounceDistance)

    -- Draw a blue cylinder to represent the returning pulse. The radius grows linearly for this pulse (unlike the outgoing pulse).
    debugDrawer:drawCylinder(firstPoint, secondPoint, radius, ColorF(0, 0, 1, alpha))
  end
end

local function createRadar(vid, args)
  return Research.SensorManager.createRadarSensor(vid, args)
end

local function getRadarReadings(sensorId)
  radarBufferPoints[sensorId] = radarBufferPoints[sensorId] or buffer.new()
  Research.Radar.getLastReadingsBuffer(sensorId,  radarBufferPoints[sensorId])
  return radarBufferPoints[sensorId]
end

local function getRadarPPIData(sensorId)
  radarBufferPPI[sensorId] = radarBufferPPI[sensorId] or buffer.new()
  Research.Radar.getPPIBuffer(sensorId,  radarBufferPPI[sensorId])
  return radarBufferPPI[sensorId]
end

local function getRadarRangeDopplerData(sensorId)
  radarBufferRangeDoppler[sensorId] = radarBufferRangeDoppler[sensorId] or buffer.new()
  Research.Radar.getRangeDopplerBuffer(sensorId,  radarBufferRangeDoppler[sensorId])
  return radarBufferRangeDoppler[sensorId]
end

local function getActiveRadarSensors()
  return Research.Radar.getActiveRadarSensors()
end

local function getRadarMaxPendingGpuRequests(sensorId)
  return Research.Radar.getMaxPendingGpuRequests(sensorId)
end

local function getRadarRequestedUpdateTime(sensorId)
  return Research.Radar.getRequestedUpdateTime(sensorId)
end

local function getRadarUpdatePriority(sensorId)
  return Research.Radar.getUpdatePriority(sensorId)
end

local function getRadarSensorPosition(sensorId)
  return Research.Radar.getSensorPosition(sensorId)
end

local function getRadarSensorDirection(sensorId)
  return Research.Radar.getSensorDirection(sensorId)
end

local function setRadarMaxPendingGpuRequests(sensorId, maxPendingGpuRequests)
  Research.Radar.setMaxPendingGpuRequests(sensorId, maxPendingGpuRequests)
end

local function setRadarRequestedUpdateTime(sensorId, requestedUpdateTime)
  Research.Radar.setRequestedUpdateTime(sensorId, requestedUpdateTime)
end

local function setRadarUpdatePriority(sensorId, updatePriority)
  Research.Radar.setUpdatePriority(sensorId, updatePriority)
end

local function createAdvancedIMU(vid, args)

  -- Set optional parameters to defaults if they are not provided by the user.
  if args.pos == nil then args.pos = vec3(0, 0, 3) end
  if args.dir == nil then args.dir = vec3(0, -1, 0) end
  if args.up == nil then args.up = vec3(0, 0, 1) end
  args.up = -args.up  -- // we need to flip the up direction vector to get the orientation correct when attaching the sensor.
  if args.GFXUpdateTime == nil then args.GFXUpdateTime = 0.1 end
  if args.isUsingGravity == nil then args.isUsingGravity = false end
  if args.isVisualised == nil then args.isVisualised = true end
  if args.isSnappingDesired == nil then args.isSnappingDesired = false end
  if args.isForceInsideTriangle == nil then args.isForceInsideTriangle = false end
  if args.isAllowWheelNodes == nil then args.isAllowWheelNodes = false end
  if args.physicsUpdateTime == nil then args.physicsUpdateTime = 0.015 end

  -- The user should provide either a window width or a cutoff frequency for the filtering.
  if args.accelWindowWidth == nil and args.accelFrequencyCutoff == nil then args.accelWindowWidth = 50 end
  if args.gyroWindowWidth == nil and args.gyroFrequencyCutoff == nil then args.gyroWindowWidth = 50 end

  -- Attach the sensor to the vehicle.
  local sensorId = Research.SensorManager.getNewSensorId()
  Research.SensorMatrixManager.attachSensor(sensorId, args.pos, args.dir, args.up, vid, false, args.isSnappingDesired,
    args.isForceInsideTriangle, args.isAllowWheelNodes, args.isDirWorldSpace)
  local attachData = Research.SensorMatrixManager.getAttachData(sensorId)

  -- Create the AdvancedIMU in vlua.
  local data =
  {
    sensorId = sensorId,
    GFXUpdateTime = args.GFXUpdateTime,
    physicsUpdateTime = args.physicsUpdateTime,
    isUsingGravity = args.isUsingGravity,
    nodeIndex1 = attachData['nodeIndex1'],
    nodeIndex2 = attachData['nodeIndex2'],
    nodeIndex3 = attachData['nodeIndex3'],
    u = attachData['u'],
    v = attachData['v'],
    signedProjDist = attachData['signedProjDist'],
    triangleSpaceForward = attachData['triangleSpaceForward'],
    triangleSpaceUp = attachData['triangleSpaceUp'],
    isVisualised = args.isVisualised,
    accelWindowWidth = args.accelWindowWidth,
    gyroWindowWidth = args.gyroWindowWidth,
    frequencyCutoff = args.frequencyCutoff
  }
  local serializedData = string.format("extensions.tech_advancedIMU.create(%q)", lpack.encode(data))
  be:queueObjectLua(vid, serializedData)

  advancedIMULastRawReadings[sensorId] = {}

  return sensorId
end

local function removeAdvancedIMU(vid, sensorId)
  local vehicleId = scenetree.findObject(vid):getID()
  be:queueObjectLua(vehicleId, "extensions.tech_advancedIMU.remove(" .. sensorId .. ")")
  advancedIMULastRawReadings[sensorId] = nil
end

local function getAdvancedIMUReadings(sensorId)
  local outData = {}
  for k, v in pairs(advancedIMULastRawReadings[sensorId]) do
    outData[k] = v
  end
  advancedIMULastRawReadings[sensorId] = {}
  return outData
end

local function updateAdvancedIMULastReadings(data)
  local newReadings = lpack.decode(data)
  if advancedIMULastRawReadings[newReadings.sensorId] == nil then
    return
  end
  local ctr = #advancedIMULastRawReadings[newReadings.sensorId]
  for k, v in pairs(newReadings.reading) do
    advancedIMULastRawReadings[newReadings.sensorId][ctr] = v
    ctr = ctr + 1
  end
end

local function updateAdvancedIMUAdHocRequest(data)
  local d = lpack.decode(data)
  adHocVluaRequests[d.requestId] = d.reading
end

local function setAdvancedIMUUpdateTime(sensorId, vid, updateTime)
  local vehicleId = scenetree.findObject(vid):getID();
  be:queueObjectLua(vehicleId, "extensions.tech_advancedIMU.setUpdateTime(" .. sensorId .. ", " .. updateTime .. ")")
end

local function setAdvancedIMUIsUsingGravity(sensorId, vid, isUsingGravity)
  local data = { sensorId = sensorId, isUsingGravity = isUsingGravity }
  local serialisedData = string.format("extensions.tech_advancedIMU.setIsUsingGravity(%q)", lpack.encode(data))
  be:queueObjectLua(scenetree.findObject(vid):getID(), serialisedData)
end

local function setAdvancedIMUIsVisualised(sensorId, vid, isVisualised)
  local data = { sensorId = sensorId, isVisualised = isVisualised }
  local serialisedData = string.format("extensions.tech_advancedIMU.setIsVisualised(%q)", lpack.encode(data))
  be:queueObjectLua(scenetree.findObject(vid):getID(), serialisedData)
end

local function createGPS(vid, args)

  -- Set optional parameters to defaults if they are not provided by the user.
  if args.pos == nil then args.pos = vec3(0, 0, 3) end
  if args.dir == nil then args.dir = vec3(0, -1, 0) end
  if args.up == nil then args.up = vec3(0, 0, 1) end
  args.up = -args.up  -- // we need to flip the up direction vector to get the orientation correct when attaching the sensor.
  if args.GFXUpdateTime == nil then args.GFXUpdateTime = 0.1 end
  if args.isVisualised == nil then args.isVisualised = true end
  if args.isSnappingDesired == nil then args.isSnappingDesired = false end
  if args.isForceInsideTriangle == nil then args.isForceInsideTriangle = false end
  if args.isAllowWheelNodes == nil then args.isAllowWheelNodes = false end
  if args.physicsUpdateTime == nil then args.physicsUpdateTime = 0.015 end
  if args.refLon == nil then args.refLon = 0.0 end
  if args.refLat == nil then args.refLat = 0.0 end

  -- Attach the sensor to the vehicle.
  local sensorId = Research.SensorManager.getNewSensorId()
  Research.SensorMatrixManager.attachSensor(sensorId, args.pos, args.dir, args.up, vid, false, args.isSnappingDesired,
    args.isForceInsideTriangle, args.isAllowWheelNodes, args.isDirWorldSpace)
  local attachData = Research.SensorMatrixManager.getAttachData(sensorId)

  -- Create the GPS sensor in vlua.
  local data =
  {
    sensorId = sensorId,
    GFXUpdateTime = args.GFXUpdateTime,
    physicsUpdateTime = args.physicsUpdateTime,
    nodeIndex1 = attachData['nodeIndex1'],
    nodeIndex2 = attachData['nodeIndex2'],
    nodeIndex3 = attachData['nodeIndex3'],
    u = attachData['u'],
    v = attachData['v'],
    refLon = args.refLon,
    refLat = args.refLat,
    signedProjDist = attachData['signedProjDist'],
    isVisualised = args.isVisualised
  }
  local serializedData = string.format("extensions.tech_GPS.create(%q)", lpack.encode(data))
  be:queueObjectLua(vid, serializedData)

  GPSLastRawReadings[sensorId] = {}

  return sensorId
end

local function removeGPS(vid, sensorId)
  local vehicleId = scenetree.findObject(vid):getID()
  be:queueObjectLua(vehicleId, "extensions.tech_GPS.remove(" .. sensorId .. ")")
  GPSLastRawReadings[sensorId] = nil
end

local function getGPSReadings(sensorId)
  local outData = {}
  for k, v in pairs(GPSLastRawReadings[sensorId]) do
    outData[k] = v
  end
  GPSLastRawReadings[sensorId] = {}
  return outData
end

local function updateGPSLastReadings(data)
  local newReadings = lpack.decode(data)
  if GPSLastRawReadings[newReadings.sensorId] == nil then
    return
  end
  local ctr = #GPSLastRawReadings[newReadings.sensorId]
  for k, v in pairs(newReadings.reading) do
    GPSLastRawReadings[newReadings.sensorId][ctr] = v
    ctr = ctr + 1
  end
end

local function updateGPSAdHocRequest(data)
  local d = lpack.decode(data)
  adHocVluaRequests[d.requestId] = d.reading
end

local function setGPSUpdateTime(sensorId, vid, updateTime)
  local vehicleId = scenetree.findObject(vid):getID();
  be:queueObjectLua(vehicleId, "extensions.tech_GPS.setUpdateTime(" .. sensorId .. ", " .. updateTime .. ")")
end

local function setGPSIsVisualised(sensorId, vid, isVisualised)
  local data = { sensorId = sensorId, isVisualised = isVisualised }
  local serialisedData = string.format("extensions.tech_GPS.setIsVisualised(%q)", lpack.encode(data))
  be:queueObjectLua(scenetree.findObject(vid):getID(), serialisedData)
end

local function createPowertrainSensor(vid, args)

  -- Set optional parameters to defaults if they are not provided by the user.
  if args.GFXUpdateTime == nil then args.GFXUpdateTime = 0.1 end
  if args.physicsUpdateTime == nil then args.physicsUpdateTime = 0.015 end

  -- Get a unique sensor Id for this Powertrain sensor.
  local sensorId = Research.SensorManager.getNewSensorId()

  -- Create the Powertrain in vlua.
  local data = { sensorId = sensorId, GFXUpdateTime = args.GFXUpdateTime, physicsUpdateTime = args.physicsUpdateTime }
  local serializedData = string.format("extensions.tech_powertrainSensor.create(%q)", lpack.encode(data))
  be:queueObjectLua(vid, serializedData)

  powertrainLastRawReadings[sensorId] = {}

  return sensorId
end

local function removePowertrainSensor(vid, sensorId)
  local vehicleId = scenetree.findObject(vid):getID()
  be:queueObjectLua(vehicleId, "extensions.tech_powertrainSensor.remove(" .. sensorId .. ")")
  powertrainLastRawReadings[sensorId] = nil
end

local function getPowertrainReadings(sensorId)
  local outData = {}
  for k, v in pairs(powertrainLastRawReadings[sensorId]) do
    outData[k] = v
  end
  powertrainLastRawReadings[sensorId] = {}
  return outData
end

local function updatePowertrainLastReadings(data)
  local newReadings = lpack.decode(data)
  if powertrainLastRawReadings[newReadings.sensorId] == nil then
    return
  end
  local ctr = #powertrainLastRawReadings[newReadings.sensorId]
  for k, v in pairs(newReadings.reading) do
    powertrainLastRawReadings[newReadings.sensorId][ctr] = v
    ctr = ctr + 1
  end
end

local function updatePowertrainAdHocRequest(data)
  local d = lpack.decode(data)
  adHocVluaRequests[d.requestId] = d.reading
end

local function setPowertrainUpdateTime(sensorId, vid, updateTime)
  local vehicleId = scenetree.findObject(vid):getID();
  be:queueObjectLua(vehicleId, "extensions.tech_powertrainSensor.setUpdateTime(" .. sensorId .. ", " .. updateTime .. ")")
end

local function createIdealRADARSensor(vid, args)

  -- Set optional parameters to defaults if they are not provided by the user.
  if args.GFXUpdateTime == nil then args.GFXUpdateTime = 0.1 end
  if args.physicsUpdateTime == nil then args.physicsUpdateTime = 0.015 end

  -- Get a unique sensor Id for this ideal RADAR sensor.
  local sensorId = Research.SensorManager.getNewSensorId()

  -- Create the ideal RADAR in vlua.
  local data = { sensorId = sensorId, GFXUpdateTime = args.GFXUpdateTime, physicsUpdateTime = args.physicsUpdateTime }
  local serializedData = string.format("extensions.tech_idealRADARSensor.create(%q)", lpack.encode(data))
  be:queueObjectLua(vid, serializedData)

  idealRADARLastRawReadings[sensorId] = {}

  return sensorId
end

local function removeIdealRADARSensor(vid, sensorId)
  local vehicleId = scenetree.findObject(vid):getID()
  be:queueObjectLua(vehicleId, "extensions.tech_idealRADARSensor.remove(" .. sensorId .. ")")
  idealRADARLastRawReadings[sensorId] = nil
end

local function getIdealRADARReadings(sensorId)
  local outData = {}
  for k, v in pairs(idealRADARLastRawReadings[sensorId]) do
    outData[k] = v
  end
  idealRADARLastRawReadings[sensorId] = {}
  return outData
end

local function updateIdealRADARLastReadings(data)
  local newReadings = lpack.decode(data)
  if idealRADARLastRawReadings[newReadings.sensorId] == nil then
    return
  end
  local ctr = #idealRADARLastRawReadings[newReadings.sensorId]
  for k, v in pairs(newReadings.reading) do
    idealRADARLastRawReadings[newReadings.sensorId][ctr] = v
    ctr = ctr + 1
  end
end

local function updateIdealRADARAdHocRequest(data)
  local d = lpack.decode(data)
  adHocVluaRequests[d.requestId] = d.reading
end

local function setIdealRADARUpdateTime(sensorId, vid, updateTime)
  local vehicleId = scenetree.findObject(vid):getID();
  be:queueObjectLua(vehicleId, "extensions.tech_idealRADARSensor.setUpdateTime(" .. sensorId .. ", " .. updateTime .. ")")
end

local function createRoadsSensor(vid, args)

  -- Set optional parameters to defaults if they are not provided by the user.
  if args.GFXUpdateTime == nil then args.GFXUpdateTime = 0.1 end
  if args.physicsUpdateTime == nil then args.physicsUpdateTime = 0.015 end

  -- Get a unique sensor Id for this roads sensor.
  local sensorId = Research.SensorManager.getNewSensorId()

  -- Create the roads sensor in vlua.
  local data = { sensorId = sensorId, GFXUpdateTime = args.GFXUpdateTime, physicsUpdateTime = args.physicsUpdateTime }
  local serializedData = string.format("extensions.tech_roadsSensor.create(%q)", lpack.encode(data))
  be:queueObjectLua(vid, serializedData)

  roadsSensorLastRawReadings[sensorId] = {}

  return sensorId
end

local function removeRoadsSensor(vid, sensorId)
  local vehicleId = scenetree.findObject(vid):getID()
  be:queueObjectLua(vehicleId, "extensions.tech_roadsSensor.remove(" .. sensorId .. ")")
  roadsSensorLastRawReadings[sensorId] = nil
end

local function getRoadsSensorReadings(sensorId)
  local outData = {}
  for k, v in pairs(roadsSensorLastRawReadings[sensorId]) do
    outData[k] = v
  end
  roadsSensorLastRawReadings[sensorId] = {}
  return outData
end

local function updateRoadsSensorLastReadings(data)
  local newReadings = lpack.decode(data)
  if roadsSensorLastRawReadings[newReadings.sensorId] == nil then
    return
  end
  local ctr = #roadsSensorLastRawReadings[newReadings.sensorId]
  for k, v in pairs(newReadings.reading) do
    roadsSensorLastRawReadings[newReadings.sensorId][ctr] = v
    ctr = ctr + 1
  end
end

local function updateRoadsSensorAdHocRequest(data)
  local d = lpack.decode(data)
  adHocVluaRequests[d.requestId] = d.reading
end

local function setRoadsSensorUpdateTime(sensorId, vid, updateTime)
  local vehicleId = scenetree.findObject(vid):getID();
  be:queueObjectLua(vehicleId, "extensions.tech_roadsSensor.setUpdateTime(" .. sensorId .. ", " .. updateTime .. ")")
end

local function createMeshSensor(vid, args)

  -- Set optional parameters to defaults if they are not provided by the user.
  if args.GFXUpdateTime == nil then args.GFXUpdateTime = 0.1 end
  if args.physicsUpdateTime == nil then args.physicsUpdateTime = 0.015 end

  -- Get a unique sensor Id for this Powertrain sensor.
  local sensorId = Research.SensorManager.getNewSensorId()

  -- Create the Mesh sensor in vlua.
  local data = { sensorId = sensorId, GFXUpdateTime = args.GFXUpdateTime }
  local serializedData = string.format("extensions.tech_mesh.create(%q)", lpack.encode(data))
  be:queueObjectLua(vid, serializedData)

  return sensorId
end

local function removeMeshSensor(vid, sensorId)
  local vehicleId = scenetree.findObject(vid):getID()
  be:queueObjectLua(vehicleId, "extensions.tech_mesh.remove(" .. sensorId .. ")")
end

local function updateMeshAdHocRequest(data)
  local d = lpack.decode(data)
  adHocVluaRequests[d.requestId] = d.reading
end

local function setMeshUpdateTime(sensorId, vid, updateTime)
  local vehicleId = scenetree.findObject(vid):getID();
  be:queueObjectLua(vehicleId, "extensions.tech_mesh.setUpdateTime(" .. sensorId .. ", " .. updateTime .. ")")
end

local function getRoadGraph()
  local rawCoords = map.getGraphpath().positions
  local coords = {}
  for k, v in pairs(rawCoords) do
    coords[k] = { v.x, v.y, v.z }
  end
  local normals = {}
  local rawNodes = map.getMap().nodes
  for k, v in pairs(rawNodes) do
    local n = v.normal
    normals[k] = { n.x, n.y, n.z }
  end
  return {
    graph = map.getGraphpath()['graph'],
    coords = coords,
    widths = map.getGraphpath().radius,
    normals = normals }
end

local function resetNavgraph()
  map.reset()
end

local function createValidation(vid, testId)
  local args = {}
  args.pos = vec3(0.0, 0.2575, 0.504)
  args.dir = vec3(0.0086, -0.9847, -0.1739)
  args.up = vec3(-0.0017, -0.1739, 0.9848)
  args.GFXUpdateTime = 0.00001
  args.isUseGravity = true
  args.isVisualised = true
  args.isSnappingDesired = false
  args.isForceInsideTriangle = false
  args.isAllowWheelNodes = false
  args.physicsUpdateTime = 0.00001
  args.accelWindowWidth = 10.0
  args.gyroWindowWidth = 2.0

  -- Attach the sensor to the vehicle.
  local sensorId = Research.SensorManager.getNewSensorId()
  Research.SensorMatrixManager.attachSensor(sensorId, args.pos, args.dir, args.up, vid, false, args.isSnappingDesired,
    args.isForceInsideTriangle, args.isAllowWheelNodes, args.isDirWorldSpace)
  local attachData = Research.SensorMatrixManager.getAttachData(sensorId)

  -- Create the AdvancedIMU in vlua.
  local data =
  {
    testId = testId,
    sensorId = sensorId,
    GFXUpdateTime = args.GFXUpdateTime,
    physicsUpdateTime = args.physicsUpdateTime,
    isUsingGravity = args.isUseGravity,
    nodeIndex1 = attachData['nodeIndex1'],
    nodeIndex2 = attachData['nodeIndex2'],
    nodeIndex3 = attachData['nodeIndex3'],
    u = attachData['u'],
    v = attachData['v'],
    signedProjDist = attachData['signedProjDist'],
    triangleSpaceForward = attachData['triangleSpaceForward'],
    triangleSpaceUp = attachData['triangleSpaceUp'],
    isVisualised = args.isVisualised,
    accelWindowWidth = args.accelWindowWidth,
    gyroWindowWidth = args.gyroWindowWidth,
    accelFrequencyCutoff = args.accelFrequencyCutoff,
    gyroFrequencyCutoff = args.gyroFrequencyCutoff
  }
  local serializedData = string.format("extensions.tech_validation.create(%q)", lpack.encode(data))
  be:queueObjectLua(vid, serializedData)
end

local function removeValidation(vid, sensorId)
  local vehicleId = scenetree.findObject(vid):getID()
  be:queueObjectLua(vehicleId, "extensions.tech_validation.remove(" .. sensorId .. ")")
end

local function isTimeEvolutionComplete(vid)
  return isVehicleFeedingComplete
end

local function markVehicleFeedingComplete()
  isVehicleFeedingComplete = true
  dump('Test complete!')
end

local function createTyreBarrierTest(vid, IMUPos, IMUDir, IMUUp, accelWindow, gyroWindow, initialVel, startPos)
  local args = {}
  args.pos = IMUPos or vec3(0, 0, 0)
  args.dir = IMUDir or vec3(0, -1, 0)
  args.up = IMUUp or vec3(0, 0, 1)
  args.GFXUpdateTime = 0.00001
  args.isUseGravity = true
  args.isVisualised = true
  args.isSnappingDesired = false
  args.isForceInsideTriangle = false
  args.isAllowWheelNodes = false
  args.physicsUpdateTime = 0.00001
  args.accelWindowWidth = accelWindow or 10.0
  args.gyroWindowWidth = gyroWindow or 2.0
  args.initialVel = initialVel
  args.startPos = startPos or vec3(0, 0, 0)

  -- Attach the sensor to the vehicle.
  local sensorId = Research.SensorManager.getNewSensorId()
  Research.SensorMatrixManager.attachSensor(sensorId, args.pos, args.dir, args.up, vid, false, args.isSnappingDesired,
    args.isForceInsideTriangle, args.isAllowWheelNodes, args.isDirWorldSpace)
  local attachData = Research.SensorMatrixManager.getAttachData(sensorId)

  -- Create the AdvancedIMU in vlua.
  local data =
  {
    sensorId = sensorId,
    GFXUpdateTime = args.GFXUpdateTime,
    physicsUpdateTime = args.physicsUpdateTime,
    isUsingGravity = args.isUseGravity,
    nodeIndex1 = attachData.nodeIndex1, nodeIndex2 = attachData.nodeIndex2, nodeIndex3 = attachData.nodeIndex3,
    u = attachData.u, v = attachData.v,
    signedProjDist = attachData.signedProjDist,
    triangleSpaceForward = attachData.triangleSpaceForward,
    triangleSpaceUp = attachData.triangleSpaceUp,
    isVisualised = args.isVisualised,
    accelWindowWidth = args.accelWindowWidth, gyroWindowWidth = args.gyroWindowWidth,
    accelFrequencyCutoff = args.accelFrequencyCutoff, gyroFrequencyCutoff = args.gyroFrequencyCutoff,
    initVel = args.initialVel,
    posX = args.startPos.x, posY = args.startPos.y, posZ = args.startPos.z
  }
  local serializedData = string.format("extensions.tech_tyreBarrier.create(%q)", lpack.encode(data))
  be:queueObjectLua(vid, serializedData)
end

local function removeTyreBarrierTest(vid, sensorId)
  local vehicleId = scenetree.findObject(vid):getID()
  be:queueObjectLua(vehicleId, "extensions.tech_tyreBarrier.remove(" .. sensorId .. ")")
end

local function onUpdate(dtReal, dtSim, dtRaw)

  -- Manage all the Raw LiDAR sensor updates.
  for _, d in pairs(rawLidarData) do

    -- Dispatch new poll requests for each sensor in the set.
    local id1, id2, id3, id4 = d.sensorId1, d.sensorId2, d.sensorId3, d.sensorId4
    table.insert(d.requests1, Research.GpuRequestManager.sendAdHocCameraGpuRequest(id1))
    table.insert(d.requests2, Research.GpuRequestManager.sendAdHocCameraGpuRequest(id2))
    table.insert(d.requests3, Research.GpuRequestManager.sendAdHocCameraGpuRequest(id3))
    table.insert(d.requests4, Research.GpuRequestManager.sendAdHocCameraGpuRequest(id4))

    -- Collect the most-recently completed set of poll request.
    if #d.requests1 > 0 then
      local req1, req2, req3, req4 = d.requests1[1], d.requests2[1], d.requests3[1], d.requests4[1]
      if isRequestComplete(req1) and isRequestComplete(req2) and isRequestComplete(req3) and isRequestComplete(req4) then
        -- Collect the completed requests from each sensor in the set [Note: the data here is in binary string format].
        cameraBufferDepth[id1], cameraBufferAnnotations[id1] = cameraBufferDepth[id1] or buffer.new(), cameraBufferAnnotations[id1] or buffer.new()
        Research.GpuRequestManager.collectAdHocCameraGpuRequestBuffer(req1, cameraBufferDepth[id1], cameraBufferAnnotations[id1])
        cameraBufferDepth[id2], cameraBufferAnnotations[id2] = cameraBufferDepth[id2] or buffer.new(), cameraBufferAnnotations[id2] or buffer.new()
        Research.GpuRequestManager.collectAdHocCameraGpuRequestBuffer(req2, cameraBufferDepth[id2], cameraBufferAnnotations[id2])
        cameraBufferDepth[id3], cameraBufferAnnotations[id3] = cameraBufferDepth[id3] or buffer.new(), cameraBufferAnnotations[id3] or buffer.new()
        Research.GpuRequestManager.collectAdHocCameraGpuRequestBuffer(req3, cameraBufferDepth[id3], cameraBufferAnnotations[id3])
        cameraBufferDepth[id4], cameraBufferAnnotations[id4] = cameraBufferDepth[id4] or buffer.new(), cameraBufferAnnotations[id4] or buffer.new()
        Research.GpuRequestManager.collectAdHocCameraGpuRequestBuffer(req4, cameraBufferDepth[id4], cameraBufferAnnotations[id4])

        -- Send the data to the user's lua update callback.
        d.fileRef.onUpdate(
          dtSim,
          cameraBufferDepth[id1], cameraBufferDepth[id2], cameraBufferDepth[id3], cameraBufferDepth[id4],
          cameraBufferAnnotations[id1], cameraBufferAnnotations[id2], cameraBufferAnnotations[id3], cameraBufferAnnotations[id4])

        -- Pop the request queues for each sensor in the set, now that the requests have been handled.
        table.remove(d.requests1, 1)
        table.remove(d.requests2, 1)
        table.remove(d.requests3, 1)
        table.remove(d.requests4, 1)
      end
    end
  end

  -- Perform visualisation for all ultrasonic sensors which require it.
  for sensorId, _ in pairs(visualisedUltrasonicSensors) do
    visualiseUltrasonicSensor(sensorId, dtSim)
  end
end

local function onDeserialized(data)
  if Research then
    Research.GpuRequestManager.reset()                                        -- On a Lua reload, we need to re-compute the GPU scheduler, since sensor parameters may have changed.
  end
end

local function onVehicleDestroyed(vid)
  removeAllSensorsFromVehicle(vid)                                            -- Removes any sensors attached to the destroyed vehicle.
end


-- Public interface:

-- General sensor functions.
M.doesSensorExist                           = doesSensorExist
M.removeSensor                              = removeSensor
M.removeAllSensorsFromVehicle               = removeAllSensorsFromVehicle

-- GPU manager functions.
M.getAverageUpdateTime                      = getAverageUpdateTime
M.getMaxLoadPerFrame                        = getMaxLoadPerFrame
M.setMaxLoadPerFrame                        = setMaxLoadPerFrame

-- Ad-hoc sensor reading functions (for C++ managed sensors).
M.getFullCameraRequest                      = getFullCameraRequest                                  -- TODO This hack should be replaced when GE-2170 is complete.
M.sendCameraRequest                         = sendCameraRequest
M.sendLidarRequest                          = sendLidarRequest
M.sendUltrasonicRequest                     = sendUltrasonicRequest
M.sendRadarRequest                          = sendRadarRequest
M.collectCameraRequest                      = collectCameraRequest
M.collectLidarRequest                       = collectLidarRequest
M.collectUltrasonicRequest                  = collectUltrasonicRequest
M.collectRadarRequest                       = collectRadarRequest
M.isRequestComplete                         = isRequestComplete

-- Ad-hoc sensor reading functions (for Lua sensors with a vlua controller).
M.sendAdvancedIMURequest                    = sendAdvancedIMURequest
M.collectAdvancedIMURequest                 = collectAdvancedIMURequest
M.sendGPSRequest                            = sendGPSRequest
M.collectGPSRequest                         = collectGPSRequest
M.sendPowertrainRequest                     = sendPowertrainRequest
M.collectPowertrainRequest                  = collectPowertrainRequest
M.sendIdealRADARRequest                     = sendIdealRADARRequest
M.collectIdealRADARRequest                  = collectIdealRADARRequest
M.sendRoadsSensorRequest                    = sendRoadsSensorRequest
M.collectRoadsSensorRequest                 = collectRoadsSensorRequest
M.sendMeshRequest                           = sendMeshRequest
M.collectMeshRequest                        = collectMeshRequest
M.isVluaRequestComplete                     = isVluaRequestComplete                                 -- this query is generic to any request from vlua.

-- Sensor matrix manager functions.
M.attachSensor                              = attachSensor
M.getSensorMatrix                           = getSensorMatrix
M.getWorldFrame                             = getWorldFrame
M.getLocalFrame                             = getLocalFrame

-- Vehicle mesh functions (direct from gameengine).
M.getBeamData                               = getBeamData
M.getFullTriangleData                       = getFullTriangleData
M.getWheelTriangleData                      = getWheelTriangleData
M.getNodePositions                          = getNodePositions
M.getClosestMeshPointToGivenPoint           = getClosestMeshPointToGivenPoint
M.getClosestTriangle                        = getClosestTriangle

-- Camera-specific sensor functions.
M.createCamera                              = createCamera
M.createCameraWithSharedMemory              = createCameraWithSharedMemory
M.getCameraImage                            = getCameraImage
M.getCameraAnnotations                      = getCameraAnnotations
M.getCameraDepth                            = getCameraDepth
M.getCameraData                             = getCameraData                                         -- returns a binary string.
M.getCameraDataShmem                        = getCameraDataShmem
M.processCameraData                         = processCameraData                                     -- returns processed data.
M.getCameraSensorPosition                   = getCameraSensorPosition
M.getCameraSensorDirection                  = getCameraSensorDirection
M.getCameraMaxPendingGpuRequests            = getCameraMaxPendingGpuRequests
M.getCameraRequestedUpdateTime              = getCameraRequestedUpdateTime
M.getCameraUpdatePriority                   = getCameraUpdatePriority
M.setCameraSensorPosition                   = setCameraSensorPosition                               -- Camera property setters (for existing camera sensors).
M.setCameraSensorDirection                  = setCameraSensorDirection
M.setCameraSensorUp                         = setCameraSensorUp
M.setCameraSensorResolution                 = setCameraSensorResolution
M.setCameraSensorFOVX                       = setCameraSensorFOVX
M.setCameraSensorNearFarPlanes              = setCameraSensorNearFarPlanes
M.setCameraMaxPendingGpuRequests            = setCameraMaxPendingGpuRequests
M.setCameraRequestedUpdateTime              = setCameraRequestedUpdateTime
M.setCameraUpdatePriority                   = setCameraUpdatePriority
M.convertWorldPointToPixel                  = convertWorldPointToPixel

-- Raw LiDAR-specific sensor functions.
M.createRawLidar                            = createRawLidar                                        -- Create up a new Raw LiDAR sensor (to be managed in a special way).

-- LiDAR-specific sensor functions.
M.createLidar                               = createLidar
M.createLidarWithSharedMemory               = createLidarWithSharedMemory
M.getLidarPointCloud                        = getLidarPointCloud                                    -- returns a binary string.
M.getLidarColourData                        = getLidarColourData                                    -- returns a binary string.
M.getLidarPointCloudShmem                   = getLidarPointCloudShmem
M.getLidarColourDataShmem                   = getLidarColourDataShmem
M.getLidarDataPositions                     = getLidarDataPositions                                 -- returns the LiDAR point cloud positions (processed data).
M.getActiveLidarSensors                     = getActiveLidarSensors
M.getLidarSensorPosition                    = getLidarSensorPosition
M.getLidarSensorDirection                   = getLidarSensorDirection
M.getLidarVerticalResolution                = getLidarVerticalResolution
M.getLidarFrequency                         = getLidarFrequency
M.getLidarMaxDistance                       = getLidarMaxDistance
M.getLidarIsVisualised                      = getLidarIsVisualised
M.getLidarIsAnnotated                       = getLidarIsAnnotated
M.getLidarMaxPendingGpuRequests             = getLidarMaxPendingGpuRequests
M.getLidarRequestedUpdateTime               = getLidarRequestedUpdateTime
M.getLidarUpdatePriority                    = getLidarUpdatePriority
M.setLidarVerticalResolution                = setLidarVerticalResolution
M.setLidarFrequency                         = setLidarFrequency
M.setLidarMaxDistance                       = setLidarMaxDistance
M.setLidarIsVisualised                      = setLidarIsVisualised
M.setLidarIsAnnotated                       = setLidarIsAnnotated
M.setLidarMaxPendingGpuRequests             = setLidarMaxPendingGpuRequests
M.setLidarRequestedUpdateTime               = setLidarRequestedUpdateTime
M.setLidarUpdatePriority                    = setLidarUpdatePriority

-- Ultrasonic-specific sensor functions.
M.createUltrasonic                          = createUltrasonic
M.getUltrasonicReadings                     = getUltrasonicReadings
M.getActiveUltrasonicSensors                = getActiveUltrasonicSensors
M.getUltrasonicIsVisualised                 = getUltrasonicIsVisualised
M.getUltrasonicMaxPendingGpuRequests        = getUltrasonicMaxPendingGpuRequests
M.getUltrasonicRequestedUpdateTime          = getUltrasonicRequestedUpdateTime
M.getUltrasonicUpdatePriority               = getUltrasonicUpdatePriority
M.setUltrasonicIsVisualised                 = setUltrasonicIsVisualised
M.getUltrasonicSensorPosition               = getUltrasonicSensorPosition
M.getUltrasonicSensorDirection              = getUltrasonicSensorDirection
M.getUltrasonicSensorRadius                 = getUltrasonicSensorRadius
M.setUltrasonicMaxPendingGpuRequests        = setUltrasonicMaxPendingGpuRequests
M.setUltrasonicRequestedUpdateTime          = setUltrasonicRequestedUpdateTime
M.setUltrasonicUpdatePriority               = setUltrasonicUpdatePriority

-- RADAR-specific sensor functions.
M.createRadar                               = createRadar
M.getRadarReadings                          = getRadarReadings
M.getRadarPPIData                           = getRadarPPIData
M.getRadarRangeDopplerData                  = getRadarRangeDopplerData
M.getActiveRadarSensors                     = getActiveRadarSensors
M.getRadarMaxPendingGpuRequests             = getRadarMaxPendingGpuRequests
M.getRadarRequestedUpdateTime               = getRadarRequestedUpdateTime
M.getRadarUpdatePriority                    = getRadarUpdatePriority
M.getRadarSensorPosition                    = getRadarSensorPosition
M.getRadarSensorDirection                   = getRadarSensorDirection
M.setRadarMaxPendingGpuRequests             = setRadarMaxPendingGpuRequests
M.setRadarRequestedUpdateTime               = setRadarRequestedUpdateTime
M.setRadarUpdatePriority                    = setRadarUpdatePriority

-- Advanced IMU-specific sensor functions.
M.createAdvancedIMU                         = createAdvancedIMU
M.removeAdvancedIMU                         = removeAdvancedIMU
M.getAdvancedIMUReadings                    = getAdvancedIMUReadings
M.updateAdvancedIMULastReadings             = updateAdvancedIMULastReadings
M.updateAdvancedIMUAdHocRequest             = updateAdvancedIMUAdHocRequest
M.setAdvancedIMUUpdateTime                  = setAdvancedIMUUpdateTime
M.setAdvancedIMUIsUsingGravity              = setAdvancedIMUIsUsingGravity
M.setAdvancedIMUIsVisualised                = setAdvancedIMUIsVisualised

-- GPS-specific sensor functions.
M.createGPS                                 = createGPS
M.removeGPS                                 = removeGPS
M.getGPSReadings                            = getGPSReadings
M.updateGPSLastReadings                     = updateGPSLastReadings
M.updateGPSAdHocRequest                     = updateGPSAdHocRequest
M.setGPSUpdateTime                          = setGPSUpdateTime
M.setGPSIsVisualised                        = setGPSIsVisualised

-- Powertrain-specific sensor functions.
M.createPowertrainSensor                    = createPowertrainSensor
M.removePowertrainSensor                    = removePowertrainSensor
M.getPowertrainReadings                     = getPowertrainReadings
M.updatePowertrainLastReadings              = updatePowertrainLastReadings
M.updatePowertrainAdHocRequest              = updatePowertrainAdHocRequest
M.setPowertrainUpdateTime                   = setPowertrainUpdateTime

-- Ideal RADAR-specific sensor functions.
M.createIdealRADARSensor                    = createIdealRADARSensor
M.removeIdealRADARSensor                    = removeIdealRADARSensor
M.getIdealRADARReadings                     = getIdealRADARReadings
M.updateIdealRADARLastReadings              = updateIdealRADARLastReadings
M.updateIdealRADARAdHocRequest              = updateIdealRADARAdHocRequest
M.setIdealRADARUpdateTime                   = setIdealRADARUpdateTime

-- Roads-specific sensor functions.
M.createRoadsSensor                         = createRoadsSensor
M.removeRoadsSensor                         = removeRoadsSensor
M.getRoadsSensorReadings                    = getRoadsSensorReadings
M.updateRoadsSensorLastReadings             = updateRoadsSensorLastReadings
M.updateRoadsSensorAdHocRequest             = updateRoadsSensorAdHocRequest
M.setRoadsSensorUpdateTime                  = setRoadsSensorUpdateTime

-- Mesh-sensor specific sensor functions.
M.createMeshSensor                          = createMeshSensor
M.removeMeshSensor                          = removeMeshSensor
M.updateMeshAdHocRequest                    = updateMeshAdHocRequest
M.setMeshUpdateTime                         = setMeshUpdateTime

-- Road-related functions.
M.getRoadGraph                              = getRoadGraph
M.resetNavgraph                             = resetNavgraph

-- Test/validation tools.
M.createValidation                          = createValidation
M.removeValidation                          = removeValidation
M.markVehicleFeedingComplete                = markVehicleFeedingComplete
M.isTimeEvolutionComplete                   = isTimeEvolutionComplete
M.createTyreBarrierTest                     = createTyreBarrierTest
M.removeTyreBarrierTest                     = removeTyreBarrierTest

-- Functions triggered by hooks.
M.onUpdate                                  = onUpdate
M.onDeserialized                            = onDeserialized
M.onVehicleDestroyed                        = onVehicleDestroyed

return M