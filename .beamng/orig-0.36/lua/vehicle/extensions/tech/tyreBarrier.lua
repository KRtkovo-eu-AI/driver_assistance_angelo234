-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local outputFilename = 'tyre_barrier_test_data.csv'             -- The filename at which to write the output data.
local tyreBarrierController = nil                               -- The tyreBarrier controller instance.
local IMUcontroller = nil                                       -- The 'Advanced IMU' controller instance.
local advancedIMU = nil                                         -- The IMU sensor used with the validation testing.
local IMUSensorId = -1                                          -- The Id of the IMU sensor used with the validation testing.
local cachedData = {}                                           -- The data being collected from the validation testing.
local dataIndex = 1                                             -- The index of the data table, used when appending to it.
local isTestComplete = false

local function create(data)

  local decodedData = lpack.decode(data)

  -- Instantiate the tyreBarrier controller.
  tyreBarrierController = controller.loadControllerExternal(
    'tech/tyreBarrier',
    'tyreBarrier',
    { initVel = decodedData.initVel, posX = decodedData.posX, posY = decodedData.posY, posZ = decodedData.posZ})

  -- Create the IMU controller.
  IMUSensorId = decodedData.sensorId
  local controllerData = {
    testId = decodedData.testId,
    sensorId = decodedData.sensorId,
    GFXUpdateTime = decodedData.GFXUpdateTime,
    physicsUpdateTime = decodedData.physicsUpdateTime,
    nodeIndex1 = decodedData.nodeIndex1,
    nodeIndex2 = decodedData.nodeIndex2,
    nodeIndex3 = decodedData.nodeIndex3,
    u = decodedData.u,
    v = decodedData.v,
    signedProjDist = decodedData.signedProjDist,
    triangleSpaceForward = decodedData.triangleSpaceForward,
    triangleSpaceUp = decodedData.triangleSpaceUp,
    isVisualised = decodedData.isVisualised,
    isUsingGravity = decodedData.isUsingGravity,
    accelWindowWidth = decodedData.accelWindowWidth,
    gyroWindowWidth = decodedData.gyroWindowWidth,
    accelFrequencyCutoff = decodedData.accelFrequencyCutoff,
    gyroFrequencyCutoff = decodedData.gyroFrequencyCutoff,
    isSendImmediately = decodedData.isSendImmediately }
  IMUcontroller = controller.loadControllerExternal('tech/advancedIMU', 'advancedIMU' .. decodedData.sensorId, controllerData)
  advancedIMU = { data = controllerData, controller = IMUcontroller }
end

local function appendLatestReading(latestReading)
  cachedData[dataIndex] = latestReading
  dataIndex = dataIndex + 1
end

local function triggerWriteDataToFile()
  local file = io.open(outputFilename, "w")

  -- Write a top row with the headers for each column.
  file:write(
    't,' ..
    'posX,posY,posZ,' ..
    'velX,velY,velZ,' ..
    'roll,pitch,yaw,' ..
    'IMUTime,' ..
    'IMUMass,' ..
    'IMUAccelRaw0,IMUAccelRaw1,IMUAccelRaw2,' ..
    'IMUAccelSmooth0,IMUAccelSmooth1,IMUAccelSmooth2,' ..
    'IMUAngVelX,IMUAngVelY,IMUAngVelZ,' ..
    'IMUAngVelSmoothX,IMUAngVelSmoothY,IMUAngVelSmoothZ,' ..
    'IMUAngAccelX,IMUAngAccelY,IMUAngAccelZ,' ..
    'IMUPosX,IMUPosY,IMUPosZ,' ..
    'IMUAxis1X,IMUAxis1Y,IMUAxis1Z,' ..
    'IMUAxis2X,IMUAxis2Y,IMUAxis2Z,' ..
    'IMUAxis3X,IMUAxis3Y,IMUAxis3Z,' ..
    'Wh1Speed,Wh2Speed,Wh3Speed,Wh4Speed,' ..
    'Wh1BrkTorque,Wh2BrkTorque,Wh3BrkTorque,Wh4BrkTorque,' ..
    'Wh1DrvTorque,Wh2DrvTorque,Wh3DrvTorque,Wh4DrvTorque,' ..
    'Wh1AngVel,Wh2AngVel,Wh3AngVel,Wh4AngVel,' ..
    'RWA_FL,RWA_FR')
  file:write('\n')

  -- Write each row of numerical data, in turn. We write it out by key, so we can guarantee correct order.
  for _, r in ipairs(cachedData) do
    file:write(r.t .. ",")
    file:write(r.posX .. "," .. r.posY .. "," .. r.posZ .. ",")
    file:write(r.velX .. "," .. r.velY .. "," .. r.velZ .. ",")
    file:write(r.roll .. "," .. r.pitch .. "," .. r.yaw .. ",")
    file:write(r.IMUTime .. ",")
    file:write(r.IMUMass .. ",")
    file:write(r.IMUAccelRaw0 .. "," .. r.IMUAccelRaw1 .. "," .. r.IMUAccelRaw2 .. ",")
    file:write(r.IMUAccelSmooth0 .. "," .. r.IMUAccelSmooth1 .. "," .. r.IMUAccelSmooth2 .. ",")
    file:write(r.IMUAngVelX .. "," .. r.IMUAngVelY .. "," .. r.IMUAngVelZ .. ",")
    file:write(r.IMUAngVelSmoothX .. "," .. r.IMUAngVelSmoothY .. "," .. r.IMUAngVelSmoothZ .. ",")
    file:write(r.IMUAngAccelX .. "," .. r.IMUAngAccelY .. "," .. r.IMUAngAccelZ .. ",")
    file:write(r.IMUPosX .. "," .. r.IMUPosY .. "," .. r.IMUPosZ .. ",")
    file:write(r.IMUAxis1X .. "," .. r.IMUAxis1Y .. "," .. r.IMUAxis1Z .. ",")
    file:write(r.IMUAxis2X .. "," .. r.IMUAxis2Y .. "," .. r.IMUAxis2Z .. ",")
    file:write(r.IMUAxis3X .. "," .. r.IMUAxis3Y .. "," .. r.IMUAxis3Z .. ",")
    file:write(r.Wh1Speed .. "," .. r.Wh2Speed .. "," .. r.Wh3Speed .. "," .. r.Wh4Speed .. ",")
    file:write(r.Wh1BrkTorque .. "," .. r.Wh2BrkTorque .. "," .. r.Wh3BrkTorque .. "," .. r.Wh4BrkTorque .. ",")
    file:write(r.Wh1DrvTorque .. "," .. r.Wh2DrvTorque .. "," .. r.Wh3DrvTorque .. "," .. r.Wh4DrvTorque .. ",")
    file:write(r.Wh1AngVel .. "," .. r.Wh2AngVel .. "," .. r.Wh3AngVel .. "," .. r.Wh4AngVel .. ",")
    file:write(r.RWA_FL .. "," .. r.RWA_FR .. ",")
    file:write('\n')
  end

  file:close()
  dump("Test data written to file: " .. outputFilename)
end

local function getLatestIMUData()
  return extensions.tech_advancedIMU.getAdvancedIMUReading(IMUSensorId)
end

local function markTestComplete()
  obj:queueGameEngineLua("tech_sensors.markVehicleFeedingComplete()")
end

local function updateGFX(dtSim)
  if IMUcontroller ~= nil then
    local data = IMUcontroller.getSensorData()
      obj.debugDrawProxy:drawSphere(0.05, data.currentPos, color(0, 255, 0, 255))
      obj.debugDrawProxy:drawLine(data.currentPos, data.currentPos + data.currentDir, color(0, 255, 0, 255))   -- direction.
  end
end

local function remove(sensorId)
  controller.unloadControllerExternal('advancedIMU' .. sensorId)
  dump("imu controller removed")
  controller.unloadControllerExternal('tyreBarrier' .. sensorId)
  dump("tyreBarrier controller removed")
end


-- Public interface:
M.create                                    = create
M.appendLatestReading                       = appendLatestReading
M.triggerWriteDataToFile                    = triggerWriteDataToFile
M.getLatestIMUData                          = getLatestIMUData
M.markTestComplete                          = markTestComplete
M.updateGFX                                 = updateGFX
M.remove                                    = remove

return M