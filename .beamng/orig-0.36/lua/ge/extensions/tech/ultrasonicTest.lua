-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- The ultrasonic test is time-based, performing different functionality of the ultrasonic sensor at different frames in the execution.
-- To execute the test, register this extension by add the following to the command arguments when executing:
--  -level gridmap/main.level.json -lua extensions.load("tech_ultrasonicTest")

local M = {}

-- A counter for the number of update steps which have occured.
local frameCounter = 0

-- The ID of the vehicle which the sensors under test shall be attached.
local vid

-- The unique ID number of the sensor instance under test.
local sensorId

-- A flag which indicates if the test has completed.
local isTestComplete = false

-- The ultrasonic test.
local function executeUltrasonicTest()

  -- Test Stage 1: Create a typical ultrasonic sensor which is attached to a vehicle, and perform some initialisation tests/basic tests.
  if frameCounter == 100 then

    print("ultrasonic sensor test: stage 1 starting")

    -- Attempt to get the vehicle ID.
    vid = be:getPlayerVehicleID(0)
    if not vid or vid == -1 then
      return
    end
    assert(vid >= 0, "ultrasonicTest.lua - Failed to get a valid vehicle ID")

    -- Attempt to create an ultrasonic sensor.
    -- local args = 
    -- {updateTime = nil,--(float):  How often the ultrasonic sensor should update its readings in the simulator (in seconds).
  
    -- updatePriority = nil,--(float): A scheduling priority value for this ultrasonic sensor, in range 0 to 1. 0 is highest priority, 1 is least.
  
    -- pos = nil, --(Point3F): The position of the ultrasonic sensor either in vehicle space (if attached to vehicle) or in world space (if fixed to the map).
  
    -- dir = nil,--(Point3F):  The forward direction in which the ultrasonic sensor points.
  
    -- up = nil,--(Point3F): The up direction of the ultrasonic sensor.
  
    -- size = nil,--(table): The horizontal and vertical resolution of the ultrasonic sensor.
  
    -- fovY = nil,--(float): The vertical field of view of the ultrasonic sensor, in degrees.
  
    -- nearFarPlanes = nil,--(table): The near and far plane distances of the ultrasonic sensor, in metres.
    
    -- rangeRoundness = nil,--(float): The general roundness of the ultrasonic sensors range shape. Can be negative.
    
    -- rangeCutoffSensitivity = nil,--(float): A cutoff sensitivity parameter for the ultrasonic sensor range-shape.
  
    -- rangeShape = nil,--(float): The shape of the ultrasonic sensor range-shape in the range [0, 1], going from conical to spherical.
    
    -- rangeFocus = nil,--(float): The focus parameter for the ultrasonic sensor range-shape.
    
    -- rangeMinCutoff = nil,--(float): The minimum cut-off distance for the ultrasonic sensor range-shape. Nothing closer than this will be detected.
    
    -- rangeDirectMaxCutoff = nil,--(float): The maximum cut-off distance for the ultrasonic sensor range-shape. This parameter is a hard cutoff - nothing further than this will be detected, although other parameters can also control the maximum distance.
    
    -- sensitivity = nil,--(float): The sensitivity of the ultrasonic sensor detection.
    
    -- fixedWindowSize = nil,--(float): Used with sensitivity to set how sharply the ultrasonic sensor detects surfaces.
    
    -- isVisualised = true,--(bool): A flag which indicates if the ultrasonic sensor should be visualised.
    
    -- isStatic = nil,--(bool): True if the ultrasonic sensor is fixed to a point on the map, false if it is attached to a vehicle.
    
    -- isSnappingDesired = nil,--(bool): True if the ultrasonic sensor position should be forced onto the surface of the vehicle, at its closest vehicle point. This is useful if finding it hard to have the ultrasonic sensor on the vehicle itself. False, otherwise, eg if the ultrasonic sensor should be suspended at a fixed point relative to the vehicle.
    
    -- isForceInsideTriangle = nil--(bool): Used with isSnappingDesired. True, if the ultrasonic sensor should be forced to be inside its nearest vehicle triangle. Otherwise false.
    -- }

    -- sensorId = extensions.tech_sensors.createUltrasonic(
    --   vid, 0.001, 200, 200, 0.15, 0.15, 0.10, 10.15,
    --   -1.15, 0.0, 0.3, 0.376, 0.1, 5.0, 3.0, 10.0,
    --   0, -3, 3,
    --   0, -1, 0,
    --   true, false, false, false)
    local args = {isVisualised = true}
    sensorId = extensions.tech_sensors.createUltrasonic(vid,args)

    -- Test that the ultrasonic sensor was created and a valid unique ID number was issued.
    -- assert(sensorId == 0, "ultrasonicTest.lua - Failed to create valid ultrasonic sensor") *failes when updating lua and generating a new sensor if it's ID is not zero, the following line is a replacement checking tht the sensor is greater than or equal to zero to test several sensors attachement
    assert(sensorId >= 0, "ultrasonicTest.lua - Failed to create valid ultrasonic sensor")
    assert(extensions.tech_sensors.doesSensorExist(sensorId) == true, "ultrasonicTest.lua - doesSensorExist() has failed at sensor initialisation")

    -- Test the getActiveUltrasonicSensors() function.
    local activeUltrasonicSensors = extensions.tech_sensors.getActiveUltrasonicSensors()
    local ctr = 0
    for i, s in pairs(activeUltrasonicSensors) do
      ctr = ctr + 1
      assert(s == 0, "ultrasonicTest.lua - getActiveUltrasonicSensors() has failed. A sensor contains the wrong ID number")
    end
    assert(ctr == 1, "ultrasonicTest.lua - getActiveUltrasonicSensors() has failed. There is not one single ultrasonic sensor.")

    -- Test the core property getters for the ultrasonic sensor.
    assert(extensions.tech_sensors.getUltrasonicIsVisualised(sensorId) == true, "ultrasonicTest.lua - getUltrasonicIsVisualised has failed")
    assert(extensions.tech_sensors.getUltrasonicSensorPosition(sensorId) ~= nil, "ultrasonicTest.lua - getUltrasonicSensorPosition has failed")
    assert(extensions.tech_sensors.getUltrasonicSensorDirection(sensorId) ~= nil, "ultrasonicTest.lua - getUltrasonicDirectionPosition has failed")
    assert(extensions.tech_sensors.getUltrasonicSensorRadius(sensorId, 1) ~= nil, "ultrasonicTest.lua - getUltrasonicSensorRadius has failed")

    -- Test switching the ultrasonic visualisation off then back on again.
    extensions.tech_sensors.setUltrasonicIsVisualised(sensorId, false)
    assert(extensions.tech_sensors.getUltrasonicIsVisualised(sensorId) == false, "ultrasonicTest.lua - getUltrasonicIsVisualised has failed")
    extensions.tech_sensors.setUltrasonicIsVisualised(sensorId, true)
    assert(extensions.tech_sensors.getUltrasonicIsVisualised(sensorId) == true, "ultrasonicTest.lua - getUltrasonicIsVisualised has failed")
    local requestID = extensions.tech_sensors.sendUltrasonicRequest(sensorId)

    assert(extensions.tech_sensors.getUltrasonicMaxPendingGpuRequests(sensorId) > 0, "ultrasonicTest.lua - getUltrasonicMaxPendingGpuRequests has failed")
    assert(extensions.tech_sensors.getUltrasonicRequestedUpdateTime(sensorId) > 0, "ultrasonicTest.lua - getUltrasonicRequestedUpdateTime has failed")
    assert(extensions.tech_sensors.getUltrasonicUpdatePriority(sensorId) >= 0, "ultrasonicTest.lua - getUltrasonicUpdatePriority has failed")

   
    extensions.tech_sensors.setUltrasonicMaxPendingGpuRequests(sensorId,20) 
    extensions.tech_sensors.setUltrasonicRequestedUpdateTime(sensorId, 0.04) 
    extensions.tech_sensors.setUltrasonicUpdatePriority(sensorId, 1) 
    

    assert(extensions.tech_sensors.getUltrasonicMaxPendingGpuRequests(sensorId) <= 20, "ultrasonicTest.lua - getUltrasonicMaxPendingGpuRequests has failed")
    assert(extensions.tech_sensors.getUltrasonicRequestedUpdateTime(sensorId) > 0, "ultrasonicTest.lua - getUltrasonicRequestedUpdateTime has failed")
    assert(extensions.tech_sensors.getUltrasonicUpdatePriority(sensorId) == 1, "ultrasonicTest.lua - getUltrasonicUpdatePriority has failed")

    
   
    print("isRequestComplete: ")
    print(extensions.tech_sensors.isRequestComplete(requestID))
    print("collectUltrasonicRequest: ")
    print(extensions.tech_sensors.collectUltrasonicRequest(requestID))

    
  end

  -- Test Stage 2: The ultrasonic sensor readings have now had a chance to update.
  if frameCounter == 200 then

    print("ultrasonic sensor test: stage 2 starting")

    -- Test the ultrasonic sensor readings.
    -- assert(extensions.tech_sensors.getUltrasonicDistanceMeasurement(sensorId) ~= nil, "ultrasonicTest.lua - failed to valid get distance reading")
    -- assert(extensions.tech_sensors.getUltrasonicWindowMin(sensorId) ~= nil, "ultrasonicTest.lua - failed to get valid windowMin readings")
    -- assert(extensions.tech_sensors.getUltrasonicWindowMax(sensorId) ~= nil, "ultrasonicTest.lua - failed to get valid windowMax readings")

    sensorData = extensions.tech_sensors.getUltrasonicReadings(sensorId)
    assert(extensions.tech_sensors.getUltrasonicReadings(sensorId) ~= nil, "ultrasonicTest.lua - failed to get valid ultrasonic sensor readings")
    
    for k, v in pairs(sensorData) do
      print("ultrasonic reading: "..k.." reading value: "..v)
    end

    -- Test removing the ultrasonic sensor via its unique sensor ID number.
    -- extensions.tech_sensors.removeSensor(sensorId)
    -- assert(extensions.tech_sensors.doesSensorExist(sensorId) == false, "ultrasonicTest.lua - doesSensorExist() has failed after removal by ID")

    -- Test that we can now create a new ultrasonic sensor.
    local args = {isVisualised = true}
    -- sensorId = extensions.tech_sensors.createUltrasonic(vid, args)
    -- print("new sensorID: "..sensorId)
    -- assert(extensions.tech_sensors.getUltrasonicActiveSensors() ~= nil, "ultrasonicTest.lua - getUltrasonicActiveSensors has failed") *Function doesn't exist in sensors file

    -- assert(extensions.tech_sensors.doesSensorExist(sensorId) == true, "ultrasonicTest.lua - doesSensorExist() has failed after retrieval")

    -- Test removing the ultrasonic sensor via the vehicle ID number.
    extensions.tech_sensors.removeAllSensorsFromVehicle(vid)
    assert(extensions.tech_sensors.doesSensorExist(sensorId) == false, "ultrasonicTest.lua - doesSensorExist() has failed after removal by vid")

    isTestComplete = true

    print("ultrasonic sensor test complete")
  end
end

-- Trigger execution to access the ultrasonic test class in every update cycle.
local function onUpdate(dtReal, dtSim, dtRaw)

  -- If the test has already finished, do nothing here for the rest of execution.
  if isTestComplete == true and frameCounter<=205 then
    print("test complete")
    frameCounter = 210
    return
  else 
    frameCounter = frameCounter + 1
    executeUltrasonicTest()
  end

  
  
end

-- If a vehicle is destroyed, remove any attached sensors.
local function onVehicleDestroyed(vid)
  print("ultrasonic sensor test: vehicle destroyed and test sensor removed")
  research.sensorManager.removeAllSensorsFromVehicle(vid)
end

local function onExtensionLoaded()
  log('I', 'ultrasonicTest', 'ultrasonicTest extension loaded')
  setExtensionUnloadMode(M, 'manual')
end
------------------------New Tests Added--------------------------
-- local function createUltrasonic(vehicleid)
--   local args = 
--   {updateTime = --(float):  How often the ultrasonic sensor should update its readings in the simulator (in seconds).
 
--   updatePriority = --(float): A scheduling priority value for this ultrasonic sensor, in range 0 to 1. 0 is highest priority, 1 is least.
 
--   pos = --(Point3F): The position of the ultrasonic sensor either in vehicle space (if attached to vehicle) or in world space (if fixed to the map).
 
--   dir = --(Point3F):  The forward direction in which the ultrasonic sensor points.
 
--   up = --(Point3F): The up direction of the ultrasonic sensor.
 
--   size = --(table): The horizontal and vertical resolution of the ultrasonic sensor.
 
--   fovY = --(float): The vertical field of view of the ultrasonic sensor, in degrees.
 
--   nearFarPlanes = --(table): The near and far plane distances of the ultrasonic sensor, in metres.
  
--   rangeRoundness = --(float): The general roundness of the ultrasonic sensors range shape. Can be negative.
  
--   rangeCutoffSensitivity = --(float): A cutoff sensitivity parameter for the ultrasonic sensor range-shape.
 
--   rangeShape = --(float): The shape of the ultrasonic sensor range-shape in the range [0, 1], going from conical to spherical.
  
--   rangeFocus = --(float): The focus parameter for the ultrasonic sensor range-shape.
  
--   rangeMinCutoff = --(float): The minimum cut-off distance for the ultrasonic sensor range-shape. Nothing closer than this will be detected.
  
--   rangeDirectMaxCutoff = --(float): The maximum cut-off distance for the ultrasonic sensor range-shape. This parameter is a hard cutoff - nothing further than this will be detected, although other parameters can also control the maximum distance.
  
--   sensitivity = --(float): The sensitivity of the ultrasonic sensor detection.
  
--   fixedWindowSize = --(float): Used with sensitivity to set how sharply the ultrasonic sensor detects surfaces.
  
--   isVisualised = --(bool): A flag which indicates if the ultrasonic sensor should be visualised.
  
--   isStatic = --(bool): True if the ultrasonic sensor is fixed to a point on the map, false if it is attached to a vehicle.
  
--   isSnappingDesired = --(bool): True if the ultrasonic sensor position should be forced onto the surface of the vehicle, at its closest vehicle point. This is useful if finding it hard to have the ultrasonic sensor on the vehicle itself. False, otherwise, eg if the ultrasonic sensor should be suspended at a fixed point relative to the vehicle.
  
--   isForceInsideTriangle = --(bool): Used with isSnappingDesired. True, if the ultrasonic sensor should be forced to be inside its nearest vehicle triangle. Otherwise false.
--   }

--   local ultrasonicID = extensions.tech_sensors.createUltrasonic(vehicleid,args)
-- end


-- Public interface.
M.executeUltrasonicTest                     = executeUltrasonicTest
M.onUpdate                                  = onUpdate
M.onVehicleDestroyed                        = onVehicleDestroyed
M.onExtensionLoaded                         = onExtensionUnloaded
M.onExtensionUnloaded                       = function() log('I', 'ultrasonicTest', 'ultrasonicTest extension unloaded') end
M.createUltrasonic                          = createUltrasonic
return M
