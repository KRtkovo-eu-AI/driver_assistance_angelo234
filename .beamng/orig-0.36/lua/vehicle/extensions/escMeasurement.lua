-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local min = math.min
local max = math.max
local abs = math.abs
local floor = math.floor
local ceil = math.ceil
local acos = math.acos

local model_key = nil
local config_key = nil
local workerCoroutine = nil

local logTag = "escCalibration"

local highSpeedFactor = 2
local speedFactor = 0

local currentTargetAngle = 0
local steeringSmoother = newTemporalSmoothing(2, 0.2)

local minSpeed = 30 / 3.6
local maxSpeed = 150 / 3.6
local targetSpeedIndex = 0
local testStartingSpeed = maxSpeed + 10
local nextMeasureSpeed = maxSpeed
local targetSpeedStepSize = 2 --m/s

local targetAngleIndex = 0
local minAngle = 0.5
local maxAngle = 20
local targetAngleStepSize = 0.1

local measurementsSkewStiffness = {}
local measurementsSTM = {}
local measurementCounter = 0
local avgMeasurements = {}
local maxSteerAngle = 0
local maxSteerAngleTimer = 0
local maxSteerAngleCounter = 0

local targetSteeringAngleStepSize = 0.1
local targetSteeringAngle = 0

local idleState = "Idling"
local initState = "Initialization"
local startingState = "Starting"
local measureState = "Measuring"
local settleState = "Settling"
local finishedState = "Finished"
local endState = "Testend"

local testState = idleState
local activeTest = 1
local initWaitingTime = 2
local settleTimer = 0
local settleWatchdogTimer = 0
local settleTime = 1

local CMU = nil

-- switches to next vehicle
local function killswitch()
  --print(" === killswitch ===")
  obj:queueGameEngineLua("util_calibrateESC.vehicleDone()")
end

local function watchdogHeartbeat()
  obj:queueGameEngineLua("util_calibrateESC.heartbeat()")
end

local ccIntegral = 0
local ccTargetSpeed
local ccThrottleSmoother = newTemporalSmoothing(0.99, 0.99)
local ccDisableSmoother = newTemporalSmoothing(0.5, 0.5)
local ccHasReachedTargetSpeed = false

local function updateCC(dt)
  local throttleOverride
  ccHasReachedTargetSpeed = false
  if ccTargetSpeed then
    -- Prevents division by zero gravity
    local gravity = obj:getGravity()
    gravity = max(0.1, abs(gravity)) * sign2(gravity)

    local acc = sensors.gy2 / abs(gravity)
    local accError = max(acc - 1.6, 0)

    local currentSpeed = electrics.values.wheelspeed or 0
    local error = ccTargetSpeed - currentSpeed
    ccIntegral = max(min(ccIntegral + error * dt, 1), 0)
    local targetThrottle = max(max(min(error * 0.5 + ccIntegral * 0.02, 1), 0) - accError * 10, 0)
    throttleOverride = ccThrottleSmoother:getUncapped(targetThrottle, dt)
    ccHasReachedTargetSpeed = abs(error) / ccTargetSpeed <= 0.03
    --print(ccHasReachedTargetSpeed)

    ccDisableSmoother:set(throttleOverride)
  else
    ccHasReachedTargetSpeed = true
    throttleOverride = ccDisableSmoother:getUncapped(0, dt)
    if throttleOverride == 0 then
      throttleOverride = nil
    end
  end

  controller.getControllerSafe("vehicleController").setAggressionOverride(1)

  electrics.values.throttleOverride = throttleOverride
end

local function updateGFX(dt)
  if workerCoroutine ~= nil then
    local errorfree, value = coroutine.resume(workerCoroutine)
    if not errorfree then
      log("E", logTag, debug.traceback(workerCoroutine, "workerCoroutine: " .. value))
    end
    watchdogHeartbeat()
    if coroutine.status(workerCoroutine) == "dead" then
      log("I", logTag, "coroutine dead, hitting killswitch")
      killswitch()
      workerCoroutine = nil
      return
    end
  end
end

local function updateUI()
  guihooks.trigger("ESCSkewStiffnessChange", {currentSpeed = 0, currentAngle = currentTargetAngle, state = testState, progress = 0, stiffnessFront = 0, stiffnessRear = 0})
end

local function setState(state)
  testState = state
  updateUI()

  if state == settleState then
    settleTimer = settleTime
  end
end

local function increaseAngle()
  targetAngleIndex = targetAngleIndex + 1
  targetSteeringAngle = clamp(targetSteeringAngle + targetSteeringAngleStepSize, 0, 1)
end

local function increaseSpeed()
  targetSpeedIndex = targetSpeedIndex + 1
end

local function setMaxPhysicsSpeed(enable)
  speedFactor = enable and highSpeedFactor or 0
  obj:queueGameEngineLua("be:setPhysicsSpeedFactor(" .. speedFactor .. ")")
end

local function updateFixedTestSkewStiffness(dt)
  if testState == idleState then
    return
  end

  updateCC(dt)

  currentTargetAngle = minAngle + targetAngleIndex * targetAngleStepSize
  local targetSteeringInput = maxSteerAngle ~= 0 and (currentTargetAngle / maxSteerAngle) or 0
  local steeringInput = steeringSmoother:getUncapped(targetSteeringInput, dt)

  if testState == initState then
    steeringInput = 1
    ccTargetSpeed = 0
    maxSteerAngleTimer = maxSteerAngleTimer + dt
    if maxSteerAngleTimer > initWaitingTime then
      maxSteerAngle = maxSteerAngle + math.deg(CMU.vehicleData.frontWheelAngle)
      maxSteerAngleCounter = maxSteerAngleCounter + 1
      if maxSteerAngleTimer > 2 * initWaitingTime then
        maxSteerAngle = maxSteerAngle / maxSteerAngleCounter
        maxSteerAngleTimer = 0
        maxSteerAngleCounter = 0

        targetAngleIndex = 0
        targetSpeedIndex = 0

        setState(settleState)
      end
    end
  elseif testState == settleState then
    ccTargetSpeed = minSpeed + targetSpeedIndex * targetSpeedStepSize
    settleWatchdogTimer = settleWatchdogTimer + dt
    if ccHasReachedTargetSpeed then
      if targetSteeringInput == steeringInput then
        settleTimer = settleTimer - dt

        if settleTimer <= 0 then
          settleWatchdogTimer = 0
          setState(measureState)
        end
      end
    else
      settleTimer = settleTime
    end
    if settleWatchdogTimer >= 20 then
      settleWatchdogTimer = 0
      if ccTargetSpeed < maxSpeed then
        increaseSpeed()
        targetAngleIndex = 0
        setState(settleState)
      else
        setState(finishedState)
      end
    end
  elseif testState == measureState then
    local velocity = obj:getVelocity()
    local directionVector = obj:getDirectionVector()
    local actualVelocity = (directionVector:dot(velocity) / (directionVector:length() * directionVector:length()) * directionVector):length()
    local velocityVector = vec3(velocity.x, velocity.y, 0)
    local dot = velocityVector:dot(directionVector)
    local floatAngle = acos(min(max(dot / (directionVector:length() * velocityVector:length()), -1), 1))

    local yawRateCalibration = -abs(CMU.sensorHub.yawAV)
    local wheelAngleFrontCalibration = -abs(CMU.vehicleData.frontWheelAngle)
    local wheelAngleRearCalibration = -abs(CMU.vehicleData.rearWheelAngle)

    local vehicleStats = CMU.vehicleData.vehicleStats

    local stiffnessFront = (vehicleStats.distanceCOGRearAxle * vehicleStats.mass * yawRateCalibration * actualVelocity) / ((vehicleStats.distanceCOGFrontAxle + vehicleStats.distanceCOGRearAxle) * (wheelAngleFrontCalibration - (vehicleStats.distanceCOGFrontAxle * yawRateCalibration / actualVelocity) - floatAngle))
    local stiffnessRear = ((yawRateCalibration * vehicleStats.mass * actualVelocity) - stiffnessFront * (wheelAngleFrontCalibration - (vehicleStats.distanceCOGFrontAxle * yawRateCalibration / actualVelocity) - floatAngle)) / (wheelAngleRearCalibration + (vehicleStats.distanceCOGRearAxle * yawRateCalibration / actualVelocity) - floatAngle)

    avgMeasurements.stiffnessFront = (avgMeasurements.stiffnessFront or 0) + stiffnessFront
    avgMeasurements.stiffnessRear = (avgMeasurements.stiffnessRear or 0) + stiffnessRear

    measurementCounter = measurementCounter + 1

    if measurementCounter == 100 then
      local avgStiffnessFront = avgMeasurements.stiffnessFront / measurementCounter
      local avgStiffnessRear = avgMeasurements.stiffnessRear / measurementCounter

      measurementsSkewStiffness.maxStiffnessFront = max((measurementsSkewStiffness.maxStiffnessFront or 0), avgStiffnessFront)
      measurementsSkewStiffness.maxStiffnessRear = max((measurementsSkewStiffness.maxStiffnessRear or 0), avgStiffnessRear)

      measurementCounter = 0
      avgMeasurements = {}

      if ccTargetSpeed < maxSpeed then
        if currentTargetAngle < maxAngle then
          increaseAngle()
          setState(settleState)
        else
          increaseSpeed()
          targetAngleIndex = 0
          setState(settleState)
        end
      else
        setState(finishedState)
      end
    end
  elseif testState == finishedState then
    currentTargetAngle = 0
    ccTargetSpeed = nil
    setMaxPhysicsSpeed(false)
    setState(endState)
    activeTest = -1
  end

  input.event("steering", steeringInput, 1)
end

local doSteer = false
local calibrationFrameCounter = 0
local calibrationSpeedPoint = 0
local calibrationWheelAnglePoint = 0
local claibrationYawRatePoint = 0
local stmDesiredTestSpeed = 200 / 3.6

local function updateTestSTMMeasurements(dt)
  if testState == idleState then
    return
  end

  updateCC(dt)

  currentTargetAngle = minAngle + targetAngleIndex * targetAngleStepSize
  --local targetSteeringInput = maxSteerAngle ~= 0 and (currentTargetAngle / maxSteerAngle) or 0
  local targetSteeringInput = targetSteeringAngle
  local steeringInput = steeringSmoother:getUncapped(doSteer and targetSteeringInput or 0, dt)

  if testState == initState then
    steeringInput = 1
    ccTargetSpeed = 0
    maxSteerAngleTimer = maxSteerAngleTimer + dt
    if maxSteerAngleTimer > initWaitingTime then
      maxSteerAngle = maxSteerAngle + math.deg(CMU.vehicleData.frontWheelAngle)
      maxSteerAngleCounter = maxSteerAngleCounter + 1
      if maxSteerAngleTimer > 2 * initWaitingTime then
        maxSteerAngle = maxSteerAngle / maxSteerAngleCounter
        maxSteerAngleTimer = 0
        maxSteerAngleCounter = 0

        targetAngleIndex = 0
        targetSpeedIndex = 0

        setState(settleState)
      end
    end
  elseif testState == settleState then
    doSteer = false
    ccTargetSpeed = stmDesiredTestSpeed
    settleWatchdogTimer = settleWatchdogTimer + dt
    if ccHasReachedTargetSpeed or settleWatchdogTimer >= 30 then
      ccTargetSpeed = nil
      doSteer = true
      if targetSteeringInput == steeringInput then
        settleTimer = settleTimer - dt
        if settleTimer <= 0 then
          settleWatchdogTimer = 0
          setState(measureState)
        end
      end
    else
      settleTimer = settleTime
      doSteer = false
    end
    if false and settleWatchdogTimer >= 20 then
      settleWatchdogTimer = 0
      if ccTargetSpeed < maxSpeed then
        increaseSpeed()
        targetAngleIndex = 0
        setState(settleState)
      else
        setState(finishedState)
      end
    end
  elseif testState == measureState then
    doSteer = true

    local speed = electrics.values.wheelspeed
    local bsa = CMU.virtualSensors.reference.bodySlipAngle
    if speed > 1 and abs(bsa) < 0.2 then
      local wheelAngle = CMU.vehicleData.frontWheelAngle
      local yawRate = CMU.sensorHub.yawAV
      calibrationSpeedPoint = calibrationSpeedPoint + speed
      calibrationWheelAnglePoint = calibrationWheelAnglePoint + wheelAngle
      claibrationYawRatePoint = claibrationYawRatePoint + yawRate
      calibrationFrameCounter = calibrationFrameCounter + 1

      if calibrationFrameCounter >= 10 then
        local speedPoint = ceil(calibrationSpeedPoint / calibrationFrameCounter * 5) * 0.2
        local wheelAnglePoint = ceil(calibrationWheelAnglePoint / calibrationFrameCounter * 500) * 0.002
        local yawRatePoint = claibrationYawRatePoint / calibrationFrameCounter
        if abs(wheelAnglePoint) > 0 and abs(yawRatePoint) > 0 then
          measurementsSTM[speedPoint] = measurementsSTM[speedPoint] or {}
          measurementsSTM[speedPoint][wheelAnglePoint] = measurementsSTM[speedPoint][wheelAnglePoint] or {}
          table.insert(measurementsSTM[speedPoint][wheelAnglePoint], yawRatePoint)

        --print(string.format("%.2f -> %.2f, %.3f -> %.2f", bsa, speedPoint, wheelAnglePoint, yawRatePoint))
        --dump(measurementsSTM)
        end
        calibrationSpeedPoint = 0
        calibrationWheelAnglePoint = 0
        claibrationYawRatePoint = 0
        calibrationFrameCounter = 0
      end
    end

    if speed < 1.5 then
      if currentTargetAngle < maxAngle then
        increaseAngle()
        print("new target angle: " .. (minAngle + targetAngleIndex * targetAngleStepSize))
        ccTargetSpeed = stmDesiredTestSpeed
        setState(settleState)
      else
        setState(finishedState)
      end
    end
  elseif testState == finishedState then
    currentTargetAngle = 0
    ccTargetSpeed = nil
    setMaxPhysicsSpeed(false)
    setState(endState)
    activeTest = -1
  end

  input.event("steering", steeringInput, 1)
end

local function updateCallback(dt)
  updateTestSTMMeasurements(dt)
end

local function updateFixedStepCallback(dt)
  updateFixedTestSkewStiffness(dt)
end

local function updateGFXCallback(dt)
end

local function combineTestData()
  -- for _, v in ipairs(measurementsTest1) do
  --   data[v.speed] = data[v.speed] or {}
  --   table.insert(data[v.speed], {frontWheelAngle = v.angle, measuredYawAV = v.yawAV, measuredBodySlipAngle = v.bodySlipAngle})
  -- end

  --TODO combine with test2 data
  if measurementsSkewStiffness.maxStiffnessFront and measurementsSkewStiffness.maxStiffnessRear then
    local filepath = "vehicles/" .. model_key .. "/drivingDynamics/" .. config_key .. ".stat.json"
    local data = {vehicleData = {cornerWheels = {"FR", "FL", "RR", "RL"}}}
    data.vehicleData.skewStiffnessFront = floor(measurementsSkewStiffness.maxStiffnessFront / 1000) * 1000
    data.vehicleData.skewStiffnessRear = floor(measurementsSkewStiffness.maxStiffnessRear / 1000) * 1000
    jsonWriteFile(filepath, data, true)
  end

  if measurementsSTM then
    local lines = {}
    for sp, spData in pairs(measurementsSTM) do
      for wap, wapData in pairs(spData) do
        local avgYawRate = 0
        local yawRateCount = tableSize(wapData)
        for i = 1, yawRateCount do
          avgYawRate = avgYawRate + wapData[i]
        end
        avgYawRate = avgYawRate / yawRateCount
        local lineData = {sp, wap, avgYawRate}
        local line = table.concat(lineData, ",")
        table.insert(lines, line)
      end
    end

    local stringData = table.concat(lines, "\r\n")
    writeFile("STMMeasurements.csv", stringData)
  end

  -- local filePivot = io.open(filepathCSV, "w")
  -- filePivot:write("Angle,Yaw,Body Slip Angle\r\n0,0,0\r\n")

  -- for _, v in pairs(data[10] or {}) do
  --   filePivot:write(string.format("%.0f,%.0f,%.0f\r\n", v.frontWheelAngle * 1000, v.measuredYawAV * 1000, v.measuredBodySlipAngle * 1000))
  -- end

  -- filePivot:close()
end

local function onInit()
  setMaxPhysicsSpeed(false)

  currentTargetAngle = 0
  targetAngleIndex = 0

  measurementsSkewStiffness = {}
  measurementsSTM = {}
  maxSteerAngle = 0
  maxSteerAngleTimer = 0
  maxSteerAngleCounter = 0

  setState(idleState)
end

local function startSkewStiffnessTest()
  log("D", "ESC", "Starting skew stiffness test")

  local CMUControllers = controller.getControllersByType("drivingDynamics/CMU")
  if not CMUControllers or #CMUControllers == 0 or #CMUControllers > 1 then
    log("E", "escMeasurement", "Vehicle does not have a valid ESC/CMU")
    setState(endState)
    return
  end

  CMU = CMUControllers[1]
  CMU.registerCalibrationCallback(M.updateFixedStepCallback, "updateFixedStep")
  --CMU.registerCalibrationCallback(M.updateGFXCallback, "updateGFX")
  CMU.disableSystemsForCalibration()

  activeTest = 1
  onInit()
  setState(initState)
  setMaxPhysicsSpeed(true)
end

local function startSTMMeasurementsTest()
  log("D", "ESC", "Starting STM measurements test")

  local CMUControllers = controller.getControllersByType("drivingDynamics/CMU")
  if not CMUControllers or #CMUControllers == 0 or #CMUControllers > 1 then
    log("E", "escMeasurement", "Vehicle does not have a valid ESC/CMU")
    setState(endState)
    return
  end

  CMU = CMUControllers[1]
  CMU.registerCalibrationCallback(updateCallback, "update")
  --CMU.registerCalibrationCallback(M.updateGFXCallback, "updateGFX")
  CMU.disableSystemsForCalibration()

  activeTest = 1
  onInit()
  setState(initState)
  setMaxPhysicsSpeed(true)
end

local function stopSkewStiffnessTest()
  log("D", "ESC", "Stopping skew stiffness test")
  ccTargetSpeed = nil

  setMaxPhysicsSpeed(false)
  setState(finishedState)
end

local function performTests(vehicleName, configName)
  log("I", logTag, "Performing tests...")
  workerCoroutine =
    coroutine.create(
    function()
      -- save for later usage
      model_key = vehicleName
      config_key = configName
      log("I", logTag, string.format(" *** testing car: %s->%s ***", model_key, config_key))

      -- local timer = HighPerfTimer()
      -- local dt = 0
      -- local fixedDt = 1 / 100

      startSkewStiffnessTest()
      --startSTMMeasurementsTest()

      while testState ~= endState do
        -- dt = dt + timer:stopAndReset() / 1000
        -- if dt >= fixedDt then
        --   if activeTest == 1 then
        --     updateFixedTest1(dt * speedFactor)
        --   elseif activeTest == 2 then
        --   end
        --   coroutine.yield()
        --   dt = 0
        -- end
        coroutine.yield()
      end

      combineTestData()

      stopSkewStiffnessTest()

      log("I", logTag, " *** finished ***")
    end
  )
end

-- public interface
M.onInit = onInit
M.onReset = onInit
M.updateGFX = updateGFX
M.startSkewStiffnessTest = startSkewStiffnessTest
M.stopSkewStiffnessTest = stopSkewStiffnessTest
M.performTests = performTests

M.updateFixedStepCallback = updateFixedStepCallback
M.updateGFXCallback = updateGFXCallback

return M
