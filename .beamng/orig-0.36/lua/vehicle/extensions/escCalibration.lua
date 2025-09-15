-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local stiffnessUpdateCounter = 0
local stiffnessFrontSum = 0
local stiffnessRearSum = 0
local currentTargetSpeed = 0
local currentTargetSpeedIndex = 1
local currentTargetAngle = 0
local currentTargetAngleIndex = 1
local targetSpeeds = {40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100}
 --, 105, 110, 115, 120}
local targetAngles = {0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0}
 --, 7.5, 8.0, 8.5, 9.0, 9.5, 10.0}
local testResults = {}
local maxSteerAngle = 0
local maxSteerAngleTimer = 0
local maxSteerAngleCounter = 0
local maxSteps = 0
local finishedSteps = 0
local progress = 0
local avgStiffnessFront = 0
local avgStiffnessRear = 0
local maxStiffnessFront = 0
local maxStiffnessRear = 0

local idleState = "Idling"
local initState = "Initialization"
local startingState = "Starting"
local incSpeedState = "Increasing Speed"
local incAngleState = "Increasing Angle"
local measureState = "Measuring"
local settleState = "Settling"
local finishedState = "Finished"

local skewStiffnessTestState = idleState
local initWaitingTime = 2

local esc = nil

local function updateUI()
  guihooks.trigger("ESCSkewStiffnessChange", {currentSpeed = currentTargetSpeed, currentAngle = currentTargetAngle, state = skewStiffnessTestState, progress = progress, stiffnessFront = maxStiffnessFront, stiffnessRear = maxStiffnessRear})
end

local function setState(state)
  skewStiffnessTestState = state
  updateUI()
end

local function setMaxPhysicsSpeed(enable)
  obj:queueGameEngineLua("be:setPhysicsSpeedFactor(" .. (enable and 1 or 0) .. ")")
end

local function updateGFX(dt)
  if skewStiffnessTestState == idleState then
    return
  end

  local steeringInput = currentTargetAngle / maxSteerAngle

  if skewStiffnessTestState == initState then
    steeringInput = 1
    maxSteerAngleTimer = maxSteerAngleTimer + dt
    if maxSteerAngleTimer > initWaitingTime then
      maxSteerAngle = maxSteerAngle + math.deg(esc.wheelAngleFront)
      maxSteerAngleCounter = maxSteerAngleCounter + 1
      if maxSteerAngleTimer > 2 * initWaitingTime then
        maxSteerAngle = maxSteerAngle / maxSteerAngleCounter
        setState(startingState)
        maxSteerAngleTimer = 0
        maxSteerAngleCounter = 0
      end
    end
  elseif skewStiffnessTestState == startingState then
    esc.pauseESCAction = true
    currentTargetSpeedIndex = 1
    currentTargetAngleIndex = 1
    currentTargetSpeed = targetSpeeds[currentTargetSpeedIndex]
    currentTargetAngle = targetAngles[currentTargetAngleIndex]
    cruiseControl.setSpeed(currentTargetSpeed / 3.6)
    skewStiffnessTestState = settleState
    updateUI()
  elseif skewStiffnessTestState == incSpeedState then
    currentTargetSpeedIndex = currentTargetSpeedIndex + 1
    currentTargetSpeed = targetSpeeds[currentTargetSpeedIndex]
    cruiseControl.setSpeed(currentTargetSpeed / 3.6)
    setState(settleState)
  elseif skewStiffnessTestState == incAngleState then
    currentTargetAngleIndex = currentTargetAngleIndex + 1
    currentTargetAngle = targetAngles[currentTargetAngleIndex]
    setState(settleState)
  elseif skewStiffnessTestState == settleState then
    if cruiseControl.hasReachedTargetSpeed then
      if not esc.doSettle and not esc.calibrationSettled then
        esc.doSettle = true
      end

      if esc.calibrationSettled then
        esc.calibrationSettled = false
        setState(measureState)
      end
    end
  elseif skewStiffnessTestState == measureState then
    if not esc.doMeasure and not esc.calibrationMeasurementReady then
      esc.doMeasure = true
    end

    if esc.calibrationMeasurementReady then
      esc.calibrationMeasurementReady = false

      -- Prevents division by zero gravity
      local gravity = obj:getGravity()
      gravity = math.max(0.1, math.abs(gravity)) * sign2(gravity)

      local gforce = sensors.gx2 / abs(gravity)

      local stiffnessFront = esc.stiffnessFront
      local stiffnessRear = esc.stiffnessRear

      table.insert(testResults, {speed = currentTargetSpeed, angle = currentTargetAngle, stiffnessFront = stiffnessFront, stiffnessRear = stiffnessRear, gForce = gforce})
      maxStiffnessFront = math.max(maxStiffnessFront, stiffnessFront)
      maxStiffnessRear = math.max(maxStiffnessRear, stiffnessRear)
      avgStiffnessFront = 0
      avgStiffnessRear = 0

      stiffnessUpdateCounter = 0
      finishedSteps = finishedSteps + 1
      progress = finishedSteps / maxSteps * 100

      if currentTargetAngleIndex < #targetAngles then
        setState(incAngleState)
      else
        currentTargetAngleIndex = 1
        currentTargetAngle = targetAngles[currentTargetAngleIndex]
        if currentTargetSpeedIndex < #targetSpeeds then
          setState(incSpeedState)
        else
          setState(finishedState)
          esc.pauseESCAction = false
        end
      end
    end
  elseif skewStiffnessTestState == finishedState then
    local filePivot = io.open("skewStiffnessPivot.csv", "w")
    filePivot:write("Speed,Angle,Front,Rear,Ratio,gForce, Float Angle, Test\r\n")

    for _, v in pairs(testResults) do
      filePivot:write(string.format("%s,%s,%s,%s,%s,%s,%s,%s\r\n", v.speed, v.angle, v.stiffnessFront, v.stiffnessRear, v.stiffnessFront / v.stiffnessRear, v.gForce, v.floatAngle, v.test))
    end

    filePivot:close()
    currentTargetSpeed = 0
    currentTargetAngle = 0
    cruiseControl.setEnabled(false)
    setMaxPhysicsSpeed(false)
    setState(idleState)
  end

  input.event("steering", -steeringInput, 1)
end

local function onInit()
  setMaxPhysicsSpeed(false)
  stiffnessUpdateCounter = 0
  stiffnessFrontSum = 0
  stiffnessRearSum = 0
  currentTargetSpeed = 0
  currentTargetSpeedIndex = 1
  currentTargetAngle = 0
  currentTargetAngleIndex = 1
  avgStiffnessFront = 0
  avgStiffnessRear = 0

  maxSteps = #targetAngles * #targetSpeeds
  finishedSteps = 0
  progress = 0
  maxStiffnessFront = 0
  maxStiffnessRear = 0

  testResults = {}
  maxSteerAngle = 0
  maxSteerAngleTimer = 0
  maxSteerAngleCounter = 0

  setState(idleState)
end

local function startSkewStiffnessTest()
  log("D", "ESC", "Starting skew stiffness test")
  esc = controller.getController("esc")
  if not esc then
    log("E", "ESC", "Vehicle does not have a valid ESC")
    return
  end
  extensions.load("cruiseControl")
  esc.startESCCalibration()

  maxSteps = #targetAngles * #targetSpeeds
  finishedSteps = 0
  progress = 0
  onInit()
  setState(initState)
  setMaxPhysicsSpeed(true)
end

local function stopSkewStiffnessTest()
  log("D", "ESC", "Stopping skew stiffness test")
  if cruiseControl then
    cruiseControl.setEnabled(false)
  end
  if esc then
    esc.stopESCCalibration()
  end
  setMaxPhysicsSpeed(false)
  setState(finishedState)
end

-- public interface
M.onInit = onInit
M.onReset = onInit
M.updateGFX = updateGFX
M.startSkewStiffnessTest = startSkewStiffnessTest
M.stopSkewStiffnessTest = stopSkewStiffnessTest

return M
