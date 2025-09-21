-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local detectionPeakPercent = 0.8 -- when we're less than this % of the peak FFB force
local detectionStDiff = 0.1 -- when we're more than this stwheel angle away from peak FFB force
local speedMin = 40 / 3.6
local speedTarget = speedMin * 2 -- cruise control loses speed easily when beginning to understeer, so we try to reach a higher speed than we want

local targetPeak = 0.2 -- we want to feel 20% of our current FFB steeringwheel force

local data
local FFBSmoother = newExponentialSmoothing(150)
local FFBMaxValue
local FFBMaxValueSm
local FFBMaxAngle

local logCountdown
local safetyCountdown
local hasReachedTargetConditions
local hasReachedSpeed

local steeringWheelLock
local timeToFulllock = 20
local timeToGradual = 1
local elapsedFulllock
local lastSteeringInput
local gradualInput
local elapsedGradual
local elapsedCruise
local airspeed

local function onPhysicsStepSteer(dtSim)
  if not data then return end
  logCountdown = logCountdown - dtSim
  safetyCountdown = safetyCountdown - dtSim

  if safetyCountdown < 0 then
    log("I", "", string.format(" >>> VLUA FFB calibration: finished due to safety timeout"))
    data.FFB.endReason = "timeout"
    M.finish()
  end

  if not hasReachedTargetConditions then
    -- target conditions not reached (speed, steering)
    if cruiseControl.hasReachedTargetSpeed then
      hasReachedSpeed = true
    else
      if not hasReachedSpeed then
        elapsedCruise = elapsedCruise + dtSim
        if elapsedCruise > 30 then
          speedTarget = airspeed * 0.9
          speedMin = speedTarget * 0.6
          log("W", "", string.format(" >>> VLUA FFB calibration: cruise control taking too long to reach speed. Falling back to reachable speeds: %5.1f kmh, %5.1f kmh", speedMin*3.6, speedTarget*3.6))
          data.FFB.cruiseControlFailures = data.FFB.cruiseControlFailures + 1
          data.FFB.speedTarget = speedTarget * 3.6
          data.FFB.speedMin = speedMin * 3.6
          hasReachedSpeed = true
          safetyCountdown = 120
        end
      end
    end
    if hasReachedSpeed then
      -- cruise control is at speed, we can proceed with the turning maneouver
      if gradualInput then
        -- turning maneouver (we want to gradually reach lastSteeringInput angle before recording any data)
        if elapsedGradual < timeToGradual then
          -- turning maneouver is in progress
          if logCountdown < 0 then
            log("I", "", string.format("VLUA FFB calibration: gradual turning in progress: %4.0f kmh (%i%%), %5.3f st (%i%%)", airspeed*3.6, math.min(1, airspeed / speedMin)*100, gradualInput, 100*clamp(gradualInput/lastSteeringInput, 0, 1)))
            logCountdown = 0.5
          end
          elapsedGradual = elapsedGradual + dtSim
          gradualInput = lastSteeringInput * clamp(elapsedGradual / timeToGradual, 0, 1)
          input.event("steering", gradualInput, FILTER_DIRECT, steeringWheelLock, 1, nil, "ffbCalibration")
        else
          -- turning finished progress
          log("I", "", string.format(" >>> VLUA FFB calibration: finished gradual turning"))
          logCountdown = 0
          gradualInput = nil
          hasReachedTargetConditions = true
        end
      else
        -- just started a gradual input to reach lastSteeringInput
        log("I", "", string.format(" >>> VLUA FFB calibration: cruise control reached target speed. Beginning gradual turning"))
        logCountdown = 0
        gradualInput = 0
      end
    else
      -- cruise control is speeding up
      if logCountdown < 0 then
        log("I", "", string.format("VLUA FFB calibration: cruise control in progress: %4.0f kmh (%i%%, %i%%)", airspeed*3.6, math.min(1, airspeed / speedMin)*100, math.min(1, airspeed / speedTarget)*100))
        logCountdown = 0.5
      end
    end
    return
  end

  if airspeed > speedMin then
    -- we're in a position to record data
    if logCountdown < 0 then
      log("I", "", string.format("VLUA FFB calibration: data being logged: %4.0f kmh (%i%%), %5.3f st (%i%%), %7.3f ffb", airspeed*3.6, math.min(1, airspeed / speedMin)*100, lastSteeringInput, 100*lastSteeringInput, hydros.forceAtWheelNorm))
      logCountdown = 0.5
    end

    lastSteeringInput = clamp(elapsedFulllock / timeToFulllock, 0, 1)
    input.event("steering", lastSteeringInput, FILTER_DIRECT, steeringWheelLock, 1, nil, "ffbCalibration")
    local FFB = hydros.forceAtWheelNorm
    local FFBSm = FFBSmoother:get(FFB, dtSim)
    if FFB > FFBMaxValue then
      FFBMaxValue = FFB
      FFBMaxValueSm = FFBSm
      FFBMaxAngle = lastSteeringInput
    end
    table.insert(data.values, {airspeed*3.6, lastSteeringInput, FFB, FFBSm} )
    elapsedFulllock = elapsedFulllock + dtSim
    if elapsedFulllock > timeToFulllock then
      data.FFB.endReason = "fullLock"
      M.finish()
      return
    end
    if lastSteeringInput-FFBMaxAngle > detectionStDiff and FFB < detectionPeakPercent*FFBMaxValue then
      data.FFB.endReason = "noIssues"
      M.finish()
      return
    end
  else
    -- we've lost too much speed
    log("I", "", string.format(" >>> VLUA FFB calibration: cruise control has lost too much speed: %4.0f kmh (%i%%)", airspeed*3.6, math.min(1, airspeed / speedMin)*100))
    logCountdown = 0
    data.FFB.cruiseControlRestarts = data.FFB.cruiseControlRestarts + 1
    hasReachedTargetConditions = nil
    hasReachedSpeed = nil
    elapsedGradual = 0
    elapsedCruise = 0
    input.event("steering", 0, FILTER_DIRECT, steeringWheelLock, 1, nil, "ffbCalibration")
    cruiseControl.setEnabled(false)
    cruiseControl.minimumSpeed = speedTarget
    cruiseControl.setSpeed(speedTarget)
  end
end

local function onPhysicsStepStraight(dtSim)
  if not data then return end
  logCountdown = logCountdown - dtSim
  safetyCountdown = safetyCountdown - dtSim

  if safetyCountdown < 0 then
    log("I", "", string.format(" >>> VLUA FFB calibration: finished succesfull"))
    data.FFB.endReason = "noIssues"
    M.finish()
    return
  end

  -- we're in a position to record data
  if logCountdown < 0 then
    log("I", "", string.format("VLUA FFB calibration: data being logged: %4.0f kmh (%i%%), %5.3f st (%i%%), %7.3f ffb", airspeed*3.6, math.min(1, airspeed / speedMin)*100, lastSteeringInput, 100*lastSteeringInput, hydros.forceAtWheelNorm))
    logCountdown = 0.5
  end

  local FFB = hydros.forceAtWheelNorm
  local FFBSm = FFBSmoother:get(FFB, dtSim)
  if FFB > FFBMaxValue then
    FFBMaxValue = FFB
    FFBMaxValueSm = FFBSm
    FFBMaxAngle = lastSteeringInput
  end
  table.insert(data.values, {airspeed*3.6, lastSteeringInput, FFB, FFBSm} )
end

local function onPhysicsStep(dtSim)
  airspeed = obj:getGroundSpeed()
  if M.mode == "steerPeak" then
    onPhysicsStepSteer(dtSim)
  elseif M.mode == "steerFull" then
    onPhysicsStepSteer(dtSim)
  elseif M.mode == "straight" then
    onPhysicsStepStraight(dtSim)
  end
end

local function finish()
  obj:queueGameEngineLua("be:setPhysicsSpeedFactor(0)")
  if data ~= nil then
    input.setAllowedInputSource("steering", nil)
    cruiseControl.setEnabled(false)
    controller.mainController.setGearboxMode('realistic')
    input.event("brake", 1, 1)
    input.event("clutch", 1, 1)
    input.event("throttle", 0, 1)
    input.event("steering", 0, FILTER_DIRECT, steeringWheelLock, 1, nil, "ffbCalibration")
    log("I", "", " >>> VLUA FFB calibration finished")
    data.FFB.FFBcoef      = v.data.input.FFBcoef
    data.FFB.maxValueTarget= 10*targetPeak
    data.FFB.correctionAdvice  = (10*targetPeak) / FFBMaxValue
    data.FFB.correctionAdviceSm= (10*targetPeak) / FFBMaxValueSm
    data.FFB.maxValue  = FFBMaxValue
    data.FFB.maxValueSm= FFBMaxValueSm
    data.FFB.maxAngle  = FFBMaxAngle
  end
  obj:queueGameEngineLua(string.format("test_ffbCalibration.onFFBCalibrationFinished(%q)", lpack.encode(data)))
  data = nil
end

local function start(mode, fastmotion)
  log("I", "", " >>> VLUA FFB calibration started in mode: '"..dumps(mode).."'")
  -- initialize variables
  if not v or not v.data or not v.data.input or not v.data.input.steeringWheelLock then
    M.finish()
    return
  end
  M.mode = mode
  steeringWheelLock = v.data.input.steeringWheelLock * 2
  logCountdown = 0
  if mode == "steerPeak" then
    safetyCountdown = 120
  elseif mode == "steerFull" then
    detectionPeakPercent = 0 -- allows to turn all the way to full lock
    safetyCountdown = 120
  elseif mode == "straight" then
    safetyCountdown = 40
    speedTarget = 200 * 3.6
  else
    log("E", "", "Unknown mode: "..dumps(M.mode))
    M.finish()
    return
  end
  safetyCountdown = mode == "straight" and 40 or 120
  hasReachedTargetConditions = nil
  hasReachedSpeed = nil
  lastSteeringInput = 0
  elapsedFulllock = 0
  elapsedGradual = 0
  elapsedCruise = 0
  FFBMaxValue = 0
  FFBMaxValueSm = 0
  FFBMaxAngle = 0
  data = {}
  data.values = {}
  data.FFB = {}
  data.FFB.cruiseControlFailures = 0
  data.FFB.cruiseControlRestarts = 0
  data.FFB.endReason = "unknown"
  data.FFB.speedTarget = speedTarget * 3.6
  data.FFB.speedMin = speedMin * 3.6

  -- initialize logic
  if fastmotion then
    obj:queueGameEngineLua("be:setPhysicsSpeedFactor(100)")
  end
  input.setAllowedInputSource("steering", "ffbCalibration", true)
  controller.mainController.setGearboxMode('arcade')
  extensions.load("cruiseControl")
  cruiseControl.minimumSpeed = speedTarget
  cruiseControl.setSpeed(speedTarget)
  input.event("steering", 0, FILTER_DIRECT, steeringWheelLock, 1, nil, "ffbCalibration")
  hydros.enableFFB = false
  hydros.enableFFBflood = true
end

local function onExtensionLoaded()
  enablePhysicsStepHook()
end

M.start = start
M.finish = finish
M.onPhysicsStep = onPhysicsStep
M.onExtensionLoaded = onExtensionLoaded

return M
