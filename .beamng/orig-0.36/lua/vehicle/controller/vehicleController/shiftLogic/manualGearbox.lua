-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local max = math.max
local min = math.min
local abs = math.abs

local constants = {rpmToAV = 0.104719755, avToRPM = 9.549296596425384}

local newDesiredGearIndex = 0
local previousGearIndex = 0
local shiftIntoGearTimer = HighPerfTimer()
local shiftAggression = 1
local gearbox = nil
local engine = nil

local sharedFunctions = nil
local gearboxAvailableLogic = nil
local gearboxLogic = nil

M.gearboxHandling = nil
M.timer = nil
M.timerConstants = nil
M.inputValues = nil
M.shiftPreventionData = nil
M.shiftBehavior = nil
M.smoothedValues = nil

M.currentGearIndex = 0
M.maxGearIndex = 0
M.minGearIndex = 0
M.throttle = 0
M.brake = 0
M.clutchRatio = 0
M.shiftingAggression = 0
M.isArcadeSwitched = false
M.isSportModeActive = false
M.isShifting = false

M.smoothedAvgAVInput = 0
M.rpm = 0
M.idleRPM = 0
M.maxRPM = 0

M.engineThrottle = 0
M.engineLoad = 0
M.engineTorque = 0
M.flywheelTorque = 0
M.gearboxTorque = 0

M.ignition = true
M.isEngineRunning = 0

M.oilTemp = 0
M.waterTemp = 0
M.checkEngine = false

M.energyStorages = {}

local clutchHandling = {
  clutchInRate = 0,
  clutchOutRate = 0,
  clutchLaunchTargetAV = 0,
  clutchLaunchStartAV = 0,
  clutchLaunchIFactor = 0,
  preShiftClutchRatio = 0,
  lastClutchInput = 0,
  shiftState = "clutchIn",
  revMatchThrottle = 0.5,
  didRevMatch = false,
  shiftHasCompletedInput = false,
  waitingForShiftTimer = 0,
  isUsingDirectShifting = false
}

local function getGearName()
  return gearbox.gearIndex
end

local function getGearPosition()
  return 0, 0 --not used with sequentials and manuals
end

local function gearboxBehaviorChanged(behavior)
  gearboxLogic = gearboxAvailableLogic[behavior]
  M.updateGearboxGFX = gearboxLogic.inGear
  M.shiftUpOnDown = gearboxLogic.shiftUpOnDown
  M.shiftDownOnDown = gearboxLogic.shiftDownOnDown
  M.shiftUpOnUp = gearboxLogic.shiftUpOnUp
  M.shiftDownOnUp = gearboxLogic.shiftDownOnUp
  M.shiftToGearIndex = gearboxLogic.shiftToGearIndex

  if behavior == "realistic" and not M.gearboxHandling.autoClutch and abs(gearbox.gearIndex) == 1 then
    gearbox:setGearIndex(0)
  end
end

local function calculateShiftAggression()
  local gearRatioDifference = abs(gearbox.gearRatios[previousGearIndex] - gearbox.gearRatios[newDesiredGearIndex])
  local inertiaCoef = linearScale(engine.inertia, 0.1, 0.5, 0.1, 1)
  local gearRatioCoef = linearScale(gearRatioDifference * inertiaCoef, 0.5, 1, 1, 0.5)
  local aggressionCoef = linearScale(M.smoothedValues.drivingAggression, 0.5, 1, 0.1, 1)

  shiftAggression = clamp(gearRatioCoef * aggressionCoef, 0.3, 1)
  M.shiftingAggression = shiftAggression
  --print(string.format("GR: %.2f, AG: %.2f, IN: %.2f -> %.2f", gearRatioCoef, aggressionCoef, inertiaCoef, shiftAggression))
end

local function shiftUpOnDown()
  --print("shift up - onDown")
  local prevGearIndex = gearbox.gearIndex
  local gearIndex = newDesiredGearIndex == 0 and gearbox.gearIndex + 1 or newDesiredGearIndex + 1
  gearIndex = min(max(gearIndex, gearbox.minGearIndex), gearbox.maxGearIndex)

  if M.gearboxHandling.gearboxSafety then
    local gearRatio = gearbox.gearRatios[newDesiredGearIndex]
    if gearbox.outputAV1 * gearRatio > engine.maxAV then
      gearIndex = prevGearIndex
    end
  end

  if gearbox.gearIndex ~= gearIndex then
    newDesiredGearIndex = gearIndex
    previousGearIndex = gearbox.gearIndex
    clutchHandling.shiftState = "clutchIn"
    clutchHandling.preShiftClutchRatio = M.clutchRatio
    clutchHandling.isUsingDirectShifting = false
    calculateShiftAggression()
    M.updateGearboxGFX = gearboxLogic.whileShifting
  end
end

local function shiftUpOnUp()
  --print("shift up - onUp")
  if clutchHandling.shiftState == "waitingForShift" then
    clutchHandling.shiftState = "shift"
  elseif clutchHandling.shiftState == "clutchIn" or clutchHandling.shiftState == "neutral" then
    clutchHandling.shiftHasCompletedInput = true
  end
end

local function shiftDownOnDown()
  --print("shift down - onDown")
  local prevGearIndex = gearbox.gearIndex
  local gearIndex = newDesiredGearIndex == 0 and gearbox.gearIndex - 1 or newDesiredGearIndex - 1
  gearIndex = min(max(gearIndex, gearbox.minGearIndex), gearbox.maxGearIndex)

  if M.gearboxHandling.gearboxSafety then
    local gearRatio = gearbox.gearRatios[gearIndex]
    if gearbox.outputAV1 * gearRatio > engine.maxAV then
      gearIndex = prevGearIndex
    end
  end

  if gearbox.gearIndex ~= gearIndex then
    newDesiredGearIndex = gearIndex
    previousGearIndex = gearbox.gearIndex
    clutchHandling.shiftState = "clutchIn"
    clutchHandling.preShiftClutchRatio = M.clutchRatio
    clutchHandling.isUsingDirectShifting = false
    calculateShiftAggression()
    M.updateGearboxGFX = gearboxLogic.whileShifting
  end
end

local function shiftDownOnUp()
  --print("shift down - onUp")
  if clutchHandling.shiftState == "waitingForShift" then
    clutchHandling.shiftState = "shift"
  elseif clutchHandling.shiftState == "clutchIn" or clutchHandling.shiftState == "neutral" then
    clutchHandling.shiftHasCompletedInput = true
  end
end

local function shiftToGearIndex(index)
  local prevGearIndex = gearbox.gearIndex
  local gearIndex = min(max(index, gearbox.minGearIndex), gearbox.maxGearIndex)

  if M.gearboxHandling.gearboxSafety then
    local gearRatio = gearbox.gearRatios[gearIndex]
    if gearbox.outputAV1 * gearRatio > engine.maxAV then
      gearIndex = prevGearIndex
    end
  end

  if gearbox.gearIndex ~= gearIndex then
    if gearIndex ~= 0 then
      clutchHandling.shiftHasCompletedInput = true
    end
    newDesiredGearIndex = gearIndex
    previousGearIndex = gearbox.gearIndex
    M.timer.shiftDelayTimer = 0
    clutchHandling.shiftState = "clutchIn"
    clutchHandling.preShiftClutchRatio = M.clutchRatio
    clutchHandling.isUsingDirectShifting = true
    calculateShiftAggression()
    M.updateGearboxGFX = gearboxLogic.whileShifting
  end

  if gearIndex == 0 and clutchHandling.shiftState == "gearGrind" then
    gearbox:setGearIndex(0)
    clutchHandling.shiftState = "clutchOut"
  end
end

local function updateExposedData()
  M.rpm = engine and (engine.outputAV1 * constants.avToRPM) or 0
  M.smoothedAvgAVInput = sharedFunctions.updateAvgAVSingleDevice("gearbox")
  M.waterTemp = (engine and engine.thermals) and (engine.thermals.coolantTemperature or engine.thermals.oilTemperature) or 0
  M.oilTemp = (engine and engine.thermals) and engine.thermals.oilTemperature or 0
  M.checkEngine = engine and engine.isDisabled or false
  M.ignition = electrics.values.ignitionLevel > 1
  M.engineThrottle = (engine and engine.isDisabled) and 0 or M.throttle
  M.engineLoad = engine and (engine.isDisabled and 0 or engine.instantEngineLoad) or 0
  M.running = engine and not engine.isDisabled or false
  M.engineTorque = engine and engine.combustionTorque or 0
  M.flywheelTorque = engine and engine.outputTorque1 or 0
  M.gearboxTorque = gearbox and gearbox.outputTorque1 or 0
  M.isEngineRunning = (engine and engine.starterMaxAV and engine.starterEngagedCoef) and ((engine.outputAV1 > engine.starterMaxAV * 0.8 and engine.starterEngagedCoef <= 0) and 1 or 0) or 1
  M.minGearIndex = gearbox.minGearIndex
  M.maxGearIndex = gearbox.maxGearIndex
end

local function updateInGearArcade(dt)
  M.throttle = M.inputValues.throttle
  M.brake = M.inputValues.brake
  M.isArcadeSwitched = false
  M.isShifting = false

  local gearIndex = gearbox.gearIndex
  local engineAV = engine.outputAV1

  -- driving backwards? - only with automatic shift - for obvious reasons ;)
  if (gearIndex < 0 and M.smoothedValues.avgAV <= 0.8) or (gearIndex <= 0 and M.smoothedValues.avgAV < -1) then
    M.throttle, M.brake = M.brake, M.throttle
    M.isArcadeSwitched = true
  end

  --Arcade mode gets a "rev limiter" in case the engine does not have one
  if engineAV > engine.maxAV and not engine.hasRevLimiter then
    local throttleAdjust = min(max((engineAV - engine.maxAV * 1.02) / (engine.maxAV * 0.03), 0), 1)
    M.throttle = min(max(M.throttle - throttleAdjust, 0), 1)
  end

  if M.timer.gearChangeDelayTimer <= 0 and gearIndex ~= 0 then
    local gearboxInputAV = gearbox.inputAV
    local tmpEngineAV = gearboxInputAV
    local relEngineAV = gearboxInputAV / gearbox.gearRatio

    sharedFunctions.selectShiftPoints(gearIndex)

    --shift down?
    local rpmTooLow = (tmpEngineAV < M.shiftBehavior.shiftDownAV) or (tmpEngineAV <= engine.idleAV * 1.05)
    while rpmTooLow and abs(gearIndex) > 1 and M.shiftPreventionData.wheelSlipShiftDown and abs(M.throttle - M.smoothedValues.throttle) < M.smoothedValues.throttleUpShiftThreshold do
      gearIndex = gearIndex - sign(gearIndex)
      tmpEngineAV = relEngineAV * (gearbox.gearRatios[gearIndex] or 0)
      if tmpEngineAV >= engine.maxAV * 0.85 then
        tmpEngineAV = relEngineAV / (gearbox.gearRatios[gearIndex] or 0)
        gearIndex = gearIndex + sign(gearIndex)
        sharedFunctions.selectShiftPoints(gearIndex)
        break
      end
      sharedFunctions.selectShiftPoints(gearIndex)
    end

    local inGearRange = gearIndex < gearbox.maxGearIndex and gearIndex > gearbox.minGearIndex
    local clutchReady = M.clutchRatio >= 1
    local isRevLimitReached = engine.revLimiterActive and not (engine.isTempRevLimiterActive or false)
    local engineRevTooHigh = (tmpEngineAV >= M.shiftBehavior.shiftUpAV or isRevLimitReached)
    local throttleSpike = abs(M.throttle - M.smoothedValues.throttle) < M.smoothedValues.throttleUpShiftThreshold
    local notBraking = M.brake <= 0
    --shift up?
    if clutchReady and engineRevTooHigh and M.shiftPreventionData.wheelSlipShiftUp and notBraking and throttleSpike and inGearRange then
      gearIndex = gearIndex + sign(gearIndex)
      tmpEngineAV = relEngineAV * (gearbox.gearRatios[gearIndex] or 0)
      if tmpEngineAV < engine.idleAV then
        gearIndex = gearIndex - sign(gearIndex)
      end
      sharedFunctions.selectShiftPoints(gearIndex)
    end
  end

  -- neutral gear handling
  if abs(gearIndex) <= 1 and M.timer.neutralSelectionDelayTimer <= 0 then
    if abs(M.smoothedValues.avgAV) < M.gearboxHandling.arcadeAutoBrakeAVThreshold and M.throttle <= 0 then
      M.brake = max(M.inputValues.brake, M.gearboxHandling.arcadeAutoBrakeAmount)
    end

    if M.smoothedValues.throttleInput > 0 and M.smoothedValues.brakeInput <= 0 and M.smoothedValues.avgAV > -1 and gearIndex < 1 then
      gearIndex = 1
      M.timer.neutralSelectionDelayTimer = M.timerConstants.neutralSelectionDelay
    end

    if M.smoothedValues.brakeInput > 0 and M.smoothedValues.throttleInput <= 0 and M.smoothedValues.avgAV <= 0.15 and electrics.values.airspeed < 2 and gearIndex > -1 then
      gearIndex = -1
      M.timer.neutralSelectionDelayTimer = M.timerConstants.neutralSelectionDelay
    end
  end

  if gearbox.gearIndex ~= gearIndex then
    newDesiredGearIndex = gearIndex
    previousGearIndex = gearbox.gearIndex
    clutchHandling.shiftState = "clutchIn"
    calculateShiftAggression()
    M.updateGearboxGFX = gearboxLogic.whileShifting
  end

  -- Control clutch to buildup engine RPM
  if abs(gearIndex) == 1 then
    if M.throttle > 0 then
      local ratio = max((engine.outputAV1 - clutchHandling.clutchLaunchStartAV * (1 + M.throttle)) / (clutchHandling.clutchLaunchTargetAV * (1 + clutchHandling.clutchLaunchIFactor)), 0)
      clutchHandling.clutchLaunchIFactor = min(clutchHandling.clutchLaunchIFactor + dt * 0.5, 1)
      M.clutchRatio = min(max(ratio * ratio, 0), 1)
    end
  elseif gearIndex ~= 0 then
    if M.smoothedValues.avgAV * gearbox.gearRatio * engine.outputAV1 >= 0 then
      M.clutchRatio = 1
    elseif abs(gearbox.gearIndex) > 1 then
      M.brake = M.throttle
      M.throttle = 0
    end
    clutchHandling.clutchLaunchIFactor = 0
  end

  if M.inputValues.clutch > 0 then
    if M.inputValues.clutch < clutchHandling.lastClutchInput then
      M.timer.gearChangeDelayTimer = M.timerConstants.gearChangeDelay
    end
    M.clutchRatio = min(1 - M.inputValues.clutch, M.clutchRatio)
  end

  --always prevent stalling
  if engine.outputAV1 < engine.idleAV then
    M.clutchRatio = 0
  end

  if (M.throttle > 0.5 and M.brake > 0.5 and electrics.values.wheelspeed < 2) or gearbox.lockCoef < 1 then
    M.clutchRatio = 0
  end

  if M.clutchRatio < 1 and abs(gearIndex) == 1 then
    M.timer.gearChangeDelayTimer = M.timerConstants.gearChangeDelay
  end

  clutchHandling.preShiftClutchRatio = M.clutchRatio
  clutchHandling.lastClutchInput = M.inputValues.clutch
  M.currentGearIndex = gearIndex
  updateExposedData()
end

local function updateWhileShiftingArcade(dt)
  -- old -> N -> wait -> new -> in gear update
  M.brake = M.inputValues.brake
  M.isArcadeSwitched = false
  M.isShifting = true

  local gearIndex = gearbox.gearIndex
  if (gearIndex < 0 and M.smoothedValues.avgAV <= 0.15) or (gearIndex <= 0 and M.smoothedValues.avgAV < -1) then
    M.throttle, M.brake = M.brake, M.throttle
    M.isArcadeSwitched = true
  end

  --set throttle to zero when we are actually shifting, this does not apply when going from N to 1 or -1
  M.throttle = (abs(gearIndex) <= 1 and abs(newDesiredGearIndex) <= 1) and M.inputValues.throttle or 0

  if clutchHandling.shiftState == "clutchIn" then
    M.clutchRatio = max(M.clutchRatio - dt * clutchHandling.clutchInRate * shiftAggression, 0)
    if M.clutchRatio <= 0 then
      if previousGearIndex ~= 0 then
        clutchHandling.shiftState = "neutral"
      else
        clutchHandling.shiftState = "shift"
      end
    end
  end

  --No "elseif" here so that we can continue directly in the neutral code and don't waste a frame
  if clutchHandling.shiftState == "neutral" then
    gearbox:setGearIndex(0)
    M.timer.shiftDelayTimer = M.timerConstants.shiftDelay / shiftAggression
    clutchHandling.shiftState = "shift"
  elseif clutchHandling.shiftState == "shift" then
    local canShift = true
    local isEngineRunning = engine.ignitionCoef >= 1 and not engine.isStalled
    local targetAV = (gearbox.gearRatios[newDesiredGearIndex] / gearbox.gearRatios[previousGearIndex]) * (gearbox.outputAV1 * gearbox.gearRatios[previousGearIndex])
    if targetAV > engine.outputAV1 and previousGearIndex ~= 0 and not clutchHandling.didRevMatch and clutchHandling.preShiftClutchRatio >= 1 and isEngineRunning and clutchHandling.revMatchThrottle > 0 then
      M.throttle = clutchHandling.revMatchThrottle
      canShift = engine.outputAV1 >= targetAV or targetAV > engine.maxAV
      clutchHandling.didRevMatch = canShift
    end
    if M.timer.shiftDelayTimer <= 0 and canShift then
      gearbox:setGearIndex(newDesiredGearIndex)
      newDesiredGearIndex = 0
      previousGearIndex = 0
      M.timer.gearChangeDelayTimer = M.timerConstants.gearChangeDelay
      clutchHandling.didRevMatch = false
      clutchHandling.shiftState = "clutchOut"
    end
  end

  --no "elseif" here so that we can right continue to declutching without wasting further time
  if clutchHandling.shiftState == "clutchOut" then
    if clutchHandling.preShiftClutchRatio > 0 then
      local stallPrevent = min(max((engine.outputAV1 * 0.9 - engine.idleAV) / (engine.idleAV * 0.1), 0), 1)
      M.clutchRatio = min(M.clutchRatio + dt * clutchHandling.clutchOutRate * shiftAggression, stallPrevent * stallPrevent)
      if M.clutchRatio >= 1 or stallPrevent < 1 then
        M.updateGearboxGFX = gearboxLogic.inGear
      end
    else
      M.updateGearboxGFX = gearboxLogic.inGear
    end
  end
  M.currentGearIndex = gearbox.gearIndex
  updateExposedData()
end

local function updateInGear(dt)
  M.throttle = M.inputValues.throttle
  M.brake = M.inputValues.brake
  M.isArcadeSwitched = false
  M.isShifting = false

  -- Control clutch to buildup engine RPM
  if M.gearboxHandling.autoClutch then
    local gearIndex = gearbox.gearIndex
    if abs(gearIndex) == 1 then
      if M.throttle > 0 then
        local ratio = max((engine.outputAV1 - clutchHandling.clutchLaunchStartAV * (1 + M.throttle)) / (clutchHandling.clutchLaunchTargetAV * (1 + clutchHandling.clutchLaunchIFactor)), 0)
        clutchHandling.clutchLaunchIFactor = min(clutchHandling.clutchLaunchIFactor + dt * 0.5, 1)
        M.clutchRatio = min(max(ratio * ratio, 0), 1)
      end
    elseif gearIndex ~= 0 then
      if M.smoothedValues.avgAV * gearbox.gearRatio * engine.outputAV1 >= 0 then
        M.clutchRatio = 1
      elseif abs(gearIndex) > 1 then
        local ratio = max((engine.outputAV1 - clutchHandling.clutchLaunchStartAV * (1 + M.throttle)) / (clutchHandling.clutchLaunchTargetAV * (1 + clutchHandling.clutchLaunchIFactor)), 0)
        clutchHandling.clutchLaunchIFactor = min(clutchHandling.clutchLaunchIFactor + dt * 0.5, 1)
        M.clutchRatio = min(max(ratio * ratio, 0), 1)
      end
      clutchHandling.clutchLaunchIFactor = 0
    end

    if M.inputValues.clutch > 0 then
      M.clutchRatio = min(1 - M.inputValues.clutch, M.clutchRatio)
    end

    --always prevent stalling
    if engine.outputAV1 < engine.idleAV then
      M.clutchRatio = 0
    end

    if (M.throttle > 0.5 and M.brake > 0.5 and electrics.values.wheelspeed < 2) or gearbox.lockCoef < 1 then
      M.clutchRatio = 0
    end

    if M.clutchRatio < 1 and abs(gearIndex) == 1 then
      M.timer.gearChangeDelayTimer = M.timerConstants.gearChangeDelay
    end

    if engine.isDisabled then
      M.clutchRatio = min(1 - M.inputValues.clutch, 1)
    end

    if engine.idleAVStartOffset > 1 and M.throttle <= 0 then
      M.clutchRatio = 0
    end
  else
    M.clutchRatio = 1 - M.inputValues.clutch
  end
  M.currentGearIndex = gearbox.gearIndex
  updateExposedData()
end

local function updateWhileShifting(dt)
  --print(clutchHandling.shiftState)
  M.isShifting = true
  -- old -> N -> wait -> new -> in gear update
  if M.gearboxHandling.autoThrottle then
    --set throttle to zero when we are actually shifting, this does not apply when going from N to 1 or -1
    M.throttle = (abs(gearbox.gearIndex) <= 1 and abs(newDesiredGearIndex) <= 1) and M.inputValues.throttle or 0
  else
    M.throttle = M.inputValues.throttle
  end
  M.brake = M.inputValues.brake
  M.isArcadeSwitched = false

  if clutchHandling.shiftState == "clutchIn" then
    if M.gearboxHandling.autoClutch then
      M.clutchRatio = max(M.clutchRatio - dt * clutchHandling.clutchInRate * shiftAggression, 0)
      if M.clutchRatio <= 0 then
        if gearbox.gearIndex ~= 0 then
          clutchHandling.shiftState = "neutral"
        else
          clutchHandling.shiftState = "waitingForShift"
        end
      end
    else
      M.clutchRatio = min(1 - M.inputValues.clutch, 1)
      if gearbox.gearIndex ~= 0 then
        clutchHandling.shiftState = "neutral"
      else
        clutchHandling.shiftState = "waitingForShift"
      end
    end
  end

  --No "elseif" here so that we can continue directly in the neutral code and don't waste a frame when not in autoClutch mode
  if clutchHandling.shiftState == "neutral" then
    gearbox:setGearIndex(0)
    M.timer.shiftDelayTimer = 0
    --M.timerConstants.shiftDelay / (M.gearboxHandling.autoClutch and shiftAggression or 1)
    clutchHandling.shiftState = "waitingForShift"
    shiftIntoGearTimer:reset()
  elseif clutchHandling.shiftState == "waitingForShift" then
    M.isShifting = false
    if not M.gearboxHandling.autoClutch then
      M.clutchRatio = min(1 - M.inputValues.clutch, 1)
    end
    clutchHandling.waitingForShiftTimer = clutchHandling.waitingForShiftTimer + dt
    if clutchHandling.shiftHasCompletedInput then
      clutchHandling.shiftState = "shift"
      clutchHandling.shiftHasCompletedInput = false
    elseif clutchHandling.waitingForShiftTimer > 2 and clutchHandling.isUsingDirectShifting then --if we are simply switching to neutral, we would be stuck here, so make sure that we only wait a certain amount of time for another shift to happen
      clutchHandling.waitingForShiftTimer = 0
      clutchHandling.shiftState = "clutchOut"
    end
  elseif clutchHandling.shiftState == "shift" then
    local canShift = true
    local targetAV = gearbox.gearRatios[newDesiredGearIndex] * gearbox.outputAV1
    local isEngineRunning = engine.ignitionCoef >= 1 and not engine.isStalled
    if M.gearboxHandling.autoThrottle and targetAV > engine.outputAV1 and not clutchHandling.didRevMatch and clutchHandling.preShiftClutchRatio >= 1 and isEngineRunning and clutchHandling.revMatchThrottle > 0 then
      M.throttle = clutchHandling.revMatchThrottle
      canShift = engine.outputAV1 >= targetAV or targetAV > engine.maxAV
      clutchHandling.didRevMatch = canShift
    end
    if not M.gearboxHandling.autoClutch then
      M.clutchRatio = min(1 - M.inputValues.clutch, 1)
    end

    if M.timer.shiftDelayTimer <= 0 and canShift then
      local availableSyncTime = shiftIntoGearTimer:stop() * 0.001 * obj:getSimulationTimeScale() --account for slow motion as well, the timer measures wall time
      if newDesiredGearIndex ~= 0 then
        shiftIntoGearTimer:reset()
      end
      if gearbox.gearIndex ~= newDesiredGearIndex then
        gearbox:setGearIndex(newDesiredGearIndex, availableSyncTime)
      end
      newDesiredGearIndex = gearbox.gearIndex == newDesiredGearIndex and 0 or newDesiredGearIndex
      previousGearIndex = 0
      M.timer.gearChangeDelayTimer = M.timerConstants.gearChangeDelay
      clutchHandling.didRevMatch = false
      if gearbox.isGrindingShift then
        clutchHandling.shiftState = "gearGrind"
      else
        clutchHandling.shiftState = "clutchOut"
      end
    end
  elseif clutchHandling.shiftState == "gearGrind" then
    if not M.gearboxHandling.autoClutch then
      M.clutchRatio = min(1 - M.inputValues.clutch, 1)
    end
    if not gearbox.isGrindingShift then
      clutchHandling.shiftState = "clutchOut"
    end
  end

  --no "elseif" here so that we can right continue to declutching without wasting further time
  if clutchHandling.shiftState == "clutchOut" then
    if M.gearboxHandling.autoClutch and clutchHandling.preShiftClutchRatio > 0 then
      local stallPrevent = min(max((engine.outputAV1 * 0.9 - engine.idleAV) / (engine.idleAV * 0.1), 0), 1)
      M.clutchRatio = min(M.clutchRatio + dt * clutchHandling.clutchOutRate * shiftAggression, stallPrevent)
      if M.clutchRatio >= 1 or stallPrevent < 1 then
        M.updateGearboxGFX = gearboxLogic.inGear
      end
    else
      if not M.gearboxHandling.autoClutch then
        M.clutchRatio = 1 - M.inputValues.clutch
      end
      M.updateGearboxGFX = gearboxLogic.inGear
      clutchHandling.isUsingDirectShifting = false
    end
  end
  M.currentGearIndex = gearbox.gearIndex
  updateExposedData()
end

local function sendTorqueData()
  if engine then
    engine:sendTorqueData()
  end
end

local function init(jbeamData, sharedFunctionTable)
  sharedFunctions = sharedFunctionTable
  engine = powertrain.getDevice("mainEngine")
  gearbox = powertrain.getDevice("gearbox")
  newDesiredGearIndex = 0
  previousGearIndex = 0

  M.currentGearIndex = 0
  M.maxGearIndex = 0
  M.minGearIndex = 0
  M.throttle = 0
  M.brake = 0
  M.clutchRatio = 0
  M.shiftingAggression = 0
  M.isArcadeSwitched = false
  M.isSportModeActive = false
  M.isShifting = false

  M.smoothedAvgAVInput = 0
  M.rpm = 0
  M.idleRPM = 0
  M.maxRPM = 0

  M.engineThrottle = 0
  M.engineLoad = 0
  M.engineTorque = 0
  M.flywheelTorque = 0
  M.gearboxTorque = 0

  M.ignition = true
  M.isEngineRunning = 0

  M.oilTemp = 0
  M.waterTemp = 0
  M.checkEngine = false

  gearboxAvailableLogic = {
    arcade = {
      inGear = updateInGearArcade,
      whileShifting = updateWhileShiftingArcade,
      shiftUpOnDown = sharedFunctions.warnCannotShiftSequential,
      shiftDownOnDown = sharedFunctions.warnCannotShiftSequential,
      shiftUpOnUp = nop,
      shiftDownOnUp = nop,
      shiftToGearIndex = sharedFunctions.switchToRealisticBehavior
    },
    realistic = {
      inGear = updateInGear,
      whileShifting = updateWhileShifting,
      shiftUpOnDown = shiftUpOnDown,
      shiftDownOnDown = shiftDownOnDown,
      shiftUpOnUp = shiftUpOnUp,
      shiftDownOnUp = shiftDownOnUp,
      shiftToGearIndex = shiftToGearIndex
    }
  }

  clutchHandling.clutchLaunchTargetAV = (jbeamData.clutchLaunchTargetRPM or 3000) * constants.rpmToAV * 0.5
  clutchHandling.clutchLaunchStartAV = ((jbeamData.clutchLaunchStartRPM or 2000) * constants.rpmToAV - engine.idleAV) * 0.5
  clutchHandling.clutchLaunchIFactor = 0

  clutchHandling.waitingForShiftTimer = 0
  clutchHandling.isUsingDirectShifting = false

  clutchHandling.clutchInRate = jbeamData.clutchInRate or 25
  clutchHandling.clutchOutRate = jbeamData.clutchOutRate or 25

  clutchHandling.revMatchThrottle = jbeamData.revMatchThrottle or 0.5

  M.maxRPM = engine.maxRPM
  M.idleRPM = engine.idleRPM
  M.maxGearIndex = gearbox.maxGearIndex
  M.minGearIndex = abs(gearbox.minGearIndex)
  M.energyStorages = sharedFunctions.getEnergyStorages({engine})

  --print("experimental gearbox logic")
end

local function getState()
  local data = {grb_idx = gearbox.gearIndex}

  return tableIsEmpty(data) and nil or data
end

local function setState(data)
  if data.grb_idx then
    shiftToGearIndex(data.grb_idx)
  end
end

M.init = init

M.gearboxBehaviorChanged = gearboxBehaviorChanged
M.shiftUpOnDown = shiftUpOnDown
M.shiftDownOnDown = shiftDownOnDown
M.shiftUpOnUp = shiftUpOnUp
M.shiftDownOnUp = shiftDownOnUp
M.shiftToGearIndex = shiftToGearIndex
M.updateGearboxGFX = nop
M.getGearName = getGearName
M.getGearPosition = getGearPosition
M.sendTorqueData = sendTorqueData

M.getState = getState
M.setState = setState

return M
