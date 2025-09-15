-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local max = math.max
local min = math.min
local abs = math.abs

local constants = {rpmToAV = 0.104719755, avToRPM = 9.549296596425384}

local newDesiredGearIndex = 0
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
M.shiftingAggression = 1
M.isArcadeSwitched = false
M.isSportModeActive = false

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
  clutchLaunchTargetAV = 0,
  clutchLaunchStartAV = 0,
  clutchLaunchIFactor = 0,
  lastClutchInput = 0
}

local neutralRejectTimer = 0 --used to reject shifts into neutral when using an H shifter
local neutralRejectTime = 0.75

local ignitionCutTime = 0.15

local function getGearName()
  return gearbox.gearIndex
end

local function getGearPosition()
  return 0, 0 --not used with sequentials and manuals
end

local function gearboxBehaviorChanged(behavior)
  gearboxLogic = gearboxAvailableLogic[behavior]
  M.updateGearboxGFX = gearboxLogic.inGear
  M.shiftUp = gearboxLogic.shiftUp
  M.shiftDown = gearboxLogic.shiftDown
  M.shiftToGearIndex = gearboxLogic.shiftToGearIndex

  if behavior == "realistic" and not M.gearboxHandling.autoClutch and abs(gearbox.gearIndex) == 1 then
    gearbox:setGearIndex(0)
  end
end

local function shiftUp()
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
    M.updateGearboxGFX = gearboxLogic.whileShifting
  end
end

local function shiftDown()
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
    M.updateGearboxGFX = gearboxLogic.whileShifting
  end
end

local function shiftToGearIndex(index, ignoreSequentialBounds)
  local prevGearIndex = gearbox.gearIndex
  if index == 0 and abs(prevGearIndex) > 1 then
    return
  end
  local gearIndex = min(max(index, gearbox.minGearIndex), gearbox.maxGearIndex)

  --if specifically requested to ignore the +-1 bounds of the sequential, don't enforce them
  if not ignoreSequentialBounds then
    local maxIndex = min(prevGearIndex + 1, gearbox.maxGearIndex)
    local minIndex = max(prevGearIndex - 1, gearbox.minGearIndex)

    --adjust expected gearIndex based on sequential limits, otherwise the safety won't work correctly as it will see a 0 gearratio when going into N from higher gears
    gearIndex = min(max(gearIndex, minIndex), maxIndex)
  end

  if M.gearboxHandling.gearboxSafety then
    local gearRatio = gearbox.gearRatios[gearIndex]
    if gearbox.outputAV1 * gearRatio > engine.maxAV then
      gearIndex = prevGearIndex
    end
  end

  if gearbox.gearIndex ~= gearIndex then
    newDesiredGearIndex = gearIndex
    if newDesiredGearIndex == 0 then
      neutralRejectTimer = neutralRejectTime
    end
    M.updateGearboxGFX = gearboxLogic.whileShifting
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
  M.shiftingAggression = 1
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
    if rpmTooLow and abs(gearIndex) > 1 and M.shiftPreventionData.wheelSlipShiftDown and abs(M.throttle - M.smoothedValues.throttle) < M.smoothedValues.throttleUpShiftThreshold then
      gearIndex = gearIndex - sign(gearIndex)
      tmpEngineAV = relEngineAV * (gearbox.gearRatios[gearIndex] or 0)
      if tmpEngineAV >= engine.maxAV * 0.85 then
        tmpEngineAV = relEngineAV / (gearbox.gearRatios[gearIndex] or 0)
        gearIndex = gearIndex + sign(gearIndex)
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
    if gearIndex ~= 0 and abs(M.smoothedValues.avgAV) < M.gearboxHandling.arcadeAutoBrakeAVThreshold and M.throttle <= 0 then
      M.brake = max(M.inputValues.brake, M.gearboxHandling.arcadeAutoBrakeAmount)
    end

    if M.smoothedValues.throttleInput > 0 and M.smoothedValues.brakeInput <= 0 and M.smoothedValues.avgAV > -1 and gearIndex < 1 then
      gearIndex = 1
      M.timer.neutralSelectionDelayTimer = M.timerConstants.neutralSelectionDelay
    end

    if M.smoothedValues.brakeInput > 0 and M.smoothedValues.throttleInput <= 0 and M.smoothedValues.avgAV <= 0.15 and gearIndex > -1 then
      gearIndex = -1
      M.timer.neutralSelectionDelayTimer = M.timerConstants.neutralSelectionDelay
    end
  end

  if gearbox.gearIndex ~= gearIndex then
    newDesiredGearIndex = gearIndex
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

  clutchHandling.lastClutchInput = M.inputValues.clutch

  M.currentGearIndex = gearIndex
  updateExposedData()
end

local function updateWhileShiftingArcade(dt)
  M.throttle = M.inputValues.throttle
  M.brake = M.inputValues.brake
  M.isArcadeSwitched = false
  M.isShifting = true

  local gearIndex = gearbox.gearIndex
  if (gearIndex < 0 and M.smoothedValues.avgAV <= 0.15) or (gearIndex <= 0 and M.smoothedValues.avgAV < -1) then
    M.throttle, M.brake = M.brake, M.throttle
    M.isArcadeSwitched = true
  end
  if newDesiredGearIndex > gearIndex and gearIndex > 0 and M.throttle > 0 then
    engine:cutIgnition(ignitionCutTime)
  end

  gearbox:setGearIndex(newDesiredGearIndex)
  newDesiredGearIndex = 0
  M.timer.gearChangeDelayTimer = M.timerConstants.gearChangeDelay
  M.updateGearboxGFX = gearboxLogic.inGear
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
      if gearbox.outputAV1 * gearbox.gearRatio * engine.outputAV1 >= 0 then
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
  -- old -> N -> wait -> new -> in gear update
  M.brake = M.inputValues.brake
  M.throttle = M.inputValues.throttle
  M.isArcadeSwitched = false
  M.isShifting = true

  --if we are shifting into neutral we need to delay this a little bit because the user might use an H pattern shifter which goes through neutral on every shift
  --if we were not to delay this neutral shift, the user can't get out of 1st gear due to gear change limitations of the sequential
  --so only shift to neutral if the new desired gear is still neutral after 0.x seconds (ie the user actually left the H shifter in neutral and did not move to the next gear)
  if newDesiredGearIndex == 0 and neutralRejectTimer > 0 then
    neutralRejectTimer = neutralRejectTimer - dt
  else
    if newDesiredGearIndex > gearbox.gearIndex and gearbox.gearIndex > 0 and M.throttle > 0 then
      engine:cutIgnition(ignitionCutTime)
    end

    gearbox:setGearIndex(newDesiredGearIndex)
    newDesiredGearIndex = 0
    M.timer.gearChangeDelayTimer = M.timerConstants.gearChangeDelay
    M.updateGearboxGFX = gearboxLogic.inGear
  end

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

  M.currentGearIndex = 0
  M.maxGearIndex = 0
  M.minGearIndex = 0
  M.throttle = 0
  M.brake = 0
  M.clutchRatio = 0
  M.shiftingAggression = 1
  M.isArcadeSwitched = false
  M.isSportModeActive = false

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

  ignitionCutTime = jbeamData.ignitionCutTime or 0.15

  gearboxAvailableLogic = {
    arcade = {
      inGear = updateInGearArcade,
      whileShifting = updateWhileShiftingArcade,
      shiftUp = sharedFunctions.warnCannotShiftSequential,
      shiftDown = sharedFunctions.warnCannotShiftSequential,
      shiftToGearIndex = sharedFunctions.switchToRealisticBehavior
    },
    realistic = {
      inGear = updateInGear,
      whileShifting = updateWhileShifting,
      shiftUp = shiftUp,
      shiftDown = shiftDown,
      shiftToGearIndex = shiftToGearIndex
    }
  }

  clutchHandling.clutchLaunchTargetAV = (jbeamData.clutchLaunchTargetRPM or 3000) * constants.rpmToAV * 0.5
  clutchHandling.clutchLaunchStartAV = ((jbeamData.clutchLaunchStartRPM or 2000) * constants.rpmToAV - engine.idleAV) * 0.5
  clutchHandling.clutchLaunchIFactor = 0
  clutchHandling.lastClutchInput = 0

  M.maxRPM = engine.maxRPM
  M.idleRPM = engine.idleRPM
  M.maxGearIndex = gearbox.maxGearIndex
  M.minGearIndex = abs(gearbox.minGearIndex)
  M.energyStorages = sharedFunctions.getEnergyStorages({engine})
end

local function getState()
  local data = {grb_idx = gearbox.gearIndex}

  return tableIsEmpty(data) and nil or data
end

local function setState(data)
  if data.grb_idx then
    shiftToGearIndex(data.grb_idx, true)
  end
end

M.init = init

M.gearboxBehaviorChanged = gearboxBehaviorChanged
M.shiftUp = shiftUp
M.shiftDown = shiftDown
M.shiftToGearIndex = shiftToGearIndex
M.updateGearboxGFX = nop
M.getGearName = getGearName
M.getGearPosition = getGearPosition
M.sendTorqueData = sendTorqueData

M.getState = getState
M.setState = setState

return M
