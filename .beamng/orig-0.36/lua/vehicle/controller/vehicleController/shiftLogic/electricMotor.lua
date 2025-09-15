-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local max = math.max
local min = math.min
local abs = math.abs
local floor = math.floor

local constants = {rpmToAV = 0.104719755, avToRPM = 9.549296596425384}

local motors = nil

local sharedFunctions = nil
local gearboxAvailableLogic = nil
local gearboxLogic = nil

local hasRegisteredQuickAccessMenu

M.gearboxHandling = nil
M.timer = nil
M.timerConstants = nil
M.inputValues = nil
M.shiftPreventionData = nil
M.shiftBehavior = nil
M.smoothedValues = nil

M.currentGearIndex = 0
M.maxGearIndex = 1
M.minGearIndex = -1
M.throttle = 0
M.brake = 0
M.regen = 0
M.clutchRatio = 1
M.shiftingAggression = 0
M.throttleInput = 0
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

local automaticHandling = {
  availableModes = {"P", "R", "N", "D"},
  hShifterModeLookup = {[-1] = "R", [0] = "N", "P", "D"},
  gearIndexLookup = {P = -2, R = -1, N = 0, D = 1},
  availableModeLookup = {},
  existingModeLookup = {},
  modeIndexLookup = {},
  modes = {},
  mode = nil,
  modeIndex = 0,
  maxAllowedGearIndex = 0,
  minAllowedGearIndex = 0
}

local regenHandling = {
  smoother = nil,
  smootherRateGain = 0,
  unavailableGears = {P = true, N = true},
  onePedalRegenCoef = 0.5,
  onePedalFrictionBrakeCoef = 1,
  numStrengthLevels = 3,
  strengthLevel = 0,
  regenTorqueToCoef = {},
  regenCoefToTorque = {},
  currentAvgFadeCoef = 0,
  instantMaxRegenTorque = 0,
  desiredOnePedalTorque = 0,
  currentRegenTorque = 0
}

local brakeHandling = {
  smoother = nil,
  smoothStopTimeToLatch = 0.25,
  smoothStopLatchTime = 0,
  smoothStopReleaseAmount = 0.85,
  maxFrictionBrakeTorque = 0,
  frictionTorqueToCoef = {},
  frictionCoefToTorque = {}
}

local function getGearName()
  return automaticHandling.mode
end

local function getGearPosition()
  return (automaticHandling.modeIndex - 1) / (#automaticHandling.modes - 1), automaticHandling.modeIndex
end

local function gearboxBehaviorChanged(behavior)
  gearboxLogic = gearboxAvailableLogic[behavior]
  M.updateGearboxGFX = gearboxLogic.inGear
  M.shiftUp = gearboxLogic.shiftUp
  M.shiftDown = gearboxLogic.shiftDown
  M.shiftToGearIndex = gearboxLogic.shiftToGearIndex
end

local function applyGearboxMode()
  local autoIndex = automaticHandling.modeIndexLookup[automaticHandling.mode]
  if autoIndex then
    automaticHandling.modeIndex = min(max(autoIndex, 1), #automaticHandling.modes)
    automaticHandling.mode = automaticHandling.modes[automaticHandling.modeIndex]
  end

  local motorDirection = 1 --D
  if automaticHandling.mode == "P" then
    motorDirection = 0
  elseif automaticHandling.mode == "N" then
    motorDirection = 0
  elseif automaticHandling.mode == "R" then
    motorDirection = -1
  end

  for _, v in ipairs(motors) do
    v.motorDirection = motorDirection
  end

  M.isSportModeActive = automaticHandling.mode == "S"
end

local function shiftUp()
  if automaticHandling.mode == "N" then
    M.timer.gearChangeDelayTimer = M.timerConstants.gearChangeDelay
  end

  automaticHandling.modeIndex = min(automaticHandling.modeIndex + 1, #automaticHandling.modes)
  automaticHandling.mode = automaticHandling.modes[automaticHandling.modeIndex]

  applyGearboxMode()
end

local function shiftDown()
  if automaticHandling.mode == "N" then
    M.timer.gearChangeDelayTimer = M.timerConstants.gearChangeDelay
  end

  automaticHandling.modeIndex = max(automaticHandling.modeIndex - 1, 1)
  automaticHandling.mode = automaticHandling.modes[automaticHandling.modeIndex]

  applyGearboxMode()
end

local function shiftToGearIndex(index)
  local desiredMode = automaticHandling.hShifterModeLookup[index]
  if not desiredMode or not automaticHandling.existingModeLookup[desiredMode] then
    if desiredMode and not automaticHandling.existingModeLookup[desiredMode] then
      guihooks.message({txt = "vehicle.vehicleController.cannotShiftAuto", context = {mode = desiredMode}}, 2, "vehicle.shiftLogic.cannotShift")
    end
    desiredMode = "N"
  end
  automaticHandling.mode = desiredMode

  applyGearboxMode()
end

local function updateExposedData()
  local motorCount = 0
  M.rpm = 0
  local load = 0
  local motorTorque = 0
  for _, v in ipairs(motors) do
    M.rpm = max(M.rpm, abs(v.outputAV1) * constants.avToRPM)
    load = load + (v.engineLoad or 0)
    motorTorque = motorTorque + (v.outputTorque1 or 0)
    motorCount = motorCount + 1
  end
  load = load / motorCount

  M.smoothedAvgAVInput = sharedFunctions.updateAvgAVDeviceCategory("engine")
  M.waterTemp = 0
  M.oilTemp = 0
  M.checkEngine = 0
  M.ignition = electrics.values.ignitionLevel > 1
  M.engineThrottle = M.throttle
  M.engineLoad = load
  M.running = electrics.values.ignitionLevel > 1
  M.engineTorque = motorTorque
  M.flywheelTorque = motorTorque
  M.gearboxTorque = motorTorque
  M.isEngineRunning = 1
end

local function updateRegen(dt)
  local avgRegenFadeCoef = 0
  local maxRegenTorque = 0
  local currentRegenTorque = 0

  electrics.values.regenFromBrake = 0
  electrics.values.regenFromOnePedal = 0

  for _, motor in ipairs(motors) do
    local motorRPM = abs(motor.outputAV1 * constants.avToRPM)
    local regenFadeCoef = 1 - min(1, motorRPM / motor.minPeakRegenRPM)

    avgRegenFadeCoef = avgRegenFadeCoef + regenFadeCoef / #motors
    maxRegenTorque = maxRegenTorque + motor.maxRegenTorque * motor.cumulativeGearRatio
    currentRegenTorque = currentRegenTorque + M.regen * motor.instantMaxRegenTorque * motor.cumulativeGearRatio
  end

  regenHandling.currentAvgFadeCoef = avgRegenFadeCoef
  regenHandling.instantMaxRegenTorque = maxRegenTorque
  regenHandling.currentRegenTorque = currentRegenTorque
  regenHandling.smootherRateGain = 1 + max(0, min(1, M.throttle * 2)) -- scale smoother rate with throttle input to release regen/brakes faster during sudden acceleration

  if regenHandling.unavailableGears[automaticHandling.mode] then
    M.regen = 0
    regenHandling.smoother:reset()
    regenHandling.strengthLevel = 0
    regenHandling.desiredOnePedalTorque = 0
  else
    regenHandling.strengthLevel = min(regenHandling.numStrengthLevels, electrics.values.regenStrength or 0)

    local onePedalDrivingCoef = regenHandling.strengthLevel / regenHandling.numStrengthLevels
    local onePedalRegenCoef = 0

    if onePedalDrivingCoef > 0 then
      local maxOffset = regenHandling.throttleNeutralPoint
      local throttleOffset = maxOffset * max(0.25, min(1, electrics.values.wheelspeed / 2.5))
      local positiveThrottle = max(0, (M.throttle - throttleOffset) / (1 - throttleOffset))
      local negativeThrottle = max(0, min(1, (throttleOffset - M.throttle) / throttleOffset))

      onePedalRegenCoef = onePedalDrivingCoef * regenHandling.onePedalRegenCoef * negativeThrottle

      M.throttle = positiveThrottle
    end

    -- when applying throttle while completely stopped, the smoother should be reset and regen should cancel immediately
    -- (otherwise there might be a slight perceived delay to the throttle response)
    if M.smoothedValues.avgAV < 0.5 and M.throttle > 1e-2 then
      onePedalRegenCoef = 0
      regenHandling.smoother:reset()
    end

    onePedalRegenCoef = regenHandling.smoother:get(onePedalRegenCoef, dt * regenHandling.smootherRateGain)
    regenHandling.desiredOnePedalTorque = regenHandling.regenCoefToTorque[floor(onePedalRegenCoef * 1000)] or (onePedalDrivingCoef * regenHandling.instantMaxRegenTorque)

    local frictionBrakeTorqueDemand = brakeHandling.frictionCoefToTorque[floor(M.brake * 1000)] or (M.brake * brakeHandling.maxFrictionBrakeTorque)
    local equivalentRegenCoefForBrakeDemand = regenHandling.regenTorqueToCoef[floor(frictionBrakeTorqueDemand)] or 1
    local brakePedalRegenCoef = (M.isArcadeSwitched or M.throttle > 0) and 0 or equivalentRegenCoefForBrakeDemand
    local finalRegenCoef = min(brakePedalRegenCoef + onePedalRegenCoef, 1)
    local emergencyBrakeCoef = M.brake >= 1 and 0.5 or 1
    local escCoef = electrics.values.escActive and 0 or 1
    local steeringCoef = abs(sensors.gx2) > 5 and 0.8 or 1

    M.regen = finalRegenCoef * escCoef * steeringCoef * emergencyBrakeCoef
    electrics.values.regenFromBrake = brakePedalRegenCoef
    electrics.values.regenFromOnePedal = onePedalRegenCoef
  end

  electrics.values.maxRegenStrength = regenHandling.numStrengthLevels
  electrics.values.regenThrottle = M.regen
end

local function updateBrakes(dt)
  if not regenHandling.unavailableGears[automaticHandling.mode] then
    local frictionBrakeTorqueDemand = brakeHandling.frictionCoefToTorque[floor(M.brake * 1000)] or (M.brake * brakeHandling.maxFrictionBrakeTorque)
    local actualBrakePedalRegenTorque = max(0, regenHandling.currentRegenTorque - regenHandling.desiredOnePedalTorque)
    local frictionBrakeDemandAfterRegen = max(0, frictionBrakeTorqueDemand - actualBrakePedalRegenTorque)
    local adjustedBrakeCoef = frictionBrakeDemandAfterRegen / brakeHandling.maxFrictionBrakeTorque
    local regenFadeCompensationBrakeCoef = 0

    -- When 1-pedal driving is at the strongest setting, blend friction brakes to bring the car to a stop and hold it there
    if regenHandling.strengthLevel == regenHandling.numStrengthLevels and M.regen > 1e-5 then
      local maxOnePedalRegenTorque = regenHandling.instantMaxRegenTorque * regenHandling.onePedalRegenCoef
      local brakeCoefForMaxRegen = brakeHandling.frictionTorqueToCoef[floor(maxOnePedalRegenTorque)] or 0
      local smoothStopCoef = 1

      if brakeHandling.smoothStopLatchTime < brakeHandling.smoothStopTimeToLatch then
        -- to achieve a smooth, comfortable stop, the brakes are gently released as the vehicle passes below 1 m/s
        -- however, once the vehicle stops completely, the brakes are latched "full on" to prevent the vehicle moving accidentally
        local smoothStopProgress = 1 - min(1, electrics.values.wheelspeed / 2) -- increases from [0..1] as vehicle slows from 2 to 0 m/s

        smoothStopCoef = 1 - smoothStopProgress * brakeHandling.smoothStopReleaseAmount -- decreases from [1..x] to gradually release brakes

        local desiredSpeedSign = electrics.values.gear == "R" and -1 or 1

        if M.smoothedAvgAVInput * desiredSpeedSign < -0.5 then -- if vehicle starts rolling back, immediately latch (don't want to use smoothed value for this)
          brakeHandling.smoothStopLatchTime = brakeHandling.smoothStopTimeToLatch
        elseif abs(M.smoothedValues.avgAV) < 0.05 then -- determine if vehicle is stopped
          brakeHandling.smoothStopLatchTime = brakeHandling.smoothStopLatchTime + dt
        end
      end

      regenFadeCompensationBrakeCoef = regenHandling.currentAvgFadeCoef * brakeCoefForMaxRegen * regenHandling.onePedalFrictionBrakeCoef * smoothStopCoef
    else
      brakeHandling.smoothStopLatchTime = 0
      brakeHandling.smoother:reset()
    end

    if M.throttle > 0.25 then
      -- to prevent brakes from "sticking" during sudden acceleration from a stop, smoother is immediately reset if throttle exceeds 25%
      brakeHandling.smoother:reset()
    end

    regenFadeCompensationBrakeCoef = brakeHandling.smoother:get(regenFadeCompensationBrakeCoef, dt * regenHandling.smootherRateGain)

    if regenFadeCompensationBrakeCoef < 0.01 then
      -- so brake lights don't linger and pads don't drag a tiny bit as the smoother levels off
      regenFadeCompensationBrakeCoef = 0
    end

    adjustedBrakeCoef = max(adjustedBrakeCoef, regenFadeCompensationBrakeCoef)
    M.brake = adjustedBrakeCoef
  end
end

local function updateInGearArcade(dt)
  M.throttle = M.inputValues.throttle
  M.brake = M.inputValues.brake
  M.isArcadeSwitched = false
  M.clutchRatio = 1

  local gearIndex = automaticHandling.gearIndexLookup[automaticHandling.mode]
  gearIndex = (automaticHandling.mode == "N" or automaticHandling.mode == "P") and 0 or gearIndex --adjust lookup so that P and N return 0, it's needed for following code
  -- driving backwards? - only with automatic shift - for obvious reasons ;)
  if (gearIndex < 0 and M.smoothedValues.avgAV <= 0.8) or (gearIndex <= 0 and M.smoothedValues.avgAV < -1) then
    M.throttle, M.brake = M.brake, M.throttle
    M.isArcadeSwitched = true
  end

  -- neutral gear handling
  if M.timer.neutralSelectionDelayTimer <= 0 then
    if automaticHandling.mode ~= "P" and abs(M.smoothedValues.avgAV) < M.gearboxHandling.arcadeAutoBrakeAVThreshold and M.throttle <= 0 then
      M.brake = max(M.brake, M.gearboxHandling.arcadeAutoBrakeAmount)
    end

    if automaticHandling.mode ~= "N" and abs(M.smoothedValues.avgAV) < M.gearboxHandling.arcadeAutoBrakeAVThreshold and M.smoothedValues.throttle <= 0 then
      gearIndex = 0
      automaticHandling.mode = "N"
      applyGearboxMode()
    else
      if M.smoothedValues.throttleInput > 0 and M.inputValues.throttle > 0 and M.smoothedValues.brakeInput <= 0 and M.smoothedValues.avgAV > -1 and gearIndex < 1 then
        gearIndex = 1
        M.timer.neutralSelectionDelayTimer = M.timerConstants.neutralSelectionDelay
        automaticHandling.mode = "D"
        applyGearboxMode()
      end

      if M.smoothedValues.brakeInput > 0.1 and M.inputValues.brake > 0 and M.smoothedValues.throttleInput <= 0 and M.smoothedValues.avgAV <= 0.5 and gearIndex > -1 then
        gearIndex = -1
        M.timer.neutralSelectionDelayTimer = M.timerConstants.neutralSelectionDelay
        automaticHandling.mode = "R"
        applyGearboxMode()
      end
    end

    if electrics.values.ignitionLevel <= 1 and automaticHandling.mode ~= "P" then
      gearIndex = 0
      M.timer.neutralSelectionDelayTimer = M.timerConstants.neutralSelectionDelay
      automaticHandling.mode = "P"
      applyGearboxMode()
    end
  end

  updateRegen(dt)
  updateBrakes(dt)

  if automaticHandling.mode == "P" then
    M.brake = max(M.brake, M.gearboxHandling.arcadeAutoBrakeAmount)
  end

  M.currentGearIndex = (automaticHandling.mode == "N" or automaticHandling.mode == "P") and 0 or gearIndex
  updateExposedData()
end

local function updateInGear(dt)
  M.throttle = M.inputValues.throttle
  M.brake = M.inputValues.brake
  M.isArcadeSwitched = false
  M.clutchRatio = 1

  updateRegen(dt)
  updateBrakes(dt)

  if electrics.values.ignitionLevel <= 1 and automaticHandling.mode ~= "P" then
    M.timer.neutralSelectionDelayTimer = M.timerConstants.neutralSelectionDelay
    automaticHandling.mode = "P"
    applyGearboxMode()
  end
  local gearIndex = automaticHandling.gearIndexLookup[automaticHandling.mode]
  M.currentGearIndex = (automaticHandling.mode == "N" or automaticHandling.mode == "P") and 0 or gearIndex
  if automaticHandling.mode == "P" then
    M.brake = max(M.brake, M.gearboxHandling.arcadeAutoBrakeAmount)
  end
  updateExposedData()
end

local function sendTorqueData()
  for _, v in ipairs(motors) do
    v:sendTorqueData()
  end
end

local function setIgnition(enabled)
  for _, motor in ipairs(motors) do
    motor:setIgnition(enabled and 1 or 0)
  end
end

local function registerQuickAccessMenu()
  if not core_quickAccess or hasRegisteredQuickAccessMenu then
    return
  end
  hasRegisteredQuickAccessMenu = true

  -- electric motor regeneration
  core_quickAccess.addEntry(
    {
      level = "/root/playerVehicle/vehicleFeatures/",
      generator = function(entries)
        local hasElectricMotor = #powertrain.getDevicesByType("electricMotor") > 0

        if hasElectricMotor and electrics.values.maxRegenStrength then
          -- TODO: icon
          table.insert(entries, {title = "ui.radialmenu2.powertrain.regen", ["goto"] = "/root/playerVehicle/vehicleFeatures/regen/", icon = "charging", uniqueID = "regen"})
        end
      end
    }
  )

  -- regenerative braking
  -- TODO: Icons
  core_quickAccess.addEntry(
    {
      level = "/root/playerVehicle/vehicleFeatures/regen/",
      generator = function(entries)
        local hasElectricMotor = #powertrain.getDevicesByType("electricMotor") > 0
        if hasElectricMotor and electrics.values.maxRegenStrength then
          local maxStrength = electrics.values.maxRegenStrength or 3

          for strength = 0, maxStrength do
            local title, icon, context
            local numItems = maxStrength + 1
            local priority = (strength + math.floor(numItems / 2)) % numItems * 10

            if strength == 0 then
              title = "ui.radialmenu2.powertrain.regen.off"
              icon = "powerOnOff"
            elseif strength == maxStrength then
              title = "ui.radialmenu2.powertrain.regen.full"
              icon = "charging"
            else
              title = "ui.radialmenu2.powertrain.regen.level"
              icon = "charge"
              context = {level = strength, percent = math.floor(strength * 100 / maxStrength + 0.5)}
            end

            local e = {
              title = title,
              icon = icon,
              context = context,
              priority = priority,
              originalActionInfo = {level = "/root/playerVehicle/vehicleFeatures/", uniqueID = "regen"},
              onSelect = function()
                electrics.values.regenStrength = strength
                return {"reload"}
              end
            }

            if (electrics.values.regenStrength or 0) == strength then
              e.color = "#ff6600"
            end

            table.insert(entries, e)
          end
        end
      end
    }
  )
end

local function init(jbeamData, sharedFunctionTable)
  sharedFunctions = sharedFunctionTable

  M.currentGearIndex = 0
  M.maxGearIndex = 1
  M.minGearIndex = -1
  M.throttle = 0
  M.brake = 0
  M.regen = 0
  M.clutchRatio = 1
  M.shiftingAggression = 0
  M.throttleInput = 0
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

  gearboxAvailableLogic = {
    arcade = {
      inGear = updateInGearArcade,
      shiftUp = sharedFunctions.warnCannotShiftSequential,
      shiftDown = sharedFunctions.warnCannotShiftSequential,
      shiftToGearIndex = sharedFunctions.switchToRealisticBehavior
    },
    realistic = {
      inGear = updateInGear,
      shiftUp = shiftUp,
      shiftDown = shiftDown,
      shiftToGearIndex = shiftToGearIndex
    }
  }

  motors = {}
  local motorNames = jbeamData.motorNames or {"mainMotor"}
  motorNames = type(motorNames) ~= "table" and {motorNames} or motorNames
  for _, v in ipairs(motorNames) do
    local motor = powertrain.getDevice(v)
    if motor then
      M.maxRPM = max(M.maxRPM, motor.maxAV * constants.avToRPM)
      table.insert(motors, motor)
    end
  end

  if #motors <= 0 then
    log("E", "shiftLogic-electricMotor", "No motors have been specified, functionality will be limited!")
  end

  -- determine maximum available friction brake torque
  local totalMaxBrakeTorque = 0
  for _, wd in pairs(wheels.wheels) do
    totalMaxBrakeTorque = totalMaxBrakeTorque + wd.brakeTorque * (wd.brakeInputSplit + (1 - wd.brakeInputSplit) * wd.brakeSplitCoef)
  end

  -- create two complimentary curves to map between a "brake input" coefficient and the resulting actual brake torque
  local tempFrictionCoefToTorqueMap = {}
  local tempFrictionTorqueToCoefMap = {}
  for i = 0, 100 do
    local brakeCoef = i / 100
    local totalBrakeTorque = 0
    for _, wd in pairs(wheels.wheels) do
      totalBrakeTorque = totalBrakeTorque + wd.brakeTorque * (min(brakeCoef, wd.brakeInputSplit) + max(brakeCoef - wd.brakeInputSplit, 0) * wd.brakeSplitCoef)
    end
    table.insert(tempFrictionCoefToTorqueMap, {i * 10, totalBrakeTorque})
    table.insert(tempFrictionTorqueToCoefMap, {totalBrakeTorque, brakeCoef})
  end

  local regenSmoothingRate = jbeamData.regenSmoothingRate or 20
  local regenSmoothingAccel = jbeamData.regenSmoothingAccel or 5

  brakeHandling.smoother = newTemporalSigmoidSmoothing(regenSmoothingRate * 2, regenSmoothingAccel * 2, regenSmoothingAccel, regenSmoothingRate)
  brakeHandling.smoothStopReleaseAmount = jbeamData.onePedalSmoothStopBrakeReleaseAmount or 0.85
  brakeHandling.maxFrictionBrakeTorque = totalMaxBrakeTorque
  brakeHandling.frictionCoefToTorque = createCurve(tempFrictionCoefToTorqueMap)
  brakeHandling.frictionTorqueToCoef = createCurve(tempFrictionTorqueToCoefMap)

  -- create a curve to map between a desired regen torque and the necessary "regen throttle" (or "coefficient") to achieve that torque
  local tempRegenCoefToTorqueMap = {}
  local tempRegenTorqueToCoefMap = {}
  for i = 0, 100 do
    local regenCoef = i / 100
    local totalRegenTorque = 0
    for _, motor in pairs(motors) do
      totalRegenTorque = totalRegenTorque + regenCoef * motor.maxRegenTorque * motor.cumulativeGearRatio
    end
    table.insert(tempRegenCoefToTorqueMap, {i * 10, totalRegenTorque})
    table.insert(tempRegenTorqueToCoefMap, {totalRegenTorque, regenCoef})
  end

  regenHandling.smoother = newTemporalSigmoidSmoothing(regenSmoothingRate, regenSmoothingAccel, regenSmoothingAccel, regenSmoothingRate)
  regenHandling.onePedalRegenCoef = jbeamData.onePedalRegenCoef or 0.5
  regenHandling.onePedalFrictionBrakeCoef = jbeamData.onePedalFrictionBrakeCoef or 1
  regenHandling.numStrengthLevels = jbeamData.regenStrengthLevels or 3
  regenHandling.strengthLevel = electrics.values.regenStrength or jbeamData.defaultRegenStrength or regenHandling.numStrengthLevels
  regenHandling.throttleNeutralPoint = jbeamData.regenThrottleNeutralPoint or 0.15
  regenHandling.regenCoefToTorque = createCurve(tempRegenCoefToTorqueMap)
  regenHandling.regenTorqueToCoef = createCurve(tempRegenTorqueToCoefMap)

  electrics.values.regenStrength = regenHandling.strengthLevel

  automaticHandling.availableModeLookup = {}
  for _, v in pairs(automaticHandling.availableModes) do
    automaticHandling.availableModeLookup[v] = true
  end

  automaticHandling.modes = {}
  automaticHandling.modeIndexLookup = {}
  local modes = jbeamData.automaticModes or "PRND"
  local modeCount = #modes
  local modeOffset = 0
  for i = 1, modeCount do
    local mode = modes:sub(i, i)
    if automaticHandling.availableModeLookup[mode] then
      automaticHandling.modes[i + modeOffset] = mode
      automaticHandling.modeIndexLookup[mode] = i + modeOffset
      automaticHandling.existingModeLookup[mode] = true
    else
      print("unknown auto mode: " .. mode)
    end
  end

  local defaultMode = jbeamData.defaultAutomaticMode or "P"
  automaticHandling.modeIndex = string.find(modes, defaultMode)
  automaticHandling.mode = automaticHandling.modes[automaticHandling.modeIndex]
  automaticHandling.maxGearIndex = 1
  automaticHandling.minGearIndex = -1

  M.idleRPM = 0
  M.maxGearIndex = automaticHandling.maxGearIndex
  M.minGearIndex = abs(automaticHandling.minGearIndex)
  M.energyStorages = sharedFunctions.getEnergyStorages(motors)

  registerQuickAccessMenu()

  applyGearboxMode()
end

local function onDeserialize(data)
  if data.regenLevel then
    electrics.values.regenStrength = min(regenHandling.numStrengthLevels, data.regenLevel)
  end
end

local function onSerialize()
  return {regenLevel = electrics.values.regenStrength or 0}
end

local function getState()
  local data = {grb_mde = automaticHandling.mode}

  return tableIsEmpty(data) and nil or data
end

local function setState(data)
  if data.grb_mde then
    automaticHandling.mode = data.grb_mde
    automaticHandling.modeIndex = automaticHandling.modeIndexLookup[automaticHandling.mode]
    applyGearboxMode()
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
M.setIgnition = setIgnition
M.onDeserialize = onDeserialize
M.onSerialize = onSerialize

M.getState = getState
M.setState = setState

return M
