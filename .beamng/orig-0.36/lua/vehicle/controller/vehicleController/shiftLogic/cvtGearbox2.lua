-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local max = math.max
local min = math.min
local abs = math.abs

local constants = {rpmToAV = 0.104719755, avToRPM = 9.549296596425384}

local gearbox = nil
local engine = nil
local torqueConverter = nil

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

M.engineTorqueData = {}
M.torqueToMinRPM = {}
M.currentGearIndex = 0
M.maxGearIndex = 1
M.minGearIndex = -1
M.throttle = 0
M.brake = 0
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
  availableModes = {"P", "R", "N", "D", "S"},
  hShifterModeLookup = {[-1] = "R", [0] = "N", "P", "D", "S"},
  cvtGearIndexLookup = {P = -2, R = -1, N = 0, D = 1, S = 2},
  availableModeLookup = {},
  existingModeLookup = {},
  modeIndexLookup = {},
  modes = {},
  mode = nil,
  modeIndex = 0,
  maxAllowedGearIndex = 0,
  minAllowedGearIndex = 0
}

local cvtHandling = {
  targetRpmMap = nil,
  fixedGearRatios = {},
  fixedGearRatioIndex = 1,
  useFixedGearRatios = false,
  sportGearShiftCooldown = 0,
  remainingShiftCooldown = 0
}

local smoother = {
  --gearRatio smoother reduces oscillations during sharp changes in throttle
  cvtGearRatioSmoother = nil,
  --lockup clutch smoothing
  lockupSmoother = nil
}

local torqueConverterHandling = {
  lockupMinWheelspeed = 0,
  lockupMinAvDiff = 0,
  lockupMaxAvDiff = 0,
  hasLockup = false,
  shouldLockUp = false
}

local function getGearName()
  local modePrefix = ""
  if automaticHandling.mode == "S" then
    modePrefix = "S"
  elseif string.sub(automaticHandling.mode, 1, 1) == "M" then
    modePrefix = "M"
  end
  return modePrefix ~= "" and modePrefix .. tostring(cvtHandling.fixedGearRatioIndex) or automaticHandling.mode
end

local function getGearPosition()
  return (automaticHandling.modeIndex - 1) / (#automaticHandling.modes - 1), automaticHandling.modeIndex
end

local function applyGearboxModeRestrictions()
  local manualModeIndex
  if string.sub(automaticHandling.mode, 1, 1) == "M" then
    manualModeIndex = string.sub(automaticHandling.mode, 2)
  end
  local maxGearIndex = gearbox.maxGearIndex
  local minGearIndex = gearbox.minGearIndex
  if automaticHandling.mode == "1" then
    maxGearIndex = 1
    minGearIndex = 1
  elseif automaticHandling.mode == "2" then
    maxGearIndex = 2
    minGearIndex = 1
  elseif manualModeIndex then
    maxGearIndex = manualModeIndex
    minGearIndex = manualModeIndex
  end

  automaticHandling.maxGearIndex = maxGearIndex
  automaticHandling.minGearIndex = minGearIndex
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

  cvtHandling.useFixedGearRatios = false
  if automaticHandling.mode == "P" then
    gearbox:setMode("park")
  elseif automaticHandling.mode == "N" then
    gearbox:setMode("neutral")
  elseif automaticHandling.mode == "R" then
    gearbox:setMode("reverse")
  else
    gearbox:setMode("drive")
    if automaticHandling.mode == "S" then
      cvtHandling.useFixedGearRatios = true
    end
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
  applyGearboxModeRestrictions()
end

local function shiftDown()
  if automaticHandling.mode == "N" then
    M.timer.gearChangeDelayTimer = M.timerConstants.gearChangeDelay
  end

  automaticHandling.modeIndex = max(automaticHandling.modeIndex - 1, 1)
  automaticHandling.mode = automaticHandling.modes[automaticHandling.modeIndex]

  applyGearboxMode()
  applyGearboxModeRestrictions()
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
  applyGearboxModeRestrictions()
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
  M.shiftingAggression = M.smoothedValues.drivingAggression
end

local function findFixedGearRatio(desiredGearRatio, dt)
  -- streams.drawGraph("Desired GR", { value = desiredGearRatio, color = "rgba(0, 0, 0, 0.5)" })

  if not cvtHandling.useFixedGearRatios then
    return desiredGearRatio
  end

  local fixedGearRatio = cvtHandling.fixedGearRatios[cvtHandling.fixedGearRatioIndex]
  local didDownshift = false
  local canShift = cvtHandling.remainingShiftCooldown <= 0

  if cvtHandling.remainingShiftCooldown > 0 then
    cvtHandling.remainingShiftCooldown = cvtHandling.remainingShiftCooldown - dt
  end

  -- Reminder: "higher gears" mean a LOWER gear ratio value!

  -- Check to see if we need to downshift (desired gear ratio is greater than the next-lowest gear)
  if cvtHandling.fixedGearRatioIndex > 1 then
    local priorFixedGearRatio = cvtHandling.fixedGearRatios[cvtHandling.fixedGearRatioIndex - 1]
    -- streams.drawGraph("Prior GR", { value = priorFixedGearRatio, color = "#ff0000" })

    if desiredGearRatio >= priorFixedGearRatio and canShift then
      -- Downshift!
      cvtHandling.fixedGearRatioIndex = cvtHandling.fixedGearRatioIndex - 1
      fixedGearRatio = priorFixedGearRatio
      cvtHandling.remainingShiftCooldown = cvtHandling.sportGearShiftCooldown
      didDownshift = true
    end
  else
    -- streams.drawGraph("Prior GR", { value = fixedGearRatio, color = "#ff0000" }) -- just to make it have a value
  end

  -- Check to see if we need to upshift (desired gear ratio is less than some shift point between the current and the next-highest gear, or RPM is too high)
  local maxDesiredRpm = cvtHandling.targetRpmMap:get(cvtHandling.targetRpmMap.xMax, cvtHandling.targetRpmMap.yMax)
  local engineRpm = engine.outputAV1 * constants.avToRPM

  if cvtHandling.fixedGearRatioIndex < #cvtHandling.fixedGearRatios and not didDownshift then
    local nextFixedGearRatio = cvtHandling.fixedGearRatios[cvtHandling.fixedGearRatioIndex + 1]
    local shiftThreshold = lerp(fixedGearRatio, nextFixedGearRatio, M.smoothedValues.drivingAggression) -- higher aggression means a higher shift point
    -- streams.drawGraph("Upshift Threshold", { value = shiftThreshold, color = "#00ffff" })
    -- streams.drawGraph("Next GR", { value = nextFixedGearRatio, color = "#00ff00" })

    if (engineRpm > maxDesiredRpm or desiredGearRatio <= shiftThreshold) and canShift then
      -- Upshift!
      cvtHandling.fixedGearRatioIndex = cvtHandling.fixedGearRatioIndex + 1
      fixedGearRatio = nextFixedGearRatio
      cvtHandling.remainingShiftCooldown = cvtHandling.sportGearShiftCooldown
    end
  else
    -- streams.drawGraph("Next GR", { value = fixedGearRatio, color = "#00ff00" }) -- just to make it have a value
  end

  -- streams.drawGraph("Fixed GR", { value = fixedGearRatio, color = "#0000ff" })

  return fixedGearRatio
end

local function updateInGearArcade(dt)
  M.throttle = M.inputValues.throttle
  M.brake = M.inputValues.brake
  M.isArcadeSwitched = false

  local gearIndex = automaticHandling.cvtGearIndexLookup[automaticHandling.mode]
  -- driving backwards? - only with automatic shift - for obvious reasons ;)
  if (gearIndex < 0 and M.smoothedValues.avgAV <= 0.8) or (gearIndex <= 0 and M.smoothedValues.avgAV < -1) then
    M.throttle, M.brake = M.brake, M.throttle
    M.isArcadeSwitched = true
  end

  --interpolate based on throttle between high/low ranges
  local wheelspeed = electrics.values.wheelspeed
  local engineTargetRPM = cvtHandling.targetRpmMap:get(wheelspeed, M.smoothedValues.drivingAggression)
  local engineTargetAV = engineTargetRPM * constants.rpmToAV
  local targetGearRatio = automaticHandling.mode == "R" and gearbox.maxGearRatio or engineTargetAV / max(1, gearbox.outputAV1)
  local desiredGearRatio, smoothDesiredGearRatio

  desiredGearRatio = clamp(targetGearRatio, gearbox.minGearRatio, gearbox.maxGearRatio)
  desiredGearRatio = findFixedGearRatio(desiredGearRatio, dt)
  smoothDesiredGearRatio = smoother.cvtGearRatioSmoother:get(desiredGearRatio, dt)
  gearbox:setGearRatio(smoothDesiredGearRatio)

  if torqueConverterHandling.hasLockup then
    local avDiff = max(0, engineTargetAV - engine.outputAV1)
    local maxLockupBySpeed = clamp((wheelspeed - torqueConverterHandling.lockupMinWheelspeed) / torqueConverterHandling.lockupMinWheelspeed, 0, 1) -- ramp up from minWheelspeed to twice minWheelspeed
    local maxLockupByThrottle = min(1, 0.5 + M.throttle / 0.05)
    local maxAllowedLockup = min(maxLockupBySpeed, maxLockupByThrottle)

    if avDiff > torqueConverterHandling.lockupMaxAvDiff or engine.outputAV1 < engine.idleAV then
      torqueConverterHandling.shouldLockUp = false
    elseif avDiff > torqueConverterHandling.lockupMinAvDiff then
      torqueConverterHandling.shouldLockUp = true
    end

    local targetLockup = min(maxAllowedLockup, torqueConverterHandling.shouldLockUp and 1 or 0)

    electrics.values.lockupClutchRatio = smoother.lockupSmoother:get(targetLockup, dt)
  else
    electrics.values.lockupClutchRatio = 0
    smoother.lockupSmoother:set(0)
  end

  -- neutral gear handling
  if abs(gearbox.gearIndex) <= 1 and M.timer.neutralSelectionDelayTimer <= 0 then
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

      if M.smoothedValues.brakeInput > 0.1 and M.inputValues.brake > 0 and M.smoothedValues.throttleInput <= 0 and M.smoothedValues.avgAV <= 0.15 and gearIndex > -1 then
        gearIndex = -1
        M.timer.neutralSelectionDelayTimer = M.timerConstants.neutralSelectionDelay
        automaticHandling.mode = "R"
        applyGearboxMode()
      end
    end

    if electrics.values.ignitionLevel ~= 2 and automaticHandling.mode ~= "P" then
      gearIndex = 0
      M.timer.neutralSelectionDelayTimer = M.timerConstants.neutralSelectionDelay
      automaticHandling.mode = "P"
      applyGearboxMode()
    end
  end

  --Arcade mode gets a "rev limiter" in case the engine does not have one
  if engine.outputAV1 > engine.maxAV and not engine.hasRevLimiter then
    M.throttle = 0
  end

  M.currentGearIndex = (automaticHandling.mode == "N" or automaticHandling.mode == "P") and 0 or gearIndex
  updateExposedData()
end

local function updateInGear(dt)
  M.throttle = M.inputValues.throttle
  M.brake = M.inputValues.brake
  M.isArcadeSwitched = false

  local gearIndex = automaticHandling.cvtGearIndexLookup[automaticHandling.mode]
  local desiredGearRatio, smoothDesiredGearRatio

  local wheelspeed = electrics.values.wheelspeed
  local engineTargetRPM = cvtHandling.targetRpmMap:get(wheelspeed, M.smoothedValues.drivingAggression)
  local engineTargetAV = engineTargetRPM * constants.rpmToAV
  local targetGearRatio = automaticHandling.mode == "R" and gearbox.maxGearRatio or engineTargetAV / max(1, gearbox.outputAV1)

  desiredGearRatio = clamp(targetGearRatio, gearbox.minGearRatio, gearbox.maxGearRatio)
  desiredGearRatio = findFixedGearRatio(desiredGearRatio, dt)

  smoothDesiredGearRatio = smoother.cvtGearRatioSmoother:get(desiredGearRatio, dt)
  gearbox:setGearRatio(smoothDesiredGearRatio)

  -- streams.drawGraph("engineTargetRPM x1000", engineTargetRPM / 1000)
  -- streams.drawGraph("desiredGearRatio", desiredGearRatio)
  -- streams.drawGraph("smoothDesiredGearRatio", smoothDesiredGearRatio)

  if torqueConverterHandling.hasLockup then
    local avDiff = max(0, engineTargetAV - engine.outputAV1)
    local maxLockupBySpeed = clamp((wheelspeed - torqueConverterHandling.lockupMinWheelspeed) / torqueConverterHandling.lockupMinWheelspeed, 0, 1) -- ramp up from minWheelspeed to twice minWheelspeed
    local maxLockupByThrottle = min(1, 0.5 + M.throttle / 0.05)
    local maxAllowedLockup = min(maxLockupBySpeed, maxLockupByThrottle)

    if avDiff > torqueConverterHandling.lockupMaxAvDiff or engine.outputAV1 < engine.idleAV then
      torqueConverterHandling.shouldLockUp = false
    elseif avDiff > torqueConverterHandling.lockupMinAvDiff then
      torqueConverterHandling.shouldLockUp = true
    end

    local targetLockup = min(maxAllowedLockup, torqueConverterHandling.shouldLockUp and 1 or 0)

    electrics.values.lockupClutchRatio = smoother.lockupSmoother:get(targetLockup, dt)
  else
    electrics.values.lockupClutchRatio = 0
    smoother.lockupSmoother:set(0)
  end

  M.currentGearIndex = (automaticHandling.mode == "N" or automaticHandling.mode == "P") and 0 or gearIndex
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
  torqueConverter = powertrain.getDevice("torqueConverter")

  M.engineTorqueData = engine:getTorqueData()
  M.torqueToMinRPM = {}

  local engineTorqueCurve = M.engineTorqueData.curves[M.engineTorqueData.finalCurveName]

  for engTorque = 0, M.engineTorqueData.maxTorque do
    for rpm = engine.idleRPM, M.engineTorqueData.maxTorqueRPM, 10 do
      if engineTorqueCurve[rpm] and engineTorqueCurve[rpm] >= engTorque then
        M.torqueToMinRPM[engTorque] = rpm
        break
      end
    end
  end

  M.engineTorqueData = {}
  M.torqueToMinRPM = {}
  M.currentGearIndex = 0
  M.maxGearIndex = 1
  M.minGearIndex = -1
  M.throttle = 0
  M.brake = 0
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

  local cvtGearRatioSmoothingIn = jbeamData.cvtGearRatioSmoothingIn or 6
  local cvtGearRatioSmoothingOut = jbeamData.cvtGearRatioSmoothingOut or 4
  local lockupSmoothing = jbeamData.cvtLockupSmoothing or 5

  smoother.cvtGearRatioSmoother = newTemporalSmoothingNonLinear(cvtGearRatioSmoothingIn, cvtGearRatioSmoothingOut)
  smoother.lockupSmoother = newTemporalSmoothingNonLinear(lockupSmoothing)

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
      if mode ~= "M" then
        automaticHandling.modes[i + modeOffset] = mode
        automaticHandling.modeIndexLookup[mode] = i + modeOffset
        automaticHandling.existingModeLookup[mode] = true
      else
        for j = 1, gearbox.maxGearIndex, 1 do
          local manualMode = "M" .. tostring(j)
          local manualModeIndex = i + j - 1
          automaticHandling.modes[manualModeIndex] = manualMode
          automaticHandling.modeIndexLookup[manualMode] = manualModeIndex
          automaticHandling.existingModeLookup[manualMode] = true
          modeOffset = j - 1
        end
      end
    else
      print("unknown auto mode: " .. mode)
    end
  end

  if torqueConverter then
    torqueConverterHandling.lockupMinWheelspeed = jbeamData.lockupMinWheelspeed or 2
    torqueConverterHandling.lockupMinAvDiff = (jbeamData.lockupMinRpmDiff or 50) * constants.rpmToAV
    torqueConverterHandling.lockupMaxAvDiff = (jbeamData.lockupMaxRpmDiff or 400) * constants.rpmToAV
    torqueConverterHandling.hasLockup = torqueConverterHandling.lockupMinWheelspeed > 0
  end

  local defaultMode = jbeamData.defaultAutomaticMode or "N"
  automaticHandling.modeIndex = string.find(modes, defaultMode)
  automaticHandling.mode = automaticHandling.modes[automaticHandling.modeIndex]
  automaticHandling.maxGearIndex = gearbox.maxGearIndex
  automaticHandling.minGearIndex = gearbox.minGearIndex

  smoother.cvtGearRatioSmoother:set(gearbox.maxGearRatio)

  M.maxRPM = engine.maxRPM
  M.idleRPM = engine.idleRPM
  M.maxGearIndex = automaticHandling.maxGearIndex
  M.minGearIndex = abs(automaticHandling.minGearIndex)
  M.energyStorages = sharedFunctions.getEnergyStorages({engine})

  local interpolatedMap = rerequire("interpolatedMap")
  cvtHandling.targetRpmMap = interpolatedMap.new()
  cvtHandling.targetRpmMap.clampToDataRange = true
  cvtHandling.targetRpmMap:loadData(jbeamData.cvtTargetRPMMap)
  cvtHandling.sportGearShiftCooldown = jbeamData.cvtSportGearShiftCooldown or 1
  cvtHandling.remainingShiftCooldown = 0

  if type(jbeamData.cvtSportGearRatios) == "table" then
    cvtHandling.fixedGearRatios = {}
    cvtHandling.useFixedGearRatios = false
    cvtHandling.fixedGearRatioIndex = 1

    for i, v in ipairs(jbeamData.cvtSportGearRatios) do
      if type(v) ~= "number" then
        log("W", "cvtGearbox2", "Invalid value #" .. tostring(i) .. " for fixedGearRatios (value was not a number)")
        break
      end
      if #cvtHandling.fixedGearRatios > 0 and v >= cvtHandling.fixedGearRatios[#cvtHandling.fixedGearRatios] then
        log("W", "cvtGearbox2", "Invalid value #" .. tostring(i) .. " for fixedGearRatios (gear ratio must be smaller than the last)")
        break
      end

      table.insert(cvtHandling.fixedGearRatios, v)
    end
  else
    cvtHandling.useFixedGearRatios = false
  end

  applyGearboxMode()
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
    applyGearboxModeRestrictions()
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
