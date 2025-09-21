-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local min = math.min

local function createCompatibilityController()
  local hasEngine = v.data.enginetorque ~= nil
  if hasEngine then
    local mainController = {fileName = "vehicleController"}

    mainController.lowShiftDownRPM = v.data.engine.lowShiftDownRPM or v.data.engine.shiftDownRPM
    mainController.lowShiftUpRPM = v.data.engine.lowShiftUpRPM or v.data.engine.shiftUpRPM
    mainController.highShiftDownRPM = v.data.engine.highShiftDownRPM or v.data.engine.shiftDownRPM
    mainController.highShiftUpRPM = v.data.engine.highShiftUpRPM or v.data.engine.shiftUpRPM
    mainController.clutchLaunchTargetRPM = math.min(v.data.engine.idleRPM * 3, v.data.engine.maxRPM / 2)
    mainController.clutchLaunchStartRPM = math.min(v.data.engine.idleRPM * 2, v.data.engine.maxRPM / 2)

    if not v.data.controller then
      v.data.controller = {}
    end
    table.insert(v.data.controller, mainController)
  end
end

local function makeDiff(oldDiff, name, inputName, inputIndex, gearRatio, friction, torqueSplit)
  local diff = {
    name = name,
    inputName = inputName,
    inputIndex = inputIndex,
    type = "differential",
    gearRatio = gearRatio,
    friction = friction,
    diffTorqueSplit = torqueSplit
  }

  local oldDiffType = oldDiff.type
  local oldDiffState = oldDiff.state
  local oldCloseTorque = oldDiff.closedTorque
  if oldDiffType then
    if oldDiffType == "open" then
      diff.diffType = "open"
    elseif oldDiffType == "lsd" and oldDiffState == "closed" and oldCloseTorque < 1000 then
      diff.diffType = "lsd"
      diff.lsdPreload = oldDiff.closedTorque
      diff.lsdLockCoef = min(oldDiff.closedTorque / 2000, 0.5)
    elseif oldDiffType == "lsd" and (oldDiffState == "locked" or oldCloseTorque >= 1000) then
      diff.diffType = "locked"
      diff.lockSpring = oldDiff.closedTorque
      diff.lockDeform = oldDiff.closedTorque * 2
    else
      log("E", "powertrain.makeDiff", "Found unknown old differential type: '" .. oldDiffType .. "', defaulting to 'open'!")
      diff.diffType = "open"
    end
  end

  return diff
end

local function makeShaft(type, name, inputName, inputIndex, friction, breakBeam, connectedWheel)
  local shaft = {
    name = name,
    inputName = inputName,
    inputIndex = inputIndex,
    type = type,
    gearRatio = 1,
    cumulativeGearRatio = 1,
    friction = friction,
    breakTriggerBeam = breakBeam,
    connectedWheel = connectedWheel,
    isPhysicallyDisconnected = true
  }

  return shaft
end

local function createCompatibilityDifferentials()
  if not v.data.differentials then
    return nil
  end

  local compatDiffs
  local oldData = deepcopy(v.data.differentials)

  local wheels = powertrain.wheels

  local numberOfOldDiffs = tableSize(oldData)
  if numberOfOldDiffs == 1 then
    local oldDiff = oldData[0]
    local frictionPart = (v.data.engine.axleFriction or 0) * 4 / 5
    local diffRatio = v.data.engine.differential or 1

    local axleBeamsWheel1 = wheels[oldDiff.wheelName1] and (wheels[oldDiff.wheelName1].axleBeams or {}) or {}
    local axleBeamsWheel2 = wheels[oldDiff.wheelName2] and (wheels[oldDiff.wheelName2].axleBeams or {}) or {}

    local driveshaft = makeShaft("shaft", "driveshaft", "gearbox", 1, frictionPart, "driveshaft")
    driveshaft.electricsName = "driveshaft"
    local diff = makeDiff(oldDiff, "diff", "driveshaft", 1, diffRatio, frictionPart * 2, 0.5)
    local wheelShaft1 = makeShaft("shaft", "axle1", "diff", 1, frictionPart, axleBeamsWheel1[1], oldDiff.wheelName1)
    local wheelShaft2 = makeShaft("shaft", "axle2", "diff", 2, frictionPart, axleBeamsWheel2[1], oldDiff.wheelName2)

    compatDiffs = {driveshaft, diff, wheelShaft1, wheelShaft2}
  elseif numberOfOldDiffs == 2 then
    local oldDiff1 = oldData[0]
    local oldDiff2 = oldData[1]
    local frictionPart = (v.data.engine.axleFriction or 0) * 4 / 18
    local diffRatio = v.data.engine.differential
    local centerDiffRatio = oldDiff1.engineTorqueCoef / (oldDiff1.engineTorqueCoef + oldDiff2.engineTorqueCoef)

    local centerdiff = makeDiff({}, "centerdiff", "gearbox", 1, 1, frictionPart * 2, centerDiffRatio)
    centerdiff.diffType = "lsd"

    local axleBeamsWheel11 = wheels[oldDiff1.wheelName1] and (wheels[oldDiff1.wheelName1].axleBeams or {}) or {}
    local axleBeamsWheel12 = wheels[oldDiff1.wheelName2] and (wheels[oldDiff1.wheelName2].axleBeams or {}) or {}
    local axleBeamsWheel21 = wheels[oldDiff2.wheelName1] and (wheels[oldDiff2.wheelName1].axleBeams or {}) or {}
    local axleBeamsWheel22 = wheels[oldDiff2.wheelName2] and (wheels[oldDiff2.wheelName2].axleBeams or {}) or {}

    local driveshaft1 = makeShaft("shaft", "driveshaft1", "centerdiff", 1, frictionPart, "driveshaft")
    driveshaft1.electricsName = "driveshaft"
    local driveshaft2 = makeShaft("shaft", "driveshaft2", "centerdiff", 2, frictionPart, nil)

    local diff1 = makeDiff(oldDiff1, "diff1", "driveshaft1", 1, diffRatio, frictionPart * 3, 0.5)
    local diff2 = makeDiff(oldDiff2, "diff2", "driveshaft2", 1, diffRatio, frictionPart * 3, 0.5)

    local wheelShaft11 = makeShaft("shaft", "wheelaxle" .. tostring(oldDiff1.wheelName1), "diff1", 1, frictionPart * 2, axleBeamsWheel11[1], oldDiff1.wheelName1)
    local wheelShaft12 = makeShaft("shaft", "wheelaxle" .. tostring(oldDiff1.wheelName2), "diff1", 2, frictionPart * 2, axleBeamsWheel12[1], oldDiff1.wheelName2)
    local wheelShaft21 = makeShaft("shaft", "wheelaxle" .. tostring(oldDiff2.wheelName1), "diff2", 1, frictionPart * 2, axleBeamsWheel21[1], oldDiff2.wheelName1)
    local wheelShaft22 = makeShaft("shaft", "wheelaxle" .. tostring(oldDiff2.wheelName2), "diff2", 2, frictionPart * 2, axleBeamsWheel22[1], oldDiff2.wheelName2)

    compatDiffs = {centerdiff, driveshaft1, driveshaft2, diff1, diff2, wheelShaft11, wheelShaft12, wheelShaft21, wheelShaft22}
  elseif numberOfOldDiffs > 2 then
    local diffRatio = v.data.engine.differential
    local frictionPart = 5
    local driveshaft = makeShaft("shaft", "driveshaft", "gearbox", 1, frictionPart, nil)
    local mvName = "multiShaft"
    local multiShaft = {
      name = mvName,
      inputName = "driveshaft",
      inputIndex = 1,
      type = "multiShaft",
      gearRatio = diffRatio,
      cumulativeGearRatio = 1,
      friction = 0,
      breakTriggerBeam = nil,
      connectedWheel = nil,
      isPhysicallyDisconnected = true
    }

    local uniqueWheels = {}
    for i = 0, tableSize(oldData) - 1, 1 do
      local oldDiff = oldData[i]
      uniqueWheels[oldDiff.wheelName1] = true
      uniqueWheels[oldDiff.wheelName2] = true
    end

    compatDiffs = {driveshaft, multiShaft}
    local shaftCounter = 1

    for k, _ in pairs(uniqueWheels) do
      local axleBeamsWheel = wheels[k] and (wheels[k].axleBeams or {}) or {}
      local wheelShaft = makeShaft("shaft", "axle" .. shaftCounter, mvName, shaftCounter, frictionPart, axleBeamsWheel[1], k)
      shaftCounter = shaftCounter + 1
      table.insert(compatDiffs, wheelShaft)
    end

    multiShaft.numberOfOutputPorts = shaftCounter - 1
  else
    log("E", "powertrain.init", "Found unsupported old differential data, please upgrade to the new system manually!")
    return nil
  end

  -- please do not commit this, only enable in local builds
  --dump(compatDiffs)
  return compatDiffs
end

local function createCompatibilityEngine()
  --set some legacy stuff so it's not nil (not actually used though)
  v.data.engine = v.data.engine or {}
  v.data.engine.fwdGearCount = 6
  v.data.engine.rwdGearCount = 1
  electrics.values.gear_A = 0
  electrics.values.gear_M = 0

  local hasEngine = v.data.enginetorque ~= nil

  if hasEngine then
    local engine = {type = "combustionEngine", name = "mainEngine", inputName = "", inputIndex = 0}
    local oldEngine = v.data.engine
    engine.idleRPM = oldEngine.idleRPM or 800
    engine.maxRPM = oldEngine.maxRPM or 6000
    engine.inertia = oldEngine.inertia or 0.2
    engine.burnEfficiency = oldEngine.burnEfficiency or 0.3
    engine.friction = oldEngine.friction or oldEngine.engineFriction or 20
    engine.dynamicFriction = (oldEngine.brakingCoefRPS or 0.2) / (2 * math.pi)
    engine.particulates = oldEngine.particulates or 0
    engine.torqueReactionNodes = oldEngine.torqueReactionNodes

    engine.lowShiftDownRPM = oldEngine.lowShiftDownRPM
    engine.lowShiftUpRPM = oldEngine.lowShiftUpRPM
    engine.highShiftDownRPM = oldEngine.highShiftDownRPM
    engine.highShiftUpRPM = oldEngine.highShiftUpRPM

    --engine thermals
    engine.thermalsEnabled = oldEngine.thermalsEnabled or false
    engine.engineBlockMaterial = oldEngine.engineBlockMaterial

    engine.coolantVolume = oldEngine.coolantVolume
    engine.oilVolume = oldEngine.oilVolume
    engine.engineBlockAirCoolingEfficiency = oldEngine.engineBlockAirCoolingEfficiency
    engine.blockFanMaxAirSpeed = oldEngine.blockFanMaxAirSpeed
    engine.radiatorFanType = oldEngine.radiatorFanType
    engine.radiatorArea = oldEngine.radiatorArea
    engine.radiatorFanMaxAirSpeed = oldEngine.radiatorFanMaxAirSpeed
    engine.radiatorEffectiveness = oldEngine.radiatorEffectiveness
    engine.radiatorFanTemperature = oldEngine.radiatorFanTemperature
    engine.thermostatTemperature = oldEngine.thermostatTemperature
    engine.oilThermostatTemperature = oldEngine.oilThermostatTemperature
    engine.oilRadiatorArea = oldEngine.oilRadiatorArea
    engine.oilRadiatorEffectiveness = oldEngine.oilRadiatorEffectiveness
    engine.radiatorDeformThreshold = oldEngine.radiatorDeformThreshold

    engine.engineBlock = oldEngine.engineBlock
    engine.radiator = oldEngine.radiator
    engine.coolantVolume = oldEngine.coolantVolume
    engine.coolantVolume = oldEngine.coolantVolume

    engine.cylinderWallTemperatureDamageThreshold = oldEngine.cylinderWallTemperatureDamageThreshold
    engine.headGasketDamageThreshold = oldEngine.headGasketDamageThreshold
    engine.pistonRingDamageThreshold = oldEngine.pistonRingDamageThreshold
    engine.connectingRodDamageThreshold = oldEngine.connectingRodDamageThreshold
    engine.engineBlockTemperatureDamageThreshold = oldEngine.engineBlockTemperatureDamageThreshold

    engine.headGasketBlownOverride = oldEngine.headGasketBlownOverride
    engine.pistonRingsDamagedOverride = oldEngine.pistonRingsDamagedOverride
    engine.connectingRodBearingsDamagedOverride = oldEngine.connectingRodBearingsDamagedOverride

    engine.breakTriggerBeam = oldEngine.onBeamBreakDisableEngine

    if v.data.enginetorque then
      engine.torque = {{"rpm", "torque"}}
      for _, v in pairs(v.data.enginetorque) do
        table.insert(engine.torque, {v.rpm, v.torque})
      end
    end

    if v.data.turbocharger then
      engine.turbocharger = "turbocharger"
    end

    if v.data.supercharger then
      engine.supercharger = "supercharger"
    end

    local clutchLikeDevice = nil
    local gearboxInput = "clutch"
    local oldGearboxType = oldEngine.transmissionType
    local newGearboxType = ""
    if oldGearboxType == "manual" and not v.data.engine.shiftableAuto then
      if oldEngine.dct then
        newGearboxType = "dctGearbox"
        gearboxInput = "mainEngine"
      else
        newGearboxType = "manualGearbox"
        clutchLikeDevice = {type = "frictionClutch", name = "clutch", inputName = "mainEngine", inputIndex = 1, thermalsEnabled = false}
      end
    elseif oldGearboxType == "automatic" or v.data.engine.shiftableAuto then
      if oldEngine.cvt then
        newGearboxType = "cvtGearbox"
      else
        newGearboxType = "automaticGearbox"
      end
      clutchLikeDevice = {type = "viscousClutch", name = "clutch", inputName = "mainEngine", inputIndex = 1}
      clutchLikeDevice.viscousCoef = oldEngine.viscousCoupling or 10
      clutchLikeDevice.cutInRPM = engine.idleRPM * 0.9
      clutchLikeDevice.stallRPM = engine.idleRPM * 3
    end
    local gearbox = {type = newGearboxType, name = "gearbox", inputName = gearboxInput, inputIndex = 1}
    gearbox.gearRatios = {}
    if oldEngine.gears then
      for _, v in ipairs(oldEngine.gears) do
        if type(v) == "number" then
          table.insert(gearbox.gearRatios, v)
        end
      end
    end
    gearbox.minGearRatio = oldEngine.minRatio
    gearbox.maxGearRatio = oldEngine.maxRatio

    if v.data.engine.fuelCapacity then
      v.data.energyStorage = v.data.energyStorage or {}
      dump("Used backwardsCompatibility")
      local fuelTank = {name = "fuelTank", type = "fuelTank", fuelCapacity = v.data.engine.fuelCapacity, fuel = v.data.engine.fuel}
      table.insert(v.data.energyStorage, fuelTank)
      engine.energyStorage = "fuelTank"
    end

    local compatEngineData = {engine, gearbox}
    if clutchLikeDevice then
      table.insert(compatEngineData, clutchLikeDevice)
    end
    return compatEngineData
  end
  return nil
end

local function createCompatibilityPowertrain()
  log("D", "backwardsCompatibility.createCompatibilityPowertrain", "Running vehicle in powertrain compatibility mode")
  v.data.powertrain = v.data.powertrain or {}
  local engineData = createCompatibilityEngine()
  if engineData then
    for _, e in pairs(engineData) do
      table.insert(v.data.powertrain, e)
    end
  end
  local diffData = createCompatibilityDifferentials()
  if diffData then
    for _, e in pairs(diffData) do
      table.insert(v.data.powertrain, e)
    end
  end

  --dump(v.data.powertrain)

  createCompatibilityController()

  return true
end

local function checkOldDrivetrain()
  if not v.data.powertrain and (v.data.differentials or v.data.engine) then
    log("D", "backwardsCompatibility.checkOldDrivetrain", "Found old drivetrain data, creating compatibility powertrain...")
    local result = createCompatibilityPowertrain()
    if not result then
      log("E", "backwardsCompatibility.checkOldDrivetrain", "Old drivetrain data can't be used to create compatibility powertrain, aborting...")
      return
    end
  end
end

local function checkOldESC()
  --ESC backwards compatiblity - hardcoded -> controller
  if v.data.escConfig then
    local hasController = false
    --check if we already have an esc controller
    for _, v in pairs(v.data.controller) do
      if v.filename == "esc" then
        hasController = true
      end
    end
    --if we don't have a controller
    if not hasController then
      --add one (no special config needed)
      table.insert(v.data.controller, {fileName = "esc"})
      --and reference the old config data (unchanged)
      v.data.esc = v.data.escConfig
      log("I", "backwardsCompatibility.init", "Converting old ESC data to controller layout")
    end
  end
end

local function checkTrailerIgnitionStates()
  --heuristic to determine if something is a "trailer" for setting the available ignition levels
  local hasTorqueSource = false
  local hasVehicleController = false

  for _, deviceData in pairs(v.data.powertrain or {}) do
    local type = deviceData.type
    if type == "combustionEngine" or type == "electricMotor" or type == "centrifugalClutch" or type == "frictionClutch" or type == "torqueConverter" then
      hasTorqueSource = true
      break
    end
  end

  for _, controllerData in pairs(v.data.controller or {}) do
    if controllerData.fileName == "vehicleController" then
      hasVehicleController = true
      break
    end
  end

  local ignitionProbablyNotNeeded = not hasTorqueSource and not hasVehicleController
  if ignitionProbablyNotNeeded and not (v.data.electrics and v.data.electrics.allowedIgnitionLevels) then
    --log("I", "backwardsCompatibility.init", "Assuming trailer-like vehicle, locking available ignition levels to [0] and disabling ignition effects, provide your own 'electrics.allowedIgnitionLevels' and 'electrics.ignitionLevelOverrideType' in jbeam to override this.")
    v.data.electrics = v.data.electrics or {}
    v.data.electrics.allowedIgnitionLevels = {0}
    v.data.electrics.ignitionLevelOverrideType = "none"
  end
end

local function init()
  checkOldESC() --convert hardcoded ESC to controller
  checkOldDrivetrain() --convert old drivetrain to powertrain data

  --must be after drivetrain -> powertrain conversion since it relies on powertrain data for heuristics
  checkTrailerIgnitionStates() --try to detect trailers to set the correct available ignition states
end

M.init = init

return M
