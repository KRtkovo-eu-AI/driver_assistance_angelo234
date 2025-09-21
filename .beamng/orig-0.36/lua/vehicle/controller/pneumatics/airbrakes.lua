-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local floor = math.floor
local min = math.min
local max = math.max
local sqrt = math.sqrt

local airTank = nil
local brakedWheelsLookup
local serviceBrakeElectricsName = nil
local parkingBrakeElectricsName = nil
local serviceBrakePressureElectricsName = nil
local parkingBrakePressureElectricsName = nil
local brakeTorqueCurve = nil
local springBrakeTorqueCurve = nil
local minBrakePressure = 0
local maxBrakePressure = 0
local minSpringBrakePressure = 0
local maxSpringBrakePressure = 0
local springBrakeAutoApplyThresholdPressure = 0
local springBrakeAutoReleaseThresholdPressure = 0
local didAutoApplyParkingBrake = false

-- Sound state
local soundNode = 0
local soundEvent = nil
local soundSampleWindow = 0.25
local soundCoolDown = 0
local serviceSoundTimer = 0
local serviceSoundCoolDown = 0
local serviceSoundStart = 0
local lastServiceBrake = 0
local doServiceSound = false
local parkingSoundTimer = 0
local parkingSoundCoolDown = 0
local parkingSoundStart = 0
local lastParkingBrake = 0
local doParkingSound = false

-- Service brake actuator
local actuatorPressure = 0
local actuatorEnergy = 0
local brakeTorqueCoef = 0

-- Parking/"Spring" brake actuator
local springActuatorPressure = 0
local springActuatorEnergy = 0
local springTorqueCoef = 0

-- Brake pipe characteristics
local brakePipeRadius = 0 -- m
local brakePipeCrossSectionArea = 0 -- m^2

-- Brake actuator characteristics
local totalActuatorCapacity = 0
local invTotalActuatorCapacity = 0

-- Flow characteristics
local dischargeCoefficient = 0.97 -- closer to 1.0 for rounded orifices, decreases with sharpness of edges
local quickReleaseFlowRate = 0 -- m^3/s

local function updateWheelBrakeABS(wd, brake, invAirspeed, airspeed, airspeedCutOff, dt)
  local absCoef = wheels.updateABSCoef(wd, brakeTorqueCoef, invAirspeed, airspeed, airspeedCutOff, dt)
  local brakeCoef = max(brakeTorqueCoef * absCoef, springTorqueCoef)
  local brakeTorque = wd.brakeTorque * brakeCoef
  --log("D", "airbrakes.updateWheelBrake", string.format("returning %.2fNm/%.2fNm for wheel %q with service: %.2f and park: %.2f", brakeTorque, wd.brakeTorque, wd.name, brakeTorqueCoef, springTorqueCoef))
  return brakeTorque
end

local function updateWheelBrakeNoABS(wd, brake, invAirspeed, airspeed, airspeedCutOff, dt)
  local brakeCoef = max(brakeTorqueCoef, springTorqueCoef)
  local brakeTorque = wd.brakeTorque * brakeCoef
  --log("D", "airbrakes.updateWheelBrake", string.format("returning %.2fNm/%.2fNm for wheel %q with service: %.2f and park: %.2f", brakeTorque, wd.brakeTorque, wd.name, brakeTorqueCoef, springTorqueCoef))
  return brakeTorque
end

local function updateServiceBrakes(dt)
  local parkingBrakeApplied = (electrics.values[parkingBrakeElectricsName] or 0) >= 0.5
  local brakeInput = parkingBrakeApplied and 0 or min(max(electrics.values[serviceBrakeElectricsName] or 0, 0), 1)
  local tankPressure = airTank.currentPressure
  local envPressure = powertrain.currentEnvPressure
  local relativeActuatorPressure = max(0, actuatorPressure - envPressure)
  local regulatorPressure = min(tankPressure, brakeInput * maxBrakePressure + envPressure) -- max pressure let into brake lines from reservoir by the brake pedal valve
  local airDensity = airTank.remainingMass * airTank.invCapacity
  local flowRate = 0

  if actuatorPressure < regulatorPressure then
    local pressureDiff = max(0, tankPressure - actuatorPressure)

    flowRate = dischargeCoefficient * brakePipeCrossSectionArea * sqrt(2 * pressureDiff / airDensity)
  elseif actuatorPressure > regulatorPressure + 1000 then -- small window of buffer
    -- if the absolute actuator pressure is more than twice the env pressure,
    -- flow rate is limited by the speed of sound, so we clamp the maximum to the constant quickReleaseFlowRate.
    -- since we can't exhaust the actuators below environmental pressure anyways, we can simplify this by comparing
    -- the relative pressure against environmental instead of the absolute pressure against twice environemntal
    local flowRateCoef = min(1, relativeActuatorPressure * powertrain.invCurrentEnvPressure)

    flowRate = relativeActuatorPressure <= 0 and 0 or -quickReleaseFlowRate * sqrt(flowRateCoef)
  end

  local airVolumeMoved = flowRate * dt
  local airEnergyMoved = (flowRate > 0 and tankPressure or actuatorPressure) * airVolumeMoved

  --log("D", "airbrakes.updateFixedStep", "pressureDiff: " .. tostring(pressureDiff))
  --log("D", "airbrakes.updateFixedStep", "flowRate: " .. tostring(flowRate))
  --log("D", "airbrakes.updateFixedStep", "airVolumeMoved: " .. tostring(airVolumeMoved))
  --log("D", "airbrakes.updateFixedStep", "airEnergyMoved: " .. tostring(airEnergyMoved))

  -- Using PV = nRT (where both sides of the equation represent energy), calculate
  -- energy delta using the flow rate and actuator volume. We can then determine
  -- actuator pressure and, in turn, brake force.
  -- (to simplify things, we assume the actuator has reached full extension already and all pressure acts towards brake force instead of towards extending the actuator)
  local prevActuatorEnergy = actuatorEnergy

  -- Add the air that flowed in this update and update actuator energy/pressure
  actuatorEnergy = max(0, actuatorEnergy + airEnergyMoved)
  actuatorPressure = actuatorEnergy * invTotalActuatorCapacity -- PV = e, therefore P = e / V
  relativeActuatorPressure = actuatorPressure - envPressure

  local energyTransferred = actuatorEnergy - prevActuatorEnergy

  --log("D", "airbrakes.updateFixedStep", "energyTransferred: " .. tostring(energyTransferred))

  -- remove the same amount of energy from the airTank that went into the cylinder
  -- (negative energy transfer means pressure is vented to the atmosphere)
  if energyTransferred > 0 then
    airTank.storedEnergy = max(0, airTank.storedEnergy - energyTransferred)
  end

  -- need to convert pressure to kPa and floor it before indexing curve
  local lookupPressure = floor(min(max(relativeActuatorPressure, minBrakePressure), maxBrakePressure) * 0.001)

  brakeTorqueCoef = brakeTorqueCurve[lookupPressure] or 0
  electrics.values[serviceBrakePressureElectricsName] = relativeActuatorPressure

  --streams.drawGraph("regulatorPressure", { unit = "kPa", value = regulatorPressure / 1000, min = 0, max = maxBrakePressure / 1000 })
  --streams.drawGraph("tankPressure", { unit = "kPa", value = tankPressure / 1000, min = 0, max = maxBrakePressure / 1000 })
  --streams.drawGraph("actuatorPressure", { unit = "kPa", value = relativeActuatorPressure / 1000, min = 0, max = maxBrakePressure / 1000 })
end

local function updateParkingBrake(dt)
  local parkingBrakeReleaseCoef = 1 - min(max(electrics.values[parkingBrakeElectricsName] or 0, 0), 1) -- when parking brake input is "0" (released), we want pressure to enter the actuator and release the spring brakes
  local tankPressure = airTank.currentPressure
  local envPressure = powertrain.currentEnvPressure
  --if our pressure tank has a low supply pressure, we need to act as if our actual pressure was 0 (this is just how these behave IRL)
  -- supply pressure is either the actual supply (on a trailer) or if there is a compressor, supply and current are set the same
  if airTank.supplyPressure < (springBrakeAutoApplyThresholdPressure + envPressure) or airTank.isDummy then
    parkingBrakeReleaseCoef = 0
    didAutoApplyParkingBrake = true
    --enable parking brake or rather the smart one if it exists
    if controller.mainController.smartParkingBrake then
      controller.mainController.smartParkingBrake(1, FILTER_DIRECT, true)
    else
      input.event("parkingbrake", 1, FILTER_KBD)
    end
  elseif didAutoApplyParkingBrake and airTank.supplyPressure > (springBrakeAutoReleaseThresholdPressure + envPressure) then
    --once we are above the auto relase pressure, stop overwriting the parkingbrake and hand control over the smart one back to vehicle controller
    didAutoApplyParkingBrake = false
    if controller.mainController.smartParkingBrake then
      --"disable" parking brake, it stays on until you try to accelerate now
      controller.mainController.smartParkingBrake(0, FILTER_DIRECT, false)
    end
  end
  local relativeActuatorPressure = max(0, springActuatorPressure - envPressure)
  local targetPressure = min(tankPressure, parkingBrakeReleaseCoef * maxSpringBrakePressure + envPressure) -- max pressure let into brake lines from reservoir by the parking release valve
  local airDensity = airTank.remainingMass * airTank.invCapacity
  local flowRate = 0

  if springActuatorPressure < targetPressure then
    local pressureDiff = max(0, tankPressure - springActuatorPressure)

    flowRate = dischargeCoefficient * brakePipeCrossSectionArea * sqrt(2 * pressureDiff / airDensity)
  elseif springActuatorPressure > targetPressure + 1000 then -- small window of buffer
    -- if the absolute actuator pressure is more than twice the env pressure,
    -- flow rate is limited by the speed of sound, so we clamp the maximum to the constant quickReleaseFlowRate.
    -- since we can't exhaust the actuators below environmental pressure anyways, we can simplify this by comparing
    -- the relative pressure against environmental instead of the absolute pressure against twice environemntal
    local flowRateCoef = min(1, relativeActuatorPressure * powertrain.invCurrentEnvPressure)

    flowRate = relativeActuatorPressure <= 0 and 0 or -quickReleaseFlowRate * sqrt(flowRateCoef)
  end

  -- Calculate energy transfer like in updateServiceBrakes
  local airVolumeMoved = flowRate * dt
  local airEnergyMoved = (flowRate > 0 and tankPressure or springActuatorPressure) * airVolumeMoved
  local prevActuatorEnergy = springActuatorEnergy

  -- Add the air that flowed in this update and update actuator energy/pressure
  springActuatorEnergy = max(0, springActuatorEnergy + airEnergyMoved)
  springActuatorPressure = springActuatorEnergy * invTotalActuatorCapacity -- PV = e, therefore P = e / V
  relativeActuatorPressure = springActuatorPressure - envPressure

  local energyTransferred = springActuatorEnergy - prevActuatorEnergy

  --log("D", "airbrakes.updateFixedStep", "energyTransferred: " .. tostring(energyTransferred))

  -- remove the same amount of energy from the airTank that went into the cylinder
  -- (negative energy transfer means pressure is vented to the atmosphere)
  if energyTransferred > 0 then
    airTank.storedEnergy = max(0, airTank.storedEnergy - energyTransferred)
  end

  -- need to convert pressure to kPa and floor it before indexing curve
  local lookupPressure = floor(min(max(relativeActuatorPressure, minSpringBrakePressure), maxSpringBrakePressure) * 0.001)

  -- At zero pressure, the spring brakes are applied, resulting in full parking brake torque
  springTorqueCoef = springBrakeTorqueCurve[lookupPressure] or 0
  electrics.values[parkingBrakePressureElectricsName] = relativeActuatorPressure

  --streams.drawGraph("springActuatorPressure", { value = relativeActuatorPressure / 1000, unit = "kPa", min = 0, max = maxSpringBrakePressure / 1000 })
end

local function updateSounds(dt)
  if airTank.isDummy then
    --don't play sounds if we have a dummy airtank,  better approach would be to actually trigger the sounds based on changes in pressure
    return
  end
  -- service brake --
  serviceSoundCoolDown = max(0, serviceSoundCoolDown - dt)
  if lastServiceBrake - brakeTorqueCoef > 0.01 and not doServiceSound and serviceSoundCoolDown <= 0 then
    doServiceSound = true
    serviceSoundTimer = 0
    serviceSoundStart = lastServiceBrake
  end

  if doServiceSound then
    serviceSoundTimer = serviceSoundTimer + dt
    if serviceSoundTimer >= soundSampleWindow then
      local dBrake = (brakeTorqueCoef - serviceSoundStart) / serviceSoundTimer
      doServiceSound = false
      if dBrake < -0.2 then
        local intensity = min(0.99, -dBrake * soundSampleWindow * 2)
        obj:playSFXOnce(soundEvent, soundNode, intensity, 1)
        serviceSoundCoolDown = soundCoolDown
      end
    end
  end

  -- parking/spring brake --
  local springBrake = 1 - springTorqueCoef

  parkingSoundCoolDown = max(0, parkingSoundCoolDown - dt)
  if lastParkingBrake - springBrake > 0.01 and not doParkingSound and parkingSoundCoolDown <= 0 then
    doParkingSound = true
    parkingSoundTimer = 0
    parkingSoundStart = lastParkingBrake
  end

  if doParkingSound then
    parkingSoundTimer = parkingSoundTimer + dt
    if parkingSoundTimer >= soundSampleWindow then
      local dBrake = (springBrake - parkingSoundStart) / parkingSoundTimer
      doParkingSound = false
      if dBrake < -0.2 then
        local intensity = min(0.99, -dBrake * soundSampleWindow)
        obj:playSFXOnce(soundEvent, soundNode, intensity, 1)
        parkingSoundCoolDown = soundCoolDown
      end
    end
  end

  lastServiceBrake = brakeTorqueCoef
  lastParkingBrake = springBrake
end

local function updateFixedStep(dt)
  updateServiceBrakes(dt)
  updateParkingBrake(dt)
end

local function updateGFX(dt)
  updateSounds(dt)
end

local function setBrakedWheelsUpdate()
  for _, wd in pairs(wheels.wheels) do
    if brakedWheelsLookup[wd.name] then
      wheels.setWheelBrakeUpdate(wd.name, updateWheelBrakeNoABS, updateWheelBrakeABS)
    end
  end
end

local function reset()
  actuatorPressure = powertrain.currentEnvPressure
  actuatorEnergy = actuatorPressure * totalActuatorCapacity

  springActuatorPressure = powertrain.currentEnvPressure
  springActuatorEnergy = springActuatorPressure * totalActuatorCapacity

  setBrakedWheelsUpdate()

  lastServiceBrake = 0
  serviceSoundCoolDown = soundCoolDown
  doServiceSound = false
  serviceSoundStart = 0

  lastParkingBrake = 0
  parkingSoundCoolDown = soundCoolDown
  doParkingSound = false
  parkingSoundStart = 0

  didAutoApplyParkingBrake = false

  electrics.values[parkingBrakePressureElectricsName] = 0
  electrics.values[serviceBrakePressureElectricsName] = 0
end

local function init(jbeamData)
  local airTankName = jbeamData.airTankName or "mainAirTank"

  airTank = energyStorage.getStorage(airTankName)

  if not airTank then
    log("D", "airbrakes.init", M.name .. ": assigned air tank not found: " .. airTankName)
    --create a dummy airtank so that stuff like parking brake etc keeps working
    airTank = {isDummy = true, currentPressure = 0, supplyPressure = 0, remainingMass = 0, invCapacity = 0}
  end

  if not jbeamData.brakeTorque then
    log("E", "airbrakes.init", "Can't find brake torque table! Air brakes will not function.")
  end
  if not jbeamData.springBrakeTorque then
    log("E", "airbrakes.init", "Can't find parking brake torque table! Air brakes will not function.")
  end

  local torqueTable = tableFromHeaderTable(jbeamData.brakeTorque)
  local rawPoints = {}
  minBrakePressure = math.huge
  maxBrakePressure = 0

  for _, v in pairs(torqueTable) do
    minBrakePressure = min(minBrakePressure, v.pressure)
    maxBrakePressure = max(maxBrakePressure, v.pressure)
    -- pressure is stored in kPa in the curve (but defined as Pa in jbeam)
    table.insert(rawPoints, {floor(v.pressure / 1000), v.torqueCoef})
  end

  brakeTorqueCurve = createCurve(rawPoints)

  -- reuse vars for spring brake torque table
  torqueTable = tableFromHeaderTable(jbeamData.springBrakeTorque)
  rawPoints = {}
  minSpringBrakePressure = math.huge
  maxSpringBrakePressure = 0

  for _, v in pairs(torqueTable) do
    minSpringBrakePressure = min(minSpringBrakePressure, v.pressure)
    maxSpringBrakePressure = max(maxSpringBrakePressure, v.pressure)
    -- pressure is stored in kPa in the curve (but defined as Pa in jbeam)
    table.insert(rawPoints, {floor(v.pressure / 1000), v.torqueCoef})
  end

  springBrakeTorqueCurve = createCurve(rawPoints)
  springBrakeAutoApplyThresholdPressure = jbeamData.springBrakeAutoApplyThresholdPressure or 275790 --40 psi by default
  springBrakeAutoReleaseThresholdPressure = jbeamData.springBrakeAutoReleaseThresholdPressure or 413685 --60 psi by default

  brakePipeRadius = jbeamData.brakePipeRadius or 0.0075 -- m
  brakePipeCrossSectionArea = math.pi * brakePipeRadius ^ 2

  local brakeActuatorDiameter = jbeamData.brakeActuatorDiameter or 0.16 -- m
  local brakeActuatorStroke = jbeamData.brakeActuatorStroke or 0.0635 -- m
  local brakeActuatorVolume = brakeActuatorStroke * math.pi * (brakeActuatorDiameter / 2) ^ 2
  local numBrakeActuators = jbeamData.numBrakeActuators or 4

  quickReleaseFlowRate = jbeamData.quickReleaseFlowRate or 0.01 -- m^3/s

  -- Calculate total actuator capacity (we assume all actuators are the same size), and store the inverse
  totalActuatorCapacity = brakeActuatorVolume * numBrakeActuators
  invTotalActuatorCapacity = 1 / totalActuatorCapacity

  actuatorPressure = powertrain.currentEnvPressure
  actuatorEnergy = actuatorPressure * totalActuatorCapacity

  springActuatorPressure = powertrain.currentEnvPressure
  springActuatorEnergy = springActuatorPressure * totalActuatorCapacity

  local brakedWheels = jbeamData.brakedWheels or {}
  brakedWheelsLookup = {}
  for _, brakedWheelName in pairs(brakedWheels) do
    brakedWheelsLookup[brakedWheelName] = true
  end

  setBrakedWheelsUpdate()

  serviceBrakeElectricsName = jbeamData.serviceBrakeElectricsName or "brake"
  parkingBrakeElectricsName = jbeamData.parkingBrakeElectricsName or "parkingbrake"
  serviceBrakePressureElectricsName = M.name .. "_pressure_service"
  parkingBrakePressureElectricsName = M.name .. "_pressure_parking"

  electrics.values[parkingBrakePressureElectricsName] = 0
  electrics.values[serviceBrakePressureElectricsName] = 0

  if jbeamData.soundNode_nodes and type(jbeamData.soundNode_nodes) == "table" and type(jbeamData.soundNode_nodes[1]) == "number" then
    soundNode = jbeamData.soundNode_nodes[1]
  else
    soundNode = 0
  end
  soundEvent = jbeamData.soundEvent or "event:>Vehicle>Pneumatics>Air_Brakes"
  soundCoolDown = jbeamData.soundCoolDown or 0.75
  serviceSoundCoolDown = soundCoolDown
  parkingSoundCoolDown = soundCoolDown

  bdebug.setNodeDebugText("Airbrakes", soundNode, M.name .. ": " .. (soundEvent or "no event"))

  didAutoApplyParkingBrake = false

  log("D", "airbrakes.init", "Air brakes initialized")
end

M.init = init
M.reset = reset
M.updateFixedStep = updateFixedStep
M.updateGFX = updateGFX

return M
