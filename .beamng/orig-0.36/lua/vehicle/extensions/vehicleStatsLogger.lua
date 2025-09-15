-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- extension for the LogVehicleStats app
-- it logs a selection of stats to a csv file and is meant for the research side of BeamNG

local M = {}

local moduleIdx = {
  general = "General",
  wheels = "Wheels",
  inputs = "Inputs",
  engine = "Engine",
  powertrain = "Powertrain"
}

local logTag = 'vsl'

local record = {}
local outputStreams = {}
local settings = {}

local doLogging = false
local timeSinceStartOfLogging = 0
local secUntilNextUpdate = 0
local stepsSinceLastFlush = 0
local csvSeparator = ","

local devices = nil

local function updateDeviceStates()
  devices = powertrain.getDevices()
end

local function getStatValuePlaceHolder()
  return ""
end

local function addStatToRecord(moduleID, statID, getValue, csvDescription, jsonID)
  if jsonID == nil then
    jsonID = csvDescription
  end
  table.insert(record[moduleID], statID, {get = getValue, csvDescription = csvDescription, jsonID = jsonID})
  return statID + 1
end

local function addGeneralModule()
  record[moduleIdx.general] = {}
  local statID = 1
  statID =
    addStatToRecord(
    moduleIdx.general,
    statID,
    function()
      return timeSinceStartOfLogging
    end,
    "time"
  )
  statID =
    addStatToRecord(
    moduleIdx.general,
    statID,
    function()
      return obj:getPosition().x
    end,
    "vehicle x-position"
  )
  statID =
    addStatToRecord(
    moduleIdx.general,
    statID,
    function()
      return obj:getPosition().y
    end,
    "vehicle y-position"
  )
  statID =
    addStatToRecord(
    moduleIdx.general,
    statID,
    function()
      return obj:getPosition().z
    end,
    "vehicle z-position"
  )
  statID =
    addStatToRecord(
    moduleIdx.general,
    statID,
    function()
      return obj:getVelocity():length()
    end,
    "velocity (m/s)",
    "velocity"
  )
  statID =
    addStatToRecord(
    moduleIdx.general,
    statID,
    function()
      return electrics.values.rpm
    end,
    "revolutions per minute",
    "rpm"
  )
  statID =
    addStatToRecord(
    moduleIdx.general,
    statID,
    function()
      local roll, _, _ = obj:getRollPitchYaw()
      return roll
    end,
    "roll (radians)",
    "roll"
  )
  statID =
    addStatToRecord(
    moduleIdx.general,
    statID,
    function()
      local _, pitch, _ = obj:getRollPitchYaw()
      return pitch
    end,
    "pitch (radians)",
    "pitch"
  )
  statID =
    addStatToRecord(
    moduleIdx.general,
    statID,
    function()
      local _, _, yaw = obj:getRollPitchYaw()
      return yaw
    end,
    "yaw (radians)",
    "yaw"
  )
  statID =
    addStatToRecord(
    moduleIdx.general,
    statID,
    function()
      return electrics.values.watertemp
    end,
    "water temperature (Celsius)",
    "waterTemperature"
  )
  statID =
    addStatToRecord(
    moduleIdx.general,
    statID,
    function()
      return obj:getVelocity():length()
    end,
    "velocity (m/s)",
    "velocity"
  )
  statID =
    addStatToRecord(
    moduleIdx.general,
    statID,
    function()
      return electrics.values.steering
    end,
    "rotation of the steering wheel",
    "steeringWheelPosition"
  )
  statID =
    addStatToRecord(
    moduleIdx.general,
    statID,
    function()
      return electrics.values.throttle
    end,
    "throttle",
    "throttle"
  )
  statID =
    addStatToRecord(
    moduleIdx.general,
    statID,
    function()
      return electrics.values.brake
    end,
    "brake",
    "brake"
  )
  statID =
    addStatToRecord(
    moduleIdx.general,
    statID,
    function()
      return electrics.values.clutch
    end,
    "clutch",
    "clutch"
  )
  statID =
    addStatToRecord(
    moduleIdx.general,
    statID,
    function()
      return electrics.values.airspeed
    end,
    "airspeed",
    "airspeed"
  )
  statID =
    addStatToRecord(
    moduleIdx.general,
    statID,
    function()
      return electrics.values.airflowspeed
    end,
    "airflow speed",
    "airflowSpeed"
  )
  statID =
    addStatToRecord(
    moduleIdx.general,
    statID,
    function()
      return electrics.values.altitude
    end,
    "altitude",
    "altitude"
  )
  statID =
    addStatToRecord(
    moduleIdx.general,
    statID,
    function()
      return electrics.values.reverse
    end,
    "reverse",
    "reverse"
  )
  statID =
    addStatToRecord(
    moduleIdx.general,
    statID,
    function()
      return electrics.values.throttle
    end,
    "throttle",
    "throttle"
  )
end

local function addWheelsModule()
  record[moduleIdx.wheels] = {}
  local statID = 1
  statID =
    addStatToRecord(
    moduleIdx.wheels,
    statID,
    function()
      return timeSinceStartOfLogging
    end,
    "time"
  )
  statID =
    addStatToRecord(
    moduleIdx.wheels,
    statID,
    function()
      return electrics.values.avgWheelAV
    end,
    "average angular velocity of all wheels",
    "avgWheelAV"
  )

  for i = 0, 3 do
    local prefix = wheels.wheels[i].name .. ": "
    local getWheel = function()
      return wheels.wheels[i]
    end
    statID =
      addStatToRecord(
      moduleIdx.wheels,
      statID,
      function()
        return getWheel().wheelSpeed
      end,
      prefix .. "wheelSpeed"
    )
    statID =
      addStatToRecord(
      moduleIdx.wheels,
      statID,
      function()
        return getWheel().angularVelocity
      end,
      prefix .. "angularVelocity"
    )
    statID =
      addStatToRecord(
      moduleIdx.wheels,
      statID,
      function()
        return getWheel().isBroken
      end,
      prefix .. "isBroken"
    )
    statID =
      addStatToRecord(
      moduleIdx.wheels,
      statID,
      function()
        return getWheel().lastTorqueMode
      end,
      prefix .. "lastTorqueMode"
    )
    statID =
      addStatToRecord(
      moduleIdx.wheels,
      statID,
      function()
        return getWheel().isTireDeflated
      end,
      prefix .. "isTireDeflated"
    )
    statID =
      addStatToRecord(
      moduleIdx.wheels,
      statID,
      function()
        return getWheel().brakeDiameter
      end,
      prefix .. "brakeDiameter"
    )
    statID =
      addStatToRecord(
      moduleIdx.wheels,
      statID,
      function()
        return getWheel().brakeCoreTemperature
      end,
      prefix .. "brakeCoreTemperature"
    )
    statID =
      addStatToRecord(
      moduleIdx.wheels,
      statID,
      function()
        return getWheel().isBrakeMolten
      end,
      prefix .. "isBrakeMolten"
    )
    statID =
      addStatToRecord(
      moduleIdx.wheels,
      statID,
      function()
        return getWheel().rimTemperature
      end,
      prefix .. "rimTemperature"
    )
    statID =
      addStatToRecord(
      moduleIdx.wheels,
      statID,
      function()
        return getWheel().tireAirTemperature
      end,
      prefix .. "tireAirTemperature"
    )
    statID =
      addStatToRecord(
      moduleIdx.wheels,
      statID,
      function()
        return getWheel().brakeMass
      end,
      prefix .. "brakeMass"
    )
    statID =
      addStatToRecord(
      moduleIdx.wheels,
      statID,
      function()
        return getWheel().tireVolume
      end,
      prefix .. "tireVolume"
    )
    statID =
      addStatToRecord(
      moduleIdx.wheels,
      statID,
      function()
        return getWheel().inertia
      end,
      prefix .. "inertia"
    )
    statID =
      addStatToRecord(
      moduleIdx.wheels,
      statID,
      function()
        return getWheel().propulsionTorque
      end,
      prefix .. "propulsionTorque"
    )
    statID =
      addStatToRecord(
      moduleIdx.wheels,
      statID,
      function()
        return getWheel().propulsionTorque
      end,
      prefix .. "propulsionTorque"
    )
    statID =
      addStatToRecord(
      moduleIdx.wheels,
      statID,
      function()
        return getWheel().brakeTorque
      end,
      prefix .. "brakeTorque"
    )
  end
end

local function addEngineModule()
  record[moduleIdx.engine] = {}
  local statID = 1
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return timeSinceStartOfLogging
    end,
    "time"
  )

  updateDeviceStates()
  if devices == nil then
    log("E", logTag, "no devices found in powertrain")
    return
  end

  if devices.mainEngine == nil then
    log("D", logTag, "no engine found")
    return
  end

  local getMainEngine = function()
    return devices.mainEngine
  end
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getMainEngine().engineLoad
    end,
    "engineLoad"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getMainEngine().outputTorqueState
    end,
    "outputTorqueState"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getMainEngine().instantEngineLoad
    end,
    "instantEngineLoad"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getMainEngine().fundamentalFrequencyRPMCoefExhaust
    end,
    "fundamentalFrequencyRPMCoefExhaust"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getMainEngine().lastOutputTorque
    end,
    "lastOutputTorque"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getMainEngine().idleStartCoef
    end,
    "idleStartCoef"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getMainEngine().idleAVReadErrorRange
    end,
    "idleAVReadErrorRange"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getMainEngine().fastIgnitionErrorCoef
    end,
    "fastIgnitionErrorCoef"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getMainEngine().invTempRevLimiterRange
    end,
    "invTempRevLimiterRange"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getMainEngine().ignitionCoef
    end,
    "ignitionCoef"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getMainEngine().tempRevLimiterMaxAVOvershoot
    end,
    "tempRevLimiterMaxAVOvershoot"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getMainEngine().hasFuel
    end,
    "hasFuel"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getMainEngine().starterMaxAV
    end,
    "starterMaxAV"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getMainEngine().overTorqueDamage
    end,
    "overTorqueDamage"
  )

  local prefix = "thermals: "
  local getEngineThermals = function()
    return devices.mainEngine.thermals
  end
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getEngineThermals().connectingRodBearingsDamaged
    end,
    prefix .. "connectingRodBearingsDamaged"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getEngineThermals().coolantTemperature
    end,
    prefix .. "coolantTemperature"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getEngineThermals().cylinderWallOverheatDamage
    end,
    prefix .. "cylinderWallOverheatDamage"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getEngineThermals().cylinderWallTemperature
    end,
    prefix .. "cylinderWallTemperature"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getEngineThermals().cylinderWallsMelted
    end,
    prefix .. "cylinderWallsMelted"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getEngineThermals().radiatorFanSpin
    end,
    prefix .. "radiatorFanSpin"
  )

  local getEngineThermalsDebug = function()
    return devices.mainEngine.thermals.debugData.engineThermalData
  end
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getEngineThermalsDebug().coolantEfficiency
    end,
    prefix .. "coolantEfficiency"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getEngineThermalsDebug().coolantLeakRate
    end,
    prefix .. "coolantLeakRate"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getEngineThermalsDebug().energyBlockToAir
    end,
    prefix .. "energyBlockToAir"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getEngineThermalsDebug().energyCoolantToAir
    end,
    prefix .. "energyCoolantToAir"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getEngineThermalsDebug().energyCoolantToBlock
    end,
    prefix .. "energyCoolantToBlock"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getEngineThermalsDebug().energyCylinderWallToBlock
    end,
    prefix .. "energyCylinderWallToBlock"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getEngineThermalsDebug().energyCylinderWallToCoolant
    end,
    prefix .. "energyCylinderWallToCoolant"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getEngineThermalsDebug().energyCylinderWallToOil
    end,
    prefix .. "energyCylinderWallToOil"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getEngineThermalsDebug().energyExhaustToAir
    end,
    prefix .. "energyExhaustToAir"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getEngineThermalsDebug().energyOilSumpToAir
    end,
    prefix .. "energyOilSumpToAir"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getEngineThermalsDebug().energyOilToAir
    end,
    prefix .. "energyOilToAir"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getEngineThermalsDebug().energyOilToBlock
    end,
    prefix .. "energyOilToBlock"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getEngineThermalsDebug().energyToCylinderWall
    end,
    prefix .. "energyToCylinderWall"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getEngineThermalsDebug().energyToExhaust
    end,
    prefix .. "energyToExhaust"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getEngineThermalsDebug().energyToOil
    end,
    prefix .. "energyToOil"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getEngineThermalsDebug().engineBlockMelted
    end,
    prefix .. "engineBlockMelted"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getEngineThermalsDebug().engineBlockOverheatDamage
    end,
    prefix .. "engineBlockOverheatDamage"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getEngineThermalsDebug().engineBlockTemperature
    end,
    prefix .. "engineBlockTemperature"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getEngineThermalsDebug().engineEfficiency
    end,
    prefix .. "engineEfficiency"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getEngineThermalsDebug().exhaustTemperature
    end,
    prefix .. "exhaustTemperature"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getEngineThermalsDebug().fanActive
    end,
    prefix .. "fanActive"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getEngineThermalsDebug().headGasketBlown
    end,
    prefix .. "headGasketBlown"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getEngineThermalsDebug().oilOverheatDamage
    end,
    prefix .. "oilOverheatDamage"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getEngineThermalsDebug().oilTemperature
    end,
    prefix .. "oilTemperature"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getEngineThermalsDebug().oilThermostatStatus
    end,
    prefix .. "oilThermostatStatus"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getEngineThermalsDebug().oilThermostatTemperature
    end,
    prefix .. "oilThermostatTemperature"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getEngineThermalsDebug().pistonRingsDamaged
    end,
    prefix .. "pistonRingsDamaged"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getEngineThermalsDebug().radiatorAirSpeed
    end,
    prefix .. "radiatorAirSpeed"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getEngineThermalsDebug().radiatorAirSpeedEfficiency
    end,
    prefix .. "radiatorAirSpeedEfficiency"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getEngineThermalsDebug().thermostatStatus
    end,
    prefix .. "thermostatStatus"
  )
  statID =
    addStatToRecord(
    moduleIdx.engine,
    statID,
    function()
      return getEngineThermalsDebug().thermostatTemperature
    end,
    prefix .. "thermostatTemperature"
  )
  -- local getEngineThermalsDebug = function() return devices.mainEngine.thermals.debugData.engineThermalData end
  -- statID = addStatToRecord(moduleIdx.powertrain, statID, function() return getEngineThermalsDebug().coolantLeakRate end, uiName .. 'coolantLeakRate')
end

local function addInputsModule()
  record[moduleIdx.inputs] = {}
  local statID = 1
  statID =
    addStatToRecord(
    moduleIdx.inputs,
    statID,
    function()
      return timeSinceStartOfLogging
    end,
    "time"
  )
  statID =
    addStatToRecord(
    moduleIdx.inputs,
    statID,
    function()
      return input.state["throttle"].val
    end,
    "throttle"
  )
  statID =
    addStatToRecord(
    moduleIdx.inputs,
    statID,
    function()
      return input.state["steering"].val
    end,
    "steering"
  )
  statID =
    addStatToRecord(
    moduleIdx.inputs,
    statID,
    function()
      return input.state["clutch"].val
    end,
    "clutch"
  )
  statID =
    addStatToRecord(
    moduleIdx.inputs,
    statID,
    function()
      return input.state["parkingbrake"].val
    end,
    "parkingbrake"
  )
  statID =
    addStatToRecord(
    moduleIdx.inputs,
    statID,
    function()
      return input.state["brake"].val
    end,
    "brake"
  )
end

local function addPowertrainModule()
  record[moduleIdx.powertrain] = {}
  local statID = 1
  statID =
    addStatToRecord(
    moduleIdx.powertrain,
    statID,
    function()
      return timeSinceStartOfLogging
    end,
    "time"
  )

  updateDeviceStates()
  if devices == nil then
    log("E", logTag, "no devices found in powertrain")
    return
  end

  if devices.driveshaft ~= nil then
    local uiName = devices.driveshaft.uiName
    statID =
      addStatToRecord(
      moduleIdx.powertrain,
      statID,
      function()
        return devices.driveshaft.isBroken
      end,
      uiName .. " is broken"
    )
  end
end

local function initStatsRecord()
  addGeneralModule()
  addWheelsModule()
  addEngineModule()
  addInputsModule()
  addPowertrainModule()
end

local function getStatValues(moduleID)
  local line = "\n"
  for statID, stat in ipairs(record[moduleID]) do
    if settings.useStat[moduleID][statID] then
      line = line .. tostring(record[moduleID][statID].get()) .. csvSeparator
    else
      line = line .. getStatValuePlaceHolder() .. csvSeparator
    end
  end
  return line
end

local function updateStreams()
  for _, moduleID in pairs(moduleIdx) do
    if settings.useModule[moduleID] then
      local moduleOutput = getStatValues(moduleID)
      outputStreams[moduleID].txt = outputStreams[moduleID].txt .. moduleOutput
    end
  end
end

local function update()
  updateDeviceStates()
  updateStreams()
end

local function writeToCSV(fpath, string, mode)
  if mode == nil then
    mode = "a"
  end
  local fhandle = io.open(fpath, mode)
  if not fhandle then
    return nil
  end
  fhandle:write(string)
  fhandle:close()
end

local function getCSVHeader(moduleID)
  local header = ""
  for _, stat in ipairs(record[moduleID]) do
    header = header .. stat.csvDescription .. csvSeparator
  end
  return header
end

local function createCSVs()
  for _, moduleID in pairs(moduleIdx) do
    local header = getCSVHeader(moduleID)
    writeToCSV(outputStreams[moduleID].fpath, header, "w")
  end
end

local function initOutput()
  for _, moduleID in pairs(moduleIdx) do
    local fpath = settings.outputDir .. "\\" .. moduleID .. ".csv"
    outputStreams[moduleID] = {
      txt = "",
      fpath = fpath
    }
  end
end

local function flushOutputStream()
  for _, moduleID in pairs(moduleIdx) do
    local fpath = outputStreams[moduleID].fpath
    local txt = outputStreams[moduleID].txt
    writeToCSV(fpath, txt)
    outputStreams[moduleID].txt = ""
  end
end

local function initSettings()
  settings.outputDir = "VSL"
  settings.updatePeriod = 5
  settings.useModule = {}
  settings.useStat = {}

  for _, moduleID in pairs(moduleIdx) do
    settings.useModule[moduleID] = true
    settings.useStat[moduleID] = {}

    for statID, _ in ipairs(record[moduleID]) do
      settings.useStat[moduleID][statID] = true
    end
  end
end

local function applySettingsFromJSON(fpath)
  -- local fpath = settings.outputDir .. fname
  log("I", logTag, "importing settings from file: " .. fpath)
  local json = readFile(fpath)
  local s = jsonDecode(json)

  if s.updatePeriod ~= nil then
    settings.updatePeriod = s.updatePeriod
  end

  if s.outputDir ~= nil then
    settings.outputDir = s.outputDir
  end

  for _, moduleID in pairs(moduleIdx) do
    if s.useModule[moduleID] ~= nil then
      settings.useModule[moduleID] = s.useModule[moduleID]
    end

    for statID, stat in ipairs(record[moduleID]) do
      local useStat = s.useStat[moduleID][stat.jsonID]
      if useStat ~= nil then
        settings.useStat[moduleID][statID] = useStat
      end
    end
  end

  -- for moduleID, useModule in pairs(settings.useModule) do
  --     log("D", logTag, moduleID..": "..tostring(useModule))
  -- end
end

local function writeSettingsToJSON(fpath)
  log("I", logTag, "exporting settings to JSON: " .. fpath)
  local s = {}
  s.outputDir = settings.outputDir
  s.updatePeriod = settings.updatePeriod
  s.useModule = {}
  s.useStat = {}

  for moduleID, useModule in pairs(settings.useModule) do
    s.useModule[moduleID] = useModule
    s.useStat[moduleID] = {}

    for statID, stat in ipairs(record[moduleID]) do
      local useStat = settings.useStat[moduleID][stat.jsonID]
      s.useStat[moduleID][stat.jsonID] = settings.useStat[moduleID][statID]
    end
  end

  if not jsonWriteFile(fpath, s, true, 0) then
    log("E", logTag, "failed writing settings to file")
  end
end

local function onExtensionLoaded()
  initStatsRecord()
  initSettings()
  guihooks.trigger("LoadedVehicleStatsLogger")
end

local function startLogging()
  log("I", logTag, "start logging")
  doLogging = true
  timeSinceStartOfLogging = 0
  secUntilNextUpdate = 0
  stepsSinceLastFlush = 0
  initOutput()
  createCSVs()
end

local function stopLogging()
  log("I", logTag, "stop logging")
  doLogging = false
  flushOutputStream()
end

local function updateGFX(dt)
  if not doLogging then
    return
  end

  timeSinceStartOfLogging = timeSinceStartOfLogging + dt

  if secUntilNextUpdate <= 0 then
    secUntilNextUpdate = settings.updatePeriod
    update()
  else
    secUntilNextUpdate = secUntilNextUpdate - dt
  end

  if stepsSinceLastFlush == 1024 then
    flushOutputStream()
    stepsSinceLastFlush = 0
  else
    stepsSinceLastFlush = stepsSinceLastFlush + 1
  end
end

local function suggestOutputFilename()
  local get_fname = function(i)
    return "vehicle_stats_settings(" .. string.format("%03d", i) .. ").json"
  end
  local fn = settings.outputDir .. "\\" .. "vehicle_stats_settings.json"
  local i = 1
  while not FS:fileExists(fn) do
    fn = get_fname(i)
    i = i + 1
  end
  return fn
end

-- public interface

-- vars
M.settings = settings

-- functions
M.onExtensionLoaded = onExtensionLoaded
M.updateGFX = updateGFX

M.suggestOutputFilename = suggestOutputFilename
M.applySettingsFromJSON = applySettingsFromJSON
M.startLogging = startLogging
M.stopLogging = stopLogging
M.writeSettingsToJSON = writeSettingsToJSON

return M
