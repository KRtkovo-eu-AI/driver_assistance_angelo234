-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local max = math.max
local abs = math.abs

M.recordingTypes = {
  power = "power",
  torque = "torque",
  speed = "speed",
  longitudinalAcceleration = "longitudinalAcceleration",
  lateralAcceleration = "lateralAcceleration"
}

M.staticTypes = {
  weight = "weight",
  powertrainLayout = "powertrainLayout",
  transmissionStyle = "transmissionStyle",
  propulsionType = "propulsionType",
  fuelType = "fuelType",
  inductionType = "inductionType"
}

local recordingStartMethods
local recordingStopMethods
local recordingUpdateMethods

local activeRecordingTypes

local recordingData

local function getWeight()
  local stats = obj:calcBeamStats()
  return stats.total_weight
end

local function getPowertrainLayout()
  local propulsedWheelsCount = 0
  local wheelCount = 0

  local avgWheelPos = vec3(0, 0, 0)
  for _, wd in pairs(wheels.wheels) do
    wheelCount = wheelCount + 1
    local wheelNodePos = v.data.nodes[wd.node1].pos --find the wheel position
    avgWheelPos = avgWheelPos + wheelNodePos --sum up all positions
    if wd.isPropulsed then
      propulsedWheelsCount = propulsedWheelsCount + 1
    end
  end

  avgWheelPos = avgWheelPos / wheelCount --make the average of all positions

  local vectorForward = vec3(v.data.nodes[v.data.refNodes[0].ref].pos) - vec3(v.data.nodes[v.data.refNodes[0].back].pos) --vector facing forward
  local vectorUp = vec3(v.data.nodes[v.data.refNodes[0].up].pos) - vec3(v.data.nodes[v.data.refNodes[0].ref].pos)
  local vectorRight = vectorForward:cross(vectorUp) --vector facing to the right

  local propulsedWheelLocations = {fr = 0, fl = 0, rr = 0, rl = 0}
  for _, wd in pairs(wheels.wheels) do
    if wd.isPropulsed then
      local wheelNodePos = vec3(v.data.nodes[wd.node1].pos) --find the wheel position
      local wheelVector = wheelNodePos - avgWheelPos --create a vector from our "center" to the wheel
      local dotForward = vectorForward:dot(wheelVector) --calculate dot product of said vector and forward vector
      local dotLeft = vectorRight:dot(wheelVector) --calculate dot product of said vector and left vector

      if dotForward >= 0 then
        if dotLeft >= 0 then
          propulsedWheelLocations.fr = propulsedWheelLocations.fr + 1
        else
          propulsedWheelLocations.fl = propulsedWheelLocations.fl + 1
        end
      else
        if dotLeft >= 0 then
          propulsedWheelLocations.rr = propulsedWheelLocations.rr + 1
        else
          propulsedWheelLocations.rl = propulsedWheelLocations.rl + 1
        end
      end
    end
  end

  local layout = {}
  layout.poweredWheelsFront = propulsedWheelLocations.fl + propulsedWheelLocations.fr
  layout.poweredWheelsRear = propulsedWheelLocations.rl + propulsedWheelLocations.rr

  return layout
end

local function getTransmissionStyle()
  local transmissionTypes = {}
  local transmissions = powertrain.getDevicesByCategory("gearbox")
  for _, v in pairs(transmissions) do
    transmissionTypes[v.type] = true
  end

  return transmissionTypes
end

local function getPropulsionType()
end

local function getFuelType()
  local energyStorages = energyStorage.getStorages()
  local fuelTypes = {}
  for _, v in pairs(energyStorages) do
    if v.type == "fuelTank" then
      fuelTypes[v.type .. ":" .. v.energyType] = true
    elseif v.type == "electricBattery" then
      fuelTypes[v.type] = true
    elseif v.type ~= "n2oTank" then
      fuelTypes[v.type] = true
    end
  end

  return fuelTypes
end

local function getInductionType()
  local engines = powertrain.getDevicesByType("combustionEngine")
  local inductionTypes = {}
  for _, v in pairs(engines) do
    inductionTypes.naturalAspiration = true
    if v.turbocharger.isExisting then
      inductionTypes.turbocharger = true
    end
    if v.supercharger.isExisting then
      inductionTypes.supercharger = true
    end
    if v.nitrousOxideInjection.isExisting then
      inductionTypes.N2O = true
    end
  end

  return inductionTypes
end

--### Power ###
local function startRecordingPower()
  recordingData[M.recordingTypes.power].propulsionPowerCombined = 0
  recordingData[M.recordingTypes.power].wheelPropulsionDevices = powertrain.getAllWheelPropulsionDevices()
end

local function stopRecordingPower()
  recordingData[M.recordingTypes.power].wheelPropulsionDevices = nil
end

local function updateRecordingPower(dt)
  local power = 0
  for _, propulsionDevice in ipairs(recordingData[M.recordingTypes.power].wheelPropulsionDevices) do
    if propulsionDevice.combustionTorque then
      power = power + propulsionDevice.combustionTorque * propulsionDevice.outputAV1
    else
      power = power + propulsionDevice.outputAV1 * propulsionDevice.outputTorque1
    end
  end
  recordingData[M.recordingTypes.power].propulsionPowerCombined = max(recordingData[M.recordingTypes.power].propulsionPowerCombined, power)
end
--################

--### Torque ###
local function startRecordingTorque()
  recordingData[M.recordingTypes.torque].propulsionTorqueCombined = 0
  recordingData[M.recordingTypes.torque].wheelPropulsionDevices = powertrain.getAllWheelPropulsionDevices()
end

local function stopRecordingTorque()
  recordingData[M.recordingTypes.torque].wheelPropulsionDevices = nil
end

local function updateRecordingTorque(dt)
  local torque = 0
  for _, propulsionDevice in ipairs(recordingData[M.recordingTypes.torque].wheelPropulsionDevices) do
    if propulsionDevice.combustionTorque then
      torque = torque + propulsionDevice.combustionTorque
    else
      torque = torque + propulsionDevice.outputTorque1
    end
  end
  recordingData[M.recordingTypes.torque].propulsionTorqueCombined = max(recordingData[M.recordingTypes.torque].propulsionTorqueCombined, torque)
end
--################

--### Speed ###
local function startRecordingSpeed()
end

local function stopRecordingSpeed()
end

local function updateRecordingSpeed(dt)
end
--################

--### Longitudinal Acceleration ###
local function startRecordingLongitudinalAcceleration()
  recordingData[M.recordingTypes.longitudinalAcceleration].maxAcceleration = 0
end

local function stopRecordingLongitudinalAcceleration()
end

local function updateRecordingLongitudinalAcceleration(dt)
  local longitudinalAcceration = abs(sensors.gy2)
  recordingData[M.recordingTypes.longitudinalAcceleration].maxAcceleration = max(recordingData[M.recordingTypes.longitudinalAcceleration].maxAcceleration, longitudinalAcceration)
end
--################

--### Lateral Acceleration ###
local function startRecordingLateralAcceleration()
  recordingData[M.recordingTypes.lateralAcceleration].maxAcceleration = 0
end

local function stopRecordingLateralAcceleration()
end

local function updateRecordingLateralAcceleration(dt)
  local lateralAcceration = abs(sensors.gx2)
  recordingData[M.recordingTypes.lateralAcceleration].maxAcceleration = max(recordingData[M.recordingTypes.lateralAcceleration].maxAcceleration, lateralAcceration)
end
--################

local function updateGFX(dt)
  --iterate active recordings and execute the update method
  for recordingType, _ in pairs(activeRecordingTypes) do
    recordingUpdateMethods[recordingType](dt)
  end
end

local function startRecording(recordingTypes)
  for _, recordingType in ipairs(recordingTypes) do
    recordingData[recordingType] = {}
    activeRecordingTypes[recordingType] = true
    if recordingStartMethods[recordingType] then
      recordingStartMethods[recordingType]()
    end
  end
  --activate updateGFX if there are active recordings
  if tableSize(activeRecordingTypes) > 0 and not M.updateGFX then
    M.updateGFX = updateGFX
    extensions.hookUpdate("updateGFX")
  end
end

local function stopRecording(recordingTypes)
  for _, recordingType in ipairs(recordingTypes) do
    activeRecordingTypes[recordingType] = nil
    if recordingStopMethods[recordingType] then
      recordingStopMethods[recordingType]()
    end
  end
  --deactivate updateGFX if there are no active recordings
  if tableSize(activeRecordingTypes) <= 0 then
    M.updateGFX = nil
    extensions.hookUpdate("updateGFX")
  end
end

local function getRecordingData(recordingTypes)
  local data = {}
  for _, recordingType in ipairs(recordingTypes) do
    data[recordingType] = recordingData[recordingType]
  end
  return data
end

local function getStaticData()
  return {
    weight = getWeight(),
    powertrainLayout = getPowertrainLayout(),
    transmissionStyle = getTransmissionStyle(),
    propulsionType = getPropulsionType(),
    fuelType = getFuelType(),
    inductionType = getInductionType()
  }
end

local function onExtensionLoaded()
  recordingData = {}
  activeRecordingTypes = {}
  recordingStartMethods = {
    [M.recordingTypes.power] = startRecordingPower,
    [M.recordingTypes.torque] = startRecordingTorque,
    [M.recordingTypes.speed] = startRecordingSpeed,
    [M.recordingTypes.longitudinalAcceleration] = startRecordingLongitudinalAcceleration,
    [M.recordingTypes.lateralAcceleration] = startRecordingLateralAcceleration
  }
  recordingStopMethods = {
    [M.recordingTypes.power] = stopRecordingPower,
    [M.recordingTypes.torque] = stopRecordingTorque,
    [M.recordingTypes.speed] = stopRecordingSpeed,
    [M.recordingTypes.longitudinalAcceleration] = stopRecordingLongitudinalAcceleration,
    [M.recordingTypes.lateralAcceleration] = stopRecordingLateralAcceleration
  }
  recordingUpdateMethods = {
    [M.recordingTypes.power] = updateRecordingPower,
    [M.recordingTypes.torque] = updateRecordingTorque,
    [M.recordingTypes.speed] = updateRecordingSpeed,
    [M.recordingTypes.longitudinalAcceleration] = updateRecordingLongitudinalAcceleration,
    [M.recordingTypes.lateralAcceleration] = updateRecordingLateralAcceleration
  }
end

M.onExtensionLoaded = onExtensionLoaded
M.updateGFX = nil

M.startRecording = startRecording
M.stopRecording = stopRecording

M.getRecordingData = getRecordingData
M.getStaticData = getStaticData

return M

--old torque/power data that we can't use in the future:
-- local function getTorquePower()
--   local engines = powertrain.getDevicesByCategory("engine")
--   if not engines or #engines <= 0 then
--     log("I", "vehicleCertifications", "Can't find any engine, not getting static performance data")
--     return 0, 0
--   end

--   local maxRPM = 0
--   local maxTorque = -1
--   local maxPower = -1
--   if #engines > 1 then
--     local torqueData = {}
--     for _, v in pairs(engines) do
--       local tData = v:getTorqueData()
--       maxRPM = max(maxRPM, tData.maxRPM)
--       table.insert(torqueData, tData)
--     end

--     local torqueCurve = {}
--     local powerCurve = {}
--     for _, td in ipairs(torqueData) do
--       local engineCurves = td.curves[td.finalCurveName]
--       for rpm, torque in pairs(engineCurves.torque) do
--         torqueCurve[rpm] = (torqueCurve[rpm] or 0) + torque
--       end
--       for rpm, power in pairs(engineCurves.power) do
--         powerCurve[rpm] = (powerCurve[rpm] or 0) + power
--       end
--     end
--     for _, torque in pairs(torqueCurve) do
--       maxTorque = max(maxTorque, torque)
--     end
--     for _, power in pairs(powerCurve) do
--       maxPower = max(maxPower, power)
--     end
--   else
--     local torqueData = engines[1]:getTorqueData()
--     maxRPM = torqueData.maxRPM
--     maxTorque = torqueData.maxTorque
--     maxPower = torqueData.maxPower
--   end

--   return maxTorque, maxPower, maxRPM
-- end
