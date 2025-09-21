-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"
M.defaultOrder = 65

M.isActive = false
M.isActing = false

M.tractionControlledWheels = {}

local abs = math.abs
local sign2 = sign2
local floor = math.floor

local CMU = nil
local isDebugEnabled = false

local controlParameters = {isEnabled = true}
local initialControlParameters

local debugPacket = {sourceType = "tractionControl", wheelData = {}, wheelGroupData = {}}
local configPacket = {sourceType = "tractionControl", packetType = "config", config = controlParameters}

local tractionControlledWheelGroups = {}
local tractionControlledWheelGroupsCount = 0

local slipProviders = {}
local slipProviderCount = 0

local tractionControlComponents = {}
local tractionControlComponentCount = 0

local tractionControlActive
local isActiveSmoothed
local isActiveSmoother = newTemporalSmoothing(10, 5)

local function updateFixedStep(dt)
  local vehicleVelocity = CMU.virtualSensors.virtual.speed
  local velocityOffset = 0
  local velocityOffsetThreshold = controlParameters.velocityOffsetThreshold
  if abs(vehicleVelocity) < velocityOffsetThreshold then
    velocityOffset = (velocityOffsetThreshold - abs(vehicleVelocity)) * sign2(vehicleVelocity)
  end

  local steeringCoef = linearScale(abs(CMU.sensorHub.steeringInput), 0.3, 0.5, 1, 0.5)

  tractionControlActive = false
  for i = 1, tractionControlledWheelGroupsCount do
    local wheelGroup = tractionControlledWheelGroups[i]

    for j = 1, slipProviderCount do
      local slipProvider = slipProviders[j]
      local success = slipProvider.calculateSlipTractionControl(wheelGroup, steeringCoef, velocityOffset, dt)
      if success then
        break
      end
    end

    for j = 1, tractionControlComponentCount do
      local component = tractionControlComponents[j]
      local didAct = component.actAsTractionControl(wheelGroup, dt)
      tractionControlActive = didAct or tractionControlActive
    end
  end
end

local function updateGFX(dt)
  isActiveSmoothed = isActiveSmoother:getUncapped(tractionControlActive and 1 or 0, dt)
  M.isActing = isActiveSmoothed >= 1
  electrics.values.tcs = floor(isActiveSmoothed) * CMU.warningLightPulse
  electrics.values.tcsActive = isActiveSmoothed >= 1
  if not controlParameters.isEnabled then
    electrics.values.tcs = 1
  end
end

local function updateGFXDebug(dt)
  updateGFX(dt)

  for i = 1, tractionControlledWheelGroupsCount do
    local wheelGroup = tractionControlledWheelGroups[i]
    for j = 1, wheelGroup.wheelCount do
      local wheelData = wheelGroup.wheels[j]
      local wd = wheelData.wd
      debugPacket.wheelData[wd.name] = debugPacket.wheelData[wd.name] or {}
      debugPacket.wheelData[wd.name].AV = wd.angularVelocityBrakeCouple * wd.wheelDir
      debugPacket.wheelData[wd.name].slip = wheelData.slip
    end
    debugPacket.wheelGroupData[wheelGroup.name] = debugPacket.wheelGroupData[wheelGroup.name] or {}
    debugPacket.wheelGroupData[wheelGroup.name].slipRange = wheelGroup.slipRange
  end

  debugPacket.isActive = isActiveSmoothed

  debugPacket.isEnabled = controlParameters.isEnabled
  debugPacket.velocityOffsetThreshold = controlParameters.velocityOffsetThreshold

  CMU.sendDebugPacket(debugPacket)
end

local function setDebugMode(debugEnabled)
  isDebugEnabled = debugEnabled

  M.updateGFX = isDebugEnabled and updateGFXDebug or updateGFX
end

local function registerCMU(cmu)
  CMU = cmu
end

local function registerComponent(component)
  table.insert(tractionControlComponents, component)
  tractionControlComponentCount = tractionControlComponentCount + 1
end

local function registerSlipProvider(slipProvider)
  table.insert(slipProviders, slipProvider)
  slipProviderCount = slipProviderCount + 1
end

local function reset()
  for _, wheelGroup in pairs(tractionControlledWheelGroups) do
    wheelGroup.slipRange = 0
    for _, wheel in pairs(wheelGroup.wheels) do
      wheel.slip = 0
      wheel.lastSlip = 0
      wheel.slipSmoother:reset()
    end
  end
end

local function init(jbeamData)
  slipProviders = {}
  slipProviderCount = 0
  tractionControlComponents = {}
  tractionControlComponentCount = 0

  controlParameters.velocityOffsetThreshold = jbeamData.velocityOffsetThreshold or 10

  M.isActive = true
end

local function initSecondStage(jbeamData)
  tractionControlledWheelGroups = {}
  tractionControlledWheelGroupsCount = 0

  M.tractionControlledWheels = {}

  local tractionControlledMotors = jbeamData.tractionControlledMotors or {}
  for _, motorName in ipairs(tractionControlledMotors) do
    local motor = powertrain.getDevice(motorName)
    if motor then
      local childWheels = powertrain.getChildWheels(motor)
      local wheelGroup = {
        name = motorName,
        motor = motor,
        wheels = {},
        maxSlip = 0,
        minSlip = 1,
        slipRange = 0
      }
      for _, wheel in ipairs(childWheels) do
        local wheelData = {
          wd = wheel,
          name = wheel.name,
          slip = 0,
          lastSlip = 0,
          slipSmoother = newExponentialSmoothing(10)
        }
        table.insert(wheelGroup.wheels, wheelData)
        table.insert(M.tractionControlledWheels, {wheel = wheel, wheelGroup = wheelGroup})
      end
      wheelGroup.wheelCount = #wheelGroup.wheels
      table.insert(tractionControlledWheelGroups, wheelGroup)
    end
  end

  tractionControlledWheelGroupsCount = #tractionControlledWheelGroups

  electrics.values.hasTCS = true

  initialControlParameters = deepcopy(controlParameters)
end

local function initLastStage(jbeamData)
  --sort components and slip providers based on their order
  table.sort(
    slipProviders,
    function(a, b)
      local ra, rb = a.providerOrder or a.order or 0, b.providerOrder or b.order or 0
      return ra < rb
    end
  )
  table.sort(
    tractionControlComponents,
    function(a, b)
      local ra, rb = a.componentOrderTractionControl or a.order or 0, b.componentOrderTractionControl or b.order or 0
      return ra < rb
    end
  )
end

local function shutdown()
  M.isActive = false
  M.updateGFX = nil
  M.update = nil
end

local function updateIsEnabled(isEnabled)
  for _, v in ipairs(slipProviders) do
    v.setParameters({["tractionControl.isEnabled"] = isEnabled})
  end
  for _, v in ipairs(tractionControlComponents) do
    v.setParameters({["tractionControl.isEnabled"] = isEnabled})
  end
end

local function setParameters(parameters)
  CMU.applyParameter(controlParameters, initialControlParameters, parameters, "velocityOffsetThreshold")
  if CMU.applyParameter(controlParameters, initialControlParameters, parameters, "isEnabled") then
    updateIsEnabled(controlParameters.isEnabled)
  end
end

local function setConfig(configTable)
  controlParameters = configTable
  updateIsEnabled(controlParameters.isEnabled)
end

local function getConfig()
  return deepcopy(controlParameters)
end

local function sendConfigData()
  configPacket.config = controlParameters
  CMU.sendDebugPacket(configPacket)
end

M.init = init
M.initSecondStage = initSecondStage
M.initLastStage = initLastStage

M.reset = reset

M.updateGFX = updateGFX
M.updateFixedStep = updateFixedStep

M.registerCMU = registerCMU
M.setDebugMode = setDebugMode
M.registerComponent = registerComponent
M.registerSlipProvider = registerSlipProvider
M.shutdown = shutdown
M.setParameters = setParameters
M.setConfig = setConfig
M.getConfig = getConfig
M.sendConfigData = sendConfigData

return M
