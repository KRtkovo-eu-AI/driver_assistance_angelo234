-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"
M.defaultOrder = 60

M.isActive = false
M.isActing = false

local floor = math.floor

local CMU = nil
local isDebugEnabled = false

local controlParameters = {isEnabled = true}
local initialControlParameters

local debugPacket = {sourceType = "yawControl"}
local configPacket = {sourceType = "yawControl", packetType = "config", config = controlParameters}

local yawProviders = {}
local yawProviderCount = 0

local yawControlComponents = {}
local yawControlComponentCount = 0

local yawControlActive
local isActiveSmoothed
local isActiveSmoother = newTemporalSmoothing(10, 5)

local yawDifferenceAdjusted
local yawDifferenceRaw
local bodySlipAngleAdjusted

local function updateFixedStep(dt)
  local cmu = CMU
  local virtualSensors = cmu.virtualSensors
  local speed = virtualSensors.virtual.speed
  local expectedYaw = 0
  for j = 1, yawProviderCount do
    local yawProvider = yawProviders[j]
    expectedYaw = yawProvider.calculateExpectedYaw(speed, dt)
    if expectedYaw then
      break
    end
  end

  local measuredYaw = cmu.sensorHub.yawAVSmooth
  local bodySlipAngleTrustCoef = linearScale(virtualSensors.trustWorthiness.bodySlipAngle, 0.5, 0.8, 0, 1)
  local bodySlipAngleRaw = virtualSensors.virtual.bodySlipAngle * bodySlipAngleTrustCoef

  if measuredYaw * expectedYaw < 0 then --check if we are counter steering while oversteering.
    expectedYaw = -expectedYaw --If we do, we need to adjust our desired yaw rate because its sign is wrong at this point (since we are steering in the "wrong" direction)
  end

  local lowSpeedCoef = 1 - linearScale(speed, 8, 10, 1, 0)
  bodySlipAngleAdjusted = bodySlipAngleRaw * lowSpeedCoef

  yawDifferenceRaw = (measuredYaw - expectedYaw) * sign(measuredYaw)
  --TODO Remove if it proves to work without the adjustment
  yawDifferenceAdjusted = yawDifferenceRaw --* lowSpeedCoef

  yawControlActive = false
  for i = 1, yawControlComponentCount do
    local component = yawControlComponents[i]
    local didAct = component.actAsYawControl(measuredYaw, expectedYaw, yawDifferenceAdjusted, bodySlipAngleAdjusted, dt)
    yawControlActive = didAct or yawControlActive
  end
end

local function updateGFX(dt)
  isActiveSmoothed = isActiveSmoother:getUncapped(yawControlActive and 1 or 0, dt)
  M.isActing = isActiveSmoothed >= 1
  electrics.values.esc = floor(isActiveSmoothed) * CMU.warningLightPulse
  electrics.values.escActive = isActiveSmoothed >= 1
  if not controlParameters.isEnabled then
    electrics.values.esc = 1
  end
end

local function updateGFXDebug(dt)
  updateGFX(dt)

  debugPacket.isActive = isActiveSmoothed

  debugPacket.yawDifferenceRaw = yawDifferenceRaw
  debugPacket.yawDifferenceAdjusted = yawDifferenceAdjusted
  debugPacket.bodySlipAngleAdjusted = bodySlipAngleAdjusted

  CMU.sendDebugPacket(debugPacket)
end

local function setDebugMode(debugEnabled)
  isDebugEnabled = debugEnabled

  M.updateGFX = isDebugEnabled and updateGFXDebug or updateGFX
end

local function registerCMU(cmu)
  CMU = cmu
end

local function registerYawProvider(yawProvider)
  table.insert(yawProviders, yawProvider)
  yawProviderCount = yawProviderCount + 1
end

local function registerComponent(component)
  table.insert(yawControlComponents, component)
  yawControlComponentCount = yawControlComponentCount + 1
end

local function reset()
end

local function init(jbeamData)
  controlParameters = {isEnabled = true}

  yawProviders = {}
  yawProviderCount = 0
  yawControlComponents = {}
  yawControlComponentCount = 0

  M.isActive = true
end

local function initSecondStage(jbeamData)
  electrics.values.hasESC = true

  initialControlParameters = deepcopy(controlParameters)
end

local function initLastStage(jbeamData)
  --sort components and slip providers based on their order
  table.sort(
    yawProviders,
    function(a, b)
      local ra, rb = a.providerOrder or a.order or 0, b.providerOrder or b.orrder or 0
      return ra < rb
    end
  )
  table.sort(
    yawControlComponents,
    function(a, b)
      local ra, rb = a.componentOrderYawControl or a.order or 0, b.componentOrderYawControl or b.order or 0
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
  for _, v in ipairs(yawProviders) do
    v.setParameters({isEnabled = controlParameters.isEnabled})
  end
  for _, v in ipairs(yawControlComponents) do
    v.setParameters({["yawControl.isEnabled"] = controlParameters.isEnabled})
  end
end

local function setParameters(parameters)
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
M.registerYawProvider = registerYawProvider
M.shutdown = shutdown
M.setParameters = setParameters
M.setConfig = setConfig
M.getConfig = getConfig
M.sendConfigData = sendConfigData

return M
