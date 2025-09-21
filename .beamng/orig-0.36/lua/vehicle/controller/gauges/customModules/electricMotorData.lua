-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local wattToHorsePower = 0.001 * 1.35962

local updatePowerDisplay = false
local updatePowerCalculation = false
local updateRemainingRange = false
local updateConsumptionGraph = false

local motors = {}
local batteries = {}

local averagePower = 0
local avgPowerSmoother = newExponentialSmoothing(1000)

local currentConsumptionSmoother = newExponentialSmoothing(50)
local currentPowerSmoother = newExponentialSmoothing(30)
local currentTorqueSmoother = newExponentialSmoothing(30)
local avgConsumptionSmoother = newExponentialSmoothing(1000)

local avgConsumptionPer100km = 0

local graphUpdateTime = 5
local graphUpdateTimer = graphUpdateTime

local function updateGFX(dt)
end

local function updateGaugeData(moduleData, dt)
  local wheelspeed = electrics.values.wheelspeed
  local isMoving = wheelspeed > 0.5

  local powerDisplay = 0
  local motorCount = 0
  local currentPower = 0
  local currentTorque = 0
  for _, motor in ipairs(motors) do
    powerDisplay = powerDisplay + (motor.throttle or 0)
    powerDisplay = powerDisplay - (motor.regenThrottle or 0)
    motorCount = motorCount + 1
    currentPower = currentPower + (motor.outputTorque1 * motor.outputAV1)
    currentTorque = currentTorque + motor.outputTorque1
  end

  currentPower = currentPowerSmoother:get(currentPower)
  currentTorque = currentTorqueSmoother:get(currentTorque)
  averagePower = avgPowerSmoother:get(isMoving and currentPower or averagePower)
  powerDisplay = motorCount > 0 and (powerDisplay / motorCount * (isMoving and 1 or 0)) or 0

  if updatePowerDisplay then
    moduleData.electricPowerDisplay = powerDisplay
  end

  if updatePowerCalculation then
    moduleData.currentPower = isMoving and (currentPower * wattToHorsePower) or 0
    moduleData.averagePower = averagePower * wattToHorsePower
    moduleData.currentTorque = currentTorque
  end

  local timeToGo100km = isMoving and (100 / (wheelspeed * 3.6)) or 0
  local currentConsumptionPer100km = currentPower * timeToGo100km

  currentConsumptionPer100km = currentConsumptionSmoother:get(currentConsumptionPer100km)
  avgConsumptionPer100km = avgConsumptionSmoother:get(isMoving and currentConsumptionPer100km or avgConsumptionPer100km)

  if updateRemainingRange then
    local energyLeft = 0
    for _, b in ipairs(batteries) do
      local storage = energyStorage.getStorage(b)
      energyLeft = energyLeft + storage.storedEnergy
    end
    moduleData.remainingRange = avgConsumptionPer100km > 0 and (energyLeft * 0.0278 / avgConsumptionPer100km) or 0
  end

  if updateConsumptionGraph then
    graphUpdateTime = graphUpdateTime + dt
    moduleData.averageConsumption = nil
    if graphUpdateTime >= graphUpdateTimer then
      moduleData.averageConsumption = isMoving and avgConsumptionPer100km or 0
      graphUpdateTime = graphUpdateTime - graphUpdateTimer
    end
  end
end

local function setupGaugeData(properties)
  updatePowerDisplay = properties.powerDisplay or false
  updatePowerCalculation = properties.currentPower or false
  updateRemainingRange = properties.remainingRange or false
  updateConsumptionGraph = properties.consumptionGraph or false
  motors = powertrain.getDevicesByType("electricMotor")
  for _, v in pairs(motors) do
    for _, j in pairs(v.registeredEnergyStorages) do
      table.insert(batteries, j)
    end
  end
end

local function reset()
  avgPowerSmoother:reset()
end

local function init(jbeamData)
end

M.init = init
M.reset = reset
M.updateGFX = updateGFX

M.setupGaugeData = setupGaugeData
M.updateGaugeData = updateGaugeData

return M
