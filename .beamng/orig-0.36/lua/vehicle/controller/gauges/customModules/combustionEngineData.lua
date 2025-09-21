-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local min = math.min
local max = math.max

local wattToHorsePower = 0.001 * 1.35962

local updateFuelConsumption = false
local updateFuelDisplay = false
local updatePowerCalculation = false
local updateRemainingRange = false

local engines = {}
local fuelTanks = {}

local averagePower = 0
local avgPowerSmoother = newExponentialSmoothing(1000)

local currentPowerSmoother = newExponentialSmoothing(30)
local currentTorqueSmoother = newExponentialSmoothing(30)
local avgConsumptionSmoother = newExponentialSmoothing(1800) --update every 0.5 seconds, use values of last 10 minutes
local fuelDisplaySmoother = newTemporalSmoothing(5, 3)

local avgConsumptionPer100km = 0
local lastAVGConsumptionPer100km = 0
local avgConsumptionSmootherTimer = 0
local avgConsumptionSmootherTime = 0.5
local avgConsumptionDefaultValue = 10

local previousFuel = 0
local fuelSmoother = newTemporalSmoothing(50, 50)

local function updateGFX(dt)
end

local function updateGaugeData(moduleData, dt)
  local wheelspeed = electrics.values.wheelspeed
  local isMoving = wheelspeed > 0.5

  local fuelVolume = electrics.values.fuelVolume or 0
  local fuelConsumption = clamp((previousFuel - fuelVolume) / (dt * wheelspeed) * 1000 * 100, 0, 50) -- l/100km
  fuelConsumption = fuelSmoother:getUncapped(fuelConsumption, dt)
  previousFuel = fuelVolume

  if updateFuelDisplay then
    local fuelDisplay = clamp((3 * fuelConsumption) / 30, 0, 3)
    if (electrics.values.engineLoad or 0) <= 0 then
      fuelDisplay = -1
    end
    if wheelspeed < 1 and (electrics.values.throttle or 0) <= 0 then
      fuelDisplay = 0
    end
    moduleData.fuelDisplay = fuelDisplaySmoother:getUncapped(fuelDisplay, dt)
  end

  if updateFuelConsumption then
    if isMoving then
      avgConsumptionSmootherTimer = avgConsumptionSmootherTimer + dt
      if avgConsumptionSmootherTimer >= avgConsumptionSmootherTime then
        avgConsumptionPer100km = avgConsumptionSmoother:get(fuelConsumption)
        avgConsumptionSmootherTimer = avgConsumptionSmootherTimer - avgConsumptionSmootherTime
      end
    else
      avgConsumptionPer100km = lastAVGConsumptionPer100km
      avgConsumptionSmootherTimer = 0
    end
    moduleData.averageFuelConsumption = avgConsumptionPer100km
    moduleData.currentFuelConsumption = fuelConsumption

    lastAVGConsumptionPer100km = avgConsumptionPer100km
  end

  if updatePowerCalculation then
    local currentPower = 0
    local currentTorque = 0
    for _, motor in ipairs(engines) do
      currentPower = currentPower + (motor.outputTorque1 * motor.outputAV1)
      currentTorque = currentTorque + motor.outputTorque1
    end
    currentPower = currentPowerSmoother:get(currentPower) * wattToHorsePower --HP
    averagePower = avgPowerSmoother:get(isMoving and currentPower or averagePower)
    currentTorque = currentTorqueSmoother:get(currentTorque) --Nm
    moduleData.currentPower = isMoving and currentPower or 0
    moduleData.averagePower = averagePower
    moduleData.currentTorque = currentTorque
  end

  if updateRemainingRange then
    local energyLeft = 0
    local JToLiterCoef = 0
    for _, b in ipairs(fuelTanks) do
      local storage = energyStorage.getStorage(b)
      energyLeft = energyLeft + storage.storedEnergy
      JToLiterCoef = storage.energyDensity * storage.fuelLiquidDensity * 0.000000001
    end
    moduleData.remainingRange = avgConsumptionPer100km > 0 and (energyLeft * JToLiterCoef / avgConsumptionPer100km * 0.0001) or 0
  end
end

local function setupGaugeData(properties)
  updateFuelConsumption = properties.fuelConsumption or false
  updateFuelDisplay = properties.fuelDisplay or false
  updatePowerCalculation = properties.currentPower or false
  updateRemainingRange = properties.remainingRange or false
  if updateRemainingRange then
    updateFuelConsumption = true --we need this if we want the range
  end
  engines = powertrain.getDevicesByType("combustionEngine")
  for _, v in pairs(engines) do
    for _, j in pairs(v.registeredEnergyStorages) do
      table.insert(fuelTanks, j)
    end
  end
end

local function reset()
  avgPowerSmoother:reset()
  currentPowerSmoother:reset()
  currentTorqueSmoother:reset()
  fuelDisplaySmoother:reset()
  fuelSmoother:reset()

  avgConsumptionSmootherTimer = 0
end

local function init(jbeamData)
  avgConsumptionSmoother:set(avgConsumptionDefaultValue)
end

M.init = init
M.reset = reset
M.updateGFX = updateGFX

M.setupGaugeData = setupGaugeData
M.updateGaugeData = updateGaugeData

return M
