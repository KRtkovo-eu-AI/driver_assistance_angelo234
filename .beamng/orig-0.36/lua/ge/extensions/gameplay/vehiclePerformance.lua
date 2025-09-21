-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local vehicleClasses = {
  {name = "X",  minPI = 101, description = "Modified"},
  {name = "S",  minPI = 86,  description = "Super Sports"},
  {name = "A",  minPI = 66,  description = "High Performance"},
  {name = "B",  minPI = 41,  description = "Sports"},
  {name = "C",  minPI = 21,  description = "Standard"},
  {name = "D",  minPI = 0,   description = "Economy/Utility"}
}

local performanceIndexMultipliers = {
  time_60 = {bias = 19.61451, multiplier = -8.29016},
  time_330 = {bias = 9.67742, multiplier = -0.96774},
  time_1000 = {bias = 15, multiplier = -0.5},
  time_1_8 = {bias = 22.40741, multiplier = -2.03704},
  time_1_4 = {bias = 29.14286, multiplier = -1.61905},
  time_0_60 = {bias = 21.42857, multiplier = -1.42857},
  velAt_1_8 = {bias = -8.00000, multiplier = 0.53333},
  velAt_1_4 = {bias = -6.00000, multiplier = 0.40000},
  brakingG = {bias = -3.57143, multiplier = 7.14286},
}

-- taken from the scintilla drag
local baseCertificationData = {
  time_60 = 1.833000027,
  time_330 = 4.06650006,
  time_1_8 = 5.666500084,
  time_1_4 = 8.133500122,
  velAt_1_4 = 90.25240982,
  velAt_1_8 = 70.94814975,
  time_0_60 = 2.366500035,
  brakingG = 1.41712463,
  power = {propulsionPowerCombined = 1395241}, -- power in watts
  torque = 1740,
  weight = 1280,
}

local function getWeightedValue(testData, category)
  if not testData[category] then return 0 end
  return math.max(performanceIndexMultipliers[category].bias + testData[category] * performanceIndexMultipliers[category].multiplier, 0)
end

local function getAggregateScores(testData)
  if not testData then return end

  local power = testData.power and testData.power.propulsionPowerCombined
  local basePower = baseCertificationData.power and baseCertificationData.power.propulsionPowerCombined

  local aggregateScores = {}

  aggregateScores.powerScore = power and (power / basePower) * 100 or 0
  aggregateScores.powerToWeightScore = (power and testData.weight) and
    (power / testData.weight) / (basePower / baseCertificationData.weight) * 100 or 0
  aggregateScores.timeTo60Score = (baseCertificationData.time_60 / testData.time_60) * 100
  aggregateScores.quarterMileScore = (baseCertificationData.time_1_4 / testData.time_1_4) * 100
  aggregateScores.speedProgressionScore = ((testData.velAt_1_4 - testData.velAt_1_8) / (baseCertificationData.velAt_1_4 - baseCertificationData.velAt_1_8)) * 100
  aggregateScores.brakingGForceScore = testData.brakingG and ((testData.brakingG / baseCertificationData.brakingG) * 100) or 0

  if testData.time_0_60 == 0 then testData.time_0_60 = nil end -- a time of 0 means the vehicle is not able to reach 60mph
  aggregateScores.time0To60Score = testData.time_0_60 and ((baseCertificationData.time_0_60 / testData.time_0_60) * 100) or 0

  return aggregateScores
end

local function getPerformanceIndex(testData)
  local weightedRawValueSum =
    getWeightedValue(testData, "time_60") +
    getWeightedValue(testData, "time_330") +
    getWeightedValue(testData, "time_1000") +
    getWeightedValue(testData, "time_1_8") +
    getWeightedValue(testData, "time_1_4") +
    getWeightedValue(testData, "time_0_60") +
    getWeightedValue(testData, "velAt_1_8") +
    getWeightedValue(testData, "velAt_1_4") +
    getWeightedValue(testData, "brakingG")

  return weightedRawValueSum
end

local function getClassFromData(testData)
  local performanceIndex = getPerformanceIndex(testData)
  if not performanceIndex then return end

  for _, class in ipairs(vehicleClasses) do
    if performanceIndex >= class.minPI then
      return {class = class, performanceIndex = performanceIndex}
    end
  end
end

local function getClassFromVehId(vehId)
  local vehicleDetails = core_vehicles.getVehicleDetails(vehId)
  if not vehicleDetails then return end

  local performanceData = vehicleDetails.configs["Drag Times"]
  if not performanceData then return end

  performanceData.power = vehicleDetails.configs["Power"]
  performanceData.weight = vehicleDetails.configs["Weight"]

  return getClassFromData(performanceData)
end

local function getClassFromConfig(model, configName)
  local vehicleDetails = core_vehicles.getConfig(model, configName)
  if not vehicleDetails then return end

  local performanceData = vehicleDetails["Drag Times"]
  if not performanceData then return end

  performanceData.power = vehicleDetails["Power"]
  performanceData.weight = vehicleDetails["Weight"]

  return getClassFromData(performanceData)
end

M.getClassFromData = getClassFromData
M.getClassFromVehId = getClassFromVehId
M.getClassFromConfig = getClassFromConfig

M.getAggregateScores = getAggregateScores

return M