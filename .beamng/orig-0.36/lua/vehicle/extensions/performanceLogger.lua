-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local measurements = {}
local averageCounts = {}
local stopwatches = {}
local minimums = {}

local function startMeasurement(name)
  if not stopwatches[name] then
    averageCounts[name] = 0
    measurements[name] = 0
    stopwatches[name] = HighPerfTimer()
  else
    stopwatches[name]:reset()
  end
end

local function stopAvergage(name)
  local result = measurements[name] / averageCounts[name]
  stopwatches[name] = nil
  return result
end

local function measureAverage(name, stopCount, showMin)
  local measurement = stopwatches[name]:stop()
  measurements[name] = measurements[name] + measurement
  averageCounts[name] = averageCounts[name] + 1

  if averageCounts[name] >= stopCount then
    local result = stopAvergage(name)
    minimums[name] = math.min(minimums[name] or 9999, result)
    local display = tostring(showMin and minimums[name] or result)
    log("I", "performanceLogger.measureAverage", string.format("%s: %.5f ms (avg over %i measurements)", name, display, stopCount))
  end
end

local function stopMeasurement(name)
  local measurement = stopwatches[name]:stop()
  measurements[name] = measurement

  return measurement
end

local function onInit()
  stopwatches = {}
  measurements = {}
  averageCounts = {}
  minimums = {}
end

M.onInit = onInit
M.onReset = onInit
M.startMeasurement = startMeasurement
M.stopAvergage = stopAvergage
M.measureAverage = measureAverage
M.stopMeasurement = stopMeasurement

return M
