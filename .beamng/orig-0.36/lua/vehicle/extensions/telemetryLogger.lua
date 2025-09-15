-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local telemetryData = {}
local dataPointIndex = 1
local isRecording = false
local csvHeaders = {"timestamp", "position", "velocity", "throttle", "brake", "steering", "parkingbrake", "gForceX", "gForceY"}
local timeStamp = 0

local function updateGFX(dt)
  if not isRecording then
    return
  end

  local throttle = input.throttle
  local brake = input.brake
  local steering = input.steering
  local velocity = obj:getVelocity():length()
  local parkingbrake = input.parkingbrake
  local gForceX = sensors.gx2
  local gForceY = sensors.gy2
  local pos = obj:getPosition()
  local position = string.format("(%s,%s,%s)", pos.x, pos.y, pos.z)
  timeStamp = timeStamp + dt

  telemetryData[dataPointIndex] = {timestamp = timeStamp, throttle = throttle, brake = brake, steering = steering, parkingbrake = parkingbrake, velocity = velocity, gForceX = gForceX, gForceY = gForceY, position = position}
  dataPointIndex = dataPointIndex + 1
end

local function escapeCSV(s)
  if string.find(s, '[,"]') then
    s = '"' .. string.gsub(s, '"', '""') .. '"'
  end
  return s
end

local function saveDataToDisk()
  isRecording = false

  local f = io.open("telemetryData_" .. os.date("%H-%M-%S") .. ".csv", "w")
  if not f then
    return false
  end
  local headers = "frame,"
  for _, p in pairs(csvHeaders) do
    headers = headers .. escapeCSV(p) .. ","
  end
  f:write(headers .. "\r\n")
  for k, v in pairs(telemetryData) do
    local s = k .. ","
    for _, p in pairs(csvHeaders) do
      s = s .. escapeCSV(v[p]) .. ","
    end
    f:write(s .. "\r\n")
  end
  f:close()

  M.onInit()
end

local function startRecording()
  isRecording = true
end

local function stopRecording()
  isRecording = false
end

local function onInit()
  telemetryData = {}
  dataPointIndex = 1
  timeStamp = 0
  isRecording = false
end

M.onReset = onInit
M.onInit = onInit
M.startRecording = startRecording
M.stopRecording = stopRecording
M.saveDataToDisk = saveDataToDisk
M.updateGFX = updateGFX

return M
