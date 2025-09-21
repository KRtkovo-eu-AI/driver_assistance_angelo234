-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local streamControl = {}
local lsensors = {position = {}}
local envsensors = {}
local wheelInfo = {}
local wheelCache = {}

local streamsHandlers = {}

local hasBeenSet = false

local function willSend(name)
  return guihooks.updateStreams and streamControl[name]
end

local function reset()
  streamControl = {}
end

streamsHandlers.wheelInfo = function()
  table.clear(wheelInfo)
  for i, wd in pairs(wheels.wheelRotators) do
    local w = wheelCache[i] or {}
    w[1] = wd.name
    w[2] = wd.radius
    w[3] = wd.wheelDir
    w[4] = wd.angularVelocity
    w[5] = wd.propulsionTorque
    w[6] = wd.lastSlip
    w[7] = 0 --deprecated, used to be lastTorqueMode
    w[8] = wd.downForce
    w[9] = wd.brakingTorque
    w[10] = wd.brakeTorque
    wheelCache[i] = w
    wheelInfo[i] = w
  end
  guihooks.queueStream("wheelInfo", wheelInfo)
end

streamsHandlers.engineInfo = function()
  guihooks.queueStream("engineInfo", controller.mainController.engineInfo)
end

streamsHandlers.stats = function()
  local stats = obj:calcBeamStats()
  stats.tri_count = obj:getTriangleCount()
  stats.collidable_tri_count = obj:getCollidableTriangleCount()
  guihooks.queueStream("stats", stats)
end

streamsHandlers.electrics = function()
  guihooks.queueStream("electrics", electrics.values)
end

streamsHandlers.sensors = function()
  lsensors.gx = sensors.gx
  lsensors.gy = sensors.gy
  lsensors.gz = sensors.gz
  lsensors.gx2 = sensors.gx2
  lsensors.gy2 = sensors.gy2
  lsensors.gz2 = sensors.gz2
  lsensors.forceAtWheelNorm = hydros.forceAtWheelNorm
  lsensors.forceAtDriverNorm = hydros.forceAtDriverNorm
  lsensors.curForceLimitNorm = hydros.curForceLimitNorm
  lsensors.torqueCurrent = hydros.torqueCurrent
  local lp = lsensors.position
  lp.x, lp.y, lp.z = obj:getPositionXYZ()
  lsensors.roll, lsensors.pitch, lsensors.yaw = obj:getRollPitchYaw()
  lsensors.gravity = obj:getGravity()
  guihooks.queueStream("sensors", lsensors)
end

streamsHandlers.environment = function()
  envsensors.temperature = obj:getEnvTemperature()
  envsensors.pressure = obj:getEnvPressure()
  guihooks.queueStream("environment", envsensors)
end

local function update()
  for k, _ in pairs(streamControl) do
    local handler = streamsHandlers[k]
    if handler then
      handler()
    end
  end
end

local function setRequiredStreams(state)
  --log('E', '', objectId .. ' - got streams: ' .. dumps(state))
  hasBeenSet = true
  table.clear(streamControl)
  for _, streamName in pairs(state) do
    streamControl[streamName] = true
  end
end

local function hasActiveStreams()
  return next(streamControl) ~= nil or not hasBeenSet
end

local graphValues
local function drawGraph(k, val)
  if willSend("genericGraphAdvanced") then
    graphValues = graphValues or {_fidx = 0}
    graphValues._fidx = graphValues._fidx + 1
    graphValues[k] = val
    guihooks.queueStream("genericGraphAdvanced", graphValues)
  end
end

-- public interface
M.reset = reset
M.update = update
M.setRequiredStreams = setRequiredStreams
M.willSend = willSend
M.hasActiveStreams = hasActiveStreams
M.drawGraph = drawGraph

return M
