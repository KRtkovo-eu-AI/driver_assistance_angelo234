-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

M.gx = 0
M.gy = 0
M.gz = 0
M.gx2 = 0
M.gy2 = 0
M.gz2 = 0
M.ffiSensors = 0

local gx_smooth2 = newTemporalSmoothingNonLinear(7)
local gy_smooth2 = newTemporalSmoothingNonLinear(7)
local gz_smooth2 = newTemporalSmoothingNonLinear(7)

local function reset()
  gx_smooth2:reset()
  gy_smooth2:reset()
  gz_smooth2:reset()
end

local function updateGFX(dt)
  local ffisensors = M.ffiSensors
  M.gx = ffisensors.sensorX
  M.gy = ffisensors.sensorY
  M.gz = ffisensors.sensorZnonInertial

  M.gx2 = gx_smooth2:get(M.gx, dt)
  M.gy2 = gy_smooth2:get(M.gy, dt)
  M.gz2 = gz_smooth2:get(M.gz, dt)
end

local function init()
  M.ffiSensors = obj:getSensorsFFI()

  if not v.data.refNodes then
    return
  end

  if v.data.engine == nil and (v.data.hydros == nil or tableIsEmpty(v.data.hydros)) then
    return
  end

  M.reset()
end
-- public interface
M.updateGFX = updateGFX
M.reset = reset
M.init = init

return M
