-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

-- sends the current vehicle data to the GE lua to be recorded alongside the other things

local function updateGFX(dt)
  local pos = obj:getPosition()
  local dir = obj:getDirectionVector()
  local d = {
    input.throttle,
    input.brake,
    input.steering,
    obj:getVelocity():length(),
    electrics.values.airspeed,
    electrics.values.wheelspeed,
    input.parkingbrake,
    {pos.x, pos.y, pos.z},
    {dir.x, dir.y, dir.z}
    --gForceX = sensors.gx2,
    --gForceY = sensors.gy2,
  }
  obj:queueGameEngineLua("extensions.test_utRecorder.onVehicleData(" .. obj:getId() .. "," .. serialize(d) .. ")")
end

M.updateGFX = updateGFX

return M
