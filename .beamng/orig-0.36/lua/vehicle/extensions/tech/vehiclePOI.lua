-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}


local min = math.min

-- Collects all the relevant vehicle Point-Of-Interest data, for use with the sensor configuration editor.
local function collectVehiclePOIData()

  -- Fetch the center of gravity positions (with and without wheels included).
  -- [Note: these are in world-space].
  local cogWithWheels = obj:calcCenterOfGravity(false)
  local cogWithoutWheels = obj:calcCenterOfGravity(true)

  -- Compute the mid front bumper and rear front bumper positions on the vehicle.
  -- [Note: these are in world-space].
  local pos, fwd, vehLength, vehFront = obj:getPosition(), obj:getDirectionVector(), obj:getInitialLength(), obj:getFrontPosition()
  fwd:normalize()
  local vehRear = vehFront - (fwd * vehLength)

  -- Compute the front and rear axle midpoints [assumes symmetric wheel layout].
  -- [Note: these are in world-space].
  local wp, ctr = {}, 0
  for _, wheel in pairs(wheels.wheels) do
    ctr = ctr + 1
    wp[ctr] = obj:getNodePosition(wheel.node1)
  end
  local frontAxleMidpoint, rearAxleMidpoint = pos + (wp[min(ctr, 3)] + wp[min(ctr, 4)]) * 0.5, pos + (wp[min(ctr, 1)] + wp[min(ctr, 2)]) * 0.5

  -- Pack the collected data and send it back to ge lua.
  local cData = {
    cogWithWheels = cogWithWheels - pos,
    cogWithoutWheels = cogWithoutWheels - pos,
    vehFront = vehFront - pos,
    vehRear = vehRear - pos,
    frontAxleMidpoint = frontAxleMidpoint - pos,
    rearAxleMidpoint = rearAxleMidpoint - pos,
    numWheels = ctr - 1 }
  obj:queueGameEngineLua(string.format("editor_sensorConfigurationEditor.updateCollectedVehiclePOIData(%q)", lpack.encode(cData)))
end


-- Public interface.
M.collectVehiclePOIData =                                 collectVehiclePOIData

return M