-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}


-- Collects all the relevant vehicle data, for use with the vehicle coupling editor.
local function collectVehicleData()

  -- Fetch the names of all the available wheels on this vehicle.
  local whData, wCtr = {}, 1
  for k, _ in pairs(wheels.wheelRotatorIDs) do
    whData[wCtr] = k
    wCtr = wCtr + 1
  end

  -- Fetch the names of all the available electrics signals on this vehicle.
  local elData, eCtr = {}, 1
  for k, v in pairs(electrics.values) do
    if type(k) == 'string' and type(v) ~= 'table' and type(v) ~= 'string' then
      if not string.find(k, "parent") then
        elData[eCtr] = { name = k, type = type(v) }
        eCtr = eCtr + 1
      end
    end
  end

  -- Collect the powertrain data.
  local pData = {}
  for _, device in pairs(powertrain.getDevices()) do
    pData[device.name] = {
      inputAV = device.inputAV,
      gearRatio = device.gearRatio,
      isBroken = device.isBroken,
      mode = device.mode,
      outputTorque1 = device.outputTorque1,
      outputTorque2 = device.outputTorque2,
      outputAV1 = device.outputAV1,
      outputAV2 = device.outputAV2 }
  end
  local pProc, pCtr = {}, 1
  for kO, vO in pairs(pData) do
    local prefix = kO .. ' '
    for kI, vI in pairs(vO) do
      local fullName = prefix .. kI
      if not string.find(fullName, "parent") and type(vI) ~= 'string' then
        pProc[pCtr] = { name = tostring(fullName), type = type(vI) }
        pCtr = pCtr + 1
      end
    end
  end

  -- Pack the collected data and send it back to ge lua.
  local cData = { wheels = whData, electrics = elData, powertrain = pProc }
  obj:queueGameEngineLua(string.format("editor_cosimulationSignalEditor.updateCollectedVehicleData(%q)", lpack.encode(cData)))
end


-- Public interface.
M.collectVehicleData =                                    collectVehicleData

return M