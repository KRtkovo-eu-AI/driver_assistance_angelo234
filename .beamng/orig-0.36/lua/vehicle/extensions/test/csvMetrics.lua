-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

local enabled = false
local frame = 1
local csvWriter = require('csvlib')
local csvFile = nil
local csvData = nil
local rate = nil
local logExtendedMetrics = nil

local time = 0

M.enable = function(path, writeRate, extended)
  csvFile = path
  rate = writeRate or 64
  logExtendedMetrics = extended == nil and true or false

  local header
  if not logExtendedMetrics then
    header = {'Time', 'PositionX', 'PositionY', 'PositionZ', 'RotationX', 'RotationY', 'RotationZ', 'RotationW'}
  else
    header = {
      'Time', 'PositionX', 'PositionY', 'PositionZ', 'RotationX', 'RotationY', 'RotationZ', 'RotationW', 'EnvTemp',
      'Airflow', 'Airspeed', 'Driveshaft', 'DriveshaftF', 'EngineLoad', 'ExhaustFlow', 'Fuel', 'FuelCapacity',
      'FuelVolume', 'GearIndex', 'Oil', 'OilTemp', 'RadiatorFanSpin', 'RPM', 'RPMSpin', 'WaterTemp', 'WheelSpeed'
    }
    for k, v in pairs(electrics.values.wheelThermals) do
      table.insert(header, 'BrakeCoreTemp' .. k)
      table.insert(header, 'BrakeSurfaceTemp' .. k)
      table.insert(header, 'BrakeThermalEfficiency' .. k)
    end
    for k, v in pairs(powertrain.getDevicesByType('combustionEngine')) do
      if v.thermals ~= nil then
        table.insert(header, 'EngineBlockTemp' .. v.name)
        table.insert(header, 'CylinderWallTemp' .. v.name)
        table.insert(header, 'CoolantTemp' .. v.name)
        table.insert(header, 'ExhaustTemp' .. v.name)
      end
    end
  end

  csvData = csvWriter.newCSV(unpack(header))
  enabled = true
end

local function writeRow()
  local pos = obj:getPosition()
  local rot = quat(obj:getRotation())

  local dataRow
  if not logExtendedMetrics then
    dataRow = {time, pos.x, pos.y, pos.z, rot.x, rot.y, rot.z, rot.w}
  else
    dataRow = {
      time, pos.x, pos.y, pos.z, rot.x, rot.y, rot.z, rot.w, obj:getEnvTemperature() - 273.15,
      electrics.values.airflowspeed or 0, electrics.values.airspeed or 0, electrics.values.driveshaft or 0,
      electrics.values.driveshaft_F or 0, electrics.values.engineLoad or 0, electrics.values.exhaustFlow or 0,
      electrics.values.fuel or 0, electrics.values.fuelCapacity or 0, electrics.values.fuelVolume or 0,
      electrics.values.gearIndex or 0, electrics.values.oil or 0, electrics.values.oiltemp or 0,
      electrics.values.radiatorFanSpin or 0, electrics.values.rpm or 0, electrics.values.rpmspin or 0,
      electrics.values.watertemp or 0, electrics.values.wheelspeed or 0
    }
    for k, v in pairs(electrics.values.wheelThermals) do
      table.insert(dataRow, v.brakeCoreTemperature or 0)
      table.insert(dataRow, v.brakeSurfaceTemperature or 0)
      table.insert(dataRow, v.brakeThermalEfficiency or 0)
    end
    for k, v in pairs(powertrain.getDevicesByType('combustionEngine')) do
      table.insert(dataRow, v.thermals.engineBlockTemperature)
      table.insert(dataRow, v.thermals.cylinderWallTemperature)
      table.insert(dataRow, v.thermals.coolantTemperature)
      table.insert(dataRow, v.thermals.exhaustTemperature)
    end
  end

  csvData:add(unpack(dataRow))
end

M.updateGFX = function(dt)
  if not enabled then
    return
  end

  time = time + dt
  frame = frame + 1

  if frame % rate == 0 then
    writeRow()
  end
end

M.disable = function()
  log('I', 'csvMetrics', 'Closing csv report file: ' .. csvFile)
  csvData:write(csvFile)
  enabled = false
  frame = 1
end

return M
