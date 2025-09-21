-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

-- transform a color from a   "0..1 0..1 0..1 0..2"    string to a   {0..1, 0..1, 0..1, 0..1}   array
local function colorStringToColorTable(colorString)
  if not colorString then return end
  local result = {}
  for i,v in ipairs(stringToTable(colorString, '%s')) do
    result[i] = tonumber(v)
  end
  result[4] = clamp(0.5*result[4], 0, 1) -- map 4th component from 0..2 to 0..1
  return result
end

local function updateVehicleDataPaint(index, colorString, vehId)
  local vd = extensions.core_vehicle_manager.getVehicleData(vehId)

  if not vd or not vd.config then
    log('I','setVehicleColor','Cannot set vehicle color. Vehicle config does not exit')
    return
  end

  vd.config.paints = vd.config.paints or {}
  local color = colorStringToColorTable(colorString)
  color[4] = color[4]*2
  local paint = createVehiclePaint({x=color[1], y=color[2], z=color[3], w=color[4]}, {color[5], color[6], color[7], color[8]})
  vd.config.paints[index] = paint

  return paint, vd.config.paints
end

local function setVehicleColor(index, colorString, objID)
  -- index from JS is zero based, LUA is 1 based
  index = index + 1
  -- log('I','setVehicleColor','setVehicleColor called: index = '..tostring(index)..' colorString = '..tostring(colorString)..' objID = '..tostring(objID))
  local objID = objID or be:getPlayerVehicleID(0)
  if not objID then return end

  local veh = getObjectByID(objID)
  local paint, allPaints = updateVehicleDataPaint(index, colorString, objID)
  extensions.core_vehicle_manager.liveUpdateVehicleColors(objID, veh, index, paint)

  if allPaints then
    -- Save paint to config
    extensions.core_vehicle_partmgmt.setConfigPaints(allPaints, false)
  end
end

local function setVehiclePaint(index, paint, objID)
  -- 1 based index
  local objID = objID or be:getPlayerVehicleID(0)
  if not objID then return end

  local veh = getObjectByID(objID)
  local vd = extensions.core_vehicle_manager.getVehicleData(objID)

  if not vd or not vd.config then
    log('I','setVehicleColor','Cannot set vehicle color. Vehicle config does not exit')
    return
  end
  vd.config.paints = vd.config.paints or {}
  vd.config.paints[index] = paint
  extensions.core_vehicle_manager.liveUpdateVehicleColors(objID, veh, index, paint)

  -- Save paint to config
  extensions.core_vehicle_partmgmt.setConfigPaints(vd.config.paints, false)
end

local function onVehicleSpawned(vehId)
  -- We set the paint data in vehicleData to the correct thing because otherwise vehicleData will be wrong when loading into the garage
  local vd = extensions.core_vehicle_manager.getVehicleData(vehId)
  vd.config.paints = vd.config.paints or {}

  local veh = getObjectByID(vehId)
  local metallicPaintData = veh:getMetallicPaintData()
  vd.config.paints[1] = createVehiclePaint(veh.color, metallicPaintData[1])
  vd.config.paints[2] = createVehiclePaint(veh.colorPalette0, metallicPaintData[2])
  vd.config.paints[3] = createVehiclePaint(veh.colorPalette1, metallicPaintData[3])

  -- round the values. same as we do in colorTableToRoundedColorString
  for i, paint in ipairs(vd.config.paints) do
    for attribute, value in pairs(paint) do
      if type(value) == "table" then
        for j, value2 in ipairs(value) do
          value[j] = round(value2*100)/100
        end
      else
        paint[attribute] = round(value*100)/100
      end
    end
  end
end

M.setVehicleColor = setVehicleColor
M.setVehiclePaint = setVehiclePaint
M.colorStringToColorTable = colorStringToColorTable

M.onVehicleSpawned = onVehicleSpawned
return M
