-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

local api = extensions.editor_api_dynamicDecals

local ACTIONS = {"material", "rename", "visibility"}
local PRESET_ACTIONS = {"rename", "visibility"}

M.layerUid = nil
M.layerData = nil

M.setLayer = function(layerUid)
  M.layerUid = layerUid
  M.layerData = deepcopy(api.getLayerByUid(layerUid))
  -- notify ui and extensions
end

M.changeColor = function(color)
  local layer = api.getLayerByUid(M.layerUid)

  -- check color palette map id, make sure it's set to zero to be editable
  if layer.colorPaletteMapId ~= 0 then
    layer.colorPaletteMapId = 0
  end

  if #color == 4 then
    layer.color = Point4F.fromTable(color)
  elseif #color == 3 then
    local r, g, b = unpack(color)
    layer.color = Point4F(r, g, b, 1)
  end
  api.setLayer(layer, true)
end

M.addLayer = function(params)
  if params and params.color then
    api.setFillLayerColorPaletteMapId(0)
    api.setFillLayerColor(params.color)
  end

  local layer = api.addFillLayer()
  M.layerUid = layer.uid

  -- set an initial color if colorPaletteMapId is set to 0
  if layer.colorPaletteMapId == 0 then
    local vehicleObj = getPlayerVehicle(0)
    layer.color = vehicleObj.color
  end

  M.layerData = deepcopy(api.getLayerByUid(layer.uid))

  return layer
end

M.updateLayer = function(params)
  if M.layerUid then
    if params.color then
      M.changeColor(params.color)
    end
  else
    log("W", "", "Unable to update non-existing layer " .. M.layerUid)
  end
end

M.saveChanges = function()
  if M.layerUid then
    M.layerData = deepcopy(api.getLayerByUid(M.layerUid))
  end
end

M.restoreLayer = function()
  if M.layerData then
    api.setLayer(M.layerData, true)
  end
end

M.restoreDefault = function()
  local layer = api.getLayerByUid(M.layerUid)
  layer.colorPaletteMapId = 1
  api.setLayer(layer, true)
  M.requestLayerData()
end

M.requestLayerData = function()
  if M.layerUid then
    local layer = api.getLayerByUid(M.layerUid)
    local data = {}

    if layer.colorPaletteMapId == 0 then
      color = layer.color:toTable()
    else
      local vehicleObj = getPlayerVehicle(0)
      data.color = vehicleObj.color:toTable()
    end

    guihooks.trigger("liveryEditor_fill_layerData", data)
  end
end

return M
