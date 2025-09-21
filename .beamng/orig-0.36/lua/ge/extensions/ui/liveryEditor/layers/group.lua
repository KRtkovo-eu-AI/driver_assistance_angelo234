-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

local api = extensions.editor_api_dynamicDecals

local ACTIONS = { "material", "ungroup", "lock", "delete" }

local getLayerActions = function()
  return ACTIONS
end

M.setColor = function(layer, rgbaArray)
  dump("group setColor", rgbaArray)
  -- for k, childLayer in ipairs(layer.children) do
  --   childLayer.color = Point4F.fromTable(rgbaArray)
  --   api.setLayer(childLayer, true)
  -- end
  -- local color = Point4F.fromTable(rgbaArray)
  if not layer.properties.color then
    layer.properties["color"] = { id = "color", value = rgbaArray }
  else
    layer.properties["color"].value = rgbaArray
  end

  layer.propertiesDirty = true
  api.setLayer(layer, false)

  layer.propertiesDirty = false
  api.setLayer(layer, true)
end


M.setMetallicIntensity = function(layer, metallicIntensity)
  for k, childLayer in ipairs(layer.children) do
    childLayer.metallicIntensity = metallicIntensity
    api.setLayer(childLayer, true)
  end
end

M.setRoughnessIntensity = function(layer, roughnessIntensity)
  for k, childLayer in ipairs(layer.children) do
    childLayer.roughnessIntensity = roughnessIntensity
    api.setLayer(childLayer, true)
  end
end

M.setNormalIntensity = function(layer, normalIntensity)
  for k, childLayer in ipairs(layer.children) do
    childLayer.normalIntensity = normalIntensity
    api.setLayer(childLayer, true)
  end
end

M.getLayerActions = getLayerActions

return M
