-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

local api = extensions.editor_api_dynamicDecals
local uiTools = extensions.ui_liveryEditor_tools
local uiLayerGroup = extensions.ui_liveryEditor_layers_group
local uiCursorApi = extensions.ui_liveryEditor_layers_cursor
local uiDecals = extensions.ui_liveryEditor_layers_decals

M.setColor = function(rgbaArray)
  dump("setColor", rgbaArray)
  uiTools.doOperation(function(layer, color)
    if not layer then
      uiCursorApi.setColor(color)
    elseif layer.type == api.layerTypes.decal then
      uiDecals.setColor(layer, color)
    elseif layer.type == api.layerTypes.linkedSet then
      uiLayerGroup.setColor(layer, color)
    end
  end, rgbaArray)
end

M.setMetallicIntensity = function(metallicIntensity)
  dump("setMetallicIntensity", metallicIntensity)
  uiTools.doOperation(function(layer, metallicIntensity)
    if metallicIntensity and metallicIntensity >= 0 or metallicIntensity <= 1 then
      if not layer then
        uiCursorApi.setMetallicIntensity(metallicIntensity)
      elseif layer.type == api.layerTypes.group then
        uiLayerGroup.setMetallicIntensity(layer, metallicIntensity)
      elseif layer.type == api.layerTypes.decal then
        uiDecals.setMetallicIntensity(layer, metallicIntensity)
      end
    else
      log("W", "", "Metallic intensity is not valid " .. (metallicIntensity or "nil"))
    end
  end, metallicIntensity)
end

M.setRoughnessIntensity = function(roughnessIntensity)
  dump("seRoughnessIntensity", roughnessIntensity)
  uiTools.doOperation(function(layer, metallicIntensity)
    if roughnessIntensity and roughnessIntensity >= 0 or roughnessIntensity <= 1 then
      if not layer then
        uiCursorApi.setRoughnessIntensity(metallicIntensity)
      elseif layer.type == api.layerTypes.group then
        uiLayerGroup.setRoughnessIntensity(layer, roughnessIntensity)
      elseif layer.type == api.layerTypes.decal then
        uiDecals.setRoughnessIntensity(layer, roughnessIntensity)
      end
    else
      log("W", "", "Roughness intensity is not valid " .. (metallicIntensity or "nil"))
    end
  end, roughnessIntensity)
end

M.setNormalIntensity = function(normalIntensity)
  dump("setNormalIntensity", normalIntensity)
  uiTools.doOperation(function(layer, normalIntensity)
    if normalIntensity and normalIntensity >= 0 or normalIntensity <= 1 then
      if layer.type == api.layerTypes.group then
        uiLayerGroup.setRoughnessIntensity(layer, normalIntensity)
      else
        layer.normalIntensity = normalIntensity
        api.setLayer(layer, true)
      end
    else
      log("W", "", "Normal intensity is not valid " .. (normalIntensity or "nil"))
    end
  end, normalIntensity)
end

M.setDecal = function(decalTexture)
  uiTools.doOperation(function(layer, decalTexture)
    if not layer then
      uiCursorApi.setDecal(decalTexture)
    elseif layer.type == api.layerTypes.decal then
      uiDecals.setDecal(layer, decalTexture)
    end
  end, decalTexture)
end

return M
