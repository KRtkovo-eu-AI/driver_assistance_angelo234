-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

local api = extensions.editor_api_dynamicDecals
local uiTools = extensions.ui_liveryEditor_tools
local uiLayers = extensions.ui_liveryEditor_layers

M.duplicate = function()
  uiTools.doOperation(function(layer)
    local uiLayer = uiLayers.getLayerByUid(layer.uid)
    api.duplicateLayer(uiLayer.order, uiLayer.parentUid)
  end)
end

return M
