-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local api = extensions.editor_api_dynamicDecals

M.undo = function()
  api.undo()
end

M.redo = function()
  api.redo()
end

-- External hooks. Do not call!
M.dynamicDecals_onLayerAdded = function(layerUid)
  guihooks.trigger("LiveryEditor_onHistoryUpdated", api.getHistory())
end

M.dynamicDecals_onLayerDeleted = function(layerUid)
  guihooks.trigger("LiveryEditor_onHistoryUpdated", api.getHistory())
end

M.dynamicDecals_onLayerUpdated = function(layerUid)
  guihooks.trigger("LiveryEditor_onHistoryUpdated", api.getHistory())
end

M.dynamicDecals_moveLayer = function(from, fromParentUid, to, toParentUid)
  guihooks.trigger("LiveryEditor_onHistoryUpdated", api.getHistory())
end

return M
