-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

local api = extensions.editor_api_dynamicDecals
local uiSelectionApi = extensions.ui_liveryEditor_selection
local uiControlsApi = extensions.ui_liveryEditor_controls
-- local uiEditMode = extensions.ui_liveryEditor_editMode

local TOOLS = {
  transform = "transform",
  deform = "deform",
  material = "material",
  mirror = "mirror"
}

local currentTool = nil
local editModeState = {
  active = nil,
  reapplyActive = nil,
  applyActive = nil,
  activeLayer = nil
}

M.useTool = function(tool)
  currentTool = tool
  uiControlsApi.useActionMap(tool)
  guihooks.trigger("LiveryEditorToolChanged", currentTool)
end

M.closeCurrentTool = function()
  currentTool = nil
  uiControlsApi.disableAllActionMaps()
  guihooks.trigger("LiveryEditorToolChanged", currentTool)
end

M.getCurrentTool = function()
  return currentTool
end

M.applyChanges = function()
  api.addDecal()
end

-- Common Operation function for all tools
M.doOperation = function(funcOperation, ...)
  dump("doOperation editModeState", M.editModeState)
  if M.editModeState.active then
    if M.editModeState.reapplyActive or M.editModeState.applyActive then
      funcOperation(nil, ...)
    else
      -- local activeLayer = editModeState.activeLayerUid
      local layer = M.editModeState.activeLayerUid and api.getLayerByUid(M.editModeState.activeLayerUid)
      funcOperation(layer, ...)
    end
    -- elseif not selectedLayerUids or #selectedLayerUids < 1 then
    -- log("W", "", "Attempting to translate with no selected layers")
    -- return
    -- funcOperation(nil, ...)
  else
    local selectedLayerUids = shallowcopy(uiSelectionApi.getSelectedLayers())
    if selectedLayerUids then
      for key, layerUid in ipairs(selectedLayerUids) do
        local layer = api.getLayerByUid(layerUid)

        if not layer then
          log("W", "", "Unable to find layer " .. layerUid .. " . Skipping...")
          return
        end

        funcOperation(layer, ...)
      end
    end
  end
end

M.TOOLS = TOOLS
M.editModeState = editModeState

M.liveryEditor_editMode_onStateChanged = function(data)
  dump("liveryEditor_editMode_onStateChanged", data)
  for k, v in pairs(data) do
    M.editModeState[k] = v
  end
end

return M
