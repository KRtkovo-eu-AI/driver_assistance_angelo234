-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = 'editor_extension_measuresInspectorHeader'
local imgui = ui_imgui
local enabled = false

local function onEditorInspectorHeaderGui(inspectorInfo)
  if not enabled then return end
  local selection

  if inspectorInfo.selection then
    selection = inspectorInfo.selection
  else
    selection = editor.selection
  end

  if not selection.object then return end

  local size

  if tableSize(selection.object) == 1 then
    local obj = scenetree.findObjectById(selection.object[1])
    local localBBox

    if obj and obj.getObjBox and obj.getScale then
      localBBox = obj:getObjBox()
      local objScale = obj:getScale()
      localBBox:scale3F(objScale)
    end

    if localBBox then
      size = localBBox:getExtents()
    end
  else
    if editor.objectSelectionBBox then
      size = editor.objectSelectionBBox:getExtents()
    end
  end

  if size then
    imgui.Text("Size: " .. string.format("%.2f", size.x) .. " x " .. string.format("%.2f", size.y) .. " x " .. string.format("%.2f", size.z))
    imgui.tooltip("The size of the selection bounding box, scale included")
  end
end

local function onExtensionLoaded()
end

local function onEditorInitialized()
  enabled = true
end

M.onEditorInitialized = onEditorInitialized
M.onExtensionLoaded = onExtensionLoaded
M.onEditorInspectorHeaderGui = onEditorInspectorHeaderGui

return M