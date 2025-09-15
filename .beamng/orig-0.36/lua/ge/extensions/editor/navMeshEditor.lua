-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local toolWindowName = "navMeshEditor"
local editModeName = "Navigation Mesh"
local im = ui_imgui
local ffi = require('ffi')
local windows = {}
local currentWindow = {}


local function onEditorGui()
  if editor.beginWindow(toolWindowName, "Navigation Meshes", im.WindowFlags_MenuBar) then
    im.Text("Hello world!")
  end
  editor.endWindow()
end

local function show()
  editor.clearObjectSelection()
  editor.showWindow(toolWindowName)
  editor.selectEditMode(editor.editModes.navMeshEditMode)
end

local function onActivate()
end

local function onDeactivate()
end

local function onEditorInitialized()
  editor.registerWindow(toolWindowName, im.ImVec2(400, 400))
  editor.editModes.navMeshEditMode =
  {
    displayName = editModeName,
    onUpdate = nop,
    onActivate = onActivate,
    onDeactivate = onDeactivate,
    auxShortcuts = {},
  }
  editor.editModes.navMeshEditMode.auxShortcuts[editor.AuxControl_LMB] = "Select"
  editor.addWindowMenuItem("Navigation Mesh", function() show() end, {groupMenuName="Gameplay"})
end

local function onEditorToolWindowHide(windowName)
  if windowName == toolWindowName then
    editor.selectEditMode(editor.editModes.objectSelect)
  end
end

local function onWindowGotFocus(windowName)
  if windowName == toolWindowName then
    editor.selectEditMode(editor.editModes.navMeshEditMode)
  end
end


M.allowGizmo = function() return editor.editMode and editor.editMode.displayName == editModeName or false end

M.show = show
M.onEditorGui = onEditorGui
M.onEditorToolWindowHide = onEditorToolWindowHide
M.onEditorToolWindowGotFocus = onWindowGotFocus

M.onEditorInitialized = onEditorInitialized
M.onExtensionLoaded = onExtensionLoaded

return M