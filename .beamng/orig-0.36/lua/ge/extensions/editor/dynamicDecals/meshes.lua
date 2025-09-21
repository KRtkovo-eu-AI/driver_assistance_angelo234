-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = "editor_dynamicDecals_meshes"
local im = ui_imgui

-- reference to the editor tool, set in setup()
local tool = nil
-- reference to the dynamics decal api
local api = nil
local settings = nil

local function setShapePath(path)
  api.setShapePath(path)
  settings.updateMaterials()
end

local function sectionGui(guiId)
  im.TextUnformatted("Shape Path")
  im.PushItemWidth(im.GetContentRegionAvailWidth() - tool.getIconSize() - im.GetStyle().ItemSpacing.x)
  im.InputText("##Shape Path", editor.getTempCharPtr(api.getShapePath()), nil, im.InputTextFlags_ReadOnly)
  im.PopItemWidth()
  im.SameLine()
  if editor.uiIconImageButton(editor.icons.folder, im.ImVec2(tool.getIconSize(), tool.getIconSize())) then
    editor_fileDialog.openFile(
      function(data)
        setShapePath(data.filepath)
      end,
      {{"Any files", "*"},{"DAE files",".dae"}},
      false,
      path.split(api.getShapepath()) or "/",
      true
    )
  end
  im.tooltip("Change Shape Path")
end

local function registerEditorPreferences(prefsRegistry)
  -- prefsRegistry:registerSubCategory("dynamicDecalsTool", "moduleName", nil, {

  -- })
end

local function editorPreferenceValueChanged(path, value)

end

local function setup(tool_in)
  tool = tool_in
  api = extensions.editor_api_dynamicDecals
  settings = extensions.editor_dynamicDecals_settings

  -- tool.registerSection("Meshes (Experimental)", sectionGui, 1050, false, {})
end

M.registerEditorPreferences = registerEditorPreferences
M.editorPreferenceValueChanged = editorPreferenceValueChanged
M.setup = setup

return M