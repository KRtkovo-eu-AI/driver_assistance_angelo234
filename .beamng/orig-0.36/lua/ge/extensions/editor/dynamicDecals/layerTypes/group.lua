-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = "editor_dynamicDecals_layerTypes_group"
local im = ui_imgui

-- reference to the editor tool, set in setup()
local tool = nil
-- reference to the dynamics decal api
local api = nil
local inspector = nil
local docs = nil

local function inspectLayerGui(layer, guiId)

end

local function toolbarItemGui()
  if editor.uiIconImageButton(editor.icons.group_work, nil, nil, nil, nil, "Add group") then
    api.addGroup()
  end
  im.tooltip("Add group")
end

local function sectionGui(guiId)

end

local function registerEditorPreferences(prefsRegistry)
  -- prefsRegistry:registerSubCategory("dynamicDecalsTool", "moduleName", nil, {

  -- })
end

local function editorPreferenceValueChanged(path, value)

end

local function documentationGui(docsSection)
  im.PushTextWrapPos(im.GetContentRegionAvailWidth())
  im.TextUnformatted([[
Group Layers offer a streamlined approach to organizing your layer stack or project.

They act as simple layers that allow you to group other layers together, providing better structure and management.
Additionally, Group Layers can be disabled, effectively hiding all the child layers within the group, simplifying your workspace and enhancing focus on specific parts of your livery design.
]])
  im.PopTextWrapPos()
end

local function setup(tool_in)
  tool = tool_in
  api = extensions.editor_api_dynamicDecals
  inspector = extensions.editor_dynamicDecals_inspector
  docs = extensions.editor_dynamicDecals_docs

  -- tool.registerSection("Group Properties", sectionGui, 110, false, {})
  inspector.registerLayerGui(api.layerTypes.group, inspectLayerGui)
  tool.registerToolbarToolItem("group", toolbarItemGui, 30)
  docs.register({section = {"Layer Types", "Group Layers"}, guiFn = documentationGui})
end

M.onGui = sectionGui
M.registerEditorPreferences = registerEditorPreferences
M.editorPreferenceValueChanged = editorPreferenceValueChanged
M.setup = setup

return M