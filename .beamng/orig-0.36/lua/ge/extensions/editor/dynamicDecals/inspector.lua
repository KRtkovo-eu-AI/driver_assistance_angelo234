-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = "editor_dynamicDecals_inspector"
local im = ui_imgui

-- reference to the editor tool, set in setup()
local tool = nil
-- reference to the dynamics decal api
local api = nil
local brushes = nil
local docs = nil
local widgets = nil
local helper = nil

local layerGui = {}

local function inspectLayerGui(layer, guiId)
  if im.Button(string.format("Dump##%s_%s", layer.uid, guiId)) then dump(layer) end
  if editor.getPreference("dynamicDecalsTool.general.debug") then
    im.SameLine()
    if im.Button(string.format("Dumpz 3##%s_%s", layer.uid, guiId)) then dumpz(layer, 3) end
  end
  im.Separator()

  local widgetId = string.format("%s_%s", layer.uid, guiId)
  im.TextUnformatted("uid")
  im.SameLine()
  im.PushItemWidth(im.GetContentRegionAvailWidth())
  im.InputText(string.format("##uid_%s", widgetId), editor.getTempCharPtr(layer.uid), nil, im.InputTextFlags_ReadOnly)
  im.tooltip("Read-only type")
  im.PopItemWidth()

  if widgets.draw(layer.name, api.propertiesMap["name"], widgetId, editor.getTempBool_BoolBool(false)) then
    layer.name = api.propertiesMap["name"].value
  end
  if editor.getTempBool_BoolBool() == true then
    layer.name = api.propertiesMap["name"].value
    api.setLayer(layer, false)
  end

  im.TextUnformatted("Type")
  im.SameLine()
  im.PushItemWidth(im.GetContentRegionAvailWidth())
  im.InputText(string.format("##type_%s", widgetId), editor.getTempCharPtr(string.format("%s Layer", helper.splitAndCapitalizeCamelCase(api.layerTypesMap[layer.type]))), nil, im.InputTextFlags_ReadOnly)
  im.tooltip("Read-only type")
  im.PopItemWidth()

  if widgets.draw(layer.enabled, api.propertiesMap["enabled"], widgetId, editor.getTempBool_BoolBool(false)) then
    layer.enabled = api.propertiesMap["enabled"].value
  end
  if editor.getTempBool_BoolBool() == true then
    layer.enabled = api.propertiesMap["enabled"].value
    api.setLayer(layer, true)
  end

  im.Separator()

  layerGui[layer.type](layer, guiId)
end

local function inspectorGuiLayer(inspectorInfo)
  local selection = editor.selection["dynamicDecalLayer"]
  if selection == nil then return end

  local isMultiSelect = function(sel)
    local i = 1
    for _, v in pairs(sel) do
      if i > 1 then return true end
      i = i + 1
    end
    return false
  end

  local multiSelect = isMultiSelect(selection)

  for layerUid, layerData in pairs(editor.selection["dynamicDecalLayer"]) do
    if multiSelect then
      if im.TreeNode1(string.format("%s - %s", layerData.uid, layerData.name)) then
        inspectLayerGui(layerData, "inspector")
        im.TreePop()
      end
    else
      inspectLayerGui(layerData, "inspector")
    end
  end
end

local function inspectorGuiBrush(inspectorInfo)
  if editor.selection["dynamicDecalBrush"] ~= nil then
    local brushData = editor.selection["dynamicDecalBrush"]
    brushes.inspectorGui(brushData)
  end
end

local function registerEditorPreferences(prefsRegistry)
  prefsRegistry:registerSubCategory("dynamicDecalsTool", "inspector", nil, {
    {texturePreviewSize = {"float", 128, "Max width of the decal texture thumbnails.", nil, 32, 512}},
  })
end

local function editorPreferenceValueChanged(path, value)

end

local function documentationGui(docsSection)
  im.PushTextWrapPos(im.GetContentRegionAvailWidth())
  im.TextUnformatted([[
The Inspector is a powerful tool that enables you to access and modify all the properties of the selected layers.
This user-friendly interface displays comprehensive information about the layer, and every property is fully editable.
With the Inspector, you have complete control over fine-tuning each layer's attributes, allowing you to perfect your design with ease.
]])
  im.PopTextWrapPos()
end

local function setup(tool_in)
  tool = tool_in
  api = extensions.editor_api_dynamicDecals
  brushes = extensions.editor_dynamicDecals_brushes
  docs = extensions.editor_dynamicDecals_docs
  widgets = extensions.editor_dynamicDecals_widgets
  helper = extensions.editor_dynamicDecals_helper

  for layerType, layerTypeId in pairs(api.layerTypes) do
    layerGui[layerTypeId] = function() im.TextUnformatted(string.format("No layerGui available for '%s'", layerType)) end
  end

  docs.register({section = {"Inspector"}, guiFn = documentationGui})
end

local function registerLayerGui(layerTypeId, guiFn)
  layerGui[layerTypeId] = guiFn
end

M.registerLayerGui = registerLayerGui
M.inspectorGuiLayer = inspectorGuiLayer
M.inspectorGuiBrush = inspectorGuiBrush
M.registerEditorPreferences = registerEditorPreferences
M.editorPreferenceValueChanged = editorPreferenceValueChanged
M.setup = setup

return M