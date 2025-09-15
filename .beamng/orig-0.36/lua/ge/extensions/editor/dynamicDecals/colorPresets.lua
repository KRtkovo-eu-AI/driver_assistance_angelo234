-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = "editor_dynamicDecals_colorPresets"
local im = ui_imgui

-- reference to the editor tool, set in setup()
local tool = nil
-- reference to the dynamics decal api
local api = nil
local docs = nil

local searchFilter = im.ImGuiTextFilter()

local newPreset = {
  name = "new color preset",
  value = {255/255,102/255,0,255/255}
}

local function checkFilter(color)
  if not color.name then return true end
  if im.ImGuiTextFilter_PassFilter(searchFilter, color.name) then
    return true
  end
  return false
end

local function onGui(guiId)
  editor.uiInputSearchTextFilter("Texture Filter", searchFilter, im.GetContentRegionAvailWidth())
  im.Separator()

  local data = editor.getPreference("dynamicDecalsTool.colorPresets.presets")
  local uiIconSize = (math.ceil(im.GetFontSize()) + 2 * im.GetStyle().FramePadding.y) * im.uiscale[0]
  for k, color in ipairs(data) do
    if checkFilter(color) then

      if im.ColorEdit4(string.format("##colorPresetColorWidget_%s_%d", guiId, k), editor.getTempFloatArray4_TableTable(color.value), im.flags(im.ColorEditFlags_NoInputs, im.ColorEditFlags_AlphaPreview)) then
        data[k].value = editor.getTempFloatArray4_TableTable()
        editor.setPreference("dynamicDecalsTool.colorPresets.presets", data)
      end

      im.PushItemWidth(im.GetContentRegionAvailWidth() - (4 * uiIconSize + 5 * im.GetStyle().ItemSpacing.x * im.uiscale[0]) - uiIconSize/2) -- nice hack so buttons are visible when uiscale is above 1.2
      im.SameLine()
      if editor.uiInputText(string.format("##colorPresetColorNameWidget_%s_%d", guiId, k), editor.getTempCharPtr(color.name or ""), nil, nil, nil, nil, editor.getTempBool_BoolBool(false)) then
        color.name = editor.getTempCharPtr()
      end
      if editor.getTempBool_BoolBool() then
        color.name = editor.getTempCharPtr()
        editor.setPreference("dynamicDecalsTool.colorPresets.presets", data)
      end
      im.PopItemWidth()

      im.SameLine()
      if editor.uiIconImageButton(editor.icons.delete, im.ImVec2(uiIconSize, uiIconSize), nil, nil, nil, string.format("remove##ColorPreset_%s_%d", guiId, k)) then
        table.remove(data, k)
        editor.setPreference("dynamicDecalsTool.colorPresets.presets", data)
      end
      im.tooltip("Remove color preset")

      im.SameLine()
      if editor.uiIconImageButton(editor.icons.border_color, im.ImVec2(uiIconSize, uiIconSize), nil, nil, nil, string.format("setAsDecalColorIconButton##ColorPreset_%s_%d", guiId, k)) then
        api.setDecalColor(Point4F(color.value[1], color.value[2], color.value[3], color.value[4]))
      end
      im.tooltip("Set color preset as decal color")

      im.SameLine()
      if editor.uiIconImageButton(editor.icons.format_color_fill, im.ImVec2(uiIconSize, uiIconSize), nil, nil, nil, string.format("setAsFillColorIconButton##ColorPreset_%s_%d", guiId, k)) then
        api.setFillLayerColor(Point4F(color.value[1], color.value[2], color.value[3], color.value[4]))
      end
      im.tooltip("Set color preset as fill color")
    end
  end

  if im.Button(string.format("Add color preset##colorPreset_%s", guiId), im.ImVec2(im.GetContentRegionAvailWidth(), 0)) then
    table.insert(data, {value = newPreset.value})
    editor.setPreference("dynamicDecalsTool.colorPresets.presets", data)
  end

  -- New color preset using a color picker widget
  -- local spacing = im.ImVec2(0, im.GetStyle().ItemSpacing.y * 2)
  -- im.Dummy(spacing)
  -- im.Separator()
  -- im.Separator()
  -- im.Dummy(spacing)

  -- if im.ColorPicker4(string.format("##NewPresetColor"), editor.getTempFloatArray4_TableTable(newPreset.value), nil, nil) then
  --   newPreset.value = editor.getTempFloatArray4_TableTable()
  -- end

  -- im.Dummy(spacing)

  -- if im.Button(string.format("Add color preset##colorPreset_%s", guiId), im.ImVec2(im.GetContentRegionAvailWidth(), 0)) then
  --   table.insert(data, {value = newPreset.value})
  --   editor.setPreference("dynamicDecalsTool.colorPresets.presets", data)
  -- end

end

local function registerEditorPreferences(prefsRegistry)
  prefsRegistry:registerSubCategory("dynamicDecalsTool", "colorPresets", nil, {
    {presets = {"table", {
        {name = "iconic oraNGe", value = {1, 0.4, 0, 1.0}},
        {name = "Aquamarine", value = {0.05,0.5,0.7,1.0}},
        {name = "Army Green", value = {0.275,0.302,0.25,1.0}},
        {name = "Autumn Yellow", value = {0.521,0.396,0.189,1.0}},
        {name = "Bermuda Blue", value = {0.08,0.2,0.24,1.0}},
        {name = "Brilliant Blue", value = {0.07,0.37,0.72,1.0}},
        {name = "Burgundy", value = {0.5,0,0,1.0}},
        {name = "Butterscotch", value = {0.81,0.638,0.476,1.0}},
        {name = "Carbon Gray", value = {0.33,0.33,0.33,1.0}},
        {name = "Champagne", value = {0.663,0.594,0.533,1.0}},
        {name = "Charcoal", value = {0.3,0.3,0.3,1.0}},
        {name = "Chestnut", value = {0.3,0.2,0.12,1.0}},
        {name = "Chocolate Brown", value = {0.488,0.316,0.192,1.0}},
        {name = "Clay Red", value = {0.45,0.28,0.233,1.0}},
        {name = "Cream", value = {0.79,0.75,0.69,1.0}},
        {name = "Deep Plum", value = {0.31,0,0.03,1.0}},
        {name = "Dusted Mica", value = {0.5,0.42,0.39,1.0}},
        {name = "Fall Gold", value = {0.58,0.43,0.1,1.0}},
        {name = "Fire Red", value = {0.8,0.1,0.1,1.0}},
        {name = "Flame Orange", value = {0.9,0.4,0,1.0}},
        {name = "Forest Green", value = {0.08,0.18,0.105,1.0}},
        {name = "Gray", value = {0.5,0.5,0.5,1.0}},
        {name = "Jet Black", value = {0,0,0,1.0}},
        {name = "Light Brown", value = {0.525,0.41,0.3,1.0}},
        {name = "Limoncello", value = {0.87,0.8,0.6,1.0}},
        {name = "Navy Blue", value = {0,0.07,0.23,1.0}},
        {name = "Olive Green", value = {0.27,0.35,0.23,1.0}},
        {name = "Opal Green", value = {0.22,0.37,0.33,1.0}},
        {name = "Pearl White", value = {0.83,0.83,0.83,1.0}},
        {name = "Pleasant Blue", value = {0.35,0.5,0.65,1.0}},
        {name = "Quicksilver", value = {0.52,0.485,0.46,1.0}},
        {name = "Royal Blue", value = {0,0.1,0.42,1.0}},
        {name = "Scarlet Red", value = {0.58,0.12,0.12,1.0}},
        {name = "Sea Blue", value = {0.09,0.21,0.47,1.0}},
        {name = "Seafoam Green", value = {0.24,0.48,0.48,1.0}},
        {name = "Silver", value = {0.65,0.65,0.65,1.0}},
        {name = "Solar Yellow", value = {0.82,0.62,0.1,1.0}},
        {name = "Sunset Purple", value = {0.24,0.25,0.75,1.0}},
        {name = "Verdant Green", value = {0.18,0.36,0.09,1.0}},
        {name = "Vibrant Red", value = {0.72,0.13,0.13,1.0}},
        {name = "Toxic Green", value = {0,0.73,0.23,1.0}},
        {name = "Furious Orange", value = {0.936,0.416,0,1.0}}
      }, "", nil, nil, nil, nil, nil, function(cat, subCat, item)
      onGui("editorPreferences")
    end}},
  })
end

local function editorPreferenceValueChanged(path, value)

end

local function documentationGui(docsSection)
  im.PushTextWrapPos(im.GetContentRegionAvailWidth())
  im.TextUnformatted([[
The Color Presets section comes with a number of predefined colors which you can use to enhance your design.

Tailor the presets to your liking, adjusting hues, saturations, and brightness to achieve the perfect balance.
Have all your presets in one place so you can reuse them for all your designs.

The colors can be used as decal or fill layer colors by a single button press. You can also drag and drop the color widgets onto any other color widget.
]])
  im.PopTextWrapPos()
end

local function setup(tool_in)
  tool = tool_in
  api = extensions.editor_api_dynamicDecals
  docs = extensions.editor_dynamicDecals_docs

  local docsTitle = "Color Presets"
  tool.registerSection("Color Presets", onGui, 121, false, {}, {
    {icon = editor.icons.help_outline, tooltip = "Docs", fn = function() docs.selectSection(docsTitle) end},
  })
  docs.register({section = {docsTitle}, guiFn = documentationGui})
end

M.onGui = onGui
M.registerEditorPreferences = registerEditorPreferences
M.editorPreferenceValueChanged = editorPreferenceValueChanged
M.setup = setup

return M