-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = "editor_dynamicDecals_colorHistory"
local im = ui_imgui
local docs = nil

-- reference to the editor tool, set in setup()
local tool = nil
-- reference to the dynamics decal api
local api = nil

local function addColorToHistory(color)
  local history = editor.getPreference("dynamicDecalsTool.colorHistory.history")
  -- Check whether there's a color in the history that matches this color and remove it
  for k, c in ipairs(history) do
    if c[1] == color.x and c[2] == color.y and c[3] == color.z and c[4] == color.w then
      table.remove(history, k)
    end
  end
  table.insert(history, 1, {color.x, color.y, color.z, color.w})
  local maxHistoryCount = editor.getPreference("dynamicDecalsTool.colorHistory.maxHistoryCount")
  if #history > maxHistoryCount then table.remove(history, maxHistoryCount + 1) end
  editor.setPreference("dynamicDecalsTool.colorHistory.history", history)
end

local function onGui(guiId)
  local data = editor.getPreference("dynamicDecalsTool.colorHistory.history")
  local uiIconSize = math.ceil(im.GetFontSize()) + 2 * im.GetStyle().FramePadding.y
  for k, color in ipairs(data) do
    im.ColorEdit4(string.format("##colorHistoryColorWidget_%s_%d", guiId, k), editor.getTempFloatArray4_TableTable(color), im.flags(im.ColorEditFlags_NoInputs, im.ColorEditFlags_AlphaPreview))
    im.SameLine()
    if im.Button(string.format("Set as decal color##colorHistory_%s_%d", guiId, k)) then
      api.setDecalColor(Point4F(color[1], color[2], color[3], color[4]))
    end
    im.SameLine()
    if im.Button(string.format("Set as fill layer color##colorHistory_%s_%d", guiId, k)) then
      api.setFillLayerColor(Point4F(color[1], color[2], color[3], color[4]))
    end
    im.SameLine()

    if editor.uiIconImageButton(editor.icons.delete, im.ImVec2(uiIconSize, uiIconSize), nil, nil, nil, string.format("remove##ColorHistory_%s_%d", guiId, k)) then
      table.remove(data, k)
      editor.setPreference("dynamicDecalsTool.colorHistory.history", data)
    end
    im.tooltip("Remove color from history")
  end
end

local function registerEditorPreferences(prefsRegistry)
  prefsRegistry:registerSubCategory("dynamicDecalsTool", "colorHistory", nil, {
    {history = {"table", {}, "", nil, nil, nil, nil, nil, function(cat, subCat, item)
      onGui("editorPreferences")
    end}},
    {maxHistoryCount = {"int", 10, "maxHistoryCount", nil, 2, 128}},
  })
end

local function editorPreferenceValueChanged(path, value)

end

local function documentationGui(docsSection)
  im.PushTextWrapPos(im.GetContentRegionAvailWidth())
  im.TextUnformatted([[
The Color History section keeps track of the colors you've used in your design.
Whenever you place a decal or add a fill layer, the color you used gets added to the history.

The colors in the history can then easily be used as decal or fill layer colors by a single button press. You can also drag and drop the color widgets onto any other color widget.
]])
  im.PopTextWrapPos()
end

local function setup(tool_in)
  tool = tool_in
  api = extensions.editor_api_dynamicDecals
  docs = extensions.editor_dynamicDecals_docs

  tool.registerSection("Color History", onGui, 122, false, {}, {
    {icon = editor.icons.help_outline, tooltip = "Docs", fn = function() docs.selectSection({"Color History"}) end},
  })
  docs.register({section = {"Color History"}, guiFn = documentationGui})
end

M.addColorToHistory = addColorToHistory
M.onGui = onGui
M.registerEditorPreferences = registerEditorPreferences
M.editorPreferenceValueChanged = editorPreferenceValueChanged
M.setup = setup

return M