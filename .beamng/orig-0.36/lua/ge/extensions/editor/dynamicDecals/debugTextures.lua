-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = "editor_dynamicDecals_debugTextures"
local im = ui_imgui

-- reference to the editor tool, set in setup()
local tool = nil
local api = nil

local textureSet = nil

local function sectionGui(guiId)
  local maxImageWidgetWidth = editor.getPreference("dynamicDecalsTool.debugTextures.maxImageWidgetWidth")

  -- if not textureSet then textureSet = api.getTextureSet() end
  -- print("debug:sectionGui")
  if textureSet then
    if im.CollapsingHeader1("Combined Textures") then
      api.drawTextureSet(textureSet, "Combined Textures", maxImageWidgetWidth)
    end
  end
  if im.CollapsingHeader1("Dynamic Textures") then
    api.drawDynamicTextures(maxImageWidgetWidth)
  end
  if im.CollapsingHeader1("Baked Textures") then
    api.drawBakedTextures(maxImageWidgetWidth)
  end
  if im.CollapsingHeader1("Highlight Textures") then
    api.drawHighlightTextures(maxImageWidgetWidth)
  end
  if im.CollapsingHeader1("Decal Textures") then
    api.drawBrushInputTextures(maxImageWidgetWidth)
  end
  if im.CollapsingHeader1("Mask Textures") then
    api.drawMaskTextures(maxImageWidgetWidth)
  end
end

local function registerEditorPreferences(prefsRegistry)
  prefsRegistry:registerSubCategory("dynamicDecalsTool", "debugTextures", nil, {
    {maxImageWidgetWidth = {"float", 512, "Maximum width of the image widgets in the Debug Textures section."}},
  })
end

local function editorPreferenceValueChanged(path, value)

end

local function setup(tool_in)
  tool = tool_in
  api = extensions.editor_api_dynamicDecals

  tool.registerSection("Debug Textures", sectionGui, 1130, false, {})
end

M.registerEditorPreferences = registerEditorPreferences
M.editorPreferenceValueChanged = editorPreferenceValueChanged
M.setup = setup

return M