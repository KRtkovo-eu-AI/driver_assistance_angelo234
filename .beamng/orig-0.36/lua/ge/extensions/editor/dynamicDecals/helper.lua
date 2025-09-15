-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = "editor_dynamicDecals_helper"
local im = ui_imgui

-- reference to the editor tool, set in setup()
local tool = nil

local function textUnformattedCentered(string_label)
  if not string_label then return end
  im.SetCursorPosX(im.GetCursorPosX() + im.GetContentRegionAvailWidth() / 2 - im.CalcTextSize(string_label).x / 2)
  im.TextUnformatted(string_label)
end

local function textColoredCentered(color, string_label)
  if not string_label then return end
  im.SetCursorPosX(im.GetCursorPosX() + im.GetContentRegionAvailWidth() / 2 - im.CalcTextSize(string_label).x / 2)
  im.TextColored(color, string_label)
end

local function imageWidget(path, overrideWidth)
  local img = editor.getTempTextureObj(path)
  local imgWidth = overrideWidth or im.GetContentRegionAvailWidth()
  local imgHeight = img.path == "/" and imgWidth or imgWidth * img.size.y / img.size.x
  im.Image(img.texId, im.ImVec2(imgWidth, imgHeight), im.ImVec2(0,0), im.ImVec2(1,1), nil, editor.color.beamng.Value)
  im.TextUnformatted(string.format("%d x %d", img.size.x, img.size.y))
end

local function imageTooltip(path, overrideTooltipWidth)
  if im.IsItemHovered() then
    im.BeginTooltip()
    local img = editor.getTempTextureObj(path)
    local imgWidth = overrideTooltipWidth or editor.getPreference("dynamicDecalsTool.imageTooltip.texturePreviewSize")
    local imgHeight = img.path == "/" and imgWidth or imgWidth * img.size.y / img.size.x
    im.Image(img.texId, im.ImVec2(imgWidth, imgHeight), im.ImVec2(0,0), im.ImVec2(1,1), nil, editor.color.beamng.Value)
    im.TextUnformatted(string.format("%d x %d", img.size.x, img.size.y))
    im.EndTooltip()
  end
end

local function iconTooltip(msg, inline)
  if inline then
    im.SameLine()
  end
  editor.uiIconImage(editor.icons.info_outline, im.ImVec2(tool.getIconSize(), tool.getIconSize()))
  im.tooltip(msg)
end

local function splitAndCapitalizeCamelCase(str)
  return str:gsub("%u%l+", function(match)
    return " " .. match
  end):gsub("(%w)(%w*)", function(first, rest)
    return first:upper() .. rest:lower()
  end)
end

local function capitalizeWords(str)
  return str:gsub("(%w)(%w*)", function(first, rest)
    return first:upper() .. rest:lower()
  end)
end

local function registerEditorPreferences(prefsRegistry)
  prefsRegistry:registerSubCategory("dynamicDecalsTool", "imageTooltip", nil, {
    {texturePreviewSize = {"float", 128, "Max width of the decal texture thumbnails.", nil, 32, 512}},
  })
end

local function editorPreferenceValueChanged(path, value)

end

local function setup(tool_in)
  tool = tool_in
end

M.textUnformattedCentered = textUnformattedCentered
M.textColoredCentered = textColoredCentered
M.imageWidget = imageWidget
M.imageTooltip = imageTooltip
M.iconTooltip = iconTooltip
M.splitAndCapitalizeCamelCase = splitAndCapitalizeCamelCase
M.capitalizeWords = capitalizeWords
M.registerEditorPreferences = registerEditorPreferences
M.editorPreferenceValueChanged = editorPreferenceValueChanged
M.setup = setup

return M