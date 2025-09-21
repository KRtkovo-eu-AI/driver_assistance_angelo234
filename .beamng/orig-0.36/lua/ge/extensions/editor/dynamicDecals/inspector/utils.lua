-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.dependencies = {
  "editor_api_dynamicDecals"
}
local logTag = "editor_dynamicDecals_inspector_utils"
local im = ui_imgui

-- reference to the editor tool, set in setup()
local tool = nil
-- reference to the dynamics decal api
local api = nil

local function decalTextureWidgetInspect(layer, property, guiId, removeTextureOverridePath)
  im.PushItemWidth(im.GetContentRegionAvailWidth() - 2 * tool.getIconSize() - 2 * im.GetStyle().ItemSpacing.x)
  im.InputText(string.format("##%s_%s_%s_%s", layer.uid, guiId, propert, "texturePath"), editor.getTempCharPtr(layer[property]), nil, im.InputTextFlags_ReadOnly)
  im.PopItemWidth()
  im.SameLine()
  if editor.uiIconImageButton(editor.icons.folder, im.ImVec2(tool.getIconSize(), tool.getIconSize()), nil, nil, nil, string.format("##%s_%s_%s_%s", layer.uid, guiId, propert, "button")) then
    editor_fileDialog.openFile(
      function(data)
        layer[property] = data.filepath
        api.setLayer(layer, true)
      end,
      {{"Any files", "*"},{"PNG files",".png"},{"Image files",{".png", ".jpg", ".jpeg"}}},
      false,
      path.split(layer[property]) or "/art/decals/dynDecals/",
      true
    )
  end
  im.SameLine()
  if editor.uiIconImageButton(editor.icons.delete, im.ImVec2(tool.getIconSize(), tool.getIconSize()), nil, nil, nil, string.format("##%s_%s_%s_%s", layer.uid, guiId, propert, "removeButton")) then
    layer[property] = removeTextureOverridePath or "/"
    api.setLayer(layer, true)
  end
  local img = editor.getTempTextureObj(layer[property])
  local imgWidthSetting = editor.getPreference("dynamicDecalsTool.inspector.texturePreviewSize")
  local imgWidth = imgWidthSetting > im.GetContentRegionAvailWidth() and im.GetContentRegionAvailWidth() or imgWidthSetting
  local imgHeight = img.path == "/" and imgWidth or imgWidth * img.size.y / img.size.x
  im.Image(img.texId, im.ImVec2(imgWidth, imgHeight), im.ImVec2(0,0), im.ImVec2(1,1), nil, editor.color.beamng.Value)
  if im.BeginDragDropTarget() then
    local payload = im.AcceptDragDropPayload("DynDecalTextureDrapDrop")
    if payload~=nil then
      assert(payload.DataSize == 256)
      local path = ffi.string(payload.Data)
      layer[property] = path
      api.setLayer(layer, true)
    end
    im.EndDragDropTarget()
  end
end

local function decalColorGradientWidgetInspect(k, layer, guiId)
  local gradientColorTopLeft = {layer.decalGradientColorTopLeft.r/255, layer.decalGradientColorTopLeft.g/255, layer.decalGradientColorTopLeft.b/255, layer.decalGradientColorTopLeft.a/255}
  local gradientColorTopRight = {layer.decalGradientColorTopRight.r/255, layer.decalGradientColorTopRight.g/255, layer.decalGradientColorTopRight.b/255, layer.decalGradientColorTopRight.a/255}
  local gradientColorBottomLeft = {layer.decalGradientColorBottomLeft.r/255, layer.decalGradientColorBottomLeft.g/255, layer.decalGradientColorBottomLeft.b/255, layer.decalGradientColorBottomLeft.a/255}
  local gradientColorBottomRight = {layer.decalGradientColorBottomRight.r/255, layer.decalGradientColorBottomRight.g/255, layer.decalGradientColorBottomRight.b/255, layer.decalGradientColorBottomRight.a/255}
  local gradientColorTopLeftU32 = im.GetColorU322(editor.getTempImVec4_TableTable(gradientColorTopLeft))
  local gradientColorTopRightU32 = im.GetColorU322(editor.getTempImVec4_TableTable(gradientColorTopRight))
  local gradientColorBottomLeftU32 = im.GetColorU322(editor.getTempImVec4_TableTable(gradientColorBottomLeft))
  local gradientColorBottomRightU32 = im.GetColorU322(editor.getTempImVec4_TableTable(gradientColorBottomRight))

  local size = im.GetContentRegionAvailWidth() / 2 > 256 and 256 or im.GetContentRegionAvailWidth() / 2

  local cursorPos = im.GetCursorPos()
  im.SetCursorPos(im.ImVec2(cursorPos.x, cursorPos.y + size - 20))
  if editor.uiColorEdit4(string.format("##%s_%s_%s", layer.uid, guiId, "gradientColorBottomLeft"), editor.getTempFloatArray4_TableTable(gradientColorBottomLeft), im.flags(im.ColorEditFlags_AlphaPreview, im.ColorEditFlags_NoInputs), editor.getTempBool_BoolBool(false)) then
    local value = editor.getTempFloatArray4_TableTable()
    layer.decalGradientColorBottomLeft = ColorI(value[1] * 255, value[2] * 255, value[3] * 255, value[4] * 255)
  end
  if editor.getTempBool_BoolBool() then
    api.setLayer(layer, true)
  end

  im.SetCursorPos(cursorPos)
  if editor.uiColorEdit4(string.format("##%s_%s_%s", layer.uid, guiId, "gradientColorTopLeft"), editor.getTempFloatArray4_TableTable(gradientColorTopLeft), im.flags(im.ColorEditFlags_AlphaPreview, im.ColorEditFlags_NoInputs), editor.getTempBool_BoolBool(false)) then
    local value = editor.getTempFloatArray4_TableTable()
    layer.decalGradientColorTopLeft = ColorI(value[1] * 255, value[2] * 255, value[3] * 255, value[4] * 255)
  end
  if editor.getTempBool_BoolBool() then
    api.setLayer(layer, true)
  end
  im.SameLine()
  local windowPos = im.GetWindowPos()
  cursorPos = im.GetCursorPos()
  local scrollPosX = im.GetScrollX()
  local scrollPosY = im.GetScrollY()
  im.ImDrawList_AddRectFilledMultiColor(
    im.GetWindowDrawList(),
    im.ImVec2(windowPos.x + cursorPos.x - scrollPosX, windowPos.y + cursorPos.y - scrollPosY),
    im.ImVec2(windowPos.x + cursorPos.x + size - scrollPosX, windowPos.y + cursorPos.y + size - scrollPosY),
    gradientColorTopLeftU32,
    gradientColorTopRightU32,
    gradientColorBottomRightU32,
    gradientColorBottomLeftU32
  )
  -- adding an invisible button so the imgui cursor is at the right location
  im.InvisibleButton("GradientButton", im.ImVec2(size, size))
  im.SameLine()

  cursorPos = im.GetCursorPos()
  im.SetCursorPos(im.ImVec2(cursorPos.x, cursorPos.y + size - 20))
  if editor.uiColorEdit4(string.format("##%s_%s_%s", layer.uid, guiId, "gradientColorBottomRight"), editor.getTempFloatArray4_TableTable(gradientColorBottomRight), im.flags(im.ColorEditFlags_AlphaPreview, im.ColorEditFlags_NoInputs), editor.getTempBool_BoolBool(false)) then
    local value = editor.getTempFloatArray4_TableTable()
    layer.decalGradientColorBottomRight = ColorI(value[1] * 255, value[2] * 255, value[3] * 255, value[4] * 255)
  end
  if editor.getTempBool_BoolBool() then
    api.setLayer(layer, true)
  end

  im.SetCursorPos(cursorPos)
  if editor.uiColorEdit4(string.format("##%s_%s_%s", layer.uid, guiId, "gradientColorTopRight"), editor.getTempFloatArray4_TableTable(gradientColorTopRight), im.flags(im.ColorEditFlags_AlphaPreview, im.ColorEditFlags_NoInputs), editor.getTempBool_BoolBool(false)) then
    local value = editor.getTempFloatArray4_TableTable()
    layer.decalGradientColorTopRight = ColorI(value[1] * 255, value[2] * 255, value[3] * 255, value[4] * 255)
  end
  if editor.getTempBool_BoolBool() then
    api.setLayer(layer, true)
  end
  im.SetCursorPos(im.ImVec2(cursorPos.x, cursorPos.y + size + im.GetStyle().ItemSpacing.y))
  im.NewLine()
end

local function onGui()

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
end

M.onGui = onGui
M.decalTextureWidgetInspect = decalTextureWidgetInspect
M.decalColorGradientWidgetInspect = decalColorGradientWidgetInspect
M.registerEditorPreferences = registerEditorPreferences
M.editorPreferenceValueChanged = editorPreferenceValueChanged
M.setup = setup

return M