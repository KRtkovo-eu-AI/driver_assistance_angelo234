-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = "editor_dynamicDecals_layerTypes_fill"
local im = ui_imgui

-- reference to the editor tool, set in setup()
local tool = nil
-- reference to the dynamics decal api
local api = nil
local inspector = nil
local docs = nil
local colorHistory = nil
local widgets = nil

local fillLayer_add_windowName = "Dynamic Decals Tool - Add Fill Layer Window"

local function inspectLayerGui(layer, guiId)
  local widgetId = string.format("%s_%s", layer.uid, guiId)

  local style = im.GetStyle()
  local colorButtonHeight = math.ceil(im.GetFontSize()) + 2 * style.FramePadding.y
  if widgets.draw(layer.colorPaletteMapId, api.propertiesMap["fill_colorPaletteMapId"], widgetId, nil, {widthMod = -(style.ItemSpacing.x + colorButtonHeight)}) then
    layer.colorPaletteMapId = api.propertiesMap["fill_colorPaletteMapId"].value
    api.setLayer(layer, true)
  end
  im.SameLine()
  local vehicleObj = getPlayerVehicle(0)
  local paletteColor = {1, 1, 1, 0}
  if layer.colorPaletteMapId == 0 then
    paletteColor = layer.color:toTable()
  elseif layer.colorPaletteMapId == 1 then
    local color = vehicleObj.color
    paletteColor = {vehicleObj.color.x, vehicleObj.color.y, vehicleObj.color.z, vehicleObj.color.w}
  elseif layer.colorPaletteMapId == 2 then
    local color = vehicleObj.colorPalette0
    paletteColor = {vehicleObj.colorPalette0.x, vehicleObj.colorPalette0.y, vehicleObj.colorPalette0.z, vehicleObj.colorPalette0.w}
  elseif layer.colorPaletteMapId == 3 then
    local color = vehicleObj.colorPalette1
    paletteColor = {vehicleObj.colorPalette1.x, vehicleObj.colorPalette1.y, vehicleObj.colorPalette1.z, vehicleObj.colorPalette1.w}
  end
  im.ColorButton(string.format("Color##fillLayer_vehicleColorPalette_colorButton_%s", guiId), editor.getTempImVec4_TableTable(paletteColor), nil, im.ImVec2(colorButtonHeight, colorButtonHeight))

  if layer.colorPaletteMapId ~= 0 then im.BeginDisabled() end
  if widgets.draw(layer.color:toTable(), api.propertiesMap["fill_color"], widgetId, editor.getTempBool_BoolBool(false)) then
    layer.color = Point4F.fromTable(api.propertiesMap["fill_color"].value)
  end
  if editor.getTempBool_BoolBool() == true then
    layer.color = Point4F.fromTable(api.propertiesMap["fill_color"].value)
    api.setLayer(layer, true)
  end
  if layer.colorPaletteMapId ~= 0 then im.EndDisabled() end

  -- Disabled for the time since it's heavily WIP
  -- if widgets.draw(layer.blendMode, api.propertiesMap["fill_blendMode"], widgetId) then
  --   layer.blendMode = api.propertiesMap["fill_blendMode"].value
  --   api.setLayer(layer, true)
  -- end
end

local function sectionGui(guiId)
  local style = im.GetStyle()
  local colorButtonHeight = math.ceil(im.GetFontSize()) + 2 * style.FramePadding.y
  local colorPaletteMapId = api.getFillLayerColorPaletteMapId()
  if widgets.draw(colorPaletteMapId, api.propertiesMap["fill_colorPaletteMapId"], guiId, nil, {widthMod = -(style.ItemSpacing.x + colorButtonHeight)}) then
    api.setFillLayerColorPaletteMapId(api.propertiesMap["fill_colorPaletteMapId"].value)
  end
  im.SameLine()
  local vehicleObj = getPlayerVehicle(0)
  local paletteColor = {1, 1, 1, 0}
  if colorPaletteMapId == 0 then
    paletteColor = api.getFillLayerColor():toTable()
  elseif colorPaletteMapId == 1 then
    local color = vehicleObj.color
    paletteColor = {vehicleObj.color.x, vehicleObj.color.y, vehicleObj.color.z, vehicleObj.color.w}
  elseif colorPaletteMapId == 2 then
    local color = vehicleObj.colorPalette0
    paletteColor = {vehicleObj.colorPalette0.x, vehicleObj.colorPalette0.y, vehicleObj.colorPalette0.z, vehicleObj.colorPalette0.w}
  elseif colorPaletteMapId == 3 then
    local color = vehicleObj.colorPalette1
    paletteColor = {vehicleObj.colorPalette1.x, vehicleObj.colorPalette1.y, vehicleObj.colorPalette1.z, vehicleObj.colorPalette1.w}
  end
  im.ColorButton(string.format("Color##fillLayer_vehicleColorPalette_colorButton_%s", guiId), editor.getTempImVec4_TableTable(paletteColor), nil, im.ImVec2(colorButtonHeight, colorButtonHeight))

  if colorPaletteMapId ~= 0 then im.BeginDisabled() end
  if widgets.draw(api.getFillLayerColor():toTable(), api.propertiesMap["fill_color"], guiId) then
    api.setFillLayerColor(Point4F.fromTable(api.propertiesMap["fill_color"].value))
  end
  if colorPaletteMapId ~= 0 then im.EndDisabled() end
end

local function onEditorGui()
  if editor.beginWindow(fillLayer_add_windowName, "Add Fill Layer") then
    im.Text("Add a new layer filled with a color")
    sectionGui("addLayerWindow")
    if im.Button("OK##fillLayer_Add_Modal") then
      api.addFillLayer()
      colorHistory.addColorToHistory(api.getFillLayerColor())
      editor.hideWindow(fillLayer_add_windowName)
    end
    im.SameLine()
    if im.Button("Cancel##fillLayer_Add_Modal") then
      editor.hideWindow(fillLayer_add_windowName)
    end
  end
  editor.endWindow()
end

local function toolbarItemGui()
  if editor.uiIconImageButton(editor.icons.format_color_fill, nil, nil, nil, nil, "Add fill layer") then
    editor.showWindow(fillLayer_add_windowName)
  end
  im.tooltip("Add fill layer")
end


local function registerEditorPreferences(prefsRegistry)
  -- prefsRegistry:registerSubCategory("dynamicDecalsTool", "moduleName", nil, {

  -- })
end

local function editorPreferenceValueChanged(path, value)

end

local function openAddLayerWindow()
  editor.showWindow(fillLayer_add_windowName)
end

local function documentationGui(docsSection)
  im.PushTextWrapPos(im.GetContentRegionAvailWidth())
  im.TextUnformatted([[
Fill Layers are fundamental layers that serve as a simple yet essential tool.
As the name suggests, they enable you to fill the entire vehicle with a single color of your choice.
This straightforward feature provides a quick and effective way to establish the base color of your design before further refining it with additional layers and details.
]])
  im.PopTextWrapPos()
end

local function setup(tool_in)
  tool = tool_in
  api = extensions.editor_api_dynamicDecals
  inspector = extensions.editor_dynamicDecals_inspector
  docs = extensions.editor_dynamicDecals_docs
  colorHistory = extensions.editor_dynamicDecals_colorHistory
  widgets = extensions.editor_dynamicDecals_widgets

  tool.registerOnEditorGuiFn("fill", onEditorGui)
  -- tool.registerSection("Fill Layer Properties", sectionGui, 1090, false, {})
  inspector.registerLayerGui(api.layerTypes.fill, inspectLayerGui)
  tool.registerToolbarToolItem("fill", toolbarItemGui, 10)
  docs.register({section = {"Layer Types", "Fill Layers"}, guiFn = documentationGui})

  editor.registerWindow(fillLayer_add_windowName, im.ImVec2(360, 140), nil, nil, nil, true)
end

M.registerEditorPreferences = registerEditorPreferences
M.editorPreferenceValueChanged = editorPreferenceValueChanged
M.setup = setup
M.openAddLayerWindow = openAddLayerWindow

return M