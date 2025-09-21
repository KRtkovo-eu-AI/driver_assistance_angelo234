-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = "editor_dynamicDecals_layerTypes_textureFill"
local im = ui_imgui

-- reference to the editor tool, set in setup()
local tool = nil
-- reference to the dynamics decal api
local api = nil
local helper = nil
local inspector = nil
local docs = nil
local widgets = nil

local lockFillTextureLayerScaleRatio = true
local textureFillLayer_add_windowName = "Dynamic Decals Tool - Add Texture Fill Layer Window"

local function inspectLayerGui(layer, guiId)
  local widgetId = string.format("%s_%s", layer.uid, guiId)

  if widgets.draw(layer.fillTexturePath, api.propertiesMap["fillTexturePath"], widgetId) then
    layer.fillTexturePath = api.propertiesMap["fillTexturePath"].value
    api.setLayer(layer, true)
  end

  local style = im.GetStyle()
  local colorButtonHeight = math.ceil(im.GetFontSize()) + 2 * style.FramePadding.y
  if widgets.draw(layer.colorPaletteMapId, api.propertiesMap["textureFill_colorPaletteMapId"], widgetId, nil, {widthMod = -(style.ItemSpacing.x + colorButtonHeight)}) then
    layer.colorPaletteMapId = api.propertiesMap["textureFill_colorPaletteMapId"].value
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
  if widgets.draw(layer.color:toTable(), api.propertiesMap["color"], widgetId, editor.getTempBool_BoolBool(false)) then
    layer.color = Point4F.fromTable(api.propertiesMap["color"].value)
  end
  if editor.getTempBool_BoolBool() == true then
    layer.color = Point4F.fromTable(api.propertiesMap["color"].value)
    api.setLayer(layer, true)
  end
  if layer.colorPaletteMapId ~= 0 then im.EndDisabled() end

  if widgets.draw({layer.scale.x, layer.scale.y}, api.propertiesMap["scale"], widgetId, editor.getTempBool_BoolBool(false)) then
    layer.scale.x = api.propertiesMap["scale"].value[1]
    layer.scale.y = api.propertiesMap["scale"].value[2]
  end
  if editor.getTempBool_BoolBool() == true then
    layer.scale.x = api.propertiesMap["scale"].value[1]
    layer.scale.y = api.propertiesMap["scale"].value[2]
    api.setLayer(layer, true)
  end

  if widgets.draw({layer.offset.x, layer.offset.y}, api.propertiesMap["offset"], widgetId, editor.getTempBool_BoolBool(false)) then
    layer.offset.x = api.propertiesMap["offset"].value[1]
    layer.offset.y = api.propertiesMap["offset"].value[2]
  end
  if editor.getTempBool_BoolBool() == true then
    layer.offset.x = api.propertiesMap["offset"].value[1]
    layer.offset.y = api.propertiesMap["offset"].value[2]
    api.setLayer(layer, true)
  end
end

local function sectionGui(guiId)
  local style = im.GetStyle()
  local colorButtonHeight = math.ceil(im.GetFontSize()) + 2 * style.FramePadding.y
  local colorPaletteMapId = api.getFillLayerColorPaletteMapId()
  if widgets.draw(colorPaletteMapId, api.propertiesMap["textureFill_colorPaletteMapId"], guiId, nil, {widthMod = -(style.ItemSpacing.x + colorButtonHeight)}) then
    api.setFillLayerColorPaletteMapId(api.propertiesMap["textureFill_colorPaletteMapId"].value)
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
  if widgets.draw(api.getFillLayerColor():toTable(), api.propertiesMap["textureFill_color"], guiId) then
    api.setFillLayerColor(Point4F.fromTable(api.propertiesMap["textureFill_color"].value))
  end
  if colorPaletteMapId ~= 0 then im.EndDisabled() end

  if widgets.draw(api.getFillTexturePath(), api.propertiesMap["fillTexturePath"], guiId) then
    api.setFillTexturePath(api.propertiesMap["fillTexturePath"].value)
  end

  if widgets.draw(api.getTextureFillLayerScale():toTable(), api.propertiesMap["scale"], guiId) then
    api.setTextureFillLayerScale(Point2F.fromTable(api.propertiesMap["scale"].value))
  end

  if widgets.draw(api.getTextureFillOffset():toTable(), api.propertiesMap["offset"], guiId) then
    api.setTextureFillOffset(Point2F.fromTable(api.propertiesMap["offset"].value))
  end

  -- Disabled for the time since it's heavily WIP
  -- if widgets.draw(api.getBlendMode(), api.propertiesMap["textureFill_blendMode"], widgetId) then
  --   api.setBlendMode(api.propertiesMap["textureFill_blendMode"].value)
  --   api.setLayer(layer, true)
  -- end
end

local function onEditorGui()
  if editor.beginWindow(textureFillLayer_add_windowName, "Add Texture Fill Layer") then
    im.Text("Add a new layer filled with a texture")
    sectionGui("addLayerWindow")
    if im.Button("OK##textureFillLayer_Add_Modal") then
      api.addTextureFillLayer()
      editor.hideWindow(textureFillLayer_add_windowName)
    end
    im.SameLine()
    if im.Button("Cancel##textureFillLayer_Add_Modal") then
      editor.hideWindow(textureFillLayer_add_windowName)
    end
  end
  editor.endWindow()
end

local function toolbarItemGui()
  if editor.uiIconImageButton(editor.icons.texture, nil, nil, nil, nil, "Add texture fill layer") then
    editor.showWindow(textureFillLayer_add_windowName)
  end
  im.tooltip("Add texture fill layer")
end

local function registerEditorPreferences(prefsRegistry)
  -- prefsRegistry:registerSubCategory("dynamicDecalsTool", "moduleName", nil, {

  -- })
end

local function editorPreferenceValueChanged(path, value)

end

local function openAddLayerWindow()
  editor.showWindow(textureFillLayer_add_windowName)
end

local function documentationGui(docsSection)
  im.PushTextWrapPos(im.GetContentRegionAvailWidth())
  im.TextUnformatted([[
Fill layers let you apply patterns or textures to the vehicle.

The texture fills are based on an input texture, allowing for creative customization.

Adjust the scale of the texture fill layer to fine-tune the appearance.
]])
  im.PopTextWrapPos()
end

local function setup(tool_in)
  tool = tool_in
  api = extensions.editor_api_dynamicDecals
  helper = extensions.editor_dynamicDecals_helper
  inspector = extensions.editor_dynamicDecals_inspector
  docs = extensions.editor_dynamicDecals_docs
  widgets = extensions.editor_dynamicDecals_widgets

  tool.registerOnEditorGuiFn("textureFill", onEditorGui)
  -- tool.registerSection("Texture Fill Layer Properties", sectionGui, 1100, false, {})
  inspector.registerLayerGui(api.layerTypes.textureFill, inspectLayerGui)
  tool.registerToolbarToolItem("textureFill", toolbarItemGui, 20)
  docs.register({section = {"Layer Types", "Texture Fill Layers"}, guiFn = documentationGui})

  editor.registerWindow(textureFillLayer_add_windowName, im.ImVec2(450, 310), nil, nil, nil, true)
end

M.registerEditorPreferences = registerEditorPreferences
M.editorPreferenceValueChanged = editorPreferenceValueChanged
M.setup = setup
M.openAddLayerWindow = openAddLayerWindow

return M