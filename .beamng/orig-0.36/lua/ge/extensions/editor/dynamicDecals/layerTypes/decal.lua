-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = "editor_dynamicDecals_layerTypes_decal"
local im = ui_imgui

local fontCharacterSelectionWindowName = logTag .. "_fontCharacterSelectionWindow"

-- reference to the editor tool, set in setup()
local tool = nil
-- reference to the dynamics decal api
local api = nil
local texturesApi = nil
local brushes = nil
local inspector = nil
local inspectorUtils = nil
local helper = nil
local docs = nil
local widgets = nil
local fonts = nil

local newBrushName = ""
local lockScaleRatio = true
local lockColorTextureScaleRatio = true
local lockAlphaMaskScaleRatio = true
local lockAlphaMaskOffsetRatio = false

local mirrorDebugProperty = nil
local mirrorOffsetProperty = nil

local sdfIntroWindowName = "Dynamic Decals - SDF Introduction"
local sdfModalHasBeenClosed = false
local highlightSdfProperties = false
local sdfExplanationTextWidget = function()
  im.TextUnformatted("Signed Distance Fields (SDF) brings a range of improvements, allowing you to create more complex decals with ease.")
  im.NewLine()
  im.TextUnformatted("What are Signed Distance Fields?")
  im.NewLine()
  im.TextUnformatted("Signed Distance Fields are mathematical representations used in computer graphics to describe the shape of objects or regions. In the context of decals, SDF allows us to precisely define the boundaries and characteristics of decals.")
  im.NewLine()
  im.TextUnformatted("Benefits of SDF for Decals:")
  im.BulletText("Crisp Outlines: With SDF, you can effortlessly add crisp outlines to your decals.\nThe distance information provided by SDF enables accurate edge detection, ensuring your outlines appear sharp and well-defined.")
  im.BulletText("Feathered Edges: SDF empowers you to achieve smooth and feathered edges in your decals.\nBy leveraging the distance values, you can create gradual transitions between the decal and the underlying surface, resulting in a more natural and visually appealing appearance.")
  im.BulletText("Differently Colored Outlines: With SDF, you can easily add outlines of different colors to your decals.")
  im.BulletText("Efficient Storage: SDF textures can be significantly smaller in size compared to normal textures.\nThe compact representation of shapes in SDF allows for efficient storage and reduced memory usage, enabling you to store a larger variety of decals without sacrificing performance or storage capacity.")

  im.NewLine()
  im.TextUnformatted("Experiment with these features by by enaling SDF and checking out the SDF properties in the 'Decal Properties' section.")
end

local blendModesNamesCharPtr = nil
local meshesFilter = im.ImGuiTextFilter()

local function setPropertyInChildrenRec(layer, property, value)
  if layer.children then
    for k, child in ipairs(layer.children) do
      if child[property] ~= nil then
        child[property] = value
      end
      setPropertyInChildrenRec(child, property, value)
    end
  end
end

M.isTexturesSdfCompatible = function(texturePath)
  local file = texturePath or api.getDecalTexturePath("color")
  local metaData = texturesApi.readSidecarFile(file)
  if metaData and metaData.isSdfCompatible then
    return true
  else
    return false
  end
end

M.checkColorDecalTexturesSdfCompatible = function()
  if M.isTexturesSdfCompatible() then
    if (sdfModalHasBeenClosed == false and editor.getPreference("dynamicDecalsTool.decalProperties.doNotShowSdfIntroAgain") == false) then
      M.showSdfIntroWindow()
    end
  end
end

local function inspectLayerGui(layer, guiId)
  local widgetId = string.format("%s_%s", layer.uid, guiId)
  local vehicleObj = getPlayerVehicle(0)

  if widgets.draw({layer.decalScale.x, layer.decalScale.y, layer.decalScale.z}, api.propertiesMap["decalScale"], widgetId, editor.getTempBool_BoolBool(false)) then
    layer.decalScale.x = api.propertiesMap["decalScale"].value[1]
    layer.decalScale.z = api.propertiesMap["decalScale"].value[3]
  end
  if editor.getTempBool_BoolBool() == true then
    layer.decalScale.x = api.propertiesMap["decalScale"].value[1]
    layer.decalScale.z = api.propertiesMap["decalScale"].value[3]
    api.setLayer(layer, true)
  end

  if widgets.draw(layer.decalRotation, api.propertiesMap["decalRotation"], widgetId, editor.getTempBool_BoolBool(false)) then
    layer.decalRotation = api.propertiesMap["decalRotation"].value
  end
  if editor.getTempBool_BoolBool() == true then
    layer.decalRotation = api.propertiesMap["decalRotation"].value
    api.setLayer(layer, true)
  end

  if widgets.draw(layer.decalSkew:toTable(), api.propertiesMap["decalSkew"], widgetId, editor.getTempBool_BoolBool(false)) then
    layer.decalSkew.x = api.propertiesMap["decalSkew"].value[1]
    layer.decalSkew.y = api.propertiesMap["decalSkew"].value[2]
  end
  if editor.getTempBool_BoolBool() == true then
    layer.decalSkew.x = api.propertiesMap["decalSkew"].value[1]
    layer.decalSkew.y = api.propertiesMap["decalSkew"].value[2]
    api.setLayer(layer, true)
  end

  im.BeginDisabled()
  im.Separator()
  if widgets.draw(layer.decalUv:toTable(), api.propertiesMap["decalUv"], widgetId, editor.getTempBool_BoolBool(false)) then
    layer.decalUv.x = api.propertiesMap["decalUv"].value[1]
    layer.decalUv.y = api.propertiesMap["decalUv"].value[2]
  end
  if editor.getTempBool_BoolBool() == true then
    layer.decalUv.x = api.propertiesMap["decalUv"].value[1]
    layer.decalUv.y = api.propertiesMap["decalUv"].value[2]
    api.setLayer(layer, true)
  end
  im.EndDisabled()

  im.TextUnformatted("Flip")
  im.SameLine()
  widgets.defaultButton(string.format("%s_flipDecal", widgetId),
  function()
    layer.decalUv.x = 1
    layer.decalUv.y = 1
    api.setLayer(layer, true)
  end,
  string.format("Reset to default: %s", dumps({false, false})))
  local btnCol = im.GetStyleColorVec4(im.Col_Button)
  im.PushStyleColor2(im.Col_Button, layer.decalUv.x == -1 and editor.color.beamng.Value or btnCol)
  local btnWidth = (im.GetContentRegionAvailWidth() - im.GetStyle().ItemSpacing.x) / 2
  if im.Button(string.format("Horizontally##%s_flipHorizontally", widgetId), im.ImVec2(btnWidth, 0)) then
    layer.decalUv.x = layer.decalUv.x * -1
    api.setLayer(layer, true)
  end
  im.PopStyleColor()
  im.SameLine()
  im.PushStyleColor2(im.Col_Button, layer.decalUv.y == -1 and editor.color.beamng.Value or btnCol)
  if im.Button(string.format("Vertically##%s_flipVertically", widgetId), im.ImVec2(btnWidth, 0)) then
    layer.decalUv.y = layer.decalUv.y * -1
    api.setLayer(layer, true)
  end
  im.PopStyleColor()

  im.Separator()
  if widgets.draw(layer.camPosition:toTable(), api.propertiesMap["camPosition"], widgetId, editor.getTempBool_BoolBool(false)) then
    layer.camPosition.x = api.propertiesMap["camPosition"].value[1]
    layer.camPosition.y = api.propertiesMap["camPosition"].value[2]
    layer.camPosition.z = api.propertiesMap["camPosition"].value[3]
  end
  if editor.getTempBool_BoolBool() == true then
    layer.camPosition.x = api.propertiesMap["camPosition"].value[1]
    layer.camPosition.y = api.propertiesMap["camPosition"].value[2]
    layer.camPosition.z = api.propertiesMap["camPosition"].value[3]
    api.setLayer(layer, true)
  end

  im.Separator()

  local style = im.GetStyle()
  local colorButtonHeight = math.ceil(im.GetFontSize()) + 2 * style.FramePadding.y
  if widgets.draw(layer.colorPaletteMapId, api.propertiesMap["colorPaletteMapId"], widgetId, nil, {widthMod = -(style.ItemSpacing.x + colorButtonHeight)}) then
    layer.colorPaletteMapId = api.propertiesMap["colorPaletteMapId"].value
    api.setLayer(layer, true)
  end
  im.SameLine()
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
  if widgets.draw(layer.decalUseGradientColor, api.propertiesMap["decalUseGradientColor"], widgetId) then
    layer.decalUseGradientColor = api.propertiesMap["decalUseGradientColor"].value
    api.setLayer(layer, true)
  end

  if layer.decalUseGradientColor == true then im.BeginDisabled() end
  if widgets.draw(layer.color:toTable(), api.propertiesMap["color"], widgetId, editor.getTempBool_BoolBool(false)) then
    layer.color.x = api.propertiesMap["color"].value[1]
    layer.color.y = api.propertiesMap["color"].value[2]
    layer.color.z = api.propertiesMap["color"].value[3]
    layer.color.w = api.propertiesMap["color"].value[4]
  end
  if editor.getTempBool_BoolBool() == true then
    layer.color.x = api.propertiesMap["color"].value[1]
    layer.color.y = api.propertiesMap["color"].value[2]
    layer.color.z = api.propertiesMap["color"].value[3]
    layer.color.w = api.propertiesMap["color"].value[4]
    api.setLayer(layer, true)
  end
  if layer.decalUseGradientColor == true then im.EndDisabled() end

  if layer.decalUseGradientColor == false then im.BeginDisabled() end
  if widgets.draw(
    {layer.decalGradientColorTopLeft:toTable(), layer.decalGradientColorTopRight:toTable(), layer.decalGradientColorBottomLeft:toTable(), layer.decalGradientColorBottomRight:toTable()},
    api.propertiesMap["decalGradientColor"],
    widgetId,
    editor.getTempBool_BoolBool(false)
  ) then
    local value = api.propertiesMap["decalGradientColor"].value
    layer.decalGradientColorTopLeft.r = value[1][1]
    layer.decalGradientColorTopLeft.g = value[1][2]
    layer.decalGradientColorTopLeft.b = value[1][3]
    layer.decalGradientColorTopLeft.a = value[1][4]

    layer.decalGradientColorTopRight.r = value[2][1]
    layer.decalGradientColorTopRight.g = value[2][2]
    layer.decalGradientColorTopRight.b = value[2][3]
    layer.decalGradientColorTopRight.a = value[2][4]

    layer.decalGradientColorBottomLeft.r = value[3][1]
    layer.decalGradientColorBottomLeft.g = value[3][2]
    layer.decalGradientColorBottomLeft.b = value[3][3]
    layer.decalGradientColorBottomLeft.a = value[3][4]

    layer.decalGradientColorBottomRight.r = value[4][1]
    layer.decalGradientColorBottomRight.g = value[4][2]
    layer.decalGradientColorBottomRight.b = value[4][3]
    layer.decalGradientColorBottomRight.a = value[4][4]
  end

  if editor.getTempBool_BoolBool() == true then
    local value = api.propertiesMap["decalGradientColor"].value
    layer.decalGradientColorTopLeft.r = value[1][1]
    layer.decalGradientColorTopLeft.g = value[1][2]
    layer.decalGradientColorTopLeft.b = value[1][3]
    layer.decalGradientColorTopLeft.a = value[1][4]

    layer.decalGradientColorTopRight.r = value[2][1]
    layer.decalGradientColorTopRight.g = value[2][2]
    layer.decalGradientColorTopRight.b = value[2][3]
    layer.decalGradientColorTopRight.a = value[2][4]

    layer.decalGradientColorBottomLeft.r = value[3][1]
    layer.decalGradientColorBottomLeft.g = value[3][2]
    layer.decalGradientColorBottomLeft.b = value[3][3]
    layer.decalGradientColorBottomLeft.a = value[3][4]

    layer.decalGradientColorBottomRight.r = value[4][1]
    layer.decalGradientColorBottomRight.g = value[4][2]
    layer.decalGradientColorBottomRight.b = value[4][3]
    layer.decalGradientColorBottomRight.a = value[4][4]
    api.setLayer(layer, true)
  end
  if layer.decalUseGradientColor == false then im.EndDisabled() end
  if layer.colorPaletteMapId ~= 0 then im.EndDisabled() end
  im.Separator()

  if widgets.draw({layer.cursorPosScreenUv.x, layer.cursorPosScreenUv.y}, api.propertiesMap["cursorPosScreenUv"], widgetId, editor.getTempBool_BoolBool(false)) then
    layer.cursorPosScreenUv.x = api.propertiesMap["cursorPosScreenUv"].value[1]
    layer.cursorPosScreenUv.y = api.propertiesMap["cursorPosScreenUv"].value[2]
  end
  if editor.getTempBool_BoolBool() == true then
    layer.cursorPosScreenUv.x = api.propertiesMap["cursorPosScreenUv"].value[1]
    layer.cursorPosScreenUv.y = api.propertiesMap["cursorPosScreenUv"].value[2]
    api.setLayer(layer, true)
  end
  im.Separator()

  if widgets.draw(layer.mirrored, api.propertiesMap["mirrored"], widgetId) then
    layer.mirrored = api.propertiesMap["mirrored"].value
    if editor.getPreference("dynamicDecalsTool.inspector.applyMirroredPropertyToChildren") then
      setPropertyInChildrenRec(layer, "mirrored", layer.mirrored)
    end
    api.setLayer(layer, true)
  end
  if editor.getPreference("dynamicDecalsTool.inspector.applyMirroredPropertyToChildren") then
    helper.iconTooltip("Affects all child layers. If you don't want this widget to affect all child layers, you can turn off the option in the preferences window.", true)
  else
    helper.iconTooltip("Does not affect child layers. If you do want this widget to affect all child layers as well, you can turn on the option in the preferences window.", true)
  end

  if widgets.draw(layer.flipMirroredDecal, api.propertiesMap["flipMirroredDecal"], widgetId) then
    layer.flipMirroredDecal = api.propertiesMap["flipMirroredDecal"].value
    if editor.getPreference("dynamicDecalsTool.inspector.applyMirroredPropertyToChildren") then
      setPropertyInChildrenRec(layer, "flipMirroredDecal", layer.flipMirroredDecal)
    end
    api.setLayer(layer, true)
  end
  if editor.getPreference("dynamicDecalsTool.inspector.applyMirroredPropertyToChildren") then
    helper.iconTooltip("Affects all child layers. If you don't want this widget to affect all child layers, you can turn off the option in the preferences window.", true)
  else
    helper.iconTooltip("Does not affect child layers. If you do want this widget to affect all child layers as well, you can turn on the option in the preferences window.", true)
  end
  im.Separator()

  local meshes = (layer.meshes and layer.meshes[vehicleObj.jbeam]) or nil
  local sMeshes = api.getShapeMeshes()

  local function setMeshEnable(name, val)
    -- mesh has been enabled
    if val == true then
      if not layer.meshes then layer.meshes = {} end
      if not layer.meshes[vehicleObj.jbeam] then layer.meshes[vehicleObj.jbeam] = {} end

      table.insert(layer.meshes[vehicleObj.jbeam], name)

      -- check if all possible meshes are enabled and if so set the object to nil
      local sMeshesCopy = api.getShapeMeshes()
      for _, n in ipairs(layer.meshes[vehicleObj.jbeam]) do
        if sMeshesCopy[n] then
          sMeshesCopy[n] = nil
        end
      end
      if tableSize(sMeshesCopy) == 0 then
        layer.meshes[vehicleObj.jbeam] = nil
        if tableSize(layer.meshes) == 0 then
          layer.meshes = nil
        end
      end

      api.setLayer(layer, true)
    else -- mesh has been disabled
      -- layer.meshes object is present, just remove the deselected one
      if meshes then
        for k, v in ipairs(layer.meshes[vehicleObj.jbeam]) do
          if v == name then
            table.remove(layer.meshes[vehicleObj.jbeam], k)
            api.setLayer(layer, true)
          end
        end
      else -- layer.meshes is nil so we need to add all meshes but the deselected one
        if not layer.meshes then layer.meshes = {} end
        if not layer.meshes[vehicleObj.jbeam] then layer.meshes[vehicleObj.jbeam] = {} end

        for name2, _ in pairs(sMeshes) do
          if name2 ~= name then
            table.insert(layer.meshes[vehicleObj.jbeam], name2)
            api.setLayer(layer, true)
          end
        end
      end
    end
  end

  local cpos = im.GetCursorPos()
  im.SetCursorPosX(cpos.x + 80)
  if im.SmallButton("enable all") then
    if layer.meshes and layer.meshes[vehicleObj.jbeam] then
      layer.meshes[vehicleObj.jbeam] = nil
    end

    if layer.meshes and tableSize(layer.meshes) == 0 then
      layer.meshes = nil
    end
    api.setLayer(layer, true)
  end
  im.SameLine()
  if im.SmallButton("disable all") then
    if not layer.meshes then layer.meshes = {} end
    layer.meshes[vehicleObj.jbeam] = {}
    api.setLayer(layer, true)
  end
  im.SameLine()
  editor.uiInputSearchTextFilter("##InspectorMeshesFilter", meshesFilter, im.GetContentRegionAvailWidth())

  im.SetCursorPos(cpos)

  if im.TreeNode1(string.format("Meshes##%s", widgetId)) then
    if im.BeginChild1(string.format("MeshesChild_%s", widgetId), im.ImVec2(0, 240), true) then
      local i = 1
      for name, _ in pairs(sMeshes) do
        if im.ImGuiTextFilter_PassFilter(meshesFilter, name) then
          local enabled = meshes == nil or tableContains(meshes, name)
          if im.Checkbox(string.format("##%s_shapeMesh_%d_checkbox", widgetId, i), editor.getTempBool_BoolBool(enabled)) then
            setMeshEnable(name, editor.getTempBool_BoolBool())
          end
          im.SameLine()
          if im.Selectable1(string.format("%s##%s_shapeMesh_%d_selectable", name, widgetId, i), enabled) then
            setMeshEnable(name, not enabled)
          end
        end
        i = i + 1
      end
      im.EndChild()
    end

    im.TreePop()
  end

  im.Separator()
  im.Separator()
  im.Separator()

  im.Columns(2, "layerDataColumns")

  im.TextUnformatted("project using surface normal")
  im.NextColumn()
  if im.Checkbox(string.format("##%s_%s_%s", layer.uid, guiId, "useSurfaceNormal"), editor.getTempBool_BoolBool(layer.useSurfaceNormal)) then
    layer.useSurfaceNormal = editor.getTempBool_BoolBool()
    -- TODO / FIXME
    -- ALERT WEEE WOOO WEEE WOOO
    -- This is a hack. The decal matrix is not recalculated for this layer as long as it has the 'decalPos' and 'decalNorm' field.
    layer.decalPos = nil
    layer.decalNorm = nil
    api.setLayer(layer, true)
  end
  im.NextColumn()

  im.TextUnformatted("color texture scale")
  im.NextColumn()
  im.PushItemWidth(im.GetContentRegionAvailWidth())
  if editor.uiSliderFloat2(string.format("##%s_%s_%s", layer.uid, guiId, "colorTextureScale"), editor.getTempFloatArray2_TableTable({layer.colorTextureScale.x, layer.colorTextureScale.y}), 0.01, 6.0, nil, nil, editor.getTempBool_BoolBool(false)) then
    local value = editor.getTempFloatArray2_TableTable()
    layer.colorTextureScale = Point2F(value[1], value[2])
  end
  im.PopItemWidth()
  if editor.getTempBool_BoolBool() == true then
    api.setLayer(layer, true)
  end
  im.NextColumn()

  im.TextUnformatted("alpha mask channel")
  im.NextColumn()
  im.PushItemWidth(im.GetContentRegionAvailWidth())
  if im.Combo2(string.format("##%s_%s_%s", layer.uid, guiId, "alphaMaskChannel"), editor.getTempInt_NumberNumber(layer.alphaMaskChannel), "red\0green\0blue\0alpha\0\0") then
    layer.alphaMaskChannel = editor.getTempInt_NumberNumber()
    api.setLayer(layer, true)
  end
  im.NextColumn()

  im.TextUnformatted("alpha mask blend mode")
  im.NextColumn()
  im.PushItemWidth(im.GetContentRegionAvailWidth())
  if im.Combo2(string.format("##%s_%s_%s", layer.uid, guiId, "alphaMaskBlendMode"), editor.getTempInt_NumberNumber(layer.alphaMaskBlendMode), "multiply\0add\0\0") then
    layer.alphaMaskBlendMode = editor.getTempInt_NumberNumber()
    api.setLayer(layer, true)
  end
  im.NextColumn()

  im.TextUnformatted("alpha mask scale")
  im.NextColumn()
  im.PushItemWidth(im.GetContentRegionAvailWidth())
  if editor.uiSliderFloat2(string.format("##%s_%s_%s", layer.uid, guiId, "alphaMaskScale"), editor.getTempFloatArray2_TableTable({layer.alphaMaskScale.x, layer.alphaMaskScale.y}), 0.01, 6.0, nil, nil, editor.getTempBool_BoolBool(false)) then
    local value = editor.getTempFloatArray2_TableTable()
    layer.alphaMaskScale = Point2F(value[1], value[2])
  end
  im.PopItemWidth()
  if editor.getTempBool_BoolBool() == true then
    api.setLayer(layer, true)
  end
  im.NextColumn()

  im.TextUnformatted("alpha mask rotation")
  im.NextColumn()
  im.PushItemWidth(im.GetContentRegionAvailWidth())
  if editor.uiSliderFloat(string.format("##%s_%s_%s", layer.uid, guiId, "alphaMaskRotation"), editor.getTempFloat_NumberNumber(layer.alphaMaskRotation * 180 / math.pi), 0, 360, nil, nil, editor.getTempBool_BoolBool(false)) then
    layer.alphaMaskRotation = (editor.getTempFloat_NumberNumber() / 180 * math.pi)
  end
  im.tooltip("alpha mask rotation in degrees")
  im.PopItemWidth()
  if editor.getTempBool_BoolBool() == true then
    api.setLayer(layer, true)
  end
  im.NextColumn()

  im.TextUnformatted("alpha mask intensity")
  im.NextColumn()
  im.PushItemWidth(im.GetContentRegionAvailWidth())
  local val = 0
  if editor.uiSliderFloat(string.format("##%s_%s_%s", layer.uid, guiId, "alphaMaskIntensity"), editor.getTempFloat_NumberNumber(layer.alphaMaskIntensity), 0.0, 2.0, "%.2f", nil, editor.getTempBool_BoolBool(false)) then
    layer.alphaMaskIntensity = editor.getTempFloat_NumberNumber()
  end
  im.PopItemWidth()
  if editor.getTempBool_BoolBool() == true then
    api.setLayer(layer, true)
  end
  im.NextColumn()

  im.TextUnformatted("wrap alpha mask X")
  im.NextColumn()
  if im.Checkbox(string.format("##%s_%s_%s", layer.uid, guiId, "wrapAlphaMaskX"), editor.getTempBool_BoolBool(layer.wrapAlphaMaskX)) then
    layer.wrapAlphaMaskX = editor.getTempBool_BoolBool()
    api.setLayer(layer, true)
  end
  im.NextColumn()

  im.TextUnformatted("wrap alpha mask Y")
  im.NextColumn()
  if im.Checkbox(string.format("##%s_%s_%s", layer.uid, guiId, "wrapAlphaMaskY"), editor.getTempBool_BoolBool(layer.wrapAlphaMaskY)) then
    layer.wrapAlphaMaskY = editor.getTempBool_BoolBool()
    api.setLayer(layer, true)
  end
  im.NextColumn()

  im.TextUnformatted("wrap color texture X")
  im.NextColumn()
  if im.Checkbox(string.format("##%s_%s_%s", layer.uid, guiId, "wrapColorTextureX"), editor.getTempBool_BoolBool(layer.wrapColorTextureX)) then
    layer.wrapColorTextureX = editor.getTempBool_BoolBool()
    api.setLayer(layer, true)
  end
  im.NextColumn()

  im.TextUnformatted("wrap color texture Y")
  im.NextColumn()
  if im.Checkbox(string.format("##%s_%s_%s", layer.uid, guiId, "wrapColorTextureY"), editor.getTempBool_BoolBool(layer.wrapColorTextureY)) then
    layer.wrapColorTextureY = editor.getTempBool_BoolBool()
    api.setLayer(layer, true)
  end
  im.NextColumn()

  im.TextUnformatted("alpha mask invert")
  im.NextColumn()
  if im.Checkbox(string.format("##%s_%s_%s", layer.uid, guiId, "alphaMaskInvert"), editor.getTempBool_BoolBool(layer.alphaMaskInvert)) then
    layer.alphaMaskInvert = editor.getTempBool_BoolBool()
    api.setLayer(layer, true)
  end
  im.NextColumn()

  -- SDF
  im.TextUnformatted("SDF enabled")
  im.NextColumn()
  if im.Checkbox(string.format("##%s_%s_%s", layer.uid, guiId, "sdfEnabled"), editor.getTempBool_BoolBool(layer.sdfEnabled)) then
    layer.sdfEnabled = editor.getTempBool_BoolBool()
    api.setLayer(layer, true)
  end
  im.NextColumn()

  if layer.sdfEnabled == false then im.BeginDisabled() end
  im.TextUnformatted("SDF thickness")
  im.NextColumn()
  im.PushItemWidth(im.GetContentRegionAvailWidth())
  if editor.uiSliderFloat(string.format("##%s_%s_%s", layer.uid, guiId, "sdfThickness"), editor.getTempFloat_NumberNumber(layer.sdfThickness), 0.0, 1.0, "%.2f", nil, editor.getTempBool_BoolBool(false)) then
    layer.sdfThickness = editor.getTempFloat_NumberNumber()
  end
  im.PopItemWidth()
  if editor.getTempBool_BoolBool() == true then
    api.setLayer(layer, true)
  end
  im.NextColumn()

  im.TextUnformatted("SDF softness")
  im.NextColumn()
  im.PushItemWidth(im.GetContentRegionAvailWidth())
  if editor.uiSliderFloat(string.format("##%s_%s_%s", layer.uid, guiId, "sdfSoftness"), editor.getTempFloat_NumberNumber(layer.sdfSoftness), 0.0, 1.0, "%.2f", nil, editor.getTempBool_BoolBool(false)) then
    layer.sdfSoftness = editor.getTempFloat_NumberNumber()
  end
  im.PopItemWidth()
  if editor.getTempBool_BoolBool() == true then
    api.setLayer(layer, true)
  end
  im.NextColumn()

  im.TextUnformatted("sdf outline color")
  im.NextColumn()
  im.PushItemWidth(im.GetContentRegionAvailWidth())
  local sdfOutlineColorTbl = layer.sdfOutlineColor:toTable()
  if editor.uiColorEdit4(string.format("##%s_%s_%s", layer.uid, guiId, "sdfOutlineColor"), editor.getTempFloatArray3_TableTable({sdfOutlineColorTbl[1]/255, sdfOutlineColorTbl[2]/255, sdfOutlineColorTbl[3]/255}), nil, editor.getTempBool_BoolBool(false)) then
    local value = editor.getTempFloatArray3_TableTable()
    layer.sdfOutlineColor = ColorI(value[1] * 255, value[2] * 255, value[3] * 255, 255)
  end
  im.PopItemWidth()
  if editor.getTempBool_BoolBool() == true then
    api.setLayer(layer, true)
  end
  im.NextColumn()

  im.TextUnformatted("SDF outline thickness")
  im.NextColumn()
  im.PushItemWidth(im.GetContentRegionAvailWidth())
  if editor.uiSliderFloat(string.format("##%s_%s_%s", layer.uid, guiId, "sdfOutlineThickness"), editor.getTempFloat_NumberNumber(layer.sdfOutlineThickness), 0.0, 1.0, "%.2f", nil, editor.getTempBool_BoolBool(false)) then
    layer.sdfOutlineThickness = editor.getTempFloat_NumberNumber()
  end
  im.PopItemWidth()
  if editor.getTempBool_BoolBool() == true then
    api.setLayer(layer, true)
  end
  im.NextColumn()

  im.TextUnformatted("SDF outline softness")
  im.NextColumn()
  im.PushItemWidth(im.GetContentRegionAvailWidth())
  if editor.uiSliderFloat(string.format("##%s_%s_%s", layer.uid, guiId, "sdfOutlineSoftness"), editor.getTempFloat_NumberNumber(layer.sdfOutlineSoftness), 0.0, 1.0, "%.2f", nil, editor.getTempBool_BoolBool(false)) then
    layer.sdfOutlineSoftness = editor.getTempFloat_NumberNumber()
  end
  im.PopItemWidth()
  if editor.getTempBool_BoolBool() == true then
    api.setLayer(layer, true)
  end
  im.NextColumn()
  if layer.sdfEnabled == false then im.EndDisabled() end
  -- ~SDF

  im.TextUnformatted("decal color texture path")
  im.NextColumn()
  inspectorUtils.decalTextureWidgetInspect(layer, "decalColorTexturePath", guiId)
  im.NextColumn()

  im.TextUnformatted("decal alpha texture path")
  im.NextColumn()
  inspectorUtils.decalTextureWidgetInspect(layer, "decalAlphaTexturePath", guiId)
  im.NextColumn()

  -- Hide all non color texture widgets for the time being
  --[[
  im.TextUnformatted("decal normal texture path")
  im.NextColumn()
  inspectorUtils.decalTextureWidgetInspect(layer, "decalNormalTexturePath", guiId, "/art/dynamicDecals/textures/_normal.png")
  im.NextColumn()

  im.TextUnformatted("decal metallic texture path")
  im.NextColumn()
  inspectorUtils.decalTextureWidgetInspect(layer, "decalMetallicTexturePath", guiId)
  im.NextColumn()

  im.TextUnformatted("decal roughness texture path")
  im.NextColumn()
  inspectorUtils.decalTextureWidgetInspect(layer, "decalRoughnessTexturePath", guiId)
  im.NextColumn()
  ]]

  -- Hide blend mode for the time being
  --[[
    im.TextUnformatted("blend mode")
    im.NextColumn()
    im.TextUnformatted(api.blendModes[layer.blendMode + 1].name)
    im.NextColumn()
  ]]

  im.TextUnformatted("useZBufferDepth")
  im.NextColumn()
  im.TextUnformatted(layer.useZBufferDepth and "true" or "false")
  im.NextColumn()

  im.TextUnformatted("zBufferDepth")
  im.NextColumn()
  im.TextUnformatted(tostring(layer.zBufferDepth))
  im.NextColumn()

  im.TextUnformatted("worldToViewToScreen")
  im.NextColumn()
  im.TextUnformatted(layer.worldToViewToScreen:__tostring())
  im.NextColumn()

  im.Columns(1, "layerDataColumns")
end

local function decalTextureWidget(type, name, removeTextureOverridePath)
  im.TextUnformatted("Decal " .. name .. " Texture Path")
  im.PushItemWidth(im.GetContentRegionAvailWidth() - 2 * tool.getIconSize() - 2 * im.GetStyle().ItemSpacing.x)
  im.InputText("##Decal" .. name .. "TexturePath", editor.getTempCharPtr(api.getDecalTexturePath(type)), nil, im.InputTextFlags_ReadOnly)
  im.PopItemWidth()
  im.SameLine()
  if editor.uiIconImageButton(editor.icons.folder, im.ImVec2(tool.getIconSize(), tool.getIconSize()), nil, nil, nil, name .. "_Button") then
    editor_fileDialog.openFile(
      function(data)
        api.setDecalTexturePath(type, data.filepath)
        return true
      end,
      {{"Any files", "*"},{"PNG files",".png"},{"Image files",{".png", ".jpg", ".jpeg"}}},
      false,
      path.split(api.getDecalTexturePath(type)) or "/art/decals/dynDecals/",
      true
    )
  end
  im.tooltip("Change Decal " .. name .. " Texture Path")
  im.SameLine()
  if editor.uiIconImageButton(editor.icons.delete, im.ImVec2(tool.getIconSize(), tool.getIconSize()), nil, nil, nil, name .. "_RemoveButton") then
    api.setDecalTexturePath(type, removeTextureOverridePath or "/")
    return true
  end
  im.tooltip("Remove Decal " .. name .. " Texture Path")
  local img = editor.getTempTextureObj(api.getDecalTexturePath(type))
  local imgWidthSetting = editor.getPreference("dynamicDecalsTool.decalProperties.texturePreviewSize")
  local imgWidth = imgWidthSetting > im.GetContentRegionAvailWidth() and im.GetContentRegionAvailWidth() or imgWidthSetting
  local imgHeight = img.path == "/" and imgWidth or imgWidth * img.size.y / img.size.x
  im.Image(img.texId, im.ImVec2(imgWidth, imgHeight), im.ImVec2(0,0), im.ImVec2(1,1), nil, editor.color.beamng.Value)
  if im.BeginDragDropTarget() then
    local payload = im.AcceptDragDropPayload("DynDecalTextureDrapDrop")
    if payload~=nil then
      assert(payload.DataSize == 256)
      local path = ffi.string(payload.Data)
      api.setDecalTexturePath(type, path)
      return true
    end
    im.EndDragDropTarget()
  end
end

local function decalColorSdfPropertiesWidget(guiId)
  if im.TreeNodeEx1(string.format("SDF Properties##DecalColor_%s", guiId), im.TreeNodeFlags_DefaultOpen) then
    if im.Button("What is SDF?", im.ImVec2(im.GetContentRegionAvailWidth(), 0)) then
      M.showSdfIntroWindow()
    end

    if widgets.draw(api.getSdfEnabled(), api.propertiesMap["sdfEnabled"], guiId) then
      api.setSdfEnabled(api.propertiesMap["sdfEnabled"].value)
    end

    if api.getSdfEnabled() then
      if im.TreeNode1(string.format("SDF Texture Preview##DecalColor_%s", guiId)) then
        api.renderSdfTextureImgui(im.GetContentRegionAvailWidth(), im.GetContentRegionAvailWidth())
        im.TreePop()
      end
    end

    if widgets.draw(api.getSdfThickness(), api.propertiesMap["sdfThickness"], guiId) then
      api.setSdfThickness(api.propertiesMap["sdfThickness"].value)
    end

    if widgets.draw(api.getSdfSoftness(), api.propertiesMap["sdfSoftness"], guiId) then
      api.setSdfSoftness(api.propertiesMap["sdfSoftness"].value)
    end

    if widgets.draw(api.getSdfOutlineThickness(), api.propertiesMap["sdfOutlineThickness"], guiId) then
      api.setSdfOutlineThickness(api.propertiesMap["sdfOutlineThickness"].value)
    end

    if widgets.draw(api.getSdfOutlineSoftness(), api.propertiesMap["sdfOutlineSoftness"], guiId) then
      api.setSdfOutlineSoftness(api.propertiesMap["sdfOutlineSoftness"].value)
    end

    if widgets.draw(api.getSdfOutlineColor():toTable(), api.propertiesMap["sdfOutlineColor"], guiId) then
      api.setSdfOutlineColor(ColorI.fromTable(api.propertiesMap["sdfOutlineColor"].value))
    end

    im.TreePop()
  end
end

local function setSdfPropertiesHighlighted()
  tool.setSectionOpenState("Decal Properties", true)
  highlightSdfProperties = {
    setScroll = true,
    timer = 10
  }
end

local function sdfIntroWindow()
  if editor.beginWindow(sdfIntroWindowName, sdfIntroWindowName, im.WindowFlags_AlwaysAutoResize) then
    im.PushTextWrapPos(800)
    im.TextUnformatted("You've selected a SDF compatible texture.")
    docs.verticalSpacing()
    sdfExplanationTextWidget()
    im.PopTextWrapPos()
    if im.Button("Highlight SDF properties") then
      setSdfPropertiesHighlighted()
    end

    docs.verticalSpacing()
    im.Separator()
    if im.Button("OK") then
      M.hideSdfIntroWindow()
      sdfModalHasBeenClosed = true
      -- im.CloseCurrentPopup()
    end
    im.SameLine()
    im.SetCursorPosX(im.GetCursorPosX() + im.GetContentRegionAvailWidth() - (im.CalcTextSize("Do not show again").x + 2*im.GetStyle().ItemSpacing.x + tool.getIconSize()))
    im.TextUnformatted("Do not show again")
    im.SameLine()
    if im.Checkbox("##dynDecals_decal_doNotShowSdfIntroAgainCheckbox", editor.getTempBool_BoolBool(editor.getPreference("dynamicDecalsTool.decalProperties.doNotShowSdfIntroAgain"))) then
      editor.setPreference("dynamicDecalsTool.decalProperties.doNotShowSdfIntroAgain", editor.getTempBool_BoolBool())
    end
    im.tooltip("Do not pop up SDF introduction modal again")
  end
  editor.endWindow()
end

local function fontCharacterSelectionWindowGui()
  if editor.beginWindow(fontCharacterSelectionWindowName, "Dynamic Decals - Font Character Selection") then

  end
  editor.endWindow()
end

local function sectionGui(guiId)
  -- SAVE BRUSH
  if im.BeginPopup("SaveBrushPopup") then
    im.TextUnformatted("Brush name")
    if im.InputText("##SaveBrushNameTextInput", editor.getTempCharPtr(newBrushName), nil, im.InputTextFlags_AutoSelectAll) then
      newBrushName = editor.getTempCharPtr()
    end
    if im.Button("Cancel") then
      im.CloseCurrentPopup()
    end
    im.SameLine()
    if im.Button("Save") then
      brushes.saveBrush(newBrushName)
      im.CloseCurrentPopup()
    end
    im.EndPopup()
  end

  if im.Button("Save as brush") then
    local dir, fileName, fileExt = path.split(api.getDecalTexturePath("color"))
    newBrushName = string.sub(fileName, 1, #fileName - (#fileExt + 1))
    im.OpenPopup("SaveBrushPopup")
  end

  im.PushStyleVar1(im.StyleVar_IndentSpacing, im.GetStyle().IndentSpacing / 4)

  -- PROPERTIES
  local decalScale = api.getDecalScale()
  if widgets.draw({decalScale.x, decalScale.y, decalScale.z}, api.propertiesMap["decalScale"], guiId) then
    api.setDecalScale(vec3(api.propertiesMap["decalScale"].value[1], 1.0, api.propertiesMap["decalScale"].value[3]))
  end

  local style = im.GetStyle()
  local widgetWidth = im.GetContentRegionAvailWidth() - (style.ItemSpacing.x + 2 * style.FramePadding.x + im.CalcTextSize("Inv").x)
  if widgets.draw(api.getDecalRotation(), api.propertiesMap["decalRotation"], guiId, nil, {widthMod = -(style.ItemSpacing.x + 2 * style.FramePadding.x + im.CalcTextSize("Inv").x)}) then
    api.setDecalRotation(api.propertiesMap["decalRotation"].value)
  end
  im.SameLine()
  if im.Button(string.format("Inv##InverseDecalRotation_%s", guiId)) then
    local newVal = api.getDecalRotation() + math.pi
    if newVal > (2 * math.pi) then newVal = newVal - (2 * math.pi) end
    api.setDecalRotation(newVal)
  end
  im.tooltip("Inverse Decal Rotation")

  im.TextUnformatted("Flip")
  im.SameLine()
  local buttonColor = im.GetStyleColorVec4(im.Col_Button)
  local uvValue = api.getDecalUv()
  local btnWidth = (im.GetContentRegionAvailWidth() - im.GetStyle().ItemSpacing.x) / 2
  local enabled = uvValue.x < 0 and true or false
  im.PushStyleColor2(im.Col_Button, enabled and editor.color.beamng.Value or buttonColor)
  if im.Button(string.format("Horizontally##%s_flipHorizontally", guiId), im.ImVec2(btnWidth, 0)) then
    uvValue.x = uvValue.x * -1
    api.setDecalUv(uvValue)
  end
  im.tooltip("Flip decal horizontally.\nEffectively inverses the component of the decal uv value.")
  im.PopStyleColor()
  im.SameLine()

  enabled = uvValue.y < 0 and true or false
  im.PushStyleColor2(im.Col_Button, enabled and editor.color.beamng.Value or buttonColor)
  if im.Button(string.format("Vertically##%s_flipVertically", guiId), im.ImVec2(btnWidth, 0)) then
    uvValue.y = uvValue.y * -1
    api.setDecalUv(uvValue)
  end
  im.tooltip("Flip decal vertically.\nEffectively inverses the component of the decal uv value.")
  im.PopStyleColor()

  if widgets.draw(api.getDecalSkew():toTable(), api.propertiesMap["decalSkew"], guiId) then
    api.setDecalSkew(Point2F.fromTable(api.propertiesMap["decalSkew"].value))
  end

  if widgets.draw(api.getMirrored(), api.propertiesMap["mirrored"], guiId) then
    api.setMirrored(api.propertiesMap["mirrored"].value)
  end
  im.SameLine()
  im.Dummy(im.ImVec2(im.GetStyle().ItemSpacing.x, 0))
  im.SameLine()
  if not api.getMirrored() then im.BeginDisabled() end
  if widgets.draw(api.getFlipMirroredDecal(), api.propertiesMap["flipMirroredDecal"], guiId) then
    api.setFlipMirroredDecal(api.propertiesMap["flipMirroredDecal"].value)
  end
  if not api.getMirrored() then im.EndDisabled() end

  -- MIRROR OFFSET
  local style = im.GetStyle()
  local widthMod = im.CalcTextSize("Debug").x + 4 * style.ItemSpacing.x + 3 * tool.getIconSize() + 4 * style.FramePadding.x
  if widgets.draw(api.getMirrorOffset(), mirrorOffsetProperty, guiId, editor.getTempBool_BoolBool(false), {widthMod = -widthMod}) then
    api.setMirrorOffset(mirrorOffsetProperty.value)
  end
  if editor.getTempBool_BoolBool() == true then
    api.reprojectLayers()
  end
  im.SameLine()
  -- DEBUG MIRROR
  if widgets.draw(api.getMirrorDebug(), mirrorDebugProperty, guiId) then
    api.setMirrorDebug(mirrorDebugProperty.value)
  end

  if im.Checkbox("Project using surface normal", editor.getTempBool_BoolBool(api.getUseSurfaceNormal())) then
    api.setUseSurfaceNormal(editor.getTempBool_BoolBool())
  end
  im.Separator()

  if im.TreeNodeEx1("Textures", im.TreeNodeFlags_DefaultOpen) then

    -- im.BeginDisabled()
    if widgets.draw(api.getDecalLayerFontPath(), api.propertiesMap["decal_fontPath"], guiId) then
    -- if widgets.draw("[WIP]", api.propertiesMap["decal_fontPath"], guiId) then
      api.setDecalLayerFontPath(api.propertiesMap["decal_fontPath"].value)
    end

    im.TextUnformatted("Character")
    im.SameLine()
    im.PushItemWidth(im.GetContentRegionAvailWidth() - (im.CalcTextSize("Select").x + 3 * im.GetStyle().ItemSpacing.x))
    if im.InputText(string.format("##%s_%s", guiId, "decalLayerFontCharacter"), editor.getTempCharPtr(api.getDecalLayerFontCharacter()), nil, im.InputTextFlags_AutoSelectAll) then
      api.setDecalLayerFontCharacter(string.sub(editor.getTempCharPtr(), 1, 1))
    end
    im.PopItemWidth()
    im.SameLine()
    if #api.getDecalLayerFontPath() <= 1 or true then -- always disabled for time being
      im.BeginDisabled()
    else
      fontCharacterSelectionWindowGui()
    end
    if im.Button(string.format("Select##Button_%s_%s", guiId, "decalLayerFontCharacter")) then
      fonts.checkOrGenerateFontBitmaps(api.getDecalLayerFontPath())
      editor.showWindow(fontCharacterSelectionWindowName)
    end
    if #api.getDecalLayerFontPath() <= 1 or true then
      im.EndDisabled()
    end

    if widgets.draw(api.getDecalTexturePath("color"), api.propertiesMap["decalColorTexturePath"], guiId) then
      api.setDecalTexturePath("color", api.propertiesMap["decalColorTexturePath"].value)
      M.checkColorDecalTexturesSdfCompatible()
    end

    if widgets.draw(api.getColorTextureScale():toTable(), api.propertiesMap["colorTextureScale"], guiId) then
      api.setColorTextureScale(Point2F.fromTable(api.propertiesMap["colorTextureScale"].value))
    end

    im.TextUnformatted("Wrap Texture")
    im.SameLine()
    if editor.uiIconImageButton(editor.icons.refresh, tool.getIconSizeVec2(), nil, nil, nil, string.format("%s_resetButton", "wrapColorTextureX")) then
      if api.isWrapColorTextureXEnabled() == false then
        api.toggleSetting(api.settingsFlags.WrapColorTextureX.value)
      end
      if api.isWrapColorTextureYEnabled() == false then
        api.toggleSetting(api.settingsFlags.WrapColorTextureY.value)
      end
    end
    im.tooltip(string.format("Reset to default: %s", dumps({horizontally = true, vertically = true})))
    im.SameLine()
    im.TextUnformatted("Horizontally")
    im.SameLine()
    local enabled = api.isWrapColorTextureXEnabled()
    if im.Checkbox(string.format("##%s_%s_checkbox", guiId, api.settingsFlags.WrapColorTextureX.name), editor.getTempBool_BoolBool(enabled)) then
      api.toggleSetting(api.settingsFlags.WrapColorTextureX.value)
    end

    im.SameLine()
    im.TextUnformatted("Vertically")
    im.SameLine()
    enabled = api.isWrapColorTextureYEnabled()
    if im.Checkbox(string.format("##%s_%s_checkbox", guiId, api.settingsFlags.WrapColorTextureY.name), editor.getTempBool_BoolBool(enabled)) then
      api.toggleSetting(api.settingsFlags.WrapColorTextureY.value)
    end

    -- Color Palete Map Id
    local style = im.GetStyle()
    local colorButtonHeight = math.ceil(im.GetFontSize()) + 2 * style.FramePadding.y
    local colorPaletteMapId = api.getColorPaletteMapId()
    if widgets.draw(colorPaletteMapId, api.propertiesMap["colorPaletteMapId"], guiId, nil, {widthMod = -(style.ItemSpacing.x + colorButtonHeight)}) then
      api.setColorPaletteMapId(api.propertiesMap["colorPaletteMapId"].value)
    end
    im.SameLine()
    local vehicleObj = getPlayerVehicle(0)
    local paletteColor = {1, 1, 1, 0}
    if colorPaletteMapId == 0 then
      paletteColor = api.getDecalColor():toTable()
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

    if colorPaletteMapId ~= 0 or api.isDecalGradientColorEnabled() then im.BeginDisabled() end
    if widgets.draw(api.getDecalColor():toTable(), api.propertiesMap["color"], guiId) then
      api.setDecalColor(Point4F.fromTable(api.propertiesMap["color"].value))
    end
    if colorPaletteMapId ~= 0 or api.isDecalGradientColorEnabled() then im.EndDisabled() end

    if colorPaletteMapId ~= 0 then im.BeginDisabled() end
    if widgets.draw(api.isDecalGradientColorEnabled(), api.propertiesMap["decalUseGradientColor"], guiId) then
      api.toggleSetting(api.settingsFlags.UseGradientColor.value)
    end
    if colorPaletteMapId ~= 0 then im.EndDisabled() end

    if colorPaletteMapId ~= 0 or api.isDecalGradientColorEnabled() == false then im.BeginDisabled() end
    if widgets.draw(
      {api.getGradientColorTopLeft():toTable(), api.getGradientColorTopRight():toTable(), api.getGradientColorBottomLeft():toTable(), api.getGradientColorBottomRight():toTable()},
      api.propertiesMap["decalGradientColor"],
      guiId
    ) then
      local value = api.propertiesMap["decalGradientColor"].value
      api.setGradientColorTopLeft(ColorI(value[1][1], value[1][2], value[1][3], value[1][4]))
      api.setGradientColorTopRight(ColorI(value[2][1], value[2][2], value[2][3], value[2][4]))
      api.setGradientColorBottomLeft(ColorI(value[3][1], value[3][2], value[3][3], value[3][4]))
      api.setGradientColorBottomRight(ColorI(value[4][1], value[4][2], value[4][3], value[4][4]))
    end
    if colorPaletteMapId ~= 0 or api.isDecalGradientColorEnabled() == false then im.EndDisabled() end

    if highlightSdfProperties then
      if highlightSdfProperties.setScroll then
        im.SetScrollHereY()
        highlightSdfProperties.setScroll = false
      end
      highlightSdfProperties.startCursorPos = im.GetCursorPos()
    end
    decalColorSdfPropertiesWidget(guiId)
    if highlightSdfProperties then
      highlightSdfProperties.endCursorPos = im.GetCursorPos()

      local wpos = im.GetWindowPos()
      local cpos = highlightSdfProperties.startCursorPos
      local scrollX = im.GetScrollX()
      local scrollY = im.GetScrollY()
      local p1 = im.ImVec2(wpos.x + highlightSdfProperties.startCursorPos.x - scrollX, wpos.y + highlightSdfProperties.startCursorPos.y - scrollY)
      local p2 = im.ImVec2(wpos.x + highlightSdfProperties.endCursorPos.x - scrollX + im.GetContentRegionAvailWidth(), wpos.y + highlightSdfProperties.endCursorPos.y - scrollY)
      im.ImDrawList_AddRect(im.GetWindowDrawList(), p1, p2, im.GetColorU322(editor.color.beamng.Value), nil, nil, 3)
    end
    im.Separator()

    if widgets.draw(api.getDecalTexturePath("alpha"), api.propertiesMap["decalAlphaTexturePath"], guiId) then
      api.setDecalTexturePath("alpha", api.propertiesMap["decalAlphaTexturePath"].value)
    end

    if widgets.draw(api.getAlphaMaskChannel(), api.propertiesMap["alphaMaskChannel"], guiId) then
      api.setAlphaMaskChannel(api.propertiesMap["alphaMaskChannel"].value)
    end

    if widgets.draw(api.getAlphaMaskBlendMode(), api.propertiesMap["alphaMaskBlendMode"], guiId) then
      api.setAlphaMaskBlendMode(api.propertiesMap["alphaMaskBlendMode"].value)
    end

    if widgets.draw(api.getAlphaMaskIntensity(), api.propertiesMap["alphaMaskIntensity"], guiId) then
      api.setAlphaMaskIntensity(api.propertiesMap["alphaMaskIntensity"].value)
    end

    if widgets.draw(api.getAlphaMaskRotation(), api.propertiesMap["alphaMaskRotation"], guiId) then
      api.setAlphaMaskRotation(api.propertiesMap["alphaMaskRotation"].value)
    end

    if widgets.draw(api.getAlphaMaskScale():toTable(), api.propertiesMap["alphaMaskScale"], guiId) then
      api.setAlphaMaskScale(Point2F.fromTable(api.propertiesMap["alphaMaskScale"].value))
    end

    if widgets.draw(api.isWrapAlphaMaskXEnabled(), api.propertiesMap["wrapAlphaMaskX"], guiId) then
      -- support for default button
      local newValue = api.propertiesMap["wrapAlphaMaskX"].value
      if newValue ~= api.isWrapAlphaMaskXEnabled() then
        api.toggleSetting(api.settingsFlags.WrapAlphaMaskX.value)
      end
    end

    if widgets.draw(api.isWrapAlphaMaskYEnabled(), api.propertiesMap["wrapAlphaMaskY"], guiId) then
      -- support for default button
      local newValue = api.propertiesMap["wrapAlphaMaskY"].value
      if newValue ~= api.isWrapAlphaMaskYEnabled() then
        api.toggleSetting(api.settingsFlags.WrapAlphaMaskY.value)
      end
    end

    if widgets.draw(api.isAlphaMaskInvertEnabled(), api.propertiesMap["alphaMaskInvert"], guiId) then
      -- support for default button
      local newValue = api.propertiesMap["alphaMaskInvert"].value
      if newValue ~= api.isAlphaMaskInvertEnabled() then
        api.toggleSetting(api.settingsFlags.AlphaMaskInvert.value)
      end
    end

    if widgets.draw(api.getAlphaMaskOffset():toTable(), api.propertiesMap["alphaMaskOffset"], guiId) then
      api.setAlphaMaskOffset(Point2F.fromTable(api.propertiesMap["alphaMaskOffset"].value))
    end

    -- Hide blend mode widget for the time being cause WIP
    --[[
      if widgets.draw(api.getBlendMode(), api.propertiesMap["blendMode"], guiId) then
        api.setBlendMode(api.propertiesMap["blendMode"].value)
      end
    ]]

    im.Separator()
    im.Separator()

    if widgets.draw(api.getMetallicIntensity(), api.propertiesMap["metallicIntensity"], guiId) then
      api.setMetallicIntensity(api.propertiesMap["metallicIntensity"].value)
    end

    if widgets.draw(api.getRoughnessIntensity(), api.propertiesMap["roughnessIntensity"], guiId) then
      api.setRoughnessIntensity(api.propertiesMap["roughnessIntensity"].value)
    end

    -- Hide all non color texture widgets for the time being
    --[[
    if im.TreeNode1("Advanced Textures (WIP)") then
        if widgets.draw(api.getDecalTexturePath("normal"), api.propertiesMap["decalNormalTexturePath"], guiId) then
          api.setDecalTexturePath("normal", api.propertiesMap["decalNormalTexturePath"].value)
        end
        if widgets.draw(api.getNormalIntensity(), api.propertiesMap["normalIntensity"], guiId) then
          api.setNormalIntensity(api.propertiesMap["normalIntensity"].value)
        end
        im.Separator()

        if widgets.draw(api.getDecalTexturePath("metallic"), api.propertiesMap["decalMetallicTexturePath"], guiId) then
          api.setDecalTexturePath("metallic", api.propertiesMap["decalMetallicTexturePath"].value)
        end
        if widgets.draw(api.getMetallicIntensity(), api.propertiesMap["metallicIntensity"], guiId) then
          api.setMetallicIntensity(api.propertiesMap["metallicIntensity"].value)
        end
        im.Separator()

        if widgets.draw(api.getDecalTexturePath("roughness"), api.propertiesMap["decalRoughnessTexturePath"], guiId) then
          api.setDecalTexturePath("roughness", api.propertiesMap["decalRoughnessTexturePath"].value)
        end
        if widgets.draw(api.getRoughnessIntensity(), api.propertiesMap["roughnessIntensity"], guiId) then
          api.setRoughnessIntensity(api.propertiesMap["roughnessIntensity"].value)
        end
        im.Separator()

      im.TreePop()
    end
    ]]

    im.TreePop()
  end
  im.PopStyleVar(1)
end

local function registerEditorPreferences(prefsRegistry)
  prefsRegistry:registerSubCategory("dynamicDecalsTool", "decalProperties", nil, {
    {scaleStep = {"float", 0.025, "Decal scale step.", nil, 0.001, 0.25}},
    {rotationStep = {"float", 15, "Decal rotation step in degrees.", nil, 1, 360}},
    {texturePreviewSize = {"float", 128, "Max width of the decal texture thumbnails.", nil, 32, 512}},
    {doNotShowSdfIntroAgain = {"bool", false, "Do not pop up SDF intro modal again"}},
  })
  prefsRegistry:registerSubCategory("dynamicDecalsTool", "inspector", nil, {
    {applyMirroredPropertyToChildren = {"bool", true, "Apply change to all child layers when setting the following properties in the inspector:\n* mirrored\n* flipMirroredDecal"}},
  })
end

local function editorPreferenceValueChanged(path, value)

end

local function onEditorGuiFn()
  sdfIntroWindow()
end

local function editModeUpdate(dtReal, dtSim, dtRaw)
  if highlightSdfProperties then
    highlightSdfProperties.timer = highlightSdfProperties.timer - dtReal
    if highlightSdfProperties.timer <= 0 then
      highlightSdfProperties = nil
    end
  end
end

local function documentationGui(docsSection)
  im.PushTextWrapPos(im.GetContentRegionAvailWidth())
  im.TextUnformatted([[
Decal Layers are the building blocks of your custom liveries, each representing a single decal element.
They offer an extensive range of properties to customize, including color, texture, scale, rotation, mask, mirroring, and more.
With this vast array of options, you can effortlessly fine-tune every aspect of your decals, allowing for endless creative possibilities in designing stunning and unique vehicle liveries.
]])
  im.PopTextWrapPos()
end

local function sdfDocumentationGui(docsSection)
  im.PushTextWrapPos(im.GetContentRegionAvailWidth())
  im.TextUnformatted("What is SDF?")
  docs.verticalSpacing()
  sdfExplanationTextWidget()
  im.PopTextWrapPos()
end

local function decalPropertiesDocumentationGui(docsSection)
  if im.BeginTable("Decal Layer Properties Table", 4, im.flags(im.TableFlags_Resizable, im.TableFlags_Hideable, im.TableFlags_RowBg)) then
    im.TableSetupColumn('Name')
    im.TableSetupColumn('id')
    im.TableSetupColumn('Type')
    im.TableSetupColumn('Highlight')
    im.TableHeadersRow()
    for _, property in ipairs(api.properties.Decal) do
      im.TableNextColumn()
      im.TextUnformatted(property.name)
      if #property.description > 0 then
        im.tooltip(property.description)
      end
      im.TableNextColumn()
      im.TextUnformatted(property.id)
      im.TableNextColumn()
      im.TextUnformatted(api.typesMap[property.type])
      im.TableNextColumn()
      if im.Button(string.format("Highlight##DecalPropertiesTable_%s", property.name)) then
        tool.setSectionOpenState("Decal Properties", true)
        widgets.highlight(string.format("##Decal Properties_section_%s", property.id), 5)
      end
      im.tooltip("- Click to highlight property -")
    end

    im.EndTable()
  end
end

local tblx = {}
local function setup(tool_in)
  tool = tool_in
  api = extensions.editor_api_dynamicDecals
  texturesApi = extensions.editor_api_dynamicDecals_textures
  brushes = extensions.editor_dynamicDecals_brushes
  inspector = extensions.editor_dynamicDecals_inspector
  inspectorUtils = extensions.editor_dynamicDecals_inspector_utils
  helper = extensions.editor_dynamicDecals_helper
  docs = extensions.editor_dynamicDecals_docs
  widgets = extensions.editor_dynamicDecals_widgets
  fonts = extensions.editor_dynamicDecals_fonts

  editor.registerWindow(sdfIntroWindowName, im.ImVec2(400, 400))
  editor.registerWindow(fontCharacterSelectionWindowName, im.ImVec2(550, 550))

  tool.registerSection("Decal Properties", sectionGui, 60, false, {size = im.ImVec2(320, 640)}, {
    {icon = editor.icons.help_outline, tooltip = "Docs", fn = function() docs.selectSection({"Layer Types", "Decal Layers", "Properties"}) end},
  })
  tool.registerEditorOnUpdateFn("decal", editModeUpdate)
  tool.registerOnEditorGuiFn("decal", onEditorGuiFn)
  inspector.registerLayerGui(api.layerTypes.decal, inspectLayerGui)
  docs.register({section = {"Layer Types", "Decal Layers"}, guiFn = documentationGui})
  docs.register({section = {"Layer Types", "Decal Layers", "Properties"}, guiFn = decalPropertiesDocumentationGui})
  docs.register({section = {"Layer Types", "Decal Layers", "SDF"}, guiFn = sdfDocumentationGui})

  tblx = {}
  for _, blendMode in ipairs(api.blendModes) do
    table.insert(tblx, blendMode.name)
  end
  blendModesNamesCharPtr = im.ArrayCharPtrByTbl(tblx)

  mirrorDebugProperty = {id = "mirroredDebug", name = "Debug", description = "Renders a line on the shape indicating the mirror plane", type = api.types.bool, default = false}
  mirrorOffsetProperty = {id = "mirrorOffsetProperty", name = "Mirror Offset", description = "Offsets the mirror plane", type = api.types.float, default = 0, min = -1, max = 1, widgetType = api.widgetTypes[api.types.float].Slider, format = "%.2f"}
end

M.showSdfIntroWindow = function() editor.showWindow(sdfIntroWindowName) end
M.hideSdfIntroWindow = function() editor.hideWindow(sdfIntroWindowName) end

M.registerEditorPreferences = registerEditorPreferences
M.editorPreferenceValueChanged = editorPreferenceValueChanged
M.setup = setup
M.editModeUpdate = editModeUpdate

return M