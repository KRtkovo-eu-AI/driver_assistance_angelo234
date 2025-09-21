-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = "editor_dynamicDecals_brushes"
local im = ui_imgui

-- reference to the editor tool, set in setup()
local tool = nil
local api = nil
local browser = nil
local selection = nil
local decal = nil

local brushes = {}
local brushesPath = "/art/dynamicDecals/brushes/brushes.json"

-- browserTabGui
local brushTextFilter = im.ImGuiTextFilter()
local textFilterWidth = 200

local alphaMaskBlendModes = {
  [0] = "multiply",
  [1] = "add",
}
local alphaMaskChannels = {
  [0] = "red",
  [1] = "green",
  [2] = "blue",
  [3] = "alpha",
}

local brushContextModal_brushId = -1
local openBrushContextPopup = false

local function deleteBrush(index)
  table.remove(brushes, index)
  for _, brush in ipairs(brushes) do
    brush.id = nil
  end
  jsonWriteFile(brushesPath, brushes, true)
end

local function saveBrushesToFile()
  for _, brush in pairs(brushes) do
    if brush["dirty"] then
      brush["dirty"] = nil
    end
  end

  jsonWriteFile(brushesPath, brushes, true)
end

local function saveBrush(name)
  local decalColorTexturePath = api.getDecalTexturePath("color")

  if not name then
    local dir, fileName, fileExt = path.split(decalColorTexturePath)
    name = string.sub(fileName, 1, #fileName - (#fileExt + 1))
  end

  local brush = {
    name = name,
    alphaMaskBlendMode = api.getAlphaMaskBlendMode(),
    alphaMaskChannel = api.getAlphaMaskChannel(),
    alphaMaskIntensity = api.getAlphaMaskIntensity(),
    alphaMaskInvert = api.isAlphaMaskInvertEnabled(),
    alphaMaskRotation = api.getAlphaMaskRotation(),
    alphaMaskScale = api.getAlphaMaskScale():toTable(),
    alphaMaskOffset = api.getAlphaMaskOffset():toTable(),
    blendMode = api.getBlendMode(),
    color = api.getDecalColor():toTable(),
    colorPaletteMapId = api.getColorPaletteMapId(),
    colorTextureScale = api.getColorTextureScale():toTable(),
    decalAlphaTexturePath = api.getDecalTexturePath("alpha"),
    decalColorTexturePath = decalColorTexturePath,
    decalMetallicTexturePath = api.getDecalTexturePath("metallic"),
    decalNormalTexturePath = api.getDecalTexturePath("normal"),
    decalRotation = api.getDecalRotation(),
    decalRoughnessTexturePath = api.getDecalTexturePath("roughness"),
    decalScale = api.getDecalScale():toTable(),
    decalSkew = api.getDecalSkew():toTable(),
    decalUseGradientColor = api.isDecalGradientColorEnabled(),
    decalGradientColorTopLeft = api.getGradientColorTopLeft():toTable(),
    decalGradientColorTopRight = api.getGradientColorTopRight():toTable(),
    decalGradientColorBottomLeft = api.getGradientColorBottomLeft():toTable(),
    decalGradientColorBottomRight = api.getGradientColorBottomRight():toTable(),
    decalUv = api.getDecalUv():toTable(),
    metallicIntensity = api.getMetallicIntensity(),
    mirrored = api.getMirrored(),
    flipMirroredDecal = api.getFlipMirroredDecal(),
    normalIntensity = api.getNormalIntensity(),
    roughnessIntensity = api.getRoughnessIntensity(),
    wrapAlphaMaskX = api.isWrapAlphaMaskXEnabled(),
    wrapAlphaMaskY = api.isWrapAlphaMaskYEnabled(),
    wrapColorTextureX = api.isWrapColorTextureXEnabled(),
    wrapColorTextureY = api.isWrapColorTextureYEnabled(),
  }
  -- sdf
  if decal.isTexturesSdfCompatible(brush.decalColorTexturePath) then
    brush.sdfThickness = api.getSdfThickness()
    brush.sdfSoftness = api.getSdfSoftness()
    brush.sdfOutlineThickness = api.getSdfOutlineThickness()
    brush.sdfOutlineSoftness = api.getSdfOutlineSoftness()
    brush.sdfOutlineColor = api.getSdfOutlineColor():toTable()
  end
  table.insert(brushes, brush)

  saveBrushesToFile()
end

local function getValueOrDefault(propertyName, value)
  local prop = api.propertiesMap[propertyName]
  if not prop then
    editor.logWarn(string.format("%s : No property found for '%s'", logTag, propertyName))
    return value
  end
  if prop.type == api.types.Point2F then
    return Point2F.fromTable(value or prop.default)
  elseif prop.type == api.types.Point3F then
    if value then
      return vec3(value[1], value[2], value[3])
    else
      return vec3(prop.default[1], prop.default[2], prop.default[3])
    end
  elseif prop.type == api.types.Point4F then
    return Point4F.fromTable(value or prop.default)
  elseif prop.type == api.types.ColorI then
    return ColorI.fromTable(value or prop.default)
  end
  return value or prop.default
end

local function loadBrush(brush)
  api.setAlphaMaskBlendMode(getValueOrDefault("alphaMaskBlendMode", brush.alphaMaskBlendMode))
  api.setAlphaMaskChannel(getValueOrDefault("alphaMaskChannel", brush.alphaMaskChannel))
  api.setAlphaMaskIntensity(getValueOrDefault("alphaMaskIntensity", brush.alphaMaskIntensity))
  -- api.isAlphaMaskInvertEnabled(getValueOrDefault("alphaMaskInvert", brush.alphaMaskInvert))
  api.setAlphaMaskRotation(getValueOrDefault("alphaMaskRotation", brush.alphaMaskRotation))
  api.setAlphaMaskScale(getValueOrDefault("alphaMaskScale", brush.alphaMaskScale))
  api.setAlphaMaskOffset(getValueOrDefault("alphaMaskOffset", brush.alphaMaskOffset))
  api.setBlendMode(getValueOrDefault("blendMode", brush.blendMode))
  api.setDecalColor(getValueOrDefault("color", brush.color))
  api.setColorPaletteMapId(getValueOrDefault("colorPaletteMapId", brush.colorPaletteMapId))
  api.setColorTextureScale(getValueOrDefault("colorTextureScale", brush.colorTextureScale))
  api.setDecalTexturePath("alpha", getValueOrDefault("decalAlphaTexturePath", brush.decalAlphaTexturePath))
  api.setDecalTexturePath("color", getValueOrDefault("decalColorTexturePath", brush.decalColorTexturePath))
  api.setDecalTexturePath("metallic", getValueOrDefault("decalMetallicTexturePath", brush.decalMetallicTexturePath))
  api.setDecalTexturePath("normal", getValueOrDefault("decalNormalTexturePath", brush.decalNormalTexturePath))
  api.setDecalRotation(getValueOrDefault("decalRotation", brush.decalRotation))
  api.setDecalTexturePath("roughness", getValueOrDefault("decalRoughnessTexturePath", brush.decalRoughnessTexturePath))
  api.setDecalScale(getValueOrDefault("decalScale", brush.decalScale))
  api.setDecalSkew(getValueOrDefault("decalSkew", brush.decalSkew))
  -- api.isDecalGradientColorEnabled(getValueOrDefault("decalUseGradientColor", brush.decalUseGradientColor))
  api.setGradientColorTopLeft(getValueOrDefault("decalGradientColorTopLeft", brush.decalGradientColorTopLeft))
  api.setGradientColorTopRight(getValueOrDefault("decalGradientColorTopRight", brush.decalGradientColorTopRight))
  api.setGradientColorBottomLeft(getValueOrDefault("decalGradientColorBottomLeft", brush.decalGradientColorBottomLeft))
  api.setGradientColorBottomRight(getValueOrDefault("decalGradientColorBottomRight", brush.decalGradientColorBottomRight))
  api.setDecalUv(getValueOrDefault("decalUv", brush.decalUv))
  api.setMetallicIntensity(getValueOrDefault("metallicIntensity", brush.metallicIntensity))
  api.setMirrored(getValueOrDefault("mirrored", brush.mirrored))
  api.setFlipMirroredDecal(getValueOrDefault("flipMirroredDecal", brush.flipMirroredDecal))
  api.setNormalIntensity(getValueOrDefault("normalIntensity", brush.normalIntensity))
  api.setRoughnessIntensity(getValueOrDefault("roughnessIntensity", brush.roughnessIntensity))
  -- api.isWrapAlphaMaskXEnabled(getValueOrDefault("wrapAlphaMaskX", brush.wrapAlphaMaskX))
  -- api.isWrapAlphaMaskYEnabled(getValueOrDefault("wrapAlphaMaskY", brush.wrapAlphaMaskY))
  -- api.isWrapColorTextureXEnabled(getValueOrDefault("wrapColorTextureX", brush.wrapColorTextureX))
  -- api.isWrapColorTextureYEnabled(getValueOrDefault("wrapColorTextureY", brush.wrapColorTextureY))

  local decalUseGradientColor = getValueOrDefault("decalUseGradientColor", brush.decalUseGradientColor)
  if decalUseGradientColor ~= api.isDecalGradientColorEnabled() then
    api.toggleSetting(api.settingsFlags.UseGradientColor.value)
  end

  local alphaMaskInvert = getValueOrDefault("alphaMaskInvert", brush.alphaMaskInvert)
  if alphaMaskInvert ~= api.isAlphaMaskInvertEnabled() then
    api.toggleSetting(api.settingsFlags.AlphaMaskInvert.value)
  end

  local wrapAlphaMaskX = getValueOrDefault("wrapAlphaMaskX", brush.wrapAlphaMaskX)
  if wrapAlphaMaskX ~= api.isWrapAlphaMaskXEnabled() then
    api.toggleSetting(api.settingsFlags.WrapAlphaMaskX.value)
  end

  local wrapAlphaMaskY = getValueOrDefault("wrapAlphaMaskY", brush.wrapAlphaMaskY)
  if wrapAlphaMaskY ~= api.isWrapAlphaMaskYEnabled() then
    api.toggleSetting(api.settingsFlags.WrapAlphaMaskY.value)
  end

  local wrapColorTextureX = getValueOrDefault("wrapColorTextureX", brush.wrapColorTextureX)
  if wrapColorTextureX ~= api.isWrapColorTextureXEnabled() then
    api.toggleSetting(api.settingsFlags.WrapColorTextureX.value)
  end

  local wrapColorTextureY = getValueOrDefault("wrapColorTextureY", brush.wrapColorTextureY)
  if wrapColorTextureY ~= api.isWrapColorTextureYEnabled() then
    api.toggleSetting(api.settingsFlags.WrapColorTextureY.value)
  end

  if brush.sdfThickness then
    api.setSdfThickness(getValueOrDefault("sdfThickness", brush.sdfThickness))
    api.setSdfSoftness(getValueOrDefault("sdfSoftness", brush.sdfSoftness))
    api.setSdfOutlineThickness(getValueOrDefault("sdfOutlineThickness", brush.sdfOutlineThickness))
    api.setSdfOutlineSoftness(getValueOrDefault("sdfOutlineSoftness", brush.sdfOutlineSoftness))
    api.setSdfOutlineColor(getValueOrDefault("sdfOutlineColor", brush.sdfOutlineColor))
  end


  -- api.setAlphaMaskBlendMode(brush.alphaMaskBlendMode or 0)
  -- api.setAlphaMaskChannel(brush.alphaMaskChannel or 3)
  -- api.setAlphaMaskIntensity(brush.alphaMaskIntensity or 1.0)
  -- api.setAlphaMaskRotation(brush.alphaMaskRotation or 0.0)
  -- api.setAlphaMaskScale(Point2F.fromTable(brush.alphaMaskScale or {1.0, 1.0}))
  -- api.setBlendMode(getValueOrDefault("blendMode", brush.blendMode))
  -- api.setDecalColor(Point4F.fromTable(getValueOrDefault("color", brush.color)))
  -- api.setColorPaletteMapId(getValueOrDefault("colorPaletteMapId", brush.colorPaletteMapId))
  -- api.setColorTextureScale(Point2F.fromTable(brush.colorTextureScale or {1.0, 1.0}))
  -- api.setDecalTexturePath("alpha", getValueOrDefault("decalAlphaTexturePath", brush.decalAlphaTexturePath))
  -- api.setDecalTexturePath("color", getValueOrDefault("decalColorTexturePath", brush.decalColorTexturePath))
  -- api.setGradientColorTopLeft(ColorI.fromTable(getValueOrDefault("decalGradientColorTopLeft", brush.decalGradientColorTopLeft)))
  -- api.setGradientColorTopRight(ColorI.fromTable(getValueOrDefault("decalGradientColorTopRight", brush.decalGradientColorTopRight)))
  -- api.setGradientColorBottomLeft(ColorI.fromTable(getValueOrDefault("decalGradientColorBottomLeft", brush.decalGradientColorBottomLeft)))
  -- api.setGradientColorBottomRight(ColorI.fromTable(getValueOrDefault("decalGradientColorBottomRight", brush.decalGradientColorBottomRight)))
  -- api.setDecalTexturePath("metallic", getValueOrDefault("decalMetallicTexturePath", brush.decalMetallicTexturePath))
  -- api.setDecalTexturePath("normal", getValueOrDefault("decalNormalTexturePath", brush.decalNormalTexturePath))
  -- api.setDecalRotation(getValueOrDefault("decalRotation", brush.decalRotation))
  -- api.setDecalTexturePath("roughness", getValueOrDefault("decalRoughnessTexturePath", brush.decalRoughnessTexturePath))
  -- api.setDecalScale(vec3(brush.decalScale[1], brush.decalScale[2], brush.decalScale[3]))
  -- api.setDecalSkew(Point2F.fromTable(getValueOrDefault("decalSkew", brush.decalSkew)))
  -- api.setMetallicIntensity(brush.metallicIntensity or 1.0)
  -- api.setMirrored(brush.mirrored or false)
  -- api.setNormalIntensity(brush.normalIntensity or 1.0)
  -- api.setRoughnessIntensity(brush.roughnessIntensity or 1.0)


  -- -- brush uses gradient colors and gradient setting is not yet enable hence we wanna toggle the setting
  -- if brush.decalUseGradientColor ~= api.isDecalGradientColorEnabled() then
  --   api.toggleSetting(api.settingsFlags.UseGradientColor.value)
  -- end
  -- api.setDecalUv(Point2F.fromTable(getValueOrDefault("decalUv", brush.decalUv)))
  -- if brush.alphaMaskInvert ~= api.isAlphaMaskInvertEnabled() then
  --   api.toggleSetting(api.settingsFlags.AlphaMaskInvert.value)
  -- end
  -- if brush.wrapAlphaMaskX ~= api.isWrapAlphaMaskXEnabled() then
  --   api.toggleSetting(api.settingsFlags.WrapAlphaMaskX.value)
  -- end
  -- if brush.wrapAlphaMaskY ~= api.isWrapAlphaMaskYEnabled() then
  --   api.toggleSetting(api.settingsFlags.WrapAlphaMaskY.value)
  -- end
  -- if brush.wrapColorTextureX ~= api.isWrapColorTextureXEnabled() then
  --   api.toggleSetting(api.settingsFlags.WrapColorTextureX.value)
  -- end
  -- if brush.wrapColorTextureY ~= api.isWrapColorTextureYEnabled() then
  --   api.toggleSetting(api.settingsFlags.WrapColorTextureY.value)
  -- end
end

local function gradientColorViewer(gradientColorTopLeft_Table, gradientColorTopRight_Table, gradientColorBottomLeft_Table, gradientColorBottomRight_Table)
  local gradientColorTopLeft = {gradientColorTopLeft_Table[1]/255, gradientColorTopLeft_Table[2]/255, gradientColorTopLeft_Table[3]/255, gradientColorTopLeft_Table[4]/255}
  local gradientColorTopRight = {gradientColorTopRight_Table[1]/255, gradientColorTopRight_Table[2]/255, gradientColorTopRight_Table[3]/255, gradientColorTopRight_Table[4]/255}
  local gradientColorBottomLeft = {gradientColorBottomLeft_Table[1]/255, gradientColorBottomLeft_Table[2]/255, gradientColorBottomLeft_Table[3]/255, gradientColorBottomLeft_Table[4]/255}
  local gradientColorBottomRight = {gradientColorBottomRight_Table[1]/255, gradientColorBottomRight_Table[2]/255, gradientColorBottomRight_Table[3]/255, gradientColorBottomRight_Table[4]/255}
  local gradientColorTopLeftU32 = im.GetColorU322(editor.getTempImVec4_TableTable(gradientColorTopLeft))
  local gradientColorTopRightU32 = im.GetColorU322(editor.getTempImVec4_TableTable(gradientColorTopRight))
  local gradientColorBottomLeftU32 = im.GetColorU322(editor.getTempImVec4_TableTable(gradientColorBottomLeft))
  local gradientColorBottomRightU32 = im.GetColorU322(editor.getTempImVec4_TableTable(gradientColorBottomRight))

  local size = im.GetContentRegionAvailWidth() / 2 > 256 and 256 or im.GetContentRegionAvailWidth() / 2

  local cursorPos = im.GetCursorPos()
  im.SetCursorPos(im.ImVec2(cursorPos.x, cursorPos.y + size - 20))

  im.ColorButton("##colorBL", editor.getTempImVec4_TableImVec4(gradientColorBottomLeft), im.ColorEditFlags_AlphaPreview)

  im.SetCursorPos(cursorPos)
  im.ColorButton("##colorTL", editor.getTempImVec4_TableImVec4(gradientColorTopLeft), im.ColorEditFlags_AlphaPreview)
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
  im.ColorButton("##colorBR", editor.getTempImVec4_TableImVec4(gradientColorBottomRight), im.ColorEditFlags_AlphaPreview)

  im.SetCursorPos(cursorPos)
  im.ColorButton("##colorTR", editor.getTempImVec4_TableImVec4(gradientColorTopRight), im.ColorEditFlags_AlphaPreview)
  im.SetCursorPos(im.ImVec2(cursorPos.x, cursorPos.y + size + im.GetStyle().ItemSpacing.y))
  im.NewLine()
end

local function inspectorGui(brush)
  if im.Button("Use") then
    loadBrush(brush)
  end
  im.SameLine()
  if im.Button("Delete") then
    editor.selection["dynamicDecalBrush"] = nil
    deleteBrush(brush.id)
  end
  im.SameLine()
  if not brush.dirty then im.BeginDisabled() end
  local saveBrushButtonPressed = im.Button("Save##BrushInspector")
  if not brush.dirty then im.EndDisabled() end
  if saveBrushButtonPressed then saveBrushesToFile() end
  im.tooltip("save changes")

  if editor.getPreference("dynamicDecalsTool.general.debug") then
    im.SameLine()
    if im.Button("dump") then
      dump(brush)
    end
  end
  im.Separator()

  local imageWidth = im.GetContentRegionAvailWidth() / 2 > 256 and 256 or im.GetContentRegionAvailWidth() / 2
  if im.TreeNodeEx1("properties##inspectorGuiBrush", im.TreeNodeFlags_DefaultOpen) then
    im.Columns(2, "InspectorGuiBrush_PropertiesColumns")

    im.TextUnformatted("name")
    im.NextColumn()
    if im.InputText("##brushNameTextInput_inspectorGuiBrush", editor.getTempCharPtr(brush.name)) then
      brush.name = editor.getTempCharPtr()
      brush.dirty = true
    end
    im.NextColumn()

    im.TextUnformatted("alphaMaskBlendMode")
    im.NextColumn()
    im.TextUnformatted(string.format("%d (%s)", brush.alphaMaskBlendMode or 0, alphaMaskBlendModes[brush.alphaMaskBlendMode or 0]))
    im.NextColumn()

    im.TextUnformatted("alphaMaskChannel")
    im.NextColumn()
    im.TextUnformatted(string.format("%d (%s)", brush.alphaMaskChannel or 0, alphaMaskChannels[brush.alphaMaskChannel or 0]))
    im.NextColumn()

    im.TextUnformatted("alphaMaskIntensity")
    im.NextColumn()
    im.TextUnformatted(tostring(brush.alphaMaskIntensity))
    im.NextColumn()

    im.TextUnformatted("alphaMaskRotation")
    im.NextColumn()
    im.TextUnformatted(string.format("%.2f", brush.alphaMaskRotation or 0.0))
    im.NextColumn()

    im.TextUnformatted("alphaMaskScale")
    im.NextColumn()
    im.TextUnformatted(string.format("[%.2f, %.2f]", brush.alphaMaskScale[1], brush.alphaMaskScale[2]))
    im.NextColumn()

    im.TextUnformatted("alphaMaskInvert")
    im.NextColumn()
    im.TextUnformatted(string.format("%s", ((brush.alphaMaskInvert or false) and "true" or "false")))
    im.NextColumn()

    im.TextUnformatted("blendMode")
    im.NextColumn()
    im.TextUnformatted(string.format("%d (%s)", brush.blendMode, api.blendModes[brush.blendMode + 1].name))
    im.NextColumn()

    im.TextUnformatted("colorPaletteMapId")
    im.NextColumn()
    im.TextUnformatted(tostring(brush.colorPaletteMapId))
    im.NextColumn()

    im.TextUnformatted("colorTextureScale")
    im.NextColumn()
    im.TextUnformatted(string.format("[%.2f, %.2f]", brush.colorTextureScale[1], brush.colorTextureScale[2]))
    im.NextColumn()

    im.TextUnformatted("decalRotation")
    im.NextColumn()
    im.TextUnformatted(string.format("%.2f", brush.decalRotation))
    im.NextColumn()

    im.TextUnformatted("decalScale")
    im.NextColumn()
    im.TextUnformatted(string.format("[%.2f, %.2f, %.2f]", brush.decalScale[1], brush.decalScale[2], brush.decalScale[3]))
    im.NextColumn()

    im.TextUnformatted("decalSkew")
    im.NextColumn()
    im.TextUnformatted(string.format("[%.2f, %.2f]", brush.decalSkew[1], brush.decalSkew[2]))
    im.NextColumn()

    im.TextUnformatted("decalUv (flipped decal)")
    im.NextColumn()
    im.TextUnformatted(string.format("[%.1f, %.1f]", brush.decalUv[1], brush.decalUv[2]))
    im.NextColumn()

    im.TextUnformatted("alphaMaskInvert")
    im.NextColumn()
    im.TextUnformatted(string.format("%s", (brush.alphaMaskInvert and "true" or "false")))
    im.NextColumn()

    im.TextUnformatted("mirrored")
    im.NextColumn()
    im.TextUnformatted(string.format("%s", (brush.mirrored or false) and "true" or "false"))
    im.NextColumn()

    im.TextUnformatted("flipMirroredDecal")
    im.NextColumn()
    im.TextUnformatted(string.format("%s", (brush.flipMirroredDecal or false) and "true" or "false"))
    im.NextColumn()

    im.TextUnformatted("metallicIntensity")
    im.NextColumn()
    im.TextUnformatted(string.format("%.2f", brush.metallicIntensity or 1.0))
    im.NextColumn()

    im.TextUnformatted("normalIntensity")
    im.NextColumn()
    im.TextUnformatted(string.format("%.2f", brush.normalIntensity or 1.0))
    im.NextColumn()

    im.TextUnformatted("roughnessIntensity")
    im.NextColumn()
    im.TextUnformatted(string.format("%.2f", brush.roughnessIntensity or 1.0))
    im.NextColumn()

    im.TextUnformatted("wrapAlphaMaskX")
    im.NextColumn()
    im.TextUnformatted(string.format("%s", (brush.wrapAlphaMaskX and "true" or "false")))
    im.NextColumn()

    im.TextUnformatted("wrapAlphaMaskY")
    im.NextColumn()
    im.TextUnformatted(string.format("%s", (brush.wrapAlphaMaskY and "true" or "false")))
    im.NextColumn()

    im.TextUnformatted("wrapColorTextureX")
    im.NextColumn()
    im.TextUnformatted(string.format("%s", (brush.wrapColorTextureX and "true" or "false")))
    im.NextColumn()

    im.TextUnformatted("wrapColorTextureY")
    im.NextColumn()
    im.TextUnformatted(string.format("%s", (brush.wrapColorTextureY and "true" or "false")))
    im.NextColumn()

    im.Columns(1)
    im.TreePop()
  end
  if im.TreeNodeEx1("textures##inspectorGuiBrush", im.TreeNodeFlags_DefaultOpen) then
    if im.TreeNodeEx1("color##inspectorGuiBrush", im.TreeNodeFlags_DefaultOpen) then
      im.TextUnformatted("texture path:")
      im.SameLine()
      im.TextUnformatted(brush.decalColorTexturePath)
      im.Image(editor.getTempTextureObj(brush.decalColorTexturePath).texId, im.ImVec2(imageWidth, imageWidth), im.ImVec2(0,0), im.ImVec2(1,1), nil, editor.color.beamng.Value)
      im.TextUnformatted("Decal Color")
      im.TextUnformatted("Use color gradient: " .. (brush.decalUseGradientColor and "true" or "false"))
      if brush.decalUseGradientColor then
        gradientColorViewer(brush.decalGradientColorTopLeft, brush.decalGradientColorTopRight, brush.decalGradientColorBottomLeft, brush.decalGradientColorBottomRight)
      else
        im.ColorEdit4("##inspectorDecalColor" .. tostring(k), editor.getTempFloatArray4_TableTable(brush.color))
      end
      im.TreePop()
    end
    if im.TreeNodeEx1("normal##inspectorGuiBrush", im.TreeNodeFlags_DefaultOpen) then
      im.TextUnformatted("texture path:")
      im.SameLine()
      im.TextUnformatted(brush.decalNormalTexturePath)
      im.Image(editor.getTempTextureObj(brush.decalNormalTexturePath).texId, im.ImVec2(imageWidth, imageWidth), im.ImVec2(0,0), im.ImVec2(1,1), nil, editor.color.beamng.Value)
      im.TreePop()
    end
    if im.TreeNodeEx1("metallic##inspectorGuiBrush", im.TreeNodeFlags_DefaultOpen) then
      im.TextUnformatted("texture path:")
      im.SameLine()
      im.TextUnformatted(brush.decalMetallicTexturePath)
      im.Image(editor.getTempTextureObj(brush.decalMetallicTexturePath).texId, im.ImVec2(imageWidth, imageWidth), im.ImVec2(0,0), im.ImVec2(1,1), nil, editor.color.beamng.Value)
      im.TreePop()
    end
    if im.TreeNodeEx1("roughness##inspectorGuiBrush", im.TreeNodeFlags_DefaultOpen) then
      im.TextUnformatted("texture path:")
      im.SameLine()
      im.TextUnformatted(brush.decalRoughnessTexturePath)
      im.Image(editor.getTempTextureObj(brush.decalRoughnessTexturePath).texId, im.ImVec2(imageWidth, imageWidth), im.ImVec2(0,0), im.ImVec2(1,1), nil, editor.color.beamng.Value)
      im.TreePop()
    end
    if im.TreeNodeEx1("alpha mask##inspectorGuiBrush", im.TreeNodeFlags_DefaultOpen) then
      im.TextUnformatted("texture path:")
      im.SameLine()
      im.TextUnformatted(brush.decalAlphaTexturePath)
      im.Image(editor.getTempTextureObj(brush.decalAlphaTexturePath).texId, im.ImVec2(imageWidth, imageWidth), im.ImVec2(0,0), im.ImVec2(1,1), nil, editor.color.beamng.Value)
      im.TreePop()
    end
    im.TreePop()
  end

  if im.TreeNode1("data##inspectorGuiBrush") then
    im.TextUnformatted(dumps(brush))
    im.TreePop()
  end
end

local function selectBrush(id, brush)
  brush.id = id
  selection.deselectLayer()
  editor.selection = {}
  editor.selection["dynamicDecalBrush"] = brush
end

local function checkFilter_brush(name)
  if im.ImGuiTextFilter_PassFilter(brushTextFilter, name) then
    return true
  end
  return false
end

local function browserTabGui()
  local brushesData = M.getBrushes()

  if im.BeginPopup("DynDecal_Browser_BrushesTab_BrushPopup") then
    if im.Button("Select Brush##BrushContextModal") then
      selectBrush(brushContextModal_brushId, brushesData[brushContextModal_brushId])
      im.CloseCurrentPopup()
    end
    if im.Button("Load Brush##BrushContextModal") then
      loadBrush(brushesData[brushContextModal_brushId])
      im.CloseCurrentPopup()
    end
    if im.Button("Delete Brush##BrushContextModal") then
      deleteBrush(brushContextModal_brushId)
      im.CloseCurrentPopup()
    end
    if editor.getPreference("dynamicDecalsTool.general.debug") then
      if im.Button("Dump Brush##BrushContextModal") then
        dump(brushesData[brushContextModal_brushId])
        im.CloseCurrentPopup()
      end
    end

    im.EndPopup()
  end

  if openBrushContextPopup then
    im.OpenPopup("DynDecal_Browser_BrushesTab_BrushPopup")
    openBrushContextPopup = false
  end

  local spaceAvailable = im.GetContentRegionAvail()

  im.BeginChild1("BrowserBrushesChild", im.ImVec2(spaceAvailable.x, spaceAvailable.y - (im.GetStyle().ItemSpacing.y +  1 * math.ceil(im.GetFontSize()) + 3)), true)
  local thumbnailSize = editor.getPreference("dynamicDecalsTool.textureBrowser.texturePreviewSize")


  if #brushesData == 0 then
    im.TextUnformatted("There are no brushes.")
  else
    for k, brush in ipairs(brushesData) do
      if checkFilter_brush(brush.name) then
        im.PushID1("Brush_" .. tostring(k))
        if im.ImageButton("##TabGuiButton", editor.getTempTextureObj(brush.decalColorTexturePath).texId, im.ImVec2(thumbnailSize, thumbnailSize), im.ImVec2Zero, im.ImVec2One, nil, (editor.selection["dynamicDecalBrush"] and editor.selection["dynamicDecalBrush"].id == k) and editor.color.beamng.Value or nil) then
          selectBrush(k, brush)
        end
        im.PopID()
        if im.IsItemHovered() and im.IsMouseDoubleClicked(0) then
          loadBrush(brush)
        end
        if im.IsItemClicked(1) then
          brushContextModal_brushId = k
          openBrushContextPopup = true
        end
        im.tooltip(brush.name or "")
        im.SameLine()
        if im.GetContentRegionAvailWidth() < thumbnailSize then
          im.NewLine()
        end
      end
    end
  end
  im.EndChild()

  local cPos = im.GetCursorPos()
  local textSpace = im.GetContentRegionAvailWidth() - (im.GetStyle().ItemSpacing.x + textFilterWidth)
  im.PushTextWrapPos(textSpace)
  im.TextUnformatted("LMB: Select brush; Double-click: Load brush; RMB: Open brush context menu")
  im.SetCursorPos(im.ImVec2(cPos.x + textSpace, cPos.y))
  editor.uiInputSearchTextFilter("Brush Filter", brushTextFilter, textFilterWidth)
  im.PopTextWrapPos()
end

local function getBrushes()
  return brushes
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
  browser = extensions.editor_dynamicDecals_browser
  selection = extensions.editor_dynamicDecals_selection
  decal = extensions.editor_dynamicDecals_layerTypes_decal

  if FS:fileExists(brushesPath) then
    brushes = jsonReadFile(brushesPath)
  else
    brushes = {}
    -- jsonWriteFile(brushesPath, brushes, true)
  end

  browser.registerBrowserTab("Brushes", browserTabGui, 30)
end

M.inspectorGui = inspectorGui
M.loadBrush = loadBrush
M.saveBrush = saveBrush
M.getBrushes = getBrushes
M.registerEditorPreferences = registerEditorPreferences
M.editorPreferenceValueChanged = editorPreferenceValueChanged
M.setup = setup

return M