-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = "editor_dynamicDecals_layerTypes_brushStroke"
local im = ui_imgui

-- reference to the editor tool, set in setup()
local tool = nil
-- reference to the dynamics decal api
local api = nil
local helper = nil
local gizmo = nil
local inspector = nil
local inspectorUtils = nil
local docs = nil
local widgets = nil

local function inspectLayerGui(layer, guiId)
  local vehicleObj = getPlayerVehicle(0)

  im.Columns(2, "layerDataColumns")
  im.TextUnformatted("uid")
  im.NextColumn()
  im.TextUnformatted(layer.uid)
  im.NextColumn()

  im.TextUnformatted("name")
  im.NextColumn()
  im.PushItemWidth(im.GetContentRegionAvailWidth())
  if editor.uiInputText(
    string.format("##%s_%s_%s", layer.uid, guiId, "layerName"),
    editor.getTempCharPtr(layer.name),
    nil,
    im.InputTextFlags_AutoSelectAll,
    nil,
    nil,
    editor.getTempBool_BoolBool(false)
  ) then
    layer.name = editor.getTempCharPtr()
  end
  im.PopItemWidth()
  if editor.getTempBool_BoolBool() == true then
    api.setLayer(layer, false)
  end
  im.NextColumn()

  im.TextUnformatted("enabled")
  im.NextColumn()
  if im.Checkbox(string.format("##%s_%s_%s", layer.uid, guiId, "enabled"), editor.getTempBool_BoolBool(layer.enabled)) then
    layer.enabled = editor.getTempBool_BoolBool()
    api.setLayer(layer, true)
  end
  im.NextColumn()

  im.TextUnformatted("type")
  im.NextColumn()
  im.TextUnformatted(string.format("%s layer", api.layerTypesMap[layer.type]))
  im.NextColumn()

  im.TextUnformatted("camera position")
  im.NextColumn()
  im.PushItemWidth(im.GetContentRegionAvailWidth())
  if im.InputFloat3(string.format("##%s_%s_%s", layer.uid, guiId, "camPosition"), editor.getTempFloatArray3_Vec3Vec3(layer.camPosition), "%.6f") then
    layer.camPosition = editor.getTempFloatArray3_Vec3Vec3()
    api.setLayer(layer, true)
  end
  im.PopItemWidth()
  im.NextColumn()

  im.TextUnformatted("colorPaletteMapId")
  im.NextColumn()
  im.PushItemWidth(im.GetContentRegionAvailWidth() - ((layer.colorPaletteMapId > 0) and (im.GetStyle().ItemSpacing.x + math.ceil(im.GetFontSize()) + 2 * im.GetStyle().FramePadding.y) or 0))
  if im.Combo2(string.format("##%s_%s_%s", layer.uid, guiId, "colorpalettemapid"), editor.getTempInt_NumberNumber(layer.colorPaletteMapId), "zero\0one\0two\0three\0\0") then
    layer.colorPaletteMapId = editor.getTempInt_NumberNumber()
    api.setLayer(layer, true)
  end
  if layer.colorPaletteMapId > 0 then
    local col = {1,1,1,1}
    if layer.colorPaletteMapId == 1 then
      local c = string.split(getVehicleColor())
      col = {tonumber(c[1]), tonumber(c[2]), tonumber(c[3]), 1}
    elseif layer.colorPaletteMapId == 2 then
      local c = string.split(getVehicleColorPalette(0))
      col = {tonumber(c[1]), tonumber(c[2]), tonumber(c[3]), 1}
    elseif layer.colorPaletteMapId == 3 then
      local c = string.split(getVehicleColorPalette(1))
      col = {tonumber(c[1]), tonumber(c[2]), tonumber(c[3]), 1}
    end
    im.SameLine()
    im.ColorButton(string.format("##%s_%s_%s", layer.uid, guiId, "colorpalettemapidbutton"), editor.getTempImVec4_TableTable(col))
    im.tooltip("Vehicle color palette color")
  end
  im.NextColumn()

  if layer.colorPaletteMapId == 0 then
    im.TextUnformatted("decal - use gradient color")
    im.NextColumn()
    if im.Checkbox(string.format("##%s_%s_%s", layer.uid, guiId, "decalUseGradientColor"), editor.getTempBool_BoolBool(layer.decalUseGradientColor)) then
      layer.decalUseGradientColor = editor.getTempBool_BoolBool()
      api.setLayer(layer, true)
    end
    im.NextColumn()

    if layer.decalUseGradientColor then
      im.TextUnformatted("decal gradient color")
      im.NextColumn()
      inspectorUtils.decalColorGradientWidgetInspect(k, layer, guiId)
      im.NextColumn()
    else
      im.TextUnformatted("color")
      im.NextColumn()
      im.PushItemWidth(im.GetContentRegionAvailWidth())
      if editor.uiColorEdit4(string.format("##%s_%s_%s", layer.uid, guiId, "color"), editor.getTempFloatArray4_TableTable(layer.color:toTable()), nil, editor.getTempBool_BoolBool(false)) then
        layer.color = Point4F.fromTable(editor.getTempFloatArray4_TableTable())
      end
      im.PopItemWidth()
      if editor.getTempBool_BoolBool() == true then
        api.setLayer(layer, true)
      end
      im.NextColumn()
    end
  end

  im.TextUnformatted("decal scale")
  im.NextColumn()
  im.PushItemWidth(im.GetContentRegionAvailWidth())
  if editor.uiSliderFloat3(string.format("##%s_%s_%s", layer.uid, guiId, "decalScale"), editor.getTempFloatArray3_Vec3Vec3(layer.decalScale), 0.05, 6.0, nil, nil, editor.getTempBool_BoolBool(false)) then
    layer.decalScale = editor.getTempFloatArray3_Vec3Vec3()
  end
  im.PopItemWidth()
  if editor.getTempBool_BoolBool() == true then
    api.setLayer(layer, true)
  end
  im.NextColumn()

  im.TextUnformatted("decal rotation")
  im.NextColumn()
  im.PushItemWidth(im.GetContentRegionAvailWidth())
  if editor.uiSliderFloat(string.format("##%s_%s_%s", layer.uid, guiId, "decalRotation"), editor.getTempFloat_NumberNumber(layer.decalRotation * 180 / math.pi), 0, 360, nil, nil, editor.getTempBool_BoolBool(false)) then
    layer.decalRotation = (editor.getTempFloat_NumberNumber() / 180 * math.pi)
  end
  im.PopItemWidth()
  if editor.getTempBool_BoolBool() == true then
    api.setLayer(layer, true)
  end
  im.NextColumn()

  im.TextUnformatted("decal uv / flipped")
  im.NextColumn()
  local buttonColor = im.GetStyleColorVec4(im.Col_Button)
  local uvValue = api.getDecalUv()
  local enabled = layer.decalUv.x < 0 and true or false
  im.PushStyleColor2(im.Col_Button, enabled and editor.color.beamng.Value or buttonColor)
  if im.Button("Hor") then
    layer.decalUv = Point2F(layer.decalUv.x * -1, layer.decalUv.y)
    api.setLayer(layer, true)
  end
  im.tooltip("Flip decal horizontally")
  im.PopStyleColor()
  im.SameLine()
  enabled = layer.decalUv.y < 0 and true or false
  im.PushStyleColor2(im.Col_Button, enabled and editor.color.beamng.Value or buttonColor)
  if im.Button("Vert") then
    layer.decalUv = Point2F(layer.decalUv.x, layer.decalUv.y * -1)
    api.setLayer(layer, true)
  end
  im.tooltip("Flip decal vertically")
  im.PopStyleColor()
  im.NextColumn()

  im.TextUnformatted("decal skew")
  im.NextColumn()
  im.PushItemWidth(im.GetContentRegionAvailWidth())
  if editor.uiSliderFloat2(string.format("##%s_%s_%s", layer.uid, guiId, "decalSkew"), editor.getTempFloatArray2_TableTable({layer.decalSkew.x, layer.decalSkew.y}), -2.0, 2.0, nil, nil, editor.getTempBool_BoolBool(false)) then
    local value = editor.getTempFloatArray2_TableTable()
    layer.decalSkew = Point2F(value[1], value[2])
  end
  im.PopItemWidth()
  if editor.getTempBool_BoolBool() == true then
    api.setLayer(layer, true)
  end
  im.NextColumn()

  im.TextUnformatted("mirrored")
  im.NextColumn()
  if im.Checkbox(string.format("##%s_%s_%s", layer.uid, guiId, "mirrored"), editor.getTempBool_BoolBool(layer.mirrored)) then
    layer.mirrored = editor.getTempBool_BoolBool()
    api.setLayer(layer, true)
  end
  im.NextColumn()

  im.TextUnformatted("flipMirroredDecal")
  im.NextColumn()
  if im.Checkbox(string.format("##%s_%s_%s", layer.uid, guiId, "flipMirroredDecal"), editor.getTempBool_BoolBool(layer.flipMirroredDecal)) then
    layer.flipMirroredDecal = editor.getTempBool_BoolBool()
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

  im.TextUnformatted("wrap color mask X")
  im.NextColumn()
  if im.Checkbox(string.format("##%s_%s_%s", layer.uid, guiId, "wrapColorTextureX"), editor.getTempBool_BoolBool(layer.wrapColorTextureX)) then
    layer.wrapColorTextureX = editor.getTempBool_BoolBool()
    api.setLayer(layer, true)
  end
  im.NextColumn()

  im.TextUnformatted("wrap color mask Y")
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

  im.TextUnformatted("interpolationSteps")
  helper.iconTooltip("Number of decals placed in-between decals of the brush stroke in order to make the stroke appear smoother.", true)
  im.NextColumn()
  im.PushItemWidth(im.GetContentRegionAvailWidth())
  if editor.uiInputInt(string.format("##%s_%s_%s", layer.uid, guiId, "interpolationSteps"), editor.getTempInt_NumberNumber(layer.interpolationSteps), 1, 2) then
    local value = editor.getTempInt_NumberNumber()
    if value < 0 then value = 0 end
    layer.interpolationSteps = value
    api.setLayer(layer, true)
  end
  im.PopItemWidth()
  im.NextColumn()

  if layer.sdfThickness then
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
  end

  im.TextUnformatted("decal color texture path")
  im.NextColumn()
  inspectorUtils.decalTextureWidgetInspect(layer, "decalColorTexturePath", guiId)
  im.NextColumn()

  im.TextUnformatted("decal alpha texture path")
  im.NextColumn()
  inspectorUtils.decalTextureWidgetInspect(layer, "decalAlphaTexturePath", guiId)
  im.NextColumn()

  im.TextUnformatted("decal normal texture path")
  im.NextColumn()
  inspectorUtils.decalTextureWidgetInspect(layer, "decalNormalTexturePath", guiId, "/art/decals/dynDecals/_normal.png")
  im.NextColumn()

  im.TextUnformatted("decal metallic texture path")
  im.NextColumn()
  inspectorUtils.decalTextureWidgetInspect(layer, "decalMetallicTexturePath", guiId)
  im.NextColumn()

  im.TextUnformatted("decal roughness texture path")
  im.NextColumn()
  inspectorUtils.decalTextureWidgetInspect(layer, "decalRoughnessTexturePath", guiId)
  im.NextColumn()

  im.TextUnformatted("blend mode")
  im.NextColumn()
  im.TextUnformatted(api.blendModes[layer.blendMode + 1].name)
  im.NextColumn()

  im.TextUnformatted("worldToViewToScreen")
  im.NextColumn()
  im.TextUnformatted(layer.worldToViewToScreen:__tostring())
  im.NextColumn()

  im.TextUnformatted("dataPoints")
  im.NextColumn()
  if im.TreeNode1("Data##dataPointsTreeNode") then
    local count = #layer.dataPoints
    im.TextUnformatted(string.format("count: %d", count))

    editor.uiButtonRightAlign(string.format("Show data points##%s", layer.uid), nil, true)
    if im.IsItemHovered() then
      local layerDataCopy = {
        zBufferDepth = layer.zBufferDepth,
        camPosition = layer.camPosition,
        camDirection = layer.camDirection,
        worldToViewToScreen = layer.worldToViewToScreen
      }

      for i, point in ipairs(layer.dataPoints) do
        layerDataCopy.cursorPosScreenUv = {
          x = layer.dataPoints[i].x,
          y = layer.dataPoints[i].y
        }
        local pos = api.getDecalLocalPos(layerDataCopy) + vehicleObj:getTransform():getPosition()
        local col = editor.getPreference("dynamicDecalsTool.general.dataPointSphereColor")
        debugDrawer:drawSphere(pos, editor.getPreference("dynamicDecalsTool.general.dataPointSphereSize"), ColorF(col[1], col[2], col[3], col[4]), col[4] < 0.99 and true or false)
        debugDrawer:drawTextAdvanced(pos, String(string.format("  %d  ", i)), ColorF(1,1,0,1), true, false, ColorI(40, 40, 40, 0.75*255))
      end
    end

    for k, data in ipairs(layer.dataPoints) do
      im.TextUnformatted(tostring(k))
      im.SameLine()
      if editor.uiIconImageButton(editor.icons.delete, im.ImVec2(tool.getIconSize(), tool.getIconSize()), nil, nil, nil, string.format("##%s_%s_%s_%d", layer.uid, guiId, "interpolationSteps_removedataPointsEntry", k)) then
        table.remove(layer.dataPoints, k)
        api.setLayer(layer, true)
      end
      im.tooltip("Remove entry")
      im.SameLine()
      if k == count then im.BeginDisabled() end
      if editor.uiIconImageButton(editor.icons.content_copy, im.ImVec2(tool.getIconSize(), tool.getIconSize()), nil, nil, nil, string.format("##%s_%s_%s_%d", layer.uid, guiId, "interpolationSteps_insertdataPointsEntry", k)) then
        local nextPoint = layer.dataPoints[k + 1]
        local dupeData = {x = (data.x + nextPoint.x) / 2, y = (data.y + nextPoint.y) / 2}
        table.insert(layer.dataPoints, k + 1, dupeData)
        api.setLayer(layer, true)
      end
      im.tooltip("Insert entry")
      if k == count then im.EndDisabled() end
      im.SameLine()

      if editor.uiIconImageButton(editor.icons.move, im.ImVec2(tool.getIconSize(), tool.getIconSize()), (gizmo.data.uid == layer.uid and gizmo.data.dataPointIndex == k) and editor.color.beamng.Value or nil, nil, nil, string.format("MoveBrushStrokePoint_%s_%d", layer.uid, k)) then
        local layerData = deepcopy(layer)
        local vehPos = vehicleObj:getTransform():getPosition()
        api.projectDynamicDecals = false
        layerData.cursorPosScreenUv = {x = data.x, y = data.y}
        local gizmoPos = api.getDecalLocalPos(layerData) + vehicleObj:getTransform():getPosition()
        gizmo.transform:setPosition(gizmoPos)
        gizmo.data.debugObjPos = gizmoPos
        editor.setAxisGizmoTransform(gizmo.transform)

        gizmo.translateFn = function(newGizmoTransform)
          local vehPos = vehicleObj:getTransform():getPosition()
          api.setDecalLocalPos(layerData, newGizmoTransform:getPosition() - vehPos)
          layerData.dataPoints[k] = deepcopy(layerData.cursorPosScreenUv)
          api.setLayer(layerData, true)
          data.debugObjPos = api.getDecalLocalPos(layerData) + vehPos
        end
        gizmo.data.uid = layerData.uid
        gizmo.data.dataPointIndex = k
        gizmo.data.type = "drag"
        gizmo.data.objectType = "path_dataPoint"
        gizmo.setTransformMode(gizmo.transformModes.translate)
      end
      im.tooltip("Move data point")
      if im.IsItemHovered() then
        local layerDataCopy = {
          zBufferDepth = layer.zBufferDepth,
          camPosition = layer.camPosition,
          camDirection = layer.camDirection,
          worldToViewToScreen = layer.worldToViewToScreen,
          cursorPosScreenUv = {
            x = layer.dataPoints[k].x,
            y = layer.dataPoints[k].y
          }
        }
        local pos = api.getDecalLocalPos(layerDataCopy)
        local col = editor.getPreference("dynamicDecalsTool.general.dataPointSphereColor")
        debugDrawer:drawSphere(pos, editor.getPreference("dynamicDecalsTool.general.dataPointSphereSize"), ColorF(col[1], col[2], col[3], col[4]), col[4] < 0.99 and true or false)
        debugDrawer:drawTextAdvanced(pos, String(string.format("  %d  ", k)), ColorF(1,1,0,1), true, false, ColorI(40, 40, 40, 0.75*255))
      end

      im.SameLine()
      im.PushItemWidth(im.GetContentRegionAvailWidth())
      if editor.uiSliderFloat2(string.format("##%s_%s_%s_%d", layer.uid, guiId, "brushdataPoints_cursorPosScreenUv", k), editor.getTempFloatArray2_TableTable({data.x, data.y}), 0.0, 1.0, nil, nil, editor.getTempBool_BoolBool(false)) then
        local value = editor.getTempFloatArray2_TableTable()
        data.x = value[1]
        data.y = value[2]
        api.setLayer(layer, false)
      end
      im.PopItemWidth()
      if editor.getTempBool_BoolBool() == true then
        api.setLayer(layer, true)
      end
    end
    im.TreePop()
  end
  im.NextColumn()

  im.Columns(1, "layerDataColumns")
end

local function sectionGui(guiId)
  if widgets.draw(api.getBrushStrokeInterpolationSteps(), api.propertiesMap["brushStroke_interpolationSteps"], guiId) then
    api.setBrushStrokeInterpolationSteps(api.propertiesMap["brushStroke_interpolationSteps"].value)
  end
end

local function registerEditorPreferences(prefsRegistry)
  -- prefsRegistry:registerSubCategory("dynamicDecalsTool", "moduleName", nil, {

  -- })
end

local function editorPreferenceValueChanged(path, value)

end

local function documentationGui(docsSection)
  im.PushTextWrapPos(im.GetContentRegionAvailWidth())
  im.TextUnformatted([[
Brush Stroke Layers function as virtual brush strokes, offering a seamless way to apply decals in a stroke-like fashion.

Brush Stroke Layers share properties with decal layers, ensuring consistency in customization.
When enabled, the Brush Stroke Layer generates a sequence of decals, placing one decal with each frame as you move the tool, mimicking the process of applying brush strokes in real life.
To ensure smooth and fluid results, Brush Stroke Layers automatically add decals in between the frames you create.
]])
  im.PopTextWrapPos()
end

local function decalPropertiesDocumentationGui(docsSection)
  if im.BeginTable("Brush Stroke Layer Properties Table", 4, im.flags(im.TableFlags_Resizable, im.TableFlags_Hideable, im.TableFlags_RowBg)) then
    im.TableSetupColumn('Name')
    im.TableSetupColumn('id')
    im.TableSetupColumn('Type')
    im.TableSetupColumn('Highlight')
    im.TableHeadersRow()
    for _, property in ipairs(api.properties["Brush Stroke Layer"]) do
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

local function setup(tool_in)
  tool = tool_in
  api = extensions.editor_api_dynamicDecals
  helper = extensions.editor_dynamicDecals_helper
  gizmo = extensions.editor_dynamicDecals_gizmo
  inspector = extensions.editor_dynamicDecals_inspector
  inspectorUtils = extensions.editor_dynamicDecals_inspector_utils
  docs = extensions.editor_dynamicDecals_docs
  widgets = extensions.editor_dynamicDecals_widgets

  tool.registerSection("Brush Stroke Properties", sectionGui, 1070, false, {})
  inspector.registerLayerGui(api.layerTypes.brushStroke, inspectLayerGui)
  docs.register({section = {"Layer Types", "Brush Stroke Layers"}, guiFn = documentationGui})
  docs.register({section = {"Layer Types", "Brush Stroke Layers", "Properties"}, guiFn = decalPropertiesDocumentationGui})
end

M.registerEditorPreferences = registerEditorPreferences
M.editorPreferenceValueChanged = editorPreferenceValueChanged
M.setup = setup

return M