-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = "editor_dynamicDecals_settings"
local im = ui_imgui

-- reference to the editor tool, set in setup()
local tool = nil
-- reference to the dynamics decal api
local api = nil
local helper = nil

local uvLayerNamesCharPtr = nil

local materialsMapMaterialNameToMaterialIdx = {} -- key: material name, value: material id
local materialsMapMaterialIdxToMaterialName = {} -- key: material id, value: material name

local shapeMeshes = nil
local materialsFilter = im.ImGuiTextFilter()
local meshesFilter = im.ImGuiTextFilter()

local textureResolutionNamesCharPtr
local textureResolutionXId = 1
local textureResolutionYId = 1

local function updateMaterials()
  if not api then return end
  local vehicleObj = getPlayerVehicle(0)
  if not vehicleObj then
    return
  end
  local vehicleName = (vehicleObj and vehicleObj.jbeam or "")
  local mNames = api.getShapeMaterialNames()
  materialsMapMaterialNameToMaterialIdx = {}
  materialsMapMaterialIdxToMaterialName = {}
  for materialId, materialName in pairs(mNames) do
    materialsMapMaterialNameToMaterialIdx[materialName] = materialId
    materialsMapMaterialIdxToMaterialName[materialId] = materialName
  end
end

local function sectionGui(guiId)
  if im.Checkbox("Enable", editor.getTempBool_BoolBool(api.getEnabled())) then
    api.toggleEnabled()
  end
  im.Separator()

  local enabled = (bit.band(api.getSettings(), api.settingsFlags.UseMousePos.value) == api.settingsFlags.UseMousePos.value)
  if im.Checkbox(api.settingsFlags.UseMousePos.name, editor.getTempBool_BoolBool(enabled)) then
    api.toggleSetting(api.settingsFlags.UseMousePos.value)
  end
  helper.iconTooltip(api.settingsFlags.UseMousePos.description, true)

  if enabled then im.BeginDisabled() end
  local widgetId = "dynamicDecals_settings_cursorPosition"
  local width = (im.GetContentRegionAvailWidth() - im.GetStyle().ItemSpacing.x) / 2
  im.PushItemWidth(width)
  local cursorPosition = api.getCursorPosition()
  local changed = false
  if im.SliderFloat(string.format("##%s_x", widgetId), editor.getTempFloat_NumberNumber(cursorPosition.x), 0, 1, "%.3f") then
    local newVal = editor.getTempFloat_NumberNumber()
    newVal = math.min(newVal, 1)
    newVal = math.max(newVal, 0)
    cursorPosition.x = newVal
    changed = true
  end
  im.PopItemWidth()

  im.SameLine()
  im.PushItemWidth(width)
  if im.SliderFloat(string.format("##%s_y", widgetId), editor.getTempFloat_NumberNumber(cursorPosition.y), 0, 1, "%.3f") then
    local newVal = editor.getTempFloat_NumberNumber()
    newVal = math.min(newVal, 1)
    newVal = math.max(newVal, 0)
    cursorPosition.y = newVal
    changed = true
  end
  im.PopItemWidth()
  if changed then
    api.setCursorPosition(cursorPosition)
  end
  if enabled then im.EndDisabled() end

  im.TextUnformatted("UV Layer")
  im.SameLine()
  im.PushItemWidth(im.GetContentRegionAvailWidth())
  if im.Combo1("##uvLayer", editor.getTempInt_NumberNumber(api:getUvLayer()), uvLayerNamesCharPtr) then
    api.setUvLayer(editor.getTempInt_NumberNumber())
  end
  im.tooltip("Changing UV layer will reproject all layers.\nDepending on the amount of layers this might take some time.")
  im.PopItemWidth()

  if im.TreeNodeEx1(string.format("Materials##VehicleLiveryEditor_Settings_%s", widgetId), im.TreeNodeFlags_DefaultOpen) then
    editor.uiInputSearchTextFilter("Materials Filter", materialsFilter, im.GetContentRegionAvailWidth())
    im.BeginChild1(string.format("MaterialsChild_VehicleLiveryEditor_Settings_%s", widgetId), im.ImVec2(0, 280), true)

    local materialIndices = api.getMaterialIndices()

    for name, id in pairs(materialsMapMaterialNameToMaterialIdx) do
      if im.ImGuiTextFilter_PassFilter(materialsFilter, name) then
        local enabled = tableContains(materialIndices, id)
        if im.Checkbox(string.format("##%s_material_%d_checkbox", widgetId, id), editor.getTempBool_BoolBool(enabled)) then
          local newValue = editor.getTempBool_BoolBool()
          if newValue == true then
            api.addMaterialIdx(id)
          else
            api.removeMaterialIdx(id)
          end
        end
        im.SameLine()
        if im.Selectable1(string.format("%s##%s_material_%d_selectable", name, widgetId, id), enabled) then
          if enabled == false then
            api.addMaterialIdx(id)
          else
            api.removeMaterialIdx(id)
          end
        end
      end
    end
    im.EndChild()

    if im.Button("Update materials") then
      updateMaterials()
    end
    im.TreePop()
  end

  if im.TreeNodeEx1(string.format("Meshes##VehicleLiveryEditor_Settings_%s", widgetId), im.TreeNodeFlags_DefaultOpen) then
    local sMeshes = api.getShapeMeshes()

    if im.Button(string.format("Enable all##Meshes_%s", widgetId)) then
      api.enableAllMeshes()
    end
    im.SameLine()
    if im.Button(string.format("Disable all##Meshes_%s", widgetId)) then
      api.disableAllMeshes()
    end
    im.SameLine()
    editor.uiInputSearchTextFilter(string.format("Meshes Filter##%s", widgetId), meshesFilter, im.GetContentRegionAvailWidth())
    im.BeginChild1(string.format("MeshesChild_VehicleLiveryEditor_Settings_%s", widgetId), im.ImVec2(0, 280), true)
    local i = 0
    for name, enabled in pairs(sMeshes) do
      if im.ImGuiTextFilter_PassFilter(meshesFilter, name) then
        if im.Checkbox(string.format("##%s_shapeMesh_%d_checkbox", widgetId, i), editor.getTempBool_BoolBool(enabled)) then
          api.setMeshEnable(name, not enabled)
        end
        im.SameLine()
        if im.Selectable1(string.format("%s##%s_shapeMesh_%d_selectable", name, widgetId, i), enabled) then
          api.setMeshEnable(name, not enabled)
        end
      end
      i = i + 1
    end
    im.EndChild()

    im.TreePop()
  end

  im.Separator()
  if im.TreeNodeEx1("Texture Resolution", im.TreeNodeFlags_DefaultOpen) then
    local textureResolution = api.getTextureResolution()
    im.TextUnformatted(string.format("Current x: %d y: %d", textureResolution.x, textureResolution.y))

    im.Dummy(im.ImVec2(0,4))
    im.TextUnformatted("x")
    im.SameLine()
    im.PushItemWidth(im.GetContentRegionAvailWidth())
    if im.Combo1("##textureResolution_x", editor.getTempInt_NumberNumber(textureResolutionXId), textureResolutionNamesCharPtr) then
      textureResolutionXId = editor.getTempInt_NumberNumber()
    end
    im.PopItemWidth()
    im.TextUnformatted("y")
    im.SameLine()
    im.PushItemWidth(im.GetContentRegionAvailWidth())
    if im.Combo1("##textureResolution_y", editor.getTempInt_NumberNumber(textureResolutionYId), textureResolutionNamesCharPtr) then
      textureResolutionYId = editor.getTempInt_NumberNumber()
    end
    im.PopItemWidth()
    if im.Button("Apply Changes##applyTextureResolution") then
      api.setTextureResolution(Point2I(api.textureResolutions[textureResolutionXId + 1].value, api.textureResolutions[textureResolutionYId + 1].value))
    end
    im.tooltip("Changing the texture resolution will reproject all layers.\nDepending on the amount of layers this might take some time.")
    local textureResolution = api.getTextureResolution()
    if textureResolution.x ~= api.textureResolutions[textureResolutionXId + 1].value or textureResolution.y ~= api.textureResolutions[textureResolutionYId + 1].value then
      im.SameLine()
      im.TextColored(editor.color.warning.Value , "Resolution has changed.")
    end
    im.TreePop()
  end

  if im.Button("Show dynamic decals preferences") then
    editor.showPreferences("dynamicDecalsTool")
  end
end

local function registerEditorPreferences(prefsRegistry)
  -- prefsRegistry:registerSubCategory("dynamicDecalsTool", "moduleName", nil, {

  -- })
end

local function editorPreferenceValueChanged(path, value)
end

local function updateShapeMeshesJob()
  coroutine.yield()
  shapeMeshes = api.getShapeMeshes()
end

local tblx = {}
local function setup(tool_in)
  tool = tool_in
  api = extensions.editor_api_dynamicDecals
  helper = extensions.editor_dynamicDecals_helper

  tblx = {"0", "1"}
  uvLayerNamesCharPtr = im.ArrayCharPtrByTbl(tblx)

  tblx = {}
  for _, textureRes in ipairs(api.textureResolutions) do
    table.insert(tblx, textureRes.name)
  end
  textureResolutionNamesCharPtr = im.ArrayCharPtrByTbl(tblx)

  if api.ready then
    local textureResolution = api.getTextureResolution()
    for k,v in pairs(api.textureResolutions) do
      if textureResolution.x == v.value then
        textureResolutionXId = k - 1
      end
      if textureResolution.y == v.value then
        textureResolutionYId = k - 1
      end
    end

    updateMaterials()
    -- for some reason we need to delay getting the shape names for the vehicle, otherwise they aren't ready
    core_jobsystem.create(updateShapeMeshesJob, 1)
  end

  tool.registerSection("Settings", sectionGui, 1040, false, {})
end

local function getUsedMaterialNames()
  local res = {}

  for _, id in ipairs(api.getMaterialIndices()) do
    table.insert(res, materialsMapMaterialIdxToMaterialName[id])
  end

  return res
end

M.updateMaterials = updateMaterials
M.getUsedMaterialNames = getUsedMaterialNames

M.registerEditorPreferences = registerEditorPreferences
M.editorPreferenceValueChanged = editorPreferenceValueChanged
M.setup = setup

return M