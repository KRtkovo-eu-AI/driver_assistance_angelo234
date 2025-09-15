-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = "editor_dynamicDecals_debugSection"
local im = ui_imgui

-- reference to the editor tool, set in setup()
local tool = nil
-- reference to the dynamics decal api
local api = nil
local textures = nil
local gizmo = nil
local notification = nil
local docs = nil
local widgets = nil
local settings = nil

local reprojectLayers = false

local function sectionGui(guiId)
  im.Separator()
  local style = im.GetStyle()
  local maxWidth = im.GetContentRegionAvailWidth()

  if im.Checkbox(string.format("%s##%s", "Reproject Layers every frame", guiId), editor.getTempBool_BoolBool(reprojectLayers)) then
    reprojectLayers = editor.getTempBool_BoolBool()
  end
  if reprojectLayers == true then
    api.reprojectLayers()
  end

  if im.Button(string.format("%s##%s", "Reproject Layers", guiId)) then
    dump(api.reprojectLayers())
  end
  im.Separator()

  im.TextUnformatted(string.format("api.projectDynamicDecals: %s", api.projectDynamicDecals and "true" or "false"))
  local buttonWidth = (maxWidth - style.ItemSpacing.x) / 2
  if im.Button(string.format("%s##%s", "set true##api.setProjectDynamicDecalsState(true)", guiId), im.ImVec2(buttonWidth, 0)) then
    api.setProjectDynamicDecalsState(true)
  end
  im.SameLine()
  if im.Button(string.format("%s##%s", "set false##api.setProjectDynamicDecalsState(false)", guiId), im.ImVec2(buttonWidth, 0)) then
    api.setProjectDynamicDecalsState(false)
  end
  im.Separator()

  if im.Button(string.format("%s##%s", "Bake Brush", guiId)) then
    api.bakeBrush()
  end

  if im.Button(string.format("%s##%s", "Dump api.layerStack", guiId)) then
    dump(api.getLayerStack())
  end

  if im.Button(string.format("%s##%s", "Reload textures", guiId)) then
    textures.reloadTextureFiles()
  end

  if im.Button(string.format("%s##%s", "getShapeMaterialNames", guiId)) then
    dump(api.getShapeMaterialNames())
  end

  if im.Button(string.format("%s##%s", "getMeshObjectCount", guiId)) then
    dump(api.getMeshObjectCount())
  end

  im.TextUnformatted(string.format("depth: %f", api.getDepth()))
  im.TextUnformatted(string.format("surfaceNormal: %s", api.getSurfaceNormal()))

  im.TextUnformatted(string.format("currentMaskEditingLayerUid: %s", tool.getCurrentMaskEditingLayerUid()))

  if im.Button(string.format("%s##%s", "api.getShapeMeshes()", guiId)) then
    dump(api.getShapeMeshes())
  end

  if im.Button(string.format("%s##%s", "dump selection", guiId)) then
    dump(editor.selection["dynamicDecalLayer"])
  end

  if im.Button(string.format("%s##%s", "dump gizmo.transform", guiId)) then
    dump(gizmo.transform)
  end

  if im.Button(string.format("%s##%s", "dump gizmo.transform:getPosition()", guiId)) then
    dump(gizmo.transform:getPosition())
  end

  if im.Button(string.format("%s##%s", "notification.add", guiId)) then
    notification.add("Debug", "test", "this is a test")
  end

  if im.Button(string.format("%s##%s", "Open 'Load/Save' section", guiId)) then
    tool.setSectionOpenState("Load/Save", true)
  end

  if im.Button(string.format("%s##%s", "Close 'Load/Save' section", guiId)) then
    tool.setSectionOpenState("Load/Save", false)
  end

  if im.Button(string.format("%s##%s", "Docs - Select 'Linked Set Layers'", guiId)) then
    docs.selectSection("Linked Set Layers")
  end

  if im.Button(string.format("%s##%s", "Highlight 'rotation' widget", guiId)) then
    widgets.highlight("##Decal Properties_section_decalRotation", 3)
  end

  if im.Button(string.format("%s##%s", "api.updateVehicleMaterials()", guiId)) then
    api.updateVehicleMaterials()
  end

  if im.Button(string.format("%s##%s", "Dump settings.getUsedMaterialNames()", guiId)) then
    dump(settings.getUsedMaterialNames())
  end

  if im.Button(string.format("%s##%s", "Generate Materials", guiId)) then
    local vehicles = FS:findFiles('/vehicles/', '*', 0, false, true)

    for _, vehicle in ipairs(vehicles) do
      local matPath = string.format("%s/main.materials.json", vehicle)

      local outData = {}

      if FS:fileExists(matPath) then
        local data = jsonReadFile(matPath)
        local materialsUsed = settings.getUsedMaterialNames()
        for _, material in ipairs(materialsUsed) do

          for materialName, materialData  in pairs(data) do
            if materialData.mapTo == material then
              local dynDecalMaterial = deepcopy(materialData)

              local newMatName = string.format("%s.skin.dynamicTextures", material)
              dynDecalMaterial.name = newMatName
              dynDecalMaterial.mapTo = newMatName
              dynDecalMaterial.persistentId = nil
              dynDecalMaterial.Stages[2]['useAnisotropic'] = true
              dynDecalMaterial.Stages[2]['instanceDiffuse'] = true
              dynDecalMaterial.Stages[2]['clearCoatRoughnessFactor'] = 0.06
              dynDecalMaterial.Stages[2]['baseColorMap'] = "@DynamicTextureBaseColor"
              dynDecalMaterial.Stages[2]['diffuseMapUseUV'] = 1
              dynDecalMaterial.Stages[2]['colorPaletteMap'] = "@DynamicTextureColorPalette"
              dynDecalMaterial.Stages[2]['colorPaletteMapUseUV'] = 1
              dynDecalMaterial.Stages[2]['metallicMap'] = "@DynamicTextureMetallic"
              dynDecalMaterial.Stages[2]['metallicMapUseUV'] = 1
              dynDecalMaterial.Stages[2]['roughnessMap'] = "@DynamicTextureRoughness"
              dynDecalMaterial.Stages[2]['roughnessMapUseUV'] = 1

              outData[newMatName] = dynDecalMaterial
            end
          end
        end

        if next(outData) then
          jsonWriteFile(string.format("%s/dynamicDecals/main.materials.json", vehicle), outData, true)
        end
      end
    end

  end
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
  textures = extensions.editor_dynamicDecals_textures
  gizmo = extensions.editor_dynamicDecals_gizmo
  notification = extensions.editor_dynamicDecals_notification
  docs = extensions.editor_dynamicDecals_docs
  widgets = extensions.editor_dynamicDecals_widgets
  settings = extensions.editor_dynamicDecals_settings

  tool.registerSection("Debug", sectionGui, 1010, false, {})
end

M.registerEditorPreferences = registerEditorPreferences
M.editorPreferenceValueChanged = editorPreferenceValueChanged
M.setup = setup

return M