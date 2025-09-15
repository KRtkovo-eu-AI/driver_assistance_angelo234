-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = "editor_dynamicDecals_export"
local im = ui_imgui

-- reference to the editor tool, set in setup()
local tool = nil
-- reference to the dynamics decal api
local api = nil
local docs = nil

local textureExport_NamesCharPtr = nil
local exportTexturesFileTypes = {
  [0] = "dds",
  [1] = "png"
}

local texturesExport_Name = "skin"
local texturesExport_DefaultDirectoryPath = "/art/dynamicDecals/export/textures"
local texturesExport_DirectoryPath = texturesExport_DefaultDirectoryPath
local texturesExport_exportPaths = {}
-- TODO: Default to 1 for now. We always export in png format.
-- local texturesExport_exportFormatId = 0
local texturesExport_exportFormatId = 1

local skinExport_Name = "skin"
local skinExport_VehicleName = ""
local skinExport_Vehicle = "vivace"

local function updateExportPaths()
  texturesExport_exportPaths = {}
  -- _normal, _metallic & _roughness is disabled for the time being
  local maps = {"_color", "_metallic", "_roughness", "_colorPalette"}
  -- local maps = {"_color", "_colorPalette"}
  local dirPath = string.sub(texturesExport_DirectoryPath, -1) == "/" and texturesExport_DirectoryPath or texturesExport_DirectoryPath .. "/"

  for _, v in ipairs(maps) do
    local path = string.format("%s%s%s.%s", dirPath, texturesExport_Name, v, exportTexturesFileTypes[texturesExport_exportFormatId])
    table.insert(texturesExport_exportPaths, {path, FS:fileExists(path)})
  end
end

local function exportTextures(directoryPath, name, format)
  editor.logInfo(logTag .. " : exportingTextures textures...")
  -- TODO: Disabled for now. We always export in png format.
  -- local res = api.exportTextures(directoryPath, name, exportTexturesFileTypes[format])
  local res = api.exportTextures(directoryPath, name, "png")
  if res then
    updateExportPaths()
  end
end

local function sectionGui()
  if im.TreeNodeEx1("Export Skin", im.TreeNodeFlags_DefaultOpen) then
    local vehicleObj = getPlayerVehicle(0)

    -- vehicle name
    -- todo: get the vehicle name based on the selected vehicle
    im.TextUnformatted("Vehicle Name")
    im.SameLine()
    im.PushItemWidth(im.GetContentRegionAvailWidth())
    if vehicleObj then
      skinExport_VehicleName = vehicleObj.jbeam
    end
    im.BeginDisabled()
    if im.InputText("##skinExport_VehicleName", editor.getTempCharPtr(skinExport_VehicleName)) then
      -- skinExport_VehicleName = editor.getTempCharPtr()
    end
    im.EndDisabled()
    im.PopItemWidth()
    -- export name
    im.TextUnformatted("Skin Name")
    im.SameLine()
    im.PushItemWidth(im.GetContentRegionAvailWidth())
    if im.InputText("##skinExport_Name", editor.getTempCharPtr(skinExport_Name)) then
      local val = editor.getTempCharPtr()
      local newVal = ""
      for c in val:gmatch(".") do
        if c == " " then
          newVal = newVal .. "_"
        else
          newVal = newVal .. c
        end
      end
      skinExport_Name = newVal
    end
    im.PopItemWidth()

    if im.Button("Export Skin##Button") then
      api.exportSkin(skinExport_VehicleName, skinExport_Name)
    end

    im.TreePop()
  end

  if im.TreeNodeEx1("Export Textures") then
    -- export name
    im.TextUnformatted("Export Name")
    im.PushItemWidth(im.GetContentRegionAvailWidth())
    if im.InputText("##texturesExport_Name", editor.getTempCharPtr(texturesExport_Name)) then
      local val = editor.getTempCharPtr()
      local newVal = ""
      for c in val:gmatch(".") do
        if c == " " then
          newVal = newVal .. "_"
        else
          newVal = newVal .. c
        end
      end
      texturesExport_Name = newVal

      updateExportPaths()
    end
    im.PopItemWidth()
    -- export dir
    im.TextUnformatted("Export Directory")
    im.PushItemWidth(im.GetContentRegionAvailWidth() - tool.getIconSize() - im.GetStyle().ItemSpacing.x)
    if im.InputText("##texturesExport_DirectoryPath", editor.getTempCharPtr(texturesExport_DirectoryPath)) then
      texturesExport_DirectoryPath = editor.getTempCharPtr()
      updateExportPaths()
    end
    im.PopItemWidth()
    im.SameLine()
    if editor.uiIconImageButton(editor.icons.folder, im.ImVec2(tool.getIconSize(), tool.getIconSize())) then
      editor_fileDialog.openFile(
        function(data)
          texturesExport_DirectoryPath = data.path
          updateExportPaths()
        end,
        {},
        true,
        texturesExport_DirectoryPath or texturesExport_DefaultDirectoryPath
      )
    end

    -- TODO: Disabled for now. We always export in png format.
    -- im.TextUnformatted("Export Filetype")
    -- im.PushItemWidth(im.GetContentRegionAvailWidth())
    -- if im.Combo1("##reflectionMode", editor.getTempInt_NumberNumber(texturesExport_exportFormatId), textureExport_NamesCharPtr) then
    --   texturesExport_exportFormatId = editor.getTempInt_NumberNumber()
    --   updateExportPaths()
    -- end
    -- im.PopItemWidth()

    if im.TreeNodeEx1("Export Paths", im.TreeNodeFlags_DefaultOpen) then
      for _, exportPath in ipairs(texturesExport_exportPaths) do
        im.TextUnformatted(exportPath[1])
        if exportPath[2] then
          im.SameLine()
          editor.uiIconImageButton(editor.icons.warning, tool.getIconSizeVec2(), editor.color.warning.Value)
          im.tooltip("File already exists! Exporting textures will overwrite the file!")
        end
      end

      im.TreePop()
    end

    if im.Button("Export Textures") then
      exportTextures(texturesExport_DirectoryPath, texturesExport_Name, texturesExport_exportFormatId)
    end

    im.TreePop()
  end
end

local function documentationGui(docsSection)
  im.PushTextWrapPos(im.GetContentRegionAvailWidth())
  im.TextUnformatted([[
The Export section offers you to bring your designs into the game or continue refining them in external programs.

You can export your designs in a format that's ready to be integrated into the game.
This export process generates all the necessary files (textures, material & JBeam) to ensure your livery functions seamlessly in game.

Alternatively, you have the option to export your designs as individual texture files. These textures can be exported as PNG or DDS files.
The exported textures include color, normal, metallic, and roughness maps, capturing every nuance of your design.
Exporting textures opens up the opportunity to continue refining your designs in other software.
This is particularly useful if you want to apply advanced editing techniques or incorporate additional visual effects.
]])
  im.PopTextWrapPos()
end

local function registerEditorPreferences(prefsRegistry)
  -- prefsRegistry:registerSubCategory("dynamicDecalsTool", "moduleName", nil, {

  -- })
end

local function editorPreferenceValueChanged(path, value)

end

local tblx = {}
local function setup(tool_in)
  tool = tool_in
  api = extensions.editor_api_dynamicDecals
  docs = extensions.editor_dynamicDecals_docs

  tblx = {}
  for _, fileType in pairs(exportTexturesFileTypes) do
    table.insert(tblx, fileType)
  end
  textureExport_NamesCharPtr = im.ArrayCharPtrByTbl(tblx)

  updateExportPaths()

  tool.registerSection("Export", sectionGui, 25, false, {}, {
    {icon = editor.icons.help_outline, tooltip = "Docs", fn = function() docs.selectSection("Export") end},
  })
  docs.register({section = {"Export"}, guiFn = documentationGui})
end

M.registerEditorPreferences = registerEditorPreferences
M.editorPreferenceValueChanged = editorPreferenceValueChanged
M.setup = setup

return M