-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = "editor_dynamicDecals_loadSave"
local im = ui_imgui

-- reference to the editor tool, set in setup()
local tool = nil
-- reference to the dynamics decal api
local api = nil
local notification = nil
local docs = nil

local defaultProjectFilePath = "/art/dynamicDecals/projects/"
local lastProjectFilePath = defaultProjectFilePath

local loadingMode = 0 -- Overwrite = 0, Append = 1
local loadingModeNamesCharPtr = nil

local function loadFileDialog()
  editor_fileDialog.openFile(
    function(data)
      local res = api.loadLayerStackFromFile(data.filepath, loadingMode)
      lastProjectFilePath = data.filepath
      if res.status.code ~= 0 then
        dump(res)
        notification.add("Load/Save", "Baking Issues", "Issues occured when baking decals.\nCheck the layer stack in order to identify the issues.", notification.levels.warning)
      end
    end,
    {{"Any files", "*"},{"Dynamic Decals Project files",".dyndecals.json"}},
    false,
    path.split(lastProjectFilePath) or "/",
    true
  )
end

local function saveAsFileDialog()
  editor_fileDialog.saveFile(
    function(data)
      api.saveLayerStackToFile(data.filepath)
      lastProjectFilePath = data.filepath
    end,
    {{"Any files", "*"},{"Dynamic Decals Project files",".dyndecals.json"}},
    false,
    path.split(lastProjectFilePath) or "/",
    "File already exists.\nOverwrite?"
  )
end

local function sectionGui(guiId)
  -- SAVE
  if im.Button("Save as...") then
    saveAsFileDialog()
  end
  im.SameLine()

  local _, __, ext = path.split(lastProjectFilePath)
  if ext == "" then im.BeginDisabled() end
  if im.Button("Save") then
    api.saveLayerStackToFile(lastProjectFilePath)
  end
  im.tooltip(string.format("Overwrites %s", lastProjectFilePath))
  if ext == "" then im.EndDisabled() end

  -- LOAD
  if im.Button("Load from file") then
    loadFileDialog()
  end
  if loadingModeNamesCharPtr then
    im.SameLine()
    im.PushItemWidth(120)
    if im.Combo1("##LoadingMode", editor.getTempInt_NumberNumber(loadingMode), loadingModeNamesCharPtr) then
      loadingMode = editor.getTempInt_NumberNumber()
    end
    im.PopItemWidth()
    im.tooltip("[Mode] Defines how the layer stack is handled when loading a project file.\nOverwrite: Layer stack will be wiped and replaced by the layers from the project file.\nAppend: The tool appends the layers from the project file to the layer stack.")
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
The Save & Load section offers a way to manage your design projects.

With the Save feature, you can preserve your current project's layer stack by exporting it as a JSON file.
This file captures all the layer arrangements and properties.

The Load function allows you to retrieve a saved project by importing the corresponding JSON file.
This seamless process ensures that your design is restored exactly as you left it.

There are two loading modes you can choose from:

Overwrite:
Replaces your current layer stack entirely with the layers from the loaded file.
This is ideal when you want to start fresh with the loaded design.

Append:
Adds the layers from the loaded file to your existing layer stack.
This mode is perfect when you want to integrate elements from the loaded project while retaining your ongoing work.
]])
  im.PopTextWrapPos()
end

local tblx = {}
local function setup(tool_in)
  tool = tool_in
  api = extensions.editor_api_dynamicDecals
  notification = extensions.editor_dynamicDecals_notification
  docs = extensions.editor_dynamicDecals_docs

  tblx = {}
  for _, mode in pairs(api.loadingModes) do
    table.insert(tblx, mode.key)
  end
  loadingModeNamesCharPtr = im.ArrayCharPtrByTbl(tblx)

  tool.registerSection("Load/Save", sectionGui, 20, false, {}, {
    {icon = editor.icons.help_outline, tooltip = "Docs", fn = function() docs.selectSection("Save & Load") end},
  })
  docs.register({section = {"Save & Load"}, guiFn = documentationGui})
end

M.onSerialize = function()
  return {
    lastProjectFilePath = lastProjectFilePath,
  }
end

M.onDeserialized = function(data)
  lastProjectFilePath = data.lastProjectFilePath or defaultProjectFilePath
end

M.getCurrentPorjectFilePath = function() return lastProjectFilePath end
M.loadFileDialog = loadFileDialog
M.saveAsFileDialog = saveAsFileDialog

M.registerEditorPreferences = registerEditorPreferences
M.editorPreferenceValueChanged = editorPreferenceValueChanged
M.setup = setup

return M