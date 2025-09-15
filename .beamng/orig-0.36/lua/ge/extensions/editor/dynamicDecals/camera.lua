-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = "editor_dynamicDecals_meshes"
local im = ui_imgui

-- reference to the editor tool, set in setup()
local tool = nil
-- reference to the dynamics decal api
local api = nil
local docs = nil

local function setCameraInJob(job)
  -- This changes the default rotation. Changing it back is probably not worth it because it requires waiting for several frames until the reset is done
  commands.setGameCamera()
  core_camera.setByName(0, "orbit", false)
  job.sleep(0.00001) -- sleep for one frame so the orbit cam can update correctly
  core_camera.setDefaultRotation(be:getPlayerVehicleID(0), job.args[1])
  core_camera.resetCamera(0)
end

local function setCamera(val)
  core_jobsystem.create(setCameraInJob, nil, val)
end

local function sectionGui(guiId)
  local presets = editor.getPreference("dynamicDecalsTool.camera.presets")
  for name, val in pairs(presets) do
    if im.Button(string.format("%s##%s_%s", name:gsub("^%l", string.upper), "Generate Materials", guiId), im.ImVec2(im.GetContentRegionAvailWidth(), 0)) then
      setCamera(vec3(val[1], val[2], val[3]))
    end
  end

  im.Separator()

  if im.Button(string.format("%s##%s", "Show Preferences", guiId), im.ImVec2(im.GetContentRegionAvailWidth(), 0)) then
    editor.showPreferences("dynamicDecalsTool")
  end
end

local function presetsGui()
  local presets = editor.getPreference("dynamicDecalsTool.camera.presets")
  local i = 1
  local changed = false
  for name, val in pairs(presets) do
    if editor.uiIconImageButton(editor.icons.delete, im.ImVec2(tool.getIconSize(), tool.getIconSize()), nil, nil, nil, string.format("##dynDecals_camera_presets_remove_%d", i)) then
      presets[name] = nil
      changed = true
    end
    im.SameLine()

    editor.uiInputText(string.format("##dynDecals_camera_presets_name_%d", i), editor.getTempCharPtr(name), nil, nil, nil, nil, editor.getTempBool_BoolBool(false))
    if editor.getTempBool_BoolBool() == true then
      local newName = editor.getTempCharPtr()
      local newVal = shallowcopy(val)
      table.remove(presets, i)
      presets[newName] = newVal
      changed = true
    end

    editor.uiInputFloat3(string.format("##dynDecals_camera_presets_val_%d", i), editor.getTempFloatArray3_TableTable(val), "%.1f", nil, editor.getTempBool_BoolBool(false))
    if editor.getTempBool_BoolBool() == true then
      presets[name] = editor.getTempFloatArray3_TableTable()
      changed = true
    end
    im.Separator()
    i = i + 1
  end

  im.Separator()
  if im.Button("Add Preset##dynDecals_camera_presets_add", im.ImVec2(im.GetContentRegionAvailWidth(), 0)) then
    local i = 0
    local name = string.format("preset_%d", i)

    while(presets[name]) do
      i = i + 1
      name = string.format("preset_%d", i)
    end

    presets[name] = {0,0,0}
    changed = true
  end

  if changed == true then
    editor.setPreference("dynamicDecalsTool.camera.presets", presets)
  end
end

local function documentationGui(docsSection)
  im.PushTextWrapPos(im.GetContentRegionAvailWidth())
  im.TextUnformatted([[
The Camera presets section allows you to effortlessly navigate and view your designs from different angles and distances.
With just a click of a button, you can dynamically switch between preset camera positions.

You can also customize your preferred camera presets in the preferences window. Tailor the viewing angles to match your workflow and design preferences.
]])
  im.PopTextWrapPos()
end

local function registerEditorPreferences(prefsRegistry)
  prefsRegistry:registerSubCategory("dynamicDecalsTool", "camera", nil, {
    {presets = {"table",
    {
      ["default"] = {145, -5, 0},
      ["front"] = {180, 0, 0},
      ["back"] = {0, 0, 0},
      ["left"] = {-90, 0, 0},
      ["right"] = {90, 0, 0},
      ["top"] = {90, -90, 0},
    }, "", nil, nil, nil, nil, nil, function(cat, subCat, item)
      presetsGui()
  end}},
  })
end

local function editorPreferenceValueChanged(path, value)

end

local function setup(tool_in)
  tool = tool_in
  api = extensions.editor_api_dynamicDecals
  docs = extensions.editor_dynamicDecals_docs

  tool.registerSection("Camera", sectionGui, 125, false, {}, {
    {icon = editor.icons.help_outline, tooltip = "Docs", fn = function() docs.selectSection("Camera") end},
  })
  docs.register({section = {"Camera"}, guiFn = documentationGui})
end

M.registerEditorPreferences = registerEditorPreferences
M.editorPreferenceValueChanged = editorPreferenceValueChanged
M.setup = setup

return M