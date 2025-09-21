-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = "editor_dynamicDecals_history"
local im = ui_imgui

-- reference to the editor tool, set in setup()
local tool = nil
-- reference to the dynamics decal api
local api = nil

local undoStackSelectedIndex = 0
local redoStackSelectedIndex = 0
local toolTipMaxLength = 20
local toolTipMaxWidth = 100

local function getTooltipTextFromAction(action)
  local actionData = dumps(action)
  local lines = {}
  for s in actionData:gmatch("[^\n]+") do
    table.insert(lines, s:sub(1, toolTipMaxWidth)) -- limit width
  end
  local endString = lines[1]
  for i = 2, toolTipMaxLength do
    if lines[i] then
      endString = endString .. "\n" .. lines[i] -- limit height
    else
      break
    end
  end
  if lines[toolTipMaxLength+1] then
    endString = endString .. "\n ..."
  end
  return endString
end

local function sectionGui(guiId)
  local history = api.getHistory()
  local childHeight = (string.endswith(guiId, "_section")) and editor.getPreference("dynamicDecalsTool.history.sectionHeight") or 0

  im.BeginChild1(string.format("DynamicDecalsHistoryChild%s", guiId), im.ImVec2(0, childHeight), true)
  if im.Button("Delete All History##DynamicDecalsTool") then history:clear() end
  if im.IsItemHovered() then
    im.SetTooltip("This will leave changes as they are. Warning, this action cannot be undone.")
  end
  im.Separator()
  im.Columns(2)
  im.TextUnformatted("Undo Stack")
  if im.Button("Undo Selected##DynamicDecalsTool") then history:undo(tableSize(history.undoStack) - undoStackSelectedIndex + 1) end
  im.BeginChild1("undos", im.ImVec2(0, im.GetContentRegionAvail().y))
  for k = tableSize(history.undoStack), 1, -1 do
    local isSel = (k >= undoStackSelectedIndex)
    local action = history.undoStack[k]
    im.PushID1(tostring(k))
    if im.Selectable1(tostring(k) .. ": " .. action.name, isSel) then undoStackSelectedIndex = k end
    if im.IsItemHovered() then
      im.SetTooltip(getTooltipTextFromAction(action))
    end
    im.PopID()
  end
  im.EndChild()
  im.NextColumn()
  im.TextUnformatted("Redo Stack")
  if im.Button("Redo Selected##DynamicDecalsTool") then history:redo(tableSize(history.redoStack) - redoStackSelectedIndex + 1) end
  im.BeginChild1("redos", im.ImVec2(0, im.GetContentRegionAvail().y))
  for k = tableSize(history.redoStack), 1, -1 do
    local isSel = (k >= redoStackSelectedIndex)
    local action = history.redoStack[k]
    im.PushID1(tostring(k) .. "redo")
    if im.Selectable1(tostring(k) .. ": " .. action.name, isSel) then redoStackSelectedIndex = k end
    if im.IsItemHovered() then
      im.SetTooltip(getTooltipTextFromAction(action))
    end
    im.PopID()
  end
  im.EndChild()
  im.Columns(1)
  im.EndChild()
end

local function registerEditorPreferences(prefsRegistry)
  prefsRegistry:registerSubCategory("dynamicDecalsTool", "history", nil, {
    {sectionHeight = {"float", 256, "Section height", nil, 64, 1024}},
  })
end

local function editorPreferenceValueChanged(path, value)

end

local function setup(tool_in)
  tool = tool_in
  api = extensions.editor_api_dynamicDecals

  tool.registerSection("History", sectionGui, 1045, false, {})
end

M.registerEditorPreferences = registerEditorPreferences
M.editorPreferenceValueChanged = editorPreferenceValueChanged
M.setup = setup

return M