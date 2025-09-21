-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local im  = ui_imgui
local C = {}
local imVec24x24 = im.ImVec2(24,24)
local imVec20x20 = im.ImVec2(20,20)
local infoColor = im.ImVec4(1,1,1,0.33)

function C:init(missionEditor)
  self.missionEditor = missionEditor
end

function C:setMission(mission)
  self.mission = mission
  self:createInputFields()
end

function C:createInputFields()
  self.inputs = {}
  for i, layer in ipairs(self.mission.layers) do
    self.inputs[i] = {
      text = im.ArrayChar(1024, layer.dir)
    }
  end

end

function C:draw()
  im.HeaderText("Layers")
  im.SameLine()
  im.SetCursorPosY(im.GetCursorPosY()+8)
  editor.uiIconImage(editor.icons.info, imVec20x20, infoColor)
  im.tooltip("Mission will try to find files in each of these folders, until it finds a matching file.")
  local width = im.GetContentRegionAvailWidth()
  im.Columns(2)
  im.SetColumnWidth(0,150)
  local moveDown, moveUp, add, remove = nil, nil, nil, nil
  local addElem = nil
  for i, layer in ipairs(self.mission.layers) do
    --im.Text('#'..i)
    if i == 1 then im.Text("Checked First") end
    if i == #self.mission.layers then im.Text("Checked Last") end
    im.NextColumn()
    im.PushID1("Layer-"..i)
    local editEnded = im.BoolPtr(false)
    --im.BeginChild1("box", im.ImVec2(width, 50), true, bit.bor(im.WindowFlags_NoScrollWithMouse, im.WindowFlags_NoScrollbar))
    if i == 1 then im.BeginDisabled() end
    if editor.uiIconImageButton(editor.icons.arrow_upward, imVec24x24) then moveUp = i end
    if i == 1 then im.EndDisabled() end
    im.SameLine()
    if i == #self.mission.layers then im.BeginDisabled() end
    if editor.uiIconImageButton(editor.icons.arrow_downward, imVec24x24) then moveDown = i end
    if i == #self.mission.layers then im.EndDisabled() end
    im.SameLine()

    if layer.fixed then im.BeginDisabled() end
    editor.uiInputText("##dir", self.inputs[i].text, 1024, nil, nil, nil, editEnded)
    if layer.fixed then im.EndDisabled() end
    if editEnded[0] then
      layer.dir = ffi.string(self.inputs[i].text)
      self.mission._dirty = true
    end
    im.SameLine()
    if editor.uiIconImageButton(editor.icons.folder, imVec24x24) then
      Engine.Platform.exploreFolder(layer.dir)
    end
    im.SameLine()
    if layer.fixed then im.BeginDisabled() end
    if editor.uiIconImageButton(editor.icons.material_pick_mapto, imVec24x24) then
      extensions.editor_fileDialog.openFile(
        function(data)
          layer.dir = data.path
          self:createInputFields()
          self.mission._dirty = true
        end, nil, true, layer.dir)
    end
    im.SameLine()
    if editor.uiIconImageButton(editor.icons.delete_forever, imVec24x24) then
      remove = i
    end
    if layer.fixed then im.EndDisabled() end
    im.SameLine()
    if editor.uiIconImageButton(editor.icons.create_new_folder, imVec24x24) then
      extensions.editor_fileDialog.openFile(
        function(data)
          add = i+1
          addElem = {dir = data.path }
          table.insert(self.mission.layers, add, addElem)
          self.mission._dirty = true
          self:createInputFields()
        end, nil, true, "/")
    end
    --im.EndChild()
    im.NextColumn()
    im.PopID()
  end

  if moveUp then
    local elem = self.mission.layers[moveUp]
    table.remove(self.mission.layers, moveUp)
    table.insert(self.mission.layers, moveUp-1, elem)
  end
  if moveDown then
    local elem = self.mission.layers[moveDown]
    table.remove(self.mission.layers, moveDown)
    table.insert(self.mission.layers, moveDown+1, elem)
  end
  if add then
    dump(add)
    table.insert(self.mission.layers, add, addElem)
  end
  if remove then
    table.remove(self.mission.layers, remove)
  end
  if moveUp or moveDown or add or remove then
    self.mission._dirty = true
    self:createInputFields()
  end

  im.Columns(1)
end
--[[
  if layer.isMissionFolderDir then
    im.tooltip("This is the mission folder and cannot be changed.")
  end
  if layer.isMissionTypeDir then
    im.tooltip("This is the mission type folder and cannot be changed.")
  end
  ]]

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end
