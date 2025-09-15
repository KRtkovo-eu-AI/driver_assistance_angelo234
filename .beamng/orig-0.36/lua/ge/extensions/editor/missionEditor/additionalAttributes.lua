-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui
local C = {}

local imVec24x24 = im.ImVec2(24,24)
local imVec16x16 = im.ImVec2(16,16)
local imVec4Red = im.ImVec4(1,0,0,1)
local imVec4Green = im.ImVec4(0,1,0,1)
local noTranslation = "No Translation found!"

function C:init(missionEditor)
  self.missionEditor = missionEditor
  self.name = "AdditionalInfo"
  self.attributes, self.sortedAttKeys = gameplay_missions_missions.getAdditionalAttributes()
end

local noneVal = {
  label = "(None)"
}

function C:setMission(mission)
  self.mission = mission
  self.missionInstance = gameplay_missions_missions.getMissionById(mission.id)
  self.groupIdInput = im.ArrayChar(1024, self.mission.grouping.id or "")
  self.groupLabelInput = im.ArrayChar(2048, self.mission.grouping.label or "")
  self.authorInput = im.ArrayChar(1024, self.mission.author or "")
  self.dateInput = im.IntPtr(self.mission.date or 0)
  self.dateHumanReadable = nil
end

function C:updateDateHumanReadable()
  self.dateHumanReadable = self.dateHumanReadable or os.date('%Y-%m-%d %H:%M:%S', self.mission.date or 0)
end

function C:getMissionIssues(m)
  self:setMission(m)
  local issues = {}
  if not m.additionalAttributes.difficulty then
    table.insert(issues, {label = 'No difficulty set!', severity='error'})
  end
  if self.mission.grouping.label ~= "" and translateLanguage(self.mission.grouping.label, self.mission.grouping.label, true) == self.mission.grouping.label then
    table.insert(issues, {label = 'Grouping Label has no translation!', severity='minor'})
  end
  if self.mission.author == nil or self.mission.author == "" then
    table.insert(issues, {label = 'No Author set!', severity='error'})
  end
  if self.mission.date == nil or self.mission.date == 0 then
    table.insert(issues, {label = 'No Date set!', severity='error'})
  end

  if shipping_build and self.mission.devMission then
    table.insert(issues, {label = 'DEV Mission should not be included in release version!', severity='error'})
  end

  return issues
end

local function todToTime(val)
  local seconds = ((val + 0.50001) % 1) * 86400
  local hours = math.floor(seconds / 3600)
  local mins = math.floor(seconds / 60 - (hours * 60))
  local secs = math.floor(seconds - hours * 3600 - mins * 60)
  return string.format("%02d:%02d:%02d", hours, mins, secs)
end

function C:draw()
  im.PushID1(self.name)
  im.Columns(2)
  im.SetColumnWidth(0,150)
  local editEnded = im.BoolPtr(false)

  editEnded = im.BoolPtr(false)
  im.Text("Author")
  im.tooltip("The Author of this Mission.")
  im.NextColumn()
  im.PushItemWidth(im.GetContentRegionAvailWidth() - 35)
  editor.uiInputText("##author", self.authorInput, 1024, nil, nil, nil, editEnded)
  if editEnded[0] then
    self.mission.author = ffi.string(self.authorInput)
    self.mission._dirty = true
  end
  im.NextColumn()

  editEnded = im.BoolPtr(false)
  im.Text("Date")
  im.tooltip("When this mission was created or last updated.")
  im.NextColumn()
  im.PushItemWidth(300)
  editor.uiInputInt("##date", self.dateInput, 60*60*24, 60*60*24*7, nil, editEnded)
  im.SameLine()
  if im.Button("Now") then
    self.dateInput[0] = os.time()
    editEnded[0] = true
  end
  im.SameLine()
  if editEnded[0] then
    self.mission.date = self.dateInput[0]
    self.mission._dirty = true
    self.dateHumanReadable = nil
  end
  self:updateDateHumanReadable()
  im.SameLine()
  im.Text(self.dateHumanReadable)
  im.NextColumn()

  im.Text("As Scenario")
  im.tooltip("If set, the mission is available as a scenario from the main menu.")
  im.NextColumn()
  if im.Checkbox("Is Available as Scenario", im.BoolPtr(self.mission.isAvailableAsScenario or false)) then
    self.mission.isAvailableAsScenario = not self.mission.isAvailableAsScenario
    self.mission._dirty = true
  end
  im.NextColumn()

  im.Text("DEV Mission")
  im.tooltip("If set, the mission is not meant for release, but only for testing.")
  im.NextColumn()
  if im.Checkbox("DEV mission", im.BoolPtr(self.mission.devMission or false)) then
    self.mission.devMission = not self.mission.devMission
    self.mission._dirty = true
  end
  im.NextColumn()

  local eh = self.missionEditor.getCurrentEditorHelperWhenActive()
  for _, attKey in ipairs(self.sortedAttKeys) do
    local attribute = self.attributes[attKey]
    local val = attribute.valuesByKey[self.mission.additionalAttributes[attKey]] or noneVal
    im.Text(attribute.label)
    im.NextColumn()
    local isAuto = eh and eh.autoAdditionalAttributes[attKey]
    im.PushItemWidth(im.GetContentRegionAvailWidth())
    if isAuto then im.BeginDisabled() end
    if im.BeginCombo('##'..attKey.."AdditionalData", isAuto and "(Automatic)" or val.label) then

      if im.Selectable1(noneVal.label, val.key == nil) then
        self.mission.additionalAttributes[attKey] = nil
        self.mission._dirty = true
      end
      im.Separator()
      for _, v in ipairs(attribute.valuesSorted) do
        if im.Selectable1(v.label, val.key == v.key) then
          self.mission.additionalAttributes[attKey] = v.key
          self.mission._dirty = true
        end
      end
      im.EndCombo()
    end
    if isAuto then im.EndDisabled() im.tooltip("This Value will be automatically set by the mission constructor.") end
    im.PopItemWidth()
    im.NextColumn()
  end

  editEnded = im.BoolPtr(false)
  im.Text("Group Id")
  im.tooltip("Missions with the same ID will be grouped together in the bigmap. Leave empty for no group.")
  im.NextColumn()
  editEnded = im.BoolPtr(false)
  im.PushItemWidth(im.GetContentRegionAvailWidth() - 35)
  editor.uiInputText("##groupId", self.groupIdInput, 1024, nil, nil, nil, editEnded)
  im.PopItemWidth()
  if editEnded[0] then
    self.mission.grouping.id = ffi.string(self.groupIdInput)
    self.mission._dirty = true
  end
  im.NextColumn()

  im.Text("Group Label")
  im.NextColumn()
  editEnded = im.BoolPtr(false)
  im.PushItemWidth(im.GetContentRegionAvailWidth() - 35)
  editor.uiInputText("##GeneralName", self.groupLabelInput, 2048, nil, nil, nil, editEnded)
  im.PopItemWidth()
  if editEnded[0] then
    self.mission.grouping.label = ffi.string(self.groupLabelInput)
    self._groupLabelTranslated = nil
    self.mission._dirty = true
  end
  im.SameLine()
  if not self._groupLabelTranslated then
    self._groupLabelTranslated = translateLanguage(self.mission.grouping.label, noTranslation, true)
  end
  editor.uiIconImage(editor.icons.translate, imVec24x24 , (self._groupLabelTranslated or noTranslation) == noTranslation and imVec4Red or imVec4Green)
  if im.IsItemHovered() then
    im.tooltip(self._groupLabelTranslated)
  end
  im.Columns(1)
  im.PopID()
end

function C:openTimeUpdater()
  im.OpenPopup("timeUpdaterForMissions")
end

local function getMissionIdsAfter(time)
  local ret = {}
  for _, m in ipairs(gameplay_missions_missions.get()) do
    if m.date and m.date >= time then
      table.insert(ret, m.id)
    end
  end
  table.sort(ret, function(a,b) return gameplay_missions_missions.getMissionById(a).date > gameplay_missions_missions.getMissionById(b).date end)
  return ret
end

function C:timeUpdaterPopup()
  if im.BeginPopup("timeUpdaterForMissions") then
    if not self._timeUpdaterData then
      local data = {
        after = im.IntPtr(os.time() - 2629743 ),
        setTo = im.IntPtr(os.time()),
        missionIds = getMissionIdsAfter(os.time() - 2629743)
      }
      self._timeUpdaterData = data
    end
    local editEnded = im.BoolPtr(false)
    im.Text("After: ")
    im.SameLine() im.PushItemWidth(200)
    editor.uiInputInt("##dateAfter", self._timeUpdaterData.after, 60*60*24, 60*60*24*7, nil, editEnded)
    im.SameLine()
    if im.Button("Now##after") then
      self._timeUpdaterData.after[0] = os.time()
      editEnded[0] = true
    end
    im.SameLine()
    im.Text(os.date('%Y-%m-%d %H:%M:%S', self._timeUpdaterData.after[0]))

    if editEnded[0] then
      self._timeUpdaterData.missionIds = getMissionIdsAfter(self._timeUpdaterData.after[0])
    end
    im.Text("Set To: ")
    im.SameLine() im.PushItemWidth(200)
    editor.uiInputInt("##datesetTo", self._timeUpdaterData.setTo, 60*60*24, 60*60*24*7)
    im.SameLine()
    if im.Button("Now##setTo") then
      self._timeUpdaterData.setTo[0] = os.time()
    end
    im.SameLine()
    im.Text(os.date('%Y-%m-%d %H:%M:%S', self._timeUpdaterData.setTo[0]))

    local list = self.missionEditor.getMissionList()
    local listById = {}
    for _, m in ipairs(list) do listById[m.id] = m end
    if im.Button("Update Affected Missions") then
      for _, mId in ipairs(self._timeUpdaterData.missionIds) do
        local m = listById[mId]
        m._dirty = true
        m.date = self._timeUpdaterData.setTo[0]
      end
      self._timeUpdaterData.missionIds = {}
    end
    im.Separator()
    local remIdx = -1
    for i, mId in ipairs(self._timeUpdaterData.missionIds) do
      local m = listById[mId]
      im.Text(string.format("%s - %s - %s",os.date('%Y-%m-%d %H:%M:%S', m.date), translateLanguage(m.name, m.name, true), m.id) )
      im.tooltip("Click to remove")
      if im.IsItemClicked() then remIdx = i end
    end
    if remIdx ~= -1 then
      table.remove(self._timeUpdaterData.missionIds, remIdx)
    end

    im.EndPopup()
  else
    self._timeUpdaterData = nil
  end
end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end
