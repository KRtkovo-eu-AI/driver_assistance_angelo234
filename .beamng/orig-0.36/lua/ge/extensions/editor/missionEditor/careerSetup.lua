-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local im  = ui_imgui
local C = {}
local inputBuffers = {}
local missionTypesDir = "/gameplay/missionTypes"
local imVec24x24 = im.ImVec2(24,24)
local imVec16x16 = im.ImVec2(16,16)
local imVec4Red = im.ImVec4(1,0,0,1)
local imVec4Green = im.ImVec4(0,1,0,1)
-- style helper
local noTranslation = "No Translation found!"
local grayColor = im.ImVec4(0.6,0.6,0.6,1)
local redColor = im.ImVec4(1,0.2,0.2,1)
local yellowColor = im.ImVec4(1,1,0.2,1)
local greenColor = im.ImVec4(0.2,1,0.2,1)


function C:init(missionEditor)
  self.missionEditor = missionEditor
  self.rawEditPerMission = {}
  self.careerSetup = {}
  self.tabName = "Objectives"
  self.rawCheckbox = im.BoolPtr(false)

  self.showInCareerCheckbox = im.BoolPtr(false)
  self.showInFreeroamCheckbox = im.BoolPtr(false)



  self.attributeOptions = {'money','vouchers'}
  self.skillOptions = {"(none)"}

  extensions.load('career_branches')
  for _, branch in ipairs(career_branches.getSortedBranches()) do
    table.insert(self.attributeOptions, branch.attributeKey)
    table.insert(self.skillOptions, branch.id)
  end

  table.sort(self.skillOptions)
end

function C:getMissionIssues(m)
  self:setMission(m)
  local issues = {}
  return issues
end

function C:setMission(mission)
  self.mission = mission
  self.missionInstance = gameplay_missions_missions.getMissionById(mission.id)
  self.careerSetup = mission.careerSetup or {}
  self.showInCareerCheckbox[0] = mission.careerSetup.showInCareer or false
  self.showInFreeroamCheckbox[0] = mission.careerSetup.showInFreeroam or false
  -- notify type editor
  self.rawCheckbox[0] = false
  if not self.rawEditPerMission[mission.id] then
    self.rawEditPerMission[mission.id] = false
  end
  self.starKeysSorted = self.missionInstance.sortedStarKeys or tableKeysSorted(self.missionInstance.starLabels or {})
  inputBuffers = {}
  self.missionTypeEditor = editor_missionEditor.getCurrentEditorHelperWhenActive()
  if self.missionTypeEditor then
    self.missionTypeEditor:setContainer(self.mission, 'mission')
  end
  self._translatedTexts = {}
end


function C:attributeDropdown()
  im.PushItemWidth(20)
  local ret
  if im.BeginCombo('','...') then
    for _, key in ipairs(self.attributeOptions) do
      if im.Selectable1(key,false) then
        ret = key
      end
    end
    im.EndCombo()
  end
  return ret
end

local function getBuffer(key, default)
  if not inputBuffers[key] then inputBuffers[key] = im.ArrayChar(2048, default or "") end
  return inputBuffers[key]
end

local editEnded = im.BoolPtr(false)
function C:drawAttributeInput(re, idx, key)
  editEnded[0] = false
  im.PushItemWidth(200)

  editor.uiInputText("##AI", getBuffer(idx.."--"..key, re.attributeKey), 512, nil, nil, nil, editEnded)
  im.SameLine()
  local att = self:attributeDropdown()
  if att or editEnded[0] then
    self.mission._dirty = true
    re.attributeKey = att or ffi.string(getBuffer(idx.."--"..key, re.attributeKey))
    inputBuffers[idx.."--"..key] = nil
  end
  im.PopItemWidth()
end



function C:drawRewardAmount(re, idx, key)
  local raInput = im.IntPtr(re.rewardAmount or 0)
  im.PushItemWidth(200)
  if im.InputInt("##RA",raInput) then
    self.mission._dirty = true
    re.rewardAmount = raInput[0]
    re._originalRewardAmount = re.rewardAmount
  end
  im.PopItemWidth()
end

function C:drawAddReward(key, rewards)

  editEnded[0] = false
  im.PushItemWidth(200)
  editor.uiInputText("##AddReward", getBuffer("addReward--"..key, ""), 512, im.InputTextFlags_EnterReturnsTrue, nil, nil, editEnded)
  im.PopItemWidth()
  im.SameLine()
  local att = self:attributeDropdown()
  im.SameLine()
  local changed = false

  if (editor.uiIconImageButton(editor.icons.add, im.ImVec2(22, 22)) or att or  editEnded[0]) then
    local addKey = att or ffi.string(getBuffer("addReward--"..key, ""))
    if addKey ~= "" then
      self.mission._dirty = true
      rewards = rewards or {}
      table.insert(rewards, {
        attributeKey = addKey,
        rewardAmount = 0,
      })
      inputBuffers["addReward--"..key] = nil
      changed = true
    end
  end

  im.SameLine()
  if editor.uiIconImageButton(editor.icons.content_copy, im.ImVec2(22, 22)) then
    self.copiedRewards = deepcopy(rewards or {})
  end
  im.tooltip("Copy Rewards")
  im.SameLine()
  if not self.copiedRewards then
    im.BeginDisabled()
  end
  if editor.uiIconImageButton(editor.icons.content_paste, im.ImVec2(22, 22)) then
     rewards = deepcopy(self.copiedRewards)
     self.mission._dirty = true
     changed = true
  end
  im.tooltip("Paste Rewards: " ..dumps(self.copiedRewards))
  if not self.copiedRewards then
    im.EndDisabled()
  end

  if changed then
    return rewards
  end
end

function C:drawEntryFee()
  local key = "entryFee--"
  im.PushID1(key.."starReward")
  local fee = self.mission.careerSetup.entryFee or {}
  local remIdx = nil
  for i, re in ipairs(fee) do
    im.PushID1("Reward"..i)
    self:drawAttributeInput(re, i, key)
    im.SameLine()
    self:drawRewardAmount(re, i, key)
    im.SameLine()
    if im.SmallButton("Rem") then
      remIdx = i
    end
    im.PopID()
  end
  if remIdx then
    table.remove(fee, remIdx)
    self.mission._dirty = true
  end
  local added = self:drawAddReward(key, fee)
  if added then
    self.mission._dirty = true
    self.mission.careerSetup.entryFee = added
  end
  im.PopID()
end


function C:drawCareerSetup()
  im.Columns(2)
  im.SetColumnWidth(0,150)

  im.Text("Gamemode")
  im.NextColumn()
  if im.Checkbox("Freeroam##ShowInFreeroam", self.showInFreeroamCheckbox) then
    self.mission.careerSetup.showInFreeroam = self.showInFreeroamCheckbox[0]
    self.mission._dirty = true
  end
  im.SameLine()
  if im.Checkbox("Career##ShowInCareer", self.showInCareerCheckbox) then
    self.mission.careerSetup.showInCareer = self.showInCareerCheckbox[0]
    self.mission._dirty = true
  end
  im.NextColumn()

  im.Text("Skill")
  im.NextColumn()
  im.PushItemWidth(im.GetContentRegionAvailWidth())
  if im.BeginCombo("##Skill", self.mission.careerSetup.skill or "(none)") then
    for _, skill in ipairs(self.skillOptions) do
      if im.Selectable1(skill, skill == self.mission.careerSetup.skill) then
        self.mission.careerSetup.skill = skill
        self.mission._dirty = true
      end
    end
    im.EndCombo()
  end
  im.PopItemWidth()
  im.NextColumn()

  im.Text("Entry Fee")
  im.NextColumn()
  self:drawEntryFee()
  im.NextColumn()

  im.Columns(1)


end


function C:draw()
  self:drawCareerSetup()
end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end
