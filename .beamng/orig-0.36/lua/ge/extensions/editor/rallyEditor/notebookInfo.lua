-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local cc = require('/lua/ge/extensions/gameplay/rally/util/colors')
local rallyUtil = require('/lua/ge/extensions/gameplay/rally/util')
local SettingsManager = require('/lua/ge/extensions/gameplay/rally/settingsManager')
local RallyEnums = require('/lua/ge/extensions/gameplay/rally/enums')

local im  = ui_imgui

local logTag = ''

-- notebook form fields
local notebookNameText = im.ArrayChar(1024, "")
local notebookAuthorsText = im.ArrayChar(1024, "")
local notebookDescText = im.ArrayChar(2048, "")

-- codriver form fields
local codriverNameText = im.ArrayChar(1024, "")
local codriverLanguageText = im.ArrayChar(1024, "")
local codriverVoiceIdText = im.ArrayChar(1024, "")

local voiceNamesSorted = {}

local C = {}
C.windowDescription = 'Notebook'

local function selectCodriverUndo(data)
  data.self:selectCodriver(data.old)
end
local function selectCodriverRedo(data)
  data.self:selectCodriver(data.new)
end
function C:selectCodriver(id)
  self.codriverId = id
  local codriver = self:selectedCodriver()

  if codriver then
    codriverNameText = im.ArrayChar(1024, codriver.name)
    codriverLanguageText = im.ArrayChar(1024, codriver.language)
    codriverVoiceIdText = im.ArrayChar(1024, codriver.voice)
  end
end

function C:init(rallyEditor)
  self.rallyEditor = rallyEditor
  self.codriverId = nil
  self.valid = true
end

function C:isValid()
  return self.valid
end

function C:validate()
  self.valid = true

  self.path:validate()
  if not self.path:is_valid() then
    self.valid = false
  end
end

function C:setPath(path)
  self.path = path
end

function C:selectedCodriver()
  if not self.path then return nil end

  if self.codriverId then
    return self.path.codrivers.objects[self.codriverId]
  else
    return nil
  end
end

-- called by RallyEditor when this tab is selected.
function C:selected()
  if not self.path then return end

  notebookNameText = im.ArrayChar(1024, self.path.name)
  notebookAuthorsText = im.ArrayChar(1024, self.path.authors)
  notebookDescText = im.ArrayChar(1024, self.path.description)

  self:selectCodriver(self.path:selectedCodriver().id)
  -- force redraw of shortcutLegend window
  extensions.hook("onEditorEditModeChanged", nil, nil)
end

-- called by RallyEditor when this tab is unselected.
function C:unselect()
  if not self.path then return end
  -- force redraw of shortcutLegend window
  extensions.hook("onEditorEditModeChanged", nil, nil)
end

function C:draw(mouseInfo)
  self:drawNotebook()
end

local function setNotebookFieldUndo(data)
  data.self.path[data.field] = data.old
end
local function setNotebookFieldRedo(data)
  data.self.path[data.field] = data.new
end
local function setCodriverFieldUndo(data)
  data.self.path.codrivers.objects[data.index][data.field] = data.old
end
local function setCodriverFieldRedo(data)
  data.self.path.codrivers.objects[data.index][data.field] = data.new
end

function C:drawNotebook()
  if not self.path then return end

  self:validate()

  if self:isValid() then
    im.HeaderText("Notebook Info")
  else
    im.HeaderText("[!] Notebook Info")
    local issues = "Issues (".. (#self.path.validation_issues) .."):\n"
    for _, issue in ipairs(self.path.validation_issues) do
      issues = issues..'- '..issue..'\n'
    end
    im.TextColored(cc.clr_error, issues)
    im.Separator()
  end

  im.Text("Current Notebook: #" .. self.path.id)

  for _ = 1,5 do im.Spacing() end

  local editEnded = im.BoolPtr(false)
  editor.uiInputText("Name", notebookNameText, nil, nil, nil, nil, editEnded)
  if editEnded[0] then
    editor.history:commitAction("Change Name of Notebook",
      {self = self, old = self.path.name, new = ffi.string(notebookNameText), field = 'name'},
      setNotebookFieldUndo, setNotebookFieldRedo)
  end

  editEnded = im.BoolPtr(false)
  editor.uiInputText("Authors", notebookAuthorsText, nil, nil, nil, nil, editEnded)
  if editEnded[0] then
    editor.history:commitAction("Change Authors of Notebook",
      {self = self, old = self.path.authors, new = ffi.string(notebookAuthorsText), field = 'authors'},
      setNotebookFieldUndo, setNotebookFieldRedo)
  end

  editEnded = im.BoolPtr(false)
  editor.uiInputText("Description", notebookDescText, nil, nil, nil, nil, editEnded)
  if editEnded[0] then
    editor.history:commitAction("Change Description of Notebook",
      {self = self, old = self.path.description, new = ffi.string(notebookDescText), field = 'description'},
      setNotebookFieldUndo, setNotebookFieldRedo)
  end

  im.SetNextItemWidth(150)
  local notebookAudioMode = RallyEnums.pacenoteAudioModeNames[self.path:getAudioMode()]
  if im.BeginCombo("Audio Mode##notebookAudioMode", notebookAudioMode) then
    for _, mode in ipairs(RallyEnums.pacenoteAudioModeNames) do
      if mode ~= "auto" then
        if im.Selectable1(mode, mode == notebookAudioMode) then
          self.path:setAudioMode(RallyEnums.pacenoteAudioMode[mode])
        end
      end
    end
    im.EndCombo()
  end

  im.SetNextItemWidth(150)
  local notebookDrivelineMode = RallyEnums.drivelineModeNames[self.path:missionSettings():getDrivelineMode()]
  if im.BeginCombo("Driveline Mode##notebookDrivelineMode", notebookDrivelineMode) then
    for _, mode in ipairs(RallyEnums.drivelineModeNames) do
      if im.Selectable1(mode, mode == notebookDrivelineMode) then
        self.path:reloadMissionSettings()
        local newMode = RallyEnums.drivelineMode[mode]
        self.path:missionSettings():setDrivelineMode(newMode)
        self.rallyEditor.setPreferredSnaproadType(RallyEnums.drivelineModeNames[newMode])
      end
    end
    im.EndCombo()
  end

  self:drawCodriversList()
end

function C:drawCodriversList()
  im.HeaderText("Co-Drivers")
  im.Text("Codrivers are only used for StructuredOnline and Freeform Audio Modes.")

  local tabContentsHeight = 0
  im.BeginChild1("codrivers", im.ImVec2(125 * im.uiscale[0], tabContentsHeight), im.WindowFlags_ChildWindow)
  for _,codriver in ipairs(self.path.codrivers.sorted) do
    local codriverName = codriver.name
    -- dump(codriver.id)
    -- dump(self.path:selectedCodriver().id)
    -- dump('-------------------------')
    if codriver.id == self.path:selectedCodriver().id then
      codriverName = codriverName..' (Current)'
    end
    if im.Selectable1(codriverName, codriver.id == self.codriverId) then
      editor.history:commitAction("Select Codriver",
        {old = self.codriverId, new = codriver.id, self = self},
        selectCodriverUndo, selectCodriverRedo)
    end
  end
  im.Separator()
  if im.Selectable1('New...', self.codriverId == nil) then
    local codriver = self.path.codrivers:create(nil, nil)
    self:selectCodriver(codriver.id)
  end
  im.EndChild() -- codrivers list child window

  im.SameLine()
  im.BeginChild1("currentCodriver", im.ImVec2(0,tabContentsHeight), im.WindowFlags_ChildWindow)

  self:drawCodriverForm(self:selectedCodriver())

  im.EndChild() -- codriver form child window
end

function C:deleteCodriverPopup(codriver)
  if im.BeginPopupModal("Delete Codriver", nil, im.WindowFlags_AlwaysAutoResize) then
  -- if im.BeginPopup("Delete Codriver") then

    local langSet = self.path:getLanguages()
    local lang = codriver.language
    local langFound = false
    local isLastCodriver = false
    for _, langInfo in ipairs(langSet) do
      if langInfo.language == lang then
        langFound = true
        if #langInfo.codrivers == 1 then
          isLastCodriver = true
        end
      end
    end

    im.HeaderText("Confirm Delete")

    im.Text("Are you sure you want to delete this codriver?")
    im.Spacing()

    if isLastCodriver then
      im.Text("This is the last codriver for this language.")
      im.Spacing()
      im.Text("You can choose to also delete all pacenotes associated with this codriver's language.")
      im.Spacing()
      im.Text("Language: "..codriver.language)
      im.Spacing()
      im.Text("Consider making a backup of the notebook file before proceeding.")
      im.Text("File: "..self.path.fname)
      im.Spacing()
    end

    im.Separator()
    if im.Button("Delete") then
      self:deleteCodriver(codriver.id)
    end
    if isLastCodriver then
      im.SameLine()
      if im.Button("Delete Codriver and Pacenotes") then
        self:deleteCodriver(codriver.id)
        self.path:deletePacenoteLanguage(codriver.language)
      end
    end
    im.SameLine()
    if im.Button("Cancel", im.ImVec2(120,0)) then
      im.CloseCurrentPopup()
    end
    im.EndPopup()
  end
end

local deleteCodriverPopupShow = false

function C:drawCodriverForm(codriver)
  if not codriver then return end

  if im.Button("Use Codriver") then
    self:useCodriver(codriver)
  end
  im.SameLine()
  if im.Button("Delete") then
    deleteCodriverPopupShow = true
  end

  if deleteCodriverPopupShow then
    deleteCodriverPopupShow = false
    im.OpenPopup("Delete Codriver")
  end
  self:deleteCodriverPopup(codriver)

  local editEnded = im.BoolPtr(false)
  editor.uiInputText("Name", codriverNameText, nil, nil, nil, nil, editEnded)
  if editEnded[0] then
    local oldVal = codriver.name
    local newVal = ffi.string(codriverNameText)
    self.path.codrivers.objects[self.codriverId].name = newVal
  end

  editEnded = im.BoolPtr(false)
  editor.uiInputText("Language", codriverLanguageText, nil, nil, nil, nil, editEnded)
  if editEnded[0] then
    editor.history:commitAction("Change Language of Codriver",
      {self = self, index = self.codriverId, old = codriver.language, new = ffi.string(codriverLanguageText), field = 'language'},
      setCodriverFieldUndo, setCodriverFieldRedo)
  end

  local editEnded = im.BoolPtr(false)
  editor.uiInputText("Voice ID##codriverVoiceId", codriverVoiceIdText, nil, nil, nil, nil, editEnded)
  if editEnded[0] then
    local oldVal = codriver.voice
    local newVal = ffi.string(codriverVoiceIdText)
    self.path.codrivers.objects[self.codriverId].voice = newVal
  end
  im.tooltip('Set the text-to-speech voice')

  im.Spacing()
  im.Text("Name, Language, and Voice ID are used to find the Codriver's audio files.")
  im.Text("Changing them will cause audio files to not be found.")
end

function C:deleteCodriver(codriver_id)
  self.path.codrivers:remove(codriver_id)
  self:selectCodriver(nil)
end

function C:useCodriver(codriver)
  self.path:useCodriver(codriver)
end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end
