-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui
local logTag = ''
local rallyUtil = require('/lua/ge/extensions/gameplay/rally/util')
local SettingsManager = require('/lua/ge/extensions/gameplay/rally/settingsManager')
local RecceSettings = require('/lua/ge/extensions/gameplay/rally/recceSettings')

local C = {}
C.windowDescription = 'Settings'

function C:init(rallyEditor)
  self.rallyEditor = rallyEditor
  self.settings = nil
  self.recce_settings = nil
end

function C:setPath(path)
  self.path = path
end

function C:load()
  local err
  self.settings, err = SettingsManager.loadMissionSettingsForMissionDir(self.path:getMissionDir())
  if err then
    log('E', logTag, err)
  end

  self.missionSettingsFormData = {
    notebooks = {},
  }

  for _,thepath in ipairs(self.rallyEditor.listNotebooks()) do
    local _, basename, _ = path.split(thepath)
    local codrivers = self:loadCodriversForNotebookBasename(basename)
    table.insert(self.missionSettingsFormData.notebooks, {basename=basename, codrivers=codrivers})
  end

  -- print(dumps(self.missionSettingsFormData))
  -- print(dumps(self.settings))

  self.recce_settings = RecceSettings()
  self.recce_settings:load()
end

-- called by RallyEditor when this tab is selected.
function C:selected()
  if not self.path then return end
  self:load()
  -- force redraw of shortcutLegend window
  extensions.hook("onEditorEditModeChanged", nil, nil)
end

-- called by RallyEditor when this tab is unselected.
function C:unselect()
  if not self.path then return end

  -- force redraw of shortcutLegend window
  extensions.hook("onEditorEditModeChanged", nil, nil)
end

function C:draw()
  if not self.path then return end

  im.HeaderText("Mission Settings")

  im.Text("When you run the mission, the below notebook and codriver will be used.")
  self:notebookFilenameSelector()

  for i = 1,5 do im.Spacing() end
  im.Separator()
  for i = 1,5 do im.Spacing() end

  im.HeaderText("Recce Settings")
  im.Text('Recce settings apply to all missions, as well as the Recce UI app.')
  self:recceSettingsSection()
end

function C:loadCodriversForNotebookBasename(basename)
  local folder = self.path:getMissionDir()
  local full_filename = folder..'/'..rallyUtil.notebooksPath..'/'..basename
  local json = jsonReadFile(full_filename)
  if not json then
    log('E', logTag, 'couldnt find notebook file')
  end
  local notebook = require('/lua/ge/extensions/gameplay/rally/notebook/path')()
  notebook:onDeserialized(json)

  local codrivers = {}
  for _,codriver in ipairs(notebook.codrivers.sorted) do
    table.insert(codrivers, codriver.name)
  end
  table.sort(codrivers)
  return codrivers
end

function C:notebookFilenameSelector()
  local name = 'Notebook'
  local tt = 'Set the notebook filename for this mission.'

  im.SetNextItemWidth(400)
  if im.BeginCombo(name..'##filename', self.settings.notebook.filename or '') then
    for _, notebookData in ipairs(self.missionSettingsFormData.notebooks) do
      local notebookBasename = notebookData.basename
      self.codrivers = notebookData.codrivers or {}
      local current = self.settings.notebook.filename == notebookBasename
      if im.Selectable1(notebookBasename, current) then
        if self.settings.notebook.filename ~= notebookBasename then
          self.settings.notebook.filename = notebookBasename
          if #self.codrivers > 0 and self.settings.notebook.codriver ~= self.codrivers[1] then
            self.settings.notebook.codriver = self.codrivers[1]
          end
          self.settings:write()
          log('I', logTag, 'SettingsManager.reset() notebook filename combobox')
          SettingsManager.reset()
          local notebookFname = editor_rallyEditor.detectNotebookToLoad()
          log('I', logTag, 'opening RallyEditor with notebookFname='..notebookFname)
          editor_rallyEditor.loadNotebook(notebookFname)
        end
      end
    end

    im.EndCombo()
  end
  im.tooltip(tt)

  name = 'Codriver'
  tt = 'Set the codriver.'

  im.SetNextItemWidth(400)
  if im.BeginCombo(name..'##codriver', (self.settings.notebook.codriver) or '') then
    for _, codriver in ipairs(self.codrivers or {}) do
      local current = self.settings.notebook.codriver == codriver
      if im.Selectable1(codriver, current) then
        if self.settings.notebook.codriver ~= codriver then
          self.settings.notebook.codriver = codriver
          self.settings:write()
        end
        log('I', logTag, 'SettingsManager.reset() codriver combobox')
        SettingsManager.reset()
      end
    end

    im.EndCombo()
  end
  im.tooltip(tt)
end

function C:recceSettingsSection()
  local name = 'Corner Call Style'
  local tt = 'Set the style of corner calls.'

  im.SetNextItemWidth(400)
  if im.BeginCombo(name..'##corner_call_style_name', self.recce_settings:getCornerCallStyleName() or '') then

    for _, style_name in ipairs(self.recce_settings:cornerCallStyleNames()) do
      local current = self.recce_settings:getCornerCallStyleName() == style_name
      if im.Selectable1(((current and '[current] ') or '')..style_name, current) then
        self.recce_settings:setCornerCallStyleName(style_name)
      end
    end

    im.EndCombo()
  end

  im.tooltip(tt)
end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end
