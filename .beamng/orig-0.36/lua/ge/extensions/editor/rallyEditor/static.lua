-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui
local logTag = ''
local normalizer = require('/lua/ge/extensions/gameplay/rally/util/normalizer')
local rallyUtil = require('/lua/ge/extensions/gameplay/rally/util')

local C = {}
C.windowDescription = 'System Pacenotes'

function C:init(rallyEditor)
  self.rallyEditor = rallyEditor

  self.systemPacenotes = {}

  -- self.columnsBasic = {}
  -- self.columnsBasic.selected = im.IntPtr(-1)
end

-- this is the notebook. why am I still calling it a path???
function C:setPath(path)
  self.path = path
end

-- called by RallyEditor when this tab is selected.
function C:selected()
  if not self.path then return end

  self.systemPacenotes = self.path:getSystemPacenotesForAudioMode()

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
  if not self.path then return end

  im.HeaderText("System Pacenotes")
  im.Text("These are special pacenotes for internal use.")
  -- im.Text("Edit /lua/ge/extensions/gameplay/rally/notebook/systemPacenotes.lua to add new system pacenotes.")

  local lang = self.path:selectedCodriverLanguage()
  -- im.Text("Current Codriver Language: "..lang)

  for _ = 1,5 do im.Spacing() end

  im.Columns(4, "spn_columns")
  im.Separator()

  im.Text("Name")
  -- im.SetColumnWidth(0, 130*im.uiscale[0])
  im.NextColumn()

  im.Text("Weight")
  -- im.SetColumnWidth(1, 100*im.uiscale[0])
  im.NextColumn()

  im.Text("Note Text")
  -- im.SetColumnWidth(2, 400*im.uiscale[0])
  im.NextColumn()

  im.Text("Files")
  -- im.SetColumnWidth(3, 400*im.uiscale[0])
  im.NextColumn()

  im.Separator()

  -- if self.path:isAudioModeOfflineStructured() then

  for name,variants in pairs(self.systemPacenotes) do
    for i,variant in ipairs(variants) do
      im.Text(name..'_'..tostring(i))
      im.NextColumn()

      im.Text(tostring(variant.weight or 'auto'))
      im.NextColumn()

      im.Text(variant.text)
      im.NextColumn()


      local tooltipStr = ""
      local voicePlayClr = nil
      local file_exists = FS:fileExists(variant.audioFname)

      if file_exists then
        tooltipStr = "Play pacenote audio file:\n" .. variant.audioFname
      else
        voicePlayClr = im.ImVec4(0.5, 0.5, 0.5, 1.0)
        tooltipStr = "Pacenote audio file not found:\n" .. variant.audioFname
      end

      if editor.uiIconImageButton(editor.icons.play_circle_filled, im.ImVec2(20, 20), voicePlayClr) then
        if FS:fileExists(variant.audioFname) then
          local audioObj = rallyUtil.buildAudioObjPacenote(variant.audioFname)
          rallyUtil.playPacenote(audioObj)
        end
      end
      im.tooltip(tooltipStr)
      im.NextColumn()
    end
  end
  -- else
  --   for _,spn in ipairs(self.path:getSystemPacenotes()) do
  --     im.Text(spn.name)
  --     im.NextColumn()

  --     im.Text(tostring(spn.metadata.weight or 'auto'))
  --     im.NextColumn()

  --     -- Display note text
  --     local noteText = ""
  --     if spn.notes and spn.notes[lang] and spn.notes[lang].note then
  --       noteText = spn.notes[lang].note.freeform or ""
  --     end
  --     im.Text(noteText)
  --     im.NextColumn()

  --     -- Display audio files
  --     local codriver = self.path:selectedCodriver()
  --     if codriver then
  --       local fname = spn:audioFnameFreeform(codriver)
  --       local tooltipStr = ""
  --       local voicePlayClr = nil
  --       local file_exists = FS:fileExists(fname)

  --       if file_exists then
  --         tooltipStr = "Codriver: " .. codriver.name .. "\nPlay pacenote audio file:\n" .. fname
  --       else
  --         voicePlayClr = im.ImVec4(0.5, 0.5, 0.5, 1.0)
  --         tooltipStr = "Codriver: " .. codriver.name .. "\nPacenote audio file not found:\n" .. fname
  --       end

  --       im.Text('[')
  --       im.SameLine()
  --       if editor.uiIconImageButton(editor.icons.play_circle_filled, im.ImVec2(20, 20), voicePlayClr) then
  --         if file_exists then
  --           local audioObj = rallyUtil.buildAudioObjPacenote(fname)
  --           rallyUtil.playPacenote(audioObj)
  --         end
  --       end
  --       im.tooltip(tooltipStr)
  --       im.SameLine()
  --       im.Text(codriver.name)
  --       im.SameLine()
  --       im.Text(']')
  --     end
  --     im.NextColumn()
  --   end
  -- end

  -- local lang_set = {}
  -- for i,langData in ipairs(self.path:getLanguages()) do
  --   lang_set[langData.language] = langData.codrivers
  -- end

  -- for _,spn in ipairs(self.path.static_pacenotes.sorted) do
  --   for lang,langData in pairs(spn.notes) do
  --     im.Text(spn.name)
  --     im.NextColumn()

  --     im.Text(lang)
  --     im.NextColumn()

  --     im.Text(langData.note.freeform)
  --     im.NextColumn()

  --     local codrivers = lang_set[lang]

  --     for _,codriver in ipairs(codrivers or {}) do
  --       local fname = ''
  --       local tooltipStr = ''
  --       local voicePlayClr = nil
  --       local file_exists = false

  --       fname = spn:audioFnameFreeform(codriver)
  --       if FS:fileExists(fname) then
  --         file_exists = true
  --         tooltipStr = "Codriver: "..codriver.name.."\nPlay pacenote audio file:\n"..fname
  --       else
  --         voicePlayClr = im.ImVec4(0.5, 0.5, 0.5, 1.0)
  --         tooltipStr = "Codriver: "..codriver.name.."\nPacenote audio file not found:\n"..fname
  --       end

  --       im.Text('[')
  --       im.SameLine()
  --       if editor.uiIconImageButton(editor.icons.play_circle_filled, im.ImVec2(20, 20), voicePlayClr) then
  --         if file_exists then
  --           local audioObj = rallyUtil.buildAudioObjPacenote(fname)
  --           rallyUtil.playPacenote(audioObj)
  --         end
  --       end
  --       im.tooltip(tooltipStr)
  --       im.SameLine()
  --       im.Text(codriver.name)
  --       im.SameLine()
  --       im.Text(']')
  --       im.SameLine()
  --     end

  --     -- im.Text(fname)
  --     im.NextColumn()
  --     -- im.tooltip(fname)
  --   end
  -- end

  im.Columns(1)
  im.Separator()
end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end
