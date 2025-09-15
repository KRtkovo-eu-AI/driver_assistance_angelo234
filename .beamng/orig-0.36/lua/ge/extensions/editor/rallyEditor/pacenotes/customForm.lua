-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local rallyUtil = require('/lua/ge/extensions/gameplay/rally/util')
local RallyEnums = require('/lua/ge/extensions/gameplay/rally/enums')
local Schema = require('/lua/ge/extensions/gameplay/rally/notebook/structured/schema')

local im  = ui_imgui
local imguiUtils = require('ui/imguiUtils')
local logTag = ''

local M = {}

local lastPacenoteId = nil

M.load = function()
end

M.draw = function(pacenote)
  if pacenote.id ~= lastPacenoteId then
    lastPacenoteId = pacenote.id
  end

  im.Separator()
  im.HeaderText("Preview - Custom")
  -- im.tooltip(dumps(pacenote.structured.fields))

  -- local noteText = noteTextPreview(pacenote)

  im.PushFont3("cairo_semibold_large")
  -- im.Text(dumps(noteText))
  im.PopFont()

  local fname = pacenote:getCustomAudioFile()

  local file_exists = nil
  local tooltipStr = nil
  local voicePlayClr = nil

  if fname and fname ~= "" and FS:fileExists(fname) then
    file_exists = true
    voicePlayClr = im.ImVec4(1, 1, 1, 1)
    tooltipStr = "\""..fname.."\"\nPlay pacenote audio file:\n"..fname
    tooltipStr = tooltipStr.."\n\nClick to copy path to clipboard."
  else
    file_exists = false
    voicePlayClr = im.ImVec4(1, 0, 0, 1.0)
    tooltipStr = "\""..fname.."\"\nPacenote audio file not found:\n"..fname
    tooltipStr = tooltipStr.."\n\nClick to copy path to clipboard."
  end

  if editor.uiIconImageButton(editor.icons.play_circle_filled, im.ImVec2(20, 20), voicePlayClr) then
    im.SetClipboardText(fname)
    if file_exists then
      local audioObj = rallyUtil.buildAudioObjPacenote(fname)
      rallyUtil.playPacenote(audioObj)
    end
  end
  im.tooltip(tooltipStr)

  if editor.uiIconImageButton(editor.icons.folder_open, im.ImVec2(20, 20), im.ImVec4(1, 1, 1, 1)) then
    -- local dir = path.dirname(fname)
    -- dump(dir)
    local dir = pacenote:customAudioFileDir()
    if dir and FS:directoryExists(dir) then
    else
      FS:directoryCreate(dir)
      log('E', logTag, 'exploreFolder: couldnt find dir: '..dir)
    end
    Engine.Platform.exploreFolder(dir)
  end
  im.tooltip("Open audio files folder in explorer.")

  im.SameLine()

  if editor.uiIconImageButton(editor.icons.content_copy, im.ImVec2(20, 20), im.ImVec4(1, 1, 1, 1)) then
    im.SetClipboardText(path.dirname(fnames[1]))
  end
  im.tooltip("Copy audio files folder path to clipboard.")

  im.SameLine()

  local codriver = pacenote:selectedCodriver()
  local codriverHelpTxt = "codriver source=mission custom"
  im.Text(codriverHelpTxt)
  im.tooltip(codriverHelpTxt)

  im.Separator()
  im.HeaderText("Edit - Custom")

  local editEnded = im.BoolPtr(false)
  local audioFileText = im.ArrayChar(1024, pacenote:getCustomAudioFile() or "")
  im.SetNextItemWidth(250)
  editor.uiInputText("Audio File", audioFileText, nil, nil, nil, nil, editEnded)
  if editEnded[0] then
    pacenote:setCustomAudioFile(ffi.string(audioFileText))
  end
  im.tooltip("Enter the path to the audio file to play for this pacenote.")

  if im.Button("Open Audio File") then
    editor_fileDialog.openFile(
      function(data)
        pacenote:setCustomAudioFile(data.filepath)
      end,
      {{"OGG files",".ogg"}},
      false,
      pacenote:customAudioFileDir()
    )
  end

  local descriptionText = im.ArrayChar(1024, pacenote:getCustomDescription() or "")
  im.SetNextItemWidth(250)
  editor.uiInputText("Description", descriptionText, nil, nil, nil, nil, editEnded)
  if editEnded[0] then
    pacenote:setCustomDescription(ffi.string(descriptionText))
  end
end

M.clear = function()
end

return M
