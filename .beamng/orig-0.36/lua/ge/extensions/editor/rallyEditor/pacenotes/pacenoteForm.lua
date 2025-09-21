-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im = ui_imgui

local cc = require('/lua/ge/extensions/gameplay/rally/util/colors')
local rallyUtil = require('/lua/ge/extensions/gameplay/rally/util')
local RallyEnums = require('/lua/ge/extensions/gameplay/rally/enums')
local StructuredForm = require('/lua/ge/extensions/editor/rallyEditor/pacenotes/structuredForm')
local CustomForm = require('/lua/ge/extensions/editor/rallyEditor/pacenotes/customForm')
local C = {}

local pacenoteUnderEdit = nil
local editingNote = false

local playbackRulesHelpText = [[Playback Rules

Available variables:
- currLap (the current lap)
- maxLap  (the maximum lap)

Any lua code is allowed, so be careful. Examples:
- '' (empty string, the default) -> audio will play
- 'true' -> audio will play
- 'false' -> audio will not play
- 'currLap > 1' -> audio will play except for on the first lap
- 'currLap == 3' -> audio will play only on the 3rd lap
- 'currLap ~= 3' -> audio will play except on the 3rd lap
- 'currLap < maxLap' -> audio will play except for on the last lap
]]

function C:init(pacenoteToolsWindow)
  self.pacenoteToolsWindow = pacenoteToolsWindow
  self.pacenoteToolsState = pacenoteToolsWindow.pacenoteToolsState
  self:setPacenote(nil)
end

function C:setPacenote(pacenote)
  self.pacenote = pacenote
  StructuredForm.clear()
  CustomForm.clear()
end

local function dumpPacenote(pacenote)
  local dumpData = {
    structured = {
      fields = pacenote.structured.fields,
    },
    fnames = {
      freeform = pacenote:audioFnameFreeform(),
      structuredOnline = pacenote:audioFnamesStructuredOnline(),
      structuredOffline = pacenote:audioFnamesStructuredOffline(),
    },
  }
  -- dump(dumpData)
end

local function freeformForm(pacenote, pacenoteToolsState, pacenoteToolsWindow)
  local codriver = pacenote:selectedCodriver()

  im.Separator()
  im.HeaderText("Preview - Freeform")

  im.PushFont3("cairo_semibold_large")
  im.Text(pacenote:noteOutputFreeform())
  im.PopFont()

  local file_exists = false
  local voicePlayClr = nil
  local tooltipStr = nil
  local fname = nil

  fname = pacenote:audioFnameFreeform()
  if FS:fileExists(fname) then
    file_exists = true
    tooltipStr = "Codriver: "..codriver.name.."\nPlay pacenote audio file:\n"..fname
    tooltipStr = tooltipStr.."\n\nClick to copy path to clipboard."
  else
    voicePlayClr = im.ImVec4(0.5, 0.5, 0.5, 1.0)
    tooltipStr = "Codriver: "..codriver.name.."\nPacenote audio file not found:\n"..fname
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

  local codriverHelpTxt = "codriver source=mission name="..codriver.name.." language="..codriver.language.." voice="..codriver.voice
  im.Text(codriverHelpTxt)
  im.tooltip(codriverHelpTxt)

  im.Separator()

  im.HeaderText("Edit - Freeform")

  local editEnded = im.BoolPtr(false)
  -- local freeformBefore = im.ArrayChar(1024, pacenote:getNoteFieldBefore())
  local freeformNote   = im.ArrayChar(1024, pacenote:getNoteFieldFreeform())
  -- local freeformAfter  = im.ArrayChar(1024, pacenote:getNoteFieldAfter())

  -- editor.uiInputText('##distBefore', freeformBefore, nil, nil, nil, nil, editEnded)
  -- if editEnded[0] then
  --   pacenote:clearTodo()
  --   local newVal = ffi.string(freeformBefore)
  --   pacenote:setNoteFieldBefore(newVal)
  --   pacenote:refreshFreeform()
  -- end
  -- im.PushFont3("cairo_regular_medium")
  local distBefore = pacenote:getNoteFieldBefore()
  if distBefore ~= "" then
    im.Text('"'..distBefore..'" ')
  else
    im.Text('<none> ')
  end
  -- im.PopFont()
  im.SameLine()
  im.Text('(distance before, var='..rallyUtil.var_db..')')

  if pacenoteToolsState.insertMode then
    im.SetKeyboardFocusHere()
    pacenoteToolsState.insertMode = false
  end

  im.PushFont3("cairo_regular_medium")
  editingNote = editor.uiInputText('##note', freeformNote, nil, nil, nil, nil, editEnded)
  if editEnded[0] then
    if pacenoteUnderEdit and pacenote.id == pacenoteUnderEdit.id then
      pacenote:clearTodo()
      local newVal = ffi.string(freeformNote)
      pacenote:setNoteFieldFreeform(newVal)
      pacenote:refreshFreeform()
    end
  end
  im.SameLine()
  im.Text('Pacenote text')
  im.PopFont()

  if editingNote and not pacenoteUnderEdit then
    pacenoteUnderEdit = pacenote
  elseif not editingNote then
    pacenoteUnderEdit = nil
  end

  local distAfter = pacenote:getNoteFieldAfter()
  if distAfter ~= "" then
    im.Text('"'..distAfter..'" ')
  else
    im.Text('<none> ')
  end
  im.SameLine()
  im.Text('(distance after, var='..rallyUtil.var_da..')')

  im.Spacing()
  im.Spacing()
  im.Text("Variables may be used in Pacenote text for custom distance call placement.")
  im.Spacing()

  if im.Button("Generate from Structured") then
    pacenote:generateFreeformFromStructured()
  end
  im.tooltip(dumps(pacenote:getNoteFieldStructured()))
end

local function dropdownSlowReleaseTypes()
  local keys = {}
  for k,v in pairs(RallyEnums.slowCornerReleaseTypeName) do
    table.insert(keys, k)
  end

  table.sort(keys)

  local types = {}
  for _,k in ipairs(keys) do
    table.insert(types, { k, RallyEnums.slowCornerReleaseTypeName[k] })
  end

  return types
end

local function dropdownTriggerTypes()
  local keys = {}
  for k,v in pairs(RallyEnums.triggerTypeName) do
    table.insert(keys, k)
  end

  table.sort(keys)

  local types = {}
  for _,k in ipairs(keys) do
    table.insert(types, { k, RallyEnums.triggerTypeName[k] })
  end

  return types
end

local function drawPacenoteActions(pacenote, pacenoteToolsState, pacenoteToolsWindow)
  -- im.Text("Current Pacenote: #" .. self.pacenote_tools_state.selected_pn_id)

  -- if im.Button("Focus Camera") then
  --   self:setCameraToPacenote()
  -- end
  -- im.SameLine()
  if im.Button("Place Vehicle") then
    pacenoteToolsWindow:placeVehicleAtPacenote()
  end
  im.SameLine()

  if pacenoteToolsState.snaproad:isRouteSourced() then
    im.BeginDisabled()
  end
  local paused = simTimeAuthority.getPause()
  if paused then
    im.Text('Unpause game to play Camera Path')
  else
    local camTxt = 'Play Camera Path'
    if pacenoteToolsWindow:cameraPathIsPlaying() then
      camTxt = 'Stop Camera Path'
    end

    if im.Button(camTxt) then
      pacenoteToolsWindow:cameraPathPlay()
    end
  end
  im.SameLine()

  local corner_call_txt = 'Show Corner Calls'
  if pacenoteToolsState.snaproad and pacenoteToolsState.snaproad.show_corner_calls then
    corner_call_txt = 'Hide Corner Calls'
  end
  if im.Button(corner_call_txt) then
    pacenoteToolsWindow:toggleCornerCalls()
  end
  if pacenoteToolsState.snaproad:isRouteSourced() then
    im.EndDisabled()
  end

  if im.Button("TODO") then
    -- local pn = pacenoteToolsWindow:selectedPacenote()
    -- if pn then
    --   pn:markTodo()
    -- end
    pacenote:markTodo()
  end
  im.SameLine()
  if im.Button("Done") then
    -- local pn = pacenoteToolsWindow:selectedPacenote()
    -- if pn then
    --   pn:clearTodo()
    -- end
    pacenote:clearTodo()
  end
  im.SameLine()

  if im.Button("Delete") then
    pacenoteToolsWindow:deleteSelectedPacenote()
  end
  im.SameLine()

  if im.Button("Dump") then
    dumpPacenote(pacenote)
  end

  if not pacenote:useStructured() then
    im.SameLine()
    if im.Button("Merge with Prev") then
      pacenoteToolsWindow:mergeSelectedWithPrevPacenote()
    end
    im.SameLine()
    if im.Button("Merge with Next") then
      pacenoteToolsWindow:mergeSelectedWithNextPacenote()
    end
  end
end

local function drawPacenoteForm(pacenote, pacenoteToolsState, pacenoteToolsWindow)
  if pacenote.missing then return end

  if pacenote:is_valid() then
    im.HeaderText("Pacenote")
  else
    im.HeaderText("[!] Pacenote")
    -- local issues = "Issues (".. (#pacenote.validation_issues) .."):\n"
    -- for _, issue in ipairs(pacenote.validation_issues) do
    --   issues = issues..'- '..issue..'\n'
    -- end
    -- im.Text(issues)
    -- im.Separator()
  end

  drawPacenoteActions(pacenote, pacenoteToolsState, pacenoteToolsWindow)

  if pacenote:is_valid() then
    -- im.HeaderText("Issues (0)")
    im.TextColored(cc.clr_no_error, "No issues")
  else
    im.TextColored(cc.clr_error, "Issues (".. (#pacenote.validation_issues) ..") (Hover for details)")
    local issues = ""
    for _, issue in ipairs(pacenote.validation_issues) do
      issues = issues..'- '..issue..'\n'
    end
    im.PushStyleColor2(im.Col_Text, cc.clr_error)
    im.tooltip(issues)
    im.PopStyleColor()
  end

  local pacenoteAudioModeSetting = RallyEnums.pacenoteAudioModeNames[pacenote:getAudioModeSetting()]
  local notebookAudioMode = RallyEnums.pacenoteAudioModeNames[pacenote.notebook:getAudioMode()]
  im.Text("audioMode notebook="..notebookAudioMode.." pacenote="..pacenoteAudioModeSetting)

  im.SetNextItemWidth(150)
  if im.BeginCombo("Audio Mode Override##pacenoteAudioMode", pacenoteAudioModeSetting) then
    for _, mode in ipairs(RallyEnums.pacenoteAudioModeNames) do
      if im.Selectable1(mode, mode == pacenoteAudioModeSetting) then
        pacenote:setAudioMode(RallyEnums.pacenoteAudioMode[mode])
      end
    end
    im.EndCombo()
  end
  im.tooltip("Override the audio mode just for this pacenote.")

  -- local editEnded = im.BoolPtr(false)
  -- language_form_fields[language] = language_form_fields[language] or {}
  -- local fields = language_form_fields[language]

  -- fields.before = im.ArrayChar(256, pacenote:getNoteFieldBefore())
  -- fields.note   = im.ArrayChar(1024, pacenote:getNoteFieldFreeform())
  -- fields.after  = im.ArrayChar(256, pacenote:getNoteFieldAfter())

  if pacenote:isAudioModeStructuredOnline() or pacenote:isAudioModeStructuredOffline() then
    StructuredForm.draw(pacenote)
  elseif pacenote:isAudioModeFreeform() then
    freeformForm(pacenote, pacenoteToolsState, pacenoteToolsWindow)
  elseif pacenote:isAudioModeCustom() then
    CustomForm.draw(pacenote)
  end

  im.Separator()
  im.HeaderText("Options")

  if im.Checkbox("Slow Corner", im.BoolPtr(pacenote.slowCorner)) then
    pacenote:toggleSlowCorner()
    pacenoteToolsWindow:autofillDistanceCalls()
    pacenote.notebook:refreshAllPacenotes()
  end
  im.SameLine()
  im.TextColored(im.ImVec4(0, 1, 1, 1), "(?)")
  im.tooltip("Default: checked\nWhen checked, the next corner will be delayed until this corner is reached.")

  local currReleaseType = pacenote:getSlowCornerReleaseType()
  local currVal = RallyEnums.slowCornerReleaseTypeName[currReleaseType]
  im.SetNextItemWidth(150)
  if im.BeginCombo('Slow Corner Release Type##slowCornerReleaseType', currVal) then
    for _,triggerType in ipairs(dropdownSlowReleaseTypes()) do
      local k = triggerType[1]
      local v = triggerType[2]
      if im.Selectable1(v, k == currReleaseType) then
        pacenote:setSlowCornerReleaseType(k)
      end
    end
    im.EndCombo()
  end
  im.tooltip("Makes this pacenote's timing later.")

  if im.Checkbox("Reset Odometer", im.BoolPtr(not pacenote.isolate)) then
    pacenote:toggleIsolate()
    pacenoteToolsWindow:autofillDistanceCalls()
    pacenote.notebook:refreshAllPacenotes()
  end
  im.SameLine()
  im.TextColored(im.ImVec4(0, 1, 1, 1), "(?)")
  im.tooltip("Default: checked\nReset the automatic odometer after the pacenote used for calculating distance calls.\n\nWhen unchecked, this pacenote will be skipped and the distance\ncall will be calculated between the previous and next pacenotes.")

  local currTriggerType = pacenote:getTriggerType()
  local currVal = RallyEnums.triggerTypeName[currTriggerType]
  im.SetNextItemWidth(150)
  if im.BeginCombo('Trigger Type##triggerType', currVal) then
    for _,triggerType in ipairs(dropdownTriggerTypes()) do
      local k = triggerType[1]
      local v = triggerType[2]
      if im.Selectable1(v, k == currTriggerType) then
        pacenote:setTriggerType(k)
      end
    end
    im.EndCombo()
  end
  im.tooltip("Makes this pacenote's timing later.")

  -- local editEnded = im.BoolPtr(false)
  -- local playbackRulesText = im.ArrayChar(1024, pacenote.playback_rules or "")
  -- im.SetNextItemWidth(150)
  -- editor.uiInputText("Playback Rules", playbackRulesText, nil, nil, nil, nil, editEnded)
  -- if editEnded[0] then
  --   pacenote.playback_rules = ffi.string(playbackRulesText)
  -- end
  -- im.tooltip(playbackRulesHelpText)
end

function C:draw()
  im.BeginChild1("##pacenoteFormChildWindow", nil, im.WindowFlags_ChildWindow)
  if self.pacenote then
    drawPacenoteForm(self.pacenote, self.pacenoteToolsState, self.pacenoteToolsWindow)
  else
    im.Text("No pacenote selected")
  end
  im.EndChild()
end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end
