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
local mapping = nil

local cornerSeveritiesForDropdown = nil
local reverseCornerSeverityMapping = nil

local cornerLengthsForDropdown = nil
local reverseCornerLengthMapping = nil

local cornerRadiusChangesForDropdown = nil
local reverseCornerRadiusChangeMapping = nil

local cautionForDropdown = nil
local reverseCautionMapping = nil

local cautionTooltip = [[
1. Caution (Lowest Level)
  Meaning: There is a potential hazard that requires the driver to be alert but does not demand a significant reduction in speed.
  Examples:
  - A small bump or dip.
  - A slightly loose surface.
  - A mild corner with a tricky entry.

2. Double Caution (Moderate Level)
  Meaning: A more significant hazard is ahead that may require the driver to adjust their speed and approach.
  Examples:
  - A sharp drop or crest with poor visibility.
  - A narrow bridge.
  - A tight corner near a steep edge or drop-off.

3. Triple Caution (Highest Level)
  Meaning: A severe hazard that demands the driver take immediate and serious precautions, such as slowing down significantly or taking extreme care.
  Examples:
  - A major jump with a blind landing.
  - A large obstacle, such as a rock or tree on the course.
  - A very narrow or high-speed section leading into a dangerous corner or chicane.
]]

local function dumpPacenote(pacenote)
  dump(pacenote.structured.fields)
end

-- Function to generate cornerSeveritiesForDropdown based on selected style and direction.
-- creates a mapping between the corner name and the corner severity.
local function makeCornerSeveritiesForDropdown()
  local corners = {}
  table.insert(corners, { "-", -1 })
  for i, cornerCall in ipairs(mapping.cornerSeverity) do
    table.insert(corners, { cornerCall.name, cornerCall.value })
  end
  return corners
end

local function makeCornerLengthsForDropdown()
  local lengths = {}

  for i, cornerLen in ipairs(mapping.cornerLength) do
    table.insert(lengths, { cornerLen.name or '-', cornerLen.value })
  end

  return lengths
end

local function makeCornerRadiusChangesForDropdown()
  local changes = {}

  for i, cornerRad in ipairs(mapping.cornerRadiusChange) do
    table.insert(changes, { cornerRad.name or '-', cornerRad.value })
  end

  return changes
end

local function makeCautionForDropdown()
  local caution = {}

  for cautionVal,cautionText in pairs(mapping.caution) do
    if cautionText == '' then
      cautionText = '-'
    end
    table.insert(caution, { cautionText, cautionVal })
  end

  return caution
end

local function reverseMapping(mapping)
  local reverseMapping = {}
  for _, entry in ipairs(mapping) do
    reverseMapping[entry[2]] = entry[1]
  end
  return reverseMapping
end

local function load(pacenote)
  local tc = pacenote.notebook:getTextCompositor()
  if not tc then
    log('E', logTag, 'load: no text compositor')
    return
  end
  mapping = tc:getConfig()

  cornerSeveritiesForDropdown = makeCornerSeveritiesForDropdown()
  reverseCornerSeverityMapping = reverseMapping(cornerSeveritiesForDropdown)

  cornerLengthsForDropdown = makeCornerLengthsForDropdown()
  reverseCornerLengthMapping = reverseMapping(cornerLengthsForDropdown)

  cornerRadiusChangesForDropdown = makeCornerRadiusChangesForDropdown()
  reverseCornerRadiusChangeMapping = reverseMapping(cornerRadiusChangesForDropdown)

  cautionForDropdown = makeCautionForDropdown()
  reverseCautionMapping = reverseMapping(cautionForDropdown)
end

local function refreshStructured(pacenote)
  pacenote:refreshStructured()
end

local function updateDirection(pacenote, val)
  pacenote.structured.fields.cornerDirection = val
  refreshStructured(pacenote)
  -- dumpPacenote(pacenote)
end

local function noteTextPreview(pacenote)
  return pacenote:noteOutputStructured()
end

local function addModifier(pacenote, uiLabel, fieldName)
  local default = Schema.default(fieldName)
  local curr = pacenote.structured.fields[fieldName] or default
  if im.Checkbox(uiLabel, im.BoolPtr(curr)) then
    pacenote.structured.fields[fieldName] = not curr
    refreshStructured(pacenote)
  end
end

M.draw = function(pacenote)
  if pacenote.id ~= lastPacenoteId then
    lastPacenoteId = pacenote.id
  end

  if not mapping then
    load(pacenote)
  end

  im.Separator()
  im.HeaderText("Preview - Structured")
  im.tooltip(dumps(pacenote.structured.fields))

  local noteText = noteTextPreview(pacenote)

  im.PushFont3("cairo_semibold_large")
  im.Text(dumps(noteText))
  im.PopFont()

  local fnames = nil

  if pacenote:getAudioMode() == RallyEnums.pacenoteAudioMode.structuredOnline then
    fnames = pacenote:audioFnamesStructuredOnline()
  elseif pacenote:getAudioMode() == RallyEnums.pacenoteAudioMode.structuredOffline then
    fnames = pacenote:audioFnamesStructuredOffline()
  end

  local file_exists = nil
  local tooltipStr = nil
  local voicePlayClr = nil

  for i,fname in ipairs(fnames) do
    if FS:fileExists(fname) then
      file_exists = true
      voicePlayClr = im.ImVec4(1, 1, 1, 1)
      tooltipStr = "\""..noteText[i].."\"\nPlay pacenote audio file:\n"..fname
      tooltipStr = tooltipStr.."\n\nClick to copy path to clipboard."
    else
      file_exists = false
      voicePlayClr = im.ImVec4(1, 0, 0, 1.0)
      tooltipStr = "\""..noteText[i].."\"\nPacenote audio file not found:\n"..fname
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
    if i < #fnames then
      im.SameLine()
    end
  end

  if editor.uiIconImageButton(editor.icons.folder_open, im.ImVec2(20, 20), im.ImVec4(1, 1, 1, 1)) then
    Engine.Platform.exploreFolder(path.dirname(fnames[1]))
  end
  im.tooltip("Open audio files folder in explorer.")

  im.SameLine()

  if editor.uiIconImageButton(editor.icons.content_copy, im.ImVec2(20, 20), im.ImVec4(1, 1, 1, 1)) then
    im.SetClipboardText(path.dirname(fnames[1]))
  end
  im.tooltip("Copy audio files folder path to clipboard.")

  im.SameLine()

  local regenClr = im.ImVec4(1, 1, 1, 1)
  local tooltipText = "Delete pacenote's audio files, allowing them to be regenerated.\nOnly applies to Freeform and StructuredOnline audio modes."
  if not pacenote:canDeleteAudioFiles() then
    -- regenClr = im.ImVec4(0.5, 0.5, 0.5, 1)
    -- tooltipText = "Audio files cannot be deleted for this pacenote.\nOnly files for Freeform and StructuredOnline audio modes can be deleted."
    im.BeginDisabled()
  end
  if editor.uiIconImageButton(editor.icons.trashBin2, im.ImVec2(20, 20), regenClr) then
    im.OpenPopup("Delete Audio Files")
  end
  if not pacenote:canDeleteAudioFiles() then
    im.EndDisabled()
  end
  im.tooltip(tooltipText)
  if im.BeginPopupModal("Delete Audio Files", nil, im.WindowFlags_AlwaysAutoResize) then
    im.Text("Delete this pacenote's audio files?")
    im.Separator()
    if im.Button("Ok", im.ImVec2(120,0)) then
      pacenote:deleteAudioFiles()
      im.CloseCurrentPopup()
    end
    im.SameLine()
    if im.Button("Cancel", im.ImVec2(120,0)) then
      im.CloseCurrentPopup()
    end
    im.EndPopup()
  end

  if pacenote:getAudioMode() == RallyEnums.pacenoteAudioMode.structuredOnline then
    local codriver = pacenote:selectedCodriver()
    local codriverHelpTxt = "codriver source=mission name="..codriver.name.." language="..codriver.language.." voice="..codriver.voice
    im.Text(codriverHelpTxt)
    im.tooltip(codriverHelpTxt)
  elseif pacenote:getAudioMode() == RallyEnums.pacenoteAudioMode.structuredOffline then
    local codriverHelpTxt = string.format("codriver source=shared voice=%s", settings.getValue('rallyTextCompositorVoice'))
    im.Text(codriverHelpTxt)
    im.tooltip(codriverHelpTxt)
  end

  im.Separator()
  im.HeaderText("Edit - Structured")

  im.HeaderText("Corner")

  -- begin corner severity dropdown
  im.SetNextItemWidth(250)
  local defaultSev = reverseCornerSeverityMapping[Schema.default('cornerSeverity')]
  local currSev = reverseCornerSeverityMapping[pacenote.structured.fields.cornerSeverity] or defaultSev
  if im.BeginCombo('##structuredCorner', currSev, im.ComboFlags_HeightLarge) then
    for i, cornerCall in ipairs(cornerSeveritiesForDropdown) do
      local name, severity = cornerCall[1], cornerCall[2]
      if im.Selectable1(name, name == currSev) then
        pacenote.structured.fields.cornerSeverity = severity
        refreshStructured(pacenote)
      end
    end
    im.EndCombo()
  end
  im.SameLine()
  im.Text("severity")
  -- end corner severity dropdown

  addModifier(pacenote, mapping.cornerSquare, "cornerSquare")

  -- begin direction radio buttons
  local direction = pacenote.structured.fields.cornerDirection or Schema.default('cornerDirection')
  local directionPtr

  if direction == -1 then
    directionPtr = im.IntPtr(0)
  elseif direction == 0 then
    directionPtr = im.IntPtr(1)
  elseif direction == 1 then
    directionPtr = im.IntPtr(2)
  end

  im.BeginGroup()
  if im.RadioButton2(mapping.cornerDirection[-1], directionPtr, im.Int(0)) then
    updateDirection(pacenote, -1)
  end
  im.SameLine()
  if im.RadioButton2(mapping.cornerDirection[0], directionPtr, im.Int(1)) then
    updateDirection(pacenote, 0)
  end
  im.SameLine()
  if im.RadioButton2(mapping.cornerDirection[1], directionPtr, im.Int(2)) then
    updateDirection(pacenote, 1)
  end
  im.EndGroup()
  -- end direction radio buttons

  -- begin corner length dropdown
  im.SetNextItemWidth(250)
  local defaultLength = reverseCornerLengthMapping[Schema.default('cornerLength')]
  local currLength = reverseCornerLengthMapping[pacenote.structured.fields.cornerLength] or defaultLength
  if im.BeginCombo('##structuredCornerLength', currLength) then
    for i, cornerLength in ipairs(cornerLengthsForDropdown) do
      local name, value = cornerLength[1], cornerLength[2]
      if im.Selectable1(name, value == currLength) then
        pacenote.structured.fields.cornerLength = value
        refreshStructured(pacenote)
      end
    end
    im.EndCombo()
  end
  im.SameLine()
  im.Text("length")
  -- end corner length input

  -- begin corner radius change dropdown
  im.SetNextItemWidth(250)
  local defaultRadiusChange = reverseCornerRadiusChangeMapping[Schema.default('cornerRadiusChange')]
  local currRadiusChange = reverseCornerRadiusChangeMapping[pacenote.structured.fields.cornerRadiusChange] or defaultRadiusChange
  if im.BeginCombo('##structuredCornerRadiusChange', currRadiusChange) then
    for i, cornerRadiusChange in ipairs(cornerRadiusChangesForDropdown) do
      local name, value = cornerRadiusChange[1], cornerRadiusChange[2]
      if im.Selectable1(name, value == currRadiusChange) then
        pacenote.structured.fields.cornerRadiusChange = value
        refreshStructured(pacenote)
      end
    end
    im.EndCombo()
  end
  im.SameLine()
  im.Text("radius change")
  -- end corner radius change input

  im.HeaderText("Modifiers")

  -- begin caution dropdown
  im.SetNextItemWidth(250)
  local defaultCaution = reverseCautionMapping[Schema.default('caution')]
  local currCaution = reverseCautionMapping[pacenote.structured.fields.caution] or defaultCaution
  if im.BeginCombo('##structuredCaution', currCaution) then
    for i, caution in ipairs(cautionForDropdown) do
      local name, value = caution[1], caution[2]
      if im.Selectable1(name, value == currCaution) then
        pacenote.structured.fields.caution = value
        refreshStructured(pacenote)
      end
    end
    im.EndCombo()
  end
  im.SameLine()
  im.Text("caution")
  im.SameLine()
  im.TextColored(im.ImVec4(0, 1, 1, 1), "(?)")
  im.tooltip(cautionTooltip)
  -- end caution dropdown


  im.Columns(2)
  -- im.SetColumnWidth(0, 150)
  addModifier(pacenote, mapping.modifiers.modDontCut.text, "modDontCut")
  addModifier(pacenote, mapping.modifiers.modNarrows.text, "modNarrows")
  addModifier(pacenote, mapping.modifiers.modWater.text,   "modWater")
  im.NextColumn()
  addModifier(pacenote, mapping.modifiers.modCrest.text, "modCrest")
  addModifier(pacenote, mapping.modifiers.modJump.text,  "modJump")
  addModifier(pacenote, mapping.modifiers.modBump.text,  "modBump")
  addModifier(pacenote, mapping.modifiers.modBumpy.text, "modBumpy")
  im.Columns(1)


  im.HeaderText("Special")

  local default = Schema.default('finishLine')
  local curr = pacenote.structured.fields.finishLine or default
  if im.Checkbox("Finish Line", im.BoolPtr(curr)) then
    pacenote.structured.fields.finishLine = not curr
    refreshStructured(pacenote)
  end

end

M.clear = function()
  mapping = nil
end

return M
