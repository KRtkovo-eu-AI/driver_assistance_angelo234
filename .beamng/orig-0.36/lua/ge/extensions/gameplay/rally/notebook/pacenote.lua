-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local waypointTypes = require('/lua/ge/extensions/gameplay/rally/notebook/waypointTypes')
local cc = require('/lua/ge/extensions/gameplay/rally/util/colors')
local rallyUtil = require('/lua/ge/extensions/gameplay/rally/util')
local RallyEnums = require('/lua/ge/extensions/gameplay/rally/enums')
local normalizer = require('/lua/ge/extensions/gameplay/rally/util/normalizer')
-- local SettingsManager = require('/lua/ge/extensions/gameplay/rally/settingsManager')
local Structured = require('/lua/ge/extensions/gameplay/rally/notebook/structured')

local C = {}
local logTag = ''
local structured = 'structured'

local pn_drawMode_noSelection = 'no_selection'
local pn_drawMode_partitionedSnaproad = 'partitioned_snaproad'
local pn_drawMode_background = 'background'
local pn_drawMode_next = 'next'
local pn_drawMode_previous = 'previous'
local pn_drawMode_selected = 'selected'

local defaultSlowCornerReleaseType = RallyEnums.slowCornerReleaseType.csHalf

C.noteFields = {
  before = 'before',
  beforeMeters = 'beforeMeters',
  note = 'note',
  after = 'after',
  afterMeters = 'afterMeters',
}

function C:init(notebook, name, forceId)
  self.notebook = notebook
  self.id = forceId or notebook:getNextUniqueIdentifier()
  self.pk = rallyUtil.randomId()
  self.name = name or ("Pacenote " .. self.id)
  self.todo = false
  self.playback_rules = nil
  self.isolate = false
  self.triggerType = RallyEnums.triggerType.dynamic
  self.slowCornerReleaseType = defaultSlowCornerReleaseType
  self.slowCorner = false
  self.audioMode = RallyEnums.pacenoteAudioMode.auto
  self.notes = {}
  for _,lang in ipairs(self.notebook:getLanguages()) do
    lang = lang.language
    self.notes[lang] = {}
    self.notes[lang].before = ''
    self.notes[lang].beforeMeters = -1
    self.notes[lang].note = {}
    self.notes[lang].after = ''
    self.notes[lang].afterMeters = -1
  end

  self.structured = Structured()

  self.pacenoteWaypoints = require('/lua/ge/extensions/gameplay/util/sortedList')(
    "pacenoteWaypoints",
    self,
    require('/lua/ge/extensions/gameplay/rally/notebook/pacenoteWaypoint')
  )
  self.metadata = {}

  self.sortOrder = 999999
  self.validation_issues = {}
  self.draw_debug_lang = nil
  self._cachedCompiledData = nil
  self._compileFailed = false
  self.halfpoint = nil
  self._cachedLength = nil
  self.visualSerialNo = -1
end

-- used by pacenoteWaypoints.lua
function C:getNextUniqueIdentifier()
  return self.notebook:getNextUniqueIdentifier()
end

function C:getNextWaypointType()
  local foundTypes = {
    [waypointTypes.wpTypeCornerStart] = false,
    [waypointTypes.wpTypeCornerEnd] = false,
    [waypointTypes.wpTypeFwdAudioTrigger] = false,
  }

  for _,wp in pairs(self.pacenoteWaypoints.objects) do
    foundTypes[wp.waypointType] = true
  end

  if foundTypes[waypointTypes.wpTypeCornerStart] == false then
    return waypointTypes.wpTypeCornerStart
  elseif foundTypes[waypointTypes.wpTypeCornerEnd] == false then
    return waypointTypes.wpTypeCornerEnd
  elseif foundTypes[waypointTypes.wpTypeFwdAudioTrigger] == false then
    return waypointTypes.wpTypeFwdAudioTrigger
  end
end

function C:setAllRadii(newRadius, wpType)
  for _,wp in ipairs(self.pacenoteWaypoints.sorted) do
    if not wpType or wp.waypointType == wpType then
      wp.radius = newRadius
    end
  end
end

function C:allToTerrain()
  for _,wp in ipairs(self.pacenoteWaypoints.sorted) do
    wp.pos.z = core_terrain.getTerrainHeight(wp.pos)
  end
end

-- function C:_noteOutputFreeformWithVars(lang)
--   lang = lang or self:selectedCodriverLanguage()
--   local txt = ''
--   -- local langData = self.notes[lang]

--   -- if not langData then
--     -- return txt
--   -- end

--   -- local note = langData[self.noteFields.note].freeform
--   -- local before = langData[self.noteFields.before]
--   -- local after = langData[self.noteFields.after]

--   local note = self:getNoteFieldFreeform(lang)
--   local before = self:getNoteFieldBefore(lang)
--   local after = self:getNoteFieldAfter(lang)

--   if rallyUtil.useNote(note) then
--     txt = note
--   else
--     -- if theres no usable note, dont bother with distance calls
--     return txt
--   end

--   -- add before and after vars to the note if they dont already exist
--   if not string.find(txt, rallyUtil.var_db) then
--     txt = rallyUtil.var_db..' '..txt
--   end

--   if not string.find(txt, rallyUtil.var_da) then
--     local punc = self.notebook:getTextCompositor():getPunctuationDistanceCalls()
--     txt = txt..' '..rallyUtil.var_da..punc
--   end

--   txt = rallyUtil.trimString(txt)

--   return txt
-- end

local function _noteOutputFreeformWithVarsHelper(note, before, after)
  -- lang = lang or self:selectedCodriverLanguage()
  local txt = ''
  -- local langData = self.notes[lang]

  -- if not langData then
    -- return txt
  -- end

  -- local note = langData[self.noteFields.note].freeform
  -- local before = langData[self.noteFields.before]
  -- local after = langData[self.noteFields.after]

  -- local note = self:getNoteFieldFreeform(lang)
  -- local before = self:getNoteFieldBefore(lang)
  -- local after = self:getNoteFieldAfter(lang)

  if rallyUtil.useNote(note) then
    txt = note
  else
    -- if theres no usable note, dont bother with distance calls
    return txt
  end

  -- add before and after vars to the note if they dont already exist
  if not string.find(txt, rallyUtil.var_db) then
    txt = rallyUtil.var_db..' '..txt
  end

  if not string.find(txt, rallyUtil.var_da) then
    txt = txt..' '..rallyUtil.var_da
  end

  txt = rallyUtil.trimString(txt)

  return txt
end

local function _interpolateFreeformVars(txt, before, after, punc)
  if rallyUtil.useNote(before) then
    txt = string.gsub(txt, rallyUtil.var_db, before)
  else
    txt = string.gsub(txt, rallyUtil.var_db, '')
  end

  if rallyUtil.useNote(after) then
    txt = string.gsub(txt, rallyUtil.var_da, after)
    txt = txt..punc
  else
    txt = string.gsub(txt, rallyUtil.var_da, '')
  end

  txt = rallyUtil.trimString(txt)

  return txt
end

function C:noteOutputFreeform(lang)
  if self._compileFailed then
    return ''
  end

  local note = self:getNoteFieldFreeform(lang)
  local before = self:getNoteFieldBefore(lang)
  local after = self:getNoteFieldAfter(lang)
  -- local tc = self.notebook:getTextCompositor()
  -- if not tc then
  --   log('E', logTag, 'noteOutputFreeform: no text compositor')
  --   self._compileFailed = true
  --   return ''
  -- end
  -- local punc = tc:getPunctuationDistanceCalls()

  local txt = _noteOutputFreeformWithVarsHelper(note, before, after)
  local punc = ''
  txt = _interpolateFreeformVars(txt, before, after, punc)

  -- txt = normalizer.replaceWords(SettingsManager.getMainSettings():getFreeformSubstitutions(), txt)

  return txt
end

function C:noteOutputStructured(lang)
  lang = lang or self:selectedCodriverLanguage()
  local notesOut = {}
  local langData = self.notes[lang]

  if not langData then
    langData = {
      note = {
        structured = {},
      },
      before = '',
      after = ''
    }

    self.notes[lang] = langData
  end

  local notesArray = langData[self.noteFields.note].structured
  notesArray = deepcopy(notesArray)
  local before = langData[self.noteFields.before]
  local after = langData[self.noteFields.after]

  -- if there are no notes, then dont bother with distance calls.
  if notesArray == nil or #notesArray == 0 then
    return notesOut
  end

  -- local substitutions = SettingsManager.getMainSettings():getStructuredSubstitutions()

  for _,txt in ipairs(notesArray) do
    -- if rallyUtil.useNote(before) then
    --   txt = string.gsub(txt, rallyUtil.var_db, before)
    -- else
    --   txt = string.gsub(txt, rallyUtil.var_db, '')
    -- end

    -- if rallyUtil.useNote(after) then
    --   txt = string.gsub(txt, rallyUtil.var_da, after)
    -- else
    --   txt = string.gsub(txt, rallyUtil.var_da, '')
    -- end

    -- txt = rallyUtil.trimString(txt)
    -- txt = normalizer.replaceWords(substitutions, txt)
    table.insert(notesOut, txt)
  end

  return notesOut
end

function C:getNoteFieldBefore(lang)
  lang = lang or self:selectedCodriverLanguage()
  local lang_data = self.notes[lang]
  if not lang_data then return '' end
  local val = lang_data[self.noteFields.before]
  if not val then
    return ''
  end
  return val
end

function C:getNoteFieldBeforeMeters(lang)
  lang = lang or self:selectedCodriverLanguage()
  local lang_data = self.notes[lang]
  if not lang_data then return -1 end
  local val = lang_data[self.noteFields.beforeMeters]
  if not val then
    return -1
  end
  return val
end

function C:getNoteFieldFreeform(lang)
  lang = lang or self:selectedCodriverLanguage()
  local lang_data = self.notes[lang]
  if not lang_data then return '' end
  local val = lang_data[self.noteFields.note].freeform
  if not val then
    return ''
  end
  return val
end

function C:getNoteFieldStructured(lang)
  lang = lang or self:selectedCodriverLanguage()
  local lang_data = self.notes[lang]
  if not lang_data then return {} end
  local val = lang_data[self.noteFields.note].structured
  if not val then
    return {}
  end
  return val
end

function C:getNoteFieldAfter(lang)
  lang = lang or self:selectedCodriverLanguage()
  local lang_data = self.notes[lang]
  if not lang_data then return '' end
  local val = lang_data[self.noteFields.after]
  if not val then
    return ''
  end
  return val
end

function C:getNoteFieldAfterMeters(lang)
  lang = lang or self:selectedCodriverLanguage()
  local lang_data = self.notes[lang]
  if not lang_data then return -1 end
  local val = lang_data[self.noteFields.afterMeters]
  if not val then
    return -1
  end
  return val
end

function C:setNoteFieldBefore(val, meters)
  local lang = self:selectedCodriverLanguage()
  local lang_data = self.notes[lang]
  if not lang_data then return end
  lang_data[self.noteFields.before] = val
  lang_data[self.noteFields.beforeMeters] = meters
end

function C:setNoteFieldFreeform(val, lang)
  lang = lang or self:selectedCodriverLanguage()
  local lang_data = self.notes[lang]
  if not lang_data then return end
  if not lang_data[self.noteFields.note] then
    lang_data[self.noteFields.note] = {}
  end
  lang_data[self.noteFields.note].freeform = val
end

function C:setNoteFieldStructured(val, lang)
  lang = lang or self:selectedCodriverLanguage()
  local lang_data = self.notes[lang]
  if not lang_data then return end
  if not lang_data[self.noteFields.note] then
    lang_data[self.noteFields.note] = {}
  end
  lang_data[self.noteFields.note].structured = val
end

function C:setNoteFieldAfter(val, meters)
  local lang = self:selectedCodriverLanguage()
  local lang_data = self.notes[lang]
  if not lang_data then return end
  lang_data[self.noteFields.after] = val
  lang_data[self.noteFields.afterMeters] = meters
end

function C:setCustomAudioFile(fname, lang)
  lang = lang or self:selectedCodriverLanguage()
  local lang_data = self.notes[lang]
  if not lang_data then return end
  if not lang_data[self.noteFields.note] then
    lang_data[self.noteFields.note] = {}
  end
  if not lang_data[self.noteFields.note].custom then
    lang_data[self.noteFields.note].custom = { audioFile = "", description = "" }
  end
  lang_data[self.noteFields.note].custom.audioFile = fname
end

function C:getCustomAudioFile(lang)
  lang = lang or self:selectedCodriverLanguage()
  local lang_data = self.notes[lang]
  if not lang_data then return '' end
  local custom = lang_data[self.noteFields.note].custom
  if not custom then return '' end
  local val = custom.audioFile
  if not val then
    return ''
  end
  return val
end

function C:setCustomDescription(desc, lang)
  lang = lang or self:selectedCodriverLanguage()
  local lang_data = self.notes[lang]
  if not lang_data then return end
  if not lang_data[self.noteFields.note] then
    lang_data[self.noteFields.note] = {}
  end
  if not lang_data[self.noteFields.note].custom then
    lang_data[self.noteFields.note].custom = { audioFile = "", description = "" }
  end
  lang_data[self.noteFields.note].custom.description = desc
end

function C:getCustomDescription(lang)
  lang = lang or self:selectedCodriverLanguage()
  local lang_data = self.notes[lang]
  if not lang_data then return '' end
  local custom = lang_data[self.noteFields.note].custom
  if not custom then return '' end
  local val = custom.description
  if not val then
    return ''
  end
  return val
end

function C:clearCachedFgData()
  self._cachedCompiledData = nil
end

function C:clearCompilationFailures()
  self._compileFailed = false
end

function C:didCompileFail()
  return self._compileFailed
end

local function getAudioLenTotal(mode, metadata, pacenote, fnames)
  if not metadata then
    -- log('E', logTag, "getAudioLenTotal: no metadata provided")
    return nil
  end

  local total = 0
  for _,fname in ipairs(fnames) do
    local _, basename, _ = path.split(fname)
    local metadataVal = metadata[basename]
    if metadataVal then
      total = total + metadataVal.audioLen
    else
      -- log('E', logTag, "getAudioLenTotal: cant find "..mode.." metadata entry for pacenote=" .. pacenote.name .. " fname=" .. fname)
      -- error('here')
    end
  end
  return total
end

function C:audioLenTotal()
  local compiled = self:asCompiled()

  if self:isAudioModeFreeform() then
    return compiled.audioLenFreeform
  elseif self:isAudioModeStructuredOnline() then
    return compiled.audioLenStructuredOnline
  elseif self:isAudioModeStructuredOffline() then
    return compiled.audioLenStructuredOffline
  elseif self:isAudioModeCustom() then
    return compiled.audioLenCustom
  else
    log('E', logTag, 'audioLenTotal: unknown audio mode')
  end
end

function C:checkFilesExist(audioMode, fnames)
  for _,fname in ipairs(fnames) do
    if not fname or fname == '' then
      return false
    end
    if not FS:fileExists(fname) then
      -- log('E', logTag, "checkFilesExist: cant find "..audioMode.." file for pacenote=" .. self.name .. " fname=" .. fname)
      return false
    end
  end
  return true
end

function C:asCompiled()
  -- TODO reuse validations here.
  if self._compileFailed then
    return nil
  end
  if self._cachedCompiledData then
    return self._cachedCompiledData
  end

  local fnameFreeform = nil
  local fnamesStructuredOnline = nil
  local fnamesStructuredOffline = nil
  local fnameCustom = nil
  local audioLenFreeform = nil
  local audioLenStructuredOnline = nil
  local audioLenStructuredOffline = nil
  local audioLenCustom = nil

  if self:isAudioModeFreeform() then
    fnameFreeform = self:audioFnameFreeform()
    if not self:checkFilesExist('freeform', {fnameFreeform}) then
      -- return nil
    end
    local pacenoteMetadataFreeform = self.notebook:loadFreeformPacenoteMetadata()
    audioLenFreeform = getAudioLenTotal('freeform', pacenoteMetadataFreeform, self, {fnameFreeform})
  elseif self:isAudioModeStructuredOnline() then
    fnamesStructuredOnline = self:audioFnamesStructuredOnline()
    if not self:checkFilesExist('structuredOnline', fnamesStructuredOnline) then
      -- return nil
    end
    local pacenoteMetadataOnlineStructured = self.notebook:loadOnlineStructuredPacenoteMetadata()
    audioLenStructuredOnline = getAudioLenTotal('structuredOnline', pacenoteMetadataOnlineStructured, self, fnamesStructuredOnline)
  elseif self:isAudioModeStructuredOffline() then
    fnamesStructuredOffline = self:audioFnamesStructuredOffline()
    if not self:checkFilesExist('structuredOffline', fnamesStructuredOffline) then
      -- return nil
    end
    local pacenoteMetadataOfflineStructured = self.notebook:loadOfflineStructuredPacenoteMetadata()
    audioLenStructuredOffline = getAudioLenTotal('structuredOffline', pacenoteMetadataOfflineStructured, self, fnamesStructuredOffline)
  elseif self:isAudioModeCustom() then
    fnameCustom = self:getCustomAudioFile()
    dump(fnameCustom)
    if not self:checkFilesExist('custom', {fnameCustom}) then
      -- return nil
    end
    local pacenoteMetadataCustom = self.notebook:loadCustomPacenoteMetadata()
    audioLenCustom = getAudioLenTotal('custom', pacenoteMetadataCustom, self, {fnameCustom})
  else
    log('E', logTag, 'asCompiled: unknown audio mode')
    self._compileFailed = true
    return nil
  end

  if not fnameFreeform and not fnamesStructuredOnline and not fnamesStructuredOffline and not fnameCustom then
    log('E', logTag, 'asCompiled: failed to find any audio files')
    -- self._compileFailed = true
    -- return nil
  end

  local noteText = nil
  if self:useStructured() then
    noteText = dumps(self:noteOutputStructured())
  else
    noteText = self:noteOutputFreeform()
  end

  if not noteText then
    log('E', logTag, 'asCompiled: failed to find any note text')
    -- self._compileFailed = true
    -- return nil
  end

  local distBefore = self:getNoteFieldBefore()
  local distBeforeMeters = self:getNoteFieldBeforeMeters()
  local distAfter = self:getNoteFieldAfter()
  local distAfterMeters = self:getNoteFieldAfterMeters()

  local vc = self.notebook:getVisualCompositor2()
  local visualPacenotes2 = nil
  if not vc then
    log('E', logTag, 'asCompiled: no visual compositor')
    -- self._compileFailed = true
    -- return nil
  else
    visualPacenotes2 = vc:compositeVisual(self, self.structured.fields, distBeforeMeters, distAfterMeters)
  end

  local compiledPacenote = {
    id = self.id,
    name = self.name,
    noteText = noteText,
    audioFnameFreeform = fnameFreeform,
    audioFnamesStructuredOnline = fnamesStructuredOnline,
    audioFnamesStructuredOffline = fnamesStructuredOffline,
    audioFnameCustom = fnameCustom,
    audioLenFreeform = audioLenFreeform,
    audioLenStructuredOnline = audioLenStructuredOnline,
    audioLenStructuredOffline = audioLenStructuredOffline,
    audioLenCustom = audioLenCustom,
    visualPacenotes2 = visualPacenotes2,
    distanceBefore = distBefore,
    distanceAfter = distAfter,
  }
  self._cachedCompiledData = compiledPacenote
  return compiledPacenote
end

function C:validate()
  self.validation_issues = {}

  if self.todo then
    table.insert(self.validation_issues, 'marked TODO')
  end

  if not self:getCornerStartWaypoint() then
    table.insert(self.validation_issues, 'missing CornerStart waypoint')
  end

  if not self:getCornerEndWaypoint() then
    table.insert(self.validation_issues, 'missing CornerEnd waypoint')
  end

  if not self:getActiveFwdAudioTrigger() then
    table.insert(self.validation_issues, 'missing AudioTrigger waypoint')
  end

  if self.name == '' then
    table.insert(self.validation_issues, 'missing pacenote name')
  end

  if self:useStructured() then
    local note_field_structured = self:getNoteFieldStructured()
    if #note_field_structured == 0 then
      table.insert(self.validation_issues, 'missing structured note')
    end
  else
    local note_field_freeform = self:getNoteFieldFreeform()
    if note_field_freeform ~= rallyUtil.autofill_blocker then
      local last_char = note_field_freeform:sub(-1)
      if note_field_freeform == '' then
        table.insert(self.validation_issues, 'missing freeform note for '..self:selectedCodriverLanguage())
      elseif note_field_freeform == rallyUtil.unknown_transcript_str then
        table.insert(self.validation_issues, "'"..rallyUtil.unknown_transcript_str.."' freeform note")
      -- elseif not rallyUtil.hasPunctuation(last_char) then
        -- table.insert(self.validation_issues, 'missing freeform puncuation')
      end
    end
  end
end

function C:getAudioModeSetting()
  return self.audioMode
end

function C:getAudioMode()
  local audioMode = self:getAudioModeSetting()

  -- local audioMode = RallyEnums.pacenoteAudioMode.auto
  -- local audioMode = RallyEnums.pacenoteAudioMode.freeform
  -- local audioMode = RallyEnums.pacenoteAudioMode.structuredOnline
  -- local audioMode = RallyEnums.pacenoteAudioMode.structuredOffline

  if audioMode == RallyEnums.pacenoteAudioMode.auto then
    return self.notebook:getAudioMode()
  else
    return audioMode
  end
end

function C:getAudioModeString()
  local audioMode = self:getAudioMode()
  return RallyEnums.pacenoteAudioModeNames[audioMode]
end

function C:isAudioModeStructuredOnline()
  return self:getAudioMode() == RallyEnums.pacenoteAudioMode.structuredOnline
end

function C:isAudioModeStructuredOffline()
  return self:getAudioMode() == RallyEnums.pacenoteAudioMode.structuredOffline
end

function C:isAudioModeFreeform()
  return self:getAudioMode() == RallyEnums.pacenoteAudioMode.freeform
end

function C:isAudioModeCustom()
  return self:getAudioMode() == RallyEnums.pacenoteAudioMode.custom
end

function C:setAudioMode(mode)
  self.audioMode = mode
end

function C:useStructured()
  if self.metadata.system then
    return false
  else
    return self:isAudioModeStructuredOnline() or self:isAudioModeStructuredOffline()
  end
end

function C:selectedCodriverLanguage()
  return self.notebook:selectedCodriverLanguage()
end

function C:selectedCodriver()
  return self.notebook:selectedCodriver()
end

function C:is_valid()
  return #self.validation_issues == 0
end

function C:pacenoteTextForSelect()
  local txtForSelectionItem = ''
  local preview = self:noteOutputPreview()

  local tokens = split(self.name, " ")
  if #tokens >= 2 then
    txtForSelectionItem = tokens[2]
  end

  -- if self.slowCorner then
  --   txtForSelectionItem = txtForSelectionItem..' [S]'
  -- end

  local triggerType = self:triggerTypeAsSmallIndicator()
  if triggerType then
    -- txtForSelectionItem = txtForSelectionItem..' ['..triggerType..']'
    txtForSelectionItem = txtForSelectionItem..' - ['..triggerType..'] '..preview
  else
    txtForSelectionItem = txtForSelectionItem..' - '..preview
  end


  if self:is_valid() then
    return txtForSelectionItem
  else
    return '[!] '..txtForSelectionItem
  end
end

function C:slowCornerAsText()
  if self.slowCorner then
    return 'slow corner'
  else
    return ''
  end
end

function C:triggerTypeAsText()
  return 'triggerType: '..RallyEnums.triggerTypeName[self.triggerType]
end

function C:triggerTypeAsSmallIndicator()
  if self.triggerType == RallyEnums.triggerType.csImmediate then
    return 'I'
  elseif self.triggerType == RallyEnums.triggerType.csStatic then
    return 'csS'
  elseif self.triggerType == RallyEnums.triggerType.csHalf then
    return 'cs+50%'
  elseif self.triggerType == RallyEnums.triggerType.ceMinus5 then
    return 'ce-5m'
  elseif self.triggerType == RallyEnums.triggerType.ceStatic then
    return 'ceS'
  else
    return nil
  end
end

function C:getCornerStartWaypoint()
  for i,wp in ipairs(self.pacenoteWaypoints.sorted) do
    if wp.waypointType == waypointTypes.wpTypeCornerStart then
      return wp
    end
  end
  return nil
end

-- function C:getHalfwayWaypoint()
  -- return nil
  -- local wp_cs = self:getCornerStartWaypoint()
  -- local wp_ce = self:getCornerEndWaypoint()
  -- if not wp_cs or not wp_ce then
  --   return nil
  -- end

  -- local wp_half = wp_cs:getHalfwayPoint(wp_ce)
  -- return wp_half
-- end

function C:getCornerEndWaypoint()
  for i,wp in ipairs(self.pacenoteWaypoints.sorted) do
    if wp.waypointType == waypointTypes.wpTypeCornerEnd then
      return wp
    end
  end
  return nil
end

-- function C:getAudioTriggerWaypoints()
--   local wps = {}

--   for i,wp in ipairs(self.pacenoteWaypoints.sorted) do
--     if wp.waypointType == waypointTypes.wpTypeFwdAudioTrigger then
--       table.insert(wps, wp)
--     elseif wp.waypointType == waypointTypes.wpTypeRevAudioTrigger then
--       table.insert(wps, wp)
--     end
--   end

--   return wps
-- end

-- function C:getDistanceMarkerWaypointsAfterEnd()
--   local cornerEndFound = false
--   local wps = {}

--   for i,wp in ipairs(self.pacenoteWaypoints.sorted) do
--     if wp.waypointType == waypointTypes.wpTypeCornerEnd then
--       cornerEndFound = true
--     end

--     if cornerEndFound and wp.waypointType == waypointTypes.wpTypeDistanceMarker then
--       table.insert(wps, wp)
--     end
--   end

--   return wps
-- end

-- function C:getDistanceMarkerWaypointsInBetween()
--   local cornerStartFound = false
--   local wps = {}

--   for i,wp in ipairs(self.pacenoteWaypoints.sorted) do
--     if wp.waypointType == waypointTypes.wpTypeCornerStart then
--       cornerStartFound = true
--     elseif wp.waypointType == waypointTypes.wpTypeCornerEnd then
--       break
--     end

--     if cornerStartFound and wp.waypointType == waypointTypes.wpTypeDistanceMarker then
--       table.insert(wps, wp)
--     end
--   end

--   return wps
-- end

-- function C:getDistanceMarkerWaypointsBeforeStart()
--   local wps = {}

--   for i,wp in ipairs(self.pacenoteWaypoints.sorted) do
--     if wp.waypointType == waypointTypes.wpTypeDistanceMarker then
--       table.insert(wps, wp)
--     elseif wp.waypointType == waypointTypes.wpTypeCornerStart then
--       break
--     end
--   end

--   return wps
-- end

-- function C:getDistanceMarkerWaypoints()
--   local wps = {}

--   for i,wp in ipairs(self.pacenoteWaypoints.sorted) do
--     if wp.waypointType == waypointTypes.wpTypeDistanceMarker then
--       table.insert(wps, wp)
--     end
--   end

--   return wps
-- end

function C:onSerialize()
  for lang,langData in pairs(self.notes) do
    -- convert from old to new file format.
    if type(langData.note) == "string" then
      langData.note = {
        freeform = langData.note,
        structured = nil,
      }
    end

    langData._out = {
      freeform = self:noteOutputFreeform(lang),
      structured = self:noteOutputStructured(lang),
    }
  end

  local ret = {
    oldId = self.id,
    name = self.name,
    pk = self.pk,
    playback_rules = self.playback_rules,
    isolate = self.isolate or false,
    triggerType = self.triggerType or RallyEnums.triggerType.dynamic,
    slowCornerReleaseType = self.slowCornerReleaseType or defaultSlowCornerReleaseType,
    audioMode = self.audioMode or RallyEnums.pacenoteAudioMode.auto,
    todo = self.todo or false,
    notes = self.notes,
    metadata = self.metadata,
    pacenoteWaypoints = self.pacenoteWaypoints:onSerialize(),
    structured = self.structured:onSerialize(),
    slowCorner = self.slowCorner,
  }

  return ret
end

function C:onDeserialized(data, oldIdMap)
  self.name = data.name
  self.pk = data.pk or rallyUtil.randomId()
  self.playback_rules = data.playback_rules
  self.isolate = data.isolate or false
  self.slowCorner = data.slowCorner or false
  self.slowCornerReleaseType = data.slowCornerReleaseType or defaultSlowCornerReleaseType

  if data.codriverWait then
    if data.codriverWait == 'none' then
      data.triggerType = RallyEnums.triggerType.dynamic
    end
    if data.codriverWait == 'small' then
      data.triggerType = RallyEnums.triggerType.dynamic
    end
    if data.codriverWait == 'medium' then
      data.triggerType = RallyEnums.triggerType.dynamic
    end
    if data.codriverWait == 'large' then
      data.triggerType = RallyEnums.triggerType.csHalf
    end
  end
  self.triggerType = data.triggerType or RallyEnums.triggerType.dynamic

  self.todo = data.todo or false
  self.notes = data.notes
  self.metadata = data.metadata or {}
  self.pacenoteWaypoints:onDeserialized(data.pacenoteWaypoints, oldIdMap)
  self.audioMode = data.audioMode or RallyEnums.pacenoteAudioMode.auto

  self.structured:onDeserialized(data.structured)
end

function C:upgradeFromV2ToV3()
  for lang,langData in pairs(self.notes) do
    local freeformnote = langData.note
    langData.note = {
      freeform = freeformnote,
      structured = {},
    }
  end
end

function C:markTodo()
  self.todo = true
end

function C:clearTodo()
  self.todo = false
end

function C:setNavgraph(navgraphName, fallback)
  log('W', logTag, 'setNavgraph() not implemented')
end

function C:intersectCorners(fromCorners, toCorners)
  local wp = self:getActiveFwdAudioTrigger()
  if not wp then
    return false
  end
  return wp:intersectCorners(fromCorners, toCorners)
end

function C:getActiveFwdAudioTrigger()
  for i,wp in ipairs(self.pacenoteWaypoints.sorted) do
    if wp.waypointType == waypointTypes.wpTypeFwdAudioTrigger then -- TODO and wp.name == 'curr' then
      return wp
    end
  end
  return nil
end

function C:setAdjacentNotes(prevNote, nextNote)
  self.prevNote = prevNote
  self.nextNote = nextNote
end

function C:clearAdjacentNotes()
  self:setAdjacentNotes(nil, nil)
end

local function textForDrawDebug(drawConfig, selection_state, wp, dist_text, hover)
  local shift = selection_state.shift
  local noteText = wp.pacenote:noteTextForDrawDebug()
  local txt = nil

  if drawConfig.cs_text and wp:isCs() then
    txt = noteText

    if editor_rallyEditor and editor_rallyEditor.getPrefLockWaypoints() and selection_state.selected_pn_id then
      txt = '[LOCK] '..txt
    end

    if shift and hover then
      txt = '[CAMERA LOCK] '..txt
    end

    if not txt or txt == '' then
      txt = '<empty pacenote>'
    end
  elseif drawConfig.ce_text and wp:isCe() then
    txt = '['..waypointTypes.shortenWaypointType(wp.waypointType)
    if dist_text then
      txt = txt..','..dist_text
    end
    txt = txt..']'
  elseif drawConfig.at_text and wp:isAt() then
    if selection_state.selected_wp_id and selection_state.selected_wp_id == wp.id then
      if dist_text then
        txt = '['..dist_text..']'
      end
    elseif drawConfig.pn_drawMode == pn_drawMode_previous or drawConfig.pn_drawMode == pn_drawMode_next then
      txt = '['..waypointTypes.shortenWaypointType(wp.waypointType)
      txt = txt..'] '..noteText
    end
  end

  return txt
end

local function drawWaypoint(drawConfig, selection_state, wp, dist_text)
  if not wp then return end

  local hover_wp_id = selection_state.hover_wp_id
  local selected_wp_id = selection_state.selected_wp_id
  local shift = selection_state.shift
  local hover = hover_wp_id and hover_wp_id == wp.id
  local clr = nil
  local globalOpacity = drawConfig.globalOpacity or 1.0

  local pn_drawMode = drawConfig.pn_drawMode

  local alpha_shape = drawConfig.base_alpha * globalOpacity
  local alpha_text = drawConfig.base_alpha
  local clr_textFg = nil
  local clr_textBg = nil
  local radius_factor = nil

  local pn = wp.pacenote
  local valid = pn:is_valid()

  if pn_drawMode == pn_drawMode_selected then
    alpha_text = cc.pacenote_alpha_text_selected
    if selected_wp_id and selected_wp_id == wp.id then
      clr = wp:colorForWpType(pn_drawMode)
      alpha_shape = cc.waypoint_alpha_selected * globalOpacity
    else
      clr = wp:colorForWpType(pn_drawMode)
    end

    if not valid then
      clr_textFg = cc.clr_white
      clr_textBg = cc.clr_red_dark
    end
  elseif pn_drawMode == pn_drawMode_previous then
    clr = wp:colorForWpType(pn_drawMode)
    if wp:isCs() then
      radius_factor = drawConfig.cs_radius
    elseif wp:isCe() then
      radius_factor = drawConfig.ce_radius
    elseif wp:isAt() then
      radius_factor = drawConfig.at_radius
    end

    if not valid then
      clr_textFg = cc.clr_white
      clr_textBg = cc.clr_red_dark
    end
  elseif pn_drawMode == pn_drawMode_next then
    clr = wp:colorForWpType(pn_drawMode)
    if wp:isCs() then
      radius_factor = drawConfig.cs_radius
    elseif wp:isCe() then
      radius_factor = drawConfig.ce_radius
    elseif wp:isAt() then
      radius_factor = drawConfig.at_radius
    end

    if not valid then
      clr_textFg = cc.clr_white
      clr_textBg = cc.clr_red_dark
    end
  elseif pn_drawMode == pn_drawMode_partitionedSnaproad then
    alpha_text = 1.0
    clr = wp:colorForWpType(pn_drawMode_previous)
    if not valid then
      clr_textFg = cc.clr_white
      clr_textBg = cc.clr_red_dark
    end
    radius_factor = cc.pacenote_adjacent_radius_factor
  elseif pn_drawMode == pn_drawMode_background then
    if valid then
      clr = cc.waypoint_clr_background
    else
      clr = cc.clr_red_dark
      clr_textFg = cc.clr_white
      clr_textBg = cc.clr_red_dark
    end
    radius_factor = cc.pacenote_adjacent_radius_factor
  elseif pn_drawMode == pn_drawMode_noSelection then
    alpha_text = 1.0
    if valid then
      -- rainbow theme
      -- clr = rainbowColor(#wp.pacenote.notebook.pacenotes.sorted, (wp.pacenote.sortOrder-1), 1)

      -- dark green theme
      -- clr = cc.clr_green_dark
      -- clr_textFg = cc.clr_white
      -- clr_textBg = cc.clr_green_dark

      -- dark theme
      clr = cc.waypoint_clr_background
      clr_textFg = cc.clr_white
      clr_textBg = cc.clr_black

      -- light theme
      -- clr = cc.clr_white
      -- clr_textFg = cc.clr_black
      -- clr_textBg = cc.clr_white
    else
      clr = cc.clr_red_dark
      clr_textFg = cc.clr_white
      clr_textBg = cc.clr_red_dark
    end
  end

  local text = textForDrawDebug(drawConfig, selection_state, wp, dist_text, hover)
  wp:drawDebug(hover, text, clr, alpha_shape, alpha_text, clr_textFg, clr_textBg, radius_factor, globalOpacity)
end

local function drawHalfPoint(point, drawConfig, pacenoteToolsState)
  local snaproad = pacenoteToolsState.snaproad
  local clr = cc.snaproads_clr_recce
  if snaproad and snaproad:isRouteSourced() then
    clr = cc.snaproads_clr_route
  end
  local alphaShape = cc.snaproads_alpha * drawConfig.globalOpacity
  local wp_selected = pacenoteToolsState.selected_wp_id

  if wp_selected then
    clr = cc.clr_white
    alphaShape = cc.waypoint_alpha_selected * drawConfig.globalOpacity
  end
  debugDrawer:drawSphere(
    point.pos,
    1.5,
    ColorF(clr[1],clr[2],clr[3], alphaShape)
  )
end

local function formatDistanceStringMeters(dist)
  return tostring(round(dist))..'m'
end

local function prettyDistanceStringMeters(from, to)
  if not (from and to) then return "?m" end
  local d = from.pos:distance(to.pos)
  return formatDistanceStringMeters(d)
end

function C:drawDebugPacenoteHelper(drawConfig, pacenoteToolsState)
  local text_dist = nil
  -- local wp_all_at = self:getAudioTriggerWaypoints()
  local wp_cs = self:getCornerStartWaypoint()
  -- local wp_halfpoint = self:getHalfwayWaypoint()
  local wp_ce = self:getCornerEndWaypoint()
  -- local wp_dist_before_start = self:getDistanceMarkerWaypointsBeforeStart()
  -- local wp_dist_between = self:getDistanceMarkerWaypointsInBetween()
  -- local wp_dist_after_end = self:getDistanceMarkerWaypointsAfterEnd()

  -- (1) draw the fwd audio triggers and link them to CS.
  -- if drawConfig.at then
  --   for _,wp in ipairs(wp_all_at) do
  --     -- distance is from AT to CS
  --     text_dist = prettyDistanceStringMeters(wp, wp_cs)
  --     drawWaypoint(drawConfig, selection_state, wp, text_dist)
  --   end
  -- end

  -- (2) draw beforeStart distance markers, draw link, draw link distance label
  -- if drawConfig.di_before then
  --   for _,wp in ipairs(wp_dist_before_start) do
  --     drawWaypoint(drawConfig, selection_state, wp, text_dist)
  --   end
  -- end

  -- (3) draw the CS
  if drawConfig.cs then
    text_dist = nil
    drawWaypoint(drawConfig, pacenoteToolsState, wp_cs, text_dist)
  end

  -- (3.1) draw the halfpoint
  if drawConfig.halfpoint and self.halfpoint then
    drawHalfPoint(self.halfpoint, drawConfig, pacenoteToolsState)
  end

  -- (4) draw the distance markers, links, and labels, that are between CS and CE
  -- if drawConfig.di_middle then
  --   for _,wp in ipairs(wp_dist_between) do
  --     text_dist = nil
  --     drawWaypoint(drawConfig, selection_state, wp, text_dist)
  --   end
  -- end

  -- (5) draw the CE
  if drawConfig.ce then
    text_dist = nil
    drawWaypoint(drawConfig, pacenoteToolsState,  wp_ce, text_dist)
  end

  -- -- (7) draw the distance markers after CE, links, labels.
  -- if drawConfig.di_after then
  --   for _,wp in ipairs(wp_dist_after_end) do
  --     drawWaypoint(drawConfig, selection_state, wp, text_dist)
  --   end
  -- end
end

function C:waypointForBeforeLink()
  local to_wp = self:getCornerStartWaypoint()
  return to_wp
end

function C:waypointForAfterLink()
  local from_wp = self:getCornerEndWaypoint()
  return from_wp
end

function C:distanceCornerEndToCornerStart(toPacenote)
  local allWaypoints = {}

  local startWp = self:getCornerEndWaypoint()
  table.insert(allWaypoints, startWp)

  -- local selfDistMarkers = self:getDistanceMarkerWaypointsAfterEnd()
  -- for _,wp in ipairs(selfDistMarkers) do
  --   table.insert(allWaypoints, wp)
  -- end

  -- local toDistMarkers = toPacenote:getDistanceMarkerWaypointsBeforeStart()
  -- for _,wp in ipairs(toDistMarkers) do
  --   table.insert(allWaypoints, wp)
  -- end

  local endWp = toPacenote:getCornerStartWaypoint()
  table.insert(allWaypoints, endWp)

  local distance = 0.0
  local lastWp = nil
  for _,wp in ipairs(allWaypoints) do
    if lastWp then
      distance = distance + lastWp.pos:distance(wp.pos)
    end
    lastWp = wp
  end

  return distance
end

function C:noteOutputPreview()
  local preview = nil

  if self:useStructured() then
    local struc = self:noteOutputStructured()
    if #struc > 0 then
      preview = dumps(struc)
      preview = string.gsub(preview, "\n", " ")
    end
  elseif self:isAudioModeFreeform() then
    preview = self:noteOutputFreeform()
  elseif self:isAudioModeCustom() then
    preview = self:getCustomDescription()
  end

  if not preview then
    preview = '<empty>'
  end

  if self.slowCorner then
    preview = '[S] '..preview
  end

  return preview
end

function C:noteTextForDrawDebug()
  return self:noteOutputPreview()
end

local function adjustFromPrefs(drawConfig)
  local show_at = editor_rallyEditor and editor_rallyEditor.getPrefShowAudioTriggers() or false
  drawConfig.at = show_at and drawConfig.at
end

function C:drawDebugPacenotePartitionedSnaproad(pacenoteToolsState)
  local drawConfig = {
    pn_drawMode = pn_drawMode_partitionedSnaproad,
    di_before = false,
    at = false,
    cs = true,
    ce = true,
    di_after = false,
    base_alpha = cc.pacenote_base_alpha_no_sel,
    at_text = false,
    cs_text = true,
    ce_text = false,
    di_text = false,
    cs_radius = cc.pacenote_adjacent_radius_factor,
    ce_radius = cc.pacenote_adjacent_radius_factor,
    at_radius = cc.pacenote_adjacent_radius_factor,
  }
  adjustFromPrefs(drawConfig)
  self:drawDebugPacenoteHelper(drawConfig, pacenoteToolsState)
end

function C:drawDebugPacenoteNoSelection(pacenoteToolsState, globalOpacity)
  local drawConfig = {
    pn_drawMode = pn_drawMode_noSelection,
    di_before = false,
    at = false,
    cs = true,
    di_middle = false,
    ce = false,
    di_after = false,
    base_alpha = cc.pacenote_base_alpha_no_sel,
    at_text = false,
    cs_text = true,
    ce_text = false,
    di_text = false,
    globalOpacity = globalOpacity,
  }
  adjustFromPrefs(drawConfig)
  self:drawDebugPacenoteHelper(drawConfig, pacenoteToolsState)
end

function C:drawDebugPacenoteBackground(pacenoteToolsState, globalOpacity)
  local drawConfig = {
    pn_drawMode = pn_drawMode_background,
    di_before = false,
    at = false,
    cs = true,
    di_middle = false,
    ce = false,
    di_after = false,
    base_alpha = cc.pacenote_base_alpha_background,
    at_text = false,
    cs_text = true,
    ce_text = false,
    di_text = false,
    globalOpacity = globalOpacity,
  }
  adjustFromPrefs(drawConfig)
  self:drawDebugPacenoteHelper(drawConfig, pacenoteToolsState)
end

function C:drawDebugPacenoteNext(pacenoteToolsState, pn_sel, globalOpacity)
  local drawConfig = {
    pn_drawMode = pn_drawMode_next,
    di_before = false,
    at = true,
    cs = true,
    di_middle = false,
    ce = true,
    di_after = false,
    base_alpha = cc.pacenote_base_alpha_next,
    at_text = false,
    cs_text = true,
    ce_text = false,
    di_text = false,
    cs_radius = cc.pacenote_adjacent_radius_factor,
    ce_radius = cc.pacenote_adjacent_radius_factor,
    at_radius = cc.pacenote_adjacent_radius_factor,
    globalOpacity = globalOpacity,
  }
  adjustFromPrefs(drawConfig)
  self:drawDebugPacenoteHelper(drawConfig, pacenoteToolsState)
end

function C:drawDebugPacenotePrev(pacenoteToolsState, pn_sel, globalOpacity)
  local drawConfig = {
    pn_drawMode = pn_drawMode_previous,
    di_before = false,
    at = true,
    cs = true,
    di_middle = true,
    ce = true,
    di_after = true,
    base_alpha = cc.pacenote_base_alpha_prev,
    at_text = false,
    cs_text = true,
    ce_text = false,
    di_text = false,
    cs_radius = cc.pacenote_adjacent_radius_factor,
    ce_radius = cc.pacenote_adjacent_radius_factor,
    at_radius = cc.pacenote_adjacent_radius_factor,
    globalOpacity = globalOpacity,
  }
  adjustFromPrefs(drawConfig)
  self:drawDebugPacenoteHelper(drawConfig, pacenoteToolsState)
end

function C:drawDebugPacenoteSelected(pacenoteToolsState, globalOpacity)
  local drawConfig = {
    pn_drawMode = pn_drawMode_selected,
    di_before = true,
    at = true,
    cs = true,
    halfpoint = true,
    di_middle = true,
    ce = true,
    di_after = true,
    base_alpha = cc.pacenote_base_alpha_selected,
    at_text = true,
    cs_text = true,
    ce_text = false,
    di_text = false,
    globalOpacity = globalOpacity,
  }
  adjustFromPrefs(drawConfig)
  self:drawDebugPacenoteHelper(drawConfig, pacenoteToolsState)
end

function C:audioFnameFreeform()
  local noteStr = self:noteOutputFreeform()
  local fname = self.notebook:missionPacenoteAudioFile(rallyUtil.freeformDir, noteStr)
  return fname
end

function C:audioFnamesStructuredOnline()
  local noteStrs = self:noteOutputStructured()
  local fnamesOut = {}

  for _,noteStr in ipairs(noteStrs) do
    local fname = self.notebook:missionPacenoteAudioFile(rallyUtil.structuredDir, noteStr)
    table.insert(fnamesOut, fname)
  end

  return fnamesOut
end

function C:audioFnamesStructuredOffline()
  local noteStrs = self:noteOutputStructured()
  local compositorVoice = settings.getValue("rallyTextCompositorVoice")
  local notesOut = {}

  for _,noteStr in ipairs(noteStrs) do
    local pacenoteHash = rallyUtil.pacenoteHashSha1(noteStr)
    local pacenoteFname = rallyUtil.getCompositorPacenoteFile(compositorVoice, pacenoteHash)
    table.insert(notesOut, pacenoteFname)
  end

  return notesOut
end

function C:playbackAllowed(currLap, maxLap)
  -- local context = { currLap = currLap, maxLap = maxLap }
  local condition = self.playback_rules
  currLap = currLap or -1
  maxLap = maxLap or -1

  -- log('D', logTag,
  --   "playbackAllowed name='"..self.name..
  --   "' condition='"..tostring(condition)..
  --   "' currLap="..tostring(currLap..
  --   " maxLap="..tostring(maxLap)))

  -- If condition is nil or empty/whitespace string, return true
  if condition == nil or condition:match("^%s*$") then
    return true, nil
  end

  -- Lowercase the condition for case-insensitive comparison
  local lowerCondition = condition:lower()

  -- Check for 'true' or 't'
  if lowerCondition == 'true' or lowerCondition == 't' then
    return true, nil
  end

  -- Check for 'false' or 'f'
  if lowerCondition == 'false' or lowerCondition == 'f' then
    return false, nil
  end

  -- Attempt to load the condition as Lua code
  local func, err = loadstring("return " .. condition)
  if func then
    -- Function compiled successfully, now execute it safely
    setfenv(func, context)
    local status, result = pcall(func, context)
    if status then
      return result, nil
    else
      -- Handle runtime error in the function
      return false, "Runtime error in condition: " .. result
    end
  else
    -- Handle syntax error in the condition
    return false, "Syntax error in condition: " .. err
  end
end

function C:vehiclePlacementPosAndRot(distAway)
  distAway = distAway or 15
  -- local at = self:getActiveFwdAudioTrigger()
  local cs = self:getCornerStartWaypoint()
  local ce = self:getCornerEndWaypoint()

  local wp_pos = cs
  local wp_dir = ce

  if wp_dir and wp_pos then
    local pos1 = wp_pos.pos + (wp_pos.normal * distAway)
    local pos2 = wp_pos.pos + (-wp_pos.normal * distAway)
    local pos = nil

    if wp_dir.pos:distance(pos1) > wp_dir.pos:distance(pos2) then
      pos = pos1
    else
      pos = pos2
    end

    local fwd = wp_pos.pos - pos
    local up = vec3(0,0,1)
    local rot = quatFromDir(fwd, up):normalized()

    return pos, rot
  else
    return nil, nil
  end
end

function C:nameComponents()
  local baseName, number = string.match(self.name, "(.-)%s*([%d%.]+)$")
  return baseName, number
end

function C:matchesSearchPattern(searchPattern)
  for lang,note in pairs(self.notes) do
    local fullNote = self:noteOutputFreeform(lang)
    -- log('D', 'wtf', 'matching "'..fullNote..'" against "'..searchPattern..'"')
    if rallyUtil.matchSearchPattern(searchPattern, fullNote) then
      return true
    end
  end

  return false
end

function C:_moveWaypointTowardsStepper(snaproad, wp, fwd)
  local newSnapPoint = nil

  if fwd then
    newSnapPoint = snaproad:nextSnapPoint(wp.pos)
  else
    newSnapPoint = snaproad:prevSnapPoint(wp.pos)
  end

  if newSnapPoint then
    wp:setPos(newSnapPoint.pos)
    wp.pacenote.notebook:autofillDistanceCalls()
    local normalVec = snaproad:forwardNormalVec(newSnapPoint)
    if normalVec then
      wp:setNormal(normalVec)
    end

    if wp:isCs() then
      local pn_sel = wp.pacenote
      local wp_sel = wp

      local wp_at = pn_sel:getActiveFwdAudioTrigger()

      local point_cs = snaproad:closestSnapPoint(wp_sel.pos)
      local point_at = snaproad:closestSnapPoint(wp_at.pos, true)

      if point_cs.id <= point_at.id then
        point_at = snaproad:pointsBackwards(point_cs, 1)
        wp_at.pos = point_at.pos

        local normalVec = snaproad:forwardNormalVec(point_at)
        if normalVec then
          wp_at:setNormal(normalVec)
        end
      end

    end
  end
end

function C:moveWaypointTowards(snaproads, wp, fwd, step)
  step = step or 1
  for _ = 1,step do
    self:_moveWaypointTowardsStepper(snaproads, wp, fwd)
  end
end

function C:isLastPacenote()
  return self.id == self.notebook.pacenotes.sorted[#self.notebook.pacenotes.sorted].id
end

function C:refreshFreeform()
  local lang = self:selectedCodriverLanguage()
  local note = self:getNoteFieldFreeform()
  local last = self:isLastPacenote()

  note = rallyUtil.trimString(note)

  if note == rallyUtil.autofill_blocker then return end
  if note == '' then return end
  if note == rallyUtil.unknown_transcript_str then return end

  local managePunctuation = false

  if managePunctuation then
    -- Get the last character of the note
    local lastChar = note:sub(-1)

    -- Remove any existing punctuation before adding new punctuation
    if rallyUtil.hasPunctuation(lastChar) then
      note = string.sub(note, 1, -2)
      lastChar = note:sub(-1)
    end

    -- Add punctuation if not present
    if not rallyUtil.hasPunctuation(lastChar) then
      local punc = nil
      if last then
        punc = self.notebook:getTextCompositor():getPunctuationLastNote()
      else
        punc = self.notebook:getTextCompositor():getPunctuationDefault()
      end
      note = note..punc
    end

    note = rallyUtil.trimString(note)
  end

  self:setNoteFieldFreeform(note)
end

function C:getBreathConfig()
  local compositor = self.notebook:getTextCompositor()
  return compositor:getBreathConfig()
end

function C:toggleSlowCorner()
  self.slowCorner = not self.slowCorner
end

function C:toggleIsolate()
  self.isolate = not self.isolate

  if self.isolate then
    if self:getNoteFieldBefore() ~= rallyUtil.autofill_blocker then
      self:setNoteFieldBefore(rallyUtil.autodist_internal_level1, -1)
    end
    if self:getNoteFieldAfter() ~= rallyUtil.autofill_blocker then
      self:setNoteFieldAfter(rallyUtil.autodist_internal_level1, -1)
    end
  else
    if self:getNoteFieldBefore() ~= rallyUtil.autofill_blocker then
      -- setting to an empty string will allow autoFillDistanceCalls to do it's thing.
      self:setNoteFieldBefore('', -1)
    end
    if self:getNoteFieldAfter() ~= rallyUtil.autofill_blocker then
      self:setNoteFieldAfter('', -1)
    end
  end
end

function C:setTriggerType(val)
  self.triggerType = val
end

function C:getTriggerType()
  return self.triggerType
end

function C:setSlowCornerReleaseType(val)
  self.slowCornerReleaseType = val
end

function C:getSlowCornerReleaseType()
  return self.slowCornerReleaseType
end

function C:isSlowCornerReleaseCsHalf()
  return self.slowCornerReleaseType == RallyEnums.slowCornerReleaseType.csHalf
end

function C:isSlowCornerReleaseCsStatic()
  return self.slowCornerReleaseType == RallyEnums.slowCornerReleaseType.csStatic
end

function C:isSlowCornerReleaseCeStatic()
  return self.slowCornerReleaseType == RallyEnums.slowCornerReleaseType.ceStatic
end

function C:isSlowCornerReleaseCeMinus5()
  return self.slowCornerReleaseType == RallyEnums.slowCornerReleaseType.ceMinus5
end

function C:refreshStructured()
  local distBefore = self:getNoteFieldBefore()
  local distAfter = self:getNoteFieldAfter()

  local compositor = self.notebook:getTextCompositor()
  local humanReadable = compositor:compositeText(self.structured, distBefore, distAfter)

  self:setNoteFieldStructured(humanReadable)
end

function C:getPosForOrbitCamera()
  if self.halfpoint then
    return self.halfpoint.pos
  else
    local wp = self:getCornerStartWaypoint()
    return wp.pos
  end
end

function C:setCachedLength(len)
  self._cachedLength = len
end

function C:getCachedLength()
  return self._cachedLength
end

function C:generateFreeformFromStructured()
  local note = self:getNoteFieldFreeform()
  local distBefore = self:getNoteFieldBefore()
  local distAfter = self:getNoteFieldAfter()
  local compositor = self.notebook:getTextCompositor()
  -- local punc = compositor:getPunctuationDistanceCalls()
  local punc = ''

  local varEscapedStructured = compositor:compositeTextEscaped(self.structured, distBefore, distAfter)
  local generatedFreeform = table.concat(varEscapedStructured, ' ')

  local varEscapedFreeform = _noteOutputFreeformWithVarsHelper(note, distBefore, distAfter)

  local specialBefore = ''
  local specialAfter = ''
  if rallyUtil.useNote(distBefore) then
    specialBefore = rallyUtil.var_db
  end
  if rallyUtil.useNote(distAfter) then
    specialAfter = rallyUtil.var_da
  end

  varEscapedFreeform = _interpolateFreeformVars(varEscapedFreeform, specialBefore, specialAfter, punc)

  -- if no custom var placement, then remove them.
  if varEscapedFreeform == generatedFreeform then
    generatedFreeform = string.gsub(generatedFreeform, rallyUtil.var_db, '')
    generatedFreeform = string.gsub(generatedFreeform, rallyUtil.var_da, '')
    generatedFreeform = rallyUtil.trimString(generatedFreeform)
  end

  self:setNoteFieldFreeform(generatedFreeform)
end

function C:canDeleteAudioFiles()
  return self:getAudioMode() == RallyEnums.pacenoteAudioMode.structuredOnline or
         self:getAudioMode() == RallyEnums.pacenoteAudioMode.freeform
end

function C:deleteAudioFiles()
  if not self:canDeleteAudioFiles() then return end
  if self:getAudioMode() == RallyEnums.pacenoteAudioMode.structuredOnline then
    local fnames = self:audioFnamesStructuredOnline()
    for _,fname in ipairs(fnames) do
      FS:removeFile(fname)
      log('I', logTag, 'deleted StructuredOnline audio file: '..fname)
    end
  elseif self:getAudioMode() == RallyEnums.pacenoteAudioMode.freeform then
    local fnameFreeform = self:audioFnameFreeform()
    FS:removeFile(fnameFreeform)
    log('I', logTag, 'deleted Freeform audio file: '..fnameFreeform)
  end
end

function C:deleteLanguage(lang)
  self.notes[lang] = nil
end

function C:customAudioFileDir()
  return self.notebook:customPacenotesDir()
end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end
