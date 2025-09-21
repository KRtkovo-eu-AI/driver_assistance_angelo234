-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local logTag = ''
local C = {}
local rallyUtil = require('/lua/ge/extensions/gameplay/rally/util')
local RallyEnums = require('/lua/ge/extensions/gameplay/rally/enums')
local MissionSettings = require('/lua/ge/extensions/gameplay/rally/notebook/missionSettings')
local VisualCompositor = require('/lua/ge/extensions/gameplay/rally/notebook/structured/visualCompositor')
local TextCompositor = require('/lua/ge/extensions/gameplay/rally/notebook/structured/textCompositor')
local LibCompositor = require('/lua/ge/extensions/gameplay/rally/notebook/structured/libCompositor')

local currentVersion = "3"

function C:getNextUniqueIdentifier()
  self._uid = self._uid + 1
  return self._uid
end

function C:init(name)
  self._uid = 0

  self.name = name or "Primary"
  self.description = ""
  self.authors = ""
  self.version = currentVersion
  self.created_at = os.time()
  self.updated_at = self.created_at

  self.codrivers = require('/lua/ge/extensions/gameplay/util/sortedList')(
    "codrivers",
    self,
    require('/lua/ge/extensions/gameplay/rally/notebook/codriver')
  )

  self.codrivers:create() -- add default

  self.pacenotes = require('/lua/ge/extensions/gameplay/util/sortedList')(
    "pacenotes",
    self,
    require('/lua/ge/extensions/gameplay/rally/notebook/pacenote')
  )

  self.id = self:getNextUniqueIdentifier()
  self.fname = nil
  self.validation_issues = {}

  self.audioMode = RallyEnums.pacenoteAudioMode.structuredOffline

  self.textCompositor = nil
  self.visualCompositor = nil
end

function C:getSnaproadType()
  local drivelineMode = self:missionSettings():getDrivelineMode()
  if not drivelineMode then
    return nil
  end
  return RallyEnums.drivelineModeNames[drivelineMode]
end

function C:getTextCompositor()
  local lang = self:selectedCodriverLanguage()

  if not lang then
    log('E', logTag, 'getTextCompositor: selected codriver lang is nil')
    return nil
  end

  -- log('D', logTag, 'getTextCompositor: lang: '..lang)

  if not self.textCompositor or lang ~= self.textCompositor.compositorName then
    self.textCompositor = TextCompositor(lang)
    if not self.textCompositor:load() then
      log('E', logTag, 'getTextCompositor: failed to load text compositor')
      self.textCompositor = nil
    end
  end
  -- log('D', logTag, 'getTextCompositor: textCompositor: '..self.textCompositor.compositorName)
  return self.textCompositor
end

function C:getVisualCompositor2()
  if not self.visualCompositor then
    if not self:getTextCompositor() then
      log('E', logTag, 'getVisualCompositor2: no text compositor')
      self.visualCompositor = nil
    else
      self.visualCompositor = VisualCompositor(self:getTextCompositor())
    end
  end
  return self.visualCompositor
end

function C:appendPacenotes(pacenotes)
  for _,pn in ipairs(pacenotes) do
    local newPn = self.pacenotes:create()
    newPn:onDeserialized(pn, {})
  end
end

function C:deleteAllPacenotes()
  self.pacenotes = require('/lua/ge/extensions/gameplay/util/sortedList')(
    "pacenotes",
    self,
    require('/lua/ge/extensions/gameplay/rally/notebook/pacenote')
  )
end

-- a path looks like:
-- /gameplay/missions/gridmap_v2/rallyStage/rally-test-1/aipacenotes/notebooks/primary.notebook.json
-- we want to return this string: gridmap_v2/rallyStage/rally-test-1
function C:missionId()
  local missionDir = self:getMissionDir()
  if not missionDir then return nil end
  -- Match everything after /missions/
  local pattern = "/missions/(.+)$"
  local segment = missionDir:match(pattern)
  return segment
end

function C:getMissionDir()
  if not self.fname then return nil end

  -- looks like: C:\...gameplay\missions\pikespeak\rallyStage\aip-pikes-peak-2\rally\notebooks\
  local notebooksDir = self:dir()
  -- log('D', 'wtf', 'notebooksDir: '..notebooksDir)
  local rallyDir = rallyUtil.stripBasename(notebooksDir)
  -- log('D', 'wtf', 'rallyDir: '..rallyDir)
  -- now looks like: C:\...gameplay\missions\pikespeak\rallyStage\aip-pikes-peak-2\rally
  local missionDir = rallyUtil.stripBasename(rallyDir)
  -- log('D', 'wtf', 'missionDir: '..missionDir)
  -- now looks like: C:\...gameplay\missions\pikespeak\rallyStage\aip-pikes-peak-2

  return missionDir
end

function C:dir()
  if not self.fname then return nil end
  local dir, filename, ext = path.split(self.fname)
  return dir
end

function C:basename()
  if not self.fname then return nil end
  local dir, filename, ext = path.split(self.fname)
  return filename
end

function C:basenameNoExt()
  if not self.fname then return nil end
  local _, filename, _ = path.splitWithoutExt(self.fname)
  _, filename, _ = path.splitWithoutExt(filename)
  return filename
end

function C:setFname(newFname)
  self.fname = newFname
end

function C:save(fname)
  fname = fname or self.fname
  if not fname then
    log('W', logTag, 'couldnt save notebook because no filename was set')
    return false
  end

  self.updated_at = os.time()

  local json = self:onSerialize()
  local saveOk = jsonWriteFile(fname, json, true)
  if not saveOk then
    log('E', logTag, 'error saving notebook')
  end
  log('I', logTag, 'saved notebook: '..fname)

  if saveOk then
    local currCodriverId = self:missionSettings():getCodriverId()
    if not currCodriverId then
      -- dump(currCodriverId)
      self:useCodriver(self:getFirstCodriver())
    end
  end

  return saveOk
end

function C:reload()
  if not self.fname then return end

  local json = jsonReadFile(self.fname)
  if not json then
    log('E', logTag, 'couldnt find notebook file')
  end

  self:onDeserialized(json)
end

function C:validate()
  self.validation_issues = {}

  local codriverNameSet = {}
  local uniqueCount = 0
  for _,codriver in ipairs(self.codrivers.sorted) do
    if not codriverNameSet[codriver.name] then
      -- Count only if the name hasn't been encountered before
      uniqueCount = uniqueCount + 1
      codriverNameSet[codriver.name] = true
    end
  end
  if uniqueCount < #self.codrivers.sorted then
    table.insert(self.validation_issues, 'Duplicate codriver names')
  elseif #self.codrivers.sorted == 0 then
    table.insert(self.validation_issues, 'At least one Codriver is required')
  end
end

function C:is_valid()
  return #self.validation_issues == 0
end

local function extractTrailingNumber(str)
  local num = string.match(str, "%d+%.?%d*$")
  return num and tonumber(num) or nil
end

local function sortByNameNumeric(a, b)
  local numA = extractTrailingNumber(a.name)
  local numB = extractTrailingNumber(b.name)

  if numA and numB then
    -- If both have numbers, compare by number
    return numA < numB
  elseif numA then
    -- If only a has a number, it comes first
    return true
  elseif numB then
    -- If only b has a number, it comes first
    return false
  else
    -- If neither has a number, compare by name
    return a.name < b.name
  end
end

function C:sortPacenotesByName()
  local newList = {}
  for i, v in ipairs(self.pacenotes.sorted) do
    table.insert(newList, v)
  end

  table.sort(newList, sortByNameNumeric)

  -- Assign "sortOrder" in the sorted list
  for i, v in ipairs(newList) do
    v.sortOrder = i
  end

  self.pacenotes:sort()
end

function C:nextImportIdent()
  local importIdentifiers = {}

  for _, pacenote in ipairs(self.pacenotes.sorted) do
    -- Extract the alphanumeric identifier from pacenote names that match "Import_X"
    local identifier = string.match(pacenote.name, "^Import_([%w]+)")
    if identifier then
      table.insert(importIdentifiers, identifier)
    end
  end

  -- Sort the identifiers and return the last one
  if #importIdentifiers > 0 then
    table.sort(importIdentifiers)
    local letter = importIdentifiers[#importIdentifiers]
    local asciiValue = string.byte(letter)
    local nextAsciiValue = asciiValue + 1
    local nextLetter = string.char(nextAsciiValue)
    -- if you hit Z, it will return non alphabetic chars.
    return nextLetter
  end

  return 'A' -- have to start somewhere
end

function C:cleanupPacenoteNames()
  for i, v in ipairs(self.pacenotes.sorted) do
    -- Pattern to match a name ending with a number: capture the non-numeric part and the numeric part
    local baseName, number = string.match(v.name, "(.-)%s*([%d%.]+)$")

    if baseName and number then
      -- If the name has a number at the end, replace it with the new index
      v.name = baseName .. " " .. i
    else
      -- If the name does not have a number at the end, append the index
      v.name = v.name .. " " .. i
    end
  end

  -- re-index names.
  self.pacenotes:buildNamesDir()
end

local function drawPacenotesAsRainbow(pacenotes, selection_state, globalOpacity)
  for _,pacenote in ipairs(pacenotes) do
    pacenote:drawDebugPacenoteNoSelection(selection_state, globalOpacity)
  end
end

local function drawPacenotesAsBackground(pacenotes, skip_pn, selection_state, globalOpacity)
  local skip_i = nil
  for i,pacenote in ipairs(pacenotes) do
    if pacenote.id == skip_pn.id then
      skip_i = i
      break
    end
  end

  if skip_i then
    local show_after_count = 1
    local start_i = skip_i+1
    local end_i = start_i+show_after_count-1
    for i = start_i,end_i do
      local pacenote = pacenotes[i]
      if pacenote then
        pacenote:drawDebugPacenoteBackground(selection_state, globalOpacity)
      end
    end

    local show_before_count = 1
    start_i = skip_i-show_before_count
    end_i = skip_i-1
    for i = start_i,end_i do
      local pacenote = pacenotes[i]
      if pacenote then
        pacenote:drawDebugPacenoteBackground(selection_state, globalOpacity)
      end
    end
  end
end

function C:getAdjacentPacenoteSet(selected_pn_id)
  local pacenotes = self.pacenotes.sorted

  local function getOrNullify(i)
    local pn = pacenotes[i]
    if pn and not pn.missing then
      return pn
    else
      return nil
    end
  end

  for i,pacenote in ipairs(pacenotes) do
    if pacenote.id == selected_pn_id then
      return getOrNullify(i-1), pacenote, getOrNullify(i+1)
    end
  end

  return nil, nil, nil
end

function C:drawDebugNotebook(pacenoteToolsState, globalOpacity)
  pacenoteToolsState = pacenoteToolsState or { hover_wp_id = nil, selected_wp_id = nil }
  local pacenotes = self.pacenotes.sorted
  local pn_prev, pn_sel, pn_next = self:getAdjacentPacenoteSet(pacenoteToolsState.selected_pn_id)

  if pn_sel and pacenoteToolsState.selected_wp_id then
    pn_sel:drawDebugPacenoteSelected(pacenoteToolsState, globalOpacity)

    if editor_rallyEditor.getPrefShowPreviousPacenote() and pn_prev and pn_prev.id ~= pn_sel.id then
      pn_prev:drawDebugPacenotePrev(pacenoteToolsState, pn_sel, globalOpacity)
    end

    if editor_rallyEditor.getPrefShowNextPacenote() and pn_next and pn_next.id ~= pn_sel.id then
      pn_next:drawDebugPacenoteNext(pacenoteToolsState, pn_sel, globalOpacity)
    end
  elseif pn_sel then
    pn_sel:drawDebugPacenoteSelected(pacenoteToolsState, globalOpacity)
    drawPacenotesAsBackground(pacenotes, pn_sel, pacenoteToolsState, globalOpacity)
  else
    drawPacenotesAsRainbow(pacenotes, pacenoteToolsState, globalOpacity)
  end
end

function C:drawDebugNotebookForPartitionedSnaproad(pacenoteToolsState, globalOpacity)
  local pacenotes = self.pacenotes.sorted

  local selectionState = {
    hover_wp_id = nil,
    selected_wp_id = nil,
  }
  for _,pacenote in ipairs(pacenotes) do
    pacenote:drawDebugPacenotePartitionedSnaproad(pacenoteToolsState, selectionState)
  end
end

-- convert text compositor system pacenotes into a list of the expected pacenote format.
function C:allStaticPacenotesForSerialize()
  -- stage 1: get all the system pacenotes into a list
  local stage1 = {}
  local langs = self:getLanguages()

  for _,langData in ipairs(langs) do
    local lang = langData.language
    local tc = TextCompositor(lang)
    if not tc:load() then
      log('E', logTag, 'onSerialize: unable to load text compositor for language: '..lang)
    else
      local sp = tc:getSystemPacenotes()
      for key,val in pairs(sp) do
        for i,variant in ipairs(val) do
          local pnName = key..'_'..i
          local pn = {
            name = pnName,
            text = variant.text,
            lang = lang,
            metadata = {
              static = true, -- legacy field
              system = true,
              chill = variant.chill,
              weight = variant.weight
            },
          }
          table.insert(stage1, pn)
        end
      end
    end
  end

  -- stage 2: convert the list into a map, merging notes with the same name
  local stage2 = {}
  local nextOldId = 1
  for _,pn in ipairs(stage1) do
    if not stage2[pn.name] then
      stage2[pn.name] = {
        oldId = nextOldId,
        name = pn.name,
        metadata = pn.metadata,
        notes = {}
      }
      nextOldId = nextOldId + 1
    end


    if not stage2[pn.name].notes then
      stage2[pn.name].notes = {}
    end

    stage2[pn.name].notes[pn.lang] = {
      note = {
        freeform = pn.text
      }
    }
  end

  -- stage 3: convert the map back into a list
  local stage3 = {}
  for k,v in pairs(stage2) do
    table.insert(stage3, v)
  end

  return stage3
end

function C:onSerialize()
  local langs = self:getLanguages()
  local sp = self:allStaticPacenotesForSerialize()

  local ret = {
    name = self.name,
    description = self.description,
    authors = self.authors,
    updated_at = self.updated_at,
    created_at = self.created_at,
    codrivers = self.codrivers:onSerialize(),
    pacenotes = self.pacenotes:onSerialize(),
    audioMode = self.audioMode,
    version = self.version,
    systemPacenotes = sp,
  }

  return ret
end

function C:onDeserialized(data)
  if not data then return end

  if not data.version then
    self.version = currentVersion
  else
    self.version = data.version
  end

  self.name = data.name or ""
  self.description = string.gsub(data.description or "", "\\n", "\n")
  self.authors = data.authors or ""
  self.created_at = data.created_at
  self.updated_at = data.updated_at
  self.audioMode = data.audioMode or RallyEnums.pacenoteAudioMode.structuredOffline

  local oldIdMap = {}

  self.codrivers:clear()
  self.codrivers:onDeserialized(data.codrivers, oldIdMap)

  self.pacenotes:clear()
  self.pacenotes:onDeserialized(data.pacenotes, oldIdMap)

  if self:isV2() then
    self:upgradeFromV2ToV3()
  end
end

function C:upgradeFromV2ToV3()
  log('I', logTag, 'upgrading '..self.name..' from v2 to v3')
  for _, pacenote in ipairs(self.pacenotes.sorted) do
    pacenote:upgradeFromV2ToV3()
  end

  self.version = "3"

  self:autofillDistanceCalls()
end

function C:allWaypoints()
  local wps = {}
  for i,pacenote in pairs(self.pacenotes.objects) do
    for j,wp in pairs(pacenote.pacenoteWaypoints.objects) do
      wps[wp.id] = wp
    end
  end
  return wps
end

function C:getWaypoint(wpId)
  for i, pacenote in pairs(self.pacenotes.objects) do
    for i, waypoint in pairs(pacenote.pacenoteWaypoints.objects) do
      if waypoint.id == wpId then
        return waypoint
      end
    end
  end
  return nil
end

function C:getLanguages()
  local lang_set = {}
  for _, codriver in pairs(self.codrivers.objects) do
    if not lang_set[codriver.language] then
      lang_set[codriver.language] = {}
    end
    table.insert(lang_set[codriver.language], codriver)
  end
  local languages = {}
  for lang, codrivers in pairs(lang_set) do
    table.insert(languages, { language = lang , codrivers = codrivers })
  end
  table.sort(languages, function(a, b)
    if a.lang and b.lang then
      return a.lang < b.lang
    else
      return false
    end
  end)
  return languages
end

function C:setAllRadii(newRadius, wpType)
  for i, pacenote in pairs(self.pacenotes.objects) do
    pacenote:setAllRadii(newRadius, wpType)
  end
end

function C:allToTerrain()
  for i, pacenote in pairs(self.pacenotes.objects) do
    pacenote:allToTerrain()
  end
end

function C:reloadMissionSettings()
  self._missionSettings = nil
  self:missionSettings()
end

function C:missionSettings()
  if self._missionSettings then
    return self._missionSettings
  end

  -- local missionSettingsPath = rallyUtil.getMissionSettingsFile(self:getMissionDir())
  local missionSettings = MissionSettings(self:getMissionDir())
  missionSettings:load(self)
  self._missionSettings = missionSettings
  return missionSettings
end

function C:selectedCodriver()
  -- self:reloadMissionSettings()
  -- if not self:missionSettings() then
  --   error('must set mission settings on notebook')
  -- end

  -- local codriverName = self:missionSettings():getCodriverName()
  -- -- log('D', logTag, 'selectedCodriver codriverName: '..codriverName)
  -- local codriver = self:getCodriverByName(codriverName)

  local codriverPk = self:missionSettings():getCodriverPk()
  -- local codriver = self:getCodriverById(codriverPk)
  local codriver = self:getCodriverByPk(codriverPk)

  if not codriver or codriver.missing then
    -- log('E', logTag, 'selectedCodriver no codriver found')
    -- error('couldnt load codriver: '..self:missionSettings().notebook.codriver)
    codriver = self:getFirstCodriver()
  end

  if not codriver then
    error('no codriver found')
  end

  return codriver
end

function C:selectedCodriverLanguage()
  local codriver = self:selectedCodriver()
  return codriver.language
end

function C:refreshAllPacenotes()
  self:refreshAllStructuredNotes()
  self:refreshAllFreeformNotes()
end

function C:refreshAllStructuredNotes()
  self.textCompositor = nil -- clear the cached text compositor
  for _,pacenote in ipairs(self.pacenotes.sorted) do
    pacenote:refreshStructured()
  end
end

function C:refreshAllFreeformNotes()
  for _,pacenote in ipairs(self.pacenotes.sorted) do
    pacenote:refreshFreeform()
  end
end

function C:generateAllFreeform()
  for _,pacenote in ipairs(self.pacenotes.sorted) do
    pacenote:generateFreeformFromStructured()
  end
end

function C:autofillDistanceCalls()
  -- first clear everything
  for _,pacenote in ipairs(self.pacenotes.sorted) do
    if pacenote:getNoteFieldBefore() ~= rallyUtil.autofill_blocker and pacenote:getNoteFieldBefore() ~= rallyUtil.autodist_internal_level1 then
      pacenote:setNoteFieldBefore('', -1)
    end
    if pacenote:getNoteFieldAfter() ~= rallyUtil.autofill_blocker and pacenote:getNoteFieldAfter() ~= rallyUtil.autodist_internal_level1 then
      pacenote:setNoteFieldAfter('', -1)
    end
  end

  local nextPrepend = ''
  local nextPrependDist = 0

  for i,pacenote in ipairs(self.pacenotes.sorted) do
    -- Apply any prepended text from the previous iteration
    if not pacenote.isolate and nextPrepend ~= '' then
      if pacenote:getNoteFieldBefore() ~= rallyUtil.autofill_blocker then
        pacenote:setNoteFieldBefore(nextPrepend, nextPrependDist)
      end
      nextPrepend = ''
      nextPrependDist = 0
    end

    local pn_next = self:findNextNonIsolated(i)

    if not pacenote.isolate and pn_next and not pn_next.missing then
      local dist = pacenote:distanceCornerEndToCornerStart(pn_next)
      local distStr = self:getTextCompositor():distanceToString(dist)

      -- Decide what to do based on the distance
      local shorthand = self:getTextCompositor():getDistanceCallShorthand(dist)
      if shorthand then
        nextPrepend = shorthand
        nextPrependDist = dist
      else
        if pacenote:getNoteFieldAfter() ~= rallyUtil.autofill_blocker then
          pacenote:setNoteFieldAfter(distStr, dist)
        end
      end
    end
  end
end

function C:findNextNonIsolated(i)
  i = i + 1
  local pn_next = self.pacenotes.sorted[i]
  while pn_next and pn_next.isolate do
    pn_next = self.pacenotes.sorted[i]
    i = i+1
  end
  return pn_next
end

function C:getCodriverByName(codriver_name)
  local codriver = nil
  for _,cd in ipairs(self.codrivers.sorted) do
    if cd.name == codriver_name then
      codriver = cd
      break
    end
  end

  return codriver
end

function C:getCodriverByPk(codriverPk)
  local codriver = nil
  for _,cd in ipairs(self.codrivers.sorted) do
    if cd.pk == codriverPk then
      codriver = cd
      break
    end
  end

  return codriver
end

function C:getCodriverById(codriverId)
  if not codriverId then
    return nil
  end

  return self.codrivers.objects[codriverId]
end

function C:getFirstCodriver()
  return self.codrivers.sorted[1]
end

function C:cacheCompiledPacenotes()
  self:clearCompilationFailures()
  for i,pn in ipairs(self.pacenotes.sorted) do
    pn.visualSerialNo = i
    pn:clearCachedFgData()
    pn:asCompiled()
    if pn:didCompileFail() then
      log('E', logTag, 'cacheCompiledPacenotes: failed to compile pacenote: '..pn.name)
      -- log('E', logTag, 'cacheCompiledPacenotes: aborting remaining pacenote compilations')
      -- break
    end
  end
end

function C:clearCompilationFailures()
  for _,pn in ipairs(self.pacenotes.sorted) do
    pn:clearCompilationFailures()
  end
end

function C:findNClosestPacenotes(pos, n)
  -- Table to store objects and their distances
  local distances = {}

  -- Calculate each object's distance from the input position and store it
  for _,pacenote in ipairs(self.pacenotes.sorted) do
    local distance = pos:distance(pacenote:getActiveFwdAudioTrigger().pos)  -- using the provided distance method
    table.insert(distances, {pacenote = pacenote, distance = distance})
  end

  -- Sort the objects by distance
  table.sort(distances, function(a, b) return a.distance < b.distance end)

  -- Retrieve the N closest objects
  local closest = {}
  n = math.min(#self.pacenotes.sorted, n)
  for i = 1, n do
    if distances[i] then  -- Ensure there's an object to add
      table.insert(closest, distances[i].pacenote)
    end
  end

  return closest
end

function C:getSystemPacenotesForAudioMode()
  local missionPacenotesDirname = nil
  local metadata = nil
  if self:isAudioModeFreeform() then
    -- missionPacenotesDirname = self:missionPacenotesDir(rallyUtil.freeformDir)
    -- metadata = self:loadFreeformPacenoteMetadata()
    missionPacenotesDirname = self:missionPacenotesDir(rallyUtil.systemDir)
    metadata = self:loadSystemPacenoteMetadata()
  elseif self:isAudioModeOnlineStructured() then
    -- missionPacenotesDirname = self:missionPacenotesDir(rallyUtil.structuredDir)
    -- metadata = self:loadOnlineStructuredPacenoteMetadata()
    missionPacenotesDirname = self:missionPacenotesDir(rallyUtil.systemDir)
    metadata = self:loadSystemPacenoteMetadata()
  elseif self:isAudioModeOfflineStructured() then
    missionPacenotesDirname = nil
    metadata = self:loadOfflineStructuredPacenoteMetadata()
  elseif self:isAudioModeCustom() then
    missionPacenotesDirname = nil
    metadata = self:loadCustomPacenoteMetadata()
  else
    log('E', logTag, 'getSystemPacenotesForAudioMode: unknown audio mode')
  end

  local compositor = self:getTextCompositor()
  return compositor:getSystemPacenotes(missionPacenotesDirname), metadata
end

function C:getSystemPacenote(name)
  -- Split the name into prefix and optional id parts
  local prefix, id
  if string.find(name, "_") then
      prefix, id = string.match(name, "^([^_]+)_([^_]*)$")
  else
      prefix = name
      id = nil
  end
  if prefix then
    local id = tonumber(id)
    if id then
      return self:getTextCompositor():getSystemPacenote(prefix, id)
    else
      log('E', logTag, 'getSystemPacenote: could not parse id: '..id)
    end
  else
    log('E', logTag, 'getSystemPacenote: could not parse name: '..name)
  end

  return nil
end

function C:getRandomSystemPacenote(desiredPrefix)
  local compositor = self:getTextCompositor()
  local systemPacenotes, metadata = self:getSystemPacenotesForAudioMode()
  local noteSet = systemPacenotes[desiredPrefix]
  if not noteSet then
    log('E', logTag, 'getRandomSystemPacenote: could not find note set for prefix "'..desiredPrefix..'"')
    return nil
  end

  if not metadata then
    log('E', logTag, 'getRandomSystemPacenote: could not find metadata for prefix "'..desiredPrefix..'"')
    return nil
  end

  return LibCompositor.getRandomWeightedItem(noteSet), metadata
end

function C:setAdjacentNotes(pacenote_id)
  local pacenotesSorted = self.pacenotes.sorted
  for i, note in ipairs(pacenotesSorted) do
    if pacenote_id == note.id then
      local prevNote = pacenotesSorted[i-1]
      local nextNote = pacenotesSorted[i+1]
      note:setAdjacentNotes(prevNote, nextNote)
    else
      note:clearAdjacentNotes()
    end
  end
end

function C:setAllAdjacentNotes()
  local pacenotesSorted = self.pacenotes.sorted
  for i, note in ipairs(pacenotesSorted) do
    note:clearAdjacentNotes()
  end

  for i, note in ipairs(pacenotesSorted) do
    local prevNote = pacenotesSorted[i-1]
    local nextNote = pacenotesSorted[i+1]
    note:setAdjacentNotes(prevNote, nextNote)
  end
end

function C:markAllTodo()
  for _,pn in ipairs(self.pacenotes.sorted) do
    pn:markTodo()
  end
end

function C:clearAllTodo()
  for _,pn in ipairs(self.pacenotes.sorted) do
    pn:clearTodo()
  end
end

function C:markRestTodo(pacenote)
  if not pacenote then return end

  local hitPacenote = false
  for _,pn in ipairs(self.pacenotes.sorted) do
    if pn.id == pacenote.id then
      hitPacenote = true
    end
    if hitPacenote then
      pn:markTodo()
    end
  end
end

function C:getAudioMode()
  return self.audioMode
end

function C:setAudioMode(mode)
  self.audioMode = mode
end

function C:isAudioModeOnlineStructured()
  return self:getAudioMode() == RallyEnums.pacenoteAudioMode.structuredOnline
end

function C:isAudioModeOfflineStructured()
  return self:getAudioMode() == RallyEnums.pacenoteAudioMode.structuredOffline
end

function C:isAudioModeFreeform()
  return self:getAudioMode() == RallyEnums.pacenoteAudioMode.freeform
end

function C:useStructured()
  return self:getAudioMode() == RallyEnums.pacenoteAudioMode.structuredOffline or
         self:getAudioMode() == RallyEnums.pacenoteAudioMode.structuredOnline
end

function C:isV2()
  return not self.version or self.version == "2" or self.version == 2
end

function C:isV3()
  return self.version == "3" or self.version == 3
end

function C:missionPacenotesDir(audioModeDir)
  if not audioModeDir then
    log('E', logTag, 'missionPacenotesDir: audioModeDir is nil')
    return nil
  end

  local missionDir = self:getMissionDir()
  local notebookBasename = rallyUtil.normalizeName(self:basenameNoExt()) or 'none'
  local codriver = self:selectedCodriver()
  local codriverName = codriver.name
  -- local codriverLang = codriver.language
  -- local codriverVoice = codriver.voice
  local codriverPk = codriver.pk
  -- local codriverDir = rallyUtil.normalizeName(codriverName..'_'..codriverLang..'_'..codriverVoice)
  local codriverDir = rallyUtil.normalizeName(codriverName..'_'..codriverPk)
  -- local dirname = table.concat({missionDir, rallyUtil.notebooksPath, rallyUtil.generatedPacenotesDir, notebookBasename, codriverDir}, '/')

  local dirname = table.concat({
    missionDir,
    rallyUtil.notebooksPath,
    rallyUtil.generatedPacenotesDir,
    notebookBasename,
    codriverDir,
    audioModeDir
  }, '/')
  return dirname
end

function C:customPacenotesDir()
  local missionDir = self:getMissionDir()
  local notebookBasename = rallyUtil.normalizeName(self:basenameNoExt()) or 'none'
  local codriver = self:selectedCodriver()
  local codriverName = codriver.name
  local codriverPk = codriver.pk
  local codriverDir = rallyUtil.normalizeName(codriverName..'_'..codriverPk)


  local dirs = {
    missionDir,
    rallyUtil.notebooksPath,
    rallyUtil.customDir,
    notebookBasename,
    codriverDir
  }

  local absPath = '/'
  for _,dir in ipairs(dirs) do
    absPath = absPath..dir..'/'
    if not FS:directoryExists(absPath) then
      FS:directoryCreate(absPath)
    end
  end

  return table.concat(dirs, '/')
end

function C:missionPacenoteAudioFile(audioModeDir, pacenoteTextOut)
  local pacenotesDir = self:missionPacenotesDir(audioModeDir)
  local pacenoteHash = rallyUtil.pacenoteHashSha1(pacenoteTextOut)
  return pacenotesDir..'/'..rallyUtil.makePacenoteAudioFilename(pacenoteHash)
end

function C:_missionPacenoteMetadataFile(audioModeDir)
  local pacenotesDir = self:missionPacenotesDir(audioModeDir)
  return pacenotesDir..'/'..rallyUtil.pacenotesMetadataBasename
end

local function readMetadataFile(fname)
  local json = jsonReadFile(fname)
  if not json then
    log('E', logTag, 'couldnt find metadata file: '..fname)
    return nil
  end
  return json
end

function C:loadCustomPacenoteMetadata()
  if self.pacenoteMetadataCustom then
    return self.pacenoteMetadataCustom
  end
  local metadataFnameCustom = self:customPacenotesDir()..'/'..rallyUtil.pacenotesMetadataBasename
  self.pacenoteMetadataCustom = readMetadataFile(metadataFnameCustom) or {}
  return self.pacenoteMetadataCustom
end

function C:loadOfflineStructuredPacenoteMetadata()
  if self.pacenoteMetadataOfflineStructured then
    return self.pacenoteMetadataOfflineStructured
  end
  local metadataFnameOfflineStructured = rallyUtil.getCompositorMetadataFile(settings.getValue('rallyTextCompositorVoice'))
  self.pacenoteMetadataOfflineStructured = readMetadataFile(metadataFnameOfflineStructured)
  return self.pacenoteMetadataOfflineStructured
end

function C:loadOnlineStructuredPacenoteMetadata()
  if self.pacenoteMetadataOnlineStructured then
    return self.pacenoteMetadataOnlineStructured
  end
  local metadataFnameOnlineStructured = self:_missionPacenoteMetadataFile(rallyUtil.structuredDir)
  self.pacenoteMetadataOnlineStructured = readMetadataFile(metadataFnameOnlineStructured)
  return self.pacenoteMetadataOnlineStructured
end

function C:loadFreeformPacenoteMetadata()
  if self.pacenoteMetadataFreeform then
    return self.pacenoteMetadataFreeform
  end
  local metadataFnameFreeform = self:_missionPacenoteMetadataFile(rallyUtil.freeformDir)
  self.pacenoteMetadataFreeform = readMetadataFile(metadataFnameFreeform)
  return self.pacenoteMetadataFreeform
end

function C:loadSystemPacenoteMetadata()
  if self.pacenoteMetadataSystem then
    return self.pacenoteMetadataSystem
  end
  local metadataFnameSystem = self:_missionPacenoteMetadataFile(rallyUtil.systemDir)
  self.pacenoteMetadataSystem = readMetadataFile(metadataFnameSystem)
  return self.pacenoteMetadataSystem
end

function C:clearPacenoteAudioModeOverrides()
  for _, pacenote in ipairs(self.pacenotes.sorted) do
    pacenote:setAudioMode(RallyEnums.pacenoteAudioMode.auto)
  end
end

function C:useCodriver(codriver)
  self:missionSettings():setCodriverId(codriver.pk)
  self:clearCompilationFailures()
end

function C:deletePacenoteLanguage(lang)
  for _, pacenote in ipairs(self.pacenotes.sorted) do
    pacenote:deleteLanguage(lang)
  end
end

return function(...)
  local o = {}

  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end
