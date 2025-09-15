-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- usage:
--
-- local rallyUtil = require('/lua/ge/extensions/editor/rallyEditor/util')
--

local logTag = ''

local M = {}

local autofill_blocker = '#'
local autodist_internal_level1 = '<none>'
local unknown_transcript_str = '[unknown]'
local missionRallyDir = 'rally'
local notebooksDir = 'notebooks'
local recceDir = 'recce'
local notebooksPath = missionRallyDir..'/'..notebooksDir
local reccePath = missionRallyDir..'/'..recceDir
local recceRecordSubdir = 'primary'
local generatedPacenotesDir = 'gen'
local freeformDir = 'freeform'
local structuredDir = 'structured'
local systemDir = 'system'
local customDir = 'custom'

local rallySettingsRoot = '/settings/'..missionRallyDir
local cornerAnglesFname = '/lua/ge/extensions/gameplay/rally/corner_angles.json'
local pacenotesMetadataBasename = 'metadata.json'

local transcriptsExt = 'transcripts.json'
local missionSettingsFname = 'mission.settings.json'
local defaultNotebookFname = 'primary.notebook.json'
local defaultCodriverName = 'Sophia'
local default_codriver_voice = 'british_female'
local default_codriver_language = 'english'
local default_waypoint_intersect_radius = 40

local validPunctuation = {"?", ".", "!"}

local var_db = '{db}'
local var_da = '{da}'

-- html code: #00ffdebf
local rally_flowgraph_color = ui_imgui.ImVec4(0, 1, 0.87, 0.75) -- rgba cyan

local function pacenoteHashSha1(s)
  local str = hashStringSHA1(s)
  return str:sub(1, 16)
end

-- returns seconds since epoch
local function getTime()
  -- os.clockhp appears to be beamng-specific.
  return os.clockhp()
end

-- function printFields(obj)
--   for k, v in pairs(obj) do
--     -- if type(v) == "function" then
--       print(k)
--     -- end
--   end
-- end

local function normalizeName(name)
  if not name then return nil end

  -- Replace everything but letters and numbers with '_'
  name = string.gsub(name, "[^a-zA-Z0-9]", "_")

  -- Replace multiple consecutive '_' with a single '_'
  name = string.gsub(name, "(_+)", "_")

  return name
end

-- assumes that the file exists.
local function playPacenote(audioObj)
  local opts = { volume=audioObj.volume }
  audioObj.time = getTime()

  -- local sfxSource = scenetree.findObjectById(res.sourceId)
  -- log('D', logTag, dumps(sfxSource))
  -- printFields(sfxSource)

  Engine.Audio.intercomPlayPacenote({ filename=audioObj.pacenoteFname })

  if audioObj.audioLen then
    -- set these fields, so that the next time flow triggers audio playing, the timeout will be respected.
    audioObj.timeout = audioObj.time + audioObj.audioLen + audioObj.breathSuffixTime
  end
  -- log('D', logTag, 'playPacenote '..dumps(audioObj))
end

local function buildAudioObjPacenote(pacenoteFname)
  local audioObj = {
    audioType = 'pacenote',
    pacenoteFname = pacenoteFname,
    volume = 1,
    created_at = getTime(),
    time = nil,
    audioLen = nil,
    timeout = nil,
    sourceId = nil,
    breathSuffixTime = 0.3, -- add time to represent the co-driver taking a breath after reading a pacenote.
  }

  return audioObj
end

local function buildAudioObjPause(pauseSecs)
  local audioObj = {
    audioType = 'pause',
    created_at = getTime(),
    time = nil,
    audioLen = nil,
    timeout = nil,
    pauseTime = pauseSecs,
  }

  return audioObj
end

local function hasPunctuation(last_char)
  for _,char in ipairs(validPunctuation) do
    if last_char == char then
      return true
    end
  end

  return false
end

local function detectMissionManagerMissionId()
  if gameplay_missions_missionManager then
    return gameplay_missions_missionManager.getForegroundMissionId()
  else
    return nil
  end
end

local function detectMissionEditorMissionId()
  if editor_missionEditor then
    local selectedMission = editor_missionEditor.getSelectedMissionId()
    if selectedMission then
      return selectedMission.id
    else
      return nil
    end
  else
    return nil
  end
end

local function detectMissionIdHelper()
  local missionId = nil
  local missionDir = nil

  -- first try the mission manager.
  local theMissionId = detectMissionManagerMissionId()
  if theMissionId then
    log('D', logTag, 'missionId "'.. theMissionId ..'"detected from missionManager')
  else
    log('D', logTag, 'no mission detected from missionManager')
  end

  -- then try the mission editor
  if not theMissionId then
    theMissionId = detectMissionEditorMissionId()
    if theMissionId then
      log('D', logTag, 'missionId "'.. theMissionId ..'"detected from missionEditor')
    else
      log('D', logTag, 'no mission detected from editor')
    end
  end

  if not theMissionId then
    log('E', logTag, 'couldnt detect missionId')
    return nil, nil, 'missionId could not be detected'
  end

  missionId = theMissionId
  missionDir = '/gameplay/missions/'..theMissionId

  return missionId, missionDir, nil
end

local function getNotebookFullPath(missionDir, basename)
  local notebookFname = missionDir..'/'..notebooksPath..'/'..basename
  return notebookFname
end

local function createNotebook(fname)
  local newPath = require('/lua/ge/extensions/gameplay/rally/notebook/path')()
  newPath:setFname(fname)
  if not newPath:save() then
    log('E', logTag, 'error saving new notebook')
    return nil
  end

  return newPath
end

local function loadNotebook(notebookFname)
  log('I', logTag, 'loading notebook: ' .. notebookFname)

  if not notebookFname then
    log('E', logTag, 'unable to load notebook: notebookFname is nil')
    return nil
  end

  if not FS:fileExists(notebookFname) then
    log('E', logTag, 'unable to load notebook: notebook file not found: '..notebookFname)
    return nil
  end

  local json = jsonReadFile(notebookFname)
  if not json then
    log('E', logTag, 'unable to load notebook json: '..notebookFname)
    return nil
  end

  local notebook = require('/lua/ge/extensions/gameplay/rally/notebook/path')()
  notebook:setFname(notebookFname)
  notebook:onDeserialized(json)
  notebook:setAllAdjacentNotes()

  return notebook
end

local function loadNotebookForMissionDir(missionDir, basename)
  local notebookFname = getNotebookFullPath(missionDir, basename)
  return loadNotebook(notebookFname)
end

local function loadRace(missionDir)
  local raceFname = missionDir..'/race.race.json'
  log('I', logTag, 'loading race: ' .. raceFname)

  if not FS:fileExists(raceFname) then
    return nil, "race file not found: "..raceFname
  end

  local racePath = require('/lua/ge/extensions/gameplay/race/path')()
  racePath:onDeserialized(jsonReadFile(raceFname))

  return racePath, nil
end


local function missionRecceDir(missionDir)
  return missionDir..'/'..reccePath
end

local function missionRecceRecordDir(missionDir)
  local rv = missionRecceDir(missionDir)..'/'..recceRecordSubdir
  return rv
end

local function missionReccePath(missionDir, basename)
  local subdir = missionRecceRecordDir(missionDir)
  local rv = subdir..'/'..basename
  return rv
end

local function drivelineFile(missionDir)
  return missionReccePath(missionDir, 'driveline.json')
end

local function cutsFile(missionDir)
  return missionReccePath(missionDir, 'cuts.json')
end

local function transcriptsFile(missionDir)
  return missionReccePath(missionDir, 'transcripts.json')
end

-- args are both vec3's representing a position.
local function calculateForwardNormal(snap_pos, next_pos)
  local flip = false
  local dx = next_pos.x - snap_pos.x
  local dy = next_pos.y - snap_pos.y
  local dz = next_pos.z - snap_pos.z

  local magnitude = math.sqrt(dx*dx + dy*dy + dz*dz)
  if magnitude == 0 then
    error("The two positions must not be identical.")
  end

  local normal = vec3(dx / magnitude, dy / magnitude, dz / magnitude)

  if flip then
    normal = -normal
  end

  return normal
end

local function loadCornerAnglesFile()
  local filename = cornerAnglesFname
  local json = jsonReadFile(filename)
  if json then
    return json, nil
  else
    local err = 'unable to find corner_angles file: ' .. tostring(filename)
    log('E', 'rally', err)
    return nil, err
  end
end

local function determineCornerCall(angles, steering)
  local absSteeringVal = math.abs(steering)
  for i,angle in ipairs(angles) do
    if absSteeringVal >= angle.fromAngleDegrees and absSteeringVal < angle.toAngleDegrees then
      local direction = steering >= 0 and "L" or "R"
      local cornerCallWithDirection = angle.cornerCall..direction
      if angle.cornerCall == '_deadzone' then
        cornerCallWithDirection = 'c'
      end

    local range = angle.toAngleDegrees - angle.fromAngleDegrees
    local pct = (absSteeringVal - angle.fromAngleDegrees) / range
      return angle, string.upper(cornerCallWithDirection), pct
    end
  end
end

local function trimString(txt)
  if not txt then return txt end
  local trimmed = string.gsub(txt, "^%s*(.-)%s*$", "%1")
  return trimmed
end

local function stripBasename(thepath)
  if not thepath then return nil end

  if thepath:sub(-1) == "/" then
    thepath = thepath:sub(1, -2)
  end
  local dirname, fn, e = path.split(thepath)

  if dirname:sub(-1) == "/" then
    dirname = dirname:sub(1, -2)
  end
  return dirname
end

local function matchSearchPattern(searchPattern, stringToMatch)
  -- Escape special characters in Lua patterns except '*'
  searchPattern = searchPattern:gsub("([%^%$%(%)%%%.%[%]%+%-%?])", "%%%1")
  -- Replace '*' with Lua's '.*' to act as a wildcard
  searchPattern = searchPattern:gsub("%*", ".*")

  return stringToMatch:match(searchPattern) ~= nil
end

local function breathSuffixTime(min, max)
  min = min or 0.1
  max = max or 0.3
  return min + math.random() * (max - min)
end

local function useNote(text)
  return text and
    text ~= '' and
    text ~= autofill_blocker and
    text ~= autodist_internal_level1
end

local function customRound(dist, round_to)
  return math.floor(dist / round_to + 0.5) * round_to
end

local function makePacenoteAudioFilename(pacenoteHash)
  return 'pacenote_'..pacenoteHash..'.ogg'
end

local function getCompositorFile(compositorVoice, basename)
  local path = string.format('/lua/ge/extensions/gameplay/rally/compositors/%s/%s', compositorVoice, basename)
  return path
end

local function getCompositorPacenoteFile(compositorVoice, pacenoteHash)
  return getCompositorFile(compositorVoice, makePacenoteAudioFilename(pacenoteHash))
end

local function getCompositorMetadataFile(compositorVoice)
  return getCompositorFile(compositorVoice, pacenotesMetadataBasename)
end

local function getMissionSettingsFile(missionDir)
  -- log('D', logTag, 'getMissionSettingsFile missionDir: '..missionDir)
  -- log('D', logTag, 'getMissionSettingsFile missionRallyDir: '..missionRallyDir)
  -- log('D', logTag, 'getMissionSettingsFile missionSettingsFname: '..missionSettingsFname)
  local settingsFname = missionDir..'/'..missionRallyDir..'/'..missionSettingsFname
  return settingsFname
end

local function getMissionName(missionId)
  -- log('D', 'rallyUtil', 'getMissionName: missionId = ' .. tostring(missionId))
  local mission = gameplay_missions_missions.getMissionById(missionId)
  if not mission then
    -- log('D', 'rallyUtil', 'getMissionName: mission not found, returning missionId')
    return missionId
  end
  -- log('D', 'rallyUtil', 'getMissionName: found mission, name = ' .. tostring(mission.name))
  return mission.name
end

local function translatedMissionName(missionName)
  -- log('D', 'rallyUtil', 'translatedMissionName: missionName = ' .. tostring(missionName))
  if not missionName then
    -- log('D', 'rallyUtil', 'translatedMissionName: missionName is nil, returning nil')
    return missionName
  end
  local translated = translateLanguage(missionName, missionName, true)
  -- log('D', 'rallyUtil', 'translatedMissionName: translated = ' .. tostring(translated))
  return translated
end

local function translatedMissionNameFromId(missionId)
  -- log('D', 'rallyUtil', 'translatedMissionNameFromId: missionId = ' .. tostring(missionId))
  local missionName = getMissionName(missionId)
  if not missionName then
    -- log('D', 'rallyUtil', 'translatedMissionNameFromId: missionName not found, returning missionId')
    return missionId
  end
  local translated = translatedMissionName(missionName)
  -- log('D', 'rallyUtil', 'translatedMissionNameFromId: translated = ' .. tostring(translated))
  return translated
end

local function randomId()
  local randomStr = tostring(math.random(1, 1000000))
  local hash = hashStringSHA1(randomStr)
  return string.sub(hash, 1, 8)
end

local function getAppropriateTextColor(clr)
  return (clr[1] < 0.1 and clr[2] < 0.5 and clr[3] > 0.7) and ColorF(1, 1, 1, 1) or ColorF(0, 0, 0, 1)
end

-- vars
M.rallySettingsRoot = rallySettingsRoot
M.missionRallyDir = missionRallyDir
M.autodist_internal_level1 = autodist_internal_level1
M.autofill_blocker = autofill_blocker
M.default_codriver_language = default_codriver_language
M.default_waypoint_intersect_radius = default_waypoint_intersect_radius
M.defaultCodriverName = defaultCodriverName
M.defaultNotebookFname = defaultNotebookFname
M.default_codriver_voice = default_codriver_voice
M.missionSettingsFname = missionSettingsFname
M.notebooksDir = notebooksDir
M.recceDir = recceDir
M.recceRecordSubdir = recceRecordSubdir
M.notebooksPath = notebooksPath
M.pacenotesMetadataBasename = pacenotesMetadataBasename
M.transcriptsExt = transcriptsExt
M.unknown_transcript_str = unknown_transcript_str
M.validPunctuation = validPunctuation
M.var_db = var_db
M.var_da = var_da

-- funcs
M.buildAudioObjPacenote = buildAudioObjPacenote
M.buildAudioObjPause = buildAudioObjPause
M.calculateForwardNormal = calculateForwardNormal
M.detectMissionEditorMissionId = detectMissionEditorMissionId
M.detectMissionIdHelper = detectMissionIdHelper
M.detectMissionManagerMissionId = detectMissionManagerMissionId
M.determineCornerCall = determineCornerCall
M.loadNotebook = loadNotebook
M.loadNotebookForMissionDir = loadNotebookForMissionDir
M.createNotebook = createNotebook
M.loadRace = loadRace
M.getTime = getTime
M.hasPunctuation = hasPunctuation
M.loadCornerAnglesFile = loadCornerAnglesFile
M.matchSearchPattern = matchSearchPattern
M.missionRecceRecordDir = missionRecceRecordDir
M.missionReccePath = missionReccePath
M.generatedPacenotesDir = generatedPacenotesDir
M.freeformDir = freeformDir
M.structuredDir = structuredDir
M.systemDir = systemDir
M.customDir = customDir

M.getAppropriateTextColor = getAppropriateTextColor

M.drivelineFile = drivelineFile
M.cutsFile = cutsFile
M.transcriptsFile = transcriptsFile

M.pacenoteHashSha1 = pacenoteHashSha1
M.playPacenote = playPacenote
M.normalizeName = normalizeName
M.trimString = trimString
M.stripBasename = stripBasename
M.breathSuffixTime = breathSuffixTime
M.useNote = useNote
M.customRound = customRound

M.getCompositorFile = getCompositorFile
M.getCompositorPacenoteFile = getCompositorPacenoteFile
M.getCompositorMetadataFile = getCompositorMetadataFile

M.makePacenoteAudioFilename = makePacenoteAudioFilename
M.getMissionSettingsFile = getMissionSettingsFile
M.getNotebookFullPath = getNotebookFullPath

M.getMissionName = getMissionName
M.translatedMissionName = translatedMissionName
M.translatedMissionNameFromId = translatedMissionNameFromId
M.rally_flowgraph_color = rally_flowgraph_color
M.randomId = randomId

return M