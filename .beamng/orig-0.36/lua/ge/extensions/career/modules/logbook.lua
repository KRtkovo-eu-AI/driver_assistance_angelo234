-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.dependencies = {'career_career'}

local fileName = "logbook.json"
local logbook = {}
local idCounter =  0
local function sortByTimeAndId(a,b)
  if a.time == b.time and a.entryId and b.entryId then
    if type(a.entryId) == "number" and type(b.entryId) == "number" then
      return a.entryId > b.entryId
    else
      return tostring(a.entryId) > tostring(b.entryId)
    end
  else
    return a.time > b.time
  end
end

local function getLogbook()
  local ret = { }
  for _, e in ipairs(logbook) do
    table.insert(ret, e)
  end

  extensions.hook("onLogbookGetEntries", ret)

  table.sort(ret, sortByTimeAndId)
  return ret
end

local function getPopups()
  local ret = {}
  for _, e in ipairs(logbook) do
    if not e.read and e.isPopup then
      table.insert(ret, e)
    end
  end
  return ret
end
M.getPopups = getPopups

local function getLogbookMostRecentUnread(limit)
  limit = limit or 3
  local ret = {}
  local byRead = {}
  byRead[true] = {}
  byRead[false] = {}
  for _, entry in ipairs(getLogbook()) do
    if not entry.hideInRecent then
      table.insert(byRead[entry.read or false], entry)
    end
  end
  for i = 1, limit do
    if next(byRead[false]) then
      table.insert(ret, byRead[false][1])
      table.remove(byRead[false],1)
    elseif next(byRead[true]) then
      table.insert(ret, byRead[true][1])
      table.remove(byRead[true],1)
    else
      return ret
    end
  end
  return ret
end
M.getLogbookMostRecentUnread = getLogbookMostRecentUnread

local function setLogbookEntryRead(entryId, read)
  if entryId == -1 then return end
  --print("Setting Logbook entry to read" .. entryId .. " ->  " ..dumps(read))
  for _, e in ipairs(logbook) do
    if e.entryId == entryId then
      e.isNew = not read
      return
    end
  end
end

-- this function is used by other functions to create the actual entry.
local function addNewLogbookEntry(entry, skipSave)
  entry = entry or {}
  entry.time = os.time()
  entry.entryId = idCounter
  entry.isNew = true
  idCounter = idCounter + 1
  if not skipSave then
    table.insert(logbook,1, entry)
    --log("I","","New Unlock Event: " ..dumps(entry))
    if entry.showMessage and not career_modules_linearTutorial.isLinearTutorialActive() then
      local helper = {}
      helper.ttl = 15
      helper.msg = {txt="ui.career.logbook.newEntry", context={title=entry.title}}
      helper.category = entry.entryId
      helper.icon = "library_books"
      guihooks.trigger('Message',helper)
    end
  end
  return entry
end

-- called whenever a new mission is unlocked.
local function missionUnlocked(id)
  local mission = gameplay_missions_missions.getMissionById(id)
  if not id then return end
  addNewLogbookEntry({
    type = "progress",
    cardTypeLabel = "ui.career.poiCard.missionUnlocked",
    missionId = id,
    title = mission.name,
    text = {txt = 'ui.career.logbook.missionUnlockedNamed', context = {missionName = mission.name}},
    cover = mission.previewFile,
  })
end

-- called whenever a facility is unlocked
local function deliveryFacilityUnlocked(id)
  local fac = career_modules_delivery_generator.getFacilityById(id)
  if not id or not fac then return end
  addNewLogbookEntry({
    type = "progress",
    cardTypeLabel = "Delivery Facility Unlocked",
    --missionId = id, -- ID needs to be for poi not mission...
    title = fac.name,
    text = {txt = 'ui.career.logbook.deliveryFacilityUnlockedNamed', context = {facilityName = fac.name}},
    cover = fac.preview,
  })
end

-- called when a spawnpoint is discovered.
local function spawnPointUnlocked(spawnPoint)
  addNewLogbookEntry({
    type = "progress",
    cardTypeLabel = "ui.career.poiCard.spawnPointUnlocked",
    name = spawnPoint.translationId,
    cover = spawnPoint.previews[1],
    title = {txt = 'ui.career.logbook.spawnPointUnlockedNamed', context = {spawnPointName = spawnPoint.translationId}},
    text = spawnPoint.logbookEntry,
    showMessage = true,
  })
end

local playedLogbookSoundThisFrame = false
-- this is the "normal" logbook entry. a useful piece of info/tutorial/helps. also plays sound.
local function genericInfoUnlocked(title, text, cover, ratio, flavour, type)
  -- TODO: add name, description, cover etc
  local entry = addNewLogbookEntry({
    type = type or "info",
    cardTypeLabel = "ui.career.poiCard.generic",
    title = title,
    text = text,
    cover = cover,
    ratio = ratio or "16x9",
    showMessage = true,
  })

  -- guard so we don't play the same sound multiple times per frame
  if not career_modules_linearTutorial.isLinearTutorialActive() and not playedLogbookSoundThisFrame then
    Engine.Audio.playOnce('AudioGui', 'event:UI_Checkpoint')
    playedLogbookSoundThisFrame = true
  end
  return entry
end


-- this is a generic entry that can be anything, but nothing special.
local function genericLogbookEntry(title, text, cover, coverText)
  return addNewLogbookEntry({
    type = "info",
    cardTypeLabel = "ui.career.poiCard.generic",
    title = title,
    text = text,
    cover = cover,
    coverText = coverText,
    showMessage = true
  })
end


local logbookEntries = {
  'welcome',
  'logbook',
  'walkingMode',
  'cameras',
  'driving',
  'crashRecover',
  'bigmap',
  'refueling',
  'missions',
  'dealership',
  'testdrive',
  'computer',
  'partShopping',
  'tuning',
  'milestones',
  'delivery',
}

-- this creates a new logbook entry from an ID. it reads content and header from the folder in ui/modules/careerLogbook/pages/ID/ . Title is ui.introPopup.ID.title
M.logbookEntry = function(id)
  M.genericInfoUnlocked("ui.introPopup."..id..".title", readFile("/ui/modules/careerLogbook/pages/"..id.."/content.html"):gsub("\r\n",""), "/ui/modules/careerLogbook/pages/"..id.."/header.jpg")
end


local function loadDataFromFile()
  local saveSlot, savePath = career_saveSystem.getCurrentSaveSlot()
  if not saveSlot then return end
  local data = {}--(savePath and jsonReadFile(savePath .. "/career/"..fileName)) or {}
  logbook = data.logbook or {}
  idCounter = #logbook
end

local function onCareerActive(active)
  local clear = false
  if not clear then
    loadDataFromFile()
  else
    logbook = {}
  end
  if not next(logbook) then
    for _, key in ipairs(arrayReverse(logbookEntries)) do
      M.logbookEntry(key)
    end
  end
end

-- this should only be loaded when the career is active
local function onSaveCurrentSaveSlot(currentSavePath)
  career_saveSystem.jsonWriteFileSafe(currentSavePath .. "/career/"..fileName,
    {
      logbook = logbook
    }, true)
end

local function onUpdate()
  playedLogbookSoundThisFrame = false
end

M.onCareerActive = onCareerActive
M.onSaveCurrentSaveSlot = onSaveCurrentSaveSlot

M.getLogbook = getLogbook
M.setLogbookEntryRead = setLogbookEntryRead
M.addNewLogbookEntry = addNewLogbookEntry

M.missionUnlocked = missionUnlocked
M.deliveryFacilityUnlocked = deliveryFacilityUnlocked
M.spawnPointUnlocked = spawnPointUnlocked
M.genericInfoUnlocked = genericInfoUnlocked
M.genericLogbookEntry = genericLogbookEntry

M.onUpdate = onUpdate
return M