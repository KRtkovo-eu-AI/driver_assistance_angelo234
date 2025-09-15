-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.dependencies = {"editor_main"}

local im = ui_imgui
local search = require('/lua/ge/extensions/editor/util/searchUtil')()

local toolWindowName = "sfxPreviewer"
local toolWindowTitle = "SFX Previewer"

local eventSearchInput = im.ArrayChar(128, "")

local eventsTree = {} -- For folder exploring
local eventsList = {} -- For searching

local currDir = {}

local sfxIDPlaying = nil
local searchResults = nil

local function stopCurrentSFX()
  if sfxIDPlaying then
    local oldSFX = scenetree.findObjectById(sfxIDPlaying)
    if oldSFX then
      oldSFX:stop(-1)
      oldSFX:delete()
    end
  end
end

local function getPathSplit(path, pattern)
  local pathSplit = {}

  for dir in string.gmatch(path, pattern) do
    table.insert(pathSplit, dir)
  end

  return pathSplit
end

local function recTableSort(inTbl, outTbl)
  -- Sort by name and folders come first
  local keysSorted = tableKeysSorted(inTbl)
  table.sort(keysSorted,
    function(a,b)
      local aType, bType = inTbl[a].dataType, inTbl[b].dataType
      if aType == bType then
        return a < b
      else
        return aType == "dir" and bType == "event"
      end
    end
  )

  for k, name in ipairs(keysSorted) do
    local tbl = inTbl[name]
    local data = tbl.data
    local dataType = tbl.dataType

    outTbl[k] = tbl

    if dataType == "dir" then
      recTableSort(data, outTbl[k].data)
    end
  end
end

local function updateEventsList()
  table.clear(eventsTree)

  local newEvents = {}
  local numEvents = Sim.getSFXTrackSet():getCount()

  for i = 0, numEvents - 1 do
    local eventObj = Sim.upcast(Sim.getSFXTrackSet():getObject(i))
    local eventDesc = eventObj:getField("description", "")

    local eventName = eventObj:getName()
    if not eventName then goto continue end
    local pathSplit = nil
    if not string.startswith(eventName, "event:>") then
      if not eventObj.getSoundFilename then
        goto continue
      end
      eventName = eventObj:getSoundFilename()
      pathSplit = getPathSplit(eventName, "([^/]+)")
    else
      pathSplit = getPathSplit(eventName, "([^>]+)")
    end

    local numDirs = #pathSplit
    local currPath = newEvents
    local currPathStr = ""

    for j, dir in ipairs(pathSplit or {}) do
      if #currPathStr == 0 then
        currPathStr = dir
      else
        currPathStr = currPathStr .. ">" .. dir
      end

      if j ~= numDirs then
        if not currPath[dir] then
          currPath[dir] = {dataType = "dir", name = dir, path = currPathStr, data = {}}
        end
        currPath = currPath[dir].data
      else
        currPath[dir] = {dataType = "event", name = dir, path = currPathStr, id = i, eventName = eventName, eventDesc = eventDesc}
      end
    end

    eventsList[eventName] = {path = currPathStr, id = i, eventName = eventName, eventDesc = eventDesc}
    ::continue::
  end

  recTableSort(newEvents, eventsTree, "")
end

local function renderSoundEventGui(event)
  if im.BeginPopup("rightClickPopupMenu" .. event.id) then
    if im.Selectable1("Copy to clipboard") then
      im.SetClipboardText(event.eventName)
    end
    im.EndPopup()
  end

  editor.uiIconImage(editor.icons.music_note, im.ImVec2(24,24), im.ImVec4(1,1,1,1))
  im.SameLine()

  -- Left clicking plays audio clip
  -- Right clicking opens popup menu
  im.Selectable1(event.eventName, nil, im.SelectableFlags_SpanAllColumns)
  local selectableHovered = im.IsItemHovered()

  if selectableHovered then
    if im.IsMouseReleased(0) then
      -- Stop previous sound
      stopCurrentSFX()

      local sourcePos
      local vehicle = getPlayerVehicle(0)
      if vehicle then
        sourcePos = vehicle:getPosition()
      else
        sourcePos = vec3(0,0,0)
      end

      local newSFX = Engine.Audio.playOnce('AudioGui', event.eventName, {position = sourcePos})
      if newSFX then
        sfxIDPlaying = newSFX.sourceId
      else
        log('E','',"Engine.Audio.playOnce('AudioGui','" .. event.eventName .. "') returns nil!")
      end
    elseif im.IsMouseReleased(1) then
      im.OpenPopup("rightClickPopupMenu" .. event.id)
    end
  end
end

local function renderCurrentDirGui(eventsDir, currDirLen, stopButtonHeight)
  if currDirLen > 0 then
    -- Go back to "root" folder
    editor.uiIconImage(editor.icons.folder, im.ImVec2(24,24), im.ImVec4(1,1,1,1))
    im.SameLine()
    if im.Selectable1("..", nil, im.SelectableFlags_SpanAllColumns) then
      table.remove(currDir, currDirLen)
    end
  end

  im.BeginChild1("##ScrollableGUI", im.ImVec2(0, im.GetContentRegionAvail().y - stopButtonHeight))
  -- Render folders and eventsTree
  for k, v in ipairs(eventsDir) do
    if v.dataType == "dir" then
      editor.uiIconImage(editor.icons.folder, im.ImVec2(24,24), im.ImVec4(1,1,1,1))
      im.SameLine()
      if im.Selectable1(v.name, nil, im.SelectableFlags_SpanAllColumns) then
        table.insert(currDir, k)
      end

    elseif v.dataType == "event" then
      renderSoundEventGui(v)
    else
      log("E", "audioEventsList.renderCurrentDirGui", "This shouldn't call!")
    end
  end
  im.EndChild()
end

local function renderSearchResultsGui(stopButtonHeight)
  im.BeginChild1("##ScrollableGUI", im.ImVec2(0, im.GetContentRegionAvail().y - stopButtonHeight))
  if searchResults then
    for _, res in ipairs(searchResults) do
      renderSoundEventGui(eventsList[res.name])
    end
  end

  im.EndChild()
end

local function updateSearchResults()
  search:startSearch(ffi.string(eventSearchInput))

  for _, event in pairs(eventsList) do
    local entry = {name = event.eventName, score = 1}
    search:queryElement(entry)
  end

  searchResults = search:finishSearch()
end

local function onUpdate(dtReal, dtSim, dtRaw)
  if not editor or not editor.beginWindow then return end
  if not editor.isWindowRegistered(toolWindowName) then return end
  if editor.beginWindow(toolWindowName, toolWindowTitle) then
    local searchInputSize = im.GetContentRegionAvailWidth() * 0.4
    im.PushItemWidth(searchInputSize)
    if im.InputText("Search", eventSearchInput, 128) then
      updateSearchResults()
    end
    im.PopItemWidth()

    local stopButtonSize = im.CalcTextSize("Stop Current Sound")

    -- If no search text inputted, render current directory
    -- otherwise render search results
    if #ffi.string(eventSearchInput) == 0 then
      -- Get current directory
      local eventsDir = eventsTree
      local currDirLen = #currDir

      local currPathStr = ""

      for i = 1, currDirLen do
        currPathStr = eventsDir[currDir[i]].path
        eventsDir = eventsDir[currDir[i]].data
      end

      im.Text('Path: "' .. currPathStr .. '"')
      renderCurrentDirGui(eventsDir, currDirLen, stopButtonSize.y + 10)
    else
      im.Text('Path: All Events')
      renderSearchResultsGui(stopButtonSize.y + 10)
    end

    im.SetCursorPosX(im.GetContentRegionAvail().x / 2 - stopButtonSize.x / 2)
    if im.Button("Stop Current Sound") then
      stopCurrentSFX()
    end
  end

  editor.endWindow()
end

local function onEditorToolWindowShow (windowName)
  if windowName == toolWindowName then
    updateEventsList()
  end
end

local function onEditorToolWindowHide(windowName)
  if windowName == toolWindowName then
    editor.saveWindowsState()
  end
end

local function onWindowMenuItem()
  editor.showWindow(toolWindowName)
end

local function onEditorInitialized()
  editor.registerWindow(toolWindowName, im.ImVec2(500, 500))
  editor.addWindowMenuItem(toolWindowTitle, M.onWindowMenuItem, {groupMenuName = 'Audio'})
end

M.onUpdate = onUpdate
M.onEditorToolWindowShow = onEditorToolWindowShow
M.onEditorToolWindowHide = onEditorToolWindowHide
M.onWindowMenuItem = onWindowMenuItem
M.onEditorInitialized = onEditorInitialized

return M