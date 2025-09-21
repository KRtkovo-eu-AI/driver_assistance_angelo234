-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local im = ui_imgui
local ffi = require('ffi')
local whiteColorF = ColorF(1,1,1,1)
local blackColorI = ColorI(0,0,0,192)

-- main working data
local toolWindowName = "driftDataEditor"
local currentFileDir = "/gameplay/temp/"
local currentFileName
local driftData = nil
local autoSelectElement = nil

-- editor data
local selectedStuntZoneIndex = 1
local mouseInfo

-- drift spots editor
local driftSpotsLoaded = false
local currDriftSpots = {}
local selectedDriftSpotId
local newDriftSpotId = im.ArrayChar(128, "")
local rawData = im.BoolPtr(false)

local defaultPointsCopy
local stuntZonesPresets = {
    driftThrough = {absolute = false, presets = {1000, 1500, 2000}},
    donut = {absolute = true, presets = {250, 500, 750}},
    hitPole = {absolute = false, presets = {500, 1000, 1500}},
    nearPole = {absolute = false, presets = {500, 1000, 1500}},
}

local driftSpotsUnsavedChanges = false

local function getNewDriftData()
  return {
    stuntZones = {}
  }
end

local function moveStuntZoneOrder(t, old, new)
  local value = t[old]
  if new < old then
     table.move(t, new, old - 1, new + 1)
  else
     table.move(t, old + 1, new, old)
  end
  t[new] = value
  M.selectStuntZone(new)
end


local stuntZoneTransform
local function drawElementDetail(elem)
  if not elem then return end

  if stuntZoneTransform and stuntZoneTransform:update(mouseInfo) then
    elem.pos = stuntZoneTransform.allowTranslate and stuntZoneTransform.pos or nil
    elem.rot = stuntZoneTransform.allowRotate and stuntZoneTransform.rot or nil
    elem.scl = stuntZoneTransform.allowScale and stuntZoneTransform.scl or nil
  end
  if elem.cooldown then
    local rv = im.IntPtr(elem.cooldown)
    if im.InputInt ("Cooldown", rv) then
      elem.cooldown = rv[0]
    end
  end
  if not elem.score then
    elem.score = defaultPointsCopy[elem.type]
  else
    local stuntZonePreset = stuntZonesPresets[elem.type] -- find preset for this stunt zone type

    -- build the string from the stunt zone type presets that will be fed to the Combo2
    local s = ""
    for _, n in ipairs(stuntZonePreset.presets) do
      s = s .. tostring(n) .. "\0"
    end

    -- the Combo2 needs a intPtr for the current chosen option
    local presetPtr = im.IntPtr((tableFindKey(stuntZonePreset.presets, elem.score) or 2) - 1)
    if im.Combo2(stuntZonePreset.absolute and "Points" or "Max points", presetPtr, s) then
      elem.score = stuntZonePreset.presets[tonumber(presetPtr[0]) + 1]
    end
  end
end

local function createNewTransform(label, allowTranslate, allowRotate, allowScale, pos, rot, scl, oneDimScale)
  local tr = require('/lua/ge/extensions/editor/util/transformUtil')(label, label)
  tr.allowTranslate = allowTranslate
  tr.allowRotate = allowRotate
  tr.allowScale = allowScale
  tr:setOneDimensionalScale(oneDimScale)
  tr:set(pos, rot, scl)
  tr:enableEditing()
  return tr
end

local transformsUtils = {}
local function selectDriftSpot(id)
  selectedDriftSpotId = id
  transformsUtils = {}
  for lineName, lineData in pairs(currDriftSpots[selectedDriftSpotId].lines) do
    transformsUtils[lineName.."driftBox"] = createNewTransform("Sign pos", true, true, true, lineData.pos, quatFromEuler(lineData.rot.x, lineData.rot.y, lineData.rot.z), lineData.scl)
    transformsUtils[lineName.."startDir"] = createNewTransform("Entry dir", true, false, false, lineData.pos + lineData.startDir, nil, nil)
  end
end

local function selectStuntZone(index)
  selectedStuntZoneIndex = index
  local elem = driftData.stuntZones[selectedStuntZoneIndex]
  local label = string.format("%d-%s",index, elem.type)

  stuntZoneTransform = nil

  if elem.type == "donut" then
    stuntZoneTransform = createNewTransform(label, true, false, true, elem.pos, nil, elem.scl, true)
  end

  if elem.type == "driftThrough" then
    stuntZoneTransform = createNewTransform(label, true, true, true, elem.pos, elem.rot, elem.scl, false)
  end

  if elem.type == "hitPole" then
    stuntZoneTransform = createNewTransform(label, true, false, false, elem.pos, nil, nil, false)
  end
end

------------------------
-- menu/saving/window --
------------------------

local function saveDriftData(savePath)
  jsonWriteFile(savePath, driftData, true)
  local dir, filename, ext = path.split(savePath)
  currentFileDir = dir
  currentFileName = filename
end

local function loadDriftData(filename)
  if not filename then
    return
  end
  local json = jsonReadFile(filename)
  if not json then
    log('E', logTag, 'unable to find driftData file: ' .. tostring(filename))
    return
  end

  -- "cast" scl, pos and rot to vec/quat
  for _, elem in ipairs(json.stuntZones or {}) do
    if elem.pos and type(elem.pos) == "table" and elem.pos.x and elem.pos.y and elem.pos.z then elem.pos = vec3(elem.pos) end
    if elem.rot and type(elem.rot) == "table" and elem.rot.x and elem.rot.y and elem.rot.z and elem.rot.w then elem.rot = quat(elem.rot) end
    if elem.scl and type(elem.scl) == "table" and elem.scl.x and elem.scl.y and elem.scl.z then elem.scl = vec3(elem.scl) end
    if elem.scl and type(elem.scl) == "number" then end
  end

  local dir, filename, ext = path.split(filename)
  currentFileDir = dir
  currentFileName = filename

  driftData = json

  M.selectStuntZone(1)
end

local function clearStuntZonesEditor()
  currentFileDir = "/gameplay/temp/"
  currentFileName = nil
  driftData = getNewDriftData()
end

local function sanitizeDriftSpot(spotData)
  local newDriftSpot = {
    racePath = spotData.racePath,
    bounds = spotData.bounds,
    lines = {},
    id = spotData.id,
    name = spotData.name,
    info = spotData.info -- Add info field to store objectives data
  }
  for lineName, lineData in pairs(spotData.lines) do
    local newLineData = {
      pos = vec3(lineData.pos),
      scl = vec3(lineData.scl),
      rot = quat(lineData.rot),
      startDir = vec3(lineData.startDir),
      markerObjects = lineData.markerObjects
    }

    newDriftSpot.lines[lineName] = newLineData
  end
  return newDriftSpot
end

local function loadDriftSpotsForCurrLevel()
  local tempSpot
  for _, spotData in pairs(gameplay_drift_saveLoad.loadAndSanitizeDriftFreeroamSpotsCurrMap()) do
    tempSpot = sanitizeDriftSpot(spotData)
    -- Load the info.json file
    local infoPath = "levels/"..getCurrentLevelIdentifier().."/driftSpots/"..tempSpot.id:match("([^/]+)$").."/info.json"
    tempSpot.info = jsonReadFile(infoPath) or {
      name = tempSpot.id:match("([^/]+)$"),
      objectives = {
        {id = "bronze", score = 1200, rewards = {money = 1000}},
        {id = "silver", score = 3100, rewards = {money = 1000}},
        {id = "gold", score = 7200, rewards = {money = 1000}}
      },
      unlock = {}
    }
    currDriftSpots[tempSpot.id] = tempSpot
  end
end

local function deleteDriftSpot(spotId)
  currDriftSpots[spotId] = nil
  for key, value in pairs(currDriftSpots) do
    selectDriftSpot(key)
    break
  end
  FS:remove("levels/"..getCurrentLevelIdentifier().."/driftSpots/"..spotId)
end

local function saveCurrentDriftSpots()
  for spotId, spotData in pairs(currDriftSpots) do
    local newSpotData = {}
    -- Extract name from spotId (gets everything after the last '/')
    local name = spotId:match("([^/]+)$")
    for lineName, lineData in pairs(spotData.lines) do
      local newLineData = {
        pos = lineData.pos:toTable(),
        scl = lineData.scl:toTable(),
        rot = lineData.rot:toTable(),
        startDir = lineData.startDir:toTable(),
        markerObjects = lineData.markerObjects
      }
      newSpotData[lineName] = newLineData
    end
    local path = "levels/"..getCurrentLevelIdentifier().."/driftSpots/"..name
    jsonWriteFile(path.."/".."spot.driftSpot.json", newSpotData, true)

    -- Save the info.json file
    if spotData.info then
      jsonWriteFile(path.."/info.json", spotData.info, true)
    end
  end
  driftSpotsUnsavedChanges = false
end

local function createNewDriftSpot(name)
  local path = "levels/"..getCurrentLevelIdentifier().."/driftSpots/"..name

  if currDriftSpots[name] then return end
  local camPos = core_camera.getPosition()
  local spotData = {
    lines = {},
    racePath = path .. "/race.race.json",
    bounds = path .. "/bounds.sites.json"
  }

  for i = 1, 2, 1 do
    local newLineData = {
      pos = camPos:toTable(),
      scl = vec3(1, 1, 1):toTable(),
      rot = quat(0, 0, 0, 1):toTable(),
      startDir = vec3(0,0,0),
      markerObjects = {}
    }

    spotData.lines[i == 1 and "lineOne" or "lineTwo"] = newLineData
  end

  local defaultInfoJson = {
    name = name,
    objectives = {
      {
        id = "bronze",
        score = 1200,
        rewards = {
          money = 1000,
        }
      },
      {
        id = "silver",
        score = 3100,
        rewards = {
          money = 1000,
        }
      },
      {
        id = "gold",
        score = 7200,
        rewards = {
          money = 1000,
        }
      }
    },
    unlock = {}
  }

  jsonWriteFile(spotData.racePath, {}, true)
  jsonWriteFile(spotData.bounds, {}, true)
  jsonWriteFile(path .. "/info.json", defaultInfoJson, true)
  jsonWriteFile(path.."/".."spot.driftSpot.json", spotData.lines, true)

  local id = getCurrentLevelIdentifier().."/"..name

  currDriftSpots[id] = sanitizeDriftSpot(spotData)

  selectDriftSpot(id)
end

local function driftStuntZonesEditor()
  -- menu bar to load/save/etc
  if im.BeginMenuBar() then
    if im.BeginMenu("File") then

      if im.MenuItem1("Load...") then
        editor_fileDialog.openFile(function(data) loadDriftData(data.filepath) end, {{"driftData Files",".driftData.json"}}, false, currentFileDir)
      end
      local canSave = currentFileDir and currentFileName and driftData
      if not canSave then im.BeginDisabled() end
      if im.MenuItem1("Save") then
        saveDriftData(currentFileDir .. currentFileName)
      end
      if not canSave then im.EndDisabled() end
      if im.MenuItem1("Save as...") then
        extensions.editor_fileDialog.saveFile(function(data) saveDriftData(data.filepath) end, {{"driftData Files",".driftData.json"}}, false, currentFileDir)
      end
      if im.MenuItem1("Clear") then
        clearStuntZonesEditor()
      end
      if im.MenuItem1("Create Stunt Zones around Vehicle") then
        local obj = scenetree.findObjectById(be:getPlayerVehicleID(0))
        if obj then
          local pos = obj:getPosition()
          driftData = { stuntZones = {{type = "donut", pos = pos + vec3(10, 0, 0), scl = 10},{type = "driftThrough", rot = quat(0, 0, 0, 1), pos = pos + vec3(10, 20, 0), scl = vec3(8, 1, 1)},{type = "hitPole", pos = pos + vec3(-10, 0, 0)}}}
        end
      end
      im.EndMenu()
    end

    if im.BeginMenu("Add new stunt zone...") then
      if im.MenuItem1("Add Donut") then
        local obj = scenetree.findObjectById(be:getPlayerVehicleID(0))
        if obj then
          local pos = obj:getPosition()
          table.insert(driftData.stuntZones,
          {
            type = "donut",
            cooldown = 8,
            pos = pos + vec3(10, 0, 0),
            scl = 10,
            score = shallowcopy(defaultPointsCopy.donutPoints)
          })
          autoSelectElement = #driftData.stuntZones
        end
      end
      if im.MenuItem1("Add Drift Through") then
        local obj = scenetree.findObjectById(be:getPlayerVehicleID(0))
        if obj then
          local pos = obj:getPosition()
          table.insert(driftData.stuntZones,
          {
            type = "driftThrough",
            cooldown = 8,
            rot = quat(0, 0, 0, 1),
            pos = pos + vec3(10, 20, 0),
            scl = vec3(8, 1, 1),
            score = shallowcopy(defaultPointsCopy.driftThroughPoints)
          })
          autoSelectElement = #driftData.stuntZones
        end
      end
      if im.MenuItem1("Add Hit Pole") then
        local obj = scenetree.findObjectById(be:getPlayerVehicleID(0))
        if obj then
          local pos = obj:getPosition()
          table.insert(driftData.stuntZones,
          {
            type = "hitPole",
            pos = pos + vec3(-10, 0, 0),
            score = shallowcopy(defaultPointsCopy.hitPolePoints)
          })
          autoSelectElement = #driftData.stuntZones
        end
      end
      im.EndMenu()
    end
    im.EndMenuBar()
  end

  if #driftData.stuntZones > 0 then
    M.updateMouseInfo()
    if autoSelectElement then
      M.selectStuntZone(autoSelectElement)
    end

    im.Columns(2,'mainDrift')
    -- element selector and "New"
    im.BeginChild1("element select", im.GetContentRegionAvail(), 1)
    im.Text("Drift Stunt Zones:")
    for i, elem in ipairs(driftData.stuntZones) do
      if im.Selectable1(string.format("%d - %s", i, elem.type), i == selectedStuntZoneIndex) then
        -- select element
        M.selectStuntZone(i)
      end
    end

    im.EndChild()

    im.NextColumn()

    -- element detail
    im.BeginChild1("element detail", im.GetContentRegionAvail(), 1)

    -- header bar
    if im.Button("Delete") then
      table.remove(driftData.stuntZones, selectedStuntZoneIndex)
      -- deletion
    end
    im.SameLine()
    if selectedStuntZoneIndex - 1 >= 1 then
      if im.Button("Move Up") then
        moveStuntZoneOrder(driftData.stuntZones, selectedStuntZoneIndex, selectedStuntZoneIndex - 1)
      end
      im.SameLine()
    end
    if selectedStuntZoneIndex + 1 <= #driftData.stuntZones then
      if im.Button("Move Down") then
        moveStuntZoneOrder(driftData.stuntZones, selectedStuntZoneIndex, selectedStuntZoneIndex + 1)
      end
    end
    im.Separator()
    M.drawElementDetail(driftData.stuntZones[selectedStuntZoneIndex])
    im.EndChild()

    im.NextColumn()
    im.Columns(0)

    -- then debug drawing
    for i, elem in ipairs(driftData.stuntZones) do
      if elem.type == "donut" then
        debugDrawer:drawTextAdvanced(elem.pos, String(string.format("%d - %s", i, elem.type)), whiteColorF, true, false, blackColorI)

        debugDrawer:drawSphere(elem.pos, elem.scl, ColorF(0,1,0,0.2))
      end

      if elem.type == "driftThrough" then
        debugDrawer:drawTextAdvanced(elem.pos, String(string.format("%d - %s", i, elem.type)), whiteColorF, true, false, blackColorI)
        local x, y, z = elem.rot * vec3(elem.scl.x,0,0), elem.rot * vec3(0,elem.scl.y,0), elem.rot * vec3(0,0,elem.scl.z)
        local scl = (x+y+z)/2
        M.drawAxisBox(((-scl*2)+elem.pos),x*2,y*2,z*2,color(0,0,255,0.2*255))
        --local scl = (x+y+z)
        --M.drawAxisBox((-scl+elem.pos),x*2,y*2,z*2,color(0,0,255,0.2*255))
      end

      if elem.type == "hitPole" then
        debugDrawer:drawTextAdvanced(elem.pos, String(string.format("%d - %s", i, elem.type)), whiteColorF, true, false, blackColorI)
        debugDrawer:drawCylinder(elem.pos, elem.pos + vec3(0,0,3), 0.2,  ColorF(1,0,0,0.2))
      end
    end


  end
  autoSelectElement = nil

end

local function driftSpotsEditor()
  if not driftSpotsLoaded then
    loadDriftSpotsForCurrLevel()
    driftSpotsLoaded = true
  end

  -- Add save/revert buttons at the top
  local wasUnsaved = driftSpotsUnsavedChanges
  if driftSpotsUnsavedChanges then
    im.PushStyleColor2(im.Col_Text, im.ImVec4(1, 0, 0, 1))
  end
  if im.Button("Save changes") then
    saveCurrentDriftSpots()
  end
  if wasUnsaved then
    im.PopStyleColor()
  end

  im.Separator()

  im.Columns(2, "DriftSpotsCurrLevel")

  im.BeginChild1("driftSpots", im.GetContentRegionAvail(), 1)
  im.TextWrapped("Drift spots in the current level: ")
  for spotId, spotData in pairs(currDriftSpots) do
    if im.Selectable1(spotId, spotId == selectedDriftSpotId) then
      selectDriftSpot(spotId)
    end
  end

  im.Separator()
  im.InputText("Spot id", newDriftSpotId)
  if im.Button("Create new drift spot") then
    if #ffi.string(newDriftSpotId) > 0 then
      createNewDriftSpot(ffi.string(newDriftSpotId))
      newDriftSpotId = im.ArrayChar(128, "")
    end
  end
  im.EndChild()

  im.NextColumn()

  im.BeginChild1("Drift spots details", im.GetContentRegionAvail(), 1)
  if selectedDriftSpotId then
    if im.Button("TP player to drift spot") then
      local player = scenetree.findObjectById(be:getPlayerVehicleID(0))
      if player then
        spawn.safeTeleport(player, vec3(currDriftSpots[selectedDriftSpotId].lines.lineOne.pos), quat(currDriftSpots[selectedDriftSpotId].lines.lineOne.rot), nil, nil, nil, nil, false)
      end
    end
    im.Checkbox("Raw data", rawData)
    if rawData[0] then
      if im.Begin("Raw drift spot's data") then
        im.Text(dumps(currDriftSpots[selectedDriftSpotId]))
        im.End()
      end
    end
    if im.Button("Delete") then
      deleteDriftSpot(selectedDriftSpotId)
    end
    im.Text("Drift spot's path")
    im.SameLine()
    if im.Button("Open Race Editor") then
      if editor_raceEditor then
        if not editor.active then
          editor.setEditorActive(true)
        end
        editor_raceEditor.show()
        if currDriftSpots[selectedDriftSpotId].racePath then
          editor_raceEditor.loadRace(currDriftSpots[selectedDriftSpotId].racePath)
        end
      end
    end
    im.Text("Drift spot's bounds")
    im.SameLine()
    if im.Button("Open Sites Editor") then
      if editor_sitesEditor then
        if not editor.active then
          editor.setEditorActive(true)
        end
        editor_sitesEditor.show()
        if currDriftSpots[selectedDriftSpotId].bounds then
          editor_sitesEditor.loadSites(currDriftSpots[selectedDriftSpotId].bounds)
        end
      end
    end
    im.Separator()
    for lineName, lineData in pairs(currDriftSpots[selectedDriftSpotId].lines) do
      im.Text(lineName)
      im.Dummy(im.ImVec2(1, 5))

      -- drift detection box
      local x, y, z = lineData.rot * vec3(lineData.scl.x,0,0), lineData.rot * vec3(0,lineData.scl.y,0), lineData.rot * vec3(0,0,lineData.scl.z)
      local scl = (x+y+z)/2
      im.Text("Drift detection box")
      debugDrawer:drawTextAdvanced(vec3(lineData.pos), String(lineName), whiteColorF, true, false, blackColorI)
      M.drawAxisBox(((-scl*2)+vec3(lineData.pos)),x*2,y*2,z*2,color(0,0,255,0.2*255))
      local driftBoxTr = transformsUtils[lineName.."driftBox"]
      if driftBoxTr and driftBoxTr:update(mouseInfo) then
        lineData.pos = driftBoxTr.allowTranslate and driftBoxTr.pos or nil
        lineData.rot = driftBoxTr.allowRotate and driftBoxTr.rot or nil
        lineData.scl = driftBoxTr.allowScale and driftBoxTr.scl or nil
        driftSpotsUnsavedChanges = true
      end

      im.Dummy(im.ImVec2(1, 10))

      im.Text("Start dir")
      debugDrawer:drawTextAdvanced(lineData.startDir + lineData.pos, "Start dir", whiteColorF, true, false, blackColorI)
      debugDrawer:drawCylinder(lineData.startDir + lineData.pos, lineData.startDir + lineData.pos + vec3(0,0,2), 0.2,  ColorF(0,0,1,0.2))
      local startDirTr = transformsUtils[lineName.."startDir"]
      if startDirTr and startDirTr:update(mouseInfo) then
        lineData.startDir = startDirTr.allowTranslate and startDirTr.pos - lineData.pos or nil
        driftSpotsUnsavedChanges = true
      end

      -- Add marker objects list
      im.Dummy(im.ImVec2(1, 10))
      im.Text("Marker Objects")
      if not lineData.markerObjects then lineData.markerObjects = {} end

      -- Add new marker object button
      if im.Button("Add Marker Object##" .. lineName) then
        table.insert(lineData.markerObjects, "")
        driftSpotsUnsavedChanges = true
      end

      -- List existing marker objects
      for i, markerObj in ipairs(lineData.markerObjects) do
        local imVal = im.ArrayChar(2048, markerObj)

        if im.InputText("##marker"..i..lineName, imVal) then
          lineData.markerObjects[i] = ffi.string(imVal)
          driftSpotsUnsavedChanges = true
        end
        im.SameLine()
        if im.Button("Delete##" .. i .. lineName) then
          table.remove(lineData.markerObjects, i)
          driftSpotsUnsavedChanges = true
        end
      end

      if lineName == "lineOne" then
        im.Dummy(im.ImVec2(1, 10))
        im.Separator()
        im.Dummy(im.ImVec2(1, 10))
      end
    end

    -- Add objectives editor section
    if currDriftSpots[selectedDriftSpotId].info then
      im.Separator()

      -- Add name field
      local info = currDriftSpots[selectedDriftSpotId].info
      im.Text("Spot Name:")
      im.SameLine()
      local namePtr = im.ArrayChar(128, info.name or "")
      if im.InputText("##name", namePtr) then
        info.name = ffi.string(namePtr)
        driftSpotsUnsavedChanges = true
      end

      im.Separator()
      im.Text("Objectives")

      for _, objective in ipairs(info.objectives or {}) do
        im.PushID1(objective.id)

        im.Text(objective.id .. " score:")
        im.SameLine()
        local scorePtr = im.IntPtr(objective.score)
        if im.InputInt("##score", scorePtr) then
          objective.score = scorePtr[0]
          driftSpotsUnsavedChanges = true
        end

        im.PopID()
      end
    end
  end
  im.EndChild()

  im.NextColumn()
  im.Columns(0)
end

local function onEditorGui()
  if editor.beginWindow(toolWindowName,toolWindowName, im.WindowFlags_MenuBar) then

    if getCurrentLevelIdentifier() ~= nil then
      defaultPointsCopy = gameplay_drift_scoring.getScoreOptions().defaultPoints

      if im.BeginTabBar("mp_tabs##") then
        if im.BeginTabItem("Drift stunt zones editor") then
          driftStuntZonesEditor()
          im.EndTabItem()
        end
        if im.BeginTabItem("Drift spots editor") then
          driftSpotsEditor()
          M.clearStuntZonesEditor()
          im.EndTabItem()
        end
        im.EndTabBar()
      end
    else
      im.Text("Please load a level to work with the drift editor")
    end

    editor.endWindow()
  end
end


local function show()
  editor.clearObjectSelection()
  editor.showWindow(toolWindowName)
end

local function onEditorInitialized()
  editor.registerWindow(toolWindowName, im.ImVec2(1500,700))
  editor.addWindowMenuItem("Drift Data Editor", function() show() end, {groupMenuName="Gameplay"})
  if driftData == nil then
    driftData = getNewDriftData()
  end
end

local function onSerialize()
  local ret = {
    selectedStuntZoneIndex = selectedStuntZoneIndex,
    currentFileDir = currentFileDir,
    currentFileName = currentFileName,
    driftData = driftData,
  }
  return ret
end

local function onDeserialized(data)
  currentFileDir = data.currentFileDir or currentFileDir
  currentFileName = data.currentFileName or currentFileName
  driftData = data.driftData or driftData
  if data.selectedStuntZoneIndex then
    autoSelectElement = data.selectedStuntZoneIndex
  end
end


M.updateMouseInfo = function()
  if not mouseInfo then mouseInfo = {} end
  if core_forest.getForestObject() then core_forest.getForestObject():disableCollision() end
  mouseInfo.camPos = core_camera.getPosition()
  mouseInfo.ray = getCameraMouseRay()
  mouseInfo.rayDir = vec3(mouseInfo.ray.dir)
  mouseInfo.rayCast = cameraMouseRayCast()
  mouseInfo.valid = mouseInfo.rayCast and true or false

  if core_forest.getForestObject() then core_forest.getForestObject():enableCollision() end
  if not mouseInfo.valid then
    mouseInfo.down = false
    mouseInfo.hold = false
    mouseInfo.up   = false
    mouseInfo.closestNodeHovered = nil
  else
    mouseInfo.down =  im.IsMouseClicked(0) and not im.GetIO().WantCaptureMouse
    mouseInfo.hold = im.IsMouseDown(0) and not im.GetIO().WantCaptureMouse
    mouseInfo.up =  im.IsMouseReleased(0) and not im.GetIO().WantCaptureMouse
    if mouseInfo.down then
      mouseInfo.hold = false
      mouseInfo._downPos = vec3(mouseInfo.rayCast.pos)
      mouseInfo._downNormal = vec3(mouseInfo.rayCast.normal)
    end
    if mouseInfo.hold then
      mouseInfo._holdPos = vec3(mouseInfo.rayCast.pos)
      mouseInfo._holdNormal = vec3(mouseInfo.rayCast.normal)
    end
    if mouseInfo.up then
      mouseInfo._upPos = vec3(mouseInfo.rayCast.pos)
      mouseInfo._upNormal = vec3(mouseInfo.rayCast.normal)
    end
  end
end

-- helper function
M.drawAxisBox = function(corner, x, y, z, clr)
  -- draw all faces in a loop
  for _, face in ipairs({{x,y,z},{x,z,y},{y,z,x}}) do
    local a,b,c = face[1],face[2],face[3]
    -- spokes
    debugDrawer:drawLine((corner    ), (corner+c    ), ColorF(0,0,0,0.75))
    debugDrawer:drawLine((corner+a  ), (corner+c+a  ), ColorF(0,0,0,0.75))
    debugDrawer:drawLine((corner+b  ), (corner+c+b  ), ColorF(0,0,0,0.75))
    debugDrawer:drawLine((corner+a+b), (corner+c+a+b), ColorF(0,0,0,0.75))
    -- first side
    debugDrawer:drawTriSolid(
      vec3(corner    ),
      vec3(corner+a  ),
      vec3(corner+a+b),
      clr)
    debugDrawer:drawTriSolid(
      vec3(corner+b  ),
      vec3(corner    ),
      vec3(corner+a+b),
      clr)
    -- back of first side
    debugDrawer:drawTriSolid(
      vec3(corner+a  ),
      vec3(corner    ),
      vec3(corner+a+b),
      clr)
    debugDrawer:drawTriSolid(
      vec3(corner    ),
      vec3(corner+b  ),
      vec3(corner+a+b),
      clr)
    -- other side
    debugDrawer:drawTriSolid(
      vec3(c+corner    ),
      vec3(c+corner+a  ),
      vec3(c+corner+a+b),
      clr)
    debugDrawer:drawTriSolid(
      vec3(c+corner+b  ),
      vec3(c+corner    ),
      vec3(c+corner+a+b),
      clr)
    -- back of other side
    debugDrawer:drawTriSolid(
      vec3(c+corner+a  ),
      vec3(c+corner    ),
      vec3(c+corner+a+b),
      clr)
    debugDrawer:drawTriSolid(
      vec3(c+corner    ),
      vec3(c+corner+b  ),
      vec3(c+corner+a+b),
      clr)
  end
end

M.onSerialize = onSerialize
M.onDeserialized = onDeserialized

M.show = show

M.onEditorInitialized = onEditorInitialized
M.onEditorGui = onEditorGui
M.drawElementDetail = drawElementDetail

-- INTERNAL
M.selectStuntZone = selectStuntZone
M.clearStuntZonesEditor = clearStuntZonesEditor
return M