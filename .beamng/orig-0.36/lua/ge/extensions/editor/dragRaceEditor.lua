-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local im = ui_imgui
local ffi = require('ffi')
local whiteColorF = ColorF(1,1,1,1)
local blackColorI = ColorI(0,0,0,192)
-- M.dependencies = {}

-- main working data
local toolWindowName = "Drag Data Editor"
local currentFileDir = "/gameplay/temp/"
local currentFileName
local dragData = nil
local autoSelectElement = nil
local usingPrefabs = im.BoolPtr(false)
local hasEndCamera = im.BoolPtr(false)
local isUsedPtr = im.BoolPtr(false)
local allKnownTypes = {"headsUpRace", "bracketRace", "loopRace", "tuffTrucksRace", "dragPracticeRace", "streetRodRace", "streetDogfightRace"}
local allKnownPhases = {"stage", "countdown", "race", "stop"}

-- editor data
local selectedElementIndex = -1
local mouseInfo
local search = require('/lua/ge/extensions/editor/util/searchUtil')()
local gExt
local treeTypes = {".400", ".500"}

local function show()
  editor.clearObjectSelection()
  editor.showWindow(toolWindowName)
end

local function getNewDragData()
  return {
    strip = {
      lanes = {}
    },
    dragType = "",
    context = "",
    stripInfo = {
      stripName = ""
    },
    canBeTeleported = false,
    canBeReseted = false,
    phases = {},
    prefabs = {
      christmasTree = {
      isUsed = false,
      treeType = ".400"
    },
      displaySign = {
        isUsed = false,
      },
      paths = {
        isUsed = false,
      },
      decorations = {
        isUsed = false,
      },
    },
  }
end

local function onEditorInitialized()
  editor.registerWindow(toolWindowName, im.ImVec2(1500,700))
  editor.addWindowMenuItem("Drag Data Editor", function() show() end, {groupMenuName="Gameplay"})
  if dragData == nil then
    dragData = getNewDragData()
  end
end

local function onSerialize()
  local ret = {
    selectedElementIndex = selectedElementIndex,
    currentFileDir = currentFileDir,
    currentFileName = currentFileName,
  }
  return ret
end

local function onDeserialized(data)
  currentFileDir = data.currentFileDir or currentFileDir
  currentFileName = data.currentFileName or currentFileName
  dragData = data.dragData or dragData
end

local function reorderLanes(t, old, new)
  local value = t[old]
  if new < old then
     table.move(t, new, old - 1, new + 1)
  else
     table.move(t, old + 1, new, old)
  end
  t[new] = value
end

local transforms = {}
local function drawElementDetail(elem)
  if not elem then return end

  local shortName = im.ArrayChar(256, elem.shortName)
  im.Text("Short Name: ")
  im.InputText("##short", shortName, 256, nil, nil, nil)
  elem.shortName = ffi.string(shortName)
  local longName = im.ArrayChar(256, elem.longName)
  im.Text("Long Name: ")
  im.InputText("##long", longName, 256, nil, nil, nil)
  elem.longName = ffi.string(longName)
  im.NewLine()
  --Boundary
  im.Text("Boundary:")
  if transforms["boundary"] and transforms["boundary"]:update(mouseInfo) then
    elem.boundary.transform.pos = transforms["boundary"].allowTranslate and transforms["boundary"].pos or nil
    elem.boundary.transform.rot = transforms["boundary"].allowRotate and transforms["boundary"].rot or nil
    elem.boundary.transform.scl = transforms["boundary"].allowScale and transforms["boundary"].scl or nil
  end
  im.NewLine()
  for _, key in ipairs({"spawn","stage", "endLine"}) do
    im.Text(key:sub(1, 1):upper() .. key:sub(2).." Waypoint:")
    im.Text("Name: ")
    local wpName = im.ArrayChar(256, elem.waypoints[key].name)
    im.InputText("##"..key, wpName, 256, nil, nil, nil)
    im.NewLine()
    elem.waypoints[key].name = ffi.string(wpName)
    if transforms[key] and transforms[key]:update(mouseInfo) then
      elem.waypoints[key].transform.pos = transforms[key].allowTranslate and transforms[key].pos or nil
      elem.waypoints[key].transform.rot = transforms[key].allowRotate and transforms[key].rot or nil
      elem.waypoints[key].transform.scl = transforms[key].allowScale and transforms[key].scl or nil
    end
    if elem.waypoints[key] ~= nil and elem.waypoints[key].waypoint ~= nil then
      local speedAux = im.FloatPtr(elem.waypoints[key].waypoint.speed or 0)
      im.InputFloat(" Arrival Speed##"..key, speedAux)
      elem.waypoints[key].waypoint.speed = speedAux[0]

      im.PushItemWidth(60)
      im.Text("AI Arrive Type: ")
      if im.IsItemHovered() then
        im.tooltip("Select one type of arrive, each one has a different type of behaviour that will affect the AI in this lane")
      end
      im.SameLine()
      local arriveType = search:beginSearchableSimpleCombo(im, "arriveType##" .. key, elem.waypoints[key].waypoint.mode, {"set", "off", "limit"})
      if arriveType then
        elem.waypoints[key].waypoint.mode = arriveType
      end
      im.PopItemWidth()
      im.NewLine()
    end
  end
end

local function drawTransformsPreview()
  for i, lane in ipairs(dragData.strip.lanes) do
    if lane.waypoints.spawn then
      -- player spawn
      debugDrawer:drawTextAdvanced(lane.waypoints.spawn.transform.pos, String(string.format("spawn %d", i)), whiteColorF, true, false, blackColorI)
      debugDrawer:drawSphere(lane.waypoints.spawn.transform.pos, 0.25, ColorF(1,1,1,0.2))
      local x, y, z, c = lane.waypoints.spawn.transform.rot * vec3(1,0,0), lane.waypoints.spawn.transform.rot * vec3(0,1,0), lane.waypoints.spawn.transform.rot * vec3(0,0,1), lane.waypoints.spawn.transform.pos
      debugDrawer:drawTriSolid(c-x/2, c+x/2, c+y, color(0,0,255,0.2*255))
    end

    if lane.waypoints.endLine then
      --endLine
      debugDrawer:drawTextAdvanced(lane.waypoints.endLine.transform.pos, String(string.format("endLine %d", i)), whiteColorF, true, false, blackColorI)
      debugDrawer:drawSphere(lane.waypoints.endLine.transform.pos, 0.25, ColorF(1,1,1,0.2))
      local x, y, z = lane.waypoints.endLine.transform.rot * vec3(lane.waypoints.endLine.transform.scl.x,0,0), lane.waypoints.endLine.transform.rot * vec3(0,lane.waypoints.endLine.transform.scl.y,0), lane.waypoints.endLine.transform.rot * vec3(0,0,lane.waypoints.endLine.transform.scl.z)
      local c = lane.waypoints.endLine.transform.pos
      -- topleft
      debugDrawer:drawTriSolid(c-x-z, c-x+z, c+x+z, color(0,0,255,0.2*255))
      debugDrawer:drawTriSolid(c-x+z, c-x-z, c+x+z, color(0,0,255,0.2*255))
      --bottomright
      debugDrawer:drawTriSolid(c+x+z, c+x-z, c-x-z, color(0,0,255,0.2*255))
      debugDrawer:drawTriSolid(c+x-z, c+x+z, c-x-z, color(0,0,255,0.2*255))
    end

    if lane.waypoints.stage then
      -- stage
      debugDrawer:drawTextAdvanced(lane.waypoints.stage.transform.pos, String(string.format("Stage %d", i)), whiteColorF, true, false, blackColorI)
      local x, y, z = lane.waypoints.stage.transform.rot * vec3(lane.waypoints.stage.transform.scl.x,0,0), lane.waypoints.stage.transform.rot * vec3(0,lane.waypoints.stage.transform.scl.y,0), lane.waypoints.stage.transform.rot * vec3(0,0,lane.waypoints.stage.transform.scl.z)
      local scl = (x+y+z)/2
      M.drawAxisBox(((-scl*2)+lane.waypoints.stage.transform.pos),x*2,y*2,z*2,color(0,0,255,0.2*255))
      local pos = vec3(lane.waypoints.stage.transform.pos)
      debugDrawer:drawLine(pos, pos + x, ColorF(1,0,0,0.8))
      debugDrawer:drawLine(pos, pos + y, ColorF(0,1,0,0.8))
      debugDrawer:drawLine(pos, pos + z, ColorF(0,0,1,0.8))
    end

    if lane.boundary then
      -- stage
      debugDrawer:drawTextAdvanced(lane.boundary.transform.pos, String(string.format("Stage %d", i)), whiteColorF, true, false, blackColorI)
      local x, y, z = lane.boundary.transform.rot * vec3(lane.boundary.transform.scl.x,0,0), lane.boundary.transform.rot * vec3(0,lane.boundary.transform.scl.y,0), lane.boundary.transform.rot * vec3(0,0,lane.boundary.transform.scl.z)
      local scl = (x+y+z)/2
      M.drawAxisBox(((-scl*2)+lane.boundary.transform.pos),x*2,y*2,z*2,color(0,0,255,0.2*255))
    end
  end

  if dragData.strip.endCamera and dragData.strip.endCamera.transform then
    -- EndCam
    local i = "End Camera"
    debugDrawer:drawTextAdvanced(dragData.strip.endCamera.transform.pos, String(string.format("End Camera %s", i)), whiteColorF, true, false, blackColorI)
    local x, y, z = quat(dragData.strip.endCamera.transform.rot) * vec3( dragData.strip.endCamera.transform.scl.x,0,0), quat(dragData.strip.endCamera.transform.rot) * vec3(0, dragData.strip.endCamera.transform.scl.y,0), quat(dragData.strip.endCamera.transform.rot) * vec3(0,0, dragData.strip.endCamera.transform.scl.z)
    local scl = (x+y+z)/2
    M.drawAxisBox(((-scl*2)+ dragData.strip.endCamera.transform.pos),x*2,y*2,z*2,color(0,0,255,0.2*255))
  end
end

local function drawPrefabHelper(label, prefabType)
  im.Text(label)
    im.SameLine()
    local boolptr = im.BoolPtr(dragData.prefabs[prefabType].isUsed or false)
    im.Checkbox("##isUsed_"..label, boolptr)
    if im.IsItemHovered() then
      im.tooltip("If it's selected but no prefab is loaded, the system will try to find the objects in the sceneTree")
    end
    dragData.prefabs[prefabType].isUsed = boolptr[0]
    im.SameLine()
    if im.Button("Load..." .. "##"..label) then
      editor_fileDialog.openFile(function(data) dragData.prefabs[prefabType].path = data.filepath end, {{"Prefab Files",".prefab.json"}}, false)
    end
    if dragData.prefabs[prefabType].path then
      im.SameLine()
      im.TextColored(im.ImVec4(0.0, 1.0, 0.0, 1.0), "Loaded!")
      im.SameLine()
      if im.Button("x##"..label.."1") then
        dragData.prefabs[prefabType].path = nil
      end
    elseif dragData.prefabs[prefabType].isUsed then
      im.SameLine()
      im.TextColored(im.ImVec4(1.0, 1.0, 0.0, 1.0), "No prefab loaded, the system will search the objects in the scenetree")
    end
end

local endCamera
local function selectCameraTransform()
  if not dragData.strip.endCamera then
    dragData.strip.endCamera = {
      transform = {pos = vec3(1, 0, 0), rot = quat(1, 0, 0, 0), scl = vec3(6,10,5)},
    }
  end
  local label = "End Camera"
  endCamera = require('/lua/ge/extensions/editor/util/transformUtil')(label, label)
  endCamera.allowTranslate = true
  endCamera.allowRotate = true
  endCamera.allowScale = true
  endCamera:set(dragData.strip.endCamera.transform.pos, dragData.strip.endCamera.transform.rot, dragData.strip.endCamera.transform.scl)

  if endCamera and endCamera:update(mouseInfo) then
    dragData.strip.endCamera.transform.pos = endCamera.allowTranslate and endCamera.pos or nil
    dragData.strip.endCamera.transform.rot = endCamera.allowRotate and endCamera.rot or nil
    dragData.strip.endCamera.transform.scl = endCamera.allowScale and endCamera.scl or nil
  end
end

M.selectElement = function(index)
  selectedElementIndex = index
  local elem = dragData.strip.lanes[selectedElementIndex]

  if not elem then return end

  local label = "Player Spawn " .. index
  transforms.spawn = require('/lua/ge/extensions/editor/util/transformUtil')(label, label)
  transforms.spawn.allowTranslate = true
  transforms.spawn.allowRotate = true
  transforms.spawn.allowScale = true
  transforms.spawn:set(elem.waypoints.spawn.transform.pos, elem.waypoints.spawn.transform.rot,  elem.waypoints.spawn.transform.scl)

  label = "Stage  " .. index
  transforms.stage = require('/lua/ge/extensions/editor/util/transformUtil')(label, label)
  transforms.stage.allowTranslate = true
  transforms.stage.allowRotate = true
  transforms.stage.allowScale = true
  transforms.stage:set(elem.waypoints.stage.transform.pos, elem.waypoints.stage.transform.rot, elem.waypoints.stage.transform.scl)

  label = "End Line " .. index
  transforms.endLine = require('/lua/ge/extensions/editor/util/transformUtil')(label, label)
  transforms.endLine.allowTranslate = true
  transforms.endLine.allowRotate = true
  transforms.endLine.allowScale = true
  transforms.endLine:set(elem.waypoints.endLine.transform.pos, elem.waypoints.endLine.transform.rot, elem.waypoints.endLine.transform.scl)

  label = "Boundary " .. index
  transforms.boundary = require('/lua/ge/extensions/editor/util/transformUtil')(label, label)
  transforms.boundary.allowTranslate = true
  transforms.boundary.allowRotate = true
  transforms.boundary.allowScale = true
  transforms.boundary:set(elem.boundary.transform.pos, elem.boundary.transform.rot, elem.boundary.transform.scl)
end

local function headsUpRace()
  im.NewLine()
  im.PushItemWidth(120)
  im.Text("Phases: ")
  if im.IsItemHovered() then
    im.tooltip("Select one or more phases, the order selected will be the one that will be followed in the Drag Race")
  end
  im.SameLine()
  local phase = search:beginSearchableSimpleCombo(im, "phase", "Select Phase", allKnownPhases)
  if phase then
    table.insert(dragData.phases, {
      name = phase,
      dependency = true, --true if this phase depends on another to be completed or started
      startedOffset = 0,
    })
  end
  im.PopItemWidth()
  for i, p in ipairs(dragData.phases) do
    im.Text(i ..". ")
    im.SameLine()
    im.Text(p.name)
    im.SameLine()
    if im.Button("x##"..i) then
      table.remove(dragData.phases, i)
    end
    --Move Around
    if i - 1 >= 1 then
      im.SameLine()
      if im.Button("Move Up##" .. i) then
        reorderLanes(dragData.phases, i, i - 1)
      end
    end
    if i + 1 <= #dragData.phases then
      im.SameLine()
      if im.Button("Move Down##" .. i) then
        reorderLanes(dragData.phases, i, i + 1)
      end
    end

    local dependency = im.BoolPtr(p.dependency)
    im.Checkbox("Dependencies: ##".. i .."dep", dependency)
    p.dependency = dependency[0]
    im.NewLine()
  end

  im.Text("Prefabs")
  if im.IsItemHovered() then
    im.tooltip("If you decide to use prefabs, you will have to create them separately and add them to this file.")
  end
  im.SameLine()
  drawPrefabHelper("Christmas Tree: ", "christmasTree")
  if dragData.prefabs.christmasTree.isUsed then
    im.PushItemWidth(80)
    im.Text("Drag Tree Type: ")
    if im.IsItemHovered() then
      im.tooltip("Select one of the two different drag tree types, .400 is a random timed lights and .500 is a temporized lights type.")
    end
    im.SameLine()
    local tType = search:beginSearchableSimpleCombo(im, "Select Type", dragData.prefabs.christmasTree.treeType, treeTypes)
    if tType then
      dragData.prefabs.christmasTree.treeType = tType
    end
    im.PopItemWidth()
  end
  drawPrefabHelper("Display Sign: ", "displaySign")
  drawPrefabHelper("AI Path: ", "paths")
  drawPrefabHelper("Decoration: ", "decorations")
  im.NewLine()

  --End Camera Transform
  im.Text("End Camera Transform: ")
  if im.IsItemHovered() then
    im.tooltip("Transform of the end camera, set it to look at the display signs for example.")
  end
  im.SameLine()
  im.Checkbox("##".."cameraTransform", hasEndCamera)
  if hasEndCamera[0] then
    selectCameraTransform()
  else
    if dragData.strip.endCamera then
      dragData.strip.endCamera = nil
    end
  end
  im.NewLine()
  local canBeTeleported = im.BoolPtr(dragData.canBeTeleported or false)
  im.Checkbox("Can racers be teleported? ##canBeTeleported", canBeTeleported)
  dragData.canBeTeleported = canBeTeleported[0]

  local canBeReseted = im.BoolPtr(dragData.canBeReseted or false)
  im.Checkbox("Can racers be Reseted? ##canBeReseted", canBeReseted)
  dragData.canBeReseted = canBeReseted[0]
  im.NewLine()

  im.BeginChild1("element select", im.GetContentRegionAvail(), 1)
  im.Text("Drag Lanes:")
  im.SameLine()
  if im.Button("Add Lane") then
    --Set the waypoints names here, so they can be generated even if there is drag waypoints in the level
    local lane = {
      shortName = "Left",
      longName = "Left Lane",
      laneOrder = #dragData.strip.lanes + 1,
      color = "blue",
      waypoints = {
        spawn = {
          name = "drag_" .. tostring(#dragData.strip.lanes + 1) .. "_spawn",
          transform = {pos = vec3(1, 0, 0), rot = quat(1, 0, 0, 0), scl = vec3(3,3,3)},
          waypoint = {
            speed = 5,
            mode = "limit"
          }
        },
        stage = {
          name = "drag_" .. tostring(#dragData.strip.lanes + 1) .. "_stage",
          transform = {pos = vec3(1, 0, 0), rot = quat(1, 0, 0, 0), scl = vec3(3,3,3)},
          waypoint = {
            speed = 5,
            mode = "limit"
          }
        },
        endLine = {
          name = "drag_" .. tostring(#dragData.strip.lanes + 1) .. "_endLine",
          transform = {pos = vec3(1, 0, 0), rot = quat(1, 0, 0, 0), scl = vec3(3,3,3)},
          waypoint = {
            speed = 5,
            mode = "limit"
          }
        }
      },
      boundary = {
        transform = {pos = vec3(1, 0, 0), rot = quat(1, 0, 0, 0), scl = vec3(3,3,3)}
      },
    }
    table.insert(dragData.strip.lanes, lane)
    M.selectElement(#dragData.strip.lanes)
  end
  if selectedElementIndex > 0 then
    im.SameLine()
    if im.Button("Remove Lane") then
      table.remove(dragData.strip.lanes, selectedElementIndex)
      selectedElementIndex = selectedElementIndex -1
      if selectedElementIndex == 0 and #dragData.strip.lanes > 0 then selectedElementIndex = 1 end
      if #dragData.strip.lanes <= 0 then selectedElementIndex = -1 end
    end
  end
  for i,_ in ipairs(dragData.strip.lanes) do
    if im.Selectable1(string.format("Lane %d", i), i == selectedElementIndex) then
      -- select element
      im.NewLine()
      M.selectElement(i)
    end
  end

  im.EndChild()

  im.NextColumn()

  -- element detail
  im.BeginChild1("element detail", im.GetContentRegionAvail(), 1)
  -- header bar
  if selectedElementIndex > 0 then
    im.Text("Lane " .. selectedElementIndex)
    if selectedElementIndex - 1 >= 1 then
      im.SameLine()
      if im.Button("Move Up") then
        reorderLanes(dragData.strip.lanes, selectedElementIndex, selectedElementIndex - 1)
        dragData.strip.lanes[selectedElementIndex].laneOrder = selectedElementIndex
        M.selectElement(selectedElementIndex)
      end
    end
    if selectedElementIndex + 1 <= #dragData.strip.lanes then
      im.SameLine()
      if im.Button("Move Down") then
        reorderLanes(dragData.strip.lanes, selectedElementIndex, selectedElementIndex + 1)
        dragData.strip.lanes[selectedElementIndex].laneOrder = selectedElementIndex
        M.selectElement(selectedElementIndex)
      end
    end
    M.drawElementDetail(dragData.strip.lanes[selectedElementIndex])
  end
  im.EndChild()

  im.NextColumn()
  im.Columns(0)
end

local function noCtxt(ctxt)
  im.NewLine()
  im.Text("There is no data for this Type of Drag yet, this is still a WIP tool, thanks for reading this ^^")
end

local function saveDragData(savePath)
  local cleanData = deepcopy(dragData)
  jsonWriteFile(savePath, cleanData, true)
  local dir, filename, ext = path.split(savePath)
  currentFileDir = dir
  currentFileName = filename
end

local function loadDragData(filename)
  if not filename then
    return
  end
  local json = jsonReadFile(filename)
  if not json then
    log('E', logTag, 'unable to find dragData file: ' .. tostring(filename))
    return
  end
  local dir, filename, ext = path.split(filename)
  currentFileDir = dir
  currentFileName = filename

  dragData = json

  for _, lane in ipairs(dragData.strip.lanes) do
    lane.boundary.transform.pos = vec3(lane.boundary.transform.pos) or vec3()
    lane.boundary.transform.rot = quat(lane.boundary.transform.rot) or quat()
    lane.boundary.transform.scl = vec3(lane.boundary.transform.scl) or vec3()
    for _, value in pairs(lane.waypoints) do
      value.transform.pos = vec3(value.transform.pos) or vec3()
      value.transform.rot = quat(value.transform.rot) or quat()
      value.transform.scl = vec3(value.transform.scl) or vec3()
    end
  end

  if dragData.strip.endCamera then
    hasEndCamera = im.BoolPtr(true)
  end

  for _, value in pairs(dragData.prefabs) do
    if value.isUsed then
      usingPrefabs = im.BoolPtr(true)
      break
    end
  end

  M.selectElement(1)
end

local funcContext = {
  ["headsUpRace"] = headsUpRace,
  ["bracketRace"] = headsUpRace,
  ["loopRace"] = noCtxt,
  ["tuffTrucksRace"] = noCtxt,
  ["dragPracticeRace"] = headsUpRace,
  ["streetRodRace"] = noCtxt,
  ["streetDogfightRace"] = noCtxt,
}

local function dragMenu()
  im.Columns(2,'mainDrag')

    im.Text("Drag Name: ")
    local dragName = im.ArrayChar(256, dragData.stripInfo.stripName)
    im.InputText("##dragName", dragName, 256, nil, nil, nil)
    dragData.stripInfo.stripName = ffi.string(dragName)

  im.Text("Drag Types: ")
  im.SameLine()
  im.PushItemWidth(120)
  local dType = search:beginSearchableSimpleCombo(im, "dragTypes", dragData.dragType ~= "" and dragData.dragType or "Select Drag Types", allKnownTypes)
  if dType then
    dragData.dragType = dType
  end
  im.PopItemWidth()

  im.Text("Drag Contexts: ")
  im.SameLine()
  im.PushItemWidth(120)
  local ctxt = search:beginSearchableSimpleCombo(im, "dragContext", dragData.context ~= "" and dragData.context or "Select Context", {"freeroam", "activity"})
  if ctxt then
    dragData.context = ctxt
  end
  im.PopItemWidth()


  if dragData.dragType ~= "" then
    funcContext[dragData.dragType]()
  end

  drawTransformsPreview()
end

local function menuBar()
  if im.BeginMenuBar() then
    if im.BeginMenu("File") then

      if im.MenuItem1("Load...") then
        editor_fileDialog.openFile(function(data) loadDragData(data.filepath) end, {{"dragData Files",".dragData.json"}}, false, currentFileDir)
      end
      local canSave = currentFileDir and currentFileName and dragData
      if not canSave then im.BeginDisabled() end
      if im.MenuItem1("Save") then
        saveDragData(currentFileDir .. currentFileName)
      end
      if not canSave then im.EndDisabled() end
      if im.MenuItem1("Save as...") then
        extensions.editor_fileDialog.saveFile(function(data) saveDragData(data.filepath) end, {{"dragData Files",".dragData.json"}}, false, currentFileDir)
      end
      if im.MenuItem1("Clear") then
        currentFileDir = "/gameplay/temp/"
        currentFileName = nil
        dragData = getNewDragData()
      end
      im.EndMenu()
    end
    im.EndMenuBar()

  end
end

local function onEditorGui()
  if editor.beginWindow(toolWindowName,toolWindowName, im.WindowFlags_MenuBar) then
    menuBar()
    dragMenu()
    M.updateMouseInfo()
    editor.endWindow()
  end
end

-- helper functions
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

M.onSerialize = onSerialize
M.onDeserialized = onDeserialized

M.show = show

M.onEditorInitialized = onEditorInitialized
M.onEditorGui = onEditorGui
M.drawElementDetail = drawElementDetail
return M