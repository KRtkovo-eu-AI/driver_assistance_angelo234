-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im = ui_imgui
local rallyUtil = require('/lua/ge/extensions/gameplay/rally/util')
local MissionSettings = require('/lua/ge/extensions/gameplay/rally/notebook/missionSettings')
local RallyEnums = require('/lua/ge/extensions/gameplay/rally/enums')
local RecceToolbox = require('/lua/ge/extensions/gameplay/rally/recceToolbox/recceToolbox')

local M = {}
local logTag = ''

local toolWindowName = "rallyEditor"
local editModeName = "Rally Editor"
local focusWindow = false
local rallyDevToolsWindow
local mouseInfo = {}

local rallyDevToolsWindowOpen = im.BoolPtr(false)
local recceToolboxOpen = im.BoolPtr(false)
local recceToolbox = nil

-- local previousFilepath = "/gameplay/missions/"
-- local previousFilename = "NewNotebook.notebook.json"
-- local currentPathFname = "/gameplay/missions/NewNotebook.notebook.json"
-- local currentPath = require('/lua/ge/extensions/gameplay/rally/notebook/path')()
local currentPath = nil
-- currentPath._fnWithoutExt = 'NewNotebook'
-- currentPath._dir = previousFilepath

local windows = {}
local pacenotesWindow, recceWindow, missionSettingsWindow, testWindow
local currentWindow = {}
local changedWindow = false
local programmaticTabSelect = false
local isDev = false

local debugDrawOpacity = 1.0

local function devTxtExists()
  return FS:fileExists('dev.txt')
end

local function select(window)
  currentWindow:unselect()
  currentWindow = window
  -- SettingsManager.load(currentPath)
  currentWindow:setPath(currentPath)
  currentWindow:selected()
  changedWindow = true
end

-- local function setNotebookRedo(data)
--   data.previous = currentPath
--   -- data.previousFilepath = previousFilepath
--   -- data.previousFilename = previousFilename
--
--   -- previousFilename = data.filename
--   -- previousFilepath = data.filepath
--   currentPath = data.path
--   -- currentPath._dir = previousFilepath
--   -- local dir, filename, ext = path.splitWithoutExt(previousFilename, true)
--   -- currentPath._fnWithoutExt = filename
--   for _, window in ipairs(windows) do
--     window:setPath(currentPath)
--     -- window:unselect()
--   end
--   -- currentWindow:selected()
--   -- select(notebookInfoWindow)
-- end

-- local function setNotebookUndo(data)
--   currentPath = data.previous
--   -- previousFilename = data.previousFilename
--   -- previousFilepath = data.previousFilepath
--   for _, window in ipairs(windows) do
--     window:setPath(currentPath)
--     -- window:unselect()
--   end
--   -- currentWindow:selected()
--   -- select(notebookInfoWindow)
-- end

-- local function getMissionDir()
--   if not currentPath then return nil end
--
--   -- looks like: C:\...gameplay\missions\pikespeak\rallyStage\aip-pikes-peak-2\rally\notebooks\
--   local notebooksDir = currentPath:dir()
--   -- log('D', 'wtf', 'notebooksDir: '..notebooksDir)
--   local rallyDir = rallyUtil.stripBasename(notebooksDir)
--   -- log('D', 'wtf', 'rallyDir: '..rallyDir)
--   -- now looks like: C:\...gameplay\missions\pikespeak\rallyStage\aip-pikes-peak-2\rally
--   local missionDir = rallyUtil.stripBasename(rallyDir)
--   -- log('D', 'wtf', 'missionDir: '..missionDir)
--   -- now looks like: C:\...gameplay\missions\pikespeak\rallyStage\aip-pikes-peak-2
--
--   return missionDir
-- end

-- local function ensureMissionSettingsFile()
--   local md = currentPath:getMissionDir()
--   if md then
--     local settingsFname = md..'/'..rallyUtil.missionRallyDir..'/'..rallyUtil.missionSettingsFname
--     if not FS:fileExists(settingsFname) then
--       log('I', logTag, 'creating mission.settings.json at '..settingsFname)
--       local settings = require('/lua/ge/extensions/gameplay/rally/notebook/missionSettings')(settingsFname)
--       settings.notebook.filename = currentPath:basename()
--       local assumedCodriverName = currentPath.codrivers.sorted[1].name
--       settings.notebook.codriver = assumedCodriverName
--       jsonWriteFile(settingsFname, settings:onSerialize(), true)
--     end
--   else
--     print('ensureMissionSettingsFile nil missionDir')
--   end
-- end

local function saveNotebook()
  if not currentPath then
    log('W', logTag, 'cant save; no notebook loaded.')
    return
  end

  if currentWindow == missionSettingsWindow then
    -- dont allow saving in the mission settings window because it kind of takes over SettingsManager
    -- its not a good design
    return
  end

  if not currentPath:save() then
    return
  end

  -- SettingsManager.ensureMissionSettingsFile(currentPath)
end

local function selectPrevPacenote()
  if currentWindow ~= pacenotesWindow then return end
  pacenotesWindow:selectPrevPacenote()
end

local function selectNextPacenote()
  if currentWindow ~= pacenotesWindow then return end
  pacenotesWindow:selectNextPacenote()
end

-- local function cycleDragMode()
--   if currentWindow ~= pacenotesWindow then return end
--   pacenotesWindow:cycleDragMode()
-- end

local function insertMode()
  if currentWindow ~= pacenotesWindow then return end
  pacenotesWindow:insertMode()
end

local function setFreeCam()
  local lastCamPos = core_camera.getPosition()
  local lastCamRot = core_camera.getQuat()

  core_camera.setByName(0, 'free')
  core_camera.setPosition(0, lastCamPos)
  core_camera.setRotation(0, lastCamRot)
end

local function deselect()
  if currentWindow ~= pacenotesWindow then return end
  pacenotesWindow:deselect()
end

local function selectNextWaypoint()
  if currentWindow ~= pacenotesWindow then return end
  pacenotesWindow:selectNextWaypoint()
end

local moveWaypointState = {
  debounce = 0.25,
  lastMoveTs = 0,
  forward = 0,
  backward = 0,
}
local function moveSelectedWaypointForward(v)
  if currentWindow ~= pacenotesWindow then return end

  if v == 0 then
    moveWaypointState.lastMoveTs = 0
  end

  moveWaypointState.forward = v
end

local function moveSelectedWaypointBackward(v)
  if currentWindow ~= pacenotesWindow then return end

  if v == 0 then
    moveWaypointState.lastMoveTs = 0
  end

  moveWaypointState.backward = v
end

local function cameraPathPlay()
  if currentWindow ~= pacenotesWindow then return end
  pacenotesWindow:cameraPathPlay()
end

local function toggleCornerCalls()
  if currentWindow ~= pacenotesWindow then return end
  pacenotesWindow:toggleCornerCalls()
end

-- local function moveSelectedWaypointForwardFast()
--   pacenotesWindow:moveSelectedWaypointForwardFast()
-- end
--
-- local function moveSelectedWaypointBackwardFast()
--   pacenotesWindow:moveSelectedWaypointBackwardFast()
-- end

local function updateWindowNotebooks()
  for _, window in ipairs(windows) do
    window:setPath(currentPath)
  end
end

local function loadOrCreateNotebook(fullFilename)
  local notebook = rallyUtil.loadNotebook(fullFilename)
  if not notebook then
    log('D', logTag, 'couldnt load notebook, so creating a new one: '..fullFilename)
    notebook = rallyUtil.createNotebook(fullFilename)
  end

  currentPath = notebook
  local snaproadType = currentPath:getSnaproadType()
  editor.setPreference("rallyEditor.editing.preferredSnaproadType", snaproadType)
  updateWindowNotebooks()
end

local function loadNotebook(fullFilename)
  -- if not full_filename then
  --   return
  -- end

  -- local json = jsonReadFile(full_filename)
  -- if not json then
  --   log('E', logTag, 'couldnt find notebook file')
  -- end

  -- local newPath = require('/lua/ge/extensions/gameplay/rally/notebook/path')()
  -- newPath:setFname(full_filename)
  -- newPath:onDeserialized(json)

  local notebook = rallyUtil.loadNotebook(fullFilename)
  if not notebook then
    log('E', logTag, 'couldnt load notebook: '..fullFilename)
    return
  end

  currentPath = notebook
  updateWindowNotebooks()
end

local function updateMouseInfo()
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
  else
    mouseInfo.down =  im.IsMouseClicked(0) and not im.GetIO().WantCaptureMouse
    mouseInfo.hold = im.IsMouseDown(0) and not im.GetIO().WantCaptureMouse
    mouseInfo.up =  im.IsMouseReleased(0) and not im.GetIO().WantCaptureMouse
    if mouseInfo.down then
      mouseInfo.hold = false
      mouseInfo._downPos = vec3(mouseInfo.rayCast.pos)
    end
    if mouseInfo.hold then
      mouseInfo._holdPos = vec3(mouseInfo.rayCast.pos)
    end
    if mouseInfo.up then
      mouseInfo._upPos = vec3(mouseInfo.rayCast.pos)
    end
  end
end

-- local function newEmptyNotebook()
--   local missionDir = currentPath:getMissionDir()
--   local notebooksFullPath = missionDir..'/'..rallyUtil.notebooksPath

--   local ts = math.floor(rallyUtil.getTime()) -- convert float to int

--   local basename = 'roadbook_'..ts..'.'..rallyUtil.notebookFileExt
--   local notebookFname = notebooksFullPath..'/'..basename

--   loadOrCreateNotebook(notebookFname)
-- end

local function openMission()
  editor_missionEditor.show()
  local mid = currentPath:missionId()
  if mid then
    editor_missionEditor.setMissionById(mid)
  end
end

local function setPreferredSnaproadType(snaproadType)
  editor.setPreference("rallyEditor.editing.preferredSnaproadType", snaproadType)
  pacenotesWindow:loadSnaproad()
end

local generateAllFreeformPopup = false

local function drawEditorGui()
  if focusWindow == true then
    im.SetNextWindowFocus()
    focusWindow = false
  end

  -- if editor.beginWindow(toolWindowName, "Rally Editor") then
  if editor.beginWindow(toolWindowName, "Rally Editor", im.WindowFlags_MenuBar) then
    if im.BeginMenuBar() then

      if im.BeginMenu("File") then
        im.Text(currentPath.fname)
        im.EndMenu()
      end -- end tools menu

      if im.BeginMenu("Tools") then
        if im.MenuItem1("Dev Tools") then
          rallyDevToolsWindow = require('/lua/ge/extensions/gameplay/rally/util/devTools')()
          rallyDevToolsWindowOpen[0] = true
        end
        -- TODO for when reccetoolbox is decoupled from recceApp
        -- if im.MenuItem1("Open Recce Toolbox") then
        --   recceToolboxOpen[0] = true
        -- end
        if im.MenuItem1("Clear Pacenote AudioMode Overrides") then
          currentPath:clearPacenoteAudioModeOverrides()
        end

        if im.MenuItem1("Generate All Freeform") then
          generateAllFreeformPopup = true
        end
        im.tooltip("Generate all freeform notes from structured notes.")

        im.EndMenu()
      end -- end tools menu

      if im.BeginMenu("Preferences") then
        im.SetNextItemWidth(70)
        if im.BeginCombo('Preferred Snaproad Type##preferredSnaproadType', editor.getPreference("rallyEditor.editing.preferredSnaproadType")) then
          for _, snaproadType in ipairs(RallyEnums.drivelineModeNames) do
            if im.Selectable1(snaproadType, snaproadType == editor.getPreference("rallyEditor.editing.preferredSnaproadType")) then
              setPreferredSnaproadType(snaproadType)
            end
          end
          im.EndCombo()
        end
        im.EndMenu()
      end -- end preferences menu

      im.EndMenuBar()
    end -- end menu bar

    if generateAllFreeformPopup then
      im.OpenPopup("Generate All Freeform##genAllFreeform")
      generateAllFreeformPopup = nil
    end

    -- if im.BeginPopupModal("Generate All Freeform##genAllFreeform") then
    if im.BeginPopupModal("Generate All Freeform##genAllFreeform", nil, im.WindowFlags_AlwaysAutoResize) then
      im.Text("Generate all freeform notes from structured notes?")
      im.Text("Text for all freeform pacenotes may be changed.")
      im.Text("Make a backup before proceeding.")
      im.Separator()
      if im.Button("Ok", im.ImVec2(120,0)) then
        currentPath:generateAllFreeform()
        im.CloseCurrentPopup()
      end
      im.SameLine()
      if im.Button("Cancel", im.ImVec2(120,0)) then
        im.CloseCurrentPopup()
      end
      im.EndPopup()
    end

    if currentPath then
      -- im.BeginChild1("##top-toolbar", im.ImVec2(0,topToolbarHeight), im.WindowFlags_ChildWindow)

      im.PushStyleColor2(im.Col_Button, im.ImColorByRGB(0,100,0,255).Value)
      if currentWindow ~= missionSettingsWindow then
        if im.Button("Save") then
          saveNotebook()
        end
      else
        im.BeginDisabled()
        if im.Button("Save") then end
        im.EndDisabled()
      end
      im.PopStyleColor(1)
      im.SameLine()
      if currentPath then
        local diff = os.time() - currentPath.updated_at
        if diff > 3600*24 then
          im.Text(string.format("saved %dd ago", math.floor(diff / (3600*24))))
        elseif diff > 3600 then
          im.Text(string.format("saved %dh ago", math.floor(diff / 3600)))
        elseif diff > 60 then
          im.Text(string.format("saved %dm ago", math.floor(diff / 60)))
        else
          im.Text(string.format("saved %ds ago", diff))
        end
      end

      -- im.SameLine()
      -- if im.Button("Refresh") then
      --   currentPath:reload()
      -- end

      if not editor.editMode or editor.editMode.displayName ~= editModeName then
        im.SameLine()
        im.PushStyleColor2(im.Col_Button, im.ImColorByRGB(255,0,0,255).Value)
        im.PushStyleColor2(im.Col_Text, im.ImColorByRGB(0,0,0,255).Value)
        if im.Button("Switch to Rally Editor Editmode", im.ImVec2(im.GetContentRegionAvailWidth(),0)) then
          editor.selectEditMode(editor.editModes.notebookEditMode)
        end
        im.PopStyleColor(2)
      end
      im.tooltip(tostring(currentPath.fname))
      -- im.SameLine()
      -- im.Text(""..tostring(currentPath.fname))

      -- im.Text("Mission: "..tostring(currentPath:missionId()))
      -- im.SameLine()
      -- if im.Button("Open Mission Editor") then
      --   openMission()
      -- end

      -- im.Text('DragMode: '..pacenotesWindow.pacenote_tools_state.drag_mode)

      -- local selParts, selMode = pacenotesWindow:selectionString()

      -- local clr = im.ImVec4(1, 0.6, 1, 1)
      -- im.PushFont3('robotomono_regular')
      -- im.TextColored(clr, 'Selection')
      -- im.TextColored(clr, '  P: '..(selParts[1] or '-'))
      -- im.TextColored(clr, '  W: '..(selParts[2] or '-'))
      -- im.PopFont()

      -- im.EndChild() -- end top-toolbar

      for i = 1,3 do im.Spacing() end

      -- local windowSize = im.GetWindowSize()
      -- local windowHeight = windowSize.y
      -- local middleChildHeight = windowHeight - topToolbarHeight - bottomToolbarHeight - heightAdditional
      -- local middleChildHeight = 1000
      -- middleChildHeight = math.max(middleChildHeight, minMiddleHeight)

      -- im.BeginChild1("##tabs-child", im.ImVec2(0,middleChildHeight), im.WindowFlags_ChildWindow and im.ImGuiWindowFlags_NoBorder )
      im.BeginChild1("##tabs-child", nil, im.WindowFlags_ChildWindow and im.ImGuiWindowFlags_NoBorder )
      if im.BeginTabBar("modes") then
        for _, window in ipairs(windows) do

          local flags = nil
          if changedWindow and currentWindow.windowDescription == window.windowDescription then
            flags = im.TabItemFlags_SetSelected
            changedWindow = false
          end

          local hasError = false
          if window.isValid then
            hasError = not window:isValid()
          end

          local tabName = (hasError and '[!] ' or '')..' '..window.windowDescription..' '..'###'..window.windowDescription

          if im.BeginTabItem(tabName, nil, flags) then
            if not programmaticTabSelect and currentWindow.windowDescription ~= window.windowDescription then
              select(window)
            end
            im.EndTabItem()
          end

        end -- for loop
        programmaticTabSelect = false
        im.EndTabBar()
      end -- tab bar

      -- local tabsHeight = 25 * im.uiscale[0]
      -- local tabContentsHeight = middleChildHeight - tabsHeight
      -- im.BeginChild1("##tab-contents-child-window", im.ImVec2(0,tabContentsHeight), im.WindowFlags_ChildWindow and im.ImGuiWindowFlags_NoBorder)
      im.BeginChild1("##tab-contents-child-window", nil, im.WindowFlags_ChildWindow and im.ImGuiWindowFlags_NoBorder)
      -- currentWindow:draw(mouseInfo, tabContentsHeight)
      currentWindow:draw(mouseInfo)
      im.EndChild() -- end top-toolbar

      im.EndChild() -- end tabs-child

      local fg_mgr = editor_flowgraphEditor.getManager()
      local paused = simTimeAuthority.getPause()
      local is_path_cam = core_camera.getActiveCamName() == "path"

      if not is_path_cam then
        if currentWindow == pacenotesWindow then
          pacenotesWindow:drawDebugEntrypoint(debugDrawOpacity)
        elseif currentWindow == recceWindow then
          recceWindow:drawDebugEntrypoint(mouseInfo)
        elseif currentWindow == testWindow then
          testWindow:drawDebugEntrypoint()
        end
      else
        if currentWindow == pacenotesWindow then
          pacenotesWindow:drawDebugCameraPlaying()
        end
      end

    else
      im.Text("No notebook loaded. Load a notebook from the Mission Editor.")
    end -- if currentPath
  end

  editor.endWindow()

  if not editor.isWindowVisible(toolWindowName) and editor.editModes and editor.editModes.displayName == editModeName then
    editor.selectEditMode(nil)
  end
end

local function onEditorGui()
  updateMouseInfo()
  drawEditorGui()
end

local function showPacenotesTab()
  programmaticTabSelect = true
  select(pacenotesWindow)
end

local function showRallyTool()
  if editor.isWindowVisible(toolWindowName) == false then
    editor.showWindow(toolWindowName)
    showPacenotesTab()
    editor.selectEditMode(editor.editModes.notebookEditMode)
  else
    focusWindow = true
    showPacenotesTab()
    editor.selectEditMode(editor.editModes.notebookEditMode)
  end
end

local function show()
  editor.clearObjectSelection()
  editor.showWindow(toolWindowName)
  editor.selectEditMode(editor.editModes.notebookEditMode)
end

local function onActivate()
  editor.clearObjectSelection()
  for _, win in ipairs(windows) do
    if win.onEditModeActivate then
      win:onEditModeActivate()
    end
  end
end

local function onDeactivate()
  for _, win in ipairs(windows) do
    if win.onEditModeDeactivate then
      win:onEditModeDeactivate()
    end
  end
  editor.clearObjectSelection()
end

local function onDeleteSelection()
  if not editor.isViewportFocused() then return end

  if currentWindow == pacenotesWindow then
    pacenotesWindow:deleteSelected()
  end
end

local function drawRallyDevToolsWindow(dtReal, dtSim, dtRaw)
  if not rallyDevToolsWindowOpen[0] then return end
  im.Begin("Rally Dev Tools", rallyDevToolsWindowOpen)
    rallyDevToolsWindow:draw(dtSim)
  im.End()
end

local function recceToolboxWindow(dtReal, dtSim, dtRaw)
  if not recceToolboxOpen[0] then return end
  im.Begin("Recce Toolbox", recceToolboxOpen)
    if not recceToolbox then
      recceToolbox = RecceToolbox()
    end
    recceToolbox:refresh()
    recceToolbox:draw()
  im.End()
end

local function onUpdate(dtReal, dtSim, dtRaw)
  local wp_fwd = moveWaypointState.forward == 1
  local wp_bak = moveWaypointState.backward == 1
  local wpMoveChanged = wp_fwd or wp_bak

  if wpMoveChanged then
    local diff = rallyUtil.getTime() - moveWaypointState.lastMoveTs
    local debounce = moveWaypointState.debounce
    local steps = 1

    if editor.keyModifiers.shift then
      debounce = debounce / 8
    end

    if editor.keyModifiers.ctrl then
      if editor.keyModifiers.shift then
        debounce = debounce * 2
      end
      steps = 10
    end

    if diff > debounce then
      moveWaypointState.lastMoveTs = rallyUtil.getTime()
      if wp_fwd then
        pacenotesWindow:moveSelectedWaypointForward(steps)
      elseif wp_bak then
        pacenotesWindow:moveSelectedWaypointBackward(steps)
      end
    end
  end

  drawRallyDevToolsWindow(dtReal, dtSim, dtRaw)
  -- recceToolboxWindow(dtReal, dtSim, dtRaw)
end

-- this is called after you Ctrl+L to reload lua.
local function onEditorInitialized()
  isDev = devTxtExists()
  -- print('isDev='..tostring(isDev))

  editor.editModes.notebookEditMode =
  {
    displayName = editModeName,
    onUpdate = onUpdate,
    onActivate = onActivate,
    onDeactivate = onDeactivate,
    onDeleteSelection = onDeleteSelection,
    actionMap = "rallyEditor", -- if available, not required
    auxShortcuts = {},
    --icon = editor.icons.tb_close_track,
    --iconTooltip = "Race Editor"
  }
  editor.editModes.notebookEditMode.auxShortcuts[editor.AuxControl_LMB] = "Select"
  editor.registerWindow(toolWindowName, im.ImVec2(500, 500))
  editor.addWindowMenuItem("Rally Editor", function() show() end,{groupMenuName="Gameplay"})

  table.insert(windows, require('/lua/ge/extensions/editor/rallyEditor/notebookInfo')(M))

  pacenotesWindow = require('/lua/ge/extensions/editor/rallyEditor/pacenotes')(M)
  table.insert(windows, pacenotesWindow)

  recceWindow = require('/lua/ge/extensions/editor/rallyEditor/recceTab')(M)
  table.insert(windows, recceWindow)

  missionSettingsWindow = require('/lua/ge/extensions/editor/rallyEditor/missionSettings')(M)
  -- table.insert(windows, missionSettingsWindow)

  table.insert(windows, require('/lua/ge/extensions/editor/rallyEditor/static')(M))

  if isDev then
    testWindow = require('/lua/ge/extensions/editor/rallyEditor/testTab')(M)
    table.insert(windows, testWindow)
  end

  for _,win in pairs(windows) do
    win:setPath(currentPath)
  end

  pacenotesWindow:attemptToFixMapEdgeIssue()

  currentWindow = pacenotesWindow
  currentWindow:selected()
end

local function onEditorToolWindowHide(windowName)
  if windowName == toolWindowName then
    editor.selectEditMode(editor.editModes.objectSelect)
  end
end

local function onWindowGotFocus(windowName)
  if windowName == toolWindowName then
    editor.selectEditMode(editor.editModes.notebookEditMode)
  end
end

local function onSerialize()
  local data = {}

  if currentPath then
    data = {
      path = currentPath:onSerialize(),
      currentPathFname = (currentPath and currentPath.fname) or nil
      -- previousFilepath = previousFilepath,
      -- previousFilename = previousFilename
    }
  end
  return data
end

local function onDeserialized(data)
  if data then
    if data.path then
      currentPath = require('/lua/ge/extensions/gameplay/rally/notebook/path')()
      currentPath:onDeserialized(data.path)
      currentPath:setFname(data.currentPathFname)
    end
    -- previousFilename = data.previousFilename  or "NewNotebook.notebook.json"
    -- previousFilepath = data.previousFilepath or "/gameplay/missions/"
    -- currentPath._dir = previousFilepath
    -- local dir, filename, ext = path.splitWithoutExt(previousFilename, true)
    -- currentPath._fnWithoutExt = filename
  end
end

local function onEditorRegisterPreferences(prefsRegistry)
  prefsRegistry:registerCategory("rallyEditor")

  prefsRegistry:registerSubCategory("rallyEditor", "editing", nil, {
    -- {name = {type, default value, desc, label (nil for auto Sentence Case), min, max, hidden, advanced, customUiFunc, enumLabels}}
    {lockWaypoints = {"bool", false, "Lock position of non-AudioTrigger waypoints.", "Lock non-AudioTrigger waypoints", nil, nil, true}},
    {showAudioTriggers = {"bool", true, "Render audio triggers in the viewport.", nil, nil, nil, true}},
    {showPreviousPacenote = {"bool", true, "When a pacenote is selected, also render the previous pacenote for reference."}},
    {showNextPacenote = {"bool", true, "When a pacenote is selected, also render the next pacenote for reference."}},
    {preferredSnaproadType = {"enum", "route", "Preferred snaproad type to use for the rally editor.", nil, nil, nil, true, nil, nil, RallyEnums.drivelineModeNames}},
  })

  prefsRegistry:registerSubCategory("rallyEditor", "waypoints", nil, {
    {defaultRadius = {"int", 8, "The radius used for displaying waypoints.", "Visual Radius", 1, 50}},
  })

  prefsRegistry:registerSubCategory("rallyEditor", "ui", nil, {
    {pacenoteNoteFieldWidth = {"int", 300, "Width of pacenote notes.note.freeform field.", nil, 1, 1000}},
  })
end

local function getPreference(key, default)
  if editor and editor.getPreference then
    return editor.getPreference(key)
  else
    return default
  end
end

local function getPrefShowAudioTriggers()
  return getPreference('rallyEditor.editing.showAudioTriggers', true)
end
local function setPrefShowAudioTriggers(val)
  editor.setPreference("rallyEditor.editing.showAudioTriggers", val)
end

local function getPrefShowPreviousPacenote()
  return getPreference('rallyEditor.editing.showPreviousPacenote', true)
end

local function getPrefShowNextPacenote()
  return getPreference('rallyEditor.editing.showNextPacenote', true)
end

local function getPrefDefaultRadius()
  return getPreference('rallyEditor.waypoints.defaultRadius', rallyUtil.default_waypoint_intersect_radius)
end

local function getPrefUiPacenoteNoteFieldWidth()
  return getPreference('rallyEditor.ui.pacenoteNoteFieldWidth', 300)
end

local function getPrefLockWaypoints()
  return getPreference("rallyEditor.editing.lockWaypoints", false)
end

local function setPrefLockWaypoints(val)
  editor.setPreference("rallyEditor.editing.lockWaypoints", val)
end

local function listNotebooks(folder)
  if not folder then
    folder = currentPath:getMissionDir()
  end
  local notebooksFullPath = folder..'/'..rallyUtil.notebooksPath
  local paths = {}
  log('I', logTag, 'loading all notebook names from '..notebooksFullPath)
  local files = FS:findFiles(notebooksFullPath, '*.notebook.json', -1, true, false)
  for _,fname in pairs(files) do
    table.insert(paths, fname)
  end
  table.sort(paths)

  log("D", logTag, dumps(paths))

  return paths
end

local function detectNotebookToLoad(missionDir)
  -- log('D', logTag, 'detectNotebookToLoad folder param: '..tostring(missionDir))
  -- if not missionDir then
    -- missionDir = currentPath:getMissionDir()
  -- end
  log('D', logTag, 'detectNotebookToLoad missionDir: '..missionDir)

  -- local migratedDriveline = false

  -- if FS:directoryExists(missionDir..'/aipacenotes') and not FS:fileExists(missionDir..'/'..rallyUtil.missionRallyDir) then
  --   log('D', logTag, 'detectNotebookToLoad renaming aipacenotes to rally')
  --   FS:directoryCreate(missionDir..'/'..rallyUtil.missionRallyDir)
  --   FS:directoryCreate(missionDir..'/'..rallyUtil.missionRallyDir..'/'..rallyUtil.notebooksDir)
  --   FS:directoryCreate(missionDir..'/'..rallyUtil.missionRallyDir..'/'..rallyUtil.recceDir)
  --   FS:directoryCreate(missionDir..'/'..rallyUtil.missionRallyDir..'/'..rallyUtil.recceDir..'/'..rallyUtil.recceRecordSubdir)

  --   local files = FS:findFiles(missionDir..'/aipacenotes/'..rallyUtil.notebooksDir, '*.notebook.json', 1, true, false)
  --   for _,fname in pairs(files) do
  --     local newFname = fname:gsub('aipacenotes', rallyUtil.missionRallyDir)
  --     log('D', logTag, 'detectNotebookToLoad renaming '..fname..' to '..newFname)
  --     FS:copyFile(fname, newFname)
  --   end

  --   local files = FS:findFiles(missionDir..'/aipacenotes/'..rallyUtil.recceDir, '*.json', 2, true, false)
  --   for _,fname in pairs(files) do
  --     local newFname = fname:gsub('aipacenotes', rallyUtil.missionRallyDir)
  --     log('D', logTag, 'detectNotebookToLoad renaming '..fname..' to '..newFname)
  --     FS:copyFile(fname, newFname)
  --     print('newFname: '..newFname)

  --     if string.find(newFname, 'driveline.json', 1, true) then
  --       print('migratedDriveline')
  --       migratedDriveline = true
  --     end
  --   end
  -- end

  local missionSettings = MissionSettings(missionDir)
  missionSettings:load()

  -- if migratedDriveline then
  --   missionSettings:setDrivelineMode(RallyEnums.drivelineMode.recce)
  -- end

  local notebookFname = missionSettings:getNotebookFullPath()
  log('D', logTag, 'detectNotebookToLoad final notebookFname: '..notebookFname)

  return notebookFname
end

local function loadForMissionEditor(missionDir)
  local notebookFname = detectNotebookToLoad(missionDir)
  editor_rallyEditor.loadOrCreateNotebook(notebookFname)
end

local function getCurrentFilename()
  if currentPath then
    return currentPath.fname
  else
    return nil
  end
end

local function changeDebugDrawOpacity(val)
  local incr = 0.2
  if val == 0 then
    debugDrawOpacity = debugDrawOpacity - incr
  elseif val == 1.0 then
    debugDrawOpacity = debugDrawOpacity + incr
  end
  local minOpacity = 0.1
  if debugDrawOpacity > 1.0 then debugDrawOpacity = 1.0 end
  if debugDrawOpacity < minOpacity then debugDrawOpacity = minOpacity end
end

local function mouseWheelZoom(val)
  if core_camera and core_camera.getActiveCamName() == 'pacenoteOrbit' then
    if val == 0 then
      core_camera.cameraZoom(0.4)
    elseif val == 1.0 then
      core_camera.cameraZoom(-0.4)
    end
  end
end

local function onVehicleResetted()
  if rallyDevToolsWindow and rallyDevToolsWindowOpen[0] then
    rallyDevToolsWindow:onVehicleResetted()
  end
end

M.onEditorRegisterPreferences = onEditorRegisterPreferences
M.onSerialize = onSerialize
M.onDeserialized = onDeserialized
M.onVehicleResetted = onVehicleResetted
M.allowGizmo = function() return editor.editMode and editor.editMode.displayName == editModeName or false end
-- M.getCurrentFilename = function() return previousFilepath..previousFilename end
M.getCurrentFilename = getCurrentFilename
M.getCurrentPath = function() return currentPath end
M.isVisible = function() return editor.isWindowVisible(toolWindowName) end
M.show = show
M.showRallyTool = showRallyTool
M.showPacenotesTab = showPacenotesTab
M.loadNotebook = loadNotebook
M.loadOrCreateNotebook = loadOrCreateNotebook
M.saveNotebook = saveNotebook
M.onEditorGui = onEditorGui
M.onEditorToolWindowHide = onEditorToolWindowHide
M.onWindowGotFocus = onWindowGotFocus
M.detectNotebookToLoad = detectNotebookToLoad
M.listNotebooks = listNotebooks
M.selectPrevPacenote = selectPrevPacenote
M.selectNextPacenote = selectNextPacenote
-- M.cycleDragMode = cycleDragMode
M.insertMode = insertMode
M.setFreeCam = setFreeCam
M.deselect = deselect
M.selectNextWaypoint = selectNextWaypoint
M.moveSelectedWaypointForward = moveSelectedWaypointForward
M.moveSelectedWaypointBackward = moveSelectedWaypointBackward
-- M.moveSelectedWaypointForwardFast = moveSelectedWaypointForwardFast
-- M.moveSelectedWaypointBackwardFast = moveSelectedWaypointBackwardFast
M.cameraPathPlay = cameraPathPlay
M.toggleCornerCalls = toggleCornerCalls
M.onEditorInitialized = onEditorInitialized
M.getPrefDefaultRadius = getPrefDefaultRadius
M.getPrefLockWaypoints = getPrefLockWaypoints
M.setPrefLockWaypoints = setPrefLockWaypoints
M.getPrefShowAudioTriggers = getPrefShowAudioTriggers
M.setPrefShowAudioTriggers = setPrefShowAudioTriggers
M.getPrefShowNextPacenote = getPrefShowNextPacenote
M.getPrefShowPreviousPacenote = getPrefShowPreviousPacenote
M.getPrefUiPacenoteNoteFieldWidth = getPrefUiPacenoteNoteFieldWidth
M.changeDebugDrawOpacity = changeDebugDrawOpacity
M.mouseWheelZoom = mouseWheelZoom
M.rallyDevToolsWindowOpen = rallyDevToolsWindowOpen
M.loadForMissionEditor = loadForMissionEditor
M.getVolatilePreferences = function() return volatilePreferences end
M.setPreferredSnaproadType = setPreferredSnaproadType

return M
