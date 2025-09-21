-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.dependencies = {'core_environment'}
local imUtils = require('ui/imguiUtils')
local logTag = 'editor_main_toolbar'
local im = ui_imgui
local toolbarOpen = im.BoolPtr(true)
local toolbarWindowName = "mainToolbar"
local editModeToolbarGap = 10
local gridSnapComboItemCurrent = im.IntPtr(-1)
local rotateSnapComboItemCurrent = im.IntPtr(-1)
local scaleSnapComboItemCurrent = im.IntPtr(-1)
local FadeIconsPtr = im.FloatPtr(0)
local camSpeedPtr = im.FloatPtr(0)
local todPtr = im.FloatPtr(0)
local vehicleActionMaps = {"VehicleCommonActionMap", "VehicleSpecificActionMap"}

local gridSnapWidgetsComboItemsTbl = {"0.1", "0.2", "0.25", "0.5", "1", "1.5", "2", "2.5", "3", "4", "5", "10", "15", "20"}
local gridSnapWidgetsComboItems = im.ArrayCharPtrByTbl(gridSnapWidgetsComboItemsTbl)
local rotateSnapWidgetsComboItemsTbl = {"5", "15", "22.5", "45"}
local rotateSnapWidgetsComboItems = im.ArrayCharPtrByTbl(rotateSnapWidgetsComboItemsTbl)

local editModeReorderVars = {}
editModeReorderVars.isDragging = false
editModeReorderVars.DragMode = nil
editModeReorderVars.DragModePutBefore = nil
editModeReorderVars.ModeSortOrderKeysTbl = {}

-- Terrain Snapping
local terrainSnapSettingsOpen = false
-- Grid Snapping
local gridSnapSettingsOpen = false

local function drawGeneralToolbarButtons()
  if editor.uiIconImageButton(editor.icons.insert_drive_file, nil, nil, nil, nil) then
    editor.doNewLevel()
  end
  if im.IsItemHovered() then im.BeginTooltip() im.Text("New Level") im.EndTooltip() end
  im.SameLine()

  if editor.uiIconImageButton(editor.icons.folder, nil, nil, nil, nil) then
    editor.doOpenLevel()
  end
  if im.IsItemHovered() then im.BeginTooltip() im.Text("Open Level") im.EndTooltip() end
  im.SameLine()

  if editor.uiIconImageButton(editor.icons.save, nil, nil, nil, nil) then
    editor.doSaveLevel()
  end
  if im.IsItemHovered() then im.BeginTooltip() im.Text("Save Level") im.EndTooltip() end
  im.SameLine()

  if editor.uiIconImageButton(editor.icons.undo, nil, nil, nil, nil) then
    editor.undo()
  end
  if im.IsItemHovered() then im.BeginTooltip() im.Text("Undo") im.EndTooltip() end
  im.SameLine()

  if editor.uiIconImageButton(editor.icons.redo, nil, nil, nil, nil) then
    editor.redo()
  end
  if im.IsItemHovered() then im.BeginTooltip() im.Text("Redo") im.EndTooltip() end
  im.SameLine()

  if editor.uiIconImageButton(editor.icons.content_cut, nil, nil, nil, nil) then
    editor.cut()
  end
  if im.IsItemHovered() then im.BeginTooltip() im.Text("Cut") im.EndTooltip() end
  im.SameLine()

  if editor.uiIconImageButton(editor.icons.content_copy, nil, nil, nil, nil) then
    editor.copy()
  end
  if im.IsItemHovered() then im.BeginTooltip() im.Text("Copy") im.EndTooltip() end
  im.SameLine()

  if editor.uiIconImageButton(editor.icons.content_paste, nil, nil, nil, nil) then
    editor.paste()
  end
  if im.IsItemHovered() then im.BeginTooltip() im.Text("Paste") im.EndTooltip() end
  im.SameLine()

  if editor.uiIconImageButton(editor.icons.settings, nil, nil, nil, nil) then
    editor.showPreferences()
  end
  if im.IsItemHovered() then im.BeginTooltip() im.Text("Editor Preferences") im.EndTooltip() end
  im.SameLine()

  local vehicleButtonBgColor = im.GetStyleColorVec4(im.Col_Button)
  if editor.getPreference("ui.general.enableVehicleControls") then vehicleButtonBgColor = im.GetStyleColorVec4(im.Col_ButtonActive) end
  if editor.uiIconImageButton(editor.icons.directions_car, nil, nil, nil, vehicleButtonBgColor) then
    editor.setPreference("ui.general.enableVehicleControls", not editor.getPreference("ui.general.enableVehicleControls"))
  end
  if im.IsItemHovered() then im.BeginTooltip() im.Text("Enable driving the vehicle in editor") im.EndTooltip() end
  im.SameLine()
end

local function axisGizmoButtonsGui()
  local mode = editor.getAxisGizmoMode()
  local bgColor = nil

  if mode == editor.AxisGizmoMode_Translate then bgColor = im.GetStyleColorVec4(im.Col_ButtonActive) else bgColor = nil end
  if editor.uiIconImageButton(editor.icons.move, nil, nil, nil, bgColor) then
    editor.setAxisGizmoMode(editor.AxisGizmoMode_Translate)
  end
  if im.IsItemHovered() then im.BeginTooltip() im.Text("Translate (Key 1)") im.EndTooltip() end
  im.SameLine()

  if mode == editor.AxisGizmoMode_Rotate then bgColor = im.GetStyleColorVec4(im.Col_ButtonActive) else bgColor = nil end
  if editor.uiIconImageButton(editor.icons.rotate, nil, nil, nil, bgColor) then
    editor.setAxisGizmoMode(editor.AxisGizmoMode_Rotate)
  end
  if im.IsItemHovered() then im.BeginTooltip() im.Text("Rotate (Key 2)") im.EndTooltip() end
  im.SameLine()

  if mode == editor.AxisGizmoMode_Scale then bgColor = im.GetStyleColorVec4(im.Col_ButtonActive) else bgColor = nil end
  if editor.uiIconImageButton(editor.icons.scale, nil, nil, nil, bgColor) then
    editor.setAxisGizmoMode(editor.AxisGizmoMode_Scale)
  end
  if im.IsItemHovered() then im.BeginTooltip() im.Text("Scale (Key 3)") im.EndTooltip() end
  im.SameLine()

  local alignment = editor.getAxisGizmoAlignment()
  local alignIcon = nil

  if alignment == 0 then alignIcon = editor.icons.align_world end
  if alignment == 1 then alignIcon = editor.icons.align_local end

  if editor.uiIconImageButton(alignIcon) then
    if alignment == 0 then
      editor.setAxisGizmoAlignment(1)
    else
      editor.setAxisGizmoAlignment(0)
    end
  end
  if im.IsItemHovered() then im.BeginTooltip()
    if alignment == 0 then im.Text("World Coordinates (Key 4)") end
    if alignment == 1 then im.Text("Local Coordinates (Key 4)") end
    im.EndTooltip()
  end

  im.SameLine()

  -- GRID SNAPPING
  local gridSnapEnabled = editor.getPreference("snapping.general.snapToGrid")
  local gridSize = worldEditorCppApi.getGridSize()

  if gridSnapEnabled then bgColor = im.GetStyleColorVec4(im.Col_ButtonActive) else bgColor = nil end
  if editor.uiIconImageButton(editor.icons.snap_grid, nil, nil, nil, bgColor) then
    gridSnapEnabled = not gridSnapEnabled
    worldEditorCppApi.setGridSnap(gridSnapEnabled, gridSize)
    editor.setPreference("snapping.general.gridSize", gridSize)
    editor.setPreference("snapping.general.snapToGrid", gridSnapEnabled)
  end
  if im.IsItemHovered() then im.BeginTooltip() im.Text("Grid Snap. Right Click for options") im.EndTooltip() end
  im.SameLine()

  if im.IsItemHovered() and im.IsMouseClicked(1) then
    gridSnapSettingsOpen = not gridSnapSettingsOpen
  end

  im.PushItemWidth(50)
  if gridSnapComboItemCurrent[0] == -1 then
    for i, val in pairs(gridSnapWidgetsComboItemsTbl) do
      if tonumber(val) == gridSize then gridSnapComboItemCurrent[0] = i - 1 end
    end
  end
  if im.Combo1("##gridsize", gridSnapComboItemCurrent, gridSnapWidgetsComboItems) then
    local size = tonumber(gridSnapWidgetsComboItemsTbl[gridSnapComboItemCurrent[0] + 1])
    worldEditorCppApi.setGridSnap(gridSnapEnabled, size)
    editor.setPreference("snapping.general.gridSize", size)
  end
  if im.IsItemHovered() then im.SetTooltip("Grid Size") end
  im.PopItemWidth()

  im.SameLine()

  -- ROTATE SNAPPING
  local rotateSnapEnabled = editor.getPreference("snapping.general.rotateSnapEnabled")
  local rotateSnapSize = worldEditorCppApi.getRotateSnapAngle()

  if rotateSnapEnabled then bgColor = im.GetStyleColorVec4(im.Col_ButtonActive) else bgColor = nil end
  if editor.uiIconImageButton(editor.icons.snap_rotate, nil, nil, nil, bgColor) then
    rotateSnapEnabled = not rotateSnapEnabled
    worldEditorCppApi.setRotateSnap(rotateSnapEnabled, rotateSnapSize)
    editor.setPreference("snapping.general.rotateSnapSize", rotateSnapSize)
    editor.setPreference("snapping.general.rotateSnapEnabled", rotateSnapEnabled)
  end
  if im.IsItemHovered() then im.BeginTooltip() im.Text("Rotate Snap") im.EndTooltip() end
  im.SameLine()

  im.PushItemWidth(50)
  if rotateSnapComboItemCurrent[0] == -1 then
    for i, val in pairs(rotateSnapWidgetsComboItemsTbl) do
      if tonumber(val) == rotateSnapSize then rotateSnapComboItemCurrent[0] = i - 1 end
    end
  end
  if im.Combo1("##rotateSnapSize", rotateSnapComboItemCurrent, rotateSnapWidgetsComboItems) then
    local size = tonumber(rotateSnapWidgetsComboItemsTbl[rotateSnapComboItemCurrent[0] + 1])
    worldEditorCppApi.setRotateSnap(rotateSnapEnabled, size)
    editor.setPreference("snapping.general.rotateSnapSize", size)
  end
  if im.IsItemHovered() then im.SetTooltip("Rotate Snap Angle") end
  im.PopItemWidth()
  im.SameLine()

  if editor.getPreference("snapping.terrain.enabled") then bgColor = im.GetStyleColorVec4(im.Col_ButtonActive) else bgColor = nil end
  local windowPos = im.ImVec2(im.GetWindowPos().x + im.GetCursorPosX(), im.GetWindowPos().y + im.GetCursorPosY() + 30)
  if editor.uiIconImageButton(editor.icons.terrain_snap, nil, nil, nil, bgColor) then
    editor.setPreference("snapping.terrain.enabled", not editor.getPreference("snapping.terrain.enabled"))
  end
  if im.IsItemHovered() then im.SetTooltip("Toggle Terrain Snap. Right Click for options") end
  im.SameLine()

  if im.IsItemHovered() and im.IsMouseClicked(1) then
    terrainSnapSettingsOpen = not terrainSnapSettingsOpen
  end

  if terrainSnapSettingsOpen then
    im.SetNextWindowPos(windowPos, im.Cond_Appearing)
    local wndOpen = im.BoolPtr(terrainSnapSettingsOpen)
    im.Begin("Terrain Snap Settings", wndOpen, im.WindowFlags_NoCollapse)
    local relRotation = im.BoolPtr(editor.getPreference("snapping.terrain.relRotation"))
    if im.Checkbox("Keep relative rotation", relRotation) then
      editor.setPreference("snapping.terrain.relRotation", relRotation[0])
    end
    local individual = im.BoolPtr(editor.getPreference("snapping.terrain.indObjects"))
    if im.Checkbox("Treat objects individually", individual) then
      editor.setPreference("snapping.terrain.indObjects", individual[0])
    end
    local useRaycast = im.BoolPtr(editor.getPreference("snapping.terrain.useRayCast"))
    if im.Checkbox("Use raycast", useRaycast) then
      editor.setPreference("snapping.terrain.useRayCast", useRaycast[0])
    end

    local snapModeRadioValue
    if editor.getPreference("snapping.terrain.snapToCenter") then snapModeRadioValue = im.IntPtr(1)
    elseif editor.getPreference("snapping.terrain.snapToBB") then snapModeRadioValue = im.IntPtr(2)
    elseif editor.getPreference("snapping.terrain.keepHeight") then snapModeRadioValue = im.IntPtr(3)
    else snapModeRadioValue = im.IntPtr(0) end
    if im.RadioButton2("Snap to Origin", snapModeRadioValue, im.Int(0)) then
      editor.setPreference("snapping.terrain.snapToCenter", false)
      editor.setPreference("snapping.terrain.snapToBB", false)
      editor.setPreference("snapping.terrain.keepHeight", false)
    end
    if im.RadioButton2("Snap to Center", snapModeRadioValue, im.Int(1)) then
      editor.setPreference("snapping.terrain.snapToCenter", true)
      editor.setPreference("snapping.terrain.snapToBB", false)
      editor.setPreference("snapping.terrain.keepHeight", false)
    end
    if im.RadioButton2("Snap to Bounding Box", snapModeRadioValue, im.Int(2)) then
      editor.setPreference("snapping.terrain.snapToCenter", false)
      editor.setPreference("snapping.terrain.snapToBB", true)
      editor.setPreference("snapping.terrain.keepHeight", false)
    end
    if im.RadioButton2("Keep height", snapModeRadioValue, im.Int(3)) then
      editor.setPreference("snapping.terrain.snapToCenter", false)
      editor.setPreference("snapping.terrain.snapToBB", false)
      editor.setPreference("snapping.terrain.keepHeight", true)
    end
    im.End()
    if not wndOpen[0] then
      terrainSnapSettingsOpen = false
    end
  end

  if gridSnapSettingsOpen then
    im.SetNextWindowPos(windowPos, im.Cond_Appearing)
    local wndOpen = im.BoolPtr(gridSnapSettingsOpen)
    im.Begin("Grid Snap Settings", wndOpen, im.WindowFlags_NoCollapse)
    local useLastObject = im.BoolPtr(editor.getPreference("snapping.grid.useLastObjectSelected"))
    if im.Checkbox("Use the last object selected as the reference object", useLastObject) then
      editor.setPreference("snapping.grid.useLastObjectSelected", useLastObject[0])
    end
    im.End()
    if not wndOpen[0] then
      gridSnapSettingsOpen = false
    end
  end
end

local function cameraTodSliders()
  if not editor.disableModifyFadeIconsDistance then
    FadeIconsPtr[0] = editor.getPreference("gizmos.objectIcons.fadeIconsDistance")
    if editor.getPreference("gizmos.objectIcons.fadeIconsDistance") ~= FadeIconsPtr[0] then
      editor.setPreference("gizmos.objectIcons.fadeIconsDistance", FadeIconsPtr[0])
    end
  end
  im.PushItemWidth(80)
  im.TextUnformatted("Icons Distance")
  im.SameLine()
  if editor.uiSliderFloat("##Icons Distance", FadeIconsPtr, 100, 500, "%.0f") then
    editor.setPreference("gizmos.objectIcons.fadeIconsDistance", FadeIconsPtr[0])
  end
  im.tooltip("Adjust icons visibility (CTRL + Scroll Wheel)")
  im.SameLine()

  if not editor.keyModifiers.shift then
    camSpeedPtr[0] = core_camera.getSpeed()
    if editor.getPreference("camera.general.freeCameraMoveSpeed") ~= camSpeedPtr[0] then
      editor.setPreference("camera.general.freeCameraMoveSpeed", camSpeedPtr[0])
    end
  end
  im.PushItemWidth(80)
  im.TextUnformatted("Camera Speed")
  im.SameLine()
  if editor.uiSliderFloat("##Camera Speed", camSpeedPtr, 2, 100, "%.1f") then
    editor.setCameraSpeed(camSpeedPtr[0])
    editor.setPreference("camera.general.freeCameraMoveSpeed", camSpeedPtr[0])
  end
  im.tooltip("Adjust camera speed (Alt + Scroll Wheel)")
  im.SameLine()

  local tod = core_environment.getTimeOfDay()
  if tod then
    todPtr[0] = tod.time * 100
  else
    im.BeginDisabled()
  end
  im.PushItemWidth(80)
  im.TextUnformatted("Time of day")
  im.SameLine()
  if editor.uiSliderFloat("##Time of day", todPtr, 0, 100, "%.1f", 1) then
    tod.time = todPtr[0] / 100
    core_environment.setTimeOfDay(tod)
  end
  if not tod then
    im.EndDisabled()
  end
end

local function drawAlwaysVisibleToolbars()
  for key, val in pairs(editor.editModes) do
    if val["toolbarAlwaysVisible"] and val.onToolbar then
      val.onToolbar()
    end
  end
end

local function toolbarAlwaysVisibleModeExists()
  for key, val in pairs(editor.editModes) do
    if val["toolbarAlwaysVisible"] and val.onToolbar then
      return true
    end
  end
  return false
end

local function createDefaultList()
  local sortedKeys = {}
  for key, val in pairs(editor.editModes) do
    if key ~= "objectSelect" and key ~= "createObject" and val.icon then table.insert(sortedKeys, key) end
  end

  local function sorter(a, b)
    local ord1 = editor.editModes[a].sortOrder
    local ord2 = editor.editModes[b].sortOrder
    return ord1 < ord2
  end

  -- first sort by name
  table.sort(sortedKeys)

  -- assign a default sort order (index of editMode) to the ones that dont have one
  local idx = 1
  for _, key in pairs(sortedKeys) do
    if key ~= "objectSelect" and key ~= "createObject" then
      if nil == editor.editModes[key].sortOrder then
        editor.editModes[key].sortOrder = idx
      end
    end
    idx = idx + 1
  end
  -- second sort by sort order (if available)
  table.sort(sortedKeys, sorter)

  -- we always want object select to be first edit mode and object create right after it
  table.insert(sortedKeys, 1, "objectSelect")
  table.insert(sortedKeys, 2, "createObject")

  return sortedKeys
end

local function getModeKeyWithDisplayName(displayName)
  for key, val in pairs(editor.editModes) do
    if val.displayName == displayName then
      return key
    end
  end
  return nil
end

local function reorderEditModes()
  editor.selectEditMode(editModeReorderVars.DragMode)
  local sortedKeys = {}
  if tableIsEmpty(editModeReorderVars.ModeSortOrderKeysTbl) then
    sortedKeys = createDefaultList()
  else
    sortedKeys = editModeReorderVars.ModeSortOrderKeysTbl
  end

  local sortedKeysNewTbl = {}
  for _, key in ipairs(sortedKeys) do
    local val = editor.editModes[key]
    if val == nil then
      goto continue
    end
    if val.displayName == editModeReorderVars.DragMode.displayName and editModeReorderVars.DragModePutBefore.displayName ~= editModeReorderVars.DragMode.displayName then
      goto continue
    end

    if val.displayName == editModeReorderVars.DragModePutBefore.displayName and editModeReorderVars.DragModePutBefore.displayName ~= editModeReorderVars.DragMode.displayName then
      editModeReorderVars.DragMode.sortOrder = val.sortOrder
      table.insert(sortedKeysNewTbl, getModeKeyWithDisplayName(editModeReorderVars.DragMode.displayName))
      table.insert(sortedKeysNewTbl, getModeKeyWithDisplayName(editModeReorderVars.DragModePutBefore.displayName))
    else
      table.insert(sortedKeysNewTbl, getModeKeyWithDisplayName(val.displayName))
    end
    ::continue::
  end
  editModeReorderVars.ModeSortOrderKeysTbl = sortedKeysNewTbl
end

local function createManuallySortedList()
  return editModeReorderVars.ModeSortOrderKeysTbl
end

local function updateDataFile()
  local mainToolbarDataPath = "settings/editor/layouts/mainToolbarData.json"
  if FS:fileExists(mainToolbarDataPath) then
    FS:removeFile(mainToolbarDataPath)
  end
  jsonWriteFile(mainToolbarDataPath, editModeReorderVars.ModeSortOrderKeysTbl, true)
end

local function onEditorGuiToolBar()
  if editor.headless then return end
  -- no menu, dont show toolbars
  if not editor.menuHeight then return end

  im.PushStyleColor2(im.Col_Button, im.ImVec4(0,0,0,0))
  if editor.beginWindow(toolbarWindowName, "Main Toolbar", nil, true) then
    drawGeneralToolbarButtons()
    im.SameLine() im.Spacing()
    editor.uiVertSeparator(32, im.ImVec2(0,0))
    im.Spacing() im.SameLine()
    axisGizmoButtonsGui()
    im.SameLine() im.Spacing()
    editor.uiVertSeparator(32, im.ImVec2(0,0))
    im.Spacing()
    extensions.hook("onEditorGuiGeneralToolbar")
    local toolbarLastY = im.GetWindowHeight() + editor.menuHeight
    im.SameLine()
    local widthLeftToolBar = im.GetContentRegionAvailWidth()
    local iconButtonWidth = editor.getDefaultIconButtonSize().x
    local cameraTodSlidersWidth = 550 * (1+im.uiscale[0])/2
    local maxNumEditModesToShow = 0
    if widthLeftToolBar >= cameraTodSlidersWidth+iconButtonWidth then
      maxNumEditModesToShow = math.floor((widthLeftToolBar - cameraTodSlidersWidth) / (iconButtonWidth + editModeToolbarGap))
    end

    -- EDIT MODE BUTTONS
    if editor.editModes then
      im.SameLine()
      local sortedKeys = {}
      
      if not tableIsEmpty(editModeReorderVars.ModeSortOrderKeysTbl) then
        sortedKeys = createManuallySortedList()
      else
        sortedKeys = createDefaultList()
      end

      local numIconsLeftToShow = maxNumEditModesToShow
      local numEditModes = 0
      for _, key in ipairs(sortedKeys) do
        local val = editor.editModes[key]
        if val and val.icon then
          numEditModes = numEditModes + 1
        end
      end
      local isSelectedModeShownOnToolbar = false
      for _, key in ipairs(sortedKeys) do
        if numIconsLeftToShow <= 0 then
          break
        end
        local val = editor.editModes[key]
        if val and val.icon then
          numIconsLeftToShow = numIconsLeftToShow - 1
          if val == editor.editMode then
            isSelectedModeShownOnToolbar = true
          end
        end
      end

      numIconsLeftToShow = maxNumEditModesToShow
      local skippedEditMode = nil
      local isDragKeyModifiersActive = editor.keyModifiers.ctrl and editor.keyModifiers.shift
      for _, key in ipairs(sortedKeys) do
        local breakIndex = isSelectedModeShownOnToolbar and 0 or 1
        if numIconsLeftToShow <= breakIndex then
          skippedEditMode = editor.editModes[key]
          break
        end
        local val = editor.editModes[key]
        if val and val.icon then
          local bgColor = nil

          if editor.editMode == val then
            bgColor = im.GetStyleColorVec4(im.Col_ButtonActive)
          end
          numIconsLeftToShow = numIconsLeftToShow - 1

          local pos = im.GetCursorPos()
          if editModeReorderVars.DragModePutBefore and editModeReorderVars.DragModePutBefore.displayName == val.displayName then
            local uiScaling = editor.getPreference("ui.general.scale") or 1.0;
            local drawList = im.GetWindowDrawList()
            im.ImDrawList_AddRectFilled(drawList, im.ImVec2(im.GetWindowPos().x + pos.x + 1, pos.y + im.GetWindowPos().y + im.GetStyle().FramePadding.y*2), im.ImVec2(im.GetWindowPos().x + pos.x + 5, pos.y + im.GetWindowPos().y + editor.getDefaultIconButtonSize().y*uiScaling), im.GetColorU322(im.ImVec4(0.2, 0.6, 1.0, 1.0)))
            im.SetCursorPos(im.ImVec2(pos.x+6, pos.y))
          else
            im.SetCursorPos(im.ImVec2(pos.x+6, pos.y))
          end

          local pushedModeBtn = editor.uiIconImageButton(val.icon, nil, nil, nil, bgColor,nil, nil, nil, true)
          if isDragKeyModifiersActive and im.IsItemHovered() and im.IsMouseDown(0) and (im.GetIO().MouseDelta.x ~= 0 or im.GetIO().MouseDelta.y ~= 0) and 
            editModeReorderVars.DragMode == nil then
            editModeReorderVars.isDragging = true
            editModeReorderVars.DragMode = val
          end

          if editModeReorderVars.isDragging and im.IsItemHovered() then
            editModeReorderVars.DragModePutBefore = val
          end

          if not editModeReorderVars.isDragging then
            editModeReorderVars.DragModePutBefore = nil
          end

          if pushedModeBtn and val ~= editor.editMode then
            editor.selectEditMode(val)
          elseif pushedModeBtn then
            editor.selectEditMode(editor.editModes.objectSelect)
          end

          if im.IsMouseReleased(0) and editModeReorderVars.isDragging then
            reorderEditModes()
            --updateDataFile()
            editModeReorderVars.isDragging = false
            editor.selectEditMode(editModeReorderVars.DragMode)
            editModeReorderVars.DragMode = nil
          end

          if pushedModeBtn then
            for _, val in pairs(editor.editModes) do
              -- toolbar is always visible in create mode
              local isVisible = editor.editMode == editor.editModes.createObject and val == editor.editMode
              val["toolbarAlwaysVisible"] = isVisible
            end
          end

          if val.iconTooltip and im.IsItemHovered() then
            im.BeginTooltip()
            im.Text(val.iconTooltip)
            im.EndTooltip()
          end
          im.SameLine()
        end
      end

      --Add the selected edit mode as the last icon on the toolbar
      for _, key in ipairs(sortedKeys) do
        local val = editor.editModes[key]
        if val and val.icon and val == editor.editMode and not isSelectedModeShownOnToolbar then
          local bgColor = nil
          if editor.editMode == val then
            bgColor = im.GetStyleColorVec4(im.Col_ButtonActive)
          end
          numIconsLeftToShow = numIconsLeftToShow - 1
          local pushedModeBtn = editor.uiIconImageButton(val.icon, nil, nil, nil, bgColor, nil, nil ,nil, true)
          if isDragKeyModifiersActive and im.IsItemHovered() and im.IsMouseDown(0) and (im.GetIO().MouseDelta.x ~= 0 or im.GetIO().MouseDelta.y ~= 0) and 
            editModeReorderVars.DragMode == nil then
            editModeReorderVars.isDragging = true
            editModeReorderVars.DragMode = val
          end
          if editModeReorderVars.isDragging and im.IsItemHovered() then
            editModeReorderVars.DragModePutBefore = val
          end
          if not editModeReorderVars.isDragging then
            editModeReorderVars.DragModePutBefore = nil
          end

          if pushedModeBtn and val ~= editor.editMode and not editModeReorderVars.isDragging then
            editor.selectEditMode(val)
          elseif pushedModeBtn then
            editor.selectEditMode(editor.editModes.objectSelect)
          end

          if im.IsMouseReleased(0) and editModeReorderVars.isDragging then
            reorderEditModes()
            updateDataFile()
            editModeReorderVars.isDragging = false
            editModeReorderVars.DragMode = nil
          end
          if pushedModeBtn then
            for _, val in pairs(editor.editModes) do
              -- toolbar is always visible in create mode
              local isVisible = editor.editMode == editor.editModes.createObject and val == editor.editMode
              val["toolbarAlwaysVisible"] = isVisible
            end
          end

          if val.iconTooltip and im.IsItemHovered() then
            im.BeginTooltip()
            im.Text(val.iconTooltip)
            im.EndTooltip()
          end
          im.SameLine()
          break
        end
      end

      if maxNumEditModesToShow < numEditModes+2 then
          if editor.uiIconImageButton(editor.icons.fast_forward, im.ImVec2(iconButtonWidth/2.0, iconButtonWidth), nil, nil, nil, nil) then
            im.OpenPopup("EditModePopup")
          end
      end

      if im.BeginPopup("EditModePopup") then
        local index = 0
        local bgColor = nil
        local buttonWidthMax = 0
        if editor.editMode == skippedEditMode then
          bgColor = im.GetStyleColorVec4(im.Col_ButtonActive)
        end

        if skippedEditMode and skippedEditMode.icon and editor.editMode == skippedEditMode then
          im.SetCursorPosY(im.GetCursorPosY() + 5)
          editor.uiIconImage(skippedEditMode.icon, im.ImVec2(iconButtonWidth*0.9, iconButtonWidth*0.9), bgColor)
          im.SameLine()

          if im.Button(skippedEditMode.iconTooltip, im.ImVec2(120, iconButtonWidth)) and skippedEditMode ~= editor.editMode then
            editor.selectEditMode(skippedEditMode)
          end
        end

        local uiScaling = editor.getPreference("ui.general.scale") or 1.0;
        for _, key in ipairs(sortedKeys) do
          local val = editor.editModes[key]
          if index < maxNumEditModesToShow - (isSelectedModeShownOnToolbar and 0 or 1) then
            goto continue
          end
          if val and val.iconTooltip then
            local textSize = (im.CalcTextSize(val.iconTooltip).x)*uiScaling+im.GetStyle().FramePadding.x*2
            if textSize > buttonWidthMax then
              buttonWidthMax = textSize
            end
          end
          ::continue::
        end

        for _, key in ipairs(sortedKeys) do
          local val = editor.editModes[key]
          local bgColor = nil
          if editor.editMode == val then
            bgColor = im.GetStyleColorVec4(im.Col_ButtonActive)
          end
          if index < maxNumEditModesToShow - (isSelectedModeShownOnToolbar and 0 or 1) then
            goto continue
          end
          if val and val.icon then
            im.SetCursorPosY(im.GetCursorPosY() + 3)
            editor.uiIconImage(val.icon, im.ImVec2(iconButtonWidth*uiScaling*0.9, iconButtonWidth*uiScaling*0.9), bgColor)
            im.SameLine()

            im.SetCursorPosY(im.GetCursorPosY() + 3)
            --if im.Button(val.iconTooltip, im.ImVec2(textSize, iconButtonWidth*uiScaling), im.ButtonFlags_AlignLeft) and val ~= editor.editMode then
              --editor.selectEditMode(val)
            --end

            if im.Selectable1(val.iconTooltip, false, nil, im.ImVec2(buttonWidthMax, iconButtonWidth*uiScaling)) and val ~= editor.editMode then
              editor.selectEditMode(val)
            end
          end
          ::continue::
          index = index + 1
        end
        im.EndPopup()
      end

      im.SameLine() im.Spacing()
      editor.uiVertSeparator(32, im.ImVec2(0,0))

      extensions.hook("onEditorGuiEditModesToolbar")
      toolbarLastY = toolbarLastY + im.GetWindowHeight() + editModeToolbarGap
    end
    local noDisplay = false
    if editor.editMode and editor.getPreference("ui.general.singleLineToolbar") then
      if not editor.editMode.onToolbar and not toolbarAlwaysVisibleModeExists() then
        noDisplay = true
        goto finishWindow
      end

      editor.uiVertSeparator(editor.getPreference("ui.general.iconButtonSize"), im.ImVec2(0,0))
      drawAlwaysVisibleToolbars()
      --Draw Current Mode's toolbar if not already drawn.
      if not editor.editMode["toolbarAlwaysVisible"] and editor.editMode.onToolbar then
        editor.editMode.onToolbar()
      end
      extensions.hook("onEditorGuiEditModeToolbar")
      im.SameLine()
    end
    if im.GetContentRegionAvailWidth() > (550 * (1+im.uiscale[0])/2) then
      im.SetCursorPosX(im.GetCursorPosX() + im.GetContentRegionAvailWidth() - (550 * (1+im.uiscale[0])/2))
    end
    cameraTodSliders()
  end
  ::finishWindow::
  editor.endWindow()
  if noDisplay then
    goto safeFinish
  end

  if editor.editMode and not editor.getPreference("ui.general.singleLineToolbar") then
    if not editor.editMode.onToolbar and not toolbarAlwaysVisibleModeExists() then
      goto safeFinish
    end
    --TODO: replace with begin/endWindow
    im.Begin("Toolbar2", nil, toolbarFlags)
    drawAlwaysVisibleToolbars()
    --Draw Current Mode's toolbar if not already drawn.
    if not editor.editMode["toolbarAlwaysVisible"] and editor.editMode.onToolbar then
      editor.editMode.onToolbar()
    end
    extensions.hook("onEditorGuiEditModeToolbar")
    im.End()
  end
  ::safeFinish::
  im.PopStyleColor()
end

local function onEditorInitialized()
  editor.registerWindow(toolbarWindowName)
  editor.showWindow(toolbarWindowName)
end

local function onEditorPreferenceValueChanged(path, value)
  if path == "ui.general.enableVehicleControls" then
    for _, mapName in ipairs(vehicleActionMaps) do
      local map = scenetree.findObject(mapName)
      if map then
        map:setEnabled(editor.getPreference("ui.general.enableVehicleControls"))
      end
    end
  end
end

local function onEditorSaveState(state)
  state.mainToolbarData = editModeReorderVars.ModeSortOrderKeysTbl
end

local function onEditorLoadState(state)
  if state.mainToolbarData then
    editModeReorderVars.ModeSortOrderKeysTbl = state.mainToolbarData

    local keyExistsInPreloadedListFunc = function(key)
      local ret = false
      for _, preloadedKey in ipairs(editModeReorderVars.ModeSortOrderKeysTbl) do
        if preloadedKey == key then
          return true;
        end
      end
      return false;
    end

    for key, val in pairs(editor.editModes) do
      if not keyExistsInPreloadedListFunc(key) then
        table.insert(editModeReorderVars.ModeSortOrderKeysTbl, key)
      end
    end

  end
end

M.onEditorGuiToolBar = onEditorGuiToolBar
M.onEditorInitialized = onEditorInitialized
M.onEditorPreferenceValueChanged = onEditorPreferenceValueChanged
M.onEditorSaveState = onEditorSaveState
M.onEditorLoadState = onEditorLoadState

return M