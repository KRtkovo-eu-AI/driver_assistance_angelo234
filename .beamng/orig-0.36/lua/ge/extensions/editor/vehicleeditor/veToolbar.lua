-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.dependencies = {'core_environment'}
local imUtils = require('ui/imguiUtils')
local im = ui_imgui
local imgui_true, imgui_false = im.BoolTrue(), im.BoolFalse()
local toolbarWindowName = "veToolbar"
local toolbarFlags = bit.bor(im.WindowFlags_HorizontalScrollbar, im.WindowFlags_NoScrollWithMouse)
local innerToolbarFlags = bit.bor(im.WindowFlags_NoDecoration, im.WindowFlags_NoMove, im.WindowFlags_NoScrollWithMouse, im.WindowFlags_NoSavedSettings, im.WindowFlags_NoBringToFrontOnFocus, im.WindowFlags_NoBackground, im.WindowFlags_MenuBar)
local camSpeedPtr = im.FloatPtr(0)
local todPtr = im.FloatPtr(0)

local camSpeedSliderSize = 50
local todSliderSize = 80

local layoutName = im.ArrayChar(128)

local saveLayoutWindowName = "saveLayoutWindow"
local saveLayoutWindowTitle = "Save Layout"
local deleteLayoutWindowName = "deleteLayoutWindow"
local deleteLayoutWindowTitle = "Delete Layout"
local resetLayoutsWindowName = "resetLayoutsWindow"
local resetLayoutsWindowTitle = "Reset Layouts"

local vehsList = {}

local function createMenu(subItems)
  for _, item in pairs(subItems) do
    if item.group then
      if im.BeginMenu(item.group) then
        createMenu(item.items)
        im.EndMenu()
      end
    else
      if im.MenuItem1(item.menuEntry) then
        item.menuOpen()
      end
    end
  end
end

local function getCameraTodSlidersSize()
  return camSpeedSliderSize + todSliderSize + im.CalcTextSize(" Camera Speed  Time of day ").x
end

local function cameraTodSliders()
  if not editor.keyModifiers.shift then
    camSpeedPtr[0] = core_camera.getSpeed()
  end
  im.PushItemWidth(camSpeedSliderSize)
  if editor.uiSliderFloat("Camera Speed", camSpeedPtr, 2, 100, "%.1f") then
    editor.setCameraSpeed(camSpeedPtr[0])
  end
  im.SameLine()

  local tod = core_environment.getTimeOfDay()
  if tod then
    todPtr[0] = tod.time * 100
  else
    im.BeginDisabled()
  end
  im.PushItemWidth(todSliderSize)
  if editor.uiSliderFloat("Time of day", todPtr, 0, 100, "%.1f", 1) then
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

local function layoutsMenu()
  if im.BeginMenu("Layouts", imgui_true) then
    for _, layoutPath in ipairs(editor_layoutManager.getWindowLayouts(vEditor.getEditorName())) do
      if im.MenuItem1(string.match(layoutPath, ".+/(.+)"), nil, imgui_false, imgui_true) then
        editor_layoutManager.loadWindowLayout(layoutPath)
      end
    end

    im.Separator()
    if im.MenuItem1("Save Layout...", nil, imgui_false, imgui_true) then
      editor.showWindow(saveLayoutWindowName)
    end
    if im.MenuItem1("Delete Layout...", nil, imgui_false, imgui_true) then
      editor.showWindow(deleteLayoutWindowName)
    end
    if im.MenuItem1("Revert to Factory Settings...", nil, imgui_false, imgui_true) then
      editor.showWindow(resetLayoutsWindowName)
    end
    im.EndMenu()
  end
end

local function layoutsWindows()
  if editor.beginWindow(saveLayoutWindowName, saveLayoutWindowTitle) then
    im.PushItemWidth(im.GetContentRegionAvailWidth())
      if im.InputText("##SaveLayout", layoutName, 128, im.InputTextFlags_EnterReturnsTrue) then
        editor.hideWindow(saveLayoutWindowName)
        editor_layoutManager.saveWindowLayout(ffi.string(layoutName), vEditor.getEditorName())
      end
      if im.Button("Save") then
        editor.hideWindow(saveLayoutWindowName)
        editor_layoutManager.saveWindowLayout(ffi.string(layoutName), vEditor.getEditorName())
      end
  end
  editor.endWindow()

  if editor.beginWindow(deleteLayoutWindowName, deleteLayoutWindowTitle) then
    for _, layoutPath in ipairs(editor_layoutManager.getWindowLayouts(vEditor.getEditorName())) do
      if im.MenuItem1(string.match(layoutPath, ".+/(.+)"), nil, imgui_false, imgui_true) then
        editor_layoutManager.deleteWindowLayout(layoutPath)
      end
    end
  end
  editor.endWindow()

  if editor.beginWindow(resetLayoutsWindowName, resetLayoutsWindowTitle) then
    im.Text("This will delete all window layouts files and set the Default factory layout.")
    if im.Button("Continue") then
      editor.hideWindow(resetLayoutsWindowName)
      editor_layoutManager.resetLayouts(vEditor.getEditorName())
    end
    im.SameLine()
    if im.Button("Cancel") then
      editor.hideWindow(resetLayoutsWindowName)
    end
  end
  editor.endWindow()
end

local function staticEditorToolbar()
  local width = im.GetContentRegionAvail().x
  im.PushStyleColor2(im.Col_MenuBarBg, editor.color.transparent.Value)
  if im.BeginChild1("vehicleEditorSpecificEditorToolbar", im.ImVec2(width, im.GetFrameHeight()), true, innerToolbarFlags) then
    if im.BeginMenuBar() then
      if im.BeginMenu("Apps") then
        for _, item in ipairs(vEditor.staticMenuItems.items) do
          if item.group then
            if im.BeginMenu(item.group) then
              createMenu(item.items)
              im.EndMenu()
            end
          else
            if im.MenuItem1(item.menuEntry) then
              item.menuOpen()
            end
          end
        end
        im.EndMenu()
      end
      if im.BeginMenu("View") then
        layoutsMenu()
        if im.MenuItem1("Add View") then
          extensions.editor_vehicleEditor_staticEditor_veStaticRenderView.createRenderViewUI()
        end
        im.EndMenu()
      end
      im.EndMenuBar()
    end
    im.EndChild()
  end
  im.PopStyleColor()
end

local function liveEditorToolbar()
  local width = im.GetContentRegionAvail().x
  local camTodSize = getCameraTodSlidersSize()
  im.PushStyleColor2(im.Col_MenuBarBg, editor.color.transparent.Value)
  if im.BeginChild1("vehicleEditorSpecificEditorToolbar", im.ImVec2(width - camTodSize, im.GetFrameHeight()), true, innerToolbarFlags) then
    if im.BeginMenuBar() then
      if im.BeginMenu("Apps") then
        for _, item in ipairs(vEditor.liveMenuItems.items) do
          if item.group then
            if im.BeginMenu(item.group) then
              createMenu(item.items)
              im.EndMenu()
            end
          else
            if im.MenuItem1(item.menuEntry) then
              item.menuOpen()
            end
          end
        end
        im.EndMenu()
      end
      if im.BeginMenu("View") then
        layoutsMenu()
        if im.MenuItem1("Add View") then
          extensions.editor_vehicleEditor_liveEditor_veView.addSceneView()
        end
        im.EndMenu()
      end
      im.Separator()

      if im.BeginMenu("Vehicles") then
        if im.BeginMenu("Spawn") then
          for k, vehData in ipairs(vehsList) do
            if im.MenuItem1(vehData.model.key) then
              local spawnPos = core_camera.getPosition()
              local spawnRot = getCameraQuat()
              local veh = core_vehicles.spawnNewVehicle(vehData.model.key, {pos = spawnPos, rot = spawnRot})
              veh:queueLuaCommand("input.event('parkingbrake', 0, 1)")
            end
          end

          im.EndMenu()
        end
        if im.BeginMenu("Remove") then
          local currVehs = getAllVehicles()

          for k, veh in ipairs(currVehs) do
            local name = veh:getJBeamFilename() .. " (" .. veh:getID() .. ")"
            if im.MenuItem1(name) then
              veh:delete()
            end
          end

          im.EndMenu()
        end
        if im.MenuItem1("Remove All") then
          local currVehs = getAllVehicles()
          for k, veh in ipairs(currVehs) do
            veh:delete()
          end
        end
        im.EndMenu()
      end

      im.EndMenuBar()
    end
    im.EndChild()
  end
  im.PopStyleColor()

  im.SameLine()
  im.SetCursorPosX(im.GetCursorPosX() + im.GetContentRegionAvailWidth() - getCameraTodSlidersSize())
  cameraTodSliders()
end

local inited = false

local function onEditorGuiToolBar()
  -- no menu, dont show toolbars
  --if not editor.menuHeight then return end
  editor.menuHeight = 0

  im.PushStyleColor2(im.Col_Button, im.ImVec4(0,0,0,0))
  if editor.beginWindow(toolbarWindowName, toolbarWindowName, toolbarFlags, true) then
    if im.BeginTabBar("##tabs") then
      local staticEditorFlag, liveEditorFlag = 0, 0
      if not inited then
        if vEditor.editorMode == vEditor.EDITOR_MODE_STATIC then
          staticEditorFlag = im.TabItemFlags_SetSelected
        elseif vEditor.editorMode == vEditor.EDITOR_MODE_LIVE then
          liveEditorFlag = im.TabItemFlags_SetSelected
        end
        inited = true
      end
      if im.BeginTabItem("Static Editor", nil, staticEditorFlag) then
        -- Change editor mode on clicking tab
        if vEditor.editorMode ~= vEditor.EDITOR_MODE_STATIC then
          vEditor.setEditorMode(vEditor.EDITOR_MODE_STATIC)
        end

        staticEditorToolbar()
        im.EndTabItem()
      end
      if im.BeginTabItem("Live Editor", nil, liveEditorFlag) then
        -- Change editor mode on clicking tab
        if vEditor.editorMode ~= vEditor.EDITOR_MODE_LIVE then
          vEditor.setEditorMode(vEditor.EDITOR_MODE_LIVE)
        end

        liveEditorToolbar()
        im.EndTabItem()
      end
      im.EndTabBar()
    end

    --im.SameLine()

    --[[
    local regularColor = im.ImVec4(1,1,1,1)
    local selectedColor = im.GetStyleColorVec4(im.Col_ButtonActive)
    im.PushStyleColor2(im.Col_Text, vEditor.mode == vEditor.MODE_PICKING_NODE and selectedColor or regularColor)
    if im.Button("Pick Node") then
      vEditor.changeMode(vEditor.MODE_PICKING_NODE)
    end
    im.PopStyleColor()
    im.SameLine()
    im.PushStyleColor2(im.Col_Text, vEditor.mode == vEditor.MODE_PICKING_BEAM and selectedColor or regularColor)
    if im.Button("Pick Beam") then
      vEditor.changeMode(vEditor.MODE_PICKING_BEAM)
    end
    im.PopStyleColor()
    im.SameLine()
    ]]--
    layoutsWindows()
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
  table.clear(vehsList)

  local unsortedList = {}

  for k, vehItem in ipairs(core_vehicles.getVehicleList().vehicles) do
    if vehItem.model.Type == 'Car' or vehItem.model.Type == 'Truck' then
      unsortedList[vehItem.model.key] = vehItem
    end
  end

  local keysSorted = tableKeysSorted(unsortedList)

  for k,v in ipairs(keysSorted) do
    table.insert(vehsList, unsortedList[v])
  end

  editor.registerWindow(toolbarWindowName)
  editor.showWindow(toolbarWindowName)
end

M.onEditorGuiToolBar = onEditorGuiToolBar
M.onEditorInitialized = onEditorInitialized

return M