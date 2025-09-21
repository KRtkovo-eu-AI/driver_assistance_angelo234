-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.dependencies = {'core_vehicle_manager'}

local ffi = require("ffi")
local im = ui_imgui

local _vEditor = M
rawset(_G, 'vEditor', _vEditor)

local imgui_true, imgui_false = im.BoolTrue(), im.BoolFalse()
local fpsSmoother = newExponentialSmoothing(50, 1)

local externalStaticApps = {}
local externalLiveApps = {{menuEntry = 'Mirror Debug', open = function () BeamNGVehicle.mirrorDebugEnabled = true end}}

local metrics = {}
local metricsTim = 0

local initialized = false

vEditor.editorActive = false

vEditor.EDITOR_MODE_STATIC = 1
vEditor.EDITOR_MODE_LIVE = 2

vEditor.EDITOR_NAMES = {"vehicleEditorStatic", "vehicleEditorLive"}

vEditor.editorMode = vEditor.EDITOR_MODE_LIVE

vEditor.staticMenuItems = {items = {}}
vEditor.liveMenuItems = {items = {}}

--[[
vEditor.MODE_DEFAULT = 1
vEditor.MODE_PICKING_NODE = 2
vEditor.MODE_PICKING_BEAM = 3

vEditor.mode = vEditor.MODE_DEFAULT
]]--

vEditor.selectedNodes = {}
vEditor.selectedBeams = {}

vEditor.veluaData = {}

vEditor.vehicle = nil
vEditor.vehiclePos = nil
vEditor.vehData = nil
vEditor.vdata = nil

vEditor.getEditorName = function()
  return vEditor.EDITOR_NAMES[vEditor.editorMode]
end

-- Play/stop button clicked
vEditor.setEditorMode = function(mode)
  --table.clear(vEditor.selectedNodes)
  --table.clear(vEditor.selectedBeams)

  -- disable old editor
  editor.enableHeadless(false, vEditor.getEditorName())
  vEditor.editorMode = mode

  -- enable new editor
  editor.enableHeadless(true, vEditor.getEditorName())
end

local function generateTable(menuItems, str, open)
  local temp = menuItems
  local list = {}
  -- Converts directory string into table of directories seperated
  -- e.g. "foo/bar/baz" -> {"foo", "bar", "baz"}
  for p in string.gmatch(str, "%s*%a+[%s%a+]*") do
    table.insert(list, p)
  end

  for k = 1, #list do
    local currGroup = list[k]

    if k < #list then
      local groupKey = nil

      for k,v in ipairs(temp.items) do
        if v.group == currGroup then
          groupKey = k
          break
        end
      end

      if not groupKey then
        groupKey = #temp.items + 1
        temp.items[groupKey] = {group = currGroup, items = {}}
      end

      local new = temp.items[groupKey]
      temp = new

    else
      temp.items[#temp.items + 1] = {menuEntry = currGroup, menuOpen = open}
    end
  end
end

local function loadAppsExtensions()
  local combinedExtNames = {}

  local extNames = {}
  local luaFiles = FS:findFiles("/lua/ge/extensions/editor/vehicleEditor/", "ve*.lua", 0, false, false)

  for _, file in ipairs(luaFiles) do
    local _, fn, _ = path.split(file)
    local name = string.sub(fn, 1, -5)
    if name ~= "veMain" then
      table.insert(extNames, "editor_vehicleEditor_" .. name)
    end
  end

  local staticExtNames = {}
  local staticLuaFiles = FS:findFiles("/lua/ge/extensions/editor/vehicleEditor/staticEditor/", "ve*.lua", 0, false, false)

  for _, file in ipairs(staticLuaFiles) do
    local _, fn, _ = path.split(file)
    local name = string.sub(fn, 1, -5)
    if name ~= "veMain" then
      table.insert(staticExtNames, "editor_vehicleEditor_staticEditor_" .. name)
    end
  end

  local liveExtNames = {}
  local liveLuaFiles = FS:findFiles("/lua/ge/extensions/editor/vehicleEditor/liveEditor/", "ve*.lua", 0, false, false)

  for _, file in ipairs(liveLuaFiles) do
    local _, fn, _ = path.split(file)
    local name = string.sub(fn, 1, -5)
    if name ~= "veMain" then
      table.insert(liveExtNames, "editor_vehicleEditor_liveEditor_" .. name)
    end
  end

  for k,v in ipairs(extNames) do
    table.insert(combinedExtNames, v)
  end
  for k,v in ipairs(staticExtNames) do
    table.insert(combinedExtNames, v)
  end
  for k,v in ipairs(liveExtNames) do
    table.insert(combinedExtNames, v)
  end

  editor.loadEditorExtensions(combinedExtNames)

  return extNames, staticExtNames, liveExtNames
end

local function createMenuItems(staticExtNames, liveExtNames, externalStaticIMGUIs, externalLiveIMGUIs)
  local entries = {}

  -- Create Static Editor Apps Menu
  for _, name in ipairs(staticExtNames) do
    local ext = extensions[name]

    if ext and ext.menuEntry and type(ext.open) == "function" and type(ext.menuEntry) == "string" then
      entries[ext.menuEntry] = ext.open
    end
  end
  for _, data in ipairs(externalStaticIMGUIs) do
    entries[data.menuEntry] = data.open
  end

  local sortedEntries = tableKeysSorted(entries)

  for _, extMenuEntry in ipairs(sortedEntries) do
    local extOpen = entries[extMenuEntry]
    generateTable(vEditor.staticMenuItems, extMenuEntry, extOpen)
  end

  -- Create Live Editor Apps Menu
  table.clear(entries)

  for _, name in ipairs(liveExtNames) do
    local ext = extensions[name]

    if ext and ext.menuEntry and type(ext.open) == "function" and type(ext.menuEntry) == "string" then
      entries[ext.menuEntry] = ext.open
    end
  end
  for _, data in ipairs(externalLiveIMGUIs) do
    entries[data.menuEntry] = data.open
  end

  sortedEntries = tableKeysSorted(entries)

  for _, extMenuEntry in ipairs(sortedEntries) do
    local extOpen = entries[extMenuEntry]
    generateTable(vEditor.liveMenuItems, extMenuEntry, extOpen)
  end
end

local function setupEditor()
  if not initialized then
    local extNames, staticExtNames, liveExtNames = loadAppsExtensions()
    createMenuItems(staticExtNames, liveExtNames, externalStaticApps, externalLiveApps)
    initialized = true
  end

  editor.selectEditMode(nil)
  editor.clearObjectSelection()
  editor.hideObjectIcons = true

  --popActionMap("EditorKeyMods")
  popActionMap("Editor")

  if vEditor.editorMode == vEditor.EDITOR_MODE_STATIC then
    pushActionMapHighestPriority("VehicleEditor")
  else
    popActionMap("VehicleEditor")
  end

  core_vehicle_manager.setDebug(true)
end

local function initVehicleData(id)
  local vehData = core_vehicle_manager.getVehicleData(id)
  vEditor.vehData = vehData
  vEditor.vdata = vehData and vehData.vdata or nil
  vEditor.nodeCIDToName = {}

  if vEditor.vdata and vEditor.vdata.nodes then
    for cid = 0, tableSizeC(vEditor.vdata.nodes) - 1 do
      local node = vEditor.vdata.nodes[cid]
      vEditor.nodeCIDToName[cid] = node.name
    end
  end

  vEditor.vehicle = getObjectByID(id)
  if vEditor.vehicle then
    vEditor.vehiclePos = vec3()
  end
end

-- Update data for other vehicle editors to use
local function onPreRender(dtReal, dtSim, dtRaw)
  if not vEditor.vehicle then
    initVehicleData(be:getPlayerVehicleID(0))
    return
  end

  if vEditor.vehicle then
    vEditor.vehiclePos:set(vEditor.vehicle:getPositionXYZ())
  end

  extensions.hook("onVehicleEditorRenderJBeams", dtReal, dtSim, dtRaw)
end

local function activateEditor()
  -- already active, return
  if vEditor.editorActive then return end

  -- initialize world editor to get access to its functions if not initialized yet
  editor.setEditorActive(true)

  -- intentionally calling onUpdate 2 times to initialize editor (because of loading popup deferred initialization)
  editor_main.onUpdate()
  editor_main.onUpdate()

  editor.enableHeadless(true, vEditor.getEditorName())
  setupEditor()
end

-- if goToWorldEditor == true, go back to world editor
-- if goToWorldEditor == false, escape vehicle editor
local function deactivateEditor(goToWorldEditor)
  if not vEditor.editorActive then return end

  -- disable headless mode
  editor.enableHeadless(false, vEditor.getEditorName())
  editor.hideObjectIcons = false

  popActionMap("VehicleEditor")
  --pushActionMapHighestPriority("EditorKeyMods")

  if goToWorldEditor then
    pushActionMapHighestPriority("Editor")
  else
    editor.setEditorActive(false)
    -- force call main update mainly for imgui layout state save
    editor_main.onUpdate()
  end

  core_vehicle_manager.setDebug(false)
end

local function toggleActive()
  if not vEditor.editorActive then
    activateEditor()
  else
    deactivateEditor(false)
  end
end

local function onEditorInitialized()
  editor.addWindowMenuItem("Vehicle Editor", activateEditor, {groupMenuName = 'Vehicles'})
end

local function onVehicleSwitched(oldVehicle, newVehicle, player)
  initVehicleData(newVehicle)
end

local function onEditorHeadlessChange(enabled, toolName)
  log('I', 'veMain', 'onEditorHeadlessChange(' .. tostring(enabled) .. ', "' .. tostring(toolName) .. '")')

  -- Setup editor in case it wasn't setup before using, e.g. reloading Lua
  if toolName == vEditor.getEditorName() then
    if enabled then
      setupEditor()
      vEditor.editorActive = true
    else
      popActionMap("VehicleEditor")
      vEditor.editorActive = false
    end
  end
end

local function onVehicleDestroyed()
  vEditor.vehicle = nil
  vEditor.vehData = nil
  vEditor.vdata = nil
  vEditor.nodeCIDToName = nil
  vEditor.vehiclePos = nil
end

local function onEditorInspectorFieldChanged(selectedIds, fieldName, fieldValue, arrayIndex)
  for i = 1, #selectedIds do
    local object = scenetree.findObjectById(selectedIds[i])
    if object and object:getClassName() == "BeamNGVehicle" then
      if fieldName == "color" or fieldName == "colorPalette0" or fieldName == "colorPalette1" then
        local color = core_vehicle_colors.colorStringToColorTable(fieldValue)
        color[4] = color[4]*2
        local paint = createVehiclePaint({x=color[1], y=color[2], z=color[3], w=color[4]}, {color[5], color[6], color[7], color[8]})
        core_vehicle_partmgmt.setConfigPaints(paint, false)
      end
    end
  end
end

local function sceneMetric()
  local io = im.GetIO()
  local fps = fpsSmoother:get(io.Framerate)

  local txtSize = im.CalcTextSize("FPS: 999 ").y + im.CalcTextSize("GpuWait: 00.0f ").y + im.CalcTextSize("Poly: 123456789").y
  im.SetCursorPosX(im.GetCursorPosX() + im.GetContentRegionAvailWidth() - txtSize*4)

  if fps < 30 then
    im.TextColored(im.ImVec4(1, 0.3, 0.3, 1), "FPS: %.0f", fps)
  elseif fps < 60 then
    im.TextColored(im.ImVec4(1, 1, 0.2, 1), "FPS: %.0f", fps)
  else
    im.Text("FPS: %.0f", fps)
  end

  if metricsTim < Engine.Platform.getRuntime() -0.5 then
    metricsTim = Engine.Platform.getRuntime()
    Engine.Debug.getLastPerformanceMetrics(metrics)
  end
  if metrics["FramePresent"] < 0.3 then
    im.Text("GpuWait: %.1f", metrics["FramePresent"])
  elseif metrics["FramePresent"] < 1 or fps > 30 then
    im.TextColored(im.ImVec4(1, 1, 0.2, 1), "GpuWait: %3.1f", metrics["FramePresent"])
  else
    im.TextColored(im.ImVec4(1, 0.3, 0.3, 1), "GpuWait: %3.1f", metrics["FramePresent"])
  end
  im.Text("Poly: "..getConsoleVariable("$GFXDeviceStatistics::polyCount"))
end

local function fileMenu()
  if im.BeginMenu("File", imgui_true) then
    if im.MenuItem1("Exit Vehicle Editor...", nil, imgui_false, imgui_true) then
      deactivateEditor(true)
    end
    im.EndMenu()
  end
end

-- this hook function is called by the editor when in headless mode to draw your own menubar
local function onEditorHeadlessMainMenuBar()
  if not editor.isHeadlessToolActive(vEditor.getEditorName()) then return end

  -- show our custom menu for the editor
  if im.BeginMainMenuBar() then
    fileMenu()

    sceneMetric()
    im.EndMainMenuBar()
  end
end

local function saveCurrentWindowLayout()
  editor_layoutManager.saveCurrentWindowLayout(vEditor.getEditorName())
end

local function onSerialize()
  local data = {}
  data.editorMode = vEditor.editorMode
  return data
end

local function onDeserialize(data)
  vEditor.editorMode = data.editorMode
end

-- Keybinding interfaces

M.onPreRender = onPreRender
M.onEditorHeadlessChange = onEditorHeadlessChange
M.onEditorInitialized = onEditorInitialized
M.onVehicleSwitched = onVehicleSwitched
M.onVehicleDestroyed = onVehicleDestroyed
M.onEditorInspectorFieldChanged = onEditorInspectorFieldChanged
M.onEditorHeadlessMainMenuBar = onEditorHeadlessMainMenuBar
M.saveCurrentWindowLayout = saveCurrentWindowLayout
M.onSerialize = onSerialize
M.onDeserialize = onDeserialize
M.toggleActive = toggleActive

return M
