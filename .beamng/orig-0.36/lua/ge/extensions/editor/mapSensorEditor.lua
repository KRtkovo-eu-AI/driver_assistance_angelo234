-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}


-- External modules used.
local util = require('editor/tech/sensorConfiguration/utilities')                                   -- A utility class for the sensor configuration editors.


-- Module constants.
local im = ui_imgui
local min, max, floor, ceil = math.min, math.max, math.floor, math.ceil
local abs, sqrt, pow = math.abs, math.sqrt, math.pow

-- Module constants (UI).
local toolWinName, toolWinSize = 'MapSensorEditor', im.ImVec2(230, 240)                             -- The main tool window of the editor. The main UI entry point.
local sensorPropWinName, sensorPropWinSize = 'MapSensorPropertiesWindow', im.ImVec2(290, 880)       -- The per-sensor 'sensor properties' window..
local isSensorPropWinOpen = false                                                                   -- A flag which indicates if the sensor properties window is open or closed.
local tCamera, tLiDAR, tUltrasonic, tRADAR = 'camera', 'LiDAR', 'ultrasonic', 'RADAR'               -- String identifiers for each sensor Cycle through available lane types.
local ctrCamera, ctrLiDAR, ctrUltrasonic, ctrRADAR = 1, 1, 1, 1                                     -- Incrementable unique id counters for each sensor type.
local dullWhite = im.ImVec4(1, 1, 1, 0.5)                                                           -- Some commonly-used Imgui colour vectors.
local redB = im.ImVec4(0.7, 0.5, 0.5, 1)
local greenB, greenD = im.ImVec4(0.5, 0.7, 0.5, 1), im.ImVec4(0.5, 0.7, 0.5, 0.5)
local blueB, blueD = im.ImVec4(0.5, 0.5, 0.7, 1), im.ImVec4(0.5, 0.5, 0.7, 0.5)
local sensorIcon = im.ImVec2(32, 32)                                                                -- Some commonly-used Imgui icon size vectors.
local beginDragRotation = vec3(0, 0)

-- Module state (back-end).
local sensors = {}                                                                                  -- An ordered list of map-attached sensors.
local selectedSensorIdx = 1                                                                         -- The index of the selected sensor, in the attached sensors list.
local isMapSensorEditor = false                                                                     -- A flag which indicates if this editor is currently active.
local isPlaceMode = false                                                                           -- A flag which indicates if the editor is in 'place sensor' mode, or not.
local placing = nil                                                                                 -- The type of sensor being placed, when using 'place sensor' mode.

local pythonCodeLen = 4096
local pythonCodePtr = im.ArrayChar(pythonCodeLen)

-- The callback function for begin axis gizmo dragging.
local function gizmoBeginDrag()
  local s = sensors[selectedSensorIdx]
  beginDragRotation = quatFromDir(s.dir, s.up)
end

-- The callback function for end axis gizmo dragging.
local function gizmoEndDrag()
end

-- The callback function for continuing axis gizmo dragging.
local function gizmoDragging()
  local s = sensors[selectedSensorIdx]
  if editor.getAxisGizmoMode() == editor.AxisGizmoMode_Translate then                               -- Handle dragging on the translation gizmo.
    local pos = editor.getAxisGizmoTransform():getColumn(3)
    s.pos:set(pos.x, pos.y, pos.z)
    return
  end
  if editor.getAxisGizmoMode() == editor.AxisGizmoMode_Rotate then                                  -- Handle dragging on the rotational gizmo.
    local rotMat = editor.getAxisGizmoTransform()
    local q2 = QuatF(0, 0, 0, 1)
    q2:setFromMatrix(rotMat)
    if editor.getAxisGizmoAlignment() == editor.AxisGizmoAlignment_Local then
      local q = quat(q2)
      s.dir, s.up = q:toDirUp()
    else
      local q = beginDragRotation * quat(q2)
      s.dir, s.up = q:toDirUp()
    end
  end
end

-- Handles the gimbals for translational and rotational adjustment of sensor poses.
local function handleGimbals(pos)
  local rotation = nil
  if editor.getAxisGizmoAlignment() == editor.AxisGizmoAlignment_Local then
    local s = sensors[selectedSensorIdx]
    local q = quatFromDir(s.dir, s.up)
    rotation = QuatF(q.x, q.y, q.z, q.w)
  else
    rotation = QuatF(0, 0, 0, 1)
  end
  local transform = rotation:getMatrix()
  transform:setPosition(pos)
  editor.setAxisGizmoTransform(transform)
  editor.updateAxisGizmo(gizmoBeginDrag, gizmoEndDrag, gizmoDragging)
  editor.drawAxisGizmo()
end

-- Handles the finishing of placing a sensor.
local function handleFinishPlacingSensor(pos)
  if placing == tCamera then
    sensors[#sensors + 1] = {
      isLive = false,
      id = nil,
      name = 'Camera ' .. ctrCamera,
      type = tCamera,
      pos = pos,
      dir = vec3(1, 0, 0),
      up = vec3(0, 0, 1),
      size = { 200, 200 },
      fovY = 70,
      nearFarPlanes = { 0.05, 100.0 },
      updateTime = 0.05,
      updatePriority = 0.0,
      isRenderColours = true,
      isRenderAnnotations = true,
      isRenderInstance = false,
      isRenderDepth = true,
      isVisualised = true,
      isSnappingDesired = false }
    ctrCamera = ctrCamera + 1
  elseif placing == tLiDAR then
    sensors[#sensors + 1] = {
      isLive = false,
      id = nil,
      name = 'LiDAR ' .. ctrLiDAR,
      type = tLiDAR,
      pos = pos,
      dir = vec3(1, 0, 0),
      up = vec3(0, 0, 1),
      verticalResolution = 64,
      verticalAngle = 26.9,
      horizontalAngle = 120.0,
      frequency = 20.0,
      maxDistance = 120.0,
      isRotate = false, is360 = true,
      updateTime = 0.05,
      updatePriority = 0.0,
      isVisualised = true,
      isAnnotated = false,
      isSnappingDesired = false }
    ctrLiDAR = ctrLiDAR + 1
  elseif placing == tUltrasonic then
    sensors[#sensors + 1] = {
      isLive = false,
      id = nil,
      name = 'Ultrasonic ' .. ctrUltrasonic,
      type = tUltrasonic,
      pos = pos,
      dir = vec3(1, 0, 0),
      up = vec3(0, 0, 1),
      size = { 200, 200 },
      fovY = 70,
      nearFarPlanes = { 0.05, 5.1 },
      rangeRoundness = -1.15,
      rangeCutoffSensitivity = 0.0,
      rangeShape = 0.3,
      rangeFocus = 0.376,
      rangeMinCutoff = 0.1,
      rangeDirectMaxCutoff = 5.0,
      sensitivity = 3.0,
      fixedWindowSize = 10.0,
      updateTime = 0.05,
      updatePriority = 0.0,
      isVisualised = true,
      isSnappingDesired = false }
    ctrUltrasonic = ctrUltrasonic + 1
  elseif placing == tRADAR then
    sensors[#sensors + 1] = {
      isLive = false,
      id = nil,
      name = 'RADAR ' .. ctrRADAR,
      type = tRADAR,
      pos = pos,
      dir = vec3(1, 0, 0),
      up = vec3(0, 0, 1),
      size = { 200, 200 },
      fovY = 70,
      nearFarPlanes = { 0.05, 5.1 },
      rangeRoundness = -1.27,
      rangeCutoffSensitivity = 0.0,
      rangeShape = 0.09,
      rangeFocus = 0.37,
      rangeMinCutoff = 0.7,
      rangeDirectMaxCutoff = 300.0,
      rangeBins = 200,
      azimuthBins = 200,
      velBins = 200,
      rangeMin = 0.1,
      rangeMax = 100.0,
      halfAngleDeg = 30.0,
      velMin = -50.0,
      velMax = 50.0,
      updateTime = 0.05,
      updatePriority = 0.0,
      isVisualised = true,
      isSnappingDesired = false }
    ctrRADAR = ctrRADAR + 1
  end
  isPlaceMode, placing = false, nil
  selectedSensorIdx = #sensors
end

-- Handles the switching between 'edit mode' and 'live mode', for a given sensor.
local function handleEditLiveModeSwitch(idx)
  local s = sensors[idx]
  local t, sid = s.type, s.id
  if s.isLive then
    if t == tCamera then
      local args = {
        pos = s.pos, dir = s.dir, up = s.up,
        updateTime = s.updateTime, updatePriority = s.updatePriority,
        size = s.size, fovY = s.fovY, nearFarPlanes = s.nearFarPlanes,
        renderColours = s.isRenderColours,
        renderAnnotations = s.isRenderAnnotations,
        renderInstance = s.isRenderInstance,
        renderDepth = s.isRenderDepth,
        isVisualised = s.isVisualised, isStatic = true,
        isDirWorldSpace = true,
        isSnappingDesired = s.isSnappingDesired, isForceInsideTriangle = s.isSnappingDesired }
      s.id = extensions.tech_sensors.createCamera(-1, args)
    elseif t == tLiDAR then
      local args = {
        pos = s.pos, dir = s.dir, up = s.up,
        updateTime = s.updateTime, updatePriority = s.updatePriority,
        verticalResolution = s.verticalResolution, verticalAngle = s.verticalAngle,
        horizontalAngle = s.horizontalAngle, frequency = s.frequency,
        maxDistance = s.maxDistance,
        isRotate = s.isRotate, is360 = s.is360,
        isVisualised = s.isVisualised, isAnnotated = s.isAnnotated, isStatic = true,
        isDirWorldSpace = true,
        isSnappingDesired = s.isSnappingDesired, isForceInsideTriangle = s.isSnappingDesired }
      s.id = extensions.tech_sensors.createLidar(-1, args)
    elseif t == tUltrasonic then
      local args = {
        pos = s.pos, dir = s.dir, up = s.up,
        updateTime = s.updateTime, updatePriority = s.updatePriority,
        size = s.size, fovY = s.fovY, nearFarPlanes = s.nearFarPlanes,
        rangeRoundness = s.rangeRoundess, rangeCutoffSensitivity = s.rangeCutoffSensitivity,
        rangeShape = s.rangeShape, rangeFocus = s.rangeFocus,
        rangeMinCutoff = s.rangeMinCutoff, rangeDirectMaxCutoff = s.rangeDirectMaxCutoff,
        sensitivity = s.sensitivity, fixedWindowSize = s.fixedWindowSize,
        isVisualised = s.isVisualised, isStatic = true,
        isDirWorldSpace = true,
        isSnappingDesired = s.isSnappingDesired, isForceInsideTriangle = s.isSnappingDesired }
      s.id = extensions.tech_sensors.createUltrasonic(-1, args)
    elseif t == tRADAR then
      local args = {
        pos = s.pos, dir = s.dir, up = s.up,
        updateTime = s.updateTime, updatePriority = s.updatePriority,
        size = s.size, fovY = s.fovY, nearFarPlanes = s.nearFarPlanes,
        rangeRoundness = s.rangeRoundess, rangeCutoffSensitivity = s.rangeCutoffSensitivity,
        rangeShape = s.rangeShape, rangeFocus = s.rangeFocus,
        rangeMinCutoff = s.rangeMinCutoff, rangeDirectMaxCutoff = s.rangeDirectMaxCutoff,
        rangeBins = s.rangeBins, azimuthBins = s.azimuthBins, velBins = s.velBins,
        rangeMin = s.rangeMin, rangeMax = s.rangeMax,
        halfAngleDeg = s.halfAngleDeg,
        velMin = s.velMin, velMax = s.velMax,
        isVisualised = s.isVisualised, isStatic = true,
        isDirWorldSpace = true,
        isSnappingDesired = s.isSnappingDesired, isForceInsideTriangle = s.isSnappingDesired }
      s.id = extensions.tech_sensors.createRadar(-1, args)
    end
  else
    if sid then
      extensions.tech_sensors.removeSensor(sid)
    end
  end
end

-- Handles the placing of sensors.
-- [Note: not all sensors require placement].
local function handlePlaceSensor()
  local ray = getCameraMouseRay()
  local rayPos, rayDir = ray.pos, ray.dir
  local pInt = rayPos + rayDir * castRayStatic(rayPos, rayDir, 1000)
  util.drawMouseSphere(pInt)
  if im.IsMouseClicked(0) then
    handleFinishPlacingSensor(pInt)
  end
end

-- Removes the given sensor.
local function removeSensor(idx)
  sensors[idx].isLive = false
  handleEditLiveModeSwitch(idx)
  table.remove(sensors, idx)
  selectedSensorIdx = max(1, min(#sensors, selectedSensorIdx))
end

-- Saves the current sensor configuration to disk.
local function saveConfiguration()
  extensions.editor_fileDialog.saveFile(
    function(data)
      jsonWriteFile(data.filepath, { data = lpack.encode(sensors) }, true)
    end,
    {{"JSON",".json"}},
    false,
    "/",
    "File already exists.\nDo you want to overwrite the file?")
end

-- Loads a previously-saved sensor configuration from disk.
local function loadConfiguration()
  extensions.editor_fileDialog.openFile(
    function(data)
      for i = #sensors, 1, -1 do
        removeSensor(i)
      end
      local loadedJson = jsonReadFile(data.filepath)
      sensors = lpack.decode(loadedJson.data)
      for i = 1, #sensors do
        sensors[i].isLive = false
      end
    end,
    {{"JSON",".json"}},
    false,
    "/")
end

-- Manages the main tool window.
local function manageMainToolWindow()
  if editor.beginWindow(toolWinName, "Map Sensors", im.WindowFlags_NoTitleBar + im.WindowFlags_AlwaysAutoResize) then
    im.Separator()
    local numSensors = #sensors
    if im.BeginListBox("", im.ImVec2(222, 180), im.WindowFlags_ChildWindow) then
      for i = 1, numSensors do
        im.Columns(4, "attachedSensorsListBoxColumns", false)
        im.SetColumnWidth(0, 110)
        im.SetColumnWidth(1, 32)
        im.SetColumnWidth(2, 32)
        im.SetColumnWidth(3, 32)

        -- Handle the individual row selection.
        local sensor = sensors[i]
        if im.Selectable1(sensor.name, i == selectedSensorIdx, bit.bor(im.SelectableFlags_SpanAllColumns, im.SelectableFlags_AllowItemOverlap)) then
          if i ~= selectedSensorIdx then
            selectedSensorIdx = i
            if sensor.isLive and isSensorPropWinOpen then
              editor.hideWindow(sensorPropWinName)
              isSensorPropWinOpen = false
            end
          end
        end
        im.SameLine()
        im.NextColumn()

        -- 'Remove Sensor' button.
        if editor.uiIconImageButton(editor.icons.trashBin2, im.ImVec2(22, 22), redB, nil, nil, 'removeSensorButton') then
          selectedSensorIdx = min(numSensors, selectedSensorIdx)
          removeSensor(i)
          return
        end
        im.tooltip('Remove this sensor from the configuration.')
        im.SameLine()
        im.NextColumn()

        -- 'Edit Sensor' button.
        -- [This is only available if the sensor is not live].
        if not sensor.isLive then
          local btnCol = greenB
          if isSensorPropWinOpen and i == selectedSensorIdx then btnCol = greenD end
          if editor.uiIconImageButton(editor.icons.build, im.ImVec2(21, 21), btnCol, nil, nil, 'editSensorButton') then
            if i == selectedSensorIdx or not isSensorPropWinOpen then
              isSensorPropWinOpen = not isSensorPropWinOpen                                         -- Only toggle window open/closed if this is the same sensor.
            end
            if isSensorPropWinOpen then                                                             -- If window is open and this is a different sensor, just update the fields.
              editor.showWindow(sensorPropWinName)
            else
              editor.hideWindow(sensorPropWinName)
            end
            selectedSensorIdx = i
          end
          im.tooltip('Edit the selected sensor.')
        end
        im.SameLine()
        im.NextColumn()

        -- 'Live Sensor' toggle button.
        local btnCol, btnIcon = blueB, editor.icons.wifi
        if sensor.isLive then btnCol, btnIcon = blueD, editor.icons.wifi_lock end
        if editor.uiIconImageButton(btnIcon, im.ImVec2(19, 19), btnCol, nil, nil, 'toggleLiveSensorButton') then
          sensor.isLive = not sensor.isLive
          handleEditLiveModeSwitch(i)
          if isSensorPropWinOpen then
            editor.hideWindow(sensorPropWinName)
            isSensorPropWinOpen = false
          end
        end
        im.tooltip('Toggle between Edit and Live modes.')
        im.NextColumn()

        im.Separator()
      end
      im.EndListBox()
    end
    im.Separator()

    -- 'Add Camera Sensor' button.
    if editor.uiIconImageButton(editor.icons.survellianceCamera, sensorIcon, greenB, nil, nil, 'addNewCamera') then
      placing, isPlaceMode = tCamera, true
    end
    im.tooltip('Add a Camera Sensor to the configuration.')
    im.SameLine()

    -- 'Add LiDAR Sensor' button.
    if editor.uiIconImageButton(editor.icons.lidar, sensorIcon, greenB, nil, nil, 'addNewLiDAR') then
      placing, isPlaceMode = tLiDAR, true
    end
    im.tooltip('Add a LiDAR Sensor to the configuration.')
    im.SameLine()

    -- 'Add Ultrasonic Sensor' button.
    if editor.uiIconImageButton(editor.icons.proximitySensorsOutline, sensorIcon, greenB, nil, nil, 'addNewUltrasonic') then
      placing, isPlaceMode = tUltrasonic, true
    end
    im.tooltip('Add an Ultrasonic Sensor to the configuration.')
    im.SameLine()

    -- 'Add RADAR Sensor' button.
    if editor.uiIconImageButton(editor.icons.radar, sensorIcon, greenB, nil, nil, 'addNewRADAR') then
      placing, isPlaceMode = tRADAR, true
    end
    im.tooltip('Add a RADAR Sensor to the configuration.')
    im.SameLine()

    -- 'Save Sensor Configuration' button.
    if editor.uiIconImageButton(editor.icons.floppyDisk, sensorIcon, nil, nil, nil, 'saveSensorConfig') then
      saveConfiguration()
    end
    im.tooltip('Save the current sensor configuration, for this vehicle, to disk.')
    im.SameLine()

    -- 'Load Sensor Configuration' button.
    if editor.uiIconImageButton(editor.icons.folder, sensorIcon, dullWhite, nil, nil, 'loadSensorConfig') then
      loadConfiguration()
    end
    im.tooltip('Load a sensor configuration, for this vehicle, from disk.')

    -- 'Export to Python' button.
    im.Separator()
    if im.Button('Export Python code...') then
      extensions.tech_pythonExport.getFullConfig(nil, sensors, pythonCodePtr)
      im.OpenPopup("Python Code")
    end
    im.SameLine()

    if im.BeginPopupModal("Python Code", nil, im.WindowFlags_AlwaysAutoResize) then
      im.Text("You can use this code to recreate the configuration in BeamNGpy:")
      im.InputTextMultiline("##pythonCode", pythonCodePtr, pythonCodeLen, im.ImVec2(1200, 400))
      im.Separator()
      if im.Button("Copy to clipboard") then
        setClipboard(ffi.string(pythonCodePtr))
      end
      im.SameLine()
      if im.Button("Close popup") then
        im.CloseCurrentPopup()
      end
      im.EndPopup()
    end
  end
  editor.endWindow()
end

-- Manages the sensor properties Open/close the roads list window.
local function manageSensorPropWindow()
  if isSensorPropWinOpen then
    local sensor = sensors[selectedSensorIdx]
    if not sensor then
      editor.hideWindow(sensorPropWinName)
      isSensorPropWinOpen = false
      return
    end
    local ctr = 1
    if editor.beginWindow(sensorPropWinName, sensor.name .. " [Edit Properties]###241") then
      if sensor.type == tCamera then

        im.Separator()
        im.Dummy(im.ImVec2(5, 0))

        -- 'Position' input box.
        im.TextColored(greenB, 'Camera Position:')
        local oldVal = sensor.pos.x
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("[X-Axis] ###" .. tostring(ctr), uiVal, 0.01, nil, "%.4f m")
        im.tooltip('Set the sensor position on the X-Axis.')
        im.PopItemWidth()
        sensor.pos.x = uiVal[0]
        ctr = ctr + 1
        local oldVal = sensor.pos.y
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("[Y-Axis] ###" .. tostring(ctr), uiVal, 0.01, nil, "%.4f m")
        im.tooltip('Set the sensor position on the Y-Axis.')
        im.PopItemWidth()
        sensor.pos.y = uiVal[0]
        ctr = ctr + 1
        local oldVal = sensor.pos.z
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("[Z-Axis] ###" .. tostring(ctr), uiVal, 0.01, nil, "%.4f m")
        im.tooltip('Set the sensor position on the Z-Axis.')
        im.PopItemWidth()
        sensor.pos.z = uiVal[0]
        ctr = ctr + 1

        im.Dummy(im.ImVec2(5, 0))
        im.Separator()
        im.Dummy(im.ImVec2(5, 0))

        -- 'Size/Resolution' input box.
        im.TextColored(greenB, 'Camera Resolution:')
        local oldVal = sensor.size[1]
        local uiVal = im.IntPtr(oldVal)
        im.PushItemWidth(130)
        im.InputInt("Horizontal Resolution ###" .. tostring(ctr), uiVal, 10, nil)
        im.tooltip('Set the horizontal resolution of the sensor, in pixels.')
        im.PopItemWidth()
        sensor.size[1] = max(10, min(10000, uiVal[0]))
        ctr = ctr + 1
        local oldVal = sensor.size[2]
        local uiVal = im.IntPtr(oldVal)
        im.PushItemWidth(130)
        im.InputInt("Vertical Resolution ###" .. tostring(ctr), uiVal, 10, nil)
        im.tooltip('Set the vertical resolution of the sensor, in pixels.')
        im.PopItemWidth()
        sensor.size[2] = max(10, min(100000, uiVal[0]))
        ctr = ctr + 1

        im.Dummy(im.ImVec2(5, 0))
        im.Separator()
        im.Dummy(im.ImVec2(5, 0))

        -- 'Set Frustum Parameters' input box.
        im.TextColored(greenB, 'Camera Frustum:')
        local oldVal = sensor.fovY
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("Field Of View ###" .. tostring(ctr), uiVal, 1.0, nil, "%.2f deg")
        im.tooltip('Set the field of view of the sensor.')
        im.PopItemWidth()
        sensor.fovY = max(1.0, min(179.0, uiVal[0]))
        ctr = ctr + 1
        local oldVal = sensor.nearFarPlanes[1]
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("Near Plane Distance ###" .. tostring(ctr), uiVal, 1.0, nil, "%.2f m")
        im.tooltip('Set the near plane distance of the sensor (min depth cutoff).')
        im.PopItemWidth()
        sensor.nearFarPlanes[1] = max(0.01, min(sensor.nearFarPlanes[2] - 0.1, uiVal[0]))
        ctr = ctr + 1
        local oldVal = sensor.nearFarPlanes[2]
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("Far Plane Distance ###" .. tostring(ctr), uiVal, 1.0, nil, "%.2f m")
        im.tooltip('Set the far plane distance of the sensor (max depth cutoff).')
        im.PopItemWidth()
        sensor.nearFarPlanes[2] = max(sensor.nearFarPlanes[1] + 0.1, min(10000, uiVal[0]))
        ctr = ctr + 1

        im.Dummy(im.ImVec2(5, 0))
        im.Separator()
        im.Dummy(im.ImVec2(5, 0))

        -- 'Set Update Time' input box.
        im.TextColored(greenB, 'Camera Update Properties:')
        local oldVal = sensor.updateTime
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("Sensor Refresh Rate ###" .. tostring(ctr), uiVal, 0.01, nil, "%.4f s")
        im.tooltip('Set the time between sensor updates.')
        im.PopItemWidth()
        sensor.updateTime = max(0.0001, min(60.0, uiVal[0]))
        ctr = ctr + 1

        -- 'Set Update Priority' slider.
        local oldVal = sensor.updatePriority
        local uiVal = im.FloatPtr(oldVal)
        im.PushStyleVar1(im.StyleVar_GrabMinSize, 20)
        im.PushItemWidth(130)
        im.SliderFloat("Update Priority [0, 1]", uiVal, 0.0, 1.0, "%.3f")
        im.tooltip('The update priority of the sensor [0 = highest, 1 = lowest]. This is used for GPU scheduling')
        im.PopItemWidth()
        im.PopStyleVar()
        sensor.updatePriority = max(0.0, min(1.0, uiVal[0]))
        ctr = ctr + 1

        im.Dummy(im.ImVec2(5, 0))
        im.Separator()
        im.Dummy(im.ImVec2(5, 0))

        -- 'Is Visualised' checkbox.
        im.TextColored(greenB, 'Camera Operation Flags:')
        local oldVal = sensor.isRenderColours
        local uiVal = im.BoolPtr(oldVal)
        im.Checkbox("Render Color Image", uiVal)
        im.tooltip('Toggle whether to include the colour image in output.')
        sensor.isRenderColours = uiVal[0]
        local oldVal = sensor.isRenderAnnotations
        local uiVal = im.BoolPtr(oldVal)
        im.Checkbox("Render Class Annotations", uiVal)
        im.tooltip('Toggle whether to include class annotations (segmentation) in output.')
        sensor.isRenderAnnotations = uiVal[0]
        local oldVal = sensor.isRenderInstance
        local uiVal = im.BoolPtr(oldVal)
        im.Checkbox("Render Instance Annotations", uiVal)
        im.tooltip('Toggle whether to include instance annotations (segmentation) in output.')
        sensor.isRenderInstance = uiVal[0]
        local oldVal = sensor.isRenderDepth
        local uiVal = im.BoolPtr(oldVal)
        im.Checkbox("Render Depth Image", uiVal)
        im.tooltip('Toggle whether to include the depth image in output.')
        sensor.isRenderDepth = uiVal[0]
        local oldVal = sensor.isVisualised
        local uiVal = im.BoolPtr(oldVal)
        im.Checkbox("Visualise On Map", uiVal)
        im.tooltip('Toggle whether to visualise the sensor, or not.')
        sensor.isVisualised = uiVal[0]

        im.Dummy(im.ImVec2(5, 0))
        im.Separator()

      elseif sensor.type == tLiDAR then

        im.Separator()
        im.Dummy(im.ImVec2(5, 0))

        -- LiDAR Mode Selection buttons.
        local lMode, isRotate, is360 = 'Full360', sensor.isRotate, sensor.is360
        if is360 and not isRotate then
          lMode = 'Full360'
        elseif not is360 and isRotate then
          lMode = 'LFO'
        elseif not is360 and not isRotate then
          lMode = 'Static'
        end
        im.TextColored(greenB, 'LiDAR Operating Mode: [' .. lMode .. ']')
        local btnCol = blueB
        if lMode == 'Full360' then btnCol = blueD end
        if editor.uiIconImageButton(editor.icons.lidarPatternFullArcHighFreq, sensorIcon, btnCol, nil, nil, 'LiDAR360Mode') then
          sensor.is360, sensor.isRotate = true, false
        end
        im.tooltip("Set the LiDAR to operate in 'Full 360 Degrees' mode.")
        im.SameLine()
        local btnCol = blueB
        if lMode == 'LFO' then btnCol = blueD end
        if editor.uiIconImageButton(editor.icons.lidarPatternFullArcMidFreq, sensorIcon, btnCol, nil, nil, 'LiDARLFOMode') then
          sensor.is360, sensor.isRotate = false, true
        end
        im.tooltip("Set the LiDAR to operate in 'LFO' (low-frequency rotation) mode.")
        im.SameLine()
        local btnCol = blueB
        if lMode == 'Static' then btnCol = blueD end
        if editor.uiIconImageButton(editor.icons.lidarPatternNarrowArc, sensorIcon, btnCol, nil, nil, 'LiDARStaticMode') then
          sensor.is360, sensor.isRotate = false, false
        end
        im.tooltip("Set the LiDAR to operate in 'Static' (no rotation) mode.")
        im.SameLine()

        im.Dummy(im.ImVec2(5, 0))
        im.Separator()
        im.Dummy(im.ImVec2(5, 0))

        -- 'Position' input box.
        im.TextColored(greenB, 'LiDAR Position:')
        local oldVal = sensor.pos.x
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("[X-Axis] ###" .. tostring(ctr), uiVal, 0.01, nil, "%.4f m")
        im.tooltip('Set the sensor position on the X-Axis.')
        im.PopItemWidth()
        sensor.pos.x = uiVal[0]
        ctr = ctr + 1
        local oldVal = sensor.pos.y
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("[Y-Axis] ###" .. tostring(ctr), uiVal, 0.01, nil, "%.4f m")
        im.tooltip('Set the sensor position on the Y-Axis.')
        im.PopItemWidth()
        sensor.pos.y = uiVal[0]
        ctr = ctr + 1
        local oldVal = sensor.pos.z
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("[Z-Axis] ###" .. tostring(ctr), uiVal, 0.01, nil, "%.4f m")
        im.tooltip('Set the sensor position on the Z-Axis.')
        im.PopItemWidth()
        sensor.pos.z = uiVal[0]
        ctr = ctr + 1

        im.Dummy(im.ImVec2(5, 0))
        im.Separator()
        im.Dummy(im.ImVec2(5, 0))

        -- 'Core Properties' input boxes.
        im.TextColored(greenB, 'LiDAR Core Properties:')
        local oldVal = sensor.verticalResolution
        local uiVal = im.IntPtr(oldVal)
        im.PushItemWidth(130)
        im.InputInt("Vertical Resolution ###" .. tostring(ctr), uiVal, 1, nil)
        im.tooltip('Set the number of vertical resolution levels.')
        im.PopItemWidth()
        sensor.verticalResolution = max(1, min(1000, uiVal[0]))
        ctr = ctr + 1
        local oldVal = sensor.verticalAngle
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("Vertical Field Of View ###" .. tostring(ctr), uiVal, 1.0, nil, "%.2f deg")
        im.tooltip('Set the vertical field of view, in degrees.')
        im.PopItemWidth()
        sensor.verticalAngle = max(1.0, min(179.0, uiVal[0]))
        ctr = ctr + 1
        local oldVal = sensor.horizontalAngle
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("Horizontal Field Of View ###" .. tostring(ctr), uiVal, 1.0, nil, "%.2f deg")
        im.tooltip('Set the horizontal field of view, in degrees.')
        im.PopItemWidth()
        sensor.horizontalAngle = max(1.0, min(179.0, uiVal[0]))
        ctr = ctr + 1
        local oldVal = sensor.frequency
        local uiVal = im.IntPtr(oldVal)
        im.PushItemWidth(130)
        im.InputInt("Rotation Frequency (Hz) ###" .. tostring(ctr), uiVal, 1, nil)
        im.tooltip('Set the rotation frequency, in Hz.')
        im.PopItemWidth()
        sensor.frequency = max(1, min(20, uiVal[0]))
        ctr = ctr + 1
        local oldVal = sensor.maxDistance
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("Max Detection Range ###" .. tostring(ctr), uiVal, 1.0, nil, "%.2f m")
        im.tooltip('Set the sensor maximum detection range, in meters.')
        im.PopItemWidth()
        sensor.maxDistance = max(10.0, min(9999.0, uiVal[0]))
        ctr = ctr + 1

        im.Dummy(im.ImVec2(5, 0))
        im.Separator()
        im.Dummy(im.ImVec2(5, 0))

        -- 'Set Update Time' input box.
        im.TextColored(greenB, 'LiDAR Update Properties:')
        local oldVal = sensor.updateTime
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("Sensor Refresh Rate ###" .. tostring(ctr), uiVal, 0.01, nil, "%.4f s")
        im.tooltip('Set the time between sensor updates.')
        im.PopItemWidth()
        sensor.updateTime = max(0.0001, min(60.0, uiVal[0]))
        ctr = ctr + 1

        -- 'Set Update Priority' slider.
        local oldVal = sensor.updatePriority
        local uiVal = im.FloatPtr(oldVal)
        im.PushStyleVar1(im.StyleVar_GrabMinSize, 20)
        im.PushItemWidth(130)
        im.SliderFloat("Update Priority [0, 1]", uiVal, 0.0, 1.0, "%.3f")
        im.tooltip('The update priority of the sensor [0 = highest, 1 = lowest]. This is used for GPU scheduling')
        im.PopItemWidth()
        im.PopStyleVar()
        sensor.updatePriority = max(0.0, min(1.0, uiVal[0]))
        ctr = ctr + 1

        im.Dummy(im.ImVec2(5, 0))
        im.Separator()
        im.Dummy(im.ImVec2(5, 0))

        -- 'Is Annotated' checkbox.
        im.TextColored(greenB, 'LiDAR Operation Flags:')
        local oldVal = sensor.isAnnotated
        local uiVal = im.BoolPtr(oldVal)
        im.Checkbox("Include Segmentation Data", uiVal)
        im.tooltip('Toggle whether to include segmentation info (semantic annotations).')
        sensor.isAnnotated = uiVal[0]

        -- 'Is Visualised' checkbox.
        local oldVal = sensor.isVisualised
        local uiVal = im.BoolPtr(oldVal)
        im.Checkbox("Visualise On Map", uiVal)
        im.tooltip('Toggle whether to visualise the sensor, or not.')
        sensor.isVisualised = uiVal[0]

        im.Dummy(im.ImVec2(5, 0))
        im.Separator()

      elseif sensor.type == tUltrasonic then

        im.Separator()
        im.Dummy(im.ImVec2(5, 0))

        -- 'Position' input box.
        im.TextColored(greenB, 'Ultrasonic Position:')
        local oldVal = sensor.pos.x
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("[X-Axis] ###" .. tostring(ctr), uiVal, 0.01, nil, "%.4f m")
        im.tooltip('Set the sensor position on the X-Axis.')
        im.PopItemWidth()
        sensor.pos.x = uiVal[0]
        ctr = ctr + 1
        local oldVal = sensor.pos.y
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("[Y-Axis] ###" .. tostring(ctr), uiVal, 0.01, nil, "%.4f m")
        im.tooltip('Set the sensor position on the Y-Axis.')
        im.PopItemWidth()
        sensor.pos.y = uiVal[0]
        ctr = ctr + 1
        local oldVal = sensor.pos.z
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("[Z-Axis] ###" .. tostring(ctr), uiVal, 0.01, nil, "%.4f m")
        im.tooltip('Set the sensor position on the Z-Axis.')
        im.PopItemWidth()
        sensor.pos.z = uiVal[0]
        ctr = ctr + 1

        im.Dummy(im.ImVec2(5, 0))
        im.Separator()
        im.Dummy(im.ImVec2(5, 0))

        -- 'Size/Resolution' input box.
        im.TextColored(greenB, 'Ultrasonic Resolution:')
        local oldVal = sensor.size[1]
        local uiVal = im.IntPtr(oldVal)
        im.PushItemWidth(130)
        im.InputInt("Horizontal Resolution ###" .. tostring(ctr), uiVal, 10, nil)
        im.tooltip('Set the horizontal resolution of the sensor, in pixels.')
        im.PopItemWidth()
        sensor.size[1] = max(10, min(10000, uiVal[0]))
        ctr = ctr + 1
        local oldVal = sensor.size[2]
        local uiVal = im.IntPtr(oldVal)
        im.PushItemWidth(130)
        im.InputInt("Vertical Resolution ###" .. tostring(ctr), uiVal, 10, nil)
        im.tooltip('Set the vertical resolution of the sensor, in pixels.')
        im.PopItemWidth()
        sensor.size[2] = max(10, min(100000, uiVal[0]))
        ctr = ctr + 1

        im.Dummy(im.ImVec2(5, 0))
        im.Separator()
        im.Dummy(im.ImVec2(5, 0))

        -- 'Set Frustum Parameters' input box.
        im.TextColored(greenB, 'Ultrasonic Frustum:')
        local oldVal = sensor.fovY
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("Field Of View ###" .. tostring(ctr), uiVal, 1.0, nil, "%.2f deg")
        im.tooltip('Set the field of view of the sensor.')
        im.PopItemWidth()
        sensor.fovY = max(1.0, min(179.0, uiVal[0]))
        ctr = ctr + 1
        local oldVal = sensor.nearFarPlanes[1]
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("Near Plane Distance ###" .. tostring(ctr), uiVal, 1.0, nil, "%.2f m")
        im.tooltip('Set the near plane distance of the sensor (min depth cutoff).')
        im.PopItemWidth()
        sensor.nearFarPlanes[1] = max(0.01, min(sensor.nearFarPlanes[2] - 0.1, uiVal[0]))
        ctr = ctr + 1
        local oldVal = sensor.nearFarPlanes[2]
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("Far Plane Distance ###" .. tostring(ctr), uiVal, 1.0, nil, "%.2f m")
        im.tooltip('Set the far plane distance of the sensor (max depth cutoff).')
        im.PopItemWidth()
        sensor.nearFarPlanes[2] = max(sensor.nearFarPlanes[1] + 0.1, min(10000, uiVal[0]))
        ctr = ctr + 1

        im.Dummy(im.ImVec2(5, 0))
        im.Separator()
        im.Dummy(im.ImVec2(5, 0))

        -- 'Core Properties' input boxes.
        im.TextColored(greenB, 'Ultrasonic Beam Properties:')
        local oldVal = sensor.rangeRoundness
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("Range Roundness ###" .. tostring(ctr), uiVal, 0.1, nil, "%.4f")
        im.tooltip("Set the 'roundness' of the beam shape.")
        im.PopItemWidth()
        sensor.rangeRoundness = max(-100.0, min(100.0, uiVal[0]))
        ctr = ctr + 1
        local oldVal = sensor.rangeCutoffSensitivity
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("Range Cutoff Sensitivity ###" .. tostring(ctr), uiVal, 0.1, nil, "%.4f")
        im.tooltip('Set the sensitivity of the range cutoff.')
        im.PopItemWidth()
        sensor.rangeCutoffSensitivity = max(0.0, min(100.0, uiVal[0]))
        ctr = ctr + 1
        local oldVal = sensor.rangeShape
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("Range Shape ###" .. tostring(ctr), uiVal, 0.1, nil, "%.4f")
        im.tooltip('Set the shape across the beam range.')
        im.PopItemWidth()
        sensor.rangeShape = max(-100.0, min(100.0, uiVal[0]))
        ctr = ctr + 1
        local oldVal = sensor.rangeFocus
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("Range Focus ###" .. tostring(ctr), uiVal, 0.1, nil, "%.4f")
        im.tooltip('Set the sharpness of the shape across the beam range.')
        im.PopItemWidth()
        sensor.rangeFocus = max(-100.0, min(100.0, uiVal[0]))
        ctr = ctr + 1
        local oldVal = sensor.rangeMinCutoff
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("Range Min Cutoff ###" .. tostring(ctr), uiVal, 0.1, nil, "%.4f m")
        im.tooltip('Set the near plane, in meters.')
        im.PopItemWidth()
        sensor.rangeMinCutoff = max(0.1, min(20.0, uiVal[0]))
        ctr = ctr + 1
        local oldVal = sensor.rangeDirectMaxCutoff
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("Range Direct Max Cutoff ###" .. tostring(ctr), uiVal, 0.1, nil, "%.4f m")
        im.tooltip('Set the far plane, in meters.')
        im.PopItemWidth()
        sensor.rangeDirectMaxCutoff = max(0.1, min(20.0, uiVal[0]))
        ctr = ctr + 1
        if editor.uiIconImageButton(editor.icons.triangleBeam, sensorIcon, blueB, nil, nil, 'usPreset1') then
          sensor.rangeRoundness = -2
          sensor.rangeCutoffSensitivity = 0.0004
          sensor.rangeShape = 0.0
          sensor.rangeFocus = 0.344
          sensor.rangeMinCutoff = 0.1
          sensor.rangeDirectMaxCutoff = 9.9
          sensor.nearFarPlanes = { 0.1, 11.0 }
        end
        im.tooltip('Set preset beam shape: Triangular, 7m range.')
        im.SameLine()
        if editor.uiIconImageButton(editor.icons.thinBulbBeam, sensorIcon, blueB, nil, nil, 'usPreset2') then
          sensor.rangeRoundness = -2
          sensor.rangeCutoffSensitivity = 0
          sensor.rangeShape = 0.28
          sensor.rangeFocus = 0.315
          sensor.rangeMinCutoff = 0.1
          sensor.rangeDirectMaxCutoff = 9.9
          sensor.nearFarPlanes = { 0.1, 11.0 }
        end
        im.tooltip('Set preset beam shape: Thin bulb, 8m range.')
        im.SameLine()
        if editor.uiIconImageButton(editor.icons.thinnerBulbBeam, sensorIcon, blueB, nil, nil, 'usPreset3') then
          sensor.rangeRoundness = -2
          sensor.rangeCutoffSensitivity = 0
          sensor.rangeShape = 0.09
          sensor.rangeFocus = 0.686
          sensor.rangeMinCutoff = 0.1
          sensor.rangeDirectMaxCutoff = 10.0
          sensor.nearFarPlanes = { 0.1, 11.0 }
        end
        im.tooltip('Set preset beam shape: Ultra thin bulb, 8m range.')
        im.SameLine()
        if editor.uiIconImageButton(editor.icons.sphericalBeam, sensorIcon, blueB, nil, nil, 'usPreset4') then
          sensor.rangeRoundness = 0.7
          sensor.rangeCutoffSensitivity = 0.0001
          sensor.rangeShape = 0.09
          sensor.rangeFocus = 0.744
          sensor.rangeMinCutoff = 0.1
          sensor.rangeDirectMaxCutoff = 5.2
          sensor.nearFarPlanes = { 0.1, 6.0 }
        end
        im.tooltip('Set preset beam shape: Spherical, 5m range.')
        im.SameLine()
        if editor.uiIconImageButton(editor.icons.flatBulbBeam, sensorIcon, blueB, nil, nil, 'usPreset5') then
          sensor.rangeRoundness = -0.01
          sensor.rangeCutoffSensitivity = 0.002
          sensor.rangeShape = 0.33
          sensor.rangeFocus = 0.344
          sensor.rangeMinCutoff = 0.1
          sensor.rangeDirectMaxCutoff = 9.9
          sensor.nearFarPlanes = { 0.1, 11.0 }
        end
        im.tooltip('Set preset beam shape: Flat head bulb, 5m range.')
        im.SameLine()
        if editor.uiIconImageButton(editor.icons.tulipBeam, sensorIcon, blueB, nil, nil, 'usPreset6') then
          sensor.rangeRoundness = -1.18
          sensor.rangeCutoffSensitivity = 0
          sensor.rangeShape = 0.14
          sensor.rangeFocus = 0.52
          sensor.rangeMinCutoff = 0.1
          sensor.rangeDirectMaxCutoff = 10.1
          sensor.nearFarPlanes = { 0.1, 11.0 }
        end
        im.tooltip('Set preset beam shape: Tulip, 10m (long) range.')

        im.Dummy(im.ImVec2(5, 0))
        im.Separator()
        im.Dummy(im.ImVec2(5, 0))

        -- 'Core Properties' input boxes.
        im.TextColored(greenB, 'Ultrasonic Detection Properties:')
        local oldVal = sensor.sensitivity
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("Sensitivity ###" .. tostring(ctr), uiVal, 1.0, nil, "%.2f")
        im.tooltip("Set the detection sensitivity.")
        im.PopItemWidth()
        sensor.sensitivity = max(0.1, min(100.0, uiVal[0]))
        ctr = ctr + 1
        local oldVal = sensor.fixedWindowSize
        local uiVal = im.IntPtr(oldVal)
        im.PushItemWidth(130)
        im.InputInt("Window Width ###" .. tostring(ctr), uiVal, 1, nil)
        im.tooltip('Set the width of the window used in processing the returns.')
        im.PopItemWidth()
        sensor.fixedWindowSize = max(1, min(10000, uiVal[0]))
        ctr = ctr + 1

        im.Dummy(im.ImVec2(5, 0))
        im.Separator()
        im.Dummy(im.ImVec2(5, 0))

        -- 'Set Update Time' input box.
        im.TextColored(greenB, 'Ultrasonic Update Properties:')
        local oldVal = sensor.updateTime
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("Sensor Refresh Rate ###" .. tostring(ctr), uiVal, 0.01, nil, "%.4f s")
        im.tooltip('Set the time between sensor updates.')
        im.PopItemWidth()
        sensor.updateTime = max(0.0001, min(60.0, uiVal[0]))
        ctr = ctr + 1

        -- 'Set Update Priority' slider.
        local oldVal = sensor.updatePriority
        local uiVal = im.FloatPtr(oldVal)
        im.PushStyleVar1(im.StyleVar_GrabMinSize, 20)
        im.PushItemWidth(130)
        im.SliderFloat("Update Priority [0, 1]", uiVal, 0.0, 1.0, "%.3f")
        im.tooltip('The update priority of the sensor [0 = highest, 1 = lowest]. This is used for GPU scheduling')
        im.PopItemWidth()
        im.PopStyleVar()
        sensor.updatePriority = max(0.0, min(1.0, uiVal[0]))
        ctr = ctr + 1

        im.Dummy(im.ImVec2(5, 0))
        im.Separator()
        im.Dummy(im.ImVec2(5, 0))

        -- 'Is Visualised' checkbox.
        im.TextColored(greenB, 'Ultrasonic Operation Flags:')
        local oldVal = sensor.isVisualised
        local uiVal = im.BoolPtr(oldVal)
        im.Checkbox("Visualise On Map", uiVal)
        im.tooltip('Toggle whether to visualise the sensor, or not.')
        sensor.isVisualised = uiVal[0]

        im.Dummy(im.ImVec2(5, 0))
        im.Separator()

      elseif sensor.type == tRADAR then

        im.Separator()
        im.Dummy(im.ImVec2(5, 0))

        -- 'Position' input box.
        im.TextColored(greenB, 'RADAR Position:')
        local oldVal = sensor.pos.x
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("[X-Axis] ###" .. tostring(ctr), uiVal, 0.01, nil, "%.4f m")
        im.tooltip('Set the sensor position on the X-Axis.')
        im.PopItemWidth()
        sensor.pos.x = uiVal[0]
        ctr = ctr + 1
        local oldVal = sensor.pos.y
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("[Y-Axis] ###" .. tostring(ctr), uiVal, 0.01, nil, "%.4f m")
        im.tooltip('Set the sensor position on the Y-Axis.')
        im.PopItemWidth()
        sensor.pos.y = uiVal[0]
        ctr = ctr + 1
        local oldVal = sensor.pos.z
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("[Z-Axis] ###" .. tostring(ctr), uiVal, 0.01, nil, "%.4f m")
        im.tooltip('Set the sensor position on the Z-Axis.')
        im.PopItemWidth()
        sensor.pos.z = uiVal[0]
        ctr = ctr + 1

        im.Dummy(im.ImVec2(5, 0))
        im.Separator()
        im.Dummy(im.ImVec2(5, 0))

        -- 'Size/Resolution' input box.
        im.TextColored(greenB, 'RADAR Resolution:')
        local oldVal = sensor.size[1]
        local uiVal = im.IntPtr(oldVal)
        im.PushItemWidth(130)
        im.InputInt("Horizontal Resolution ###" .. tostring(ctr), uiVal, 10, nil)
        im.tooltip('Set the horizontal resolution of the sensor, in pixels.')
        im.PopItemWidth()
        sensor.size[1] = max(10, min(10000, uiVal[0]))
        ctr = ctr + 1
        local oldVal = sensor.size[2]
        local uiVal = im.IntPtr(oldVal)
        im.PushItemWidth(130)
        im.InputInt("Vertical Resolution ###" .. tostring(ctr), uiVal, 10, nil)
        im.tooltip('Set the vertical resolution of the sensor, in pixels.')
        im.PopItemWidth()
        sensor.size[2] = max(10, min(100000, uiVal[0]))
        ctr = ctr + 1

        im.Dummy(im.ImVec2(5, 0))
        im.Separator()
        im.Dummy(im.ImVec2(5, 0))

        -- 'Set Frustum Parameters' input box.
        im.TextColored(greenB, 'RADAR Frustum:')
        local oldVal = sensor.fovY
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("Field Of View ###" .. tostring(ctr), uiVal, 1.0, nil, "%.2f deg")
        im.tooltip('Set the field of view of the sensor.')
        im.PopItemWidth()
        sensor.fovY = max(1.0, min(179.0, uiVal[0]))
        ctr = ctr + 1
        local oldVal = sensor.nearFarPlanes[1]
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("Near Plane Distance ###" .. tostring(ctr), uiVal, 1.0, nil, "%.2f m")
        im.tooltip('Set the near plane distance of the sensor (min depth cutoff).')
        im.PopItemWidth()
        sensor.nearFarPlanes[1] = max(0.01, min(sensor.nearFarPlanes[2] - 0.1, uiVal[0]))
        ctr = ctr + 1
        local oldVal = sensor.nearFarPlanes[2]
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("Far Plane Distance ###" .. tostring(ctr), uiVal, 1.0, nil, "%.2f m")
        im.tooltip('Set the far plane distance of the sensor (max depth cutoff).')
        im.PopItemWidth()
        sensor.nearFarPlanes[2] = max(sensor.nearFarPlanes[1] + 0.1, min(10000, uiVal[0]))
        ctr = ctr + 1

        im.Dummy(im.ImVec2(5, 0))
        im.Separator()
        im.Dummy(im.ImVec2(5, 0))

        -- 'Core Properties' input boxes.
        im.TextColored(greenB, 'RADAR Beam Properties:')
        local oldVal = sensor.rangeRoundness
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("Range Roundness ###" .. tostring(ctr), uiVal, 0.1, nil, "%.4f")
        im.tooltip("Set the 'roundness' of the beam shape.")
        im.PopItemWidth()
        sensor.rangeRoundness = max(-100.0, min(100.0, uiVal[0]))
        ctr = ctr + 1
        local oldVal = sensor.rangeCutoffSensitivity
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("Range Cutoff Sensitivity ###" .. tostring(ctr), uiVal, 0.1, nil, "%.4f")
        im.tooltip('Set the sensitivity of the range cutoff.')
        im.PopItemWidth()
        sensor.rangeCutoffSensitivity = max(0.0, min(100.0, uiVal[0]))
        ctr = ctr + 1
        local oldVal = sensor.rangeShape
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("Range Shape ###" .. tostring(ctr), uiVal, 0.1, nil, "%.4f")
        im.tooltip('Set the shape across the beam range.')
        im.PopItemWidth()
        sensor.rangeShape = max(-100.0, min(100.0, uiVal[0]))
        ctr = ctr + 1
        local oldVal = sensor.rangeFocus
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("Range Focus ###" .. tostring(ctr), uiVal, 0.1, nil, "%.4f")
        im.tooltip('Set the sharpness of the shape across the beam range.')
        im.PopItemWidth()
        sensor.rangeFocus = max(-100.0, min(100.0, uiVal[0]))
        ctr = ctr + 1
        local oldVal = sensor.rangeMinCutoff
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("Range Min Cutoff ###" .. tostring(ctr), uiVal, 0.1, nil, "%.4f m")
        im.tooltip('Set the near plane, in meters.')
        im.PopItemWidth()
        sensor.rangeMinCutoff = max(0.1, min(1000.0, uiVal[0]))
        ctr = ctr + 1
        local oldVal = sensor.rangeDirectMaxCutoff
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("Range Direct Max Cutoff ###" .. tostring(ctr), uiVal, 1.0, nil, "%.4f m")
        im.tooltip('Set the far plane, in meters.')
        im.PopItemWidth()
        sensor.rangeDirectMaxCutoff = max(0.1, min(1000.0, uiVal[0]))
        ctr = ctr + 1
        if editor.uiIconImageButton(editor.icons.shortRangeBeam1, sensorIcon, blueB, nil, nil, 'usPreset1') then
          sensor.rangeRoundness = -1.27
          sensor.rangeCutoffSensitivity = 0.0
          sensor.rangeShape = 0.09
          sensor.rangeFocus = 0.54
          sensor.rangeMinCutoff = 0.5
          sensor.rangeDirectMaxCutoff = 30.0
          sensor.nearFarPlanes = { 0.5, 30.0 }
        end
        im.tooltip('Set preset beam shape: Short Range (0.5m - 30m).')
        im.SameLine()
        if editor.uiIconImageButton(editor.icons.midRangeBeam1, sensorIcon, blueB, nil, nil, 'usPreset2') then
          sensor.rangeRoundness = -1.27
          sensor.rangeCutoffSensitivity = 0
          sensor.rangeShape = 0.09
          sensor.rangeFocus = 0.466
          sensor.rangeMinCutoff = 1.0
          sensor.rangeDirectMaxCutoff = 70.0
          sensor.nearFarPlanes = { 1.0, 70.0 }
        end
        im.tooltip('Set preset beam shape: Medium Range (1m - 70m).')
        im.SameLine()
        if editor.uiIconImageButton(editor.icons.longRangeBeam1, sensorIcon, blueB, nil, nil, 'usPreset3') then
          sensor.rangeRoundness = -1.27
          sensor.rangeCutoffSensitivity = 0
          sensor.rangeShape = 0.09
          sensor.rangeFocus = 0.37
          sensor.rangeMinCutoff = 10.0
          sensor.rangeDirectMaxCutoff = 300.0
          sensor.nearFarPlanes = { 10.0, 300.0 }
        end
        im.tooltip('Set preset beam shape: Long Range (10m - 250m).')

        im.Dummy(im.ImVec2(5, 0))
        im.Separator()
        im.Dummy(im.ImVec2(5, 0))

        -- 'Core Properties' input boxes.
        im.TextColored(greenB, 'RADAR Post-Processing Properties:')
        local oldVal = sensor.rangeBins
        local uiVal = im.IntPtr(oldVal)
        im.PushItemWidth(130)
        im.InputInt("Range Bins ###" .. tostring(ctr), uiVal, 1, nil)
        im.tooltip('Set the number of bins to divide the data into, in the range dimension.')
        im.PopItemWidth()
        sensor.rangeBins = max(10, min(1000, uiVal[0]))
        ctr = ctr + 1
        local oldVal = sensor.azimuthBins
        local uiVal = im.IntPtr(oldVal)
        im.PushItemWidth(130)
        im.InputInt("Azimuth Bins ###" .. tostring(ctr), uiVal, 1, nil)
        im.tooltip('Set the number of bins to divide the data into, in the azimuth dimension.')
        im.PopItemWidth()
        sensor.azimuthBins = max(10, min(1000, uiVal[0]))
        ctr = ctr + 1
        local oldVal = sensor.velBins
        local uiVal = im.IntPtr(oldVal)
        im.PushItemWidth(130)
        im.InputInt("Velocity Bins ###" .. tostring(ctr), uiVal, 1, nil)
        im.tooltip('Set the number of bins to divide the data into, in the velocity dimension.')
        im.PopItemWidth()
        sensor.velBins = max(10, min(1000, uiVal[0]))
        ctr = ctr + 1
        local oldVal = sensor.rangeMin
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("Min Range ###" .. tostring(ctr), uiVal, 1, nil, "%.2f")
        im.tooltip("Set the minimum range to display in the data.")
        im.PopItemWidth()
        sensor.rangeMin = max(0.1, min(1000.0, uiVal[0]))
        ctr = ctr + 1
        local oldVal = sensor.rangeMax
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("Max Range ###" .. tostring(ctr), uiVal, 1.0, nil, "%.2f")
        im.tooltip("Set the maximum range to display in the data.")
        im.PopItemWidth()
        sensor.rangeMax = max(0.1, min(1000.0, uiVal[0]))
        ctr = ctr + 1
        local oldVal = sensor.velMin
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("Min Velocity ###" .. tostring(ctr), uiVal, 1.0, nil, "%.2f")
        im.tooltip("Set the minimum velocity to display in the data.")
        im.PopItemWidth()
        sensor.velMin = max(-250.0, min(250.0, uiVal[0]))
        ctr = ctr + 1
        local oldVal = sensor.velMax
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("Max Velocity ###" .. tostring(ctr), uiVal, 1.0, nil, "%.2f")
        im.tooltip("Set the maximum velocity to display in the data.")
        im.PopItemWidth()
        sensor.velMax = max(-250.0, min(250.0, uiVal[0]))
        ctr = ctr + 1
        local oldVal = sensor.halfAngleDeg
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("Azimuth Half-Angle ###" .. tostring(ctr), uiVal, 1.0, nil, "%.2f deg")
        im.tooltip("Set the azimuth half-angle, used in data display.")
        im.PopItemWidth()
        sensor.halfAngleDeg = max(10.0, min(179.0, uiVal[0]))
        ctr = ctr + 1

        im.Dummy(im.ImVec2(5, 0))
        im.Separator()
        im.Dummy(im.ImVec2(5, 0))

        -- 'Set Update Time' input box.
        im.TextColored(greenB, 'RADAR Update Properties:')
        local oldVal = sensor.updateTime
        local uiVal = im.FloatPtr(oldVal)
        im.PushItemWidth(130)
        im.InputFloat("Sensor Refresh Rate ###" .. tostring(ctr), uiVal, 0.01, nil, "%.4f s")
        im.tooltip('Set the time between sensor updates.')
        im.PopItemWidth()
        sensor.updateTime = max(0.0001, min(60.0, uiVal[0]))
        ctr = ctr + 1

        -- 'Set Update Priority' slider.
        local oldVal = sensor.updatePriority
        local uiVal = im.FloatPtr(oldVal)
        im.PushStyleVar1(im.StyleVar_GrabMinSize, 20)
        im.PushItemWidth(130)
        im.SliderFloat("Update Priority [0, 1]", uiVal, 0.0, 1.0, "%.3f")
        im.tooltip('The update priority of the sensor [0 = highest, 1 = lowest]. This is used for GPU scheduling')
        im.PopItemWidth()
        im.PopStyleVar()
        sensor.updatePriority = max(0.0, min(1.0, uiVal[0]))
        ctr = ctr + 1

        im.Dummy(im.ImVec2(5, 0))
        im.Separator()
        im.Dummy(im.ImVec2(5, 0))

        -- 'Is Visualised' checkbox.
        im.TextColored(greenB, 'RADAR Operation Flags:')
        local oldVal = sensor.isVisualised
        local uiVal = im.BoolPtr(oldVal)
        im.Checkbox("Visualise On Map", uiVal)
        im.tooltip('Toggle whether to visualise the sensor, or not.')
        sensor.isVisualised = uiVal[0]

        im.Dummy(im.ImVec2(5, 0))
        im.Separator()
      end
    else
      isSensorPropWinOpen = false -- Handle window close.
    end
    editor.endWindow()
  end
end

-- World editor main callback for rendering the UI.
local function onEditorGui()
  if not isMapSensorEditor then
    return
  end

  -- Handle the placing of new sensors, if required.
  if isPlaceMode then
    handlePlaceSensor()
  end

  -- Render 'edit sphere' markers at each sensor in the configuration.
  local numSensors = #sensors
  for i = 1, numSensors do
    local s = sensors[i]
    if not s.isLive and s.pos then
      local pos, dir, up, right = s.pos, s.dir, s.up, s.dir:cross(s.up)
      util.renderSensorBoxAndFrame(pos, dir, up, right)
      if s.type == tUltrasonic or s.type == tRADAR then
        util.renderBeamShape(s, pos, dir, up, right)
      end
    end
  end

  if not isPlaceMode and #sensors > 0 then
    local s = sensors[selectedSensorIdx]
    local posWS = s.pos
    if s and posWS and not s.isLive then
      handleGimbals(posWS)
      util.renderLocalFrame(posWS, s.dir, s.up)
    end
  end

  -- Keep the UI indexing in range.
  selectedSensorIdx = max(1, min(#sensors, selectedSensorIdx))

  -- Manage the front end.
  manageMainToolWindow()
  manageSensorPropWindow()
end

-- Called when the 'Sensor Configuration Editor' icon is pressed.
local function onActivate()
  editor.clearObjectSelection()
  editor.showWindow(toolWinName)
  isMapSensorEditor = true
end

-- Called when the 'Sensor Configuration Editor' is exited.
local function onDeactivate()
  editor.hideWindow(toolWinName)
  editor.hideWindow(sensorPropWinName)
  isMapSensorEditor = false
  isSensorPropWinOpen = false
end

-- Called upon world editor initialization.
local function onEditorInitialized()
  if tech_license.isValid() then
    editor.editModes.mapSensorEditMode = {
      displayName = "Edit Map Sensor Configuration",
      onUpdate = nop,
      onActivate = onActivate,
      onDeactivate = onDeactivate,
      icon = editor.icons.mapWithEmitter,
      iconTooltip = "Map Sensor Editor",
      auxShortcuts = {},
      hideObjectIcons = true }
    editor.registerWindow(toolWinName, toolWinSize)
    editor.registerWindow(sensorPropWinName, sensorPropWinSize)
  end
end

-- Serialization function.
local function onSerialize()
  for i = 1, #sensors do
    sensors[i].isLive = false
    handleEditLiveModeSwitch(i)
  end
  return { d = lpack.encode(sensors) }
end

-- Deserialization function.
local function onDeserialized(dataIn)
  table.clear(sensors)
  sensors = lpack.decode(dataIn.d)
end


-- Public interface.
M.onEditorGui =                                           onEditorGui
M.onEditorInitialized =                                   onEditorInitialized
M.onSerialize =                                           onSerialize
M.onDeserialized =                                        onDeserialized

return M