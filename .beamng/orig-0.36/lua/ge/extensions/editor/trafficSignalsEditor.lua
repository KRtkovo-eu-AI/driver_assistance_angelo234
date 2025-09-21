-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local im = ui_imgui

local logTag = "editor_trafficSignals"
local editWindowName = "Traffic Signals Editor"
local editModeName = "signalsEditMode"

local instances, controllers, sequences, elements, controllerDefinitions = {}, {}, {}, {}, {}
local selected = {signal = 1, controller = 1, sequence = 1, phase = 1, ctrlDefState = 1, ctrlDefType = 1, group = 1, flashingLight = 1}
local selectedObject, signalCtrlDefinitions
local signalName = im.ArrayChar(256, "")
local ctrlName = im.ArrayChar(256, "")
local sequenceName = im.ArrayChar(256, "")
local ctrlDefinitionStateName = im.ArrayChar(256, "")
local ctrlDefinitionTypeName = im.ArrayChar(256, "")
local colorWarning = im.ImVec4(1, 1, 0, 1)
local colorError = im.ImVec4(1, 0, 0, 1)
local dummyVec = im.ImVec2(0, 5)
local iconVec = im.ImVec2(24, 24)

local lastUsed = {signalType = "lightsBasic"}
local oldTransform = {pos = vec3(), rot = quat(), scl = 1}
local options = {smartName = true, smartObjectSelection = true, showClosestRoad = false}
local windowFlags = {ctrlDefinitions = im.BoolPtr(false)}
local imFlags = {imTable = bit.bor(im.TableFlags_BordersOuterH, im.TableFlags_BordersV)}
local tabFlags = {}
local timedTexts = {}
local selectableControllers = {}
local simLogs = {}
local isDragging = false
local contentWidth, inputWidth
local trafficSignals

local mousePos, targetPos = vec3(), vec3()
local vecUp = vec3(0, 0, 1)
local vecY = vec3(0, 1, 0)
local cylinderRadius = 0.25
local debugColors = {
  main = ColorF(1, 1, 1, 0.4),
  textFG = ColorF(1, 1, 1, 1),
  textBG = ColorI(0, 0, 0, 255)
}

local signalsInitialized, running = false, false

local function staticRayCast()
  if core_forest.getForestObject() then core_forest.getForestObject():disableCollision() end
  local rayCast = cameraMouseRayCast()
  if core_forest.getForestObject() then core_forest.getForestObject():enableCollision() end

  if rayCast then return rayCast.pos end
end

local function updateGizmoTransform()
  local data = instances[selected.signal]
  if data and editor.setAxisGizmoTransform then
    local nodeTransform = MatrixF(true)
    nodeTransform:setPosition(data.pos)
    editor.setAxisGizmoTransform(nodeTransform)
  end
end

local function renameSignal(instance, newName) -- renames the signal instance, and updates properties of related signal objects, if applicable
  instance.tempSignalObjects = instance:getSignalObjects(true) -- before renaming, get the signal objects that are referenced by the old name

  instance.name = newName
  ffi.copy(signalName, newName)

  for _, objId in ipairs(instance.tempSignalObjects) do
    instance:linkSignalObject(objId) -- now links using the new name
  end

  if instance.tempSignalObjects and instance.tempSignalObjects[1] then
    timedTexts.renameObjects = {"Signal objects renamed, please remember to save the level", 10}
  end
end

local function autoNameSignal(instance) -- smartly names the signal instance (what about controllers and sequences?)
  local instanceCtrl = elements[instance.controllerId]
  local name
  if not instanceCtrl or not instanceCtrl.name or instance.controllerId == 0 then
    name = "Signal "
  else
    name = instanceCtrl.name.." "
  end

  local idx = 0
  for _, otherInstance in ipairs(instances) do
    if instance.id ~= otherInstance.id and string.startswith(otherInstance.name, name) then
      local num = string.gsub(otherInstance.name, name, "")
      idx = math.max(idx, tonumber(num) or idx) -- finds the highest number out of all instances with the same prefix
    end
  end
  if idx == 0 then
    idx = instance.id
  else
    idx = idx + 1
  end

  name = name..idx
  return name
end

local function checkSignalErrors() -- checks and appends errors to the simLogs table
  table.clear(simLogs)
  for _, instance in ipairs(instances) do
    if instance.controllerId == 0 then
      table.insert(simLogs, "Controller not defined, this signal instance will not function: "..instance.name)
    end
    if instance.controllerId > 0 and not elements[instance.controllerId] then
      table.insert(simLogs, "Controller failed for signal instance: "..instance.name)
    end
    if instance.sequenceId > 0 and not elements[instance.sequenceId] then
      table.insert(simLogs, "Sequence failed for signal instance: "..instance.name)
    end
  end

  local elementNames = {}
  for _, element in pairs(elements) do
    if not element.name or element.name == "" then
      table.insert(simLogs, "Missing name for element id: "..element.id)
    else
      if not elementNames[element.name] then
        elementNames[element.name] = 1
      else
        table.insert(simLogs, "Duplicate name found: "..element.name)
      end
    end
  end
end

local function getCapColor(controller, useAllControllers) -- returns the color for the debugDrawer top cap of the signal instance
  local r, g, b = 0, 0, 0
  if controller then
    local ctrlArray = useAllControllers and controllers or selectableControllers
    for i, sc in ipairs(ctrlArray) do
      if controller.id == sc.id then
        r, g, b = HSVtoRGB(i / (#ctrlArray + 1), 1, 1)
        break
      end
    end
  end

  return ColorF(r, g, b, 0.6)
end

local function calcControllerDuration(ctrl) -- calculates the total duration of a signal controller
  local totalDuration = 0

  if not ctrl.isSimple and ctrl.states[1] then -- checks if state durations are enabled
    for _, state in ipairs(ctrl.states) do
      totalDuration = totalDuration + state.duration
    end
  end

  return totalDuration
end

local function selectInstance(idx)
  idx = idx or 1
  local instance = instances[idx]
  if not instance then return end

  ffi.copy(signalName, instance.name)
  selected.signal = idx
  table.clear(selectableControllers)
  instance.tempSignalObjects = nil
  updateGizmoTransform()
end

local function selectController(idx)
  idx = idx or 1
  local ctrl = controllers[idx]
  if not ctrl then return end

  ffi.copy(ctrlName, ctrl.name)
  selected.controller = idx
  ctrl.totalDuration = calcControllerDuration(ctrl)
end

local function selectSequence(idx)
  idx = idx or 1
  local sequence = sequences[idx]
  if not sequence then return end

  ffi.copy(sequenceName, sequence.name)
  selected.sequence = idx
  selected.phase = 1
end

local function processGroups() -- groups instances by intersections, via a simple algorithm
  for _, instance in ipairs(instances) do -- first, reset all groups
    instance.group = nil
    instance.intersectionId = nil
  end
  for _, instance in ipairs(instances) do
    if not instance.group and not instance.intersectionId then
      local refPos = instance.pos + instance.dir * clamp(instance.radius, 3, 30)
      local idList = {}
      for _, other in ipairs(instances) do
        if not other.intersectionId and instance.sequenceId == other.sequenceId then -- sequence ids must match
          if instance.pos:squaredDistance(other.pos) <= 1600 and other.dir:dot(refPos - other.pos) > 0 then
            table.insert(idList, other.id)
            other.intersectionId = instance.id -- temporary id to help identify the group
          end
        end
      end

      if idList[2] then
        local str = table.concat(idList, "_")
        str = "group_"..str

        for _, id in ipairs(idList) do
          if not elements[id].group then
            elements[id].group = str
          end
        end
      end
    end
  end
end

local function resetSignals() -- resets editor signals data
  table.clear(instances)
  table.clear(controllers)
  table.clear(sequences)
  table.clear(simLogs)
  selected.signal, selected.controller, selected.sequence, selected.phase = 1, 1, 1, 1
  lastUsed = {signalType = "lightsBasic"}
end

local function getSerializedSignals() -- returns serialized signals data (intended for save data)
  local instancesSerialized, controllersSerialized, sequencesSerialized = {}, {}, {}

  for _, instance in ipairs(instances) do
    table.insert(instancesSerialized, instance:onSerialize())
  end
  for _, ctrl in ipairs(controllers) do
    table.insert(controllersSerialized, ctrl:onSerialize())
  end
  for _, sequence in ipairs(sequences) do
    table.insert(sequencesSerialized, sequence:onSerialize())
  end

  return {instances = instancesSerialized, controllers = controllersSerialized, sequences = sequencesSerialized}
end

local function getCurrentElements() -- returns the elements data from the editor
  return elements
end

local function getCurrentSignals() -- returns the signals data from the editor
  return {instances = instances, controllers = controllers, sequences = sequences}
end

local function setCurrentSignals(data) -- directly sets the signals data for the editor
  data = data or core_trafficSignals.getData() -- uses current level signal data by default
  data.instances = data.instances or {}
  data.controllers = data.controllers or {}
  data.sequences = data.sequences or {}
  resetSignals()

  for _, instance in ipairs(data.instances) do
    instance.pos = vec3(instance.pos)
    instance.dir = vec3(instance.dir)
    local new = trafficSignals.newSignal(instance)
    table.insert(instances, new)
    elements[new.id] = new
  end
  for _, ctrl in ipairs(data.controllers) do
    local new = trafficSignals.newController(ctrl)
    table.insert(controllers, new)
    elements[new.id] = new
  end
  for _, sequence in ipairs(data.sequences) do
    local new = trafficSignals.newSequence(sequence)
    table.insert(sequences, new)
    elements[new.id] = new
  end

  selectInstance(selected.signal)
  selectController(selected.controller)
  selectSequence(selected.sequence)
end

local function loadFile(fileName) -- loads the main signals data
  fileName = fileName or editor.levelPath.."signals.json"
  local data = jsonReadFile(fileName)
  if data then
    if not data.intersections then
      setCurrentSignals(data)
    else
      log("W", logTag, "Obsolete signals data!")
      setCurrentSignals({})
    end
  end
end

local function saveFile(fileName) -- saves the main signals data
  fileName = fileName or editor.levelPath.."signals.json"
  jsonWriteFile(fileName, getSerializedSignals(), true)
  timedTexts.save = {"Signals saved!", 3}
end

local function simulate(val) -- runs the simulation for the traffic lights
  if val then
    for _, sequence in ipairs(sequences) do
      sequence.enableTestTimer = true
    end

    trafficSignals.setupSignals(getCurrentSignals())
    trafficSignals.debugLevel = 2
    map.reset() -- this forces the mapmgr signals to update ("temporary" solution)
    running = true
  else
    trafficSignals.setActive(false)
    trafficSignals.debugLevel = 0
    running = false
  end
end

local function createInstanceActionUndo(data)
  elements[instances[data.deleteIdx].id] = nil
  table.remove(instances, data.deleteIdx or #instances)
  selected.signal = math.max(1, selected.signal - 1)
  selectInstance(selected.signal)
end

local function createInstanceActionRedo(data)
  table.insert(instances, trafficSignals.newSignal(data))
  selected.signal = #instances
  instances[selected.signal].name = data.name
  if not data.name then
    instances[selected.signal].name = options.smartName and autoNameSignal(instances[selected.signal]) or "Signal "..selected.signal
  end
  elements[instances[selected.signal].id] = instances[selected.signal]
  selectInstance(selected.signal)
end

local function transformInstanceActionUndo(data)
  instances[selected.signal].pos:set(data.oldTransform.pos)
  instances[selected.signal].dir = vecY:rotated(data.oldTransform.rot)
  instances[selected.signal].radius = clamp(data.oldTransform.scl, 1, 100)
  instances[selected.signal].road = nil
  updateGizmoTransform()
end

local function transformInstanceActionRedo(data)
  instances[selected.signal].pos:set(data.newTransform.pos)
  instances[selected.signal].dir = vecY:rotated(data.newTransform.rot)
  instances[selected.signal].radius = clamp(data.newTransform.scl, 1, 100)
  instances[selected.signal].road = nil
  updateGizmoTransform()
end

local function createControllerActionUndo(data)
  elements[controllers[data.deleteIdx].id] = nil
  table.remove(controllers, data.deleteIdx or #controllers)
  selected.controller = math.max(1, selected.controller - 1)
  selectController(selected.controller)
end

local function createControllerActionRedo(data)
  table.insert(controllers, trafficSignals.newController())
  selected.controller = #controllers
  controllers[selected.controller]:onDeserialized(data)
  controllers[selected.controller].name = data.name or "Controller "..selected.controller
  elements[controllers[selected.controller].id] = controllers[selected.controller]
  selectController(selected.controller)
end

local function createSequenceActionUndo(data)
  elements[sequences[data.deleteIdx].id] = nil
  table.remove(sequences, data.deleteIdx or #sequences)
  selected.sequence = math.max(1, selected.sequence - 1)
  selectSequence(selected.sequence)
end

local function createSequenceActionRedo(data)
  table.insert(sequences, trafficSignals.newSequence())
  selected.sequence = #sequences
  sequences[selected.sequence]:onDeserialized(data)
  sequences[selected.sequence].name = data.name or "Sequence "..selected.sequence
  elements[sequences[selected.sequence].id] = sequences[selected.sequence]
  selectSequence(selected.sequence)
end

local function gizmoBeginDrag()
  if instances[selected.signal] then
    instances[selected.signal].rot = quatFromDir(instances[selected.signal].dir, vecUp)
    oldTransform.pos = vec3(instances[selected.signal].pos)
    oldTransform.rot = quat(instances[selected.signal].rot)
    oldTransform.scl = instances[selected.signal].radius
  end
end

local function gizmoEndDrag()
  if instances[selected.signal] then
    isDragging = false
    local newTransform = {
      pos = vec3(instances[selected.signal].pos),
      rot = quat(instances[selected.signal].rot),
      scl = instances[selected.signal].radius
    }

    local act = {oldTransform = oldTransform, newTransform = newTransform}
    editor.history:commitAction("Transform Signal Instance", act, transformInstanceActionUndo, transformInstanceActionRedo)
  end
end

local function gizmoMidDrag()
  if instances[selected.signal] then
    isDragging = true
    if editor.getAxisGizmoMode() == editor.AxisGizmoMode_Translate then
      instances[selected.signal].pos:set(editor.getAxisGizmoTransform():getColumn(3))
    elseif editor.getAxisGizmoMode() == editor.AxisGizmoMode_Rotate then
      local rotation = QuatF(0, 0, 0, 1)
      rotation:setFromMatrix(editor.getAxisGizmoTransform())

      if editor.getAxisGizmoAlignment() == editor.AxisGizmoAlignment_Local then
        instances[selected.signal].rot = quat(rotation)
      else
        instances[selected.signal].rot = oldTransform.rot * quat(rotation)
      end
      instances[selected.signal].dir = vecY:rotated(instances[selected.signal].rot)
    elseif editor.getAxisGizmoMode() == editor.AxisGizmoMode_Scale then
      local scl = vec3(editor.getAxisGizmoScale())
      local sclMin, sclMax = math.min(scl.x, scl.y, scl.z), math.max(scl.x, scl.y, scl.z)
      instances[selected.signal].radius = clamp(sclMin < 1 and oldTransform.scl * sclMin or oldTransform.scl * sclMax, 1, 100)
    end
  end
end

local function smartSelectObjects(radius) -- selects multiple similar objects within the same area and rotation
  if not editor.selection.object then return end

  local selectedObj = scenetree.findObjectById(editor.selection.object[1])
  local validIds = {}
  radius = radius or 40

  if selectedObj:getClassName() == "TSStatic" then
    local internalName = selectedObj:getInternalName()

    local dir1 = selectedObj:getTransform():getForward()
    for _, obj in ipairs(getObjectsByClass("TSStatic") or {}) do
      if (internalName and obj:getInternalName() == internalName) or obj.shapeName == selectedObj.shapeName then
        if selectedObj:getPosition():squaredDistance(obj:getPosition()) <= square(radius) then
          -- strong assumption that the TSStatics were created with the same initial rotation
          local dir2 = obj:getTransform():getForward()
          if dir1:dot(dir2) >= 0.93 then -- roughly within 20 degrees
            table.insert(validIds, obj:getId())
          end
        end
      end
    end
  end

  if validIds[2] then
    editor.selectObjects(validIds)
  end
end

local function tabCtrlDefinitionTypes()
  if im.BeginCombo("Type##ctrlDefinitionTypes", signalCtrlDefinitions.typesSorted[selected.ctrlDefType] or "(None)") then
    for i, name in ipairs(signalCtrlDefinitions.typesSorted) do
      if im.Selectable1(name.."##ctrlDefinitionType", selected.ctrlDefType == i) then
        selected.ctrlDefType = i
        ffi.copy(ctrlDefinitionTypeName, name)
      end
    end
    im.EndCombo()
  end
  if im.Button("New...##ctrlDefinitionTypes") then
    local name = "Type "..(#signalCtrlDefinitions.typesSorted + 1)
    selected.ctrlDefType = 0
    ffi.copy(ctrlDefinitionTypeName, name)
    signalCtrlDefinitions.types[name] = {name = name, states = {"basicStop"}}
    signalCtrlDefinitions._update = true
  end
  im.SameLine()
  if im.Button("Remove##ctrlDefinitionTypes") then
    signalCtrlDefinitions.types[signalCtrlDefinitions.typesSorted[selected.ctrlDefType] or ""] = nil
    selected.ctrlDefType = 0
    signalCtrlDefinitions._update = true
  end

  im.Dummy(dummyVec)
  local currName = signalCtrlDefinitions.typesSorted[selected.ctrlDefType] or ""
  local currData = signalCtrlDefinitions.types[currName]
  if currData then
    currData._edited = true

    if editor.uiInputText("Name##ctrlDefinitionTypes", ctrlDefinitionTypeName, nil, im.InputTextFlags_EnterReturnsTrue) then
      local name = ffi.string(ctrlDefinitionTypeName)
      signalCtrlDefinitions.types[name] = signalCtrlDefinitions.types[currName]
      signalCtrlDefinitions.types[name].name = name
      signalCtrlDefinitions.types[currName] = nil
      selected.ctrlDefType = 0
      signalCtrlDefinitions._update = true
    end

    -- temp data for this window
    if not currData.statesArraySize then
      currData.statesArraySize = currData.states and #currData.states or 1
    end
    currData.defaultIndex = currData.defaultIndex or 1

    local var = im.IntPtr(currData.statesArraySize)
    im.PushItemWidth(inputWidth)
    if im.InputInt("States Array Size".."##ctrlDefinitionTypeData", var, 1) then
      currData.statesArraySize = clamp(var[0], 1, 20)
    end
    im.PopItemWidth()

    im.Dummy(dummyVec)

    local columnWidth = im.GetContentRegionAvailWidth() * 0.5

    im.Columns(2)
    im.SetColumnWidth(1, columnWidth)

    im.TextUnformatted("State")
    im.NextColumn()
    im.TextUnformatted("Is Default")
    im.NextColumn()

    for i = 1, currData.statesArraySize do
      if not currData.states[i] then
        table.insert(currData.states, "basicStop")
      end

      if im.BeginCombo("##ctrlDefinitionTypeDataState"..i, currData.states[i] or "(None)") then
        for _, state in ipairs(signalCtrlDefinitions.tempStatesSorted) do
          if im.Selectable1(state.."##ctrlDefinitionTypeData"..i, currData.states[i] == state) then
            currData.states[i] = state
          end
        end
        im.EndCombo()
      end
      im.NextColumn()

      local val = im.IntPtr(currData.defaultIndex)

      if im.RadioButton2("##ctrlDefinitionTypeDataDefaultState"..i, val, im.Int(i)) then
        currData.defaultIndex = val[0]
      end
      im.NextColumn()
    end
  end
  im.Columns(1)

  im.Dummy(dummyVec)
end

local function tabCtrlDefinitionStates()
  if im.BeginCombo("State##ctrlDefinitionStates", signalCtrlDefinitions.statesSorted[selected.ctrlDefState] or "(None)") then
    for i, name in ipairs(signalCtrlDefinitions.statesSorted) do
      if im.Selectable1(name.."##ctrlDefinitionState", selected.ctrlDefState == i) then
        selected.ctrlDefState = i
        ffi.copy(ctrlDefinitionStateName, name)
      end
    end
    im.EndCombo()
  end
  if im.Button("New...##ctrlDefinitionStates") then
    local name = "State "..(#signalCtrlDefinitions.statesSorted + 1)
    selected.ctrlDefState = 0
    ffi.copy(ctrlDefinitionStateName, name)
    signalCtrlDefinitions.states[name] = {name = name, action = "stop", duration = 3, flashingInterval = 0, flashingLights = {}, enableFlashingLights = false, flashingLightsArraySize = 1, lightsArraySize = 3}
    signalCtrlDefinitions._update = true
  end
  im.SameLine()
  if im.Button("Remove##ctrlDefinitionStates") then
    signalCtrlDefinitions.states[signalCtrlDefinitions.statesSorted[selected.ctrlDefState] or ''] = nil
    selected.ctrlDefState = 0
    signalCtrlDefinitions._update = true
  end

  im.Dummy(dummyVec)
  local currName = signalCtrlDefinitions.statesSorted[selected.ctrlDefState] or ''
  local currData = signalCtrlDefinitions.states[currName]
  if currData then
    currData._edited = true

    -- temp data for this window
    if not currData.lightsArraySize then
      currData.lightsArraySize = currData.lights and #currData.lights or 3
    end
    if not currData.flashingLights then
      currData.flashingLights = {currData.lights and deepcopy(currData.lights) or {}}
    end
    if not currData.flashingLightsArraySize then
      currData.flashingLightsArraySize = #currData.flashingLights
      currData.enableFlashingLights = currData.flashingLightsArraySize > 1 and true or false
    end
    currData.duration = currData.duration or 0

    if editor.uiInputText("Name##ctrlDefinitionState", ctrlDefinitionStateName, nil, im.InputTextFlags_EnterReturnsTrue) then
      local name = ffi.string(ctrlDefinitionStateName)
      signalCtrlDefinitions.states[name] = signalCtrlDefinitions.states[currName]
      signalCtrlDefinitions.states[name].name = name
      signalCtrlDefinitions.states[currName] = nil
      selected.ctrlDefState = 0
      signalCtrlDefinitions._update = true
    end

    im.PushItemWidth(contentWidth)
    if im.BeginCombo("Signal Action##ctrlDefinitionState", currData.action or "(None)") then
      for _, action in ipairs(tableKeysSorted(signalCtrlDefinitions.signalActions)) do
        if im.Selectable1(action.."##ctrlDefinitionState", currData.action == action) then
          currData.action = action
        end
      end
      im.EndCombo()
    end
    im.PopItemWidth()

    local var = im.IntPtr(currData.lightsArraySize)
    im.PushItemWidth(inputWidth)
    if im.InputInt("Lights Array Size".."##ctrlDefinitionStateLight", var, 1) then
      currData.lightsArraySize = clamp(var[0], 1, 5)
    end
    im.PopItemWidth()

    var = im.FloatPtr(currData.duration)
    im.PushItemWidth(inputWidth)
    if im.InputFloat("Default Duration##ctrlDefinitionStateLight", var, 0.1, 0.1, "%.2f", im.InputTextFlags_EnterReturnsTrue) then
      currData.duration = math.max(0, var[0])
    end
    im.PopItemWidth()
    im.tooltip("Set this to 0 to disable duration.")

    var = im.BoolPtr(currData.enableFlashingLights)
    if im.Checkbox("Enable Flashing Lights Sequence", var) then
      currData.enableFlashingLights = var[0]
    end

    if currData.enableFlashingLights then
      var = im.FloatPtr(currData.flashingInterval)
      im.PushItemWidth(inputWidth)
      if im.InputFloat("Flashing Lights Interval##ctrlDefinitionStateLight", var, 0.1, 0.1, "%.2f", im.InputTextFlags_EnterReturnsTrue) then
        currData.flashingInterval = math.max(0, var[0])
      end
      im.PopItemWidth()
      im.Dummy(dummyVec)

      if im.Button("Create##flashingLight") then
        currData.flashingLightsArraySize = currData.flashingLightsArraySize + 1
        selected.flashingLight = currData.flashingLightsArraySize
      end
      im.SameLine()
      if im.Button("Delete##sequencePhase") then
        currData.flashingLightsArraySize = math.max(1, currData.flashingLightsArraySize - 1)
        selected.flashingLight = math.min(selected.flashingLight, currData.flashingLightsArraySize)
      end

      for i = 1, currData.flashingLightsArraySize do
        local isCurrentLight = i == selected.flashingLight
        if isCurrentLight then
          im.PushStyleColor2(im.Col_Button, im.GetStyleColorVec4(im.Col_ButtonActive))
        end
        if im.Button(" "..i.." ") then
          selected.flashingLight = i
        end
        if isCurrentLight then
          im.PopStyleColor()
        end
        im.SameLine()
      end
      im.Dummy(dummyVec)

      im.TextUnformatted("Flashing Light #"..selected.flashingLight)
      im.Dummy(dummyVec)
    else
      selected.flashingLight = 1
    end

    for i = 1, currData.lightsArraySize do
      if not currData.flashingLights[selected.flashingLight] then
        currData.flashingLights[selected.flashingLight] = {}
      end
      if not currData.flashingLights[selected.flashingLight][i] then
        table.insert(currData.flashingLights[selected.flashingLight], "black")
      end

      im.PushItemWidth(contentWidth)
      if im.BeginCombo("Light Color #"..i.."##ctrlDefinitionStateLight"..i, currData.flashingLights[selected.flashingLight][i] or "(None)") then
        for _, color in ipairs(tableKeysSorted(signalCtrlDefinitions.signalColors)) do
          if im.Selectable1(color.."##ctrlDefinitionStateLight"..i, currData.flashingLights[selected.flashingLight][i] == color) then
            currData.flashingLights[selected.flashingLight][i] = color
          end
        end
        im.EndCombo()
      end
      im.PopItemWidth()
      im.SameLine()
      local iconColor = signalCtrlDefinitions.signalColors[currData.flashingLights[selected.flashingLight][i]]
      if iconColor then
        local c = iconColor:toTable()
        iconColor = im.ImVec4(c[1], c[2], c[3], c[4])
      else
        iconColor = im.ImVec4(1, 1, 1, 1)
      end
      editor.uiIconImage(editor.icons.lens, iconVec, iconColor)
    end
  end

  im.Dummy(dummyVec)
end

local function windowSignalCtrlDefinitions()
  im.SetNextWindowSize(im.ImVec2(500, 600), im.Cond_FirstUseEver)
  if im.Begin("Controller Definitions##ctrlDefinitions", windowFlags.instanceGroups) then
    if not signalCtrlDefinitions then
      core_trafficSignals.loadControllerDefinitions()
      signalCtrlDefinitions = deepcopy(core_trafficSignals.getControllerDefinitions())
      signalCtrlDefinitions.origStates = deepcopy(signalCtrlDefinitions.states)

      table.clear(signalCtrlDefinitions.states)
      table.clear(signalCtrlDefinitions.types)
      tableMerge(signalCtrlDefinitions, jsonReadFile(editor.levelPath.."signalControllerDefinitions.json") or {}) -- loads only custom data, and only from the current level
      signalCtrlDefinitions._update = true
    end

    if signalCtrlDefinitions._update then
      signalCtrlDefinitions.statesSorted = tableKeysSorted(signalCtrlDefinitions.states)
      signalCtrlDefinitions.typesSorted = tableKeysSorted(signalCtrlDefinitions.types)

      local tempStates = tableMerge(signalCtrlDefinitions.origStates, signalCtrlDefinitions.states)
      signalCtrlDefinitions.tempStatesSorted = tableKeysSorted(tempStates)

      if selected.ctrlDefState == 0 then
        if ctrlDefinitionStateName then
          selected.ctrlDefState = arrayFindValueIndex(signalCtrlDefinitions.statesSorted, ffi.string(ctrlDefinitionStateName)) or 1
        else
          selected.ctrlDefState = 1
        end
      end
      if selected.ctrlDefType == 0 then
        if ctrlDefinitionTypeName then
          selected.ctrlDefType = arrayFindValueIndex(signalCtrlDefinitions.typesSorted, ffi.string(ctrlDefinitionTypeName)) or 1
        else
          selected.ctrlDefType = 1
        end
      end
      signalCtrlDefinitions._update = nil
    end

    if im.BeginTabBar("Controller Definition Tabs##ctrlDefinitions") then
      if im.BeginTabItem("Types") then
        tabCtrlDefinitionTypes()
        im.EndTabItem()
      end
      if im.BeginTabItem("States") then
        tabCtrlDefinitionStates()
        im.EndTabItem()
      end
    end

    im.Separator()

    if im.Button("Save & Close##ctrlDefinitions") then
      for _, typeData in pairs(signalCtrlDefinitions.types) do
        if typeData.statesArraySize then
          if typeData.statesArraySize == 1 or typeData.defaultIndex > typeData.statesArraySize then
            typeData.defaultIndex = nil
          end
          for i = #typeData.states, typeData.statesArraySize + 1, -1 do
            table.remove(typeData.states, i)
          end
        end

        typeData.statesArraySize = nil
        typeData._edited = nil
      end

      for _, state in pairs(signalCtrlDefinitions.states) do
        if state._edited then
          for i = #state.flashingLights, state.flashingLightsArraySize + 1, -1 do
            table.remove(state.flashingLights, i)
          end
          for i, light in ipairs(state.flashingLights) do
            for j = #light, state.lightsArraySize + 1, -1 do
              table.remove(state.flashingLights[i], j)
            end
          end

          state.lights = deepcopy(state.flashingLights[1] or {})

          if not state.enableFlashingLights then
            state.flashingLights = nil
            state.flashingInterval = 0
          end

          if state.duration <= 0 then
            state.duration = nil
          end

          state.enableFlashingLights = nil
          state.flashingLightsArraySize = nil
          state.lightsArraySize = nil
        end
        state._edited = nil
      end

      local saveData = {states = signalCtrlDefinitions.states, types = signalCtrlDefinitions.types}
      jsonWriteFile(editor.levelPath.."signalControllerDefinitions.json", saveData, true)
      core_trafficSignals.resetControllerDefinitions()
      core_trafficSignals.setControllerDefinitions(saveData)
      windowFlags.ctrlDefinitions[0] = false
      log("I", logTag, "Custom signal controller data saved")
    end
    im.SameLine()
    if im.Button("Discard & Close##ctrlDefinitions") then
      windowFlags.ctrlDefinitions[0] = false
    end
    im.End()
  end
end

local function tabInstances()
  im.BeginChild1("instances", im.ImVec2(150 * im.uiscale[0], 0), im.WindowFlags_ChildWindow)

  for i, instance in ipairs(instances) do
    if im.Selectable1(instance.name, selected.signal == i) then
      selectInstance(i)
    end
  end

  im.Separator()

  if im.Selectable1("New...##instance", false) then -- if this is clicked, creates a new signal instance at the current camera position
    targetPos:set(core_camera.getPosition())
    targetPos.z = be:getSurfaceHeightBelow(targetPos)
    local act = {pos = vec3(targetPos), controllerId = lastUsed.controllerId, sequenceId = lastUsed.sequenceId}
    editor.history:commitAction("Create Signal Instance", act, createInstanceActionUndo, createInstanceActionRedo)
    selectInstance(selected.signal)
  end
  im.tooltip("Shift-Click in the world to create a new signal instance point.")

  im.EndChild()
  im.SameLine()

  im.BeginChild1("instanceData", im.ImVec2(0, 0), im.WindowFlags_ChildWindow)
  contentWidth = im.GetContentRegionAvailWidth() * 0.5
  if not im.IsWindowHovered(im.HoveredFlags_AnyWindow) and editor.getCurrentEditModeName() ~= "objectSelect" and editor.keyModifiers.shift then
    debugDrawer:drawTextAdvanced(mousePos, "Create Signal Instance", debugColors.textFG, true, false, debugColors.textBG)

    if im.IsMouseClicked(0) then
      local act = {pos = vec3(mousePos), controllerId = lastUsed.controllerId, sequenceId = lastUsed.sequenceId}
      editor.history:commitAction("Create Signal Instance", act, createInstanceActionUndo, createInstanceActionRedo)
      selectInstance(selected.signal)
    end
  end

  local currInstance = instances[selected.signal]
  if currInstance then
    im.TextUnformatted("Current Signal: "..currInstance.name.." ["..currInstance.id.."]")
    im.SameLine()
    if im.Button("Delete##instance") then
      local act = instances[selected.signal]:onSerialize()
      act.deleteIdx = selected.signal
      editor.history:commitAction("Delete Signal Instance", act, createInstanceActionRedo, createInstanceActionUndo)
    end

    im.PushItemWidth(contentWidth)
    if editor.uiInputText("Name##instance", signalName, nil, im.InputTextFlags_EnterReturnsTrue) then
      renameSignal(currInstance, ffi.string(signalName))
    end
    im.PopItemWidth()

    local buttonText = currInstance.group and "Update Grouped Signals##instance" or "Show Grouped Signals##instance"
    if im.Button(buttonText) then
      processGroups()
    end
    if currInstance.group then
      im.BeginChild1("signalGroup", im.ImVec2(contentWidth, 80 * im.uiscale[0]), im.WindowFlags_ChildWindow)
      for i, instance in ipairs(instances) do
        if instance.name ~= currInstance.name and instance.group == currInstance.group then
          if im.Selectable1(instance.name, false) then
            selectInstance(i)
          end
        end
      end
      im.EndChild()
      im.SameLine()
      im.TextUnformatted("Grouped Signals")
    end

    im.Dummy(dummyVec)

    local signalPos = im.ArrayFloat(3)
    local changed = false
    signalPos[0], signalPos[1], signalPos[2] = currInstance.pos.x, currInstance.pos.y, currInstance.pos.z

    im.PushItemWidth(contentWidth)
    if im.InputFloat3("Position##instance", signalPos, "%0."..editor.getPreference("ui.general.floatDigitCount").."f", im.InputTextFlags_EnterReturnsTrue) then
      changed = true
    end
    im.PopItemWidth()
    if im.Button("Down to Terrain##instance") then
      if core_terrain.getTerrain() then
        signalPos[2] = core_terrain.getTerrainHeight(currInstance.pos)
        changed = true
      end
    end

    if changed then -- commits changes to history
      gizmoBeginDrag()
      instances[selected.signal].pos = vec3(signalPos[0], signalPos[1], signalPos[2])
      gizmoEndDrag()
    end

    im.Dummy(dummyVec)

    ---- select sequence for signal instance ----
    local elem = elements[currInstance.sequenceId]
    local name = elem and elem.name or "(Missing)"
    if currInstance.sequenceId == 0 then
      name = "Basic" -- this is a nil sequence
    end

    im.PushItemWidth(contentWidth)
    if im.BeginCombo("Sequence##instance", name) then
      if im.Selectable1("Basic##instanceSequenceBasic", not elem) then
        currInstance:setSequence(0)
        lastUsed.sequenceId = 0
        table.clear(selectableControllers)
      end
      for _, sequence in ipairs(sequences) do
        if im.Selectable1(sequence.name.."##instanceSequence", sequence.name == name) then
          currInstance:setSequence(sequence.id)
          lastUsed.sequenceId = sequence.id
          table.clear(selectableControllers)
        end
      end
      im.EndCombo()
    end
    im.PopItemWidth()

    if currInstance.sequenceId > 0 then
      if not elements[currInstance.sequenceId] then
        im.SameLine()
        editor.uiIconImage(editor.icons.error_outline, iconVec, colorError)
      end

      if im.Button("Edit##instanceSequence") or currInstance._newSequence then
        tabFlags = {im.flags(im.TabItemFlags_None), im.flags(im.TabItemFlags_None), im.flags(im.TabItemFlags_SetSelected)} -- selects the sequence tab
        currInstance._newSequence = nil
        for i, sequence in ipairs(sequences) do
          if sequence.id == currInstance.sequenceId then
            selectSequence(i)
          end
        end
      end
    else
      if im.Button("New...##instanceSequence") then
        editor.history:commitAction("Create Sequence", {}, createSequenceActionUndo, createSequenceActionRedo)
        currInstance.sequenceId = sequences[#sequences].id
        currInstance._newSequence = true
      end
    end

    -- update selectable controllers here
    if not selectableControllers[1] then
      local sequence = elements[currInstance.sequenceId]
      if sequence then
        local temp = {}
        for _, phase in ipairs(sequence.phases) do
          for _, data in ipairs(phase.controllerData) do
            if elements[data.id] and not temp[data.id] then
              table.insert(selectableControllers, elements[data.id])
              temp[data.id] = 1
            end
          end
        end
      else
        for _, ctrl in ipairs(controllers) do
          table.insert(selectableControllers, ctrl)
        end
      end
    end

    im.Dummy(dummyVec)

    ---- select controller for signal instance ----
    elem = elements[currInstance.controllerId]
    name = elem and elem.name or "(Missing)"
    if currInstance.controllerId == 0 then
      name = "(None)"
    end

    im.PushItemWidth(contentWidth)
    if im.BeginCombo("Controller##instance", name) then
      if im.Selectable1("(None)##instanceController", not elem) then
        currInstance:setController(0)
        lastUsed.controllerId = 0
        if options.smartName then
          renameSignal(currInstance, autoNameSignal(currInstance))
        end
      end
      -- only controllers found within the current sequence should be selectable
      for _, ctrl in ipairs(selectableControllers) do
        if im.Selectable1(ctrl.name, ctrl.name == name) then
          currInstance:setController(ctrl.id)
          lastUsed.controllerId = ctrl.id
          if options.smartName then
            renameSignal(currInstance, autoNameSignal(currInstance))
          end
        end
      end
      im.EndCombo()
    end
    im.PopItemWidth()

    if currInstance.controllerId > 0 then
      if not elements[currInstance.controllerId] then
        im.SameLine()
        editor.uiIconImage(editor.icons.error_outline, iconVec, colorError)
      end

      if im.Button("Edit##instanceCtrl") or currInstance._newController then
        tabFlags = {im.flags(im.TabItemFlags_None), im.flags(im.TabItemFlags_SetSelected), im.flags(im.TabItemFlags_None)} -- selects the controller tab
        currInstance._newController = nil
        for i, ctrl in ipairs(controllers) do
          if ctrl.id == currInstance.controllerId then
            selectController(i)
          end
        end
      end
    else
      if im.Button("New...##instanceCtrl") then
        editor.history:commitAction("Create Controller", {}, createControllerActionUndo, createControllerActionRedo)
        currInstance.controllerId = controllers[#controllers].id
        currInstance._newController = true
      end
    end

    if not currInstance.road then -- finds best road for this instance
      currInstance.road = currInstance:getBestRoad()
      if not currInstance.road then
        im.TextColored(colorWarning, "Warning, could not find closest road node!")
      end
    end

    im.Dummy(dummyVec)
    im.Separator()
    im.TextUnformatted("Signal Objects")

    im.SameLine()
    im.Button("?")
    im.tooltip("Select signal objects, such as traffic lights, to link with this instance. Remember to Save Level (Ctrl+S) after you are done.")

    im.Dummy(dummyVec)

    if not currInstance.tempSignalObjects then -- creates a temporary table of linked signal object ids
      currInstance.tempSignalObjects = currInstance:getSignalObjects(true)
    end

    if editor.getCurrentEditModeName() ~= "objectSelect" then
      if im.Button("Select Objects##signalObjects") then
        editor.selectEditMode(editor.editModes["objectSelect"])
        timedTexts.selectObjects = {"Currently in Object Selection mode.", 100000}
      end
      im.tooltip("Enables object selection mode to select signal objects.")

      im.SameLine()

      if im.Button("Reset Objects##signalObjects") then
        if currInstance.tempSignalObjects then
          for _, objId in ipairs(currInstance.tempSignalObjects) do
            currInstance:unlinkSignalObject(objId)
          end
          table.clear(currInstance.tempSignalObjects)
        end
      end
      im.tooltip("Resets linked signal objects.")
    else
      if options.smartObjectSelection and timedTexts.selectObjects then
        -- whenever a single object is selected, all others of the same type, area, and rotation are also selected
        -- use internalName if you want to group varying traffic light shapes together
        -- otherwise, this will only match by shapeName
        if editor.selection.object and not editor.selection.object[2] then
          smartSelectObjects()
        end
      end

      local count = tableSize(editor.selection.object)
      if im.Button("Confirm Selection ("..count..")##signalObjects") then
        if count > 0 then
          for _, objId in ipairs(editor.selection.object) do
            currInstance:linkSignalObject(objId)

            if not arrayFindValueIndex(currInstance.tempSignalObjects, objId) then
              table.insert(currInstance.tempSignalObjects, objId)
            end
          end
          editor.selectEditMode(editor.editModes[editModeName])
          editor.selection.object = nil
          timedTexts.selectObjects = nil
          timedTexts.applyFields = {"Updated "..count.." objects: [signalInstance] = "..currInstance.name, 6}
        end
      end
      im.tooltip("Apply the dynamic field [signalInstance] to objects in this selection.")
      im.SameLine()

      if im.Button("Cancel##signalObjects") then
        editor.selectEditMode(editor.editModes[editModeName])
        editor.selection.object = nil
        timedTexts.selectObjects = nil
      end
    end

    im.Dummy(dummyVec)

    if timedTexts.selectObjects then
      im.TextColored(colorWarning, timedTexts.selectObjects[1])
    elseif timedTexts.applyFields then
      im.TextColored(colorWarning, timedTexts.applyFields[1])
    else
      im.TextUnformatted(" ")
    end

    if not currInstance.tempSignalObjects[1] then
      im.TextUnformatted("No linked objects found.")
    else
      im.BeginChild1("signalObjects", im.ImVec2(contentWidth, 100 * im.uiscale[0]), im.WindowFlags_ChildWindow)
      for _, oid in ipairs(currInstance.tempSignalObjects) do
        local obj = scenetree.findObjectById(oid)
        if obj then
          if im.Selectable1(tostring(oid), selectedObject == oid) then
            editor.selectObjects({oid})
            selectedObject = oid
          end
        end
      end
      im.EndChild()
    end

    if editor.isViewportHovered() and im.IsMouseClicked(0) and not editor.isAxisGizmoHovered() and not editor.keyModifiers.shift then
      for i, instance in ipairs(instances) do
        if mousePos:squaredDistance(instance.pos) <= square(cylinderRadius * 2) then
          selectInstance(i)
        end
      end
      updateGizmoTransform()
    end
    editor.updateAxisGizmo(gizmoBeginDrag, gizmoEndDrag, gizmoMidDrag)
    editor.drawAxisGizmo()
  end
  im.EndChild()
end

local function tabControllers()
  table.clear(selectableControllers)

  im.BeginChild1("controllers", im.ImVec2(150 * im.uiscale[0], 0), im.WindowFlags_ChildWindow)

  for i, ctrl in ipairs(controllers) do
    if im.Selectable1(ctrl.name, selected.controller == i) then
      selectController(i)
    end
  end
  im.Separator()
  if im.Selectable1("Create...##controller", false) then
    editor.history:commitAction("Create Controller", {}, createControllerActionUndo, createControllerActionRedo)
  end
  im.EndChild()
  im.SameLine()

  im.BeginChild1("controllerData", im.ImVec2(0, 0), im.WindowFlags_ChildWindow)
  contentWidth = im.GetContentRegionAvailWidth() * 0.5
  local currController = controllers[selected.controller]
  if currController then
    im.TextUnformatted("Current Controller: "..currController.name.." ["..currController.id.."]")

    im.SameLine()
    if im.Button("Delete##controller") then
      local act = currController:onSerialize()
      act.deleteIdx = selected.controller
      editor.history:commitAction("Delete Controller", act, createControllerActionRedo, createControllerActionUndo)
    end

    im.PushItemWidth(contentWidth)
    if editor.uiInputText("Name##controller", ctrlName, nil, im.InputTextFlags_EnterReturnsTrue) then
      currController.name = ffi.string(ctrlName)
    end
    im.PopItemWidth()

    local currCtrlType = currController.type
    if currCtrlType == "none" then
      currController.type = lastUsed.signalType or "none"
    end
    local signalTypes = core_trafficSignals.getControllerDefinitions().types
    local typeName = signalTypes[currCtrlType] and signalTypes[currCtrlType].name or "(None)"

    im.PushItemWidth(contentWidth)
    if im.BeginCombo("Signal Type##controller", typeName) then
      for _, k in ipairs(tableKeysSorted(signalTypes)) do
        if im.Selectable1(signalTypes[k].name, k == currController.type) then
          currController.type = k
          lastUsed.signalType = k
        end
      end
      im.EndCombo()
    end
    im.PopItemWidth()

    if currController.type ~= currCtrlType then
      currController:applyDefinition(currController.type)
    end

    if im.Button("Manage Custom Controllers...##controller") then
      windowFlags.ctrlDefinitions[0] = true
    end

    im.Dummy(dummyVec)
    im.Separator()
    im.TextUnformatted("States")

    im.SameLine()
    im.Button("?")
    im.tooltip("States run in order; the next state starts when the timer reaches the current state duration.")

    im.Dummy(dummyVec)

    if currController.isSimple or not currController.states[1] then
      for _, state in ipairs(currController.states) do
        local stateData = currController:getStateData(state.state)
        if stateData then
          im.TextUnformatted(stateData.name)
        end
      end

      currController.totalDuration = 0

      im.Dummy(dummyVec)
      im.TextUnformatted("No settings available for this controller.")
    else
      local columnWidth = im.GetContentRegionAvailWidth() * 0.333
      local durationEdited = false

      im.Columns(3)
      im.SetColumnWidth(0, columnWidth)
      im.SetColumnWidth(1, columnWidth)

      im.TextUnformatted("State Name")
      im.NextColumn()
      im.TextUnformatted("Duration")
      im.NextColumn()
      im.TextUnformatted("Is Infinite")
      im.NextColumn()

      for i, state in ipairs(currController.states) do
        local stateData = currController:getStateData(state.state)

        if stateData then
          im.TextUnformatted(stateData.name)
          im.NextColumn()

          state.duration = state.duration or -1

          local var = im.FloatPtr(state.duration)
          im.PushItemWidth(columnWidth - 10)
          if im.InputFloat("##controllerState"..i, var, 0.1, 0.1, "%.2f", im.InputTextFlags_EnterReturnsTrue) then
            state.duration = math.max(0, var[0])
            durationEdited = true
          end
          im.PopItemWidth()
          if state.state == "redTrafficLight" then
            im.tooltip("This is usually the delay time until the next signal phase starts.")
          end
          im.NextColumn()

          var = im.BoolPtr(state.duration == -1)
          if im.Checkbox("##controllerStateInfinite"..i, var) then
            state.duration = var[0] and -1 or 0
          end
          im.NextColumn()
        end
      end

      if durationEdited then
        currController.totalDuration = calcControllerDuration(currController)
      end

      im.Columns(1)
    end
  end
  im.EndChild()
end

local function tabSequences()
  table.clear(selectableControllers)

  im.BeginChild1("sequences", im.ImVec2(150 * im.uiscale[0], 0), im.WindowFlags_ChildWindow)

  for i, sequence in ipairs(sequences) do
    if im.Selectable1(sequence.name, selected.sequence == i) then
      selectSequence(i)
    end
  end
  im.Separator()
  if im.Selectable1("Create...##sequence", false) then
    editor.history:commitAction("Create Sequence", {}, createSequenceActionUndo, createSequenceActionRedo)
  end
  im.EndChild()
  im.SameLine()

  im.BeginChild1("sequenceData", im.ImVec2(0, 0), im.WindowFlags_ChildWindow)
  contentWidth = im.GetContentRegionAvailWidth() * 0.5
  local currSequence = sequences[selected.sequence]
  if currSequence then
    im.TextUnformatted("Current Sequence: "..currSequence.name.." ["..currSequence.id.."]")

    im.SameLine()
    if im.Button("Delete##sequence") then
      local act = currSequence:onSerialize()
      act.deleteIdx = selected.sequence
      editor.history:commitAction("Delete Sequence", act, createSequenceActionRedo, createSequenceActionUndo)
    end

    im.PushItemWidth(contentWidth)
    if editor.uiInputText("Name##sequence", sequenceName, nil, im.InputTextFlags_EnterReturnsTrue) then
      currSequence.name = ffi.string(sequenceName)
    end
    im.PopItemWidth()

    local var = im.FloatPtr(currSequence.startTime)
    im.PushItemWidth(inputWidth)
    if im.InputFloat("Start Delay##sequence", var, 0.01, 0.1, "%.2f", im.InputTextFlags_EnterReturnsTrue) then
      currSequence.startTime = var[0]
    end
    im.PopItemWidth()
    im.tooltip("This can also be negative, to skip ahead in the sequence.")

    var = im.BoolPtr(currSequence.startDisabled)
    if im.Checkbox("Start Disabled##sequence", var) then
      currSequence.startDisabled = var[0]
    end
    im.tooltip("If true, this sequence starts with all signals in the off state.")

    im.Dummy(dummyVec)
    im.Separator()
    im.TextUnformatted("Phases")

    im.SameLine()
    im.Button("?")
    im.tooltip("Phases run in order, and each phase plays the assigned controllers. Avoid duplicate controllers across all phases in this sequence.")

    if im.BeginTable("phasesSummary", 3, imFlags.imTable) then
      im.TableSetupScrollFreeze(0, 1)
      im.TableSetupColumn("Phase #", nil, 15)
      im.TableSetupColumn("Duration", nil, 15)
      im.TableSetupColumn("Controllers", nil, 70)
      im.TableHeadersRow()
      im.TableNextColumn()

      for i, phase in ipairs(currSequence.phases) do
        phase.totalDuration = 0

        im.TextUnformatted(tostring(i))
        im.TableNextColumn()

        for _, cd in ipairs(phase.controllerData) do
          local ctrl = elements[cd.id]
          if ctrl then
            ctrl.totalDuration = ctrl.totalDuration or 0
            if ctrl.totalDuration == 0 then
              ctrl.totalDuration = calcControllerDuration(ctrl)
            end

            phase.totalDuration = math.max(phase.totalDuration, ctrl.totalDuration) -- takes the longest controller duration
          end
        end

        im.TextUnformatted(string.format("%0.2f s", phase.totalDuration))
        im.TableNextColumn()

        local ctrlNames = {}
        for _, cd in ipairs(phase.controllerData) do
          if elements[cd.id] then
            table.insert(ctrlNames, elements[cd.id].name)
          end
        end

        im.TextWrapped(table.concat(ctrlNames, ", "))
        im.TableNextColumn()
      end

      im.EndTable()
    end

    im.Dummy(dummyVec)

    if im.Button("Create##sequencePhase") then
      currSequence:createPhase()
      selected.phase = #currSequence.phases
    end
    im.SameLine()
    if im.Button("Delete##sequencePhase") then
      currSequence:deletePhase()
      if not currSequence.phases[selected.phase] then
        selected.phase = math.max(1, #currSequence.phases)
      end
    end

    for i, phase in ipairs(currSequence.phases) do
      local isCurrentPhase = i == selected.phase
      if isCurrentPhase then
        im.PushStyleColor2(im.Col_Button, im.GetStyleColorVec4(im.Col_ButtonActive))
      end
      if im.Button(" "..i.." ") then
        selected.phase = i
      end
      if isCurrentPhase then
        im.PopStyleColor()
      end
      im.SameLine()
    end
    im.Dummy(dummyVec)

    local phase = currSequence.phases[selected.phase]
    if phase then
      im.TextUnformatted("Phase #"..selected.phase)

      im.Dummy(dummyVec)
      im.TextUnformatted("Controllers")
      local count = #phase.controllerData
      if count <= 0 then
        table.insert(phase.controllerData, {id = 0, required = true})
        count = 1
      end

      var = im.IntPtr(count)
      im.PushItemWidth(inputWidth)
      if im.InputInt("Count##phaseController", var, 1) then
        while var[0] > #phase.controllerData do
          table.insert(phase.controllerData, {id = 0, required = true})
        end
        while var[0] < #phase.controllerData do
          table.remove(phase.controllerData, #phase.controllerData)
        end
      end
      im.PopItemWidth()

      local columnWidth = im.GetContentRegionAvailWidth() * 0.5

      im.Columns(2)
      im.SetColumnWidth(1, columnWidth)

      im.TextUnformatted("Controller Name")
      im.NextColumn()
      im.TextUnformatted("Is Required")
      im.SameLine()
      im.Button("?")
      im.tooltip("The sequence phase will advance when all of the required controller cycles are completed.")
      im.NextColumn()

      local controllersDict = {}
      for _, cd in ipairs(phase.controllerData) do
        controllersDict[cd.id] = 1
      end

      for i, cd in ipairs(phase.controllerData) do
        if im.BeginCombo("##phaseControllerName"..i, elements[cd.id] and elements[cd.id].name or "(None)") then
          for _, ctrl in ipairs(controllers) do
            if not controllersDict[ctrl.id] and im.Selectable1(ctrl.name, cd.id == ctrl.id) then -- prevents duplicates by limiting controller selection
              cd.id = ctrl.id
            end
          end
          im.EndCombo()
        end
        im.NextColumn()

        var = im.BoolPtr(cd.required)
        if im.Checkbox("##phaseControllerRequired"..i, var) then
          cd.required = var[0]
        end
        -- this should be reconsidered; it's possible to break the sequence with some combinations of required controllers
        im.NextColumn()
      end

      im.Columns(1)
    end
  end
  im.EndChild()
end

local function tabSimulation()
  if instances[1] then
    if not running then
      if editor.uiIconImageButton(editor.icons.play_arrow, im.ImVec2(30, 30)) then
        checkSignalErrors()
        if not simLogs[1] then
          simulate(true)
        end
      end
    else
      local debugData = trafficSignals.getData()
      if debugData.active then
        if editor.uiIconImageButton(editor.icons.pause, im.ImVec2(30, 30)) then
          core_trafficSignals.setActive(false)
        end
      else
        if editor.uiIconImageButton(editor.icons.play_arrow, im.ImVec2(30, 30)) then
          core_trafficSignals.setActive(true)
        end
      end
      im.SameLine()
      if editor.uiIconImageButton(editor.icons.stop, im.ImVec2(30, 30)) then
        simulate(false)
      end
    end
    if not be:getEnabled() then
      im.SameLine()
      im.TextColored(colorWarning, "Main simulation is currently paused.")
    end

    if running then
      local debugData = trafficSignals.getData()
      if debugData.nextTime then
        im.TextUnformatted("Current timer: "..tostring(string.format("%.2f", debugData.timer)))
        im.TextUnformatted("Next event time: "..tostring(string.format("%.2f", debugData.nextTime)))
      end

      im.Dummy(dummyVec)
      im.Separator()

      local columnWidth = im.GetContentRegionAvailWidth() * 0.25
      im.Columns(4)
      im.SetColumnWidth(0, columnWidth)
      im.SetColumnWidth(1, columnWidth)
      im.SetColumnWidth(2, columnWidth)

      -- table of sequences and current progress
      for i, sequence in ipairs(core_trafficSignals.getSequences()) do
        im.TextUnformatted(sequence.name)

        im.NextColumn()
        im.TextUnformatted("Step: "..sequence.currStep)

        im.NextColumn()
        local currTime = sequence.testTimer or 0
        local maxTime = math.max(1e-6, sequence.sequenceDuration)

        im.ProgressBar(currTime / maxTime, im.ImVec2(im.GetContentRegionAvailWidth(), 0))

        im.NextColumn()
        if im.Button("Advance##simulation"..i) then
          if sequence.active then
            sequence:advance()
          else
            sequence:setActive(true)
          end
        end
        im.SameLine()
        if not sequence.ignoreTimer then
          if im.Button("Pause##simulation"..i) then
            sequence:enableTimer(false)
          end
        else
          if im.Button("Resume##simulation"..i) then
            sequence:enableTimer(true)
          end
        end

        im.NextColumn()
      end
      im.Columns(1)
    end

    im.Separator()

    -- error logs (and other logs, maybe) go here
    for _, s in ipairs(simLogs) do
      im.TextUnformatted(s)
    end
  else
    im.TextUnformatted("Signals need to exist before running simulation.")
    running = false
  end
end

local function debugDraw()
  if running then return end

  local mapNodes = map.getMap().nodes
  local targetSequenceId = instances[selected.signal] and instances[selected.signal].sequenceId or 0 -- sequence id of currently selected signal

  for i, instance in ipairs(instances) do
    local camDist = instance.pos:squaredDistance(core_camera.getPosition())
    if camDist <= square(editor.getPreference("gizmos.visualization.visualizationDrawDistance")) or selected.signal == i then
      instance:drawDebug(true, selected.signal == i, true, getCapColor(elements[instance.sequenceId == targetSequenceId and instance.controllerId or -1]), 5)
      if selected.signal == i and options.showClosestRoad and instance.road and instance.road.n1 then
        local n1, n2 = mapNodes[instance.road.n1], mapNodes[instance.road.n2]
        debugDrawer:drawSquarePrism(n1.pos, n2.pos, Point2F(0.3, n1.radius * 2), Point2F(0.3, n2.radius * 2), debugColors.main)
      end
    end
  end
end

local function initTables() -- runs on first load
  editor.selectEditMode(editor.editModes[editModeName])
  trafficSignals = extensions.core_trafficSignals
  trafficSignals.loadControllerDefinitions(editor.levelPath.."signalControllerDefinitions.json")
  controllerDefinitions = trafficSignals.getControllerDefinitions()

  if not instances[1] then
    setCurrentSignals() -- automatically loads active signals data from map
  end

  selectInstance(selected.signal)
  selectController(selected.controller)
  selectSequence(selected.sequence)

  signalsInitialized = true
end

local function onEditorGui(dt)
  if editor.beginWindow(editModeName, editWindowName, im.WindowFlags_MenuBar) then
    if not signalsInitialized then initTables() end

    inputWidth = 100 * im.uiscale[0]
    mousePos:set(staticRayCast() or vec3())

    im.BeginMenuBar()
    if im.BeginMenu("File") then
      if im.MenuItem1("Load") then
        loadFile() -- only loads the default signal file (should this support other files?)
      end
      if im.MenuItem1("Save") then
        saveFile() -- only saves the default signal file
      end
      if im.MenuItem1("Clear") then
        windowFlags.clear = true
      end
      im.EndMenu()
    end
    if im.BeginMenu("Preferences") then
      local val = im.BoolPtr(options.smartName)
      if im.Checkbox("Smart Naming", val) then
        options.smartName = val[0]
      end
      im.tooltip("Automatically sets names of signals based on their properties.")

      val = im.BoolPtr(options.smartObjectSelection)
      if im.Checkbox("Smart Object Selection Mode", val) then
        options.smartObjectSelection = val[0]
      end
      im.tooltip("Automatically selects all similar traffic light objects when one is selected.")

      val = im.BoolPtr(options.showClosestRoad)
      if im.Checkbox("Draw Closest Road Segment", val) then
        options.showClosestRoad = val[0]
      end
      im.tooltip("Displays the closest road while the current signal is selected.")

      im.EndMenu()
    end

    if im.BeginMenu("Tools") then
      if im.MenuItem1("Check Signal Errors") then
        checkSignalErrors()

        if not simLogs[1] then
          log('I', logTag, "Traffic signals validated with no errors")
          timedTexts.signalsValid = {"Signals validated!", 3}
          timedTexts.signalsInvalid = nil
        else
          log('W', logTag, "Traffic signals validated with errors, see below for details")
          for _, s in ipairs(simLogs) do
            dump(s)
          end
          timedTexts.signalsInvalid = {"Signal errors: "..tostring(#simLogs).."; see Simulation tab", 12}
          timedTexts.signalsValid = nil
        end
      end

      im.EndMenu()
    end

    if timedTexts.save then
      im.SameLine()
      im.TextColored(colorWarning, timedTexts.save[1])
    end
    if timedTexts.renameObjects then
      im.SameLine()
      im.TextColored(colorWarning, timedTexts.renameObjects[1])
    end
    if timedTexts.signalsValid then
      im.SameLine()
      im.TextColored(colorWarning, timedTexts.signalsValid[1])
    end
    if timedTexts.signalsInvalid then
      im.SameLine()
      im.TextColored(colorError, timedTexts.signalsInvalid[1])
    end
    im.EndMenuBar()

    if windowFlags.ctrlDefinitions[0] then
      windowSignalCtrlDefinitions()
    end

    if im.BeginTabBar("Signal Tools") then
      if im.BeginTabItem("Signals", nil, tabFlags[1]) then
        tabInstances()
        im.EndTabItem()
      end
      if im.BeginTabItem("Controllers", nil, tabFlags[2]) then
        tabControllers()
        im.EndTabItem()
      end
      if im.BeginTabItem("Sequences", nil, tabFlags[3]) then
        tabSequences()
        im.EndTabItem()
      end
      if im.BeginTabItem("Simulation", nil, tabFlags[4]) then
        tabSimulation()
        im.EndTabItem()
      end
      im.EndTabBar()
    end
    table.clear(tabFlags)

    if windowFlags.clear then
      im.OpenPopup("Confirm##trafficSignals")
      windowFlags.clear = nil
    end

    if im.BeginPopupModal("Confirm##trafficSignals", nil, im.WindowFlags_AlwaysAutoResize) then
      im.TextUnformatted("Are you sure you want to clear signals data?")
      im.Dummy(dummyVec)

      if im.Button("YES", im.ImVec2(inputWidth, 20 * im.uiscale[0])) then
        resetSignals()
        im.CloseCurrentPopup()
      end
      im.SameLine()
      if im.Button("NO", im.ImVec2(inputWidth, 20 * im.uiscale[0])) then
        im.CloseCurrentPopup()
      end

      im.EndPopup()
    end

    debugDraw()
  end

  for k, v in pairs(timedTexts) do
    if v[2] then
      v[2] = v[2] - dt
      if v[2] <= 0 then timedTexts[k] = nil end
    end
  end

  editor.endWindow()
end

local function onActivate()
  editor.clearObjectSelection()
end

local function onClientEndMission()
  signalsInitialized = false
end

local function onSerialize()
  local data = {options = options, signals = getSerializedSignals()}
  return data
end

local function onDeserialized(data)
  trafficSignals = core_trafficSignals
  options = data.options
  setCurrentSignals(data.signals)
end

local function onWindowMenuItem()
  if not signalsInitialized then initTables() end
  editor.clearObjectSelection()
  editor.showWindow(editModeName)
end

local function onEditorInitialized()
  editor.registerWindow(editModeName, im.ImVec2(540, 600))
  editor.editModes[editModeName] = {
    displayName = editWindowName,
    onActivate = onActivate,
    auxShortcuts = {}
  }
  editor.editModes[editModeName].auxShortcuts[bit.bor(editor.AuxControl_LMB, editor.AuxControl_Shift)] = "Create Signal"
  editor.editModes[editModeName].auxShortcuts[editor.AuxControl_LMB] = "Select"
  editor.addWindowMenuItem(editWindowName, onWindowMenuItem, {groupMenuName = "Gameplay"})
end

M.getCurrentElements = getCurrentElements
M.getCurrentSignals = getCurrentSignals
M.setCurrentSignals = setCurrentSignals
M.loadFile = loadFile
M.saveFile = saveFile

M.onEditorInitialized = onEditorInitialized
M.onWindowMenuItem = onWindowMenuItem
M.onEditorGui = onEditorGui
M.onClientEndMission = onClientEndMission
M.onSerialize = onSerialize
M.onDeserialized = onDeserialized

return M