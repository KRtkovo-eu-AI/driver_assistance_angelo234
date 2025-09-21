-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

M.wheels = {}
M.cumulativeGearRatio = 0
M.engineData = {}
M.stabilityCoef = 250

M.currentGravity = obj:getGravity()
M.invCurrentGravity = 1 / M.currentGravity
M.currentEnvTemperature = obj:getEnvTemperature()
M.invCurrentEnvTemperature = 1 / M.currentEnvTemperature
M.currentEnvTemperatureCelsius = M.currentEnvTemperature - 273.15
M.invCurrentEnvTemperatureCelsius = 1 / M.currentEnvTemperatureCelsius
M.currentEnvPressure = obj:getEnvPressure()
M.invCurrentEnvPressure = 1 / M.currentEnvPressure

--we need to initialize this with {0} so that powertrain.torqueReactionCoefs[1] works, 1 in this case is the default torsionReactorID for all wheels/rotators,
--which we need to use in case that powertrain does not init at all/correctly (trailers, mods with old stuff, etc). The {0} is only used when powertrain does not init!
M.torqueReactionCoefs = {0}
local torqueReactionCoefs2 = {0}
local torsionReactorList = {}
local torsionReactorCount = 0
local torsionReactorIndexes = {}

local max = math.max
local min = math.min
local tableSize = tableSize
local log = log
local pi = math.pi
local twoPi = pi * 2
local visualShaftAngleCoef = 180 / pi

--local warningState = {}
local vehiclePath = nil

local hasPowertrain = false
local canResetDevices = false

local deviceFactories = nil
local availableDeviceFactories = nil
local factoryBlackList = {combustionEngineThermals = true, supercharger = true, turbocharger = true, nitrousOxideInjection = true, hydraulicCylinder = true}
local powertrainDevices = {} --keeps track of all available powertrain devices, also used as LUT
local orderedDevices = {}
local deviceCount = 0
local beamBrokenEvents = {}
local beamBrokenEventCount = 0
local deviceJbeamData = {}

local breakTriggerBeams = {} --shaft break beam cache
local breakTriggerBeamLookup = {} --beam name vs cid lookup
local previousDeviceModes = {}

local dummyShaftCounter = 0

local engineSoundIDCounter = -1
local deviceStream = {}
local streamData = {devices = deviceStream}
local outputTorqueStr = {}
local outputAVStr = {}

for i = 0, 10 do
  outputTorqueStr[i] = "outputTorque" .. tostring(i)
  outputAVStr[i] = "outputAV" .. tostring(i)
end

local wheelPropulsionDevices = {}

local serializeInfoRes = {}
local data
local function serializeDevicesInfo()
  if tableSize(powertrainDevices) < tableSize(serializeInfoRes) then
    table.clear(serializeInfoRes)
  end
  local i = 1
  for _, device in pairs(powertrainDevices) do
    serializeInfoRes[i] = serializeInfoRes[i] or {}
    data = serializeInfoRes[i]
    data.name = device.name
    data.type = device.type
    data.engineLoad = device.engineLoad
    data.forcedInductionCoef = device.forcedInductionCoef
    data.intakeAirDensityCoef = device.intakeAirDensityCoef
    data.diffAngle = device.diffAngle
    data.outputAV2 = device.outputAV2
    data.outputTorque2 = device.outputTorque2
    data.primaryOutputAVName = device.primaryOutputAVName
    data.secondaryOutputAVName = device.secondaryOutputAVName
    data.primaryOutputTorqueName = device.primaryOutputTorqueName
    data.secondaryOutputTorqueName = device.secondaryOutputTorqueName
    data.gearDamages = device.gearDamages
    data.clutchAngle = device.clutchAngle
    data.torqueDiff = device.torqueDiff
    data.lockSpring = device.lockSpring
    data.lockDamp = device.lockDamp
    data.lockupClutchAngle = device.lockupClutchAngle
    data.lockupClutchSpring = device.lockupClutchSpring
    data.lockupClutchDamp = device.lockupClutchDamp
    data.parkClutchAngle = device.parkClutchAngle
    data.oneWayTorqueSmoother = device.oneWayTorqueSmoother and device.oneWayTorqueSmoother:value() or nil
    data.parkLockSpring = device.parkLockSpring
    data.clutchAngle1 = device.clutchAngle1
    data.clutchAngle2 = device.clutchAngle2
    data.lockSpring1 = device.lockSpring1
    data.lockSpring2 = device.lockSpring2
    data.lockDamp1 = device.lockDamp1
    data.lockDamp2 = device.lockDamp2
    data.gearRatio1 = device.gearRatio1
    data.gearRatio2 = device.gearRatio2
    data.inputAV = device.inputAV
    data.outputAV1 = device.outputAV1
    data.outputTorque1 = device.outputTorque1
    data.isBroken = device.isBroken
    data.mode = device.mode
    data.virtualMassAV = device.virtualMassAV
    data.isPhysicallyDisconnected = device.isPhysicallyDisconnected
    data.gearRatio = device.gearRatio
    data.cumulativeGearRatio = device.cumulativeGearRatio
    data.cumulativeInertia = device.cumulativeInertia
    i = i + 1
  end
  return serialize(serializeInfoRes)
end

local function dumpsDeviceData(device)
  if device then
    local deviceData = deepcopy(device)
    if deviceData.children then
      deviceData.children = {}
      for _, v in pairs(device.children) do
        table.insert(deviceData.children, v.name or "unknown")
      end
    end
    if deviceData.clutchChildren then
      for k, child in ipairs(deviceData.clutchChildren) do
        deviceData.clutchChildren[k] = child.name
      end
    end
    if deviceData.clutchChild then
      deviceData.clutchChild = deviceData.clutchChild.name
    end
    if deviceData.parent then
      deviceData.parent = deviceData.parent.name or "unknown"
    end
    return dumps(deviceData)
  else
    return "nil"
  end
end

local function sendDeviceData()
  if streams.willSend("powertrainDeviceData") then
    for _, v in pairs(powertrainDevices) do
      deviceStream[v.name] = deviceStream[v.name] or {outputTorque = {}, outputAV = {}, isBroken = false, uiSimpleModeControl = v.uiSimpleModeControl}
      local di = 1
      for i, _ in pairs(v.outputPorts) do
        deviceStream[v.name].outputTorque[di] = v[outputTorqueStr[i]]
        deviceStream[v.name].outputAV[di] = v[outputAVStr[i]]
        di = di + 1
      end
      deviceStream[v.name].currentMode = (v.availableModes and #v.availableModes > 1) and v.mode or nil
      deviceStream[v.name].isBroken = v.isBroken or false
    end
    -- dump(deviceStream)
    gui.send("powertrainDeviceData", streamData)
  end
end

local function updateGFX(dt)
  M.currentGravity = obj:getGravity()
  M.invCurrentGravity = 1 / M.currentGravity
  M.currentEnvTemperature = obj:getEnvTemperature()
  M.invCurrentEnvTemperature = 1 / M.currentEnvTemperature
  M.currentEnvTemperatureCelsius = M.currentEnvTemperature - 273.15
  M.invCurrentEnvTemperatureCelsius = 1 / M.currentEnvTemperatureCelsius
  M.currentEnvPressure = obj:getEnvPressure()
  M.invCurrentEnvPressure = 1 / M.currentEnvPressure

  for i = 1, deviceCount, 1 do
    local device = orderedDevices[i]

    for _, deformGroupData in ipairs(device.deformGroups) do
      device.deformGroupDamages[deformGroupData.groupType] = 0
      for _, deformGroup in ipairs(deformGroupData.groupNames) do
        if beamstate.deformGroupDamage[deformGroup] then
          device.deformGroupDamages[deformGroupData.groupType] = device.deformGroupDamages[deformGroupData.groupType] + beamstate.deformGroupDamage[deformGroup].damage
        end
      end
      local currentDamage = device.deformGroupDamages[deformGroupData.groupType]
      local lastDamage = device.deformGroupLastDamages[deformGroupData.groupType]
      if currentDamage > lastDamage then
        --print(string.format("Damage detected: %s:%s -> %.4f (%.4f)", device.name, deformGroupData.groupType, currentDamage - lastDamage, currentDamage))
        if device.applyDeformGroupDamage then
          device:applyDeformGroupDamage(currentDamage - lastDamage, deformGroupData.groupType)
        end
        device.deformGroupLastDamages[deformGroupData.groupType] = currentDamage
      end
    end

    --profilerPushEvent(orderedDevices[i].name .. ":updateGFX")
    if device.updateGFX then
      device:updateGFX(dt)
    end
    --profilerPopEvent(orderedDevices[i].name .. ":updateGFX")

    --profilerPushEvent(orderedDevices[i].name .. ":updateSounds")
    if device.updateSounds then
      device:updateSounds(dt)
    end
    --profilerPopEvent(orderedDevices[i].name .. ":updateSounds")
    if device.electricsName and device.visualShaftAngle then --only take care of devices that are meant to have a public angle
      device.visualShaftAngle = (device.visualShaftAngle + device[device.visualShaftAVName] / device.gearRatio * dt) % twoPi
      electrics.values[device.electricsName] = device.visualShaftAngle * visualShaftAngleCoef
    end
  end

  sendDeviceData()
end

local function updateGFXLastStage(dt)
  for i = 1, deviceCount, 1 do
    local device = orderedDevices[i]
    if device.updateGFXLastStage then
      device:updateGFXLastStage(dt)
    end
  end
end

local function update(dt)
  M.torqueReactionCoefs, torqueReactionCoefs2 = torqueReactionCoefs2, M.torqueReactionCoefs

  --performanceLogger.startMeasurement("speeds")
  for i = deviceCount, 1, -1 do
    --profilerPushEvent(orderedDevices[i].name .. ":velocityUpdate")
    orderedDevices[i]:velocityUpdate(dt)
    --profilerPopEvent(orderedDevices[i].name .. ":velocityUpdate")
  end
  --performanceLogger.measureAverage("speeds", 10000, false)

  --performanceLogger.startMeasurement("torques")
  for i = 1, deviceCount, 1 do
    --profilerPushEvent(orderedDevices[i].name .. ":torqueUpdate")
    orderedDevices[i]:torqueUpdate(dt)
    --profilerPopEvent(orderedDevices[i].name .. ":torqueUpdate")
  end
  --performanceLogger.measureAverage("torques", 10000, false)

  local trCoefs = M.torqueReactionCoefs
  for i = 1, torsionReactorCount do
    trCoefs[i] = torsionReactorList[i].outputTorque1 / (trCoefs[i] + 1e-30)
    torqueReactionCoefs2[i] = 0
  end
end

local function sendDeviceTree()
  if not playerInfo.firstPlayerSeated then
    return
  end

  local maxPower = 0
  local maxTorque = 0
  local devices = {}
  for _, d in pairs(powertrainDevices) do
    if d.parent.isFake then
      maxPower = max(maxPower, d.maxPower or 0)
      maxTorque = max(maxTorque, (d.maxTorque or 0) * (d.maxCumulativeGearRatio or 1))
    end
    local device = {
      type = d.visualType or d.type,
      modes = (d.availableModes and #d.availableModes > 1) and d.availableModes or nil,
      pos = d.visualPosition,
      children = {}
    }
    if d.children then
      local inverseMap = {}
      for _, d1 in pairs(d.children) do
        inverseMap[d1.inputIndex] = d1.name
      end
      for i, _ in pairs(d.outputPorts) do
        table.insert(device.children, inverseMap[i])
      end
    end
    device.currentMode = (v.availableModes and #v.availableModes > 1) and v.mode or nil
    devices[d.name] = device
  end

  --dump(devices)
  guihooks.trigger("PowertrainDeviceTreeChanged", {devices = devices, maxPower = maxPower, maxTorque = maxTorque})
end

local function sendTorqueData()
  if playerInfo.firstPlayerSeated then
    for _, device in pairs(powertrainDevices) do
      if device.sendTorqueData then
        device:sendTorqueData()
      end
    end
  end
end

local function updateSimpleControlButtons()
  for _, device in pairs(powertrainDevices) do
    if device.updateSimpleControlButtons then
      device:updateSimpleControlButtons()
    end
  end
end

local function calculateTreeInertia()
  --iterate starting at the wheels for various calculations throughout the tree(s)
  for i = deviceCount, 1, -1 do
    orderedDevices[i]:calculateInertia()
    --log("D", "powertrain.calculateTreeInertia", string.format("Cumulative downstream inertia for %s: %.3f", orderedDevices[i].name, orderedDevices[i].cumulativeInertia))
  end
end

local function validatePowertrain()
  for i = deviceCount, 1, -1 do
    if orderedDevices[i].validate and not orderedDevices[i]:validate() then
      log("E", "powertrain.init", "Failed to validate powertrain device. Look above for more information. Aborting powertrain init!")
      return
    end
  end
end

local function makeDummyShaft(name, inputName, inputIndex)
  local shaft = {
    name = name,
    inputName = inputName,
    inputIndex = inputIndex,
    type = "shaft",
    gearRatio = 1,
    cumulativeGearRatio = 1,
    friction = 1,
    isPhysicallyDisconnected = true
  }

  deviceJbeamData[name] = shaft

  return shaft
end

local function buildDeviceTree(t)
  if t.parent then
    t.cumulativeGearRatio = t.gearRatio * t.parent.cumulativeGearRatio
  end
  M.cumulativeGearRatio = max(M.cumulativeGearRatio, t.cumulativeGearRatio)

  if t.requiredExternalInertiaOutputs then
    for _, index in pairs(t.requiredExternalInertiaOutputs) do
      local hasMatchingChild = false
      for _, child in pairs(t.children or {}) do
        if child.inputIndex == index then
          hasMatchingChild = true
          break
        end
      end

      if not hasMatchingChild then
        if not deviceFactories["shaft"] then
          deviceFactories["shaft"] = require(availableDeviceFactories["shaft"])
        end
        log("W", "powertrain.buildDeviceTree", string.format("Adding a dummy shaft to device '%s' on output '%d'", t.name, index))
        t.children = t.children or {}
        local dummyShaft = deviceFactories["shaft"].new(makeDummyShaft("dummyShaft" .. tostring(dummyShaftCounter), t.name, index))
        dummyShaftCounter = dummyShaftCounter + 1
        dummyShaft.parent = t
        table.insert(t.children, dummyShaft)
      end
    end
  end

  --check how many of our children ARE actually connected properly and adjust their parent if they aren't
  t.connectedChildrenCount = tableSize(t.children)
  if t.children then
    for _, v in pairs(t.children) do
      if not t.outputPorts[v.inputIndex] then
        v.parent = nil
        t.connectedChildrenCount = t.connectedChildrenCount - 1
        log("E", "powertrain.buildDeviceTree", string.format("Can't add child (%q) to parent (%q) on port %d, parent does not have a matching output port", v.name, t.name, v.inputIndex))
      end
    end
  end

  --check if we actually have a parent and if we have (properly connected) cildren or a connected wheel
  if t.parent and (t.connectedChildrenCount > 0 or t.connectedWheel) then
    --only if the above are true our device is physically connected to something else
    t.isPhysicallyDisconnected = false
    --we have a proper parent, so send down the propulsion info
    t.isPropulsed = t.parent.isPropulsed or false
  end

  if t.connectedWheel and t.isPropulsed then
    M.wheels[t.connectedWheel].isPropulsed = true
  end

  powertrainDevices[t.name] = t
  table.insert(orderedDevices, t)

  if t.children then
    local keys = tableKeysSorted(t.children)
    for _, k in ipairs(keys) do
      local v = t.children[k]
      buildDeviceTree(v)
    end
  end
end

local function init()
  M.update = nop

  M.cumulativeGearRatio = 0
  M.engineData = {}
  --warningState = {}

  orderedDevices = {}
  powertrainDevices = {}
  breakTriggerBeams = {}
  breakTriggerBeamLookup = {}

  deviceCount = 0
  dummyShaftCounter = 0
  engineSoundIDCounter = -1
  deviceFactories = {}

  M.currentGravity = obj:getGravity()
  M.invCurrentGravity = 1 / M.currentGravity
  M.currentEnvTemperature = obj:getEnvTemperature()
  M.invCurrentEnvTemperature = 1 / M.currentEnvTemperature
  M.currentEnvTemperatureCelsius = M.currentEnvTemperature - 273.15
  M.invCurrentEnvTemperatureCelsius = 1 / M.currentEnvTemperatureCelsius
  M.currentEnvPressure = obj:getEnvPressure()
  M.invCurrentEnvPressure = 1 / M.currentEnvPressure

  if not availableDeviceFactories then
    availableDeviceFactories = {}
    local globalDirectory = "lua/vehicle/powertrain"
    local vehicleDirectory = vehiclePath .. "lua/powertrain"
    local globalFiles = FS:findFiles(globalDirectory, "*.lua", -1, true, false)
    local vehicleFiles = FS:findFiles(vehicleDirectory, "*.lua", -1, true, false)
    local files = arrayConcat(globalFiles, vehicleFiles)

    if files then
      for _, filePath in ipairs(files) do
        local _, file, _ = path.split(filePath)
        local fileName = file:sub(1, -5)
        if not factoryBlackList[fileName] then
          local deviceFactoryPath = "powertrain/" .. fileName
          availableDeviceFactories[fileName] = deviceFactoryPath
        end
      end
    else
      log("E", "powertrain.init", "Can't load powertrain device factories, looking for directory: " .. tostring(globalDirectory))
    end
  end

  --TODO FIXME: This is a hack to get the combustionEngineMF working
  availableDeviceFactories["combustionEngineMF"] = "powertrain/combustionEngineMF/combustionEngineMF"

  --dump(availableDeviceFactories)

  M.wheels = {}
  for i = 0, wheels.wheelRotatorCount - 1 do
    local wheel = wheels.wheelRotators[i]
    M.wheels[wheel.name] = wheel
  end

  if v.data.powertrain then
    local count = tableSize(v.data.powertrain)
    if count <= 0 then
      log("W", "powertrain.init", "Found empty powertrain section. Aborting powertrain init!")
      return
    end

    local deviceLookup = {}
    deviceJbeamData = {}
    for _, jbeamData in pairs(deepcopy(v.data.powertrain)) do
      tableMergeRecursive(jbeamData, v.data[jbeamData.name] or {})

      --we need these during the tree building, so we need to init them right now
      jbeamData.gearRatio = jbeamData.gearRatio or 1
      jbeamData.cumulativeGearRatio = jbeamData.gearRatio
      --all devices start out as physically disconnected, when we walk through the tree later we can see which actually are connected
      jbeamData.isPhysicallyDisconnected = true

      if availableDeviceFactories[jbeamData.type] and not deviceFactories[jbeamData.type] then
        local deviceFactory = require(availableDeviceFactories[jbeamData.type])
        deviceFactories[jbeamData.type] = deviceFactory
      end

      --load our actual device via the device factory
      if deviceFactories[jbeamData.type] then
        local device = deviceFactories[jbeamData.type].new(jbeamData)
        device.uiName = jbeamData.uiName or device.name
        device.uiSimpleModeControl = jbeamData.uiSimpleModeControl == nil and true or jbeamData.uiSimpleModeControl
        if type(jbeamData.visualPositionRelativeParent) == "table" and tableSize(jbeamData.visualPositionRelativeParent) == 3 then
          device.visualPositionRelativeParent = {
            x = jbeamData.visualPositionRelativeParent[1],
            y = jbeamData.visualPositionRelativeParent[2],
            z = jbeamData.visualPositionRelativeParent[3]
          }
        end
        if type(jbeamData.visualPositionRelativeChildren) == "table" and tableSize(jbeamData.visualPositionRelativeChildren) > 0 then
          for _, childRelativePosition in pairs(jbeamData.visualPositionRelativeChildren) do
            device.visualPositionRelativeChildren = device.visualPositionRelativeChildren or {}
            local pos = {}
            if tableSize(childRelativePosition) == 3 then
              pos = {
                x = childRelativePosition[1],
                y = childRelativePosition[2],
                z = childRelativePosition[3]
              }
            end

            table.insert(device.visualPositionRelativeChildren, pos)
          end
        end
        deviceLookup[device.name] = device
        deviceJbeamData[device.name] = jbeamData
      else
        log("E", "powertrain.init", "Found unknown powertrain device type: " .. jbeamData.type)
        log("E", "powertrain.init", "Powertrain will not work!")
        return
      end
    end

    --dump(deviceFactories)
    local devicesSorted = tableKeysSorted(deviceLookup)

    for _, deviceName in ipairs(devicesSorted) do
      local device = deviceLookup[deviceName]
      if device.name == device.inputName then
        log("E", "powertrain.init", "You can't link a device to itself. Device name: " .. device.name)
        log("E", "powertrain.init", "Powertrain will not work!")
        return
      end
      if deviceLookup[device.inputName] then
        deviceLookup[device.inputName].children = deviceLookup[device.inputName].children or {}
        device.parent = deviceLookup[device.inputName]
        table.insert(deviceLookup[device.inputName].children, device)
      end
    end

    for _, device in pairs(deviceLookup) do
      if not device.parent then
        buildDeviceTree(device)
      end
    end

    deviceCount = tableSize(powertrainDevices)

    local beamTriggers = {}
    beamBrokenEvents = {}
    wheelPropulsionDevices = {}

    for _, device in pairs(powertrainDevices) do
      device.parent = device.parent or {isFake = true, outputTorque0 = 0, outputTorque1 = 0, outputTorque2 = 0, deviceCategories = {}}
      device.parentOutputAVName = "outputAV" .. tostring(device.inputIndex)
      device.parentOutputTorqueName = "outputTorque" .. tostring(device.inputIndex)
      device.visualShaftAVName = device.visualShaftAVName or "inputAV"

      if device.breakTriggerBeam then
        if type(device.breakTriggerBeam) ~= "table" then
          device.breakTriggerBeam = {device.breakTriggerBeam}
        end
        for _, name in ipairs(device.breakTriggerBeam) do
          beamTriggers[name] = beamTriggers[name] or {}
          table.insert(beamTriggers[name], device.name)
        end
      end

      if device.beamBroke then
        table.insert(beamBrokenEvents, device.name)
      end

      device.deformGroups = {}
      device.deformGroupDamages = {}
      device.deformGroupLastDamages = {}

      local deviceJbeamDataKeysSorted = tableKeysSorted(deviceJbeamData[device.name] or {})
      for _, k in ipairs(deviceJbeamDataKeysSorted) do
        local v = deviceJbeamData[device.name][k]
        if k:sub(1, 12) == "deformGroups" then --check for magic prefix
          local delim = "_" --underscore is used as the delim (eg "deformGroups_turbo")
          if string.byte(k, 13) == 58 then --backwards compat for using : instead of _ for the delim
            delim = ":"
          end
          local splits = split(k, delim)
          if type(v) == "table" then
            local groupType = splits[2] or "main" --if no groupType is specified, use "main", this means that these are equivalent: "deformGroups": [] and "deformGroups_main":[]
            table.insert(device.deformGroups, {groupType = groupType, groupNames = v})
            device.deformGroupDamages[groupType] = 0
            device.deformGroupLastDamages[groupType] = 0
          end
        end
      end

      if device.deviceCategories.engine then
        local attachedWheels = M.getChildWheels(device)
        for _, wheel in ipairs(attachedWheels) do
          wheelPropulsionDevices[wheel.name] = device
        end
      end
    end

    for i = 1, deviceCount do
      local device = orderedDevices[i]
      if device.visualPositionRelativeParent and device.parent and device.parent.visualPosition and not device.visualPosition then
        device.visualPosition = {
          x = device.parent.visualPosition.x + device.visualPositionRelativeParent.x,
          y = device.parent.visualPosition.y + device.visualPositionRelativeParent.y,
          z = device.parent.visualPosition.z + device.visualPositionRelativeParent.z
        }
      end
    end

    for i = deviceCount, 1, -1 do
      local device = orderedDevices[i]
      if device.visualPositionRelativeChildren then
        for childIndex, childRelativePosition in ipairs(device.visualPositionRelativeChildren) do
          if tableSize(childRelativePosition) == 3 and device.children[childIndex] and device.children[childIndex].visualPosition then
            device.visualPosition = {
              x = device.children[childIndex].visualPosition.x + childRelativePosition.x,
              y = device.children[childIndex].visualPosition.y + childRelativePosition.y,
              z = device.children[childIndex].visualPosition.z + childRelativePosition.z
            }
            break
          end
        end
      end
    end

    --    for _,device in pairs(powertrainDevices) do
    --      print(device.name)
    --    end

    beamBrokenEventCount = #beamBrokenEvents

    validatePowertrain()
    calculateTreeInertia()

    --dump(beamTriggers)
    --dump(beamBrokenEvents)
    --    for k,v in pairs(powertrainDevices) do
    --      print(v.name)
    --      print(dumpsDeviceData(v))
    --    end
    --dump(speedOrderedDevices)

    for _, v in pairs(v.data.beams) do
      if v.name and v.name ~= "" and beamTriggers[v.name] then
        breakTriggerBeams[v.cid] = beamTriggers[v.name]
        breakTriggerBeamLookup[v.name] = v.cid
      end
    end

    --dump(breakTriggerBeams)
    --dump(breakTriggerBeamLookup)

    hasPowertrain = true
    canResetDevices = true
    for _, device in pairs(powertrainDevices) do
      local hasReset = device.reset ~= nil
      local hasResetSounds = device.initSounds ~= nil and device.resetSounds ~= nil or true
      canResetDevices = canResetDevices and hasReset and hasResetSounds

      damageTracker.setDamage("powertrain", device.name, device.isBroken or false)
    end

    if not tableIsEmpty(previousDeviceModes) then
      for k, v in pairs(previousDeviceModes) do
        powertrainDevices[k]:setMode(v)
      end
    end

    M.update = update

    --extensions.load("performanceLogger")

    sendDeviceTree()
  end

  M.torqueReactionCoefs = {}
  torqueReactionCoefs2 = {}
  torsionReactorCount = 0
  torsionReactorIndexes = {}
  for _, rotator in pairs(wheels.wheelRotators) do
    if rotator.torsionReactor then
      if not torsionReactorIndexes[rotator.torsionReactor.name] then
        torsionReactorCount = torsionReactorCount + 1
        torsionReactorIndexes[rotator.torsionReactor.name] = torsionReactorCount
      end

      local trIdx = torsionReactorIndexes[rotator.torsionReactor.name]
      rotator.torsionReactorIdx = trIdx
      torsionReactorList[trIdx] = rotator.torsionReactor
      M.torqueReactionCoefs[trIdx] = 0
      torqueReactionCoefs2[trIdx] = 0
    end
  end
end

local function initSounds()
  if not hasPowertrain then
    return
  end

  for _, device in pairs(powertrainDevices) do
    if device.initSounds then
      device:initSounds(deviceJbeamData[device.name])
    end
  end
end

local function reset()
  if not hasPowertrain then
    return
  end

  if not canResetDevices then
    log("W", "powertrain.reset", "One or more powertrain devices do not support dedicated reset, using full init instead!")
    init()
    return
  end

  M.currentGravity = obj:getGravity()
  M.invCurrentGravity = 1 / M.currentGravity
  M.currentEnvTemperature = obj:getEnvTemperature()
  M.invCurrentEnvTemperature = 1 / M.currentEnvTemperature
  M.currentEnvTemperatureCelsius = M.currentEnvTemperature - 273.15
  M.invCurrentEnvTemperatureCelsius = 1 / M.currentEnvTemperatureCelsius
  M.currentEnvPressure = obj:getEnvPressure()
  M.invCurrentEnvPressure = 1 / M.currentEnvPressure

  for _, device in pairs(powertrainDevices) do
    for _, groupData in ipairs(device.deformGroups) do
      device.deformGroupDamages[groupData.groupType] = 0
      device.deformGroupLastDamages[groupData.groupType] = 0
    end
    device:reset(deviceJbeamData[device.name])
    damageTracker.setDamage("powertrain", device.name, device.isBroken or false)
  end

  calculateTreeInertia()
  sendDeviceTree()

  M.torqueReactionCoefs = {}
  torqueReactionCoefs2 = {}
  for _, rotator in pairs(wheels.wheelRotators) do
    if rotator.torsionReactor then
      local trIdx = torsionReactorIndexes[rotator.torsionReactor.name]
      M.torqueReactionCoefs[trIdx] = 0
      torqueReactionCoefs2[trIdx] = 0
    end
  end
end

local function resetSounds()
  if not hasPowertrain then
    return
  end

  if not canResetDevices then
    log("W", "powertrain.resetSounds", "One or more powertrain devices do not support dedicated reset, using full init instead!")
    initSounds()
    return
  end

  for _, device in pairs(powertrainDevices) do
    if device.resetSounds then
      device:resetSounds(deviceJbeamData[device.name])
    end
  end
end

local function breakDevice(device)
  if device.isBroken then
    return
  end
  device:onBreak()

  for _, beamName in ipairs(device.breakTriggerBeam) do
    obj:breakBeam(breakTriggerBeamLookup[beamName])
  end

  guihooks.message({txt = "vehicle.powertrain.deviceBroken", context = {deviceName = device.uiName}}, 10, "vehicle.damage.device." .. device.uiName)
  damageTracker.setDamage("powertrain", device.name, true)
end

local function beamBroke(id)
  for i = 1, beamBrokenEventCount, 1 do
    powertrainDevices[beamBrokenEvents[i]]:beamBroke(id)
  end

  if not breakTriggerBeams[id] then
    return
  end

  local deviceNames = breakTriggerBeams[id]
  --dump(deviceNames)
  for _, name in ipairs(deviceNames) do
    local device = powertrainDevices[name]
    breakDevice(device)
  end
end

local function onCouplerFound(nodeId, obj2id, obj2nodeId, nodeDist)
  for i = 1, deviceCount, 1 do
    local device = orderedDevices[i]
    if device.onCouplerFound then
      device:onCouplerFound(nodeId, obj2id, obj2nodeId, nodeDist)
    end
  end
end

local function onCouplerAttached(nodeId, obj2id, obj2nodeId, attachSpeed, attachEnergy)
  for i = 1, deviceCount, 1 do
    local device = orderedDevices[i]
    if device.onCouplerAttached then
      device:onCouplerAttached(nodeId, obj2id, obj2nodeId, attachEnergy)
    end
  end
end

local function onCouplerDetached(nodeId, obj2id, obj2nodeId, breakForce)
  for i = 1, deviceCount, 1 do
    local device = orderedDevices[i]
    if device.onCouplerDetached then
      device:onCouplerDetached(nodeId, obj2id, obj2nodeId, breakForce)
    end
  end
end

local function setDeviceMode(name, mode)
  local device = powertrainDevices[name]
  if not device then
    return
  end

  previousDeviceModes[name] = mode
  device:setMode(mode)
  guihooks.message(device.uiName .. " Mode: " .. mode, 5, "vehicle.powertrain.diffmode." .. device.name)
end

local function toggleDeviceMode(name)
  local device = powertrainDevices[name]
  if not device then
    return
  end

  local found = false
  local newMode = device.mode
  for _, v in pairs(device.availableModes) do
    if found then
      newMode = v
      found = false
      break
    elseif device.mode == v then
      found = true
    end
  end

  if found then
    newMode = device.availableModes[next(device.availableModes)]
  end

  setDeviceMode(name, newMode)
  return newMode
end

local function toggleDefaultDiffs()
  for _, v in pairs(powertrainDevices) do
    if v.type == "differential" and v.defaultToggle then
      toggleDeviceMode(v.name)
    end
  end
end

local function getDevices()
  return powertrainDevices
end

local function getOrderedDevices()
  return orderedDevices
end

local function getDevice(name)
  return name and powertrainDevices[name] or nil
end

local function getDevicesByType(deviceType)
  local result = {}
  for _, v in pairs(powertrainDevices) do
    if v.type == deviceType then
      table.insert(result, v)
    end
  end
  return result
end

local function getDevicesByCategory(category)
  local result = {}
  for _, v in pairs(powertrainDevices) do
    if v.deviceCategories[category] then
      table.insert(result, v)
    end
  end
  return result
end

local function getChildWheels(parentDevice, outputID)
  outputID = outputID or -1
  if parentDevice.type == "shaft" and parentDevice.wheel then
    return {parentDevice.wheel}
  elseif parentDevice.children and #parentDevice.children > 0 then
    local result = {}
    for _, device in ipairs(parentDevice.children) do
      if device.inputIndex == outputID or outputID < 0 then
        local childWheels = getChildWheels(device, -1)
        for _, childWheel in ipairs(childWheels) do
          table.insert(result, childWheel)
        end
      end
    end
    return result
  end

  return {}
end

local function getPropulsionDeviceForWheel(wheelName)
  return wheelPropulsionDevices[wheelName]
end

local function getPoweredWheelNames()
  local poweredWheels = {}
  for _, wheel in pairs(wheels.wheels) do
    local propulsionDevice = powertrain.getPropulsionDeviceForWheel(wheel.name)
    if propulsionDevice then
      poweredWheels[wheel.name] = propulsionDevice.name
    end
  end
  return poweredWheels
end

local function getAllWheelPropulsionDevices()
  local duplicatedPropulsionDevices = {}

  for _, wheel in pairs(wheels.wheels) do
    local propulsionDevice = powertrain.getPropulsionDeviceForWheel(wheel.name)
    if propulsionDevice then
      duplicatedPropulsionDevices[propulsionDevice.name] = wheel.name
    end
  end
  local propulsionDevices = {}
  for propulsionDevice, _ in pairs(duplicatedPropulsionDevices) do
    table.insert(propulsionDevices, powertrain.getDevice(propulsionDevice))
  end

  return propulsionDevices
end

local function getHydraulicConsumer(consumerName)
  local hydraulicPowerSources = getDevicesByCategory("hydraulicPowerSource")
  for _, powerSource in ipairs(hydraulicPowerSources) do
    if powerSource.connectedConsumers then
      for _, hydraulicConsumer in ipairs(powerSource.connectedConsumers) do
        if hydraulicConsumer.name == consumerName then
          return hydraulicConsumer
        end
      end
    end
  end

  return nil --couldn't find requested consumer
end

local function getPropulsionDeviceForDevice(device)
  local currenDevice = device
  while currenDevice ~= nil do
    if currenDevice.deviceCategories.engine then
      return currenDevice
    else
      currenDevice = currenDevice.parent
    end
  end

  return nil --didn't find anything...
end

local function setVehiclePath(path)
  vehiclePath = path
end

local function getEngineSoundID()
  engineSoundIDCounter = engineSoundIDCounter + 1
  return engineSoundIDCounter
end

local function getPartRelevantDevices(partTypeData)
  local relevantDevices = {}

  for _, partType in ipairs(partTypeData or {}) do
    local split = split(partType, ":")
    if split[1] == "powertrainDevice" then
      local deviceName = split[2]
      table.insert(relevantDevices, {device = deviceName, subSystem = split[3]})
    end
  end
  return relevantDevices
end

local function setPartCondition(partTypeData, odometer, integrity, visual)
  local deviceIntegrity = integrity

  local relevantDevices = getPartRelevantDevices(partTypeData)
  for _, relevantDevice in ipairs(relevantDevices) do
    --print("--> " .. dumps(relevantDevice))
    local device = M.getDevice(relevantDevice.device)
    if device and device.setPartCondition then
      if type(integrity) == "table" then
        deviceIntegrity = integrity.powertrain[device.name]
      end
      device:setPartCondition(relevantDevice.subSystem, odometer, deviceIntegrity, visual)
    end
  end
end

local function getPartCondition(partTypeData)
  local canProvideIntegrityCondition = false
  local canProvideVisualCondition = false
  local powertrainCondition = {integrityValue = 1, integrityState = {}, visualValue = 1, visualState = {}}

  --skip any powertrain wear/damage for the time being, reenable later

  local relevantDevices = getPartRelevantDevices(partTypeData)
  for _, deviceData in ipairs(relevantDevices) do
    local device = M.getDevice(deviceData.device)
    if device and device.getPartCondition then
      local deviceIntegrityValue, deviceIntegrityState = device:getPartCondition(deviceData.subSystem)
      powertrainCondition.integrityState[deviceData.device] = deviceIntegrityState
      powertrainCondition.integrityValue = min(powertrainCondition.integrityValue, deviceIntegrityValue)

      canProvideIntegrityCondition = true
    end
  end

  return powertrainCondition, canProvideIntegrityCondition, canProvideVisualCondition
end

local function isPhysicsStepUsed()
  return hasPowertrain
end

local stateEvents = {}

local function getStateEvents()
  return stateEvents
end

M.init = init
M.reset = reset
M.initSounds = initSounds
M.resetSounds = resetSounds
M.update = nop
M.updateGFX = updateGFX
M.updateGFXLastStage = updateGFXLastStage

M.beamBroke = beamBroke
M.breakDevice = breakDevice

M.onCouplerFound = onCouplerFound
M.onCouplerAttached = onCouplerAttached
M.onCouplerDetached = onCouplerDetached

M.calculateTreeInertia = calculateTreeInertia

M.toggleDefaultDiffs = toggleDefaultDiffs
M.toggleDeviceMode = toggleDeviceMode
M.setDeviceMode = setDeviceMode
M.getOrderedDevices = getOrderedDevices
M.getDevices = getDevices
M.getDevicesByType = getDevicesByType
M.getDevicesByCategory = getDevicesByCategory
M.getDevice = getDevice
M.getChildWheels = getChildWheels
M.getPropulsionDeviceForWheel = getPropulsionDeviceForWheel
M.getPropulsionDeviceForDevice = getPropulsionDeviceForDevice
M.getAllWheelPropulsionDevices = getAllWheelPropulsionDevices
M.getHydraulicConsumer = getHydraulicConsumer
M.getPoweredWheelNames = getPoweredWheelNames

M.dumpsDeviceData = dumpsDeviceData
M.serializeDevicesInfo = serializeDevicesInfo

M.sendDeviceTree = sendDeviceTree
M.sendTorqueData = sendTorqueData
M.updateSimpleControlButtons = updateSimpleControlButtons
M.setVehiclePath = setVehiclePath
M.getEngineSoundID = getEngineSoundID

M.getPartCondition = getPartCondition
M.setPartCondition = setPartCondition

M.getState = nop
M.setState = nop
M.publishStateEvent = nop
M.triggerStateEvent = nop
M.getStateEvents = getStateEvents

M.isPhysicsStepUsed = isPhysicsStepUsed

--function startProfile()
--  require("extensions/p").start("Fplm0i0", "beam-profiler.log")
--end

--function endProfile()
--  require("extensions/p").stop(true)
--end

return M

-------------------------------------------------------------
------ Don't remove, left it here for future reference ------
-------------------------------------------------------------

--[[
deviceSpeedUpdateNameLookup = {
  [shaftUpdateSpeed] = "shaftUpdateSpeed",
  [shaftDisconnectedUpdateSpeed] = "shaftDisconnectedUpdateSpeed",
  [wheelShaftUpdateSpeed] = "wheelShaftUpdateSpeed",
  [wheelShaftDisconnectedUpdateSpeed] = "wheelShaftDisconnectedUpdateSpeed",
  [differentialUpdateSpeed] = "differentialUpdateSpeed",
  [diffConnectorUpdateSpeed] = "diffConnectorUpdateSpeed"
}
deviceTorqueUpdateNameLookup ={
  [shaftUpdateTorque] = "shaftUpdateTorque",
  [shaftDisconnectedUpdateTorque] = "shaftDisconnectedUpdateTorque",
  [wheelShaftUpdateTorque] = "wheelShaftUpdateTorque",
  [wheelShaftDisconnectedUpdateTorque] = "wheelShaftDisconnectedUpdateTorque",
  [differentialOpenUpdateTorque] = "differentialOpenUpdateTorque",
  [differentialLSDUpdateTorque] = "differentialLSDUpdateTorque",
  [differentialViscousLSDUpdateTorque] = "differentialViscousLSDUpdateTorque",
  [differentialLockedUpdateTorque] = "differentialLockedUpdateTorque",
  [diffConnectorLockedUpdateTorque] = "diffConnectorLockedUpdateTorque",
  [diffConnectorViscousUpdateTorque] = "diffConnectorViscousUpdateTorque",
  [diffConnectorDisconnectedUpdateTorque] = "diffConnectorDisconnectedUpdateTorque"
}

function createEnvironment()
  local variables = {}
  local idx = 1
  while true do
    local ln, lv = debug.getupvalue(M.init, idx)
    if ln ~= nil then
      variables[ln] = lv
    else
      break
    end
    idx = 1 + idx
  end
  tableMerge(variables, _G)
  return variables
end

--this works in theory but didn't give us any performance benefits, leaving it in here for future reference
local function compileUpdateMethods()
  local speedUpdateTable = {}
  for i = 1, deviceCount, 1 do
    table.insert(speedUpdateTable, string.format("%s(speedOrderedDevices[%i], dt)", deviceSpeedUpdateNameLookup[speedOrderedDevices[i].speedUpdate], i))
  end
  table.insert(speedUpdateTable, "drivetrain.wheelBasedEngAV = transmissionInputDevice.inputAV")

  local speedUpdateString = "return function(dt) " .. table.concat(speedUpdateTable, ";") .. " end"
  local env = createEnvironment()
  M.updateSpeeds = load(speedUpdateString, nil, "t", env)()


  local torqueUpdateTable = {}

  table.insert(torqueUpdateTable, "transmissionInputDevice.parent.outputTorque0 = drivetrain.torqueTransmission")
  for i = 1, deviceCount, 1 do
    table.insert(torqueUpdateTable, string.format("%s(torqueOrderedDevices[%i], dt)", deviceTorqueUpdateNameLookup[torqueOrderedDevices[i].torqueUpdate], i))
  end

  local torqueUpdateString = "return function(dt) " .. table.concat(torqueUpdateTable, "; ") .. " end"
  M.updateTorques = load(torqueUpdateString, nil, "t", env)()
end
--]]
