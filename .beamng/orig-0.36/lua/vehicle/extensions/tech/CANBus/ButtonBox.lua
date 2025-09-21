-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

M.dependencies = {"tech_CANBus_CANBusPeak"}

local messageIds = {
  receiveIgnitionState = 0x301,
  receiveHeartbeat = 0x300
}

local canBus

local hardwareIndexLookup = {
  buttons = {
    ignition = 0,
    starter = 1
  },
  axes = {}
}

local hardwareState = {
  buttons = {
    ignition = 0,
    starter = 0
  },
  axes = {}
}

local lastHardwareState = deepcopy(hardwareState)

local virtualInputDeviceName = "Button Box"
local virtualInputVidPid = 0xbea90813
local virtualInputNumberOfAxes = 0
local virtualInputNumberOfButtons = 2
local virtualInputNumberOfPOVs = 0

local virtualInputDeviceInstance
local hasRegisteredVirtualInputDevice

local updateTimer = 0
local updateTime = 1 / 50

local hardwareDetected = false

local function emitVirtualInput(objType, objInstance, action, value)
  if not virtualInputDeviceInstance then
    print("no virtual device...")
    return
  end
  obj:queueGameEngineLua(string.format("core_input_virtualInput.emit(%d, %q, %d, %q, %.3f)", virtualInputDeviceInstance, objType, objInstance, action, value))
end

local function requestExistingVirtualInput()
  local cmdStringTemplate = [[
    local deviceId, deviceInfo = core_input_virtualInput.getDeviceInfo(%d)
    be:queueObjectLua(%d, 'extensions.tech_CANBus_ButtonBox.virtualInputCallback('..(deviceId or 'nil')..','..serialize(deviceInfo)..')')
    ]]
  local id = objectId
  local cmdString = string.format(cmdStringTemplate, virtualInputVidPid, id)
  log("I", "ButtonBox.requestExistingVirtualInput", "Requesting existing virtual input device for vidpid: " .. virtualInputVidPid)
  obj:queueGameEngineLua(cmdString)
end

local function registerNewVirtualInput()
  local cmdStringTemplate = [[
    local deviceId, deviceInfo = core_input_virtualInput.createDevice(%q, %d, %d, %d, %d, true)
    be:queueObjectLua(%d, 'extensions.tech_CANBus_ButtonBox.virtualInputCallback('..deviceId..','..serialize(deviceInfo)..')')
    ]]
  local id = objectId
  local cmdString = string.format(cmdStringTemplate, virtualInputDeviceName, virtualInputVidPid, virtualInputNumberOfAxes, virtualInputNumberOfButtons, virtualInputNumberOfPOVs, id)
  log("I", "ButtonBox.registerNewVirtualInput", "Registering new virtual input device for vidpid: " .. virtualInputVidPid)
  obj:queueGameEngineLua(cmdString)
end

local function deleteVirtualInput(deviceInstance)
  local cmdStringTemplate = [[
    core_input_virtualInput.deleteDevice(%d)
    ]]
  local cmdString = string.format(cmdStringTemplate, deviceInstance)
  obj:queueGameEngineLua(cmdString)
end

local function virtualInputCallback(deviceInstance, deviceInfo)
  log("I", "ButtonBox.virtualInputCallback", string.format("Received virtual input callback with deviceInstance %d and device info %s", deviceInstance or -1, dumps(deviceInfo)))

  if deviceInstance and deviceInfo then
    --callback from requesting an existing virtual input device or creating a new one
    if deviceInfo[1] == virtualInputDeviceName and deviceInfo[3] == virtualInputNumberOfAxes and deviceInfo[4] == virtualInputNumberOfButtons and deviceInfo[5] == virtualInputNumberOfPOVs then
      --found existing virtual input device
      log("I", "ButtonBox.virtualInputCallback", "Received existing virtual input device with vidpid " .. virtualInputVidPid .. " is valid and will be used")
      virtualInputDeviceInstance = deviceInstance
    else
      log("W", "ButtonBox.virtualInputCallback", "Received existing virtual input device seems incorrect, deleting and recreating...")
      --existing virtual input device seems incorrect, delete it and register a new one
      deleteVirtualInput(deviceInstance)
      registerNewVirtualInput()
    end
  else
    hasRegisteredVirtualInputDevice = false
    virtualInputDeviceInstance = nil
    log("I", "ButtonBox.virtualInputCallback", "No matching virtual input device exists, creating a new one...")
    registerNewVirtualInput()
  end
end

local function ensureVirtualInputDevice()
  if not virtualInputDeviceInstance and not hasRegisteredVirtualInputDevice then
    requestExistingVirtualInput()
    hasRegisteredVirtualInputDevice = true
  end
end

local function emitInputs()
  for buttonName, buttonValue in pairs(hardwareState.buttons) do
    if lastHardwareState.buttons[buttonName] ~= buttonValue then
      local action = buttonValue > 0 and "down" or "up"
      local value = buttonValue > 0 and 1 or 0
      emitVirtualInput("button", hardwareIndexLookup.buttons[buttonName], action, value)
      lastHardwareState.buttons[buttonName] = buttonValue
    end
  end

  for axisName, axisValue in pairs(hardwareState.axes) do
    if lastHardwareState.axes[axisName] ~= axisValue then
      emitVirtualInput("axis", hardwareIndexLookup.axes[axisName], "change", axisValue)
      lastHardwareState.axes[axisName] = axisValue
    end
  end
end

local function applyButtonInputs(msgData)
  hardwareState.buttons.ignition = msgData[0]
  hardwareState.buttons.starter = msgData[1]

  emitInputs()
end

local function readInputs(msg)
  if msg.ID == messageIds.receiveIgnitionState then
    applyButtonInputs(msg.DATA)
  elseif msg.ID == messageIds.receiveHeartbeat then --"heartbeat"
    if not hardwareDetected then
      hardwareDetected = true
      log("I", "ButtonBox.readInputs", "Communication with ButtonBox established")
    end
  end
end

local function updateHardwareState(dt)
  --nothing to do here atm
end

local function canMessageReceived(msg)
  if playerInfo.firstPlayerSeated then
    readInputs(msg)
  end
end

local function updateGFX(dt)
  if playerInfo.firstPlayerSeated then
    if canBus then
      updateTimer = updateTimer - dt
      if updateTimer <= 0 then
        ensureVirtualInputDevice()
        updateHardwareState(dt)

        updateTimer = updateTimer + updateTime
      end
    end
  end
end

local function onExtensionLoaded()
  log("I", "ButtonBox.onExtensionLoaded", "ButtonBox extension loaded")
  canBus = extensions.tech_CANBus_CANBusPeak
  if canBus then
    log("D", "ButtonBox.onExtensionLoaded", "CANBus extension found")
    canBus.registerCANMessageCallback("ButtonBox", canMessageReceived)
    if not canBus.isConnected then
      log("D", "ButtonBox.onExtensionLoaded", "CANBus extension is not connected, connecting now...")
      local connectionResult = canBus.initCANBus()
      if connectionResult ~= canBus.errorCodes.OK then
        log("E", "ButtonBox.onExtensionLoaded", "Non-OK init result for CAN Bus, shutting down... Result: " .. canBus.errorCodeLookup[connectionResult])
        canBus = nil
      end
    end
  else
    log("E", "ButtonBox.onExtensionLoaded", "CANBus extension NOT found, ButtonBox won't work...")
  end
end

-- public interface
M.onExtensionLoaded = onExtensionLoaded
M.updateGFX = updateGFX

M.virtualInputCallback = virtualInputCallback

return M
