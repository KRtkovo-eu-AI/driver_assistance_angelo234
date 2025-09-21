-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

M.dependencies = {"tech_CANBus_CANBusPeak"}

local band = bit.band

local wakeUpMessage = {0x63}
local backlightMessage = {0x00, 0x00}
local initDialMessage = {0x1D, 0xE1, 0x0, 0x0, 0x0, 0x0, 0xDE, 0x0}

local messageIds = {
  receiveTouchData = 0x0bf,
  sendBacklightDimmer = 0x202,
  receiveStateDial = 0x264,
  receiveStateButton = 0x267,
  sendInitDial = 0x273,
  sendWakeUp = 0x563,
  receiveHeartbeat = 0x5e7
}

local messageIdsToIgnore = {
  0x197, --Shifter State
  0x55e, --Shifter Heartbeat
  0x5de, --Shifter Shift Error
  0x65e, --Shifter Diagnostic
  0x3FD, --Shifter Appearance
  0x202, --Backlight
  0x277, --unknown
  0x567, --unknown
  0x667 --unknown
}

local buttonTypes = {
  none = 0xC000,
  menu = 0xC001,
  back = 0xC002,
  option = 0xC004,
  audio = 0xC008,
  media = 0xC010,
  navigation = 0xC020,
  telephone = 0xC040,
  dialPOV2Way = 0xDD00,
  dialPOV4Way = 0xDD01,
  dialCenter = 0xDE01
}

local buttonStates = {
  release = 0x0,
  press = 0x1,
  longPress = 0x2,
  povRightPress = 0x21,
  povRightLongPress = 0x22,
  povLeftPress = 0x81,
  povLeftLongPress = 0x82,
  povUpPress = 0x11,
  povUpLongPress = 0x12,
  povDownPress = 0x41,
  povDownLongPress = 0x42
}

local dialStates = {
  active = 0x1,
  inactive = 0x6
}

local dialPositions = {
  minimum = 0,
  maximum = 0xFFFE,
  center = 0x7FFF
}

local buttonTypeLookup = {}
local buttonStateLookup = {}

local canBus
local lastDialPosition = 0

local hardwareIndexLookup = {
  buttons = {
    menu = 0,
    back = 1,
    option = 2,
    audio = 3,
    media = 4,
    navigation = 5,
    telephone = 6,
    dialCenter = 7,
    povLeft = 8,
    povRight = 9,
    povUp = 10,
    povDown = 11,
    dialCW = 12,
    dialCCW = 13
  },
  axes = {
    touchX1 = 0,
    touchY1 = 1
  }
}

local buttonHardwareLookup = {
  [buttonTypes.menu] = "menu",
  [buttonTypes.back] = "back",
  [buttonTypes.option] = "option",
  [buttonTypes.audio] = "audio",
  [buttonTypes.media] = "media",
  [buttonTypes.navigation] = "navigation",
  [buttonTypes.telephone] = "telephone",
  [buttonTypes.dialCenter] = "dialCenter"
}

local povButtonHardwareLookup = {
  [buttonStates.povRightPress] = "povRight",
  [buttonStates.povLeftPress] = "povLeft",
  [buttonStates.povUpPress] = "povUp",
  [buttonStates.povDownPress] = "povDown"
}

local buttonValueLookup = {
  [buttonStates.release] = 0,
  [buttonStates.press] = 1,
  [buttonStates.povRightPress] = 1,
  [buttonStates.povLeftPress] = 1,
  [buttonStates.povUpPress] = 1,
  [buttonStates.povDownPress] = 1
}

local hardwareState = {
  buttons = {
    menu = 0,
    back = 0,
    option = 0,
    audio = 0,
    media = 0,
    navigation = 0,
    telephone = 0,
    dialCenter = 0,
    povLeft = 0,
    povRight = 0,
    povUp = 0,
    povDown = 0,
    dialCW = 0,
    dialCCW = 0
  },
  axes = {
    touchX1 = 0,
    touchY1 = 0
  }
}

local lastHardwareState = deepcopy(hardwareState)

local virtualInputDeviceName = "CAN-Bus Controller"
local virtualInputVidPid = 0xbea90812
local virtualInputNumberOfAxes = 2
local virtualInputNumberOfButtons = 14
local virtualInputNumberOfPOVs = 0

local virtualInputDeviceInstance
local hasRegisteredVirtualInputDevice

local updateTimer = 0
local updateTime = 1 / 100

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
    be:queueObjectLua(%d, 'extensions.tech_CANBus_ProjectBavariaController.virtualInputCallback('..(deviceId or 'nil')..','..serialize(deviceInfo)..')')
    ]]
  local id = objectId
  local cmdString = string.format(cmdStringTemplate, virtualInputVidPid, id)
  log("I", "ProjectBavariaController.requestExistingVirtualInput", "Requesting existing virtual input device for vidpid: " .. virtualInputVidPid)
  obj:queueGameEngineLua(cmdString)
end

local function registerNewVirtualInput()
  local cmdStringTemplate = [[
    local deviceId, deviceInfo = core_input_virtualInput.createDevice(%q, %d, %d, %d, %d, true)
    be:queueObjectLua(%d, 'extensions.tech_CANBus_ProjectBavariaController.virtualInputCallback('..deviceId..','..serialize(deviceInfo)..')')
    ]]
  local id = objectId
  local cmdString = string.format(cmdStringTemplate, virtualInputDeviceName, virtualInputVidPid, virtualInputNumberOfAxes, virtualInputNumberOfButtons, virtualInputNumberOfPOVs, id)
  log("I", "ProjectBavariaController.registerNewVirtualInput", "Registering new virtual input device for vidpid: " .. virtualInputVidPid)
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
  log("I", "ProjectBavariaController.virtualInputCallback", string.format("Received virtual input callback with deviceInstance %d and device info %s", deviceInstance or -1, dumps(deviceInfo)))

  if deviceInstance and deviceInfo then
    --callback from requesting an existing virtual input device or creating a new one
    if deviceInfo[1] == virtualInputDeviceName and deviceInfo[3] == virtualInputNumberOfAxes and deviceInfo[4] == virtualInputNumberOfButtons and deviceInfo[5] == virtualInputNumberOfPOVs then
      --found existing virtual input device
      log("I", "ProjectBavariaController.virtualInputCallback", "Received existing virtual input device with vidpid " .. virtualInputVidPid .. " is valid and will be used")
      virtualInputDeviceInstance = deviceInstance
    else
      log("W", "ProjectBavariaController.virtualInputCallback", "Received existing virtual input device seems incorrect, deleting and recreating...")
      --existing virtual input device seems incorrect, delete it and register a new one
      deleteVirtualInput(deviceInstance)
      registerNewVirtualInput()
    end
  else
    hasRegisteredVirtualInputDevice = false
    virtualInputDeviceInstance = nil
    log("I", "ProjectBavariaController.virtualInputCallback", "No matching virtual input device exists, creating a new one...")
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

local function initDial(position)
  local positionByte1, positionByte2 = canBus.twoBytes(position)
  initDialMessage[5] = positionByte1
  initDialMessage[6] = positionByte2
  canBus.sendCANMessage(messageIds.sendInitDial, initDialMessage, "sendInitDial")
  lastDialPosition = position
end

local function applyButtonInputs(buttonType, buttonState)
  --print(buttonTypeLookup[buttonType] .. " -> " .. buttonStateLookup[buttonState])

  if buttonType == buttonTypes.dialPOV2Way or buttonType == buttonTypes.dialPOV4Way then
    local povButton = povButtonHardwareLookup[buttonState]
    local povButtonValue = buttonValueLookup[buttonState]
    if povButtonValue ~= 0 then
      hardwareState.buttons[povButton] = povButtonValue
    else
      hardwareState.buttons.povLeft = 0
      hardwareState.buttons.povRight = 0
      hardwareState.buttons.povUp = 0
      hardwareState.buttons.povDown = 0
    end
  else
    local button = buttonHardwareLookup[buttonType]
    local value = buttonValueLookup[buttonState]
    if button then
      hardwareState.buttons[button] = value
    end
  end

  emitInputs()
end

local function applyDialInputs(dialPosition)
  --print("Dial: " .. dialPosition)
  local button = dialPosition > lastDialPosition and "dialCW" or "dialCCW"

  hardwareState.buttons[button] = 1
  emitInputs()
  hardwareState.buttons[button] = 0
  emitInputs()

  lastDialPosition = dialPosition

  if dialPosition >= dialPositions.maximum or dialPosition <= dialPositions.minimum then
    initDial(dialPositions.center)
  end
end

local function applyTouchInputs(x1Pos, x1Quadrant, y1Pos, x2Pos, x2Quadrant, y2Pos, touchState, counter)
  local x1Sign = band(x1Quadrant, 0x01)
  local x1 = x1Sign <= 0 and (-255 + x1Pos) or x1Pos
  local y1 = y1Pos - 15
  local x2Sign = band(x2Quadrant, 0x01)
  local x2 = x2Sign <= 0 and (-255 + x2Pos) or x2Pos
  local y2 = y2Pos - 15
  if touchState == 0x11 then
    x1 = 0
    y1 = 0
    x2 = 0
    y2 = 0
  end
  if touchState == 0x10 then
    x2 = 0
    y2 = 0
  end

  hardwareState.axes.touchX1 = linearScale(x1, -255, 255, 0, 1)
  hardwareState.axes.touchY1 = linearScale(y1, -15, 15, 0, 1)

  emitInputs()

  --dump({x1, y1})
  --dumpByte(band(x1Quadrant, 0xF0))
end

local function readInputs(msg)
  if msg.ID == messageIds.receiveStateButton then
    local typeByte1 = msg.DATA[4]
    local typeByte2 = msg.DATA[5]
    local stateByte = msg.DATA[3]
    applyButtonInputs(canBus.combineTwoBytes(typeByte1, typeByte2), stateByte)
  elseif msg.ID == messageIds.receiveStateDial then
    local position = canBus.combineTwoBytes(msg.DATA[4], msg.DATA[3])
    applyDialInputs(position)
  elseif msg.ID == messageIds.receiveTouchData then
    local counter = msg.DATA[0]
    local x1Pos = msg.DATA[1]
    local x1Quadrant = msg.DATA[2]
    local y1Pos = msg.DATA[3]
    local touchState = msg.DATA[4]
    local x2Pos = msg.DATA[5]
    local x2Quadrant = msg.DATA[6]
    local y2Pos = msg.DATA[7]

    applyTouchInputs(x1Pos, x1Quadrant, y1Pos, x2Pos, x2Quadrant, y2Pos, touchState, counter)
  elseif msg.ID == messageIds.receiveHeartbeat then --"heartbeat", relevant for initializing the dial
    local dialState = msg.DATA[4]
    if dialState == dialStates.inactive then
      initDial(dialPositions.center) --set dial to middle range of int16
    end
    if not hardwareDetected then
      hardwareDetected = true
      log("I", "ProjectBavariaController.readInputs", "Communication with Project Bavaria Controller established")
    end
  else
    -- for _, messageId in ipairs(messageIdsToIgnore) do
    --   if msg.ID == messageId then
    --     return
    --   end
    -- end
    --log("W", "ProjectBavariaController.readInputs", "Unknown message received: " .. canBus.dumpsMsg(msg))
  end
end

local function updateHardwareState(dt)
  canBus.sendCANMessage(messageIds.sendWakeUp, wakeUpMessage, "wakeUp") --keep the controller awake

  local backlight = true --electrics.values.lights > 0.5
  backlightMessage[1] = backlight and 0xFD or 0x00
  canBus.sendCANMessage(messageIds.sendBacklightDimmer, backlightMessage, "backlightDimmer") --set backlight
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
  log("I", "ProjectBavariaController.onExtensionLoaded", "CANBus Controller extension loaded")
  buttonTypeLookup = {}
  buttonStateLookup = {}
  for key, value in pairs(buttonTypes) do
    buttonTypeLookup[value] = key
  end
  for key, value in pairs(buttonStates) do
    buttonStateLookup[value] = key
  end
  canBus = extensions.tech_CANBus_CANBusPeak
  if canBus then
    log("D", "ProjectBavariaController.onExtensionLoaded", "CANBus extension found")
    canBus.registerCANMessageCallback("ProjectBavariaController", canMessageReceived)
    if not canBus.isConnected then
      log("D", "ProjectBavariaController.onExtensionLoaded", "CANBus extension is not connected, connecting now...")
      local connectionResult = canBus.initCANBus()
      if connectionResult ~= canBus.errorCodes.OK then
        log("E", "ProjectBavariaController.onExtensionLoaded", "Non-OK init result for CAN Bus, shutting down... Result: " .. canBus.errorCodeLookup[connectionResult])
        canBus = nil
      end
    end
  else
    log("E", "ProjectBavariaController.onExtensionLoaded", "CANBus extension NOT found CANBus Controller won't work...")
  end
end

-- public interface
M.onExtensionLoaded = onExtensionLoaded
M.updateGFX = updateGFX

M.virtualInputCallback = virtualInputCallback

return M
