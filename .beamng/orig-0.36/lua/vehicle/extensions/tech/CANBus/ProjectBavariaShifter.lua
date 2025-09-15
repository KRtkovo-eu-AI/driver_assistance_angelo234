-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

M.dependencies = {"tech_CANBus_CANBusPeak"}

local bxor = bit.bxor
local lshift = bit.lshift
local band = bit.band

local shifterModeCRCPayload = {0x0, 0x0, 0x0, 0x0}
local shifterModeMessage = {0x0, 0x0, 0x0, 0x0, 0x0}
local backlightMessage = {0x00, 0x00}

local messageIds = {
  receiveState = 0x197,
  sendBacklightDimmer = 0x202,
  sendShifterMode = 0x3FD,
  receiveShiftError = 0x5de
}

local messageIdsToIgnore = {
  0x65e, --Shifter Diagnostic
  0x55e, --Shifter Heartbeat
  0x267, --Controller Button
  0x264, --Controller Dial
  0x5e7, --Controller Heartbeat
  0x0bf, --Controller Touch
  0x563, --Controller Wakeup
  0x273, --Controller Init Dial
  0x202, --Backlight
  0x277, --unknown
  0x567, --unknown
  0x667 --unknown
}

local gearStateLookup = {
  P = 0x20,
  R = 0x40,
  N = 0x60,
  D = 0x81,
  S = 0x81,
  M = 0x81,
  Off = 0x00
}

local leverStates = {
  idle = 0x0E,
  forwardOne = 0x1E,
  forwardTwo = 0x2E,
  backwardOne = 0x3E,
  backwardTwo = 0x4E,
  forwardMS = 0x5E,
  backwardMS = 0x6E,
  idleMS = 0x7E
}

local leverStateLookup = {}

local parkButtonStates = {
  pressed = 0xD2,
  --pressed = 0xD5, --also pressed?
  idle = 0xC0
}

local parkButtonLookup = {}

local shiftErrorStates = {
  onDown = 0x15,
  onUp = 0x14
}

local crc8FinalXOR = {
  receive = 0x53,
  send = 0x70
}

local canBus

local writeMessageCounter = 0
local lastLeverState

local updateTimer = 0
local updateTime = 1 / 100

local hardwareDetected = false

--J1850-CRC8 variant with initial value: 0x00, polynomial: 0x1D, final XOR: variable
local function crc8(data, xorOutput)
  local crc = 0x0
  for _, b in ipairs(data) do
    crc = bxor(crc, b)
    for _ = 0, 7 do
      if band(crc, 0x80) ~= 0 then
        crc = lshift(crc, 1)
        crc = bxor(crc, 0x1D)
      else
        crc = lshift(crc, 1)
      end
    end
  end
  return band(bxor(crc, xorOutput), 0xFF)
end

local function applyInputs(leverState, parkButtonState)
  local gear = string.sub(electrics.values.gear or "", 1, 1)

  if parkButtonState ~= parkButtonStates.idle and gear ~= "P" then
    controller.mainController.shiftToGearIndex(1)
  else
    if leverState == leverStates.idle then
      if gear == "S" or gear == "M" then
        controller.mainController.shiftToGearIndex(2)
      end
    elseif leverState == leverStates.forwardOne then
      if lastLeverState == leverStates.idle then
        if gear == "P" or gear == "N" then
          controller.mainController.shiftToGearIndex(-1)
        elseif gear == "D" then
          controller.mainController.shiftToGearIndex(0)
        end
      end
    elseif leverState == leverStates.forwardTwo then
      if lastLeverState == leverStates.idle or lastLeverState == leverStates.forwardOne then
        if gear == "P" or gear == "N" or gear == "D" then
          controller.mainController.shiftToGearIndex(-1)
        end
      end
    elseif leverState == leverStates.backwardOne then
      if lastLeverState == leverStates.idle then
        if gear == "P" or gear == "N" then
          controller.mainController.shiftToGearIndex(2)
        elseif gear == "R" then
          controller.mainController.shiftToGearIndex(0)
        end
      end
    elseif leverState == leverStates.backwardTwo then
      if lastLeverState == leverStates.idle or lastLeverState == leverStates.backwardOne then
        if gear == "P" or gear == "R" or gear == "N" then
          controller.mainController.shiftToGearIndex(2)
        end
      end
    elseif leverState == leverStates.idleMS then
      if gear == "D" then
        controller.mainController.shiftToGearIndex(3)
      end
    elseif leverState == leverStates.forwardMS then
      if lastLeverState == leverStates.idleMS then
        if gear == "S" then
          controller.mainController.shiftToGearIndex(6)
        elseif gear == "M" and electrics.values.gearIndex > 1 then
          controller.mainController.shiftDownOnDown()
          controller.mainController.shiftDownOnUp()
        end
      end
    elseif leverState == leverStates.backwardMS then
      if lastLeverState == leverStates.idleMS then
        if gear == "S" then
          controller.mainController.shiftToGearIndex(6)
        elseif gear == "M" then
          controller.mainController.shiftUpOnDown()
          controller.mainController.shiftUpOnUp()
        end
      end
    end
    --print(string.format("Lever: %s, Park: %s", leverStateLookup[leverState], parkButtonLookup[parkButtonState]))
    lastLeverState = leverState
  end
end

local function readInputs(msg)
  if msg.ID == messageIds.receiveState then --GWS input state
    local parkByte = msg.DATA[3]
    local leverByte = msg.DATA[2]
    applyInputs(leverByte, parkByte)

    if not hardwareDetected then
      hardwareDetected = true
      log("I", "ProjectBavariaShifter.readInputs", "Communication with Project Bavaria Shifter established")
    end
  elseif msg.ID == messageIds.receiveShiftError then --"shift blocked", this is send when we try to command an illegal shift (aka out of P or into R without pressing "Unlock")
    if msg.DATA[3] == shiftErrorStates.onDown then --21 indicates "onDown", 20 indicates "onUp", we want to act as early as possible, so we use "onDown"
      guihooks.message({txt = "Can't shift, please press the 'Unlock' button", context = {}}, 2, "vehicle.shiftLogic.cannotShiftCANBus")
    end
  else
    -- for _, messageId in ipairs(messageIdsToIgnore) do
    --   if msg.ID == messageId then
    --     return
    --   end
    -- end
    -- log("W", "ProjectBavariaShifter.readInputs", "Unknown message received: " .. canBus.dumpsMsg(msg))
  end
end

local function updateHardwareState(dt)
  local state = gearStateLookup[string.sub(electrics.values.gear or "", 1, 1)] or 0
  if electrics.values.ignitionLevel <= 0 then
    state = gearStateLookup.Off
  end

  shifterModeCRCPayload[1] = writeMessageCounter
  shifterModeCRCPayload[2] = state
  local crc = crc8(shifterModeCRCPayload, crc8FinalXOR.send)
  shifterModeMessage[1] = crc
  shifterModeMessage[2] = writeMessageCounter
  shifterModeMessage[3] = state

  canBus.sendCANMessage(messageIds.sendShifterMode, shifterModeMessage, "appearance")

  writeMessageCounter = writeMessageCounter + 1 --always increase message counter
  if writeMessageCounter == 0x0F then --0x0F is a magic number that needs to be avoided here, just skip it and start over at 0x00
    writeMessageCounter = 0
  end

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
        updateHardwareState(dt)

        updateTimer = updateTimer + updateTime
      end
    end
  end
end

local function onExtensionLoaded()
  log("I", "ProjectBavariaShifter.onExtensionLoaded", "CANBus Shifter extension loaded")
  writeMessageCounter = 0
  leverStateLookup = {}
  parkButtonLookup = {}
  for key, value in pairs(leverStates) do
    leverStateLookup[value] = key
  end
  for key, value in pairs(parkButtonStates) do
    parkButtonLookup[value] = key
  end
  canBus = extensions.tech_CANBus_CANBusPeak
  if canBus then
    log("D", "ProjectBavariaShifter.onExtensionLoaded", "CANBus extension found")
    canBus.registerCANMessageCallback("ProjectBavariaShifter", canMessageReceived)
    if not canBus.isConnected then
      log("D", "ProjectBavariaShifter.onExtensionLoaded", "CANBus extension is not connected, connecting now...")
      local connectionResult = canBus.initCANBus()
      if connectionResult ~= canBus.errorCodes.OK then
        log("E", "ProjectBavariaShifter.onExtensionLoaded", "Non-OK init result for CAN Bus, shutting down... Result: " .. canBus.errorCodeLookup[connectionResult])
        canBus = nil
      end
    end
  else
    log("E", "ProjectBavariaShifter.onExtensionLoaded", "CANBus extension NOT found CANBus Controller won't work...")
  end
end

-- public interface
M.onExtensionLoaded = onExtensionLoaded
M.updateGFX = updateGFX

return M
