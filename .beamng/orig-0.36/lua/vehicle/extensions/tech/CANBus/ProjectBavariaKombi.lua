-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

M.dependencies = {"tech_CANBus_CANBusPeak"}

local band = bit.band
local bxor = bit.bxor
local lshift = bit.lshift

local canBus

local hardwareDetected = false

local gearStateLookup = {
  P = 0x20,
  R = 0x40,
  N = 0x60,
  D = 0x80,
  DS = 0x81, --this is basically "S0"
  S = 0x81, --set to 81 and then set the actual gear in the upper 4 bit of the counter
  M = 0x82, --set to 82 and then set the actual gear in the upper 4 bit of the counter
  Off = 0x00
}

local tripComputerButtonPressed
local tripComputerButtonPressTimer = 0

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

local function bruteForceCRCXorValue(expectedCRC, payload)
  for i = 0, 255 do
    local crc = crc8(payload, i)
    if crc == expectedCRC then
      print(string.format("Found matching CRC XOR Value : %s", canBus.dumpsByteHex(i)))
    end
  end
end

local function readInputs(msg)
  if msg.ID == 0x1B3 then --from Kombi
    if not hardwareDetected then
      hardwareDetected = true
      log("I", "ProjectBavariaKombi.readInputs", "Communication with Project Bavaria Kombi established")
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

local messageInfos = {
  contactState = {id = 0x12F, updateFrequency = 10, checksumXOR = 0xB1, updateTimer = 0, counterInfo = {value = 0x00}},
  vehicleSpeed = {id = 0x1A1, updateFrequency = 100, checksumXOR = 0x0F, updateTimer = 0, counterInfo = {value = 0x00}},
  engineSpeed = {id = 0x0F3, updateFrequency = 100, checksumXOR = 0x8F, updateTimer = 0, counterInfo = {value = 0x00}},
  checkControl1 = {id = 0x2A7, updateFrequency = 50, checksumXOR = 0x38, updateTimer = 0, counterInfo = {value = 0x00}},
  checkControl2 = {id = 0x36E, updateFrequency = 20, checksumXOR = 0x7E, updateTimer = 0, counterInfo = {value = 0x00}},
  checkControl3 = {id = 0x592, updateFrequency = 20, checksumXOR = 0x00, updateTimer = 0, counterInfo = {value = 0x00}}, --TODO: not sure how this works...
  drivetrainData = {id = 0x3F9, updateFrequency = 20, checksumXOR = 0x04, updateTimer = 0, counterInfo = {value = 0x00}},
  gearboxData = {id = 0x3FD, updateFrequency = 50, checksumXOR = 0x70, updateTimer = 0, counterInfo = {value = 0x00}},
  emergencyCallState = {id = 0x2C3, updateFrequency = 20, checksumXOR = 0x00, updateTimer = 0, counterInfo = {value = 0x00}},
  fuelConsumption = {id = 0x2C4, updateFrequency = 20, checksumXOR = 0x71, updateTimer = 0, counterInfo = {value = 0x00, maxValue = 0xFE}},
  fuelLevelSensors = {id = 0x349, updateFrequency = 10, checksumXOR = 0x00, updateTimer = 0, counterInfo = {value = 0x00, maxValue = 0x00}},
  wakeUp = {id = 0x510, updateFrequency = 10, checksumXOR = 0x00, updateTimer = 0, counterInfo = {value = 0x00, maxValue = 0x00}},
  airbagAlive = {id = 0x0D7, updateFrequency = 1, checksumXOR = 0x00, updateTimer = 0, counterInfo = {value = 0x00, maxValue = 0xFF}},
  lightState = {id = 0x21A, updateFrequency = 10, checksumXOR = 0x00, updateTimer = 0, counterInfo = {value = 0x00, maxValue = 0x00}},
  backgroundDimmer = {id = 0x202, updateFrequency = 10, checksumXOR = 0x00, updateTimer = 0, counterInfo = {value = 0x00, maxValue = 0x00}},
  vehicleSpeed2 = {id = 0x19F, updateFrequency = 100, checksumXOR = 0x01, updateTimer = 0, counterInfo = {value = 0x00}},
  handbrakeStatus = {id = 0x34F, updateFrequency = 10, checksumXOR = 0x00, updateTimer = 0, counterInfo = {value = 0x00, maxValue = 0x00}},
  throttleRPM = {id = 0x0A5, updateFrequency = 50, checksumXOR = 0x6A, updateTimer = 0, counterInfo = {value = 0x00}},
  indicatorState = {id = 0x1F6, updateFrequency = 20, checksumXOR = 0x00, updateTimer = 0, counterInfo = {value = 0x00, maxValue = 0x00}},
  doorState = {id = 0x2FC, updateFrequency = 20, checksumXOR = 0x00, updateTimer = 0, counterInfo = {value = 0x00, maxValue = 0x00}},
  setTimeDate = {id = 0x39E, updateFrequency = 1, checksumXOR = 0x00, updateTimer = 0, counterInfo = {value = 0x00, maxValue = 0x00}},
  seatBeltLight = {id = 0x581, updateFrequency = 1, checksumXOR = 0x00, updateTimer = 0, counterInfo = {value = 0x00, maxValue = 0x00}},
  languageUnits = {id = 0x291, updateFrequency = 1, checksumXOR = 0x00, updateTimer = 0, counterInfo = {value = 0x00, maxValue = 0x00}},
  leftStalkButtons = {id = 0x1EE, updateFrequency = 20, checksumXOR = 0x00, updateTimer = 0, counterInfo = {value = 0x00, maxValue = 0x00}},
  tirePressure = {id = 0x31C, updateFrequency = 20, checksumXOR = 0xB4, updateTimer = 0, counterInfo = {value = 0x00, maxValue = 0x0E}},
  tireStatus = {id = 0x368, updateFrequency = 20, checksumXOR = 0xB4, updateTimer = 0, counterInfo = {value = 0x00, maxValue = 0x0E}},
  odometer = {id = 0x2BB, updateFrequency = 20, checksumXOR = 0x78, updateTimer = 0, counterInfo = {value = 0x00, maxValue = 0x0E}}
}

local function sendCountedAndCheckSummedMessage4BitShifted(messageInfo, counterHighBits, data, dt)
  messageInfo.updateTimer = messageInfo.updateTimer - dt
  if messageInfo.updateTimer > 0 then
    return
  end
  messageInfo.updateTimer = messageInfo.updateTimer + 1 / messageInfo.updateFrequency
  local counterInfo = messageInfo.counterInfo
  local counter = counterInfo.value
  local minValue = counterInfo.minValue or 0
  if counter < minValue then
    counter = minValue
  end
  counter = counter + 1
  if counter > (counterInfo.maxValue or 0x0E) then
    counter = counterInfo.minValue or 0
  end
  local payload = {counter + counterHighBits}
  for _, dataByte in pairs(data) do
    table.insert(payload, dataByte)
  end
  local crcXOR = messageInfo.checksumXOR
  local crc = crc8(payload, crcXOR)
  table.insert(payload, 1, crc)
  canBus.sendCANMessage(messageInfo.id, payload, messageInfo.id)

  counterInfo.value = counter
end

local function sendCountedAndCheckSummedMessage4Bit(messageInfo, counterHighBits, data, dt)
  sendCountedAndCheckSummedMessage4BitShifted(messageInfo, lshift(counterHighBits, 4), data, dt)
end

local function sendCountedAndCheckSummedMessage(messageInfo, data, dt)
  local counterInfo = messageInfo.counterInfo
  local counter = counterInfo.value
  local minValue = counterInfo.minValue or 0
  if counter < minValue then
    counter = minValue
  end
  counter = counter + 1
  if counter > (counterInfo.maxValue or 0x0E) then
    counter = counterInfo.minValue or 0
  end
  local payload = {counter}
  for _, dataByte in pairs(data) do
    table.insert(payload, dataByte)
  end
  local crcXOR = messageInfo.checksumXOR
  local crc = crc8(payload, crcXOR)
  table.insert(payload, 1, crc)
  canBus.sendCANMessage(messageInfo.id, payload, messageInfo.id)

  counterInfo.value = counter
end

local function sendCountedMessage(messageInfo, data, dt)
  local counterInfo = messageInfo.counterInfo
  local counter = counterInfo.value
  local minValue = counterInfo.minValue or 0
  if counter < minValue then
    counter = minValue
  end
  counter = counter + 1
  if counter > (counterInfo.maxValue or 0x0E) then
    counter = counterInfo.minValue or 0
  end
  local payload = {counter}
  for _, dataByte in pairs(data) do
    table.insert(payload, dataByte)
  end
  canBus.sendCANMessage(messageInfo.id, payload, messageInfo.id)

  counterInfo.value = counter
end

local function sendCountedMessageSecondByte4Bit(messageInfo, firstByte, counterHighBits, data, dt)
  local counterInfo = messageInfo.counterInfo
  local counter = counterInfo.value
  local minValue = counterInfo.minValue or 0
  if counter < minValue then
    counter = minValue
  end
  counter = counter + 1
  if counter > (counterInfo.maxValue or 0x0E) then
    counter = counterInfo.minValue or 0
  end
  local payload = {firstByte, (counter + lshift(counterHighBits, 4))}
  for _, dataByte in pairs(data) do
    table.insert(payload, dataByte)
  end
  canBus.sendCANMessage(messageInfo.id, payload, messageInfo.id)

  counterInfo.value = counter
end

local function sendMessage(messageInfo, data)
  canBus.sendCANMessage(messageInfo.id, data, messageInfo.id)
end

local function updateHardwareState(dt)
  --bruteForceCRCXorValue(0x4E, {0xF8, 0x13, 0x06, 0xF2})
  -- print(canBus.dumpsByteHex(crc))

  local speed = electrics.values.wheelspeed * 3.6
  local speedLow, speedHigh = canBus.twoBytes(speed * 63.5)
  sendCountedAndCheckSummedMessage4Bit(messageInfos.vehicleSpeed, 0x0D, {speedLow, speedHigh, 0x81}, dt) --TODO: find out why upper counter 4bits are sometimes 0xC (still stand?) and sometimes 0xD

  sendCountedAndCheckSummedMessage4Bit(messageInfos.checkControl1, 0x0F, {0xFE, 0xFF, 0x14}, dt) --gets rid of steering error
  sendCountedAndCheckSummedMessage4Bit(messageInfos.checkControl2, 0x0F, {0xFE, 0xFF, 0x15}, dt) --gets rid of ABS, parkingbrake, ESC error

  local state = gearStateLookup[string.sub(electrics.values.gear or "", 1, 1)] or 0
  if electrics.values.ignitionLevel <= 0 then
    state = gearStateLookup.Off
  end
  local gear = electrics.values.gearIndex
  sendCountedAndCheckSummedMessage4Bit(messageInfos.gearboxData, gear, {state, 0x0C, 0xFF}, dt) --displays gear and gets rid of "antrieb" error
  sendCountedAndCheckSummedMessage(messageInfos.fuelConsumption, {0xFF, 0x64, 0x64, 0x64, 0xC1, 0xF0}, dt)

  local oilTemp = electrics.values.oiltemp
  oilTemp = oilTemp + 48
  sendCountedAndCheckSummedMessage4Bit(messageInfos.drivetrainData, 0x0F, {0x82, 0x2E, 0x71, oilTemp, 0x43, 0x83}, dt)

  sendCountedMessage(messageInfos.emergencyCallState, {0x15, 0x0F, 0x00, 0xFF, 0x70, 0xFF, 0xFF}, dt) --get rid of emergency call error

  local rpm = electrics.values.rpm
  local rpm1, rpm2 = canBus.twoBytes(lshift(band(rpm * 0.1, 0xFFF), 4)) --12bits for RPM, shifted 4 to the left to make room for the counter in the remaining 4 bits
  sendCountedAndCheckSummedMessage4BitShifted(messageInfos.engineSpeed, rpm1, {rpm2, 0xC0, 0xF0, 0xC4, 0xFF, 0xFF}, dt)

  sendCountedAndCheckSummedMessage(messageInfos.tirePressure, {0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, dt)
  sendCountedMessageSecondByte4Bit(messageInfos.tireStatus, 0xFF, 0x0F, {0xFF, 0xFF, 0xFF}, dt)

  sendCountedMessage(messageInfos.airbagAlive, {0xFF}, dt)
  sendMessage(messageInfos.handbrakeStatus, {electrics.values.parkingbrake > 0 and 0xFE or 0xFD, 0xFF}, dt)

  local fuel = linearScale(electrics.values.fuel, 0, 1, 9500, 750) --750 -> full, 9500  -> empty
  local fuel1, fuel2 = canBus.twoBytes(fuel)
  sendMessage(messageInfos.fuelLevelSensors, {fuel1, fuel2, fuel1, fuel2, 0x00}, dt)

  local lowbeam = electrics.values.lowbeam > 0
  local highbeam = electrics.values.highbeam > 0
  local frontFog = electrics.values.fog > 0
  local rearFog = electrics.values.fog > 0
  local lightstate = 0x0
  if lowbeam then
    lightstate = lightstate + 0x04
  end
  if highbeam then
    lightstate = lightstate + 0x02
    lightstate = lightstate + 0x04 --add "on" as well
  end
  if frontFog then
    lightstate = lightstate + 0x20
  end
  if rearFog then
    lightstate = lightstate + 0x40
  end
  sendMessage(messageInfos.lightState, {lightstate, 0x32, 0xF7}, dt) --gets rid of light system error

  local backlight = true --electrics.values.lights > 0.5
  sendMessage(messageInfos.backgroundDimmer, {backlight and 0xFD or 0x00, 0x00}, dt) --set backlight

  local indicatorState1, indicatorState2 = 0x80, 0xF0
  if electrics.values.hazard_enabled > 0 then
    indicatorState1, indicatorState2 = 0xB1, 0xF2
  elseif electrics.values.signal_left_input > 0 then
    indicatorState1, indicatorState2 = 0x91, 0xF2
  elseif electrics.values.signal_right_input > 0 then
    indicatorState1, indicatorState2 = 0xA1, 0xF2
  end
  sendMessage(messageInfos.indicatorState, {indicatorState1, indicatorState2}, dt) --working!

  --sendMessage(messageInfos.doorState, {0x81, 0xF0, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}) --nope, not yet http://www.loopybunny.co.uk/CarPC/can/2FC.html

  local timeDate = os.date("*t")
  local year1, year2 = canBus.twoBytes(timeDate.year)
  --TODO year not working
  sendMessage(messageInfos.setTimeDate, {timeDate.hour, timeDate.min, timeDate.sec, timeDate.day, lshift(timeDate.month, 4) + 0xF, year1, year2, 0xF2}, dt) --works! gets rid of one yellow warning

  local seatBeltLightOn = false
  sendMessage(messageInfos.seatBeltLight, {0x40, 0x4D, 0x00, seatBeltLightOn and 0x29 or 0x28, 0xFF, 0xFF, 0xFF, 0xFF}, dt) --works mostly, TODO, why does it flash at lower update rates? 0x4D seems to vary sometimes as well

  local tripComputerButtonState = 0x00
  if tripComputerButtonPressed then
    tripComputerButtonState = 0x40
    tripComputerButtonPressTimer = tripComputerButtonPressTimer - dt
    if tripComputerButtonPressTimer <= 0 then
      tripComputerButtonPressed = false
      tripComputerButtonPressTimer = 0
    end
  end
  sendMessage(messageInfos.leftStalkButtons, {tripComputerButtonState, 0xFC}, dt)

  local odometer = ((electrics.values.odometer or 0) * 100) % 65535
  local odoByte1, odoByte2 = canBus.twoBytes(odometer)
  sendCountedAndCheckSummedMessage4Bit(messageInfos.odometer, 0x0F, {odoByte1, odoByte2, 0xF2}, dt)

  local languageLookup = {
    german = 0x01,
    english = 0x02,
    spanish = 0x04,
    french = 0x06,
    dutch = 0x08
  }
  local language = languageLookup.german
  local unit1 = 0x03 --TODO find these out
  local unit2 = 0x00
  local unit3 = 0x00
  local unit4 = 0x00
  sendMessage(messageInfos.languageUnits, {language, unit1, unit2, unit3, unit4, 0xF0}, dt)

  local contactByte0 = 0x08 -- 0x02, 0x05, 0x09, 0x0B, 0x07, 0x04, 0x08
  local contactByte1 = 0x86 -- 0x86, 0x8A, 0x8C, 0x8D, 0x88, 0x87
  local contactByte2 = 0xDD -- 0xDD, 0x1C, 0xCD, 0x6D, 0x5D, 0x4D, 0x3D, 0x2D, 0x1D
  local contactByte3 = 0xF1 -- 0xF1, 0xF4, 0xF2
  local contactByte4 = 0x15 -- 0x15, 0x01, 0x41, 0x81, 0x05
  local contactByte5 = 0x30 -- 0x30, 0x33 --> 33 engine on?
  local contactByte6 = 0x02 -- 0x02, 0x42

  if electrics.values.ignitionLevel == 0 then
    contactByte1 = 0x86
  elseif electrics.values.ignitionLevel == 1 then
    contactByte1 = 0x88
  else
    contactByte1 = 0x8A
  end

  sendCountedAndCheckSummedMessage4Bit(messageInfos.contactState, contactByte0, {contactByte1, contactByte2, contactByte3, contactByte4, contactByte5, contactByte6}, dt) --TODO 4bit: sometimes 0x07 (standing?), sometimes 0x08 (driving?)
  sendMessage(messageInfos.wakeUp, {0, 0, 0, 0, 0xFF, 0xFF, 0, 0x10}, dt) --keep  awake, constant data in all my logs
end

local function tripComputerButtonPress()
  tripComputerButtonPressTimer = 0.3
  tripComputerButtonPressed = true
end

local function canMessageReceived(msg)
  if playerInfo.firstPlayerSeated then
    readInputs(msg)
  end
end

local function updateGFX(dt)
  if playerInfo.firstPlayerSeated then
    if canBus then
      updateHardwareState(dt)
    end
  end
end

local function onExtensionLoaded()
  log("I", "ProjectBavariaController.onExtensionLoaded", "CANBus Controller extension loaded")

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
M.tripComputerButtonDown = tripComputerButtonDown
M.tripComputerButtonUp = tripComputerButtonUp
M.tripComputerButtonPress = tripComputerButtonPress

return M
