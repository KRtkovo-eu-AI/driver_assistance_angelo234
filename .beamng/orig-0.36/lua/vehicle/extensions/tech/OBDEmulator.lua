-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

local receiveBuffer
local engines = {}

local comPortId

local updateTimer = 0
local updateTime = 1 / 10

local hardwareDetected = false

local portName = "com12"
local portBaudrate = 38400

local function openSerialPort(port, baudrate)
  port = port or portName
  baudrate = baudrate or portBaudrate
  if comPortId and comPortId >= 0 then
    return
  end

  comPortId = obj:serialPortOpen(port, baudrate, 100 * 1024)
end

local function closeSerialPort()
  if comPortId and comPortId >= 0 then
    return
  end

  obj:serialPortClose(comPortId)
  comPortId = nil
end

local function writeSerialPort(data)
  if not comPortId or comPortId < 0 then
    return
  end
  data = data .. "\r"
  local bytesWritten = obj:serialPortWrite(comPortId, data, string.len(data))
  if bytesWritten == 0 then --port disconnected?
    log("W", "OBDEmulator.writeSerialPort", "serialPortWrite returned 0, trying to establish a new COM connection...")
    comPortId = nil
    openSerialPort()
  elseif bytesWritten == -1 then --port not valid
    log("W", "OBDEmulator.writeSerialPort", "serialPortWrite returned -1, trying to establish a new COM connection...")
    comPortId = nil
    openSerialPort()
  end
end

local function serialPortDataReceived(data, bufferOverflown)
  data = data:gsub("%z", "")
  receiveBuffer = receiveBuffer .. data

  --dump(receiveBuffer)

  local messages = {}
  local b = ""
  for c in receiveBuffer:gmatch "." do
    if c ~= "\r" then
      b = b .. c
    else
      if #b > 0 then
        table.insert(messages, b)
      end
      b = ""
    end
  end
  --dump(messages)
  receiveBuffer = b
  --dump(receiveBuffer)

  if not hardwareDetected then
    for _, response in ipairs(messages) do
      if response == "OK" then
        hardwareDetected = true
        log("I", "OBDEmulator.serialPortDataReceived", "Communication with OBD Emulator established")
        break
      end
    end
  end
end

local function sendOBDData(dt)
  writeSerialPort(string.format("ATSET 010C=%d", clamp((electrics.values.rpm or 0), 0, 16000)))
  writeSerialPort(string.format("ATSET 010D=%d", clamp((electrics.values.wheelspeed or 0) * 3.6, 0, 255)))
  writeSerialPort(string.format("ATSET 0105=%d", clamp((electrics.values.watertemp or 0), 0, 215)))
  writeSerialPort(string.format("ATSET 015C=%d", clamp((electrics.values.oiltemp or 0), 0, 210)))
  writeSerialPort(string.format("ATSET 0104=%d", clamp((electrics.values.engineLoad or 0) * 100, 0, 100))) --todo
  writeSerialPort(string.format("ATSET 0146=%d", clamp(powertrain.currentEnvTemperatureCelsius, -40, 215)))
  writeSerialPort(string.format("ATSET 0111=%d", clamp((electrics.values.throttle or 0) * 100, 0, 100)))
  writeSerialPort(string.format("ATSET 0133=%d", clamp(powertrain.currentEnvPressure * 0.001, 0, 255)))
  writeSerialPort(string.format("ATSET 010B=%d", clamp((electrics.values.turboBoost or 0) * 6.89476 + powertrain.currentEnvPressure * 0.001, 0, 255)))

  local torque = 0
  for _, engine in ipairs(engines) do
    torque = torque + engine.outputTorque1 or 0
  end
  writeSerialPort(string.format("ATSET 0163=%d", clamp(torque, 0, 65000)))
end

local function receiveOBDData(dt)
  if not comPortId or comPortId < 0 then
    return
  end

  local data, bufferOverflown = obj:serialPortRead(comPortId)
  if string.len(data) > 0 then
    serialPortDataReceived(data, bufferOverflown)
  end
end

local function updateGFX(dt)
  if playerInfo.firstPlayerSeated then
    updateTimer = updateTimer + dt
    if updateTimer >= updateTime then
      sendOBDData(dt)
      receiveOBDData(dt)
      updateTimer = updateTimer - updateTime
    end
  end
end

local function onExtensionLoaded()
  receiveBuffer = ""
  log("I", "OBDAdapter.onExtensionLoaded", "OBD Adapter extension loaded")

  openSerialPort()

  engines = {}
  for _, engine in ipairs(powertrain.getDevicesByCategory("engine")) do
    table.insert(engines, engine)
  end
end

-- public interface
M.onExtensionLoaded = onExtensionLoaded
M.updateGFX = updateGFX
M.writeSerialPort = writeSerialPort

return M
