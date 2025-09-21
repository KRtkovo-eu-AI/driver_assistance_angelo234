-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local isUDPConnected = false
local debugSendFPS = 1 / 30
local debugReceiveFPS = 1 / 10
local debugHeartbeatFPS = 1 / 1
local debugSendTimer = 0
local debugReceiveTimer = 0
local debugHeartbeatTimer = 0
local debugPackets = {}
local udpSocket

local mainDebugPacket = {sourceType = "AED", isActiveVehicle = false}

local function debugPacket(packet)
  packet.vehicleID = objectId
  table.insert(debugPackets, packet)
end

local function sendDebugData()
  if isUDPConnected then
    local result, error = udpSocket:send(jsonEncode(debugPackets))
    if not result then
      print("Error sending debug data: " .. error)
    end
    table.clear(debugPackets)
  end
end

local function sendDebugHeartbeat()
  if isUDPConnected then
    udpSocket:send("BNGAED")
  end
end

local function sendConfigData()
  debugPacket({sourceType = "AED", packetType = "config", config = {}})
end

local function receiveDebugCommands()
  if not isUDPConnected then
    return
  end

  local data = udpSocket:receive()
  if data then
    --dump(data)
    local splits = split(data, "->")
    local commandType = splits[1]
    local controllerName = splits[2]
    if commandType and controllerName then
      if commandType == "RequestConfig" then
        sendConfigData()
      elseif commandType == "EnableDebugMode" then
        local receiver = subControllerLookup[controllerName]
        if receiver.setDebugMode then
          receiver.setDebugMode(true)
        end
      elseif commandType == "SetConfig" then
        local receiver = subControllerLookup[controllerName]
        local controllerConfig = splits[3]
        if receiver and receiver.setConfig and controllerConfig then
          local configTable = jsonDecode(controllerConfig, "CMU SetConfig")
          --dump(configTable)
          receiver.setConfig(configTable)
          sendConfigData()
        end
      elseif commandType == "SetProperty" then
        local receiver = subControllerLookup[controllerName]
        if receiver and receiver.setParameters then
          local param = splits[3]
          local paramTable = jsonDecode(param, "CMU SetProperty")
          --dump(paramTable)
          receiver.setParameters(paramTable)
        end
      end
    end
  end
end

local function updateGFX(dt)
  mainDebugPacket.isActiveVehicle = playerInfo.firstPlayerSeated
  debugPacket(mainDebugPacket)

  debugSendTimer = debugSendTimer + dt
  if debugSendTimer >= debugSendFPS then
    debugSendTimer = debugSendTimer - debugSendFPS
    sendDebugData()
  end

  debugReceiveTimer = debugReceiveTimer + dt
  if debugReceiveTimer >= debugReceiveFPS then
    debugReceiveTimer = debugReceiveTimer - debugReceiveFPS
  --receiveDebugCommands()
  end
end

local function onReset()
  table.clear(debugPackets)
end

local function onInit()
  isUDPConnected = false
  local debugSettings = {}
  table.clear(debugPackets)
  local peerIP = debugSettings.peerIP or "127.0.0.1"
  local peerPort = debugSettings.peerPort or 43812

  --socket is not always available
  if socket then
    udpSocket = socket.udp()
    udpSocket:settimeout(0.00)
    local result, error = udpSocket:setpeername(peerIP, peerPort)
    if result and not error then
      isUDPConnected = true
    end
  end

  --Tell the debug app that we spawned a new car
  debugPacket({sourceType = "AED", packetType = "init"})
end

local function onExtensionLoaded()
  onInit()
end

M.onInit = onInit
M.onReset = onReset
M.updateGFX = updateGFX
M.onExtensionLoaded = onExtensionLoaded

M.debugPacket = debugPacket

return M
