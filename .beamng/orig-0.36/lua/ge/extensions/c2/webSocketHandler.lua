-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

-- extensions.load('c2_webSocketHandler')

local wsUtils     = require("utils/wsUtils")

local port          = 8088
local server        = nil
local chosenAddress = nil

local function handleWebSocketData(evt, jsonData)
  if jsonData.type == "loadExtension" then
    local extName = jsonData.extensionName
    if extName then
      local ok, err = pcall(extensions.load, extName)
      if not ok then
        server:sendData(evt.peerId, jsonEncode({
          type = "error",
          msg  = "Failed to load extension: " .. tostring(err)
        }))
      else
        server:sendData(evt.peerId, jsonEncode({
          type = "info",
          msg  = "Successfully loaded extension: " .. extName
        }))
      end
    else
      server:sendData(evt.peerId, jsonEncode({
        type = "error",
        msg  = "No extension name provided."
      }))
    end
    return
  end
  extensions.hook("onC2WebSocketHandlerMessage", {
    event   = evt,
    message = jsonData
  })
end

local function onExtensionLoaded()

  server, chosenAddress = wsUtils.createOrGetWS("127.0.0.1", port, "", 'c2', "", false)
  log("I", "c2_webSocketHandler", "c2_simScene WebSocket server at: http://" .. chosenAddress .. ":" .. tostring(port))
end

local function onExtensionUnloaded()
  if server then
    BNGWebWSServer.destroy(server)
    server = nil
  end
  log("I", "c2_webSocketHandler", "Cleaned up WebSocket Handler")
end

local function onUpdate(dt)
  if not server then return end
  local events = server:getPeerEvents()
  if #events == 0 then return end

  for _, evt in ipairs(events) do
    if evt.type == "D" and evt.msg ~= "" then
      if evt.msg == "ping" then
        server:sendData(evt.peerId, "pong")
      else
        local ok, decodedMsg = pcall(jsonDecode, evt.msg)
        if ok and decodedMsg then
          handleWebSocketData(evt, decodedMsg)
        else
          log("E", "c2_simScene", "Failed to decode JSON: " .. tostring(evt.msg))
        end
      end
    end
  end

  server:update()
end

M.onExtensionLoaded   = onExtensionLoaded
M.onExtensionUnloaded = onExtensionUnloaded
M.onUpdate            = onUpdate

return M