-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

-- extensions.load('util_nodeStream')

local wsUtils     = require("utils/wsUtils")

local port          = 8088
local server        = nil
local chosenAddress = nil
local nodeStreamActive = false
local streamInterval = 0.1
local streamTimer = 0

-- Define broadcastVehicleNodes function before it's used by handleWebSocketData
local function broadcastVehicleNodes()
  if not server then
    log("E", "nodeStream", "Server not initialized")
    return
  end

  local vehicle = getPlayerVehicle(0)
  if not vehicle then
    log("W", "nodeStream", "No player vehicle found")
    return
  end

  local nodeCount = vehicle:getNodeCount()
  log("D", "nodeStream", "Broadcasting " .. nodeCount .. " nodes")

  local nodeData = {
    type = "vehicleNodes",
    nodes = {},
    initialNodes = {}
  }

  -- Get model origin from reference node
  local refNodePos = nil

  local refNodeId = vehicle:getRefNodeId()
  if refNodeId then
    refNodePos = vehicle:getInitialNodePosition(refNodeId)
  end


  for j = 0, nodeCount - 1 do
    -- Current positions
    local nodePos = vehicle:getNodePosition(j)
    nodeData.nodes[j] = {
      x = nodePos.x,
      y = nodePos.z,
      z = -nodePos.y
    }
  end

  local encodedData = jsonEncode(nodeData)

  -- Use broadcastData instead of individual sends to all peers
  server:broadcastData(encodedData)
end

local function getInitialNodePositions(peerId)
  if not server then
    log("E", "nodeStream", "Server not initialized")
    return
  end

  local vehicle = getPlayerVehicle(0)
  if not vehicle then
    log("W", "nodeStream", "No player vehicle found")
    return
  end

  local nodeCount = vehicle:getNodeCount()
  log("D", "nodeStream", "Sending initial positions for " .. nodeCount .. " nodes")

  local nodeData = {
    type = "initialNodePositions",
    nodes = {}
  }

  -- Get model origin from reference node (usually node 0)
  local refNodeId = vehicle:getRefNodeId()
  local refNodePos = nil
  if refNodeId then
    refNodePos = vehicle:getInitialNodePosition(refNodeId)
  end

  for j = 0, nodeCount - 1 do
    local initialNodePos = vehicle:getInitialNodePosition(j)
    nodeData.nodes[j] = {
      x = initialNodePos.x - refNodePos.x,
      y = initialNodePos.z - refNodePos.z,
      z = -initialNodePos.y + refNodePos.y
    }
  end

  local encodedData = jsonEncode(nodeData)
  server:sendData(peerId, encodedData)
end

local function getFlexbodyMappingData(peerId)
  if not server then
    log("E", "nodeStream", "Server not initialized")
    return
  end

  local vehicle = getPlayerVehicle(0)
  if not vehicle then
    log("W", "nodeStream", "No player vehicle found")
    return
  end

  log("D", "nodeStream", "Sending flexbody mapping data")

  local flexbodyData = {
    type = "flexbodyMappingData",
    flexbodies = {}
  }

  -- Get the flexbodies data from the vehicle
  local vehId = vehicle:getId()
  local vData = extensions.core_vehicle_manager.getVehicleData(vehId)

  if not vData or not vData.vdata or not vData.vdata.flexbodies then
    log("W", "nodeStream", "No flexbody data found for vehicle")
    server:sendData(peerId, jsonEncode({
      type = "error",
      msg = "No flexbody data available for this vehicle"
    }))
    return
  end

  -- Process each flexbody and collect mapping parameters
  for flexKey, flexbody in pairs(vData.vdata.flexbodies) do
    local mappingData = {
      mesh = flexbody.mesh,
      groupNodes = flexbody._group_nodes,
      pos = flexbody.pos,
      rot = flexbody.rot,
      scale = flexbody.scale,
      flatMap = flexbody.flatMap
    }

    flexbodyData.flexbodies[flexKey] = mappingData
  end

  local encodedData = jsonEncode(flexbodyData)
  server:sendData(peerId, encodedData)
end

local function handleWebSocketData(evt, jsonData)
  log("I", "nodeStream", "Received WS data: " .. dumps(jsonData))

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
  elseif jsonData.type == "startNodeStream" then
    log("I", "nodeStream", "Node streaming activated by client request")
    nodeStreamActive = true
    server:sendData(evt.peerId, jsonEncode({
      type = "info",
      msg  = "Node streaming activated"
    }))
    -- Send first node update immediately
    broadcastVehicleNodes()
    return
  elseif jsonData.type == "stopNodeStream" then
    log("I", "nodeStream", "Node streaming deactivated by client request")
    nodeStreamActive = false
    server:sendData(evt.peerId, jsonEncode({
      type = "info",
      msg  = "Node streaming deactivated"
    }))
    return
  elseif jsonData.type == "getInitialNodePositions" then
    log("I", "nodeStream", "Received request for initial node positions")
    getInitialNodePositions(evt.peerId)
    return
  elseif jsonData.type == "getFlexbodyMappingData" then
    log("I", "nodeStream", "Received request for flexbody mapping data")
    getFlexbodyMappingData(evt.peerId)
    return
  end

  extensions.hook("onWebSocketHandlerMessage", {
    event   = evt,
    message = jsonData
  })
end

local function onExtensionLoaded()
  log("I", "nodeStream", "Extension loading, creating WebSocket on port " .. tostring(port))
  server, chosenAddress = wsUtils.createOrGetWS("127.0.0.1", port, "", 'nodestream', "", false)

  if server then
    log("I", "nodeStream", "Node stream WebSocket server started at: http://" .. chosenAddress .. ":" .. tostring(port))
  else
    log("E", "nodeStream", "Failed to create WebSocket server")
  end
end

local function onExtensionUnloaded()
  if server then
    log("I", "nodeStream", "Cleaning up WebSocket server")
    BNGWebWSServer.destroy(server)
    server = nil
  end
  log("I", "nodeStream", "Extension unloaded")
end

local function onUpdate(dt)
  if not server then
    return
  end

  -- Handle node streaming
  if nodeStreamActive then
    streamTimer = streamTimer + dt
    if streamTimer >= streamInterval then
      broadcastVehicleNodes()
      streamTimer = 0
    end
  end

  local events = server:getPeerEvents()
  if #events > 0 then
    log("D", "nodeStream", "Processing " .. #events .. " WebSocket events")
  end

  for _, evt in ipairs(events) do
    if evt.type == "D" and evt.msg ~= "" then
      if evt.msg == "ping" then
        server:sendData(evt.peerId, "pong")
      else
        local ok, decodedMsg = pcall(jsonDecode, evt.msg)
        if ok and decodedMsg then
          handleWebSocketData(evt, decodedMsg)
        else
          log("E", "nodeStream", "Failed to decode JSON: " .. tostring(evt.msg))
        end
      end
    elseif evt.type == "C" then
      -- Client connected
      log("I", "nodeStream", "Client connected with ID: " .. tostring(evt.peerId))
    elseif evt.type == "DC" then
      -- Client disconnected
      log("I", "nodeStream", "Client disconnected with ID: " .. tostring(evt.peerId))
    end
  end

  server:update()
end

M.onExtensionLoaded   = onExtensionLoaded
M.onExtensionUnloaded = onExtensionUnloaded
M.onUpdate            = onUpdate

return M