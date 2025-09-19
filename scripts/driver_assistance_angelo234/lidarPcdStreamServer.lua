-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local socket = require('socket')

local logger = require('scripts/driver_assistance_angelo234/logger')

local M = {}

local state = {
  server = nil,
  host = '127.0.0.1',
  port = nil,
  clients = {},
  heartbeatInterval = 1.0,
  heartbeatTimer = 0
}

local function logError(message)
  logger.log('E', 'lidar', message)
  if ui_message then
    ui_message(message)
  end
end

local function closeClient(client)
  if not client then return end
  local ok, err = pcall(function()
    client:close()
  end)
  if not ok and err then
    logger.log('E', 'lidar', string.format('Failed to close client: %s', tostring(err)))
  end
end

local function clearClients()
  for i = #state.clients, 1, -1 do
    closeClient(state.clients[i])
    state.clients[i] = nil
  end
end

local function stopServer()
  if state.server then
    local ok, err = pcall(function()
      state.server:close()
    end)
    if not ok and err then
      logger.log('E', 'lidar', string.format('Failed to close LiDAR stream server: %s', tostring(err)))
    end
  end
  state.server = nil
end

function M.stop()
  clearClients()
  stopServer()
  state.heartbeatTimer = 0
end

local function sendAll(client, data)
  if not client or not data or data == '' then return true end
  local totalSent = 0
  local totalSize = #data
  while totalSent < totalSize do
    local sent, err, partial = client:send(data, totalSent + 1)
    if not sent then
      if err == 'timeout' then
        if partial and partial > totalSent then
          totalSent = partial
        else
          return false, err
        end
      else
        return false, err
      end
    else
      totalSent = sent
    end
  end
  return true
end

local function removeClient(index)
  local client = state.clients[index]
  if client then
    closeClient(client)
  end
  table.remove(state.clients, index)
end

local function broadcastRaw(data)
  if not data or data == '' then return end
  for i = #state.clients, 1, -1 do
    local client = state.clients[i]
    local ok = sendAll(client, data)
    if not ok then
      removeClient(i)
    end
  end
end

local function acceptClients()
  if not state.server then return end
  while true do
    local client, err = state.server:accept()
    if not client then
      if err and err ~= 'timeout' and err ~= 'wantread' then
        logger.log('E', 'lidar', string.format('LiDAR PCD stream accept error: %s', tostring(err)))
      end
      break
    end
    client:settimeout(0)
    if client.setoption then
      pcall(client.setoption, client, 'tcp-nodelay', true)
    end
    state.clients[#state.clients + 1] = client
  end
end

function M.start(host, port)
  host = host or state.host or '127.0.0.1'
  local server, err = socket.bind(host, port)
  if not server then
    logError(string.format('Failed to start LiDAR PCD stream on %s:%s (%s)', tostring(host), tostring(port), tostring(err)))
    return false, err
  end
  server:settimeout(0)
  M.stop()
  state.server = server
  state.host = host
  state.port = port
  state.clients = {}
  state.heartbeatTimer = 0
  return true
end

function M.update(dt)
  if not state.server then return end
  acceptClients()
  state.heartbeatTimer = state.heartbeatTimer + (dt or 0)
  if state.heartbeatTimer >= state.heartbeatInterval then
    state.heartbeatTimer = state.heartbeatTimer - state.heartbeatInterval
    if #state.clients > 0 then
      broadcastRaw('PING\n')
    end
  end
end

function M.broadcast(pcdBytes)
  if not state.server or not pcdBytes or pcdBytes == '' then return end
  local header = string.format('PCD %d\n', #pcdBytes)
  for i = #state.clients, 1, -1 do
    local client = state.clients[i]
    local ok = sendAll(client, header)
    if ok then
      ok = sendAll(client, pcdBytes)
    end
    if not ok then
      removeClient(i)
    end
  end
end

function M.setHeartbeatInterval(interval)
  if type(interval) == 'number' and interval > 0 then
    state.heartbeatInterval = interval
  end
end

function M.clientCount()
  return #state.clients
end

function M.isRunning()
  return state.server ~= nil
end

return M
