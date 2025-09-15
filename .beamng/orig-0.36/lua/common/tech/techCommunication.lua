-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}
local logTag = 'TechCom'

local mp = require('libs/lua-MessagePack/MessagePack')
local socket = require('libs/luasocket/socket.socket')
local ffi = require('ffi')

local HEADER_SIZE = 4
local BUF_SIZE = 131072
local headerBuffer = ffi.new('char[?]', HEADER_SIZE)
local recvBufs = {} -- used only when receiving messages.

local tcomDebug = not shipping_build -- when true, errors in BeamNGpy protocol crash the communication and full stacktrace is shown
local isRecording = nil
local recorder = nil

M.protocolVersion = 'v1.23'

local function packUnsignedInt32Network(n)
  headerBuffer[0] = math.floor(n / 0x1000000)
  headerBuffer[1] = math.floor(n / 0x10000) % 0x100
  headerBuffer[2] = math.floor(n / 0x100) % 0x100
  headerBuffer[3] = n % 0x100
  return headerBuffer
end

local function unpackUnsignedInt32Network(c)
  local b1, b2, b3, b4 = c:byte(1, 4)
  return ((b1 * 0x100 + b2) * 0x100 + b3) * 0x100 + b4
end

-- Simple set implementation from the LuaSocket samples
local function newSet()
  local reverse = {}
  local set = {}
  return setmetatable(set, {
    __index = {
      insert = function(set, value)
        if not reverse[value] then
          table.insert(set, value)
          reverse[value] = #set
        end
      end,
      remove = function(set, value)
        local index = reverse[value]
        if index then
          reverse[value] = nil
          local top = table.remove(set)
          if top ~= value then
            reverse[top] = index
            set[index] = top
          end
        end
      end
    }
  })
end

local function checkForClients(servers)
  local ret = {}
  local readable, _, err = socket.select(servers, nil, 0)
  for _, input in ipairs(readable) do
    local client = input:accept()
    table.insert(ret, client)
  end
  return ret
end

local function receive(skt)
  local lengthPacked, err = skt:receive(4)

  if err then
    log('E', logTag, 'Error reading from socket: ' .. tostring(err))
    return nil, err
  end

  table.clear(recvBufs)

  local length = unpackUnsignedInt32Network(lengthPacked)
  if length == 808464432 then -- potentially a client with an old version of BeamNGpy, 808464432 = '0000' unpacked as uint32
    local received, err = skt:receive(12) -- length used to be encoded in first 16 bytes as a string
    local lengthRest = tonumber(received)
    if err then
      log('E', logTag, 'Error reading from socket: ' .. tostring(err))
      return nil, err
    end

    if lengthRest == 34 then -- the length of a Hello message from an old client
      log('E', logTag, 'Unsupported client version. Disconnecting client.')
      M.sendLegacyError(skt, 'Unsupported client version. Please use the version of BeamNGpy corresponding to this release of BeamNG.')
      return nil, err
    else -- it was not a Hello message, add the data to the received buffer
      table.insert(recvBufs, received)
      length = length - #received
    end
  end

  while true do
    local received, err = skt:receive(math.min(length, BUF_SIZE))
    if err then
      log('E', logTag, 'Error reading from socket: ' .. tostring(err))
      return nil, err
    end

    table.insert(recvBufs, received)
    length = length - #received
    if length <= 0 then
      break
    end
  end
  if err then
    log('E', logTag, 'Error reading from socket: ' .. tostring(err))
    return nil, err
  end

  return table.concat(recvBufs), nil
end

local Request = {}

local function checkIfRecording()
  recorder = extensions['tech_techCapture']
  if recorder == nil then
    isRecording = false
    return
  end

  if not recorder.isRecordingRequests() then
    recorder = nil
    isRecording = false
    return
  end

  isRecording = true
end

local function handleRequest(handler, request)
  if not tcomDebug then
    local status, result = pcall(handler, request)
    if status then
      request:markHandled()
      return result
    end

    log('E', logTag, 'A fatal error encountered during handling the command: ' .. result)
    request:sendBNGError(result)
    return false
  else
    if isRecording == nil then
      checkIfRecording()
    end
    if isRecording then
      isRecording = recorder.recordRequest(request)
    end
  end

  local result = handler(request)
  request:markHandled()

  return result
end

local function callRequestHandler(E, request)
  local msgType = request['type']
  if msgType ~= nil then
    local handler
    if E.handlers then
      handler = E.handlers[msgType]
    end
    if handler == nil then
      msgType = 'handle' .. msgType
      handler = E[msgType]
    end
    if handler ~= nil then
      if handleRequest(handler, request) == false then
        return false
      end
    else
      extensions.hook('onSocketMessage', request)
    end
  else
    log('E', logTag, 'Got message without message type: ' .. tostring(message))
    request:sendBNGError('Got message without message type.')
  end
end

local function checkMessages(E, clients)
  local message
  local readable, writable, err = socket.select(clients, clients, 0)
  local ret = true

  for i = 1, #readable do
    local skt = readable[i]

    if writable[skt] == nil then
      goto continue
    end

    message, err = M.receive(skt)

    if err ~= nil then
      clients:remove(skt)
      log('E', logTag, 'Error reading from socket: ' .. tostring(skt) .. ' - ' .. tostring(err))
      goto continue
    end

    if message ~= nil then
      local request = Request:new(mp.unpack(message), skt)
      if callRequestHandler(E, request) == false then
        ret = false
      end

      if not request.handled then
        request:sendBNGError([[The request was not handled by BeamNG.tech. This can mean:
- an incompatible version of BeamNG.tech/BeamNGpy
- an internal error in BeamNG.tech
- incorrectly implemented custom command]])
      end
    end

    ::continue::
  end
  if #readable > 0 then
    return ret
  else
    return false
  end
end

local function sanitizeTable(tab)
  local ret = {}

  for k, v in pairs(tab) do
    k = type(k) == 'number' and k or tostring(k)

    local t = type(v)

    if t == 'table' then
      ret[k] = M.sanitizeTable(v)
    end

    if t == 'vec3' then
      ret[k] = {v.x, v.y, v.z}
    end

    if t == 'quat' then
      ret[k] = {v.x, v.y, v.z, v.w}
    end

    if t == 'number' or t == 'boolean' or t == 'string' then
      ret[k] = v
    end
  end

  return ret
end

local function sendAll(skt, data, length)
  local index = 1
  while index < length do
    local sent, err = skt:send(data, index, length)
    if sent == nil then
      return err
    end
    index = sent + 1
  end

  return nil
end

local function sendLegacyError(skt, error) -- send an error to a legacy BeamNGpy client so the client can parse it
  local message = mp.pack({bngError = error})

  local length = #message
  local stringLength = string.format('%016d', length)
  message = stringLength .. message
  local err = sendAll(skt, message, #message)
  if err then
    log('E', logTag, 'Error writing to socket: ' .. tostring(err))
    return
  end
end

local function sendMessage(skt, message)
  if skt == nil then
    return
  end

  message = mp.packPrefixWorkBuffer('\0\0\0\0', message)
  local length = #message
  local lenPrefix = packUnsignedInt32Network(length - HEADER_SIZE)
  ffi.copy(message, lenPrefix, HEADER_SIZE)
  local err = sendAll(skt, message, length)

  if err then
    log('E', logTag, 'Error writing to socket: ' .. tostring(err))
    return
  end
end


local function openServer(port, ip)
  ip = ip or '*'
  local server, error = socket.bind(ip, port)
  if server == nil then
    log('E', logTag, error)
    if error == 'permission denied' then
      local msg =
[[Access to the port %d was denied by the operating system. That can mean:
    1. The port is already in use by another program (can be also another instance of BeamNG).
    2. The port is inaccessible with your user privileges (ports under 1000 can cause this issue).
Try to restart BeamNG and use a different port number.]]
      log('E', logTag, string.format(msg, port))
    end
    return nil
  end
  ip, port = server:getsockname()
  log('I', logTag, 'Started listening on ' .. ip .. '/' .. tostring(port) .. '.')
  return server
end

function Request:new(o, skt)
  o = o or {}
  setmetatable(o, self)
  self.__index = self
  o.skt = skt
  o.handled = false
  return o
end

function Request:markHandled() self.handled = true end

function Request:sendResponse(message)
  message['_id'] = self['_id']
  self:markHandled()
  M.sendMessage(self.skt, message)
end

function Request:sendACK(type)
  local message = {}
  if type ~= nil then
    message.type = type
  end
  self:sendResponse(message)
end

function Request:sendBNGError(message)
  local message = {bngError = message}
  self:sendResponse(message)
end

function Request:sendBNGValueError(message)
  local message = {bngValueError = message}
  self:sendResponse(message)
end

local function enableDebug()
  log('I', logTag, 'Enabled tech communication debug mode.')
  tcomDebug = true
end

local function disableDebug()
  log('I', logTag, 'Disabled tech communication debug mode.')
  tcomDebug = false
end

M.newSet = newSet
M.checkForClients = checkForClients
M.receive = receive
M.checkMessages = checkMessages
M.sanitizeTable = sanitizeTable
M.sendLegacyError = sendLegacyError
M.sendMessage = sendMessage
M.openServer = openServer
M.enableDebug = enableDebug
M.disableDebug = disableDebug

M.callRequestHandler = callRequestHandler

return M
