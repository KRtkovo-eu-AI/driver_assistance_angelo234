-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

--[[
  TCPServer Lua Module
  This module provides a basic TCP server implementation using LuaSocket and LuaJIT FFI.
  It supports non-blocking sockets with custom message framing based on a fixed-size header
  and JSON encoded messages.

  Usage Example:

  local TCPServer = require('tcpServer')

  -- Create a server instance
  local server = TCPServer:new('localhost', 12345)

  -- Main loop
  while true do
    local data = server:receiveData()
    if data then
      for _, clientData in ipairs(data) do
        local connection, message = unpack(clientData)
        -- Process message
      end
    end
  end
--]]


--[[
TCP Server Message Protocol:

The protocol used by the TCP server involves sending and receiving messages
with a fixed-size header followed by a JSON-encoded payload. Each message
is structured as follows:

+-------------------+---------------------+
| 4-byte Identifier | 4-byte Message Size |
+-------------------+---------------------+
| JSON Data (variable length)             |
| (Null-terminated)                       |
+-----------------------------------------+

1. 4-byte Identifier:
   - A fixed 4-character string ('BN01') used to identify and validate
     the message format.

2. 4-byte Message Size:
   - A 32-bit unsigned integer (little-endian) that specifies the length
     of the JSON data that follows. This length includes the null terminator.

3. JSON Data:
   - A JSON-encoded string representing the message payload.
   - The JSON string is null-terminated (i.e., ends with a '\0' character).

Example:

Assume we're sending a JSON object {"cmd": "ping"}. The message might look like:

+------+------+------+------+------+------+------+------+------+------+------+
| 'B'  | 'N'  | '0'  | '1'  | size | size | size | size | '{'  | '"'  | 'c'  |
+------+------+------+------+------+------+------+------+------+------+------+
| 'm'  | 'd'  | '"'  | ':'  | ' '  | '"'  | 'p'  | 'i'  | 'n'  | 'g'  | '"'  |
+------+------+------+------+------+------+------+------+------+------+------+
| '}'  | '\0' |      |      |      |      |      |      |      |      |      |
+------+------+------+------+------+------+------+------+------+------+------+

Where "size" is the 4-byte representation of the length of the JSON data plus one
for the null terminator.

]]
local socket = require("socket.socket")
local stringBuffer = require('string.buffer')
local ffi = require("ffi")

local messageJsonHeader = 'BN01'
ffi.cdef[[
  typedef struct {
    char identifier[4];
    uint32_t length;  // little-endian (LE)
  } message_header_t;
]]
local headerSize = ffi.sizeof("message_header_t")


local TCPServer = {}
function TCPServer:new(listenHost, port)
  local newObj = {
    buffers = {},
    connections = {},
    listenHost = listenHost,
    port = port,
    serverLuasocket = nil,  -- Initialize to nil
    serverASIO = nil,      -- Initialize to nil
  }
  self.__index = self

  self.headerBuffer = ffi.new("message_header_t")
  ffi.copy(self.headerBuffer.identifier, messageJsonHeader, 4)

  -- availability of createNetworkServer means ASIO is available
  if not createNetworkServer then
    local status, err = pcall(function()
      self.serverLuasocket = socket.bind(listenHost, port)
      self.serverLuasocket:settimeout(0) -- Non-blocking
    end)
    if status then
      log('I', 'TCPServer', 'Editor toolchain loaded using luasocket')
    else
      log('E', 'TCPServer', 'Failed to start luasocket server: ' .. tostring(err))
      return false
    end
  else
    self.serverASIO = createNetworkServer('tcp', port)
    --log('I', 'TCPServer', 'Editor toolchain loaded using ASIO')
  end


  return setmetatable(newObj, self)
end

function TCPServer:send(connection, sendData, recData)
  if recData and recData.context then
    sendData.context = recData.context
  end
  --dump{'>>> ', sendData}
  local jsonData = jsonEncode(sendData)
  local messageLength = #jsonData + 1  -- +1 for the null character

  self.headerBuffer.length = ffi.new("uint32_t", messageLength) -- little-endian format

  -- Send the header
  connection:send(ffi.string(self.headerBuffer, headerSize))

  -- Send the JSON data and the null character
  connection:send(jsonData .. '\0')
  sendData.context = nil
end

function TCPServer:_onDataRaw(connection, dataRaw, res)
  local buffer = self.buffers[connection]
  if not buffer then
    buffer = require("string.buffer").new()
    self.buffers[connection] = buffer
  end

  buffer:put(dataRaw)

  while true do
    if #buffer < headerSize then break end -- Header length check

    -- Use the FFI to directly access the buffer's memory for header inspection
    local header = ffi.new("message_header_t")
    ffi.copy(header, buffer, ffi.sizeof("message_header_t"))

    if header.identifier[0] ~= 66 or -- 66 = B
      header.identifier[1] ~= 78 or  -- 78 = N
      header.identifier[2] ~= 48 or  -- 48 = 0
      header.identifier[3] ~= 49     -- 49 = 1
    then
      --log('E', 'TCPServer', 'Message format error: Incorrect identifier.')
      return
    end

    if #buffer < headerSize + header.length then break end

    buffer:skip(headerSize)
    local message = buffer:get(header.length) -- Extract the message
    -- Check for null character
    if message:byte(header.length) ~= 0 then
      log('E', 'TCPServer', 'Message format error: Null character not found.')
      return
    end

    local data = jsonDecode(message, 'tcpConnection')
    if not data then
      log('E', 'TCPServer', 'Unable to decode message from connection ' .. tostring(connection:get_remote_endpoint()))
      self:sendData(connection, {cmd = "error", message = "unable to decode json data"})
      return
    end
    table.insert(res, {connection, data})
  end
end

function TCPServer:receiveData()
  local res = {}
  if self.serverLuasocket then
    -- Accept new connections
    while true do
      local connection, err = self.serverLuasocket:accept()
      if not connection then
        if err ~= "timeout" then
          log('E', 'TCPServer', 'Error accepting connection: ' .. tostring(err))
        end
        break
      end
      connection:settimeout(0) -- Non-blocking
      self.buffers[connection] = stringBuffer.new()
      table.insert(self.connections, connection)
    end

    -- Receive and process data from connections
    local i = 1
    while i <= #self.connections do
      local connection = self.connections[i]
      local dataRaw, err, partial = connection:receive(4096)

      if err == "closed" then
        -- Handle the closed connection
        -- Log the disconnect and remove the connection from the list
        table.insert(res, {connection, 'disconnect'})
        connection:close()
        table.remove(self.connections, i)
        self.buffers[connection] = nil

        -- Do not increment i since we removed the current element
      elseif err and err ~= "timeout" then
        -- Handle other errors (excluding timeout)
        -- Log the error and remove the connection from the list
        table.insert(res, {connection, 'disconnect', err})
        connection:close()
        table.remove(self.connections, i)
        self.buffers[connection] = nil

        -- Do not increment i since we removed the current element
      else
        -- No error or just a timeout, process the data
        if dataRaw or partial then
          self:_onDataRaw(connection, dataRaw or partial, res)
        end

        -- Increment i only if we didn't remove the connection
        i = i + 1
      end
    end

  elseif self.serverASIO then
    local recData = self.serverASIO:receive()
    if not recData then return {} end

    for connection, dataChunks in pairs(recData) do
      for _, dataRaw in ipairs(dataChunks) do
        self:_onDataRaw(connection, dataRaw, res)
      end
    end
  end
  return res
end

function TCPServer:destroy()
  self.buffers = {}
  if self.serverASIO and destroyNetworkServer then
    destroyNetworkServer('tcp', self.port)
  end
  if self.serverLuasocket then
    -- Close all connections
    for _, connection in ipairs(self.connections or {}) do
      if connection then
        connection:close()
      end
    end
    -- Clear connections list
    self.connections = {}

    -- Close the server socket
    self.serverLuasocket:close()
    self.serverLuasocket = nil
  end
end

return TCPServer