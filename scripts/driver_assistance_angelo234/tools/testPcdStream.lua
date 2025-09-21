-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local socket = require('socket')

local M = {}

local function receiveExact(client, expected)
  local parts = {}
  local received = 0
  while received < expected do
    local chunk, err, partial = client:receive(expected - received)
    if chunk then
      parts[#parts + 1] = chunk
      received = received + #chunk
    elseif err == 'timeout' then
      if partial and #partial > 0 then
        parts[#parts + 1] = partial
        received = received + #partial
      else
        socket.sleep(0.05)
      end
    else
      return nil, err
    end
  end
  return table.concat(parts)
end

function M.run(host, port, duration)
  host = host or '127.0.0.1'
  port = tonumber(port) or 23511
  duration = duration or 5

  local client, err = socket.tcp()
  if not client then
    return nil, 'failed to create socket: ' .. tostring(err)
  end

  local ok, connectErr = client:connect(host, port)
  if not ok then
    return nil, 'failed to connect: ' .. tostring(connectErr)
  end

  client:settimeout(0)

  local deadline = socket.gettime() + duration
  local stats = {pcd = 0, heartbeat = 0}

  while socket.gettime() < deadline do
    local line, err = client:receive('*l')
    if line then
      if line:sub(1, 3) == 'PCD' then
        local size = tonumber(line:match('PCD%s+(%d+)')) or 0
        if size > 0 then
          local payload, perr = receiveExact(client, size)
          if payload then
            stats.pcd = stats.pcd + 1
            print(string.format('Received PCD payload (%d bytes)', #payload))
          else
            print('Failed to receive PCD payload: ' .. tostring(perr))
            break
          end
        else
          print('Received empty PCD payload')
        end
      elseif line == 'PING' then
        stats.heartbeat = stats.heartbeat + 1
        print('Heartbeat received')
      else
        print('Unknown message: ' .. line)
      end
    else
      if err == 'timeout' then
        socket.sleep(0.05)
      else
        print('Connection closed: ' .. tostring(err))
        break
      end
    end
  end

  client:close()
  return stats
end

return M
