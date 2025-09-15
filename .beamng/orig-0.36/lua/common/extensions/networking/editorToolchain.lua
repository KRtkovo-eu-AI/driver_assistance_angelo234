-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- extensions.load('networking_editorToolchain')

local M = {}

local TCPServer = require('tcpServer')
local server

local selectedNodes = nil
local sphereColor = ColorF(1,0,0,1)
local subscriptions = {}

local function getPeerName(connection)
  if connection.getpeername then
    -- luasocket
    local ip, port = connection:getpeername()
    if ip and port then
      return tostring(ip) .. ':' .. tostring(port)
    end
  else
    -- ASIO
    return tostring(connection:get_remote_endpoint())
  end
end

local function onData(connection, data)
  --dump{'<<< ', data}

  if data == 'disconnect' then
    subscriptions[connection] = nil
    selectedNodes = nil
    print('Connection closed: ' .. tostring(connection))
    return
  end

  if data['cmd'] == 'ping' then
    --log('I', 'editorToolchain', getPeerName(connection) .. ' - ping')
    server:send(connection, {cmd='pong'}, data)

  elseif data['cmd'] == 'init' then
    local info = {
      versionb = beamng_versionb,
      versiond = beamng_versiond,
      windowtitle = beamng_windowtitle,
      buildtype = beamng_buildtype,
      buildinfo = beamng_buildinfo,
      arch = beamng_arch,
      buildnumber = beamng_buildnumber,
      shipping_build = shipping_build,
      root=FS:getGamePath(),
      userpath=FS:getUserPath(),
    }
    server:send(connection, {cmd='siminfo', data=info}, data)

  elseif data['cmd'] == 'getPlayerVehicleInfo' then
    local veh = getPlayerVehicle(0)
    if not veh then
      server:send(connection, {cmd='noPlayerVehicle'}, data)
      return
    end
    local res = {
      jbeam = veh.Jbeam,
      partConfig = veh.partConfig,
    }
    server:send(connection, {cmd = 'playerVehicleInfo', data = res}, data)
  elseif data['cmd'] == 'selectNodes' then
    dump{'selecting nodes: ', data.nodes}
    selectedNodes = nil

    local playerVehicle = core_vehicle_manager.getPlayerVehicleData()
    if playerVehicle then
      local nodes = playerVehicle.vdata.nodes
      for _, nodeName in ipairs(data.nodes) do
        local nodeFound = false
        for _, node in ipairs(nodes) do
          if node.name == nodeName then
            --dump{"found node", node}
            if not selectedNodes then selectedNodes = {} end
            table.insert(selectedNodes, node)
            nodeFound = true
            break
          end
        end
        if not nodeFound then
          log('W', 'node to highlight not found / not spawned? ' .. tostring(nodeName))
        end
      end
    end
  elseif data['cmd'] == 'subscribe' then
    dump({'client wants to subscribe to data', getPeerName(connection), data})
    if not subscriptions[connection] then
      subscriptions[connection] = {}
    end
    data.cmd = nil
    table.insert(subscriptions[connection], data)
  else
    server:send(connection, {cmd = "error", message = "unknown command: '" .. tostring(data.cmd) .. "'"}, data)
  end
end

local function onPreRender(dtReal, dtSim, dtRaw)
  if not server then return end

  local messages = server:receiveData()
  for _, msg in pairs(messages or {}) do
    onData(msg[1], msg[2])
  end

  if selectedNodes then
    local veh = getPlayerVehicle(0)
    if veh then
      local vehPos = veh:getPosition()
      for _, node in ipairs(selectedNodes) do
        local npos = veh:getNodePosition(node.cid) + vehPos
        --dump{'npos', npos}
        debugDrawer:drawSphere(npos, 0.1, sphereColor)
        debugDrawer:drawText(npos, String(node.name), ColorF(0, 0, 0, 1))
      end
    end

  end

  local frameId = Engine.Render.getFrameId()
  local vId = be:getPlayerVehicleID(0)
  local vData = core_vehicle_manager.getVehicleData(vId)
  local veh = getObjectByID(vId)

  for connection, subs in pairs(subscriptions) do
    for _, sub in ipairs(subs) do
      if sub.type == 'nodePositions' and sub.what == '*' and veh and vData then
        local data = {}
        local vPos = veh:getPosition()
        for _, node in pairs(vData.vdata.nodes) do
          local name = node.name or tostring(node.cid)
          local pos = veh:getNodePosition(node.cid)
          data[name] = pos:toTable()
        end
        server:send(connection, {cmd="data", type="nodePositions", frameId=frameId, vId=vId, nodePositions=data})
      elseif sub.type == 'velocity' and veh then
        local vel = veh:getVelocity()
        server:send(connection, {cmd="data", type="velocity", frameId=frameId, vId=vId, velocity=vel:toTable()})
      end
    end
  end
end

local function onExtensionLoaded()
  server = TCPServer:new('*', 7000)
end

local function onExtensionUnloaded()
  if server then
    server:destroy()
  end
end

M.onExtensionLoaded = onExtensionLoaded
M.onExtensionUnloaded = onExtensionUnloaded
M.onPreRender = onPreRender

return M
