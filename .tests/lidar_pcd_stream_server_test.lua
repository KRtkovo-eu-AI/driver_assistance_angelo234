local laura = require('laura')
local describe = laura.describe
local it = laura.it
local expect = laura.expect
local hooks = laura.hooks

local original_socket_module = package.loaded['socket']
local original_socket_preload = package.preload['socket']
local original_logger_module = package.loaded['scripts/driver_assistance_angelo234/logger']

local function makeClientStub(options)
  options = options or {}
  local client = {
    sends = {},
    closed = false,
    setoptionCalls = {},
  }

  function client:settimeout(value)
    self.timeout = value
  end

  if options.disableSetOption ~= true then
    function client:setoption(opt, value)
      self.setoptionCalls[#self.setoptionCalls + 1] = { opt = opt, value = value }
      if options.failSetOption then
        error('setoption failure')
      end
      return true
    end
  end

  function client:send(data, index)
    local chunk = data
    if index and index > 1 then
      chunk = data:sub(index)
    end
    self.sends[#self.sends + 1] = chunk
    if options.failOnSend then
      return nil, options.failOnSend
    end
    return #data
  end

  function client:close()
    self.closed = true
  end

  return client
end

local function makeServerStub()
  local server = {
    acceptQueue = {},
    closed = false,
  }

  function server:settimeout(value)
    self.timeout = value
  end

  function server:accept()
    if #self.acceptQueue == 0 then
      return nil, 'timeout'
    end

    local nextItem = table.remove(self.acceptQueue, 1)
    if type(nextItem) == 'table' and nextItem.error then
      return nil, nextItem.error
    end

    return nextItem
  end

  function server:close()
    self.closed = true
  end

  return server
end

local function makeFakeSocket()
  local stub = {
    binds = {},
    nextServer = nil,
    failNextBind = false,
    nextError = 'bind failed',
  }

  function stub.bind(host, port)
    stub.binds[#stub.binds + 1] = { host = host, port = port }
    if stub.failNextBind then
      stub.failNextBind = false
      return nil, stub.nextError
    end

    local server = stub.nextServer or makeServerStub()
    stub.nextServer = nil
    stub.lastServer = server
    server.boundHost = host
    server.boundPort = port
    return server
  end

  return stub
end

describe('lidarPcdStreamServer', function()
  local fakeSocket
  local logs
  local moduleUnderTest

  local function loadModule()
    package.loaded['scripts/driver_assistance_angelo234/lidarPcdStreamServer'] = nil
    moduleUnderTest = require('scripts/driver_assistance_angelo234/lidarPcdStreamServer')
    return moduleUnderTest
  end

  hooks.beforeEach(function()
    logs = {}
    fakeSocket = makeFakeSocket()

    package.loaded['socket'] = nil
    package.preload['socket'] = function()
      return fakeSocket
    end

    local loggerStub = {}
    function loggerStub.log(level, tag, message)
      logs[#logs + 1] = { level = level, tag = tag, message = message }
    end
    package.loaded['scripts/driver_assistance_angelo234/logger'] = loggerStub

    package.loaded['scripts/driver_assistance_angelo234/lidarPcdStreamServer'] = nil
    moduleUnderTest = nil
  end)

  hooks.afterEach(function()
    if moduleUnderTest and moduleUnderTest.stop then
      moduleUnderTest.stop()
    end
    package.loaded['scripts/driver_assistance_angelo234/lidarPcdStreamServer'] = nil
    package.loaded['socket'] = original_socket_module
    package.preload['socket'] = original_socket_preload
    package.loaded['scripts/driver_assistance_angelo234/logger'] = original_logger_module
  end)

  it('accepts clients and broadcasts PCD payloads with headers', function()
    local serverStub = makeServerStub()
    local clientA = makeClientStub()
    local clientB = makeClientStub()
    serverStub.acceptQueue = { clientA, clientB }
    fakeSocket.nextServer = serverStub

    local server = loadModule()
    local ok = server.start('127.0.0.1', 8765)
    expect(ok).toBeTruthy()

    server.update(0.0)
    expect(server.clientCount()).toEqual(2)
    expect(clientA.timeout).toEqual(0)
    expect(clientB.timeout).toEqual(0)
    expect(#clientA.setoptionCalls).toEqual(1)
    expect(clientA.setoptionCalls[1].opt).toEqual('tcp-nodelay')

    local payload = 'ABCD'
    server.broadcast(payload)

    expect(clientA.sends).toDeepEqual({ 'PCD 4\n', payload })
    expect(clientB.sends).toDeepEqual({ 'PCD 4\n', payload })
  end)

  it('removes clients that fail to send payloads', function()
    local serverStub = makeServerStub()
    local healthyClient = makeClientStub()
    local failingClient = makeClientStub({ failOnSend = 'closed' })
    serverStub.acceptQueue = { healthyClient, failingClient }
    fakeSocket.nextServer = serverStub

    local server = loadModule()
    expect(server.start('127.0.0.1', 9000)).toBeTruthy()

    server.update(0.0)
    expect(server.clientCount()).toEqual(2)

    server.broadcast('HELLO')

    expect(server.clientCount()).toEqual(1)
    expect(healthyClient.closed).toBeFalsy()
    expect(failingClient.closed).toBeTruthy()
    expect(healthyClient.sends).toDeepEqual({ 'PCD 5\n', 'HELLO' })
  end)

  it('sends heartbeat messages at the configured interval', function()
    local serverStub = makeServerStub()
    local client = makeClientStub()
    serverStub.acceptQueue = { client }
    fakeSocket.nextServer = serverStub

    local server = loadModule()
    expect(server.start('127.0.0.1', 7000)).toBeTruthy()

    server.setHeartbeatInterval(0.25)

    server.update(0.0) -- accept the pending client
    expect(server.clientCount()).toEqual(1)

    server.update(0.2)
    expect(#client.sends).toEqual(0)

    server.update(0.1)
    expect(client.sends).toDeepEqual({ 'PING\n' })

    server.update(0.25)
    expect(client.sends).toDeepEqual({ 'PING\n', 'PING\n' })
  end)

  it('stops the server and closes all clients', function()
    local serverStub = makeServerStub()
    local clientA = makeClientStub()
    local clientB = makeClientStub()
    serverStub.acceptQueue = { clientA, clientB }
    fakeSocket.nextServer = serverStub

    local server = loadModule()
    expect(server.start('127.0.0.1', 8800)).toBeTruthy()

    server.update(0.0)
    expect(server.clientCount()).toEqual(2)

    server.stop()

    expect(server.isRunning()).toBeFalsy()
    expect(server.clientCount()).toEqual(0)
    expect(serverStub.closed).toBeTruthy()
    expect(clientA.closed).toBeTruthy()
    expect(clientB.closed).toBeTruthy()
  end)

  it('logs an error when the server fails to bind', function()
    fakeSocket.failNextBind = true
    fakeSocket.nextError = 'address in use'

    local server = loadModule()
    local ok, err = server.start('127.0.0.1', 9100)

    expect(ok).toBeFalsy()
    expect(err).toEqual('address in use')
    expect(server.isRunning()).toBeFalsy()
    expect(#logs).toEqual(1)
    expect(logs[0]).toBeNil()
    expect(logs[1].level).toEqual('E')
    expect(logs[1].tag).toEqual('lidar')
    expect(string.find(logs[1].message, 'address in use', 1, true)).toBeTruthy()
  end)
end)

