local ok, pcdLib = pcall(require, 'common.tech.pcdLib')
if not ok or not pcdLib then
  local missingLibError = not ok and pcdLib or nil

  local function createStubPcd()
    local stub = {
      fields = {}
    }

    function stub:clearFields()
      self.fields = {}
    end

    function stub:addField(name, ftype, size)
      self.fields[#self.fields + 1] = {name = name, type = ftype, size = size}
    end

    function stub:setBinaryData(payload, count)
      self.data = payload
      self.pointCount = count
    end

    function stub:setPayload(payload, count)
      self:setBinaryData(payload, count)
    end

    function stub:setData(payload)
      self.data = payload
    end

    local function assignCount(self, count)
      self.pointCount = count
    end

    function stub:setPointCount(count)
      assignCount(self, count)
    end

    stub.setCount = stub.setPointCount
    stub.setPoints = stub.setPointCount
    stub.setPointNumber = stub.setPointCount

    function stub:setViewpoint(origin, quat)
      self.viewpoint = {origin = origin, rotation = quat}
    end

    local function saveTo(self, path)
      local file = io.open(path, 'wb')
      if file then
        if self.data then
          file:write(self.data)
        end
        file:close()
      end
    end

    function stub:save(path)
      saveTo(self, path)
    end

    stub.write = stub.save
    stub.saveToFile = stub.save
    stub.writeToFile = stub.save
    stub.store = stub.save

    function stub:toString()
      return self.data or ''
    end

    return stub
  end

  pcdLib = {
    newPcd = function()
      return createStubPcd()
    end,
    _missing_error = missingLibError
  }

  package.loaded['common.tech.pcdLib'] = pcdLib
end

local M = {}

local packPoint

do
  local hasStringPack = type(string.pack) == 'function'

  if hasStringPack then
    packPoint = function(x, y, z, intensity)
      return string.pack('<ffff', x, y, z, intensity)
    end
  else
    local function encodeFloat(value)
      if value ~= value then
        return 0xffc00000
      end
      if value == math.huge then
        return 0x7f800000
      end
      if value == -math.huge then
        return 0xff800000
      end

      local sign = 0
      if value < 0 or (value == 0 and 1 / value < 0) then
        sign = 0x80000000
        value = -value
      end

      if value == 0 then
        return sign
      end

      local mantissa, exponent = math.frexp(value)
      exponent = exponent - 1
      mantissa = mantissa * 2 - 1
      exponent = exponent + 127

      if exponent <= 0 then
        mantissa = math.floor((mantissa + 1) * math.ldexp(1, exponent + 22) + 0.5)
        exponent = 0
      else
        mantissa = math.floor(mantissa * 0x800000 + 0.5)
      end

      if mantissa == 0x800000 then
        exponent = exponent + 1
        mantissa = 0
      end

      if exponent >= 255 then
        exponent = 255
        mantissa = 0
      end

      return sign + exponent * 0x800000 + mantissa
    end

    local function packFloat(value)
      local bits = encodeFloat(value)
      local b1 = bits % 256
      bits = (bits - b1) / 256
      local b2 = bits % 256
      bits = (bits - b2) / 256
      local b3 = bits % 256
      bits = (bits - b3) / 256
      local b4 = bits % 256
      return string.char(b1, b2, b3, b4)
    end

    packPoint = function(x, y, z, intensity)
      return packFloat(x) .. packFloat(y) .. packFloat(z) .. packFloat(intensity)
    end
  end
end

local function computeDefaultPath()
  if FS and FS.getUserPath then
    local ok, base = pcall(function()
      return FS:getUserPath()
    end)
    if ok and type(base) == 'string' and base ~= '' then
      return base .. 'virtual_lidar/latest.pcd'
    end
  end
  return 'virtual_lidar/latest.pcd'
end

local DEFAULT_PATH = computeDefaultPath()

local state = {
  enabled = false,
  outputPath = DEFAULT_PATH,
  tmpPath = DEFAULT_PATH .. '.tmp',
  throttle = 0.25,
  lastWrite = -math.huge,
  writeMode = 'atomic',
  lastErrorMessage = nil,
  lastErrorTime = -math.huge
}

local function ensureDirectory(path)
  if not path then return end
  local dir = path:match('^(.*)[/\\][^/\\]+$')
  if dir and dir ~= '' then
    if FS and FS.directoryCreate then
      FS:directoryCreate(dir)
    elseif os and os.execute then
      local escaped = dir:gsub("'", "'\\''")
      os.execute("mkdir -p '" .. escaped .. "'")
    end
  end
end

local function renameFile(from, to)
  if not from or not to then return false, 'invalid path' end
  if FS and FS.rename then
    local ok, res = pcall(function()
      return FS:rename(from, to)
    end)
    if not ok then
      return false, res
    end
    if res == false then
      return false
    end
    return true
  elseif os and os.rename then
    local ok, err = os.rename(from, to)
    if ok then
      return true
    end
    return false, err
  end
  return false, 'rename unavailable'
end

local function reportError(msg)
  if not msg then return end
  local now = os.clock and os.clock() or 0
  if state.lastErrorMessage ~= msg or (now - state.lastErrorTime) > 2 then
    if log then
      log('E', 'virtualLidarPcd', msg)
    end
    state.lastErrorMessage = msg
    state.lastErrorTime = now
  end
end

local function reportInfo(msg)
  if not msg then return end
  if log then
    log('I', 'virtualLidarPcd', msg)
  end
end

local function configure(opts)
  opts = opts or {}
  local previousPath = state.outputPath
  if opts.path and opts.path ~= '' then
    state.outputPath = opts.path
  elseif not state.outputPath then
    state.outputPath = DEFAULT_PATH
  end
  state.tmpPath = state.outputPath .. '.tmp'
  if previousPath ~= state.outputPath then
    state.writeMode = 'atomic'
  end
  if opts.throttle then
    state.throttle = opts.throttle
  end
  if opts.enabled ~= nil then
    state.enabled = opts.enabled and true or false
  end
  ensureDirectory(state.outputPath)
end

local function appendPoints(segments, points, intensity)
  if not points then return 0 end
  local count = 0
  for i = 1, #points do
    local pt = points[i]
    if pt then
      local x = pt.x or pt[1] or 0
      local y = pt.y or pt[2] or 0
      local z = pt.z or pt[3] or 0
      segments[#segments + 1] = packPoint(x, y, z, intensity)
      count = count + 1
    end
  end
  return count
end

local function applyPayload(pcd, payload, pointCount)
  if pcd.setBinaryData then
    pcd:setBinaryData(payload, pointCount)
  elseif pcd.setPayload then
    pcd:setPayload(payload, pointCount)
  elseif pcd.setData then
    pcd:setData(payload)
    if pcd.setPointCount then
      pcd:setPointCount(pointCount)
    elseif pcd.setCount then
      pcd:setCount(pointCount)
    end
  else
    pcd.data = payload
    pcd.pointCount = pointCount
  end
  if pcd.setPointCount then
    pcd:setPointCount(pointCount)
  elseif pcd.setCount then
    pcd:setCount(pointCount)
  elseif pcd.setPoints then
    pcd:setPoints(pointCount)
  elseif pcd.setPointNumber then
    pcd:setPointNumber(pointCount)
  end
end

local function savePcd(pcd, path)
  if pcd.save then
    pcd:save(path)
  elseif pcd.write then
    pcd:write(path)
  elseif pcd.saveToFile then
    pcd:saveToFile(path)
  elseif pcd.writeToFile then
    pcd:writeToFile(path)
  elseif pcd.store then
    pcd:store(path)
  else
    local file = io.open(path, 'wb')
    if file then
      if pcd.toString then
        file:write(pcd:toString())
      elseif pcd.data then
        file:write(pcd.data)
      end
      file:close()
    end
  end
end

function M.configure(opts)
  configure(opts)
end

function M.setEnabled(flag)
  state.enabled = flag and true or false
end

local function shouldThrottle()
  local now = os.clock and os.clock() or 0
  if not now then return false, now end
  if state.lastWrite ~= -math.huge and now - state.lastWrite < state.throttle then
    return true, now
  end
  return false, now
end

local function ensureConfigured()
  if not state.outputPath then
    state.outputPath = DEFAULT_PATH
    state.tmpPath = state.outputPath .. '.tmp'
  end
  ensureDirectory(state.outputPath)
end

function M.publish(frame, scan, opts)
  opts = opts or {}
  local writeFile = opts.writeFile
  if writeFile == nil then
    writeFile = state.enabled
  end
  local wantPayload = opts.wantPayload
  if wantPayload == nil then
    wantPayload = writeFile
  end
  if not writeFile and not wantPayload then return nil end

  local throttled, now = shouldThrottle()
  if throttled then return nil end
  if not frame or not frame.origin or not frame.dir or not frame.up then return nil end
  scan = scan or {}

  local pcd = pcdLib.newPcd()
  if not pcd then return nil end

  if pcd.clearFields then pcd:clearFields() end
  if pcd.addField then
    pcd:addField('x', 'F', 4)
    pcd:addField('y', 'F', 4)
    pcd:addField('z', 'F', 4)
    pcd:addField('intensity', 'F', 4)
  end

  local segments = {}
  local count = 0
  count = count + appendPoints(segments, scan.main, 1.0)
  count = count + appendPoints(segments, scan.ground, 0.2)
  count = count + appendPoints(segments, scan.vehicle, 0.8)

  local payload = table.concat(segments)

  applyPayload(pcd, payload, count)

  if pcd.setViewpoint and quatFromDir then
    local origin = frame.origin
    local dir = frame.dir
    local up = frame.up
    local q = quatFromDir(dir, up)
    pcd:setViewpoint(origin, q)
  end

  if writeFile then
    ensureConfigured()

    local writeSucceeded = false
    local writeError = nil

    if state.writeMode ~= 'direct' and state.tmpPath then
      local ok, err = pcall(savePcd, pcd, state.tmpPath)
      if ok and err ~= false then
        local renamed, renameErr = renameFile(state.tmpPath, state.outputPath)
        if renamed then
          writeSucceeded = true
        else
          writeError = string.format('Failed to rename temporary PCD file (%s)', tostring(renameErr or 'unknown'))
          state.writeMode = 'direct'
        end
      else
        writeError = string.format('Failed to write temporary PCD file (%s)', tostring(err or 'unknown'))
        state.writeMode = 'direct'
      end
    end

    if not writeSucceeded then
      local ok, err = pcall(savePcd, pcd, state.outputPath)
      if ok and err ~= false then
        writeSucceeded = true
        if writeError then
          reportInfo('Virtual LiDAR export fell back to direct writes for path: ' .. tostring(state.outputPath))
        end
      else
        writeError = writeError or string.format('Failed to write PCD file (%s)', tostring(err or 'unknown'))
      end
    end

    if not writeSucceeded and writeError then
      reportError(string.format('Virtual LiDAR PCD export error for %s: %s', tostring(state.outputPath), writeError))
    end
  end

  state.lastWrite = now or (os.clock and os.clock()) or state.lastWrite
  if wantPayload then
    return payload
  end
  return true
end

return M
