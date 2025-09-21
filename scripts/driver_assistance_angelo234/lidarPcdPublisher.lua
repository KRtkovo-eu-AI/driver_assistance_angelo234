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

local userPathBase = nil
local toFilesystemPath

local function getUserPathBase()
  if userPathBase == nil or (userPathBase == false and FS and FS.getUserPath) then
    userPathBase = false
    if FS and FS.getUserPath then
      local ok, base = pcall(function()
        return FS:getUserPath()
      end)
      if ok and type(base) == 'string' and base ~= '' then
        base = base:gsub('\\', '/')
        if base:sub(-1) ~= '/' then
          base = base .. '/'
        end
        userPathBase = base
      end
    end
  end
  return userPathBase ~= false and userPathBase or nil
end

local function computeDefaultPath()
  local base = getUserPathBase()
  if base then
    return base .. 'settings/krtektm_lidar/latest.pcd'
  end
  return 'settings/krtektm_lidar/latest.pcd'
end

local DEFAULT_PATH = computeDefaultPath()

local function buildCandidatePaths(path)
  if type(path) ~= 'string' or path == '' then return {} end

  local normalized = path:gsub('\\', '/')
  local candidates = {}

  local base = getUserPathBase()
  local function pushCandidate(value)
    if not value or value == '' then return end
    candidates[#candidates + 1] = value
  end

  local function appendUserSchemes(relative)
    if not relative then return end
    relative = relative:gsub('^/+', '')
    if relative == '' then
      pushCandidate('user://')
      pushCandidate('user:/')
      pushCandidate('/')
    else
      pushCandidate('user://' .. relative)
      pushCandidate('user:/' .. relative)
      pushCandidate('/' .. relative)
      if relative:sub(-1) ~= '/' then
        pushCandidate('/' .. relative .. '/')
      end
    end
  end

  if normalized:sub(1, 7) == 'user://'
      or normalized:sub(1, 6) == 'user:/' then
    local relative = normalized:gsub('^user:/+', '')
    if base then
      pushCandidate(base .. relative)
    end
    appendUserSchemes(relative)
  else
    pushCandidate(normalized)
    if base and normalized:sub(1, #base) == base then
      local relative = normalized:sub(#base + 1)
      appendUserSchemes(relative)
    end
  end

  return candidates
end

local state = {
  enabled = false,
  outputPath = DEFAULT_PATH,
  requestedPath = DEFAULT_PATH,
  tmpPath = DEFAULT_PATH .. '.tmp',
  throttle = 0.25,
  lastWrite = -math.huge,
  writeMode = 'atomic',
  lastErrorMessage = nil,
  lastErrorTime = -math.huge,
  sandboxFallback = nil,
  pcdLibBlocked = false
}

local lfsLib = nil
local lfsChecked = false

local function getLfs()
  if not lfsChecked then
    lfsChecked = true
    local ok, lib = pcall(require, 'lfs')
    if ok then
      lfsLib = lib
    else
      lfsLib = nil
    end
  end
  return lfsLib
end

local function directoryExistsOnDisk(path)
  if not path or path == '' then return false end

  local lfs = getLfs()
  if lfs then
    local attr = lfs.attributes(path)
    if attr == 'directory' then
      return true
    end
    if type(attr) == 'table' then
      if attr.mode == 'directory' then
        return true
      elseif attr.mode ~= nil then
        return false
      end
    elseif attr ~= nil then
      return false
    end
  end

  local ok, _, code = os.rename(path, path)
  if ok or code == 13 then
    return true
  end

  return false
end

local function normalizePath(path)
  if not path then return nil end
  local normalized = path:gsub('\\', '/')
  normalized = normalized:gsub('/+', '/')
  return normalized
end

local function getUserRelativePath(fsPath)
  local base = getUserPathBase()
  if not base or not fsPath then return nil end
  local normalizedBase = normalizePath(base)
  local normalizedPath = normalizePath(fsPath)
  if not normalizedBase or not normalizedPath then return nil end
  local compareBase = normalizedBase
  local comparePath = normalizedPath
  if compareBase:match('^%a:[/]') then
    compareBase = compareBase:lower()
    comparePath = comparePath:lower()
  end
  if comparePath:sub(1, #compareBase) == compareBase then
    local relative = normalizedPath:sub(#normalizedBase + 1)
    relative = relative:gsub('^/+', '')
    return relative ~= '' and relative or nil
  end
  return nil
end

local function guessUserRelativePath(fsPath)
  if not fsPath or fsPath == '' then return nil end
  local normalized = normalizePath(fsPath)
  if not normalized or normalized == '' then return nil end

  local relative = getUserRelativePath(fsPath)
  if relative then return relative end

  if not normalized:match('^%a:[/]') and normalized:sub(1, 1) ~= '/' then
    return normalized
  end

  local rootMatch, remainder = normalized:match('^(.*BeamNG%.drive/)()')
  if rootMatch and remainder then
    local rest = normalized:sub(remainder)
    if rest and rest ~= '' then
      local first, tail = rest:match('^([^/]+)/(.+)$')
      if first and tail then
        local lower = first:lower()
        if lower == 'current' or lower == 'latest' or lower:match('^%d') then
          tail = tail:gsub('^/+', '')
          if tail ~= '' then
            return tail
          end
        end
      end
      rest = rest:gsub('^/+', '')
      if rest ~= '' then
        return rest
      end
    end
  end

  return nil
end

local function getSandboxFriendlyPath(path)
  if not path or path == '' then return path end

  if path:sub(1, 7) == 'user://' or path:sub(1, 6) == 'user:/' then
    local relative = path:gsub('^user:/+', '')
    if relative ~= '' then
      return '/' .. relative
    end
    return path
  end

  if path:match('^%a:[/]') or path:sub(1, 2) == '\\' then
    local relative = guessUserRelativePath(path)
    if relative and relative ~= '' then
      return '/' .. relative
    end
  end

  return path
end

local function ensureDirectoryWithFs(relativePath)
  if not relativePath or relativePath == '' or not FS then return false end

  local normalized = normalizePath(relativePath)
  if not normalized or normalized == '' then return false end

  normalized = normalized:gsub('^/+', '')
  if normalized == '' then return false end

  local base = getUserPathBase()

  local function variantToFsPath(path)
    if not path or path == '' then return nil end
    if path:match('^%a:[/]') then
      return path
    end
    if base and path:sub(1, #base) == base then
      return path
    end
    if path:sub(1, 7) == 'user://' or path:sub(1, 6) == 'user:/' then
      local relative = path:gsub('^user:/+', '')
      return base and (base .. relative) or nil
    end
    if path:sub(1, 1) == '/' then
      local relative = path:gsub('^/+', '')
      return base and (base .. relative) or nil
    end
    return base and (base .. path) or nil
  end

  local function eachFsVariant(subPath)
    local variants = {}
    local seen = {}

    local function push(value)
      if not value or value == '' or seen[value] then return end
      variants[#variants + 1] = value
      seen[value] = true
    end

    if base then
      local absolute = base .. subPath
      push(absolute)
      if absolute:sub(-1) ~= '/' then
        push(absolute .. '/')
      end
    end

    push(subPath)
    if subPath:sub(-1) ~= '/' then
      push(subPath .. '/')
    end

    if subPath:sub(1, 1) ~= '/' then
      push('user://' .. subPath)
      push('user://' .. subPath:gsub('/*$', '') .. '/')
      push('user:/' .. subPath)
      push('user:/' .. subPath:gsub('/*$', '') .. '/')
      push('/' .. subPath)
      if subPath:sub(-1) ~= '/' then
        push('/' .. subPath .. '/')
      end
    end

    return variants
  end

  local function fsDirectoryExists(path)
    if not FS or not FS.directoryExists then return nil end
    local ok, res = pcall(function()
      return FS:directoryExists(path)
    end)
    if not ok then return nil end
    if res == nil then
      local fsPath = variantToFsPath(path)
      if directoryExistsOnDisk(fsPath) then
        return true
      end
      return false
    end
    if res == false then return false end
    if type(res) == 'boolean' then
      return res
    end
    return true
  end

  local function fsDirectoryCreate(path, recursive)
    if not FS or not FS.directoryCreate then return false end
    local ok, res = pcall(function()
      return FS:directoryCreate(path, recursive)
    end)
    if not ok then return false end
    if res == false then
      return false
    end
    local exists = fsDirectoryExists(path)
    if exists == nil then
      local fsPath = variantToFsPath(path)
      if directoryExistsOnDisk(fsPath) then
        return true
      end
      -- Treat indeterminate FS responses as failures so fallbacks can still
      -- attempt to create the directory on disk.
      return false
    end
    if exists then
      return true
    end
    return false
  end

  local segments = {}
  for segment in normalized:gmatch('[^/]+') do
    segments[#segments + 1] = segment
  end
  if #segments == 0 then return false end

  local prefix = ''
  for i = 1, #segments do
    prefix = prefix == '' and segments[i] or (prefix .. '/' .. segments[i])

    local ensured = false
    local variants = eachFsVariant(prefix)

    for j = 1, #variants do
      if fsDirectoryExists(variants[j]) then
        ensured = true
        break
      end
    end

    if not ensured then
      for j = 1, #variants do
        local variant = variants[j]
        if fsDirectoryCreate(variant) or fsDirectoryCreate(variant, true) then
          ensured = true
          break
        end
      end
    end

    if not ensured then
      return false
    end
  end

  return true
end

local function ensureDirectoryWithLfs(fsPath)
  if not fsPath or fsPath == '' then return false end
  local lfs = getLfs()
  if not lfs then return false end

  local normalized = normalizePath(fsPath)
  if not normalized or normalized == '' then return false end

  local prefix = ''
  local remainder = normalized

  if remainder:match('^%a:[/]') then
    prefix = remainder:sub(1, 3)
    remainder = remainder:sub(4)
  elseif remainder:sub(1, 1) == '/' then
    prefix = '/'
    remainder = remainder:sub(2)
  end

  local function ensurePrefix(current)
    local attr = lfs.attributes(current)
    if attr == 'directory' then
      return true
    elseif attr ~= nil then
      return false
    end
    local ok, err = lfs.mkdir(current)
    if ok or err == 'File exists' then
      return true
    end
    return false
  end

  if prefix ~= '' and not ensurePrefix(prefix) then
    return false
  end

  local current = prefix
  for segment in remainder:gmatch('[^/]+') do
    if current ~= '' and current:sub(-1) ~= '/' then
      current = current .. '/'
    end
    current = current .. segment
    if not ensurePrefix(current) then
      return false
    end
  end

  return true
end

local function ensureDirectory(path)
  if not path or path == '' then return end
  local candidates = buildCandidatePaths(path)
  if #candidates == 0 then return end

  for i = 1, #candidates do
    local dir = candidates[i]:match('^(.*)[/\\][^/\\]+$')
    if dir then
      local fsDir = toFilesystemPath(dir)
      local ensured = false
      if fsDir then
        local relative = guessUserRelativePath(fsDir)
        if relative then
          ensured = ensureDirectoryWithFs(relative)
        end
        if not ensured then
          ensured = ensureDirectoryWithLfs(fsDir)
        end
      end

      if not ensured and dir ~= fsDir then
        local relativeFromCandidate = dir
        if relativeFromCandidate then
          relativeFromCandidate = relativeFromCandidate:gsub('^user:/+', '')
          relativeFromCandidate = relativeFromCandidate:gsub('^/+', '')
          ensured = ensureDirectoryWithFs(relativeFromCandidate)
        end
      end

      if ensured then
        return
      end
    end
  end
end

local function fileExists(path)
  if not path or path == '' then return false end
  local candidates = buildCandidatePaths(path)
  for i = 1, #candidates do
    local candidate = candidates[i]
    local fsCandidate = candidate
    if FS and FS.fileExists then
      local ok, result = pcall(function()
        return FS:fileExists(candidate)
      end)
      if ok and result ~= nil and result ~= false then
        return true
      end
    end
    if fsCandidate:sub(1, 6) == 'user:/' then
      local base = getUserPathBase()
      if base then
        local relative = fsCandidate:gsub('^user:/+', '')
        fsCandidate = base .. relative
      else
        fsCandidate = nil
      end
    end
    local file = fsCandidate and io.open(fsCandidate, 'rb') or nil
    if file then
      file:close()
      return true
    end
  end
  return false
end

toFilesystemPath = function(path)
  if not path or path == '' then return nil end
  if FS and FS.getFileRealPath then
    local ok, real = pcall(function()
      return FS:getFileRealPath(path)
    end)
    if ok and type(real) == 'string' and real ~= '' then
      real = real:gsub('\\', '/')
      return real
    end
  end

  local normalized = normalizePath(path)
  if not normalized or normalized == '' then
    return nil
  end

  local base = getUserPathBase()
  local function resolveWithBase(relative)
    if not base then return nil end
    if not relative or relative == '' then
      return base
    end
    return base .. relative
  end

  if normalized:sub(1, 7) == 'user://' or normalized:sub(1, 6) == 'user:/' then
    local relative = normalized:gsub('^user:/+', '')
    return resolveWithBase(relative) or normalized
  end

  if normalized:sub(1, 2) == '//' then
    return normalized
  end

  if normalized:match('^%a:[/]') then
    return normalized
  end

  if base then
    if normalized:sub(1, #base) == base then
      return normalized
    end
    if normalized:sub(1, 1) == '/' then
      local relative = normalized:gsub('^/+', '')
      return resolveWithBase(relative) or normalized
    end
    return base .. normalized
  end

  return normalized
end

local function buildWriteAttempts(path)
  local attempts = {}
  local seen = {}

  local function push(value)
    if not value or value == '' or seen[value] then return end
    attempts[#attempts + 1] = value
    seen[value] = true
  end

  local friendly = getSandboxFriendlyPath(path)
  if friendly and friendly ~= path then
    push(friendly)
  end

  if path and path ~= '' and path:sub(1, 5) ~= 'user:' then
    push(path)
  end

  local fsPath = toFilesystemPath(path)
  if fsPath and fsPath ~= path and fsPath ~= friendly then
    push(fsPath)
  end

  return attempts
end

local function ensureFile(path)
  if not path or path == '' then return end
  if fileExists(path) then return end
  local candidates = buildCandidatePaths(path)
  for i = 1, #candidates do
    local candidate = candidates[i]
    local attempts = buildWriteAttempts(candidate)
    for j = 1, #attempts do
      local target = attempts[j]
      ensureDirectory(target)
      local ok, created = pcall(function()
        local file = io.open(target, 'wb')
        if file then
          file:close()
          return true
        end
        return false
      end)
      if ok and created then
        return
      end
    end
  end
end

local function renameFile(from, to)
  if not from or not to then return false, 'invalid path' end
  local fromCandidates = buildCandidatePaths(from)
  local toCandidates = buildCandidatePaths(to)
  local lastErr = nil
  for i = 1, #fromCandidates do
    for j = 1, #toCandidates do
      local candidateFrom = fromCandidates[i]
      local candidateTo = toCandidates[j]
      if FS and FS.rename then
        local ok, res = pcall(function()
          return FS:rename(candidateFrom, candidateTo)
        end)
        if ok and res ~= false then
          return true
        end
        if not ok then
          lastErr = res or lastErr
        end
      end
    end
  end
  return false, lastErr or 'rename unavailable'
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
    state.requestedPath = opts.path
    state.sandboxFallback = nil
  elseif not state.outputPath then
    state.outputPath = DEFAULT_PATH
    state.requestedPath = state.requestedPath or state.outputPath
  end
  if not state.requestedPath then
    state.requestedPath = state.outputPath
  end
  state.tmpPath = state.outputPath .. '.tmp'
  if previousPath ~= state.outputPath then
    state.writeMode = 'atomic'
    state.pcdLibBlocked = false
  end
  if opts.throttle then
    state.throttle = opts.throttle
  end
  if opts.enabled ~= nil then
    state.enabled = opts.enabled and true or false
  end
  ensureDirectory(state.outputPath)
  ensureFile(state.outputPath)
  if state.tmpPath and state.writeMode ~= 'direct' then
    ensureDirectory(state.tmpPath)
    ensureFile(state.tmpPath)
  end
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

local function callWithWriteAttempts(method, target, path)
  local attempts = buildWriteAttempts(path)
  for i = 1, #attempts do
    local candidate = attempts[i]
    local result = method(target, candidate)
    if result ~= false then
      return result
    end
  end
  return false
end

local function savePcd(pcd, path)
  if pcd.save then
    return callWithWriteAttempts(function(self, candidate)
      return self:save(candidate)
    end, pcd, path)
  elseif pcd.write then
    return callWithWriteAttempts(function(self, candidate)
      return self:write(candidate)
    end, pcd, path)
  elseif pcd.saveToFile then
    return callWithWriteAttempts(function(self, candidate)
      return self:saveToFile(candidate)
    end, pcd, path)
  elseif pcd.writeToFile then
    return callWithWriteAttempts(function(self, candidate)
      return self:writeToFile(candidate)
    end, pcd, path)
  elseif pcd.store then
    return callWithWriteAttempts(function(self, candidate)
      return self:store(candidate)
    end, pcd, path)
  else
    local attempts = buildWriteAttempts(path)
    for i = 1, #attempts do
      local candidate = attempts[i]
      local file = io.open(candidate, 'wb')
      if file then
        if pcd.toString then
          file:write(pcd:toString())
        elseif pcd.data then
          file:write(pcd.data)
        end
        file:close()
        return true
      end
    end
  end
  return false
end

local function safeIndex(value, key)
  if value == nil then return nil end
  local ok, result = pcall(function()
    return value[key]
  end)
  if ok and type(result) == 'number' then
    return result
  end
  return nil
end

local function extractVec3(v)
  if not v then return 0, 0, 0 end
  if type(v) == 'table' then
    local x = v.x or v[1] or 0
    local y = v.y or v[2] or 0
    local z = v.z or v[3] or 0
    return x, y, z
  end
  local x = safeIndex(v, 'x') or safeIndex(v, 1) or 0
  local y = safeIndex(v, 'y') or safeIndex(v, 2) or 0
  local z = safeIndex(v, 'z') or safeIndex(v, 3) or 0
  return x, y, z
end

local function extractQuat(q)
  if not q then return 1, 0, 0, 0 end
  if type(q) == 'table' then
    local qw = q.w or q[4] or 1
    local qx = q.x or q[1] or 0
    local qy = q.y or q[2] or 0
    local qz = q.z or q[3] or 0
    return qw, qx, qy, qz
  end
  local qw = safeIndex(q, 'w') or safeIndex(q, 4) or 1
  local qx = safeIndex(q, 'x') or safeIndex(q, 1) or 0
  local qy = safeIndex(q, 'y') or safeIndex(q, 2) or 0
  local qz = safeIndex(q, 'z') or safeIndex(q, 3) or 0
  return qw, qx, qy, qz
end

local function formatFloat(value)
  if type(value) ~= 'number' then
    value = tonumber(value) or 0
  end
  return string.format('%.9g', value)
end

local function writeBinaryPcd(path, payload, count, origin, quat)
  local candidates = buildCandidatePaths(path)
  if #candidates == 0 then
    return false, 'invalid path'
  end

  local ox, oy, oz = extractVec3(origin)
  local qw, qx, qy, qz = extractQuat(quat)

  local header = table.concat({
    '# .PCD v0.7 - Point Cloud Data file format\n',
    'VERSION 0.7\n',
    'FIELDS x y z intensity\n',
    'SIZE 4 4 4 4\n',
    'TYPE F F F F\n',
    'COUNT 1 1 1 1\n',
    'WIDTH ' .. tostring(count) .. '\n',
    'HEIGHT 1\n',
    string.format(
      'VIEWPOINT %s %s %s %s %s %s %s\n',
      formatFloat(ox),
      formatFloat(oy),
      formatFloat(oz),
      formatFloat(qw),
      formatFloat(qx),
      formatFloat(qy),
      formatFloat(qz)
    ),
    'POINTS ' .. tostring(count) .. '\n',
    'DATA binary\n'
  })

  local lastErr = nil
  for i = 1, #candidates do
    local candidate = candidates[i]
    local attempts = buildWriteAttempts(candidate)
    for j = 1, #attempts do
      local target = attempts[j]
      ensureDirectory(target)
      local file, err = io.open(target, 'wb')
      if file then
        local okHeader, headerErr = file:write(header)
        if not okHeader then
          file:close()
          lastErr = headerErr or 'failed to write header'
        else
          local payloadErr = nil
          if payload and #payload > 0 then
            local okPayload, writeErr = file:write(payload)
            if not okPayload then
              payloadErr = writeErr or 'failed to write payload'
            end
          end
          file:close()
          if not payloadErr then
            return true
          end
          lastErr = payloadErr
        end
      else
        lastErr = err or lastErr
      end
    end
  end

  return false, lastErr or 'failed to open file'
end

local function shouldFallbackToDefault(err)
  if not err then return false end
  local msg = string.lower(tostring(err))
  if msg:find('domain') or msg:find('permission') or msg:find('denied') or msg:find('access') then
    return true
  end
  return false
end

local function applySandboxFallback(pcd, err, payload, count, origin, viewpointQuat)
  if state.outputPath == DEFAULT_PATH or not shouldFallbackToDefault(err) then
    return false, err
  end

  local blockedPath = state.requestedPath or state.outputPath
  local fallbackPath = DEFAULT_PATH
  ensureDirectory(fallbackPath)

  local wroteFallback = false
  local fallbackErr = nil

  local attemptedAtomic = false

  if pcd and not state.pcdLibBlocked then
    attemptedAtomic = true
    local ok, saveErr = pcall(savePcd, pcd, fallbackPath)
    if ok and saveErr ~= false then
      wroteFallback = true
    else
      fallbackErr = saveErr or fallbackErr
      state.pcdLibBlocked = true
    end
  end

  if not wroteFallback then
    local ok, directErr = writeBinaryPcd(fallbackPath, payload, count, origin, viewpointQuat)
    if ok then
      wroteFallback = true
      state.writeMode = 'direct'
      state.pcdLibBlocked = true
    else
      fallbackErr = directErr or fallbackErr
    end
  elseif attemptedAtomic then
    state.writeMode = 'atomic'
    state.pcdLibBlocked = false
  end

  if wroteFallback then
    if reportError then
      reportError(string.format(
        'Virtual LiDAR export path %s is not writable (%s); using default path %s instead',
        tostring(blockedPath),
        tostring(err or 'unknown'),
        tostring(fallbackPath)
      ))
    end
    state.outputPath = fallbackPath
    state.tmpPath = fallbackPath .. '.tmp'
    state.sandboxFallback = {
      blockedPath = blockedPath,
      reason = tostring(err or 'unknown')
    }
    return true
  end

  return false, fallbackErr or err
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
  ensureFile(state.outputPath)
  if state.tmpPath and state.writeMode ~= 'direct' then
    ensureDirectory(state.tmpPath)
    ensureFile(state.tmpPath)
  end
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
  count = count + appendPoints(segments, scan.overhead, 0.6)

  local payload = table.concat(segments)

  applyPayload(pcd, payload, count)

  local origin = frame.origin
  local dir = frame.dir
  local up = frame.up
  local viewpointQuat = nil

  if pcd.setViewpoint and quatFromDir and origin and dir and up then
    viewpointQuat = quatFromDir(dir, up)
    pcd:setViewpoint(origin, viewpointQuat)
  end

  if writeFile then
    ensureConfigured()

    local writeSucceeded = false
    local writeError = nil

    if state.writeMode ~= 'direct' and state.tmpPath and not state.pcdLibBlocked then
      local ok, err = pcall(savePcd, pcd, state.tmpPath)
      if ok and err ~= false then
        if fileExists(state.tmpPath) then
          local renamed, renameErr = renameFile(state.tmpPath, state.outputPath)
          if renamed then
            writeSucceeded = true
          else
            writeError = string.format('Failed to rename temporary PCD file (%s)', tostring(renameErr or 'unknown'))
            state.writeMode = 'direct'
            state.pcdLibBlocked = true
          end
        else
          writeError = string.format('Failed to write temporary PCD file (%s)', tostring(err or 'temporary file missing'))
          state.writeMode = 'direct'
          state.pcdLibBlocked = true
        end
      else
        writeError = string.format('Failed to write temporary PCD file (%s)', tostring(err or 'unknown'))
        state.writeMode = 'direct'
        state.pcdLibBlocked = true
      end
    end

    if not writeSucceeded then
      if state.writeMode == 'direct' then
        local ok, err = writeBinaryPcd(state.outputPath, payload, count, origin, viewpointQuat)
        if ok then
          writeSucceeded = true
          if writeError then
            reportInfo('Virtual LiDAR export fell back to direct writes for path: ' .. tostring(state.outputPath))
          end
          state.sandboxFallback = nil
        else
          local fallbackOk, fallbackErr = applySandboxFallback(pcd, err, payload, count, origin, viewpointQuat)
          if fallbackOk then
            writeSucceeded = true
            writeError = nil
          else
            writeError = writeError or string.format('Failed to write PCD file (%s)', tostring(fallbackErr or err or 'unknown'))
          end
        end
      else
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

function M.getOutputPath()
  return state.outputPath
end

function M.getRequestedPath()
  return state.requestedPath or state.outputPath
end

function M.getSandboxFallback()
  return state.sandboxFallback
end

return M
