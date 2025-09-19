local pcdLib = require('common.tech.pcdLib')

local M = {}

local DEFAULT_PATH = FS:getUserPath() .. 'virtual_lidar/latest.pcd'

local state = {
  enabled = false,
  outputPath = DEFAULT_PATH,
  tmpPath = DEFAULT_PATH .. '.tmp',
  throttle = 0.25,
  lastWrite = -math.huge
}

local function ensureDirectory(path)
  if not path then return end
  local dir = path:match('^(.*)[/\\][^/\\]+$')
  if dir and dir ~= '' then
    FS:directoryCreate(dir)
  end
end

local function configure(opts)
  opts = opts or {}
  if opts.path and opts.path ~= '' then
    state.outputPath = opts.path
  elseif not state.outputPath then
    state.outputPath = DEFAULT_PATH
  end
  state.tmpPath = state.outputPath .. '.tmp'
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
      segments[#segments + 1] = string.pack('<fff f', x, y, z, intensity)
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

function M.publish(frame, scan)
  if not state.enabled then return false end
  local throttled, now = shouldThrottle()
  if throttled then return false end
  if not frame or not frame.origin or not frame.dir or not frame.up then return false end
  scan = scan or {}

  local pcd = pcdLib.newPcd()
  if not pcd then return false end

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

  ensureConfigured()

  local tmpPath = state.tmpPath or (state.outputPath .. '.tmp')
  savePcd(pcd, tmpPath)
  FS:rename(tmpPath, state.outputPath)
  state.lastWrite = now or (os.clock and os.clock()) or state.lastWrite
  return true
end

return M
