local laura = require('laura')
local describe = laura.describe
local it = laura.it
local expect = laura.expect
local hooks = laura.hooks

local unpack = table.unpack or _G.unpack

local string_unpack = string.unpack

if not string_unpack then
  local function readFloatLE(data, index)
    local b1 = data:byte(index) or 0
    local b2 = data:byte(index + 1) or 0
    local b3 = data:byte(index + 2) or 0
    local b4 = data:byte(index + 3) or 0

    local bits = b1 + b2 * 256 + b3 * 65536 + b4 * 16777216
    local sign = 1
    if bits >= 0x80000000 then
      sign = -1
      bits = bits - 0x80000000
    end

    local exponent = math.floor(bits / 0x800000)
    local mantissa = bits % 0x800000

    if exponent == 255 then
      if mantissa == 0 then
        return sign * math.huge, index + 4
      end
      return 0 / 0, index + 4
    end

    if exponent == 0 then
      if mantissa == 0 then
        local zero = 0.0
        return sign > 0 and zero or -zero, index + 4
      end
      local value = mantissa / 0x800000
      return sign * math.ldexp(value, -126), index + 4
    end

    local value = 1 + mantissa / 0x800000
    return sign * math.ldexp(value, exponent - 127), index + 4
  end

  string_unpack = function(fmt, data, index)
    index = index or 1
    local values = {}
    local littleEndian = true

    for i = 1, #fmt do
      local c = fmt:sub(i, i)
      if c == '<' then
        littleEndian = true
      elseif c == '>' then
        littleEndian = false
      elseif c == 'f' then
        if not littleEndian then
          error('fallback string.unpack only supports little-endian floats')
        end
        local value
        value, index = readFloatLE(data, index)
        values[#values + 1] = value
      end
    end

    values[#values + 1] = index
    return unpack(values, 1, #values)
  end

  string.unpack = string_unpack
end

local path_sep = package.config:sub(1, 1)
local is_windows = path_sep == '\\'

local function escape_path(path)
  if is_windows then
    return path
  end
  return path:gsub("'", "'\\''")
end

local function make_dir(path)
  if is_windows then
    os.execute(string.format('if not exist "%s" mkdir "%s"', path, path))
  else
    os.execute(string.format("mkdir -p '%s'", escape_path(path)))
  end
end

local function remove_path(path)
  if is_windows then
    os.execute(string.format('if exist "%s" rmdir /S /Q "%s"', path, path))
  else
    os.execute(string.format("rm -rf '%s'", escape_path(path)))
  end
end

local function install_pcd_stub(records)
  package.preload['common.tech.pcdLib'] = function()
    local lib = {}

    function lib.newPcd()
      local pcd = {
        fields = {},
        pointCount = 0,
      }

      function pcd:clearFields()
        self.fields = {}
      end

      function pcd:addField(name, ftype, size)
        self.fields[#self.fields + 1] = { name = name, type = ftype, size = size }
      end

      local function copy_fields(fields)
        local copy = {}
        for i, field in ipairs(fields) do
          copy[i] = {
            name = field.name,
            type = field.type,
            size = field.size,
          }
        end
        return copy
      end

      function pcd:setBinaryData(payload, count)
        self.data = payload
        self.pointCount = count or 0
      end

      function pcd:setPayload(payload, count)
        self:setBinaryData(payload, count)
      end

      function pcd:setData(payload)
        self.data = payload
      end

      function pcd:setPointCount(count)
        self.pointCount = count or 0
      end

      function pcd:setCount(count)
        self.pointCount = count or 0
      end

      function pcd:setPoints(count)
        self.pointCount = count or 0
      end

      function pcd:setPointNumber(count)
        self.pointCount = count or 0
      end

      function pcd:setViewpoint(origin, quat)
        self.viewpoint = {
          origin = origin,
          rotation = quat,
        }
      end

      local function ensure_viewpoint(self)
        if self.viewpoint then
          return self.viewpoint
        end
        return {
          origin = { x = 0, y = 0, z = 0 },
          rotation = { w = 1, x = 0, y = 0, z = 0 },
        }
      end

      function pcd:save(path)
        local fh, err = io.open(path, 'wb')
        if not fh then
          error(('unable to open %s: %s'):format(path, err or 'unknown'))
        end

        local header = { 'VERSION 0.7' }
        local names, sizes, types, counts = {}, {}, {}, {}
        for i, field in ipairs(self.fields) do
          names[i] = field.name or ('f' .. i)
          sizes[i] = tostring(field.size or 4)
          types[i] = field.type or 'F'
          counts[i] = '1'
        end

        header[#header + 1] = 'FIELDS ' .. table.concat(names, ' ')
        header[#header + 1] = 'SIZE ' .. table.concat(sizes, ' ')
        header[#header + 1] = 'TYPE ' .. table.concat(types, ' ')
        header[#header + 1] = 'COUNT ' .. table.concat(counts, ' ')
        header[#header + 1] = 'WIDTH ' .. tostring(self.pointCount)
        header[#header + 1] = 'HEIGHT 1'

        local viewpoint = ensure_viewpoint(self)
        local origin = viewpoint.origin or { x = 0, y = 0, z = 0 }
        local rotation = viewpoint.rotation or { w = 1, x = 0, y = 0, z = 0 }
        header[#header + 1] = string.format(
          'VIEWPOINT %.6f %.6f %.6f %.6f %.6f %.6f %.6f',
          origin.x or 0,
          origin.y or 0,
          origin.z or 0,
          rotation.w or rotation[1] or 1,
          rotation.x or rotation[2] or 0,
          rotation.y or rotation[3] or 0,
          rotation.z or rotation[4] or 0
        )
        header[#header + 1] = 'POINTS ' .. tostring(self.pointCount)
        header[#header + 1] = 'DATA binary'

        fh:write(table.concat(header, '\n'))
        fh:write('\n')
        if self.data then
          fh:write(self.data)
        end
        fh:close()

        records.saved[#records.saved + 1] = {
          path = path,
          pointCount = self.pointCount,
          fields = copy_fields(self.fields),
          viewpoint = self.viewpoint,
        }
      end

      return pcd
    end

    return lib
  end
end

local function read_pcd(path)
  local fh, err = io.open(path, 'rb')
  if not fh then
    error(('unable to read %s: %s'):format(path, err or 'unknown'))
  end

  local header = {}
  while true do
    local line = fh:read('*l')
    if not line then
      error('unexpected end of file while reading header')
    end
    header[#header + 1] = line
    if line == 'DATA binary' then
      break
    end
  end

  local payload = fh:read('*a') or ''
  fh:close()
  return header, payload
end

local tmp_root = 'tmp/lidar_pcd_publisher_spec'

describe('lidarPcdPublisher', function()
  hooks.beforeEach(function()
    remove_path(tmp_root)
    make_dir(tmp_root)
    package.loaded['scripts/driver_assistance_angelo234/lidarPcdPublisher'] = nil
    package.loaded['common.tech.pcdLib'] = nil
    package.preload['common.tech.pcdLib'] = nil
    _G.FS = nil
    _G.quatFromDir = nil
  end)

  hooks.afterEach(function()
    remove_path(tmp_root)
    package.loaded['scripts/driver_assistance_angelo234/lidarPcdPublisher'] = nil
    package.loaded['common.tech.pcdLib'] = nil
    package.preload['common.tech.pcdLib'] = nil
    _G.FS = nil
    _G.quatFromDir = nil
  end)

  it('generates a valid binary PCD file with expected point data', function()
    local records = { saved = {} }
    install_pcd_stub(records)

    local created_dirs = {}
    local rename_calls = {}

    local FS = {}

    function FS:getUserPath()
      return tmp_root .. '/'
    end

    function FS:directoryCreate(path)
      created_dirs[#created_dirs + 1] = path
      make_dir(path)
    end

    function FS:rename(from, to)
      rename_calls[#rename_calls + 1] = { from = from, to = to }
      os.remove(to)
      local ok, err = os.rename(from, to)
      if not ok then
        error(('rename failed from %s to %s: %s'):format(from, to, err or 'unknown'))
      end
    end

    _G.FS = FS

    _G.quatFromDir = function(dir, up)
      return {
        w = 1,
        x = dir.x or dir[1] or 0,
        y = up.y or up[2] or 0,
        z = up.z or up[3] or 0,
      }
    end

    local publisher = require('scripts/driver_assistance_angelo234/lidarPcdPublisher')

    local output_path = tmp_root .. '/captures/latest.pcd'
    publisher.configure({ path = output_path, enabled = true })
    publisher.setEnabled(true)

    local frame = {
      origin = { x = 10, y = 20, z = 30 },
      dir = { x = 0, y = 1, z = 0 },
      up = { x = 0, y = 0, z = 1 },
    }

    local scan = {
      main = {
        { x = 1.5, y = 2.5, z = 3.5 },
        { x = 2.5, y = 3.5, z = 4.5 },
      },
      ground = {
        { x = 5.5, y = 6.5, z = 7.5 },
      },
      vehicle = {
        { x = -1.5, y = -2.5, z = -3.5 },
      },
    }

    local ok = publisher.publish(frame, scan)
    expect(ok).toBeTruthy()

    expect(#created_dirs > 0).toBeTruthy()
    expect(created_dirs[#created_dirs]).toEqual(tmp_root .. '/captures')

    expect(#rename_calls).toEqual(1)
    expect(rename_calls[1].from).toEqual(output_path .. '.tmp')
    expect(rename_calls[1].to).toEqual(output_path)

    expect(#records.saved).toEqual(1)
    expect(records.saved[1].pointCount).toEqual(4)

    local fields = records.saved[1].fields
    expect(#fields).toEqual(4)
    expect(fields[1].name).toEqual('x')
    expect(fields[2].name).toEqual('y')
    expect(fields[3].name).toEqual('z')
    expect(fields[4].name).toEqual('intensity')

    local saved_viewpoint = records.saved[1].viewpoint
    expect(saved_viewpoint ~= nil).toBeTruthy()
    expect(saved_viewpoint.origin.x).toEqual(10)
    expect(saved_viewpoint.origin.y).toEqual(20)
    expect(saved_viewpoint.origin.z).toEqual(30)

    local header, payload = read_pcd(output_path)
    expect(header[1]).toEqual('VERSION 0.7')
    expect(header[2]).toEqual('FIELDS x y z intensity')
    expect(header[3]).toEqual('SIZE 4 4 4 4')
    expect(header[4]).toEqual('TYPE F F F F')
    expect(header[5]).toEqual('COUNT 1 1 1 1')
    expect(header[6]).toEqual('WIDTH 4')
    expect(header[7]).toEqual('HEIGHT 1')

    local viewpoint_line = header[8]
    expect(viewpoint_line:sub(1, 9)).toEqual('VIEWPOINT')

    local tokens = {}
    for token in viewpoint_line:gmatch('[^%s]+') do
      tokens[#tokens + 1] = token
    end

    expect(#tokens).toEqual(8)
    expect(math.abs(tonumber(tokens[2]) - 10) < 1e-6).toBeTruthy()
    expect(math.abs(tonumber(tokens[3]) - 20) < 1e-6).toBeTruthy()
    expect(math.abs(tonumber(tokens[4]) - 30) < 1e-6).toBeTruthy()

    expect(header[9]).toEqual('POINTS 4')
    expect(header[10]).toEqual('DATA binary')

    expect(#payload).toEqual(4 * 16)

    local offset = 1
    local points = {}
    for i = 1, 4 do
      local x, y, z, intensity
      x, y, z, intensity, offset = string_unpack('<ffff', payload, offset)
      points[i] = { x = x, y = y, z = z, intensity = intensity }
    end

    local function almost_equal(a, b)
      return math.abs(a - b) < 1e-5
    end

    expect(almost_equal(points[1].x, 1.5)).toBeTruthy()
    expect(almost_equal(points[1].intensity, 1.0)).toBeTruthy()

    expect(almost_equal(points[2].y, 3.5)).toBeTruthy()
    expect(almost_equal(points[2].intensity, 1.0)).toBeTruthy()

    expect(almost_equal(points[3].z, 7.5)).toBeTruthy()
    expect(almost_equal(points[3].intensity, 0.2)).toBeTruthy()

    expect(almost_equal(points[4].x, -1.5)).toBeTruthy()
    expect(almost_equal(points[4].intensity, 0.8)).toBeTruthy()
  end)
end)
