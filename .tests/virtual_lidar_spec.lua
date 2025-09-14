local vec_mt = {}
vec_mt.__index = vec_mt
function vec_mt.__add(a, b) return vec3(a.x + b.x, a.y + b.y, a.z + b.z) end
function vec_mt.__sub(a, b) return vec3(a.x - b.x, a.y - b.y, a.z - b.z) end
function vec_mt.__mul(a, b)
  if type(a) == 'number' then return vec3(a * b.x, a * b.y, a * b.z)
  elseif type(b) == 'number' then return vec3(a.x * b, a.y * b, a.z * b) end
end
function vec_mt:dot(b) return self.x * b.x + self.y * b.y + self.z * b.z end
function vec_mt:cross(b)
  return vec3(self.y * b.z - self.z * b.y, self.z * b.x - self.x * b.z, self.x * b.y - self.y * b.x)
end
function vec_mt:length() return math.sqrt(self:dot(self)) end
function vec_mt:normalized()
  local l = self:length()
  if l == 0 then return vec3(0, 0, 0) end
  return self * (1 / l)
end

function vec3(x, y, z)
  return setmetatable({x = x or 0, y = y or 0, z = z or 0}, vec_mt)
end

describe('virtual lidar scan', function()
  before_each(function()
    package.loaded['scripts/driver_assistance_angelo234/virtualLidar'] = nil
  end)

  it('filters hits from the host vehicle', function()
    _G.castRayStatic = function() return nil end
    _G.castRay = function() return {dist = 5, pt = vec3(0, 5, 0), obj = {getID = function() return 1 end}} end
    local lidar = require('scripts/driver_assistance_angelo234/virtualLidar')
    local pts = lidar.scan(vec3(), vec3(0, 1, 0), vec3(0, 0, 1), 10, 0, 0, 1, 1, 0, 1)
    assert.equal(0, #pts)
  end)

  it('includes hits from other vehicles', function()
    _G.castRayStatic = function() return nil end
    _G.castRay = function() return {dist = 5, pt = vec3(0, 5, 0), obj = {getID = function() return 2 end}} end
    local lidar = require('scripts/driver_assistance_angelo234/virtualLidar')
    local pts = lidar.scan(vec3(), vec3(0, 1, 0), vec3(0, 0, 1), 10, 0, 0, 1, 1, 0, 1)
    assert.equal(1, #pts)
  end)
end)
