-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

--[[
Usage:

a = vec3(1,2,3)
b = vec3({1,2,3})
c = vec3({x = 1, y = 2, z = 3})
print(a == b)
print( (a-b) == vec3(0, 0, 0) )
print( (c*1) )
print( vec3(10,0,0):dot(vec3(10,0,0)) )
]]

local min, max, sqrt, abs, random, floor = math.min, math.max, math.sqrt, math.abs, math.random, math.floor

local newLuaVec3xyz
local newLuaQuatxyzw

local LuaVec3 = {}
LuaVec3.__index = LuaVec3

local _, ffi = pcall(require, 'ffi')
if ffi then
  -- FFI available, so use it
  local ok
  ok, newLuaVec3xyz = pcall(ffi.typeof, "struct __luaVec3_t")
  if not ok then
    ffi.cdef [[
      struct __luaVec3_t {double x, y, z;};
      struct __luaQuat_t {double x, y, z, w;};
    ]]
    newLuaVec3xyz = ffi.typeof("struct __luaVec3_t")
  end
  ffi.metatype("struct __luaVec3_t", LuaVec3)
else
  -- no FFI available, compatibility mode
  ffi = nil
  newLuaVec3xyz = function (x, y, z)
    return (setmetatable({x = x, y = y, z = z}, LuaVec3)) -- parenthesis to workaround slowdown from: NYI return to lower frame
  end
end

--MARK: vec3
local tmpv1, tmpv2, tmpv3 = newLuaVec3xyz(0, 0, 0), newLuaVec3xyz(0, 0, 0), newLuaVec3xyz(0, 0, 0)

function vec3(x, y, z)
  if rawequal(y, nil) then
    if rawequal(x, nil) then
      return newLuaVec3xyz(0, 0, 0)
    else
      if rawequal(x.xyz, nil) then
        return newLuaVec3xyz(x.x or x[1], x.y or x[2], x.z or x[3])
      else
        return newLuaVec3xyz(x:xyz())
      end
    end
  else
    return newLuaVec3xyz(x, y, z or 0)
  end
end

function LuaVec3:set(x, y, z)
  if rawequal(y, nil) then
    self.x, self.y, self.z = x:xyz()
  else
    self.x, self.y, self.z = x, y, z
  end
end

function LuaVec3:xyz()
  return self.x, self.y, self.z
end

function LuaVec3:xy()
  return self.x, self.y
end

function LuaVec3:copy()
  return newLuaVec3xyz(self.x, self.y, self.z)
end

function LuaVec3:fromString(s)
  local x, y, z = s:match('([%d.+eE-]+)[,%s]+([%d.+eE-]+)[,%s]+([%d.+eE-]+)')
  self.x, self.y, self.z = tonumber(x) or 0, tonumber(y) or 0, tonumber(z) or 0
  return self
end

local function numSer(v)
  return v * 0 ~= 0 and (v > 0 and '9e999' or '-9e999') or string.format('%.10g', v)
end

function LuaVec3:__tostring()
  return string.format('vec3(%s,%s,%s)', numSer(self.x), numSer(self.y), numSer(self.z))
end

vec3toString = LuaVec3.__tostring -- back compat

function LuaVec3:toTable()
  return {self.x, self.y, self.z}
end

function LuaVec3:setFromTable(t)
  self.x, self.y, self.z = t[1], t[2], t[3]
end

function LuaVec3:toDict()
  return {x = self.x, y = self.y, z = self.z}
end

function LuaVec3:length()
  local ax, ay, az = self.x, self.y, self.z
  return sqrt(ax*ax + ay*ay + az*az)
end

function LuaVec3:lengthGuarded()
  local ax, ay, az = self.x, self.y, self.z
  return sqrt(ax*ax + ay*ay + az*az) + 1e-30
end

function LuaVec3:squaredLength()
  local ax, ay, az = self.x, self.y, self.z
  return ax*ax + ay*ay + az*az
end

function LuaVec3.__add(a, b)
  return newLuaVec3xyz(a.x + b.x, a.y + b.y, a.z + b.z)
end

function LuaVec3.__sub(a, b)
  return newLuaVec3xyz(a.x - b.x, a.y - b.y, a.z - b.z)
end

function LuaVec3.__unm(a)
  return newLuaVec3xyz(-a.x, -a.y, -a.z)
end

function LuaVec3.__mul(a, b)
  if type(b) == 'number' then
    return newLuaVec3xyz(b * a.x, b * a.y, b * a.z)
  else
    return newLuaVec3xyz(a * b.x, a * b.y, a * b.z)
  end
end

function LuaVec3.__div(a,b)
    local c = 1 / b
    return newLuaVec3xyz(c * a.x, c * a.y, c * a.z)
end

function LuaVec3.__eq(a, b)
  return (not rawequal(b, nil)) and a.x == b.x and a.y == b.y and a.z == b.z
end

function LuaVec3:dot(a)
  return self.x * a.x + self.y * a.y + self.z * a.z
end

function LuaVec3:cross(a)
  return newLuaVec3xyz(self.y * a.z - self.z * a.y, self.z * a.x - self.x * a.z, self.x * a.y - self.y * a.x)
end

function LuaVec3:z0()
  return newLuaVec3xyz(self.x, self.y, 0)
end

function LuaVec3:perpendicular()
  local k = abs(self.x) + 0.5
  k = k - floor(k)
  return newLuaVec3xyz(-self.y, self.x - k * self.z, k * self.y)
end

function LuaVec3:perpendicularN()
  local p = self:perpendicular()
  local r = 1 / (p:length() + 1e-30)
  p.x, p.y, p.z = p.x * r, p.y * r, p.z * r
  return p
end

-- inputs should be normalized, output is unormalized
function LuaVec3:slerp(b, t)
  local dot = clamp(self:dot(b), -1, 1)
  if dot > 0.9995 then return self + t*(b-self) end
  local th = math.acos(dot)
  return math.sin(th * (1-t)) * self + math.sin(th * t) * b
end

function LuaVec3:cosAngle(a)
  return max(min(self:dot(a) / (sqrt(self:squaredLength() * a:squaredLength()) + 1e-30), 1), -1)
end

function LuaVec3:normalize()
  local r = 1 / (self:length() + 1e-30)
  self.x, self.y, self.z = self.x * r, self.y * r, self.z * r
end

function LuaVec3:resize(a)
  local r = a / (self:length() + 1e-30)
  self.x, self.y, self.z = self.x * r, self.y * r, self.z * r
end

-- p = p + vel * dt; if p:ropeRock(x) < x then stationary end
function LuaVec3:ropeRock(cutOff)
  local selfLen = self:length()
  local r = min(selfLen, cutOff) / (selfLen + 1e-30)
  self.x, self.y, self.z = self.x * r, self.y * r, self.z * r
  return selfLen
end

function LuaVec3:normalized()
  local r = 1 / (self:length() + 1e-30)
  return newLuaVec3xyz(self.x * r, self.y * r, self.z * r)
end

function LuaVec3:resized(m)
  local r = m / (self:length() + 1e-30)
  return newLuaVec3xyz(self.x * r, self.y * r, self.z * r)
end

function LuaVec3:distance(a)
  local tmp = self.x - a.x
  local d = tmp * tmp
  tmp = self.y - a.y
  d = d + tmp * tmp
  tmp = self.z - a.z
  return sqrt(d + tmp * tmp)
end

function LuaVec3:squaredDistance(a)
  local tmp = self.x - a.x
  local d = tmp * tmp
  tmp = self.y - a.y
  d = d + tmp * tmp
  tmp = self.z - a.z
  return d + tmp * tmp
end

-- a, b are line points, self is the point
function LuaVec3:squaredDistanceToLine(a, b)
  local abx, aby, abz, asx, asy, asz = a.x-b.x, a.y-b.y, a.z-b.z, a.x-self.x, a.y-self.y, a.z-self.z
  local xnorm = (abx*asx + aby*asy + abz*asz) / (abx*abx + aby*aby + abz*abz + 1e-30) -- ab:dot(a-self)/ab:dot(ab)
  return square(asx - abx*xnorm) + square(asy - aby*xnorm) + square(asz - abz*xnorm) -- (a-self):squaredDistance(ab * xnorm)
end

function LuaVec3:distanceToLine(a, b)
  return sqrt(self:squaredDistanceToLine(a, b))
end

function LuaVec3:squaredDistanceToLineSegment(a, b)
  local abx, aby, abz, asx, asy, asz = a.x-b.x, a.y-b.y, a.z-b.z, a.x-self.x, a.y-self.y, a.z-self.z
  local xnormC = min(max((abx*asx + aby*asy + abz*asz) / (abx*abx + aby*aby + abz*abz + 1e-30), 0), 1) -- min(max(ab:dot(a-self)/ab:dot(ab), 0), 1)
  return square(asx - abx*xnormC) + square(asy - aby*xnormC) + square(asz - abz*xnormC) -- (a-self):squaredDistance(ab * xnormC)
end

function LuaVec3:distanceToLineSegment(a, b)
  return sqrt(self:squaredDistanceToLineSegment(a, b))
end

function LuaVec3:xnormSquaredDistanceToLineSegment(a, b)
  local abx, aby, abz, asx, asy, asz = a.x-b.x, a.y-b.y, a.z-b.z, a.x-self.x, a.y-self.y, a.z-self.z
  local xnorm = (abx*asx + aby*asy + abz*asz) / (abx*abx + aby*aby + abz*abz + 1e-30)
  local xnormC = min(max(xnorm, 0), 1)
  return xnorm, square(asx - abx*xnormC) + square(asy - aby*xnormC) + square(asz - abz*xnormC)
end

function LuaVec3:xnormDistanceToLineSegment(a, b)
  local xnorm, sqdist = self:xnormSquaredDistanceToLineSegment(a, b)
  return xnorm, sqrt(sqdist)
end

function LuaVec3:xnormOnLine(a, b)
  local bax, bay, baz = b.x-a.x, b.y-a.y, b.z-a.z
  return (bax*(self.x-a.x) + bay*(self.y-a.y) + baz*(self.z-a.z)) / (bax*bax + bay*bay + baz*baz + 1e-30) -- (b-a):dot(self-a) / (b-a):squaredLength()
end

-- returns u, v, normal : u*a + v*b + (1-u-v)*c
function LuaVec3:triangleBarycentricNorm(a, b, c)
  local ca, bc = c - a, b - c
  local norm = ca:cross(bc)
  local normsqlen = norm:squaredLength() + 1e-30
  local pacnorm = (self - c):cross(norm)
  return bc:dot(pacnorm) / normsqlen, ca:dot(pacnorm) / normsqlen, norm / sqrt(normsqlen)
end

function LuaVec3:setTrianglePointFromUV(a, b, c, u, v)
  self:set(push3(a) * u + push3(b) * v + push3(c) * (1-u-v))
end

-- returns u, v satisfying p(u,v) = lerp( lerp(a1, a2, u), lerp(b1, b2, u), v ) in 2D
-- https://www.reedbeta.com/blog/quadrilateral-interpolation-part-2
function LuaVec3:invBilinear2D(a1, a2, b1, b2)
  local ex, ey = a2.x - a1.x, a2.y - a1.y
  local fx, fy = b1.x - a1.x, b1.y - a1.y
  local gx, gy = b2.x - b1.x - ex, b2.y - b1.y - ey
  local hx, hy = self.x - a1.x, self.y - a1.y

  local A = gx*fy - gy*fx
  local B = ex*fy - ey*fx + hx*gy - hy*gx
  local C = hx*ey - hy*ex

  local v = abs(A) > 0.001 and 0.5 * (sqrt(max(B*B - 4*A*C, 0)) - B) / A or -C/B
  local Dx, Dy = ex + v * gx, ey + v * gy
  return abs(Dx) > abs(Dy) and (hx - fx * v) / Dx or (hy - fy * v) / Dy, v
end

-- returns u, v : u*a + v*b + (1-u-v)*c
function LuaVec3:triangleClosestPointUV(a, b, c)
  tmpv1:setSub2(b, a)
  tmpv2:setSub2(c, a)

  tmpv3:setSub2(self, a)
  local d1, d2 = tmpv1:dot(tmpv3), tmpv2:dot(tmpv3)
  if max(d1, d2) <= 0 then return 1, 0 end

  tmpv3:setSub2(self, b)
  local d3, d4 = tmpv1:dot(tmpv3), tmpv2:dot(tmpv3)
  if max(d4, 0) <= d3 then return 0, 1 end

  tmpv3:setSub2(self, c)
  local d5, d6 = tmpv1:dot(tmpv3), tmpv2:dot(tmpv3)
  if max(d5, 0) <= d6 then return 0, 0 end

  local vc1, vc2 = d1 * d4, d3 * d2 -- (a.c)(b.d)– (a.d)(b.c) = (axb).(cxd)
  if vc1 <= vc2 and d1 >= 0 and d3 <= 0 then
    local t = min(max(d1 / (d1 - d3), 0), 1)
    return 1 - t, t
  end

  local vb1, vb2 = d5 * d2, d1 * d6
  if vb1 <= vb2 and d6 <= 0 then
    return min(max(d6 / (d6 - d2), 0), 1), 0
  end

  local va1, va2 = d3 * d6, d5 * d4
  if va1 <= va2 then
    d5 = d5 - d6
    return 0, min(max(d5 / (d5 + d4 - d3), 0), 1)
  end

  local va, vb = va1 - va2, vb1 - vb2
  local d = va + vb + vc1 - vc2
  return va / d, vb / d
end

-- returns point, u, v : u*a + v*b + (1-u-v)*c
function LuaVec3:triangleClosestPoint(a, b, c)
  local u, v = self:triangleClosestPointUV(a, b, c)
  local t = newLuaVec3xyz(self.x, self.y, self.z)
  t:setTrianglePointFromUV(a, b, c, u, v)
  return t, u, v
end

-- https://love2d.org/wiki/PointWithinShape
-- SPDX-License-Identifier: Zlib OR BSD-2-Clause
function LuaVec3:inPolygon(...)
  local x, y, inside = self.x, self.y, false
  local points = select("#", ...) == 1 and select(1, ...) or {...}
  local pcount = #points
  local pl1, pl2 = points[pcount], points[1]
  local yflag0 = pl1.y >= y

  for i = 2, pcount + 1 do
    local yflag1 = pl2.y >= y
    if yflag1 ~= yflag0 and yflag1 == ((pl2.y - y)*(pl1.x - pl2.x) >= (pl2.x - x)*(pl1.y - pl2.y)) then
      inside = not inside
    end
    yflag0, pl1, pl2 = yflag1, pl2, points[i]
  end
  return inside
end

function LuaVec3:projectToOriginPlane(pnorm)
  local t = self.x*pnorm.x + self.y*pnorm.y + self.z*pnorm.z
  return newLuaVec3xyz(self.x - t*pnorm.x, self.y - t*pnorm.y, self.z - t*pnorm.z)
end

-- self is a plane' point
function LuaVec3:xnormPlaneWithLine(pnorm, a, b)
  return (pnorm.x*(self.x-a.x) + pnorm.y*(self.y-a.y) + pnorm.z*(self.z-a.z)) *
          max(min(1 / (pnorm.x*(b.x-a.x) + pnorm.y*(b.y-a.y) + pnorm.z*(b.z-a.z)), 1e300), -1e300) -- pnorm:dot(self-a)/pnorm:dot(b-a)
end

-- self is center of sphere, returns (low, high) xnorms. Returns pair 1,0 if no hit found
function LuaVec3:xnormsSphereWithLine(radius, a, b)
  local lDif, ac = b - a, a - self
  local invDif2len = 1 / max(lDif:squaredLength(), 1e-30)
  local dotab = -ac:dot(lDif) * invDif2len
  local D = dotab * dotab + (radius * radius - ac:squaredLength()) * invDif2len
  if D >= 0 then
    D = sqrt(D)
    return dotab - D, dotab + D
  else
    return 1, 0
  end
end

function LuaVec3:basisCoordinates(c1, c2, c3)
  local c2xc3 = c2:cross(c3)
  local invDet = 1 / c1:dot(c2xc3)
  return newLuaVec3xyz(c2xc3:dot(self)*invDet, c3:cross(c1):dot(self)*invDet, c1:cross(c2):dot(self)*invDet)
end

function LuaVec3:setToBase(nx, ny, nz)
  local x, y, z = self.x, self.y, self.z
  self:setScaled2(nx, x)
  tmpv1:setScaled2(ny, y)
  self:setAdd(tmpv1)
  tmpv1:setScaled2(nz, z)
  self:setAdd(tmpv1)
end

function LuaVec3:toBase(nx, ny, nz)
  local t = newLuaVec3xyz(self.x, self.y, self.z)
  t:setToBase(nx, ny, nz)
  return t
end

function LuaVec3:componentMul(b)
  return newLuaVec3xyz(self.x * b.x, self.y * b.y, self.z * b.z)
end

--MARK: vec3 set

function LuaVec3:setMin(a)
  local x, y, z = a:xyz()
  self.x, self.y, self.z = min(self.x, x), min(self.y, y), min(self.z, z)
end

function LuaVec3:setMax(a)
  local x, y, z = a:xyz()
  self.x, self.y, self.z = max(self.x, x), max(self.y, y), max(self.z, z)
end

function LuaVec3:setAddXYZ(x, y, z)
  self.x, self.y, self.z = self.x + x, self.y + y, self.z + z
end

function LuaVec3:setAdd(a)
  local x, y, z = a:xyz()
  self.x, self.y, self.z = self.x + x, self.y + y, self.z + z
end

function LuaVec3:setAdd2(a, b)
  local x, y, z = a:xyz()
  local bx, by, bz = b:xyz()
  self.x, self.y, self.z = x + bx, y + by, z + bz
end

function LuaVec3:setSub(a)
  local x, y, z = a:xyz()
  self.x, self.y, self.z = self.x - x, self.y - y, self.z - z
end

function LuaVec3:setSub2(a, b)
  local x, y, z = a:xyz()
  local bx, by, bz = b:xyz()
  self.x, self.y, self.z = x - bx, y - by, z - bz
end

function LuaVec3:setScaled(b)
  self.x, self.y, self.z = self.x * b, self.y * b, self.z * b
end

function LuaVec3:setScaled2(a, b)
  local x, y, z = a:xyz()
  self.x, self.y, self.z = x * b, y * b, z * b
end

function LuaVec3:setComponentMul(a)
  local x, y, z = a:xyz()
  self.x, self.y, self.z = self.x * x, self.y * y, self.z * z
end

function LuaVec3:setLerp(from, to, t)
  local x, y, z = from:xyz()
  local bx, by, bz = to:xyz()
  local t1 = 1 - t
  self.x, self.y, self.z = x*t1 + bx*t, y*t1 + by*t, z*t1 + bz*t  -- preserves from and to, non monotonic
end

function LuaVec3:setCross(a, b)
  local x, y, z = a:xyz()
  local bx, by, bz = b:xyz()
  self.x, self.y, self.z = y*bz - z*by, z*bx - x*bz, x*by - y*bx
end

function LuaVec3:setRotate(q, a)
  a = a or self
  local x, y, z = a:xyz()
  local tx,ty,tz = (push3(q):cross(push3(x,y,z)) * 2):xyz()
  self:set(push3(x,y,z) - push3(tx,ty,tz) * q.w + push3(q):cross(push3(tx,ty,tz)))
end

--http://bediyap.com/programming/convert-quaternion-to-euler-rotations/
function LuaVec3:setEulerYXZ(q)
  local wxsq = q.w*q.w - q.x*q.x
  local yzsq = q.z*q.z - q.y*q.y
  self.x, self.y, self.z = math.atan2(2*(q.x*q.y+q.w*q.z), wxsq-yzsq), math.asin(max(min(-2*(q.y*q.z-q.w*q.x), 1), -1)), math.atan2(2*(q.x*q.z+q.w*q.y), wxsq+yzsq)
end

function LuaVec3:setProjectToOriginPlane(pnorm, a)
  a = a or self
  local x,y,z = a:xyz()
  local px,py,pz = pnorm:xyz()
  local t = x*px + y*py + z*pz
  self.x, self.y, self.z = x - t*px, y - t*py, z - t*pz
end

-- Returns quat (-w) from self->v. Both vecs should be normalized
function LuaVec3:getRotationTo(v)
  local w = 1 + self:dot(v)
  local qv

  if (w < 1e-6) then
    w, qv = 0, v:perpendicular()
  else
    qv = self:cross(v)
  end
  local q = newLuaQuatxyzw(qv.x, qv.y, qv.z, -w)
  q:normalize()
  return q
end

-- Rotates by quaternion q (-w)
function LuaVec3:rotated(q)
  local qv = newLuaVec3xyz((push3(q):cross(self) * 2):xyz())
  qv:set(push3(self) - push3(qv) * q.w + push3(q):cross(qv))
  return qv
end

local function fractPos(x)
  return x - floor(x)
end

-- x is previous value, returns [0..1)
function getBlueNoise1d(x)
  return fractPos(x + 0.6180339887498949)
end

-- vec3 is seed/state/value, returns [0..1), [0..1)
function LuaVec3:getBlueNoise2d()
  self.x, self.y = fractPos(self.x + 0.75487766624669276), fractPos(self.y + 0.56984029099805327)
  return self
end

-- vec3 is seed/state/value, returns [0..1), [0..1), [0..1)
function LuaVec3:getBlueNoise3d()
  self.x, self.y, self.z = fractPos(self.x + 0.81917251339616443), fractPos(self.y + 0.6710436067037892084), fractPos(self.z + 0.54970047790197026)
  return self
end

function LuaVec3:getRandomPointInSphere(radius)
  radius = radius or 1
  local sx, sy, sz = 0, 0, 0;
  for i = 1, 4 do
    local x, y, z = random(), random(), random()
    sx, sy, sz = sx + x, sy + y, sz + z
    local xc, yc, zc = x - 0.5, y - 0.5, z - 0.5
    if xc * xc + yc * yc + zc * zc <= 0.25 then
      local r2 = radius * 2
      self.x, self.y, self.z = xc * r2, yc * r2, zc * r2
      return self
    end
  end

  sx, sy, sz = sx - 2, sy - 2, sz - 2
  local u = random()
  local norm = sqrt(0.5 * (sqrt(u) + u)/(sx*sx + sy*sy + sz*sz + 1e-25)) * radius
  self.x, self.y, self.z = sx*norm, sy*norm, sz*norm
  return self
end

function LuaVec3:getRandomPointInCircle(radius)
  radius = radius or 1
  local sx, sy = 0, 0
  for i = 1, 4 do
    local x, y = random(), random()
    sx, sy = sx + x, sy + y
    local xc, yc = x - 0.5, y - 0.5
    if xc * xc + yc * yc <= 0.25 then
      local r2 = radius * 2
      self.x, self.y, self.z = xc * r2, yc * r2, 0
      return self
    end
  end

  sx, sy = sx - 2, sy - 2
  local norm = sqrt(random()/(sx*sx+sy*sy + 1e-25)) * radius
  self.x, self.y, self.z = sx*norm, sy*norm, 0
  return self
end

function LuaVec3:getBluePointInSphere(radius)
  radius = radius or 1
  local bx, by, bz = self.x, self.y, self.z
  for i = 1, 8 do -- seen up to 6
    local x, y, z = fractPos(bx + 0.81917251339616443 * i), fractPos(by + 0.6710436067037892084 * i), fractPos(bz + 0.54970047790197026 * i)
    local xc, yc, zc = x - 0.5, y - 0.5, z - 0.5
    if xc * xc + yc * yc + zc * zc <= 0.25 then
      self.x, self.y, self.z = x, y, z
      local r2 = radius * 2
      return newLuaVec3xyz(xc * r2, yc * r2, zc * r2)
    end
  end
  return newLuaVec3xyz(0, 0, 0)
end

function LuaVec3:getBluePointInCircle(radius)
  radius = radius or 1
  local bx, by = self.x, self.y
  for i = 1, 5 do -- seen up to 3
    local x, y = fractPos(bx + 0.75487766624669276 * i), fractPos(by + 0.56984029099805327 * i)
    local xc, yc = x - 0.5, y - 0.5
    if xc * xc + yc * yc <= 0.25 then
      self.x, self.y = x, y
      local r2 = radius * 2
      return newLuaVec3xyz(xc * r2, yc * r2, 0)
    end
  end
  return newLuaVec3xyz(0, 0, 0)
end

-- backward compatibility
function LuaVec3:getAxis(i)
  return self[string.char(120 + i)]
end

function LuaVec3:setAxis(i, v)
  self[string.char(120 + i)] = v
end

function LuaVec3:toFloat3()
  return self
end

LuaVec3.toPoint3F = LuaVec3.toFloat3
LuaVec3.len = LuaVec3.length
LuaVec3.lenSquared = LuaVec3.squaredLength
LuaVec3.normalizeSafe = LuaVec3.normalize

-- returns random gauss number in [0..3]
function randomGauss3()
  return random() + random() + random()
end

-- returns random number in [0..1], v in [0..1]
function randomState(v)
  return max(floor(v * 5779561388494110) % 4294967291, 1) / 4294967290
end

-- returns xnormals for the two lines: http://geomalgorithms.com/a07-_distance.html
function closestLinePoints(l1p1, l1p2, l2p1, l2p2)
  local ux, uy, uz, vx, vy, vz = l1p2.x - l1p1.x, l1p2.y - l1p1.y, l1p2.z - l1p1.z, l2p2.x - l2p1.x, l2p2.y - l2p1.y, l2p2.z - l2p1.z
  local uu, vv, uv = ux*ux + uy*uy + uz*uz, vx*vx + vy*vy + vz*vz, ux*vx + uy*vy + uz*vz
  local D = uu*vv - uv*uv
  local rx, ry, rz = l1p1.x - l2p1.x, l1p1.y - l2p1.y, l1p1.z - l2p1.z
  local ru, rv = rx*ux + ry*uy + rz*uz, rx*vx + ry*vy + rz*vz

  if D < 1e-20 then
    -- handles the following cases vv == 0, uu == 0, u // v
    if vv == 0 then
      return -ru / (uu + 1e-30), 0
    else
      return 0, rv / (vv + 1e-30)
    end
  else
    return (uv*rv - vv*ru) / D, (uu*rv - uv*ru) / D
  end
end

-- returns normalized line-local coordinates (xnorms) at which the distance between the two line segments is minimum
function closestLineSegmentPoints(l1p1, l1p2, l2p1, l2p2)
  local ux, uy, uz, vx, vy, vz = l1p2.x - l1p1.x, l1p2.y - l1p1.y, l1p2.z - l1p1.z, l2p2.x - l2p1.x, l2p2.y - l2p1.y, l2p2.z - l2p1.z
  local uu, vv, uv = ux*ux + uy*uy + uz*uz, vx*vx + vy*vy + vz*vz, ux*vx + uy*vy + uz*vz
  local D = uu*vv - uv*uv
  local rx, ry, rz = l1p1.x - l2p1.x, l1p1.y - l2p1.y, l1p1.z - l2p1.z
  local ru, rv = rx*ux + ry*uy + rz*uz, rx*vx + ry*vy + rz*vz

  if D < 1e-20 then
    local t = clamp(rv / (vv + 1e-30), 0, 1)
    return clamp((t*uv - ru) / (uu + 1e-30), 0, 1), t
  else
    return clamp((clamp((uu*rv - uv*ru) / D, 0, 1)*uv - ru) / uu, 0, 1), clamp((clamp((uv*rv - vv*ru) / D, 0, 1)*uv + rv) / vv, 0, 1)
  end
end

function linePointFromXnorm(p0, p1, xnorm)
  return newLuaVec3xyz(p0.x + (p1.x-p0.x) * xnorm, p0.y + (p1.y-p0.y) * xnorm, p0.z + (p1.z-p0.z) * xnorm)
end

--MARK: push3

local StackVec3 = {}
StackVec3.__index = StackVec3
local push3obj, stacki, stackv3 = setmetatable(StackVec3, StackVec3), 1, {}
for i = 100, 1, -1 do stackv3[i] = newLuaVec3xyz(0,0,0) end

function push3(x, y, z)
  local s = stackv3[stacki]
  if rawequal(y, nil) then
    s.x, s.y, s.z = x:xyz()
  else
    s.x, s.y, s.z = x, y, z
  end
  stacki = stacki + 1
  return push3obj
end

function StackVec3:xyz()
  stacki = stacki - 1
  local s = stackv3[stacki]
  return s.x, s.y, s.z
end

function StackVec3:__tostring()
  stacki = stacki - 1
  local s = stackv3[stacki]
  return string.format('push3(%.10g,%.10g,%.10g)', numSer(s.x), numSer(s.y), numSer(s.z))
end

function StackVec3.__add(a, b)
  local bx, by, bz = b:xyz()
  local s = stackv3[stacki-1]
  s.x, s.y, s.z = s.x + bx, s.y + by, s.z + bz
  return push3obj
end

function StackVec3.__sub(a, b)
  local bx, by, bz = b:xyz()
  local s = stackv3[stacki-1]
  s.x, s.y, s.z = s.x - bx, s.y - by, s.z - bz
  return push3obj
end

function StackVec3.__unm(a)
  local s = stackv3[stacki-1]
  s.x, s.y, s.z = -s.x, -s.y, -s.z
  return push3obj
end

function StackVec3.__mul(a, b)
  b = type(a) == 'number' and a or b
  local s = stackv3[stacki-1]
  s.x, s.y, s.z = s.x*b, s.y*b, s.z*b
  return push3obj
end

function StackVec3.__div(a, b)
  local c = 1 / b
  local s = stackv3[stacki-1]
  s.x, s.y, s.z = s.x*c, s.y*c, s.z*c
  return push3obj
end

function StackVec3:dot(a)
  local ax, ay, az = a:xyz()
  stacki = stacki - 1
  local s = stackv3[stacki]
  return s.x * ax + s.y * ay + s.z * az
end

function StackVec3:cross(b)
  local bx, by, bz = b:xyz()
  local s = stackv3[stacki-1]
  local ax, ay, az = s.x, s.y, s.z
  s.x, s.y, s.z = ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx
  return push3obj
end

function StackVec3:z0()
  stackv3[stacki-1].z = 0
  return push3obj
end

function StackVec3:length()
  stacki = stacki - 1
  local s = stackv3[stacki]
  local ax, ay, az = s.x, s.y, s.z
  return sqrt(ax*ax + ay*ay + az*az)
end

function StackVec3:squaredLength()
  stacki = stacki - 1
  local s = stackv3[stacki]
  local ax, ay, az = s.x, s.y, s.z
  return ax*ax + ay*ay + az*az
end

function StackVec3:normalized()
  local s = stackv3[stacki-1]
  local ax, ay, az = s.x, s.y, s.z
  local r = 1 / (sqrt(ax*ax + ay*ay + az*az) + 1e-30)
  s.x, s.y, s.z = ax * r, ay * r, az * r
  return push3obj
end

function StackVec3:resized(m)
  local s = stackv3[stacki-1]
  local ax, ay, az = s.x, s.y, s.z
  local r = m / (sqrt(ax*ax + ay*ay + az*az) + 1e-30)
  s.x, s.y, s.z = ax * r, ay * r, az * r
  return push3obj
end

function StackVec3:distance(a)
  local ax, ay, az = a:xyz()
  stacki = stacki - 1
  local s = stackv3[stacki]
  ax, ay, ax = ax - s.x, ay - s.y, az - s.z
  return sqrt(ax*ax + ay*ay + az*az)
end

function StackVec3:squaredDistance(a)
  local ax, ay, az = a:xyz()
  stacki = stacki - 1
  local s = stackv3[stacki]
  ax, ay, ax = ax - s.x, ay - s.y, az - s.z
  return ax*ax + ay*ay + az*az
end

--MARK: quat

local LuaQuat = {}
LuaQuat.__index = LuaQuat

if ffi then
  newLuaQuatxyzw = ffi.typeof("struct __luaQuat_t")
  ffi.metatype("struct __luaQuat_t", LuaQuat)
else
  newLuaQuatxyzw = function (_x, _y, _z, _w)
    return (setmetatable({ x = _x, y = _y, z = _z, w = _w }, LuaQuat)) -- parenthesis needed to workaround extreme slowdown from: NYI return to lower frame
  end
end

-- T3d's quats use -w
function quat(x, y, z, w)
  if rawequal(y, nil) then
    if type(x) == 'table' and x[4] ~= nil then
      return newLuaQuatxyzw(x[1], x[2], x[3], x[4])
    elseif rawequal(x, nil) then
      return newLuaQuatxyzw(1, 0, 0, 0)
    else
      return newLuaQuatxyzw(x.x, x.y, x.z, x.w)
    end
  else
    return newLuaQuatxyzw(x, y, z, w)
  end
end

function LuaQuat:xyz()
  return self.x, self.y, self.z
end

function LuaQuat:xyzw()
  return self.x, self.y, self.z, self.w
end

function LuaQuat:__tostring()
  return string.format('quat(%.10g,%.10g,%.10g,%.10g)', numSer(self.x), numSer(self.y), numSer(self.z), numSer(self.w))
end

function LuaQuat:toTable()
  return {self.x, self.y, self.z, self.w}
end

function LuaQuat:toDict()
  return {x = self.x, y = self.y, z = self.z, w = self.w}
end

function LuaQuat:copy()
  return newLuaQuatxyzw(self.x, self.y, self.z, self.w)
end

function LuaQuat:set(x, y, z, w)
  if rawequal(y, nil) then
    self.x, self.y, self.z, self.w = x.x, x.y, x.z, x.w
  else
    self.x, self.y, self.z, self.w = x, y, z, w
  end
end

function LuaQuat:norm()
  return sqrt(self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w)
end

function LuaQuat:squaredNorm()
  return self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w
end

function LuaQuat:normalize()
  local r = 1/(self:norm() + 1e-30)
  self.x, self.y, self.z, self.w = self.x * r, self.y * r, self.z * r, self.w * r
end

function LuaQuat:normalized()
  local r = 1/(self:norm() + 1e-30)
  return newLuaQuatxyzw(self.x * r, self.y * r, self.z * r, self.w * r)
end

function LuaQuat:inverse()
  local invSqNorm = -1 / (self:squaredNorm() + 1e-30)
  self.x, self.y, self.z, self.w = self.x * invSqNorm, self.y * invSqNorm, self.z * invSqNorm, -self.w * invSqNorm
end

function LuaQuat:inversed()
  local invSqNorm = -1 / (self:squaredNorm() + 1e-30)
  return newLuaQuatxyzw(self.x * invSqNorm, self.y * invSqNorm, self.z * invSqNorm, -self.w * invSqNorm)
end

function LuaQuat.__unm(a)
  return newLuaQuatxyzw(-a.x, -a.y, -a.z, -a.w)
end

function LuaQuat.__mul(a, b)
  if type(a) == 'number' then
    return newLuaQuatxyzw(b.x*a, b.y*a, b.z*a, b.w*a)
  elseif type(b) == 'number' then
    return newLuaQuatxyzw(a.x*b, a.y*b, a.z*b, a.w*b)
  elseif (ffi and ffi.istype('struct __luaVec3_t', b)) or b.w == nil then
    local qv = newLuaVec3xyz((push3(a):cross(b) * 2):xyz())
    qv:set(push3(b) - push3(qv)*a.w + push3(a):cross(qv))
    return qv
  else
    return newLuaQuatxyzw(a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
                a.w*b.y + a.y*b.w + a.z*b.x - a.x*b.z,
                a.w*b.z + a.z*b.w + a.x*b.y - a.y*b.x,
                a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z)
  end
end

function LuaQuat.__sub(a, b)
  return newLuaQuatxyzw(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w)
end

function LuaQuat.__div(a, b)
  if type(b) == 'number' then
    return newLuaQuatxyzw(a.x / b, a.y / b, a.z / b, a.w / b)
  else
    local q = newLuaQuatxyzw(a.x, a.y, a.z, a.w)
    q:setMulInv2(q, b)
    return q
  end
end

function LuaQuat.__add(a, b)
  return newLuaQuatxyzw(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w)
end

function LuaQuat:dot(a)
  return self.x * a.x + self.y * a.y + self.z * a.z + self.w * a.w
end

function LuaQuat:distance(a)
  return 0.5 * (self - a):squaredNorm()
end

function LuaQuat:nlerp(a, t)
  local tmp = (1 - t) * self + (self:dot(a) < 0 and -t or t) * a
  tmp:normalize()
  return tmp
end

function LuaQuat:slerp(a, t)
  local dot = clamp(self:dot(a), -1, 1)
  if dot > 0.9995 then return self:nlerp(a, t) end
  local theta = math.acos(dot)*t
  return (self*math.cos(theta) + (a - self*dot):normalized()*math.sin(theta)):normalized()
end

-- returns reverse rotation
function LuaQuat:conjugated()
  return newLuaQuatxyzw(-self.x, -self.y, -self.z, self.w)
end

function LuaQuat:scale(a)
  self.x, self.y, self.z, self.w = self.x * a, self.y * a, self.z * a, self.w * a
  return self
end

function LuaQuat:setFromEuler(x, y, z) -- in radians
  x, y, z = x * 0.5, y * 0.5, z * 0.5
  local sx, cx, sy, cy, sz, cz  = math.sin(x), math.cos(x), math.sin(y), math.cos(y), math.sin(z), math.cos(z)
  local cycz, sysz, sycz, cysz = cy*cz, sy*sz, sy*cz, cy*sz
  self.x, self.y, self.z, self.w = cycz*sx + sysz*cx, sycz*cx + cysz*sx, cysz*cx - sycz*sx, cycz*cx - sysz*sx
end

local q = {}
local matTable = {[0]={}, {}, {}}
function LuaQuat:setFromDir(dir, up)
  if not up then tmpv1:set(0, 0, 1) else tmpv1:set(up) end
  tmpv2:set(dir) -- tmpv2 is dirNorm
  tmpv2:normalize()
  if abs(tmpv2:dot(tmpv1)) > 0.9999 then
    tmpv2:set((push3(tmpv2) + tmpv2:perpendicularN() * 1e-5):normalized())
  end

  matTable[1][0], matTable[1][1], matTable[1][2] = tmpv2.x, tmpv2.y, tmpv2.z
  tmpv1:setCross(tmpv2, tmpv1)
  tmpv1:normalize()
  matTable[0][0], matTable[0][1], matTable[0][2] = tmpv1.x, tmpv1.y, tmpv1.z
  tmpv1:setCross(tmpv1, tmpv2)
  tmpv1:normalize()
  matTable[2][0], matTable[2][1], matTable[2][2] = tmpv1.x, tmpv1.y, tmpv1.z

  -- quatFromAxesMatrix
  local m = matTable
  local trace = m[0][0] + m[1][1] + m[2][2]
  if trace > 0 then
    local s = sqrt(trace + 1)
    self.w = s * 0.5
    s = 0.5 / s
    self.x = (m[2][1] - m[1][2]) * s
    self.y = (m[0][2] - m[2][0]) * s
    self.z = (m[1][0] - m[0][1]) * s
  else
    local i = 0
    if m[1][1] > m[0][0] then i = 1 end
    if m[2][2] > m[i][i] then i = 2 end
    local j = (i + 1) % 3
    local k = (j + 1) % 3

    q[0], q[1], q[2] = 0, 0, 0
    local s = sqrt((m[i][i] - m[j][j]) - (m[k][k] - 1))
    q[i] = s * 0.5
    s = 0.5 / s
    q[j] = (m[j][i] + m[i][j]) * s
    q[k] = (m[k][i] + m[i][k]) * s
    self.w = (m[k][j] - m[j][k]) * s
    self.x, self.y, self.z = q[0], q[1], q[2]
  end
  self:normalize()
end

function LuaQuat:setMulXYZW(ax, ay, az, aw, bx, by, bz, bw)
  self.x = aw*bx + ax*bw + ay*bz - az*by
  self.y = aw*by + ay*bw + az*bx - ax*bz
  self.z = aw*bz + az*bw + ax*by - ay*bx
  self.w = aw*bw - ax*bx - ay*by - az*bz
end

function LuaQuat:setMul2(a, b)
  if type(a) == 'number' then
    self.x, self.y, self.z, self.w = b.x * a, b.y * a, b.z * a, b.w * a
  elseif type(b) == 'number' then
    self.x, self.y, self.z, self.w = a.x * b, a.y * b, a.z * b, a.w * b
  else
    self:setMulXYZW(a.x, a.y, a.z, a.w, b.x, b.y, b.z, b.w)
  end
end

-- = a^-1 * b
function LuaQuat:setInvMul2(a, b)
  local invSqNorm = -1 / (a:squaredNorm() + 1e-30)
  self:setMulXYZW(a.x * invSqNorm, a.y * invSqNorm, a.z * invSqNorm, -a.w * invSqNorm, b.x, b.y, b.z, b.w)
end

-- = a * b^-1
function LuaQuat:setMulInv2(a, b)
  local invSqNorm = -1 / (b:squaredNorm() + 1e-30)
  self:setMulXYZW(a.x, a.y, a.z, a.w, b.x * invSqNorm, b.y * invSqNorm, b.z * invSqNorm, -b.w * invSqNorm)
end

function LuaQuat:toEulerYXZ()
  local v = newLuaVec3xyz(0, 0, 0)
  v:setEulerYXZ(self)
  return v
end

function LuaQuat:toTorqueQuat()
  local sinhalf = math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)
  local tw = math.acos(clamp(self.w, -1, 1)) * 360 / math.pi
  if sinhalf ~= 0 then
    return {x = self.x / sinhalf, y = self.y / sinhalf, z = self.z / sinhalf, w = tw}
  else
    return {x = 1, y = 0, z = 0, w = tw}
  end
end

function LuaQuat:toDirUp()
  return self * vec3(0,1,0), self * vec3(0,0,1)
end

function quatFromDir(dir, up)
  local tmp = newLuaQuatxyzw(0, 0, 0, 1)
  tmp:setFromDir(dir, up)
  return tmp
end

function quatFromAxisAngle(axle, angleRad)
  angleRad = angleRad * 0.5
  local fsin = math.sin(angleRad)
  return newLuaQuatxyzw(fsin * axle.x, fsin * axle.y, fsin * axle.z, math.cos(angleRad))
end

function quatFromEuler(x, y, z) -- in radians
  local q = newLuaQuatxyzw(0, 0, 0, 1)
  q:setFromEuler(x, y, z)
  return q
end

--MARK: generic

-- returns -1, 1
function sign2(x)
  return max(min(x * math.huge, 1), -1)
end

-- returns -1, 0, 1
function sign(x)
  return max(min((x * 1e200) * 1e200, 1), -1)
end

fsign = sign

-- returns sign(s) * abs(v)
function signApply(s, v)
  local absv = abs(v)
  return max(min((s * 1e200) * 1e200, absv), -absv)
end

function guardZero(x) --branchless
  return 1 / max(min(1/x, 1e300), -1e300)
end

-- returns minValue when x is NaN
function clamp(x, minValue, maxValue )
  return min(max(x, minValue), maxValue)
end

function square(a)
  return a * a
end

function round(a)
  return floor(a+.5)
end

-- to round to Nth decimal use roundNear(x, 1e-N)
function roundNear(x, m)
  return floor(.5 + x/m)*m
end

function isnan(a)
  return a ~= a
end

function isinf(a)
  return abs(a) == math.huge
end

function isnaninf(a)
  return a * 0 ~= 0
end

function nanError(x)
  return x == x and x or error('NaN found')
end

-- Note: linearScale clamps
function linearScale(v, minValue, maxValue, minOutput, maxOutput)
  return minOutput + min(max((v - minValue) / (maxValue - minValue), 0), 1) * (maxOutput - minOutput)
end

function rescale(v, minValue, maxValue, minOutput, maxOutput)
  return minOutput + (maxOutput - minOutput) * (v - minValue) / (maxValue - minValue)
end

function lerp(from, to, t)
  return from + (to - from) * t  -- monotonic
end

function inverseLerp(from, to, value)
  local dif = to - from
  return abs(dif) > 1e-60 and (value - from) / dif or 0
end

function smoothstep(x)
  x = min(max(x, 0), 1) -- monotonic guard
  return x*x*(3 - 2*x)
end

function smootherstep(x)
  return min(max(x*x*x*(x*(x*6 - 15) + 10), 0), 1)
end

function smootheststep(x)
  x = min(max(x, 0), 1)
  return square(x*x)*(35-x*(x*(x*20-70)+84))
end

function smoothmin(a, b, k)
  k = k or 0.1
  local h = max(min(0.5+(b-a)/k, 1), 0)
  return h*a + (1-h) * (b - h*k*0.5)
end

function smoothmax(a, b, k)
  return smoothmin(a, b, -k)
end

function biasFun(x, k)
  local xk = x * k
  return (x + xk)/(1 + xk)
end

-- https://arxiv.org/pdf/2010.09714.pdf
-- D: \left\{0<x<t:t*x/(x+s*(t-x)+0.001), t<x<1:1-(1-t)*(1-x)/((1-x)-s*(t-x)+0.001)\right\}
function biasGainFun(x, t, s)
  t, s = t or 0.5, s or 0.25
  if x < t then
    return t * x/(x + s*(t - x) + 1e-20)
  else
    local x1 = 1-x
    return 1 - (1-t)*x1/(x1 - s*(t - x) + 1e-20)
  end
end

-- symmetric around 0
function sigmoid1(x, a)
  return x/((a or 1) + abs(x))
end

function bumpFun(x, peakLeftX, peakRightX, leftSlope, rightSlope, leftY, peakY, rightY, roundness)
  leftY, peakY, roundness = leftY or 0, peakY or 1, roundness or 10
  return leftY+0.5*((peakY-leftY)*(1 + sigmoid1(roundness*(x-peakLeftX), (leftSlope or 1))) +
    ((rightY or 0)-peakY)*(1+sigmoid1(roundness*(x-peakRightX), (rightSlope or 1))))
end

function median3(a,b,c)
  return max(min(a,b),min(max(a,b),c))
end

function median4(a,b,c,d)
  return ((a+b+c+d)-max(a,b,c,d)-min(a,b,c,d))*0.5
end

function median5(a,b,c,d,e)
  return median3(e, max(min(a,b),min(c,d)), min(max(a,b),max(c,d)))
end

function pointBB2d(x, y, radius)
  return x - radius, y - radius, x + radius, y + radius
end

function lineBB2d(x1, y1, x2, y2, radius)
  local enlarge = radius or 0
  return min(x1, x2) - enlarge, min(y1, y2) - enlarge, max(x1, x2) + enlarge, max(y1, y2) + enlarge
end

--MARK: curves

function cardinalSpline(p0, p1, p2, p3, t, s, d1, d2, d3)
  d1, d2, d3 = max(d1 or 1, 1e-30), d2 or 1, max(d3 or 1, 1e-30)
  s = (s or 0.5) * 2
  local sd2, tt, t_1 = s*d2, t*t, t-1
  local t_1sq = t_1 * t_1
  local m1 = (p1 - p0) / d1 + (p0 - p2) / (d1 + d2)
  local m2 = (p1 - p3) / (d2 + d3) + (p3 - p2) / d3
  return t*t_1sq*sd2 * m1 + tt*t_1*sd2 * m2 + t_1sq*(2*t+1) * p1 - tt*(2*t-3) * p2 + s*t_1*(t*t_1 + tt) * (p2 - p1)
end

-- x, y, z independent from each other (no distance needed)
function catmullRom(p0, p1, p2, p3, t, s)
  return cardinalSpline(p0, p1, p2, p3, t, s or 0.5, 1, 1, 1)
end

function catmullRomChordal(p0, p1, p2, p3, t, s)
  return cardinalSpline(p0, p1, p2, p3, t, s or 0.5, p0:distance(p1), p1:distance(p2), p2:distance(p3))
end

-- best stability, no loops or self-intersections
function catmullRomCentripetal(p0, p1, p2, p3, t, s)
  return cardinalSpline(p0, p1, p2, p3, t, s or 0.5, sqrt(p0:distance(p1)), sqrt(p1:distance(p2)), sqrt(p2:distance(p3)))
end

function monotonicSteffen(y0, y1, y2, y3, x0, x1, x2, x3, x)
  local x1x0, x2x1, x3x2 = x1-x0, x2-x1, x3-x2
  local delta0, delta1, delta2 = (y1-y0) / (x1x0 + 1e-30), (y2-y1) / (x2x1 + 1e-30), (y3-y2) / (x3x2 + 1e-30)
  local m1 = (sign(delta0)+sign(delta1)) * min(abs(delta0),abs(delta1), 0.5*abs((x2x1*delta0 + x1x0*delta1) / (x2-x0 + 1e-30)))
  local m2 = (sign(delta1)+sign(delta2)) * min(abs(delta1),abs(delta2), 0.5*abs((x3x2*delta1 + x2x1*delta2) / (x3-x1 + 1e-30)))
  local xx1 = x - x1
  local xrel = xx1 / max(x2x1, 1e-30)
  return y1 + xx1*(m1 + xrel*(delta1 - m1 + (xrel - 1)*(m1 + m2 - 2*delta1)))
end

function quadraticBezier(p1, p2, p3, t)
  local t1 = 1-t
  return t1*t1*p1 + 2*t*t1*p2 + t*t*p3
end

-- it can represent segments of any conic section: hyperbolas, parabolas, ellipses, and circles
function conicBezier(p1, p2, p3, t, w)
  w = w or 1
  local t1 = 1-t
  local t1t1, wtt12, tt = t1*t1, 2*w*t*t1, t*t
  local d = 1/(t1t1 + wtt12 + tt)
  return t1t1*d*p1 + wtt12*d*p2 + tt*d*p3
end

-- does not pass through points, smooths out the signal
function biQuadratic(p0, p1, p2, p3, t)
  local p12 =  p1 + (p2 - p1) * (t * 0.5 + 0.25)
  if t <= 0.5 then
    local p01 = p0 + (p1 - p0) * (t * 0.5 + 0.75)
    return p01 + (p12 - p01) * (t + 0.5)
  else
    return p12 + (p2 + (p3 - p2) * (t * 0.5 - 0.25) - p12) * (t - 0.5)
  end
end

--MARK: geom

local function axisCheck(v1, v2, b1e1, b1e2, b2e1, b2e2)
  tmpv2:setCross(v1, v2) -- axis
  return abs(tmpv1:dot(tmpv2))-abs(b2e1:dot(tmpv2))-abs(b2e2:dot(tmpv2))<=abs(b1e1:dot(tmpv2))+abs(b1e2:dot(tmpv2))
end
function overlapsOBB_OBB(c1, x1, y1, z1, c2, x2, y2, z2)
  tmpv1:setSub2(c1, c2)
  local d11,d12,d13=abs(x1:dot(x2)),abs(x1:dot(y2)),abs(x1:dot(z2))
  if abs(tmpv1:dot(x1))-d11-d12-d13>x1:squaredLength() then return false end
  local d21,d22,d23=abs(y1:dot(x2)),abs(y1:dot(y2)),abs(y1:dot(z2))
  if abs(tmpv1:dot(y1))-d21-d22-d23>y1:squaredLength() then return false end
  local d31,d32,d33=abs(z1:dot(x2)),abs(z1:dot(y2)),abs(z1:dot(z2))
  return abs(tmpv1:dot(z1))-d31-d32-d33<=z1:squaredLength() and abs(tmpv1:dot(x2))-d11-d21-d31<=x2:squaredLength()
     and abs(tmpv1:dot(y2))-d12-d22-d32<=y2:squaredLength() and abs(tmpv1:dot(z2))-d13-d23-d33<=z2:squaredLength()
     and axisCheck(x1,x2,y1,z1,y2,z2) and axisCheck(x1,y2,y1,z1,x2,z2) and axisCheck(x1,z2,y1,z1,x2,y2)
     and axisCheck(y1,x2,x1,z1,y2,z2) and axisCheck(y1,y2,x1,z1,x2,z2) and axisCheck(y1,z2,x1,z1,x2,y2)
     and axisCheck(z1,x2,x1,y1,y2,z2) and axisCheck(z1,y2,x1,y1,x2,z2) and axisCheck(z1,z2,x1,y1,x2,y2)
end

function containsOBB_OBB(c1, x1, y1, z1, c2, x2, y2, z2)
  tmpv1:setSub2(c1, c2)
  return abs(tmpv1:dot(x1))+abs(x1:dot(x2))+abs(x1:dot(y2))+abs(x1:dot(z2))<=x1:squaredLength()
     and abs(tmpv1:dot(y1))+abs(y1:dot(x2))+abs(y1:dot(y2))+abs(y1:dot(z2))<=y1:squaredLength()
     and abs(tmpv1:dot(z1))+abs(z1:dot(x2))+abs(z1:dot(y2))+abs(z1:dot(z2))<=z1:squaredLength()
end

function OBBsquaredDistance(c1, x1, y1, z1, p)
  tmpv1:setSub2(p, c1)
  local x1sql, y1sql, z1sql = x1:squaredLength(), y1:squaredLength(), z1:squaredLength()
  local a, b, c = tmpv1:dot(x1), tmpv1:dot(y1), tmpv1:dot(z1)
  if abs(a) < x1sql and abs(b) < y1sql and abs(c) < z1sql then return 0 end -- fix small r2 num accuracy
  a, b, c = min(max(a/max(x1sql, 1e-30),-1),1), min(max(b/max(y1sql, 1e-30),-1),1), min(max(c/max(z1sql, 1e-30),-1),1)
  return square(a*x1.x+b*y1.x+c*z1.x-tmpv1.x) + square(a*x1.y+b*y1.y+c*z1.y-tmpv1.y) + square(a*x1.z+b*y1.z+c*z1.z-tmpv1.z)
end

function overlapsOBB_Sphere(c1, x1, y1, z1, c2, r2)
  return OBBsquaredDistance(c1, x1, y1, z1, c2) <= r2*r2
end

function overlapsOBB_Plane(c1, x1, y1, z1, plpos, pln)
  tmpv1:setSub2(c1, plpos)
  return abs(tmpv1:dot(pln))<=abs(x1:dot(pln))+abs(y1:dot(pln))+abs(z1:dot(pln))
end

function containsOBB_Sphere(c1, x1, y1, z1, c2, r2)
  tmpv1:setSub2(c1, c2)
  local x1len, y1len, z1len = x1:length(), y1:length(), z1:length()
  return abs(tmpv1:dot(x1))<=x1len*(x1len-r2) and abs(tmpv1:dot(y1))<=y1len*(y1len-r2) and abs(tmpv1:dot(z1))<=z1len*(z1len-r2)
end

function containsSphere_OBB(c1, r1, c2, x2, y2, z2)
  local cc = c1 - c2
  local ccx1, cc_x2, y2z2, y2_z2 = cc+x2, cc-x2, y2+z2, y2-z2
  return max((ccx1+y2z2):squaredLength(), (ccx1+y2_z2):squaredLength(), (ccx1-y2_z2):squaredLength(), (ccx1-y2z2):squaredLength(),
      (cc_x2+y2z2):squaredLength(), (cc_x2+y2_z2):squaredLength(), (cc_x2-y2_z2):squaredLength(), (cc_x2-y2z2):squaredLength())<=r1*r1
end

function containsOBB_point(c1, x1, y1, z1, p)
  tmpv1:setSub2(c1, p)
  return abs(tmpv1:dot(x1))<=x1:squaredLength() and abs(tmpv1:dot(y1))<=y1:squaredLength() and abs(tmpv1:dot(z1))<=z1:squaredLength()
end

function containsEllipsoid_Point(c1, x1, y1, z1, p)
  tmpv1:setSub2(p, c1)
  local x, y, z = tmpv1:dot(x1), tmpv1:dot(y1), tmpv1:dot(z1)
  local a2, b2, c2 = x1:squaredLength(), y1:squaredLength(), z1:squaredLength()
  a2, b2, c2 = a2*a2, b2*b2, c2*c2
  local b2c2 = b2*c2
  return x*x*b2c2 + a2*(y*y*c2 + z*z*b2) <= a2*b2c2
end

function constainsCylinder_Point(cposa, cposb, cR, p)
  local xnorm, r2 = p:xnormSquaredDistanceToLineSegment(cposa, cposb)
  return xnorm >=0 and xnorm <= 1 and r2 <= cR*cR
end

function altitudeOBB_Plane(c1, x1, y1, z1, plpos, pln)
  tmpv1:setSub2(c1, plpos)
  return tmpv1:dot(pln)+abs(x1:dot(pln))+abs(y1:dot(pln))+abs(z1:dot(pln))
end

-- returns signed distance of plane on the ray
function intersectsRay_Plane(rpos, rdir, plpos, pln)
  tmpv1:setSub2(plpos, rpos)
  return min(tmpv1:dot(pln) / rdir:dot(pln), math.huge)
end

-- hit: minhit < maxhit and maxhit > 0, inside: minhit < 0 and maxhit > 0
function intersectsRay_OBB(rpos, rdir, c1, x1, y1, z1)
  tmpv1:setSub2(c1, rpos)
  local rposc1x1, x1sq, invrdirx1 = tmpv1:dot(x1), x1:squaredLength(), 1 / rdir:dot(x1)
  local dx1, dx2 = (rposc1x1 - x1sq) * invrdirx1, min((rposc1x1 + x1sq) * invrdirx1, math.huge)
  local rposc1y1, y1sq, invrdiry1 = tmpv1:dot(y1), y1:squaredLength(), 1 / rdir:dot(y1)
  local dy1, dy2 = (rposc1y1 - y1sq) * invrdiry1, min((rposc1y1 + y1sq) * invrdiry1, math.huge)
  local rposc1z1, z1sq, invrdirz1 = tmpv1:dot(z1), z1:squaredLength(), 1 / rdir:dot(z1)
  local dz1, dz2 = (rposc1z1 - z1sq) * invrdirz1, min((rposc1z1 + z1sq) * invrdirz1, math.huge)

  local minhit, maxhit = max(min(dx1, dx2), min(dy1, dy2), min(dz1, dz2)), min(max(dx1, dx2), max(dy1, dy2), max(dz1, dz2))
  return (minhit <= maxhit and minhit or math.huge), maxhit
end

function intersectsRay_Sphere(rpos, rdir, cpos, cr)
  local rcpos = cpos - rpos
  local dcr = rdir:dot(rcpos)
  local s = dcr*dcr - rcpos:squaredLength() + cr*cr
  if s < 0 then return math.huge, math.huge end
  s = sqrt(s)
  return dcr - s, dcr + s
end

function intersectsRay_Ellipsoid(rpos, rdir, c1, x1, y1, z1)
  local invx1, invy1, invz1 = 1 / (x1:squaredLength() + 1e-30), 1 / (y1:squaredLength() + 1e-30), 1 / (z1:squaredLength() + 1e-30)
  tmpv1:setSub2(rpos, c1)
  local pM = vec3(tmpv1:dot(x1)*invx1, tmpv1:dot(y1)*invy1, tmpv1:dot(z1)*invz1)
  local dirM = vec3(rdir:dot(x1)*invx1, rdir:dot(y1)*invy1, rdir:dot(z1)*invz1)

  local a, b, c = dirM:squaredLength(), 2*pM:dot(dirM), pM:squaredLength() - 1
  local d = b*b - 4*a*c
  if d < 0 then return math.huge, math.huge end
  d = -b -sign(b)*sqrt(d)
  local r1, r2 = 0.5*d / a, 2*c / d
  return min(r1, r2), max(r1,r2)
end

function intersectsRay_Cylinder(rpos, rdir, cposa, cposb, cR)
  local rca = cposa - rpos
  local cpnorm = cposb - cposa; cpnorm:normalize()
  local cp = rca:projectToOriginPlane(cpnorm)
  local rdp = rdir:projectToOriginPlane(cpnorm)
  local invrdplen = 1 / (rdp:length() + 1e-30)
  rdp:setScaled(invrdplen)
  local minhit, maxhit = intersectsRay_Sphere(newLuaVec3xyz(0, 0, 0), rdp, cp, cR)
  minhit, maxhit = minhit * invrdplen, maxhit * invrdplen
  local plhita, plhitb = intersectsRay_Plane(rpos, rdir, cposa, cpnorm), intersectsRay_Plane(rpos, rdir, cposb, cpnorm)
  minhit, maxhit = max(minhit, min(plhita, plhitb)), min(maxhit, max(plhita, plhitb))
  return (minhit <= maxhit and minhit or math.huge), maxhit
end

-- https://iquilezles.org/articles/intersectors , returns first hit, untested
function intersectsRay_Capsule(rpos, rdir, cposa, cposb, cR )
  local ba, oa = cposb - cposa, rpos - cposa
  local baba, bard, baoa = ba:squaredLength(), ba:dot(rdir), ba:dot(oa)
  local a, b = baba - bard*bard, rdir:dot(oa) * baba - baoa*bard
  local h = b*b - a*((oa:squaredLength()-cR*cR) * baba - baoa*baoa)
  if h >= 0 then
    local t = (-b-sqrt(h))/a
    local y = baoa + t*bard
    if y > 0 and y < baba then return t end -- body
    oa:set(y <= 0 and oa or rpos - cposb) -- caps
    b = rdir:dot(oa)
    h = b*b - oa:squaredLength() - cR*cR
    if h > 0 then return -b-sqrt(h) end
  end
  return math.huge
end

-- returns hit distance, barycentric x, y
function intersectsRay_Triangle(rpos, rdir, a, b, c)
  local ca, bc = c - a, b - c
  local norm = ca:cross(bc)
  local rposc = rpos - c
  local pOnTri = rposc:dot(norm) / rdir:dot(norm)
  if pOnTri <= 0 then
    local pacnorm = (rposc - rdir * pOnTri):cross(norm)
    local bx, by = bc:dot(pacnorm), ca:dot(pacnorm)
    if min(bx, by) >= 0 then
      local normSq = norm:squaredLength() + 1e-30
      if bx + by <= normSq then
        return -pOnTri, bx / normSq, by / normSq
      end
    end
  end
  return math.huge, -1, -1
end