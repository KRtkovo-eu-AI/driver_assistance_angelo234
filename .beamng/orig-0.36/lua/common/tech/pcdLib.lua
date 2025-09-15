-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- This file creates pointcloud (.pcd) files, which are the standard
-- for Lidar, Radar, etc. pointclouds.

local M = {}
local logTag = 'pcdLib'

local sbuffer = require('string.buffer')
local header = {
  VERSION,
  FIELDS,
  SIZE,
  TYPE,
  COUNT,
  WIDTH,
  HEIGHT,
  VIEWPOINT,
  POINTS,
  DATA,
}

local Pcd = {}

function Pcd:new(o)
  o = o or {}
  setmetatable(o, self)
  self.__index = self
  self.version = '.7'
  self.fields = {}
  self.size = {}
  self.type = {}
  self.count = {}
  self.width = 0
  self.height = 1 -- unorganized dataset
  self.viewpoint = '0 0 0 1 0 0 0' -- pos.x pos.y pos.z rot.w rot.x rot.y rot.z
  self.pos = vec3(0, 0, 0)
  self.rotQuat = quat(0, 0, 0, 1)
  self.points = nil
  self.data = 'binary'
  self.payload = sbuffer.new()
  return o
end

function Pcd:addField(name, size, type)
  type = string.sub(type, 1, 1):upper()
  local n = #self.fields + 1

  self.fields[n] = name
  self.size[n] = size
  self.type[n] = type
  self.count[n] = 1
end

function Pcd:setViewpoint(pos, rotQuat)
  self.viewpoint = string.format('%f %f %f %f %f %f %f', pos.x, pos.y, pos.z, rotQuat.w, rotQuat.x, rotQuat.y, rotQuat.z)
end

function Pcd:setData(data, bytes)
  self.payload:set(data, bytes)
end

local function fieldToStr(x)
  if type(x) == 'table' then
    return table.concat(x, ' ')
  end
  return tostring(x)
end

function Pcd:writeHeader(file)
  file:write('VERSION .7\n')
  file:write('FIELDS ' .. fieldToStr(self.fields) .. '\n')
  file:write('SIZE ' .. fieldToStr(self.size) .. '\n')
  file:write('TYPE ' .. fieldToStr(self.type) .. '\n')
  file:write('COUNT ' .. fieldToStr(self.count) .. '\n')
  file:write('WIDTH ' .. fieldToStr(self.width) .. '\n')
  file:write('HEIGHT ' .. fieldToStr(self.height) .. '\n')
  file:write('VIEWPOINT ' .. fieldToStr(self.viewpoint) .. '\n')
  file:write('POINTS ' .. fieldToStr(self.points) .. '\n')
  file:write('DATA ' .. fieldToStr(self.data) .. '\n')
end

function Pcd:save(filename)
  local file = io.open(filename, 'w')
  if not file then
    log('E', logTag, 'Cannot save to ' .. filename .. '.')
    return false
  end

  local payload = self.payload
  if self.points == nil then
    local pointSize = 0
    for _, v in ipairs(self.size) do
      pointSize = pointSize + v
    end

    local bytes = #self.payload
    if bytes % pointSize ~= 0 then
      log('E', logTag, 'Byte size not dividable by point size!')
    end
    self.points = bytes / pointSize
    self.width = self.points
  end

  self:writeHeader(file)
  file:write(payload:tostring())
  file:flush()
  file:close()
  log('I', logTag, 'Saved to ' .. filename .. '.')
end

local function newPcd()
  return Pcd:new()
end

M.newPcd = newPcd

return M