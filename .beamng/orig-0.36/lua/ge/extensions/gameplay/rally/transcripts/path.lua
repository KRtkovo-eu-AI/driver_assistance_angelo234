-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local C = {}
local logTag = ''

local cc = require('/lua/ge/extensions/gameplay/rally/util/colors')
local rallyUtil = require('/lua/ge/extensions/gameplay/rally/util')

function C:getNextUniqueIdentifier()
  if not self._uid then
    self._uid = 0
  end
  self._uid = self._uid + 1
  return self._uid
end

function C:init(fname)
  self.fname = fname

  self.selection_sphere_r = 4
  self._draw_debug_hover_tsc_id = nil

  self.transcripts = require('/lua/ge/extensions/gameplay/util/sortedList')(
    "transcripts",
    self,
    require('/lua/ge/extensions/gameplay/rally/transcripts/entry')
  )
end

function C:load()
  if not self.fname  then
    log('W', logTag, 'load: transcript fname is nil')
    return false
  end

  if not FS:fileExists(self.fname) then
    log('W', logTag, 'load: transcript fname doesnt exist')
    return false
  end

  local json = jsonReadFile(self.fname)
  if not json then
    log('E', logTag, 'couldnt find notebook file')
    return false
  end
  self:onDeserialized(json)
  log('I', logTag, 'loaded '.. (#self.transcripts.sorted) ..' transcripts from '..self.fname)
  return true
end

function C:save()
  local json = self:onSerialize()
  jsonWriteFile(self.fname, json, false)
  log('I', logTag, 'saved transcripts file '..self.fname)
end

function C:onSerialize()
  local ret = {
    transcripts = self.transcripts:onSerialize(),
  }

  return ret
end

function C:onDeserialized(data)
  if not data then return end
  local oldIdMap = {}

  self.transcripts:clear()
  self.transcripts:onDeserialized(data.transcripts, oldIdMap)
end

function C:drawDebug(selected_id)
  self:drawDebugTranscripts(selected_id)
end

-- helped me figure out quat rotation
-----------------------------------------------------------------------
-- test 1
--
-- local function toRadians(degrees)
--   return degrees * (math.pi / 180)
-- end
--
-- local theta = toRadians(45) / 2  -- Convert 45 degrees to radians and halve it
--
-- local testQ = quat({
--   w = math.cos(theta),
--   x = 0,
--   y = 0,
--   z = math.sin(theta)
-- })
--
-- print("Quaternion for 45-degree rotation around Z-axis:", testQ.w, testQ.x, testQ.y, testQ.z)
--
-- local testP = {x=-373.3353577, y=18.58174515, z=49.07889557}
--
-- local h = 2
-- local l = 5
--
-- testP = pos
-- testP = testP + vec3(0,0,20)
-- testQ = rot
--
-- local upVector = vec3(0, 0, 1)  -- 'up' in a Z-up system
-- local rotatedUpVector = testQ * upVector * h  -- Rotate and scale the up vector
-- local topOfCar = testP + rotatedUpVector  -- This gives the top point of the car
--
-- local forwardVector = vec3(0, 1, 0)
-- local rotatedForwardVector = testQ * forwardVector * l
-- local frontOfCar = testP + rotatedForwardVector
--
-- debugDrawer:drawSphere(testP, 1, ColorF(0,1,0,shapeAlpha)) -- green
-- debugDrawer:drawSphere(topOfCar, 1, ColorF(0,0,1,shapeAlpha)) -- blue
-- debugDrawer:drawSphere(frontOfCar, 1, ColorF(0,1,1,shapeAlpha)) -- aqua
--
function C:drawDebugTranscripts(selected_id)
  for i,tsc in ipairs(self.transcripts.sorted) do
    local is_hovered = self._draw_debug_hover_tsc_id == tsc.id
    local is_selected = selected_id == tsc.id
    tsc:drawDebug(is_hovered, is_selected)
  end
end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end
