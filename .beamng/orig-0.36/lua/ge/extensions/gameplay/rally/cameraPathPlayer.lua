-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local logTag = ''
local rallyUtil = require('/lua/ge/extensions/gameplay/rally/util')
local cc = require('/lua/ge/extensions/gameplay/rally/util/colors')

local C = {}

function C:init(snaproad)
  self.snaproad = snaproad
end

-- {
--   "looped":false,
--   "manualFov":false,
--   "markers":[
--     {
--       "fov":20,
--       "movingEnd":true,
--       "movingStart":true,
--       "pos":{
--         "x":547.2601318,
--         "y":576.2945557,
--         "z":118.7703247
--       },
--       "rot":{
--         "z":-0.7501421743,
--         "w":0.6541676806,
--         "x":0.06355749063,
--         "y":0.072882161
--       },
--       "time":0,
--       "trackPosition":true
--     },
--     {
--       "fov":20,
--       "movingEnd":true,
--       "movingStart":true,
--       "pos":{
--         "x":549.3669434,
--         "y":573.342041,
--         "z":118.89608
--       },
--       "rot":{
--         "z":-0.7501421743,
--         "w":0.6541676806,
--         "x":0.06355749063,
--         "y":0.072882161
--       },
--       "time":3,
--       "trackPosition":true
--     }
--   ],
--   "name":"camPath1",
--   "rotFixId":2,
--   "version":"6"
-- }

-- copied from core_paths.loadPath
local function loadPath(points)
  local capture_markers = {}

  local t_zero = points[1].ts

  for i,cap in ipairs(points) do
    local marker = {
      fov = 60,
      movingEnd = true,
      movingStart = true,
      pos = cap.pos + vec3(0,0,1),
      rot = cap.quat,
      time = cap.ts - t_zero,
      trackPosition = false,
      -- positionSmooth = 0.0,
    }
    table.insert(capture_markers, marker)
  end

  local pathJsonObj = {
    looped = false,
    loopTime = nil,
    name = 'rally_campath',
    rotFixId = 2,
    version = "6",
    markers = capture_markers,
  }

  local defaultSplineSmoothing = 0.5

  -- unchanged from original below this line --

  local res = { markers = {}}

  res.looped = pathJsonObj.looped or false
  res.loopTime = pathJsonObj.loopTime


  -- extract all its markers
  for i, markerData in ipairs(pathJsonObj.markers) do
    local marker = {
      pos = vec3(markerData.pos.x, markerData.pos.y, markerData.pos.z),
      rot = quat(markerData.rot.x, markerData.rot.y, markerData.rot.z, markerData.rot.w),
      time = markerData.time,
      fov = markerData.fov,
      trackPosition = markerData.trackPosition,
      positionSmooth = markerData.positionSmooth or defaultSplineSmoothing,
      bullettime = markerData.bullettime or 1,
      cut = markerData.cut,
      movingStart = markerData.movingStart or true,
      movingEnd = markerData.movingEnd or true
    }
    table.insert(res.markers, marker)
  end
  -- fix up the rotations
  local markerCount = tableSize(pathJsonObj.markers)
  for i = 2, markerCount do
    if res.markers[i] then
      if res.markers[i].rot:dot(res.markers[i - 1].rot) < 0 then
        res.markers[i].rot = -res.markers[i].rot
      end
    end
  end
  res.rotFixId = markerCount

  -- res.filename = pathFileName
  -- res.replay = pathJsonObj.replay
  res.name = pathJsonObj.name
  -- addPath(res)
  return res
end

function C:play()
  local points = self.snaproad:pointsForCameraPath()
  if not points then return end

  local cam_path = loadPath(points)
  log('D', logTag, 'playing camPath')
  core_paths.playPath(cam_path)
end

function C:stop()
  core_paths.stopCurrentPath()
end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end

