-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

local raycastLength = 1.5
local maxGroundDistance = 1.1


-- for GC
local tempVec = vec3()
local vec3TempCenter = vec3()
local vecDown = vec3(0,0,-1)
local vec3TempCenter = vec3()
local vec3Offset = vec3(0,0,0.2)
local vehOOBB

local tempVec2 = vec3()
local oobbCenter = vec3()
local function isOnWheels(vehId)
  if not map.objects[vehId] then return false end

  oobbCenter:set(be:getObjectOOBBCenterXYZ(vehId))
  local x, y, z = be:getObjectOOBBHalfExtentsXYZ(vehId);
  oobbCenter.z = oobbCenter.z - z + 0.3

  local distToGround = castRayStatic(oobbCenter, vecDown, raycastLength)
  return distToGround <= maxGroundDistance
end

M.isOnWheels = isOnWheels

return M


