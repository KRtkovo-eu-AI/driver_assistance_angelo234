-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

M.getSurfaceHeight = function(pos)
  local get = obj and obj or be

  -- first, try a reasonable position for the raycast - two meters above the object
  local topPos = vec3(pos.x, pos.y, pos.z + 2)
  local height = get:getSurfaceHeightBelow(topPos)
  if height > -1e10 then -- the function returns -1e20 when the raycast fails
    return height
  end

  -- try a safe z - 100km above
  topPos.z = 1e5
  height = get:getSurfaceHeightBelow(topPos)
  if height > -1e10 then
    return height
  end

  -- both raycasts failed, use the input z value
  return pos.z
end

return M