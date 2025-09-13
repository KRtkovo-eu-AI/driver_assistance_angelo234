local M = {}

local sin, cos, rad = math.sin, math.cos, math.rad

-- Scans environment using ray casts and returns a table of 3D points
-- origin: vec3 start position in world space
-- dir: forward direction vector
-- up: up direction vector
-- maxDist: max scan distance
-- hFov, vFov: horizontal and vertical field of view in radians
-- hRes, vRes: number of rays horizontally and vertically
-- minDist: ignore hits closer than this distance from origin
local function scan(origin, dir, up, maxDist, hFov, vFov, hRes, vRes, minDist)
  local points = {}
  dir = dir:normalized()
  up = up:normalized()
  local right = dir:cross(up)
  right = right:normalized()
  minDist = minDist or 0

  for i = 0, hRes - 1 do
    local hAng = -hFov * 0.5 + hFov * i / math.max(1, hRes - 1)
    local ch, sh = cos(hAng), sin(hAng)
    for j = 0, vRes - 1 do
      local vAng = -vFov * 0.5 + vFov * j / math.max(1, vRes - 1)
      local cv, sv = cos(vAng), sin(vAng)
      local rayDir = dir * (cv * ch) + right * (cv * sh) + up * sv
      local dest = origin + rayDir * maxDist
      local hit = castRay(origin, dest)
      if hit and hit.dist and hit.dist >= minDist and hit.dist < maxDist then
        points[#points + 1] = origin + rayDir * hit.dist
      end
    end
  end
  return points
end

M.scan = scan

return M

