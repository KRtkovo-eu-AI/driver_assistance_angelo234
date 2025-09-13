local M = {}

-- luacheck: globals castRay castRayStatic
local sin, cos = math.sin, math.cos

-- Scans environment using ray casts and returns a table of 3D points
-- origin: vec3 start position in world space
-- dir: forward direction vector
-- up: up direction vector
-- maxDist: max scan distance
-- hFov, vFov: horizontal and vertical field of view in radians
-- hRes, vRes: number of rays horizontally and vertically
-- ignoreId: optional vehicle id to exclude from results
local function scan(origin, dir, up, maxDist, hFov, vFov, hRes, vRes, minDist, ignoreId)
  local points = {}
  dir = dir:normalized()
  up = up:normalized()
  -- BeamNG uses a left-handed coordinate system, so derive the horizontal
  -- right vector using forward × up to avoid mirroring the scan
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
      -- Perform both static and dynamic raycasts and keep the closest hit.
      local staticDist = castRayStatic(origin, rayDir, maxDist)
      local dynHit = castRay(origin, origin + rayDir * maxDist, true, true)

      local dist, pt
      if staticDist and staticDist < maxDist then
        dist = staticDist
        pt = origin + rayDir * staticDist
      end
      if dynHit and dynHit.dist and dynHit.dist < maxDist then
        local hitId = dynHit.objectId or dynHit.objectID or dynHit.cid
        if not hitId and dynHit.obj and dynHit.obj.getID then
          hitId = dynHit.obj:getID()
        end
        if not ignoreId or hitId ~= ignoreId then
          if not dist or dynHit.dist < dist then
            dist = dynHit.dist
            pt = dynHit.pt
          end
        end
      end

      if pt and dist and dist >= minDist then
        points[#points + 1] = pt
      end
    end
  end
  return points
end

M.scan = scan

return M

