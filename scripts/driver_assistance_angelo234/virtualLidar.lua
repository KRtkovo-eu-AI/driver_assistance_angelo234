local M = {}

-- luacheck: globals castRay
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
      local dest = origin + rayDir * maxDist
      -- cast a general ray that reports detailed hit information so we can
      -- distinguish between obstructed and free paths
      local hit = castRay(origin, dest, true, true)
      -- "castRay" returns nil when nothing is hit, otherwise a table that
      -- always contains a distance. Rays that travel unobstructed return a
      -- distance equal to the cast length and have no object identifier. To
      -- avoid coloring free space, only keep results that report a valid object
      -- id and fall within the desired distance range.
      if hit and hit.dist and hit.dist >= minDist and hit.dist < maxDist then
        local hitId = hit.objectId or hit.objectID or hit.cid or hit.obj
        -- BeamNG returns an object id of 0 for terrain and unobstructed rays.
        -- Treat those as free space by requiring a positive id so only
        -- collisions with actual objects populate the point cloud.
        if hitId and hitId > 0 and (not ignoreId or hitId ~= ignoreId) then
          points[#points + 1] = origin + rayDir * hit.dist
        end
      end
    end
  end
  return points
end

M.scan = scan

return M

