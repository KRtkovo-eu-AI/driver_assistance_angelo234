local M = {}

-- luacheck: globals castRay castRayStatic
local sin, cos = math.sin, math.cos

-- Scans environment using ray casts and returns a table of 3D points
-- origin: vec3 start position in world space
-- dir: forward direction vector
-- up: up direction vector
-- maxDist: max scan distance
-- hFov, vFov: horizontal and vertical field of view in radians
-- ignoreId: optional vehicle id to exclude from results
-- opts: optional table {hStart, hStep, vStart, vStep, maxRays, maxPoints}
--       to scan only a subset of rays or clamp the total number processed
local function scan(origin, dir, up, maxDist, hFov, vFov, hRes, vRes, minDist, ignoreId, opts)
  local points = {}
  dir = dir:normalized()
  up = up:normalized()
  -- BeamNG uses a left-handed coordinate system, so derive the horizontal
  -- right vector using forward × up to avoid mirroring the scan
  local right = dir:cross(up)
  right = right:normalized()
  minDist = minDist or 0

  opts = opts or {}
  local hStart = opts.hStart or 0
  local baseHStep = math.max(1, opts.hStep or 1)
  local vStart = opts.vStart or 0
  local baseVStep = math.max(1, opts.vStep or 1)
  local maxRays = opts.maxRays and math.max(1, math.floor(opts.maxRays))
  local maxPoints = opts.maxPoints and math.max(1, math.floor(opts.maxPoints))

  local hStep = baseHStep
  local vStep = baseVStep

  if maxRays then
    local function count(stepH, stepV)
      local hCount = 0
      local vCount = 0
      if hRes > 0 and hStart <= (hRes - 1) then
        hCount = math.floor(((hRes - 1) - hStart) / stepH) + 1
      end
      if vRes > 0 and vStart <= (vRes - 1) then
        vCount = math.floor(((vRes - 1) - vStart) / stepV) + 1
      end
      if hCount == 0 or vCount == 0 then
        return hCount, vCount, 0
      end
      return hCount, vCount, hCount * vCount
    end
    local _, _, total = count(hStep, vStep)
    local guard = 0
    while total > maxRays and guard < 64 do
      local newVStep = vStep + baseVStep
      local _, _, reduced = count(hStep, newVStep)
      if reduced >= total then break end
      vStep = newVStep
      total = reduced
      guard = guard + 1
    end
    guard = 0
    while total > maxRays and guard < 64 do
      local newHStep = hStep + baseHStep
      local _, _, reduced = count(newHStep, vStep)
      if reduced >= total then break end
      hStep = newHStep
      total = reduced
      guard = guard + 1
    end
  end

  local processed = 0
  local stop = false

  for i = hStart, hRes - 1, hStep do
    local hAng = -hFov * 0.5 + hFov * i / math.max(1, hRes - 1)
    local ch, sh = cos(hAng), sin(hAng)
    for j = vStart, vRes - 1, vStep do
      if maxRays and processed >= maxRays then
        stop = true
        break
      end
      local vAng = -vFov * 0.5 + vFov * j / math.max(1, vRes - 1)
      local cv, sv = cos(vAng), sin(vAng)
      local rayDir = dir * (cv * ch) + right * (cv * sh) + up * sv
      processed = processed + 1
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
        if hitId and (not ignoreId or hitId ~= ignoreId) then
          if not dist or dynHit.dist < dist then
            dist = dynHit.dist
            pt = dynHit.pt
          end
        end
      end

      if pt and dist and dist >= minDist then
        points[#points + 1] = pt
        if maxPoints and #points >= maxPoints then
          stop = true
          break
        end
      end
    end
    if stop then break end
  end
  return points
end

M.scan = scan

return M

