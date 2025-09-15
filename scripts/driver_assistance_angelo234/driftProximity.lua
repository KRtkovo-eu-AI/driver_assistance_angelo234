local M = {}

-- Adjusts a point from an old scan frame into the current scan frame
-- point: table {x, y, z} in old frame's local coordinates
-- oldF: table {origin, dir, right, up}
-- newF: table {origin, dir, right, up}
function M.apply(point, oldF, newF)
  if not point or not oldF or not newF then return point end
  -- convert local point to world space using old frame
  local world = oldF.origin + oldF.right * point.x + oldF.dir * point.y + oldF.up * point.z
  -- shift into new frame
  local rel = world - newF.origin
  return {
    x = rel:dot(newF.right),
    y = rel:dot(newF.dir),
    z = rel:dot(newF.up)
  }
end

return M
