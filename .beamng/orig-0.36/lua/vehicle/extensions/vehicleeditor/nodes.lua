-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

function M.calculateNodesWeight(nodesTbl)
  if not nodesTbl then nodesTbl = v.data.nodes end
  local totalWeight = 0
  local max = -math.huge
  local min = math.huge
  for _,node in pairs(nodesTbl) do
    totalWeight = totalWeight + obj:getNodeMass(node.cid)
    if obj:getNodeMass(node.cid) > max then max = obj:getNodeMass(node.cid) end
    if obj:getNodeMass(node.cid) < min then min = obj:getNodeMass(node.cid) end
  end
  return totalWeight, min, max
end

return M