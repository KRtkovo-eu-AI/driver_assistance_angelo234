-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local electricsName = nil
local attachedValue = nil
local detachedValue = nil

local function onCouplerAttached(nodeId, obj2id, obj2nodeId)
  if obj:getId() ~= obj2id then
    electrics.values[electricsName] = attachedValue
  end
end

local function onCouplerDetached(nodeId, obj2id, obj2nodeId, breakForce)
  if obj:getId() ~= obj2id then
    electrics.values[electricsName] = detachedValue
  end
end

local function init(jbeamData)
  electricsName = jbeamData.electricsName or "feet"
  attachedValue = jbeamData.attachedValue or 1
  detachedValue = jbeamData.detachedValue or 0

  local startValue = jbeamData.startValue or 0
  electrics.values[electricsName] = startValue
end

M.init = init
M.onCouplerAttached = onCouplerAttached
M.onCouplerDetached = onCouplerDetached

return M
