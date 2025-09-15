-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local kingpinNodeCid
local kingpinKey

local function sendDataToVehicle(objId, controllerName, couplerTag)
  local hasMatchingCoupler = true
  if hasMatchingCoupler then
    local position = obj:getPosition()
    local nodePosition = obj:getNodePosition(kingpinNodeCid)
    local data = {nodeId = kingpinNodeCid, nodePosition = position + nodePosition}
    local fifthwheelCmd = string.format([[
        controller.getControllerSafe(%q).kingpinDataCallback(%d, %s)
      ]], controllerName, objectId, serialize(data))
    --dump(fifthwheelCmd)
    obj:queueObjectLuaCommand(objId, fifthwheelCmd)
  end
end

local function debugDrawMethod(focusPos)
  obj.debugDrawProxy:drawNodeSphere(kingpinNodeCid, 0.15, getContrastColor(stringHash(kingpinKey), 150))
end

local function setKingpinVisibility(key, visible)
  if kingpinKey ~= key then
    return
  end
  M.debugDraw = visible and debugDrawMethod or nil
  controller.cacheAllControllerFunctions()
end

local function reset(jbeamData)
end

local function init(jbeamData)
  local kingpinNodeName = jbeamData.kingpinNode
  kingpinNodeCid = beamstate.nodeNameMap[kingpinNodeName]
  kingpinKey = jbeamData.kingpinKey or "fifthwheel_v2"
end

M.init = init
M.reset = reset

M.sendDataToVehicle = sendDataToVehicle
M.setKingpinVisibility = setKingpinVisibility
M.debugDraw = nil

return M
