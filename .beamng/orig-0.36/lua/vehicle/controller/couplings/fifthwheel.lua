-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local collisionCheckTimer = 0
local collisionCheckTime = 1 / 5

local kingpinCache = {}
local kingpinRequestTimeouts = {}
local kingpinTimeoutsToDelete = {}

local fifthwheelNodeCid
local fifthwheelKey
local currentKingpinObjId
local currentKingpinNodeCid
local detachingKingpinObjId
local detachingKingpinNodeCid

local fifthwheelCouplerStrength
local fifthwheelCouplerRadius
local fifthwheelCouplerLockRadius
local fifthWheelCouplerLatchSpeed

local fithwheelAttachSoundEvent
local fithwheelDetachSoundEvent

local fithwheelAttachSoundVolume
local fithwheelDetachSoundVolume

local attachmentStateElectricsName

local couplerState = {
  attached = "attached",
  attaching = "attaching",
  detaching = "detaching",
  detached = "detached"
}
local state = couplerState.detached

local isCollidingWithLookup = {}

local function debugDrawMethod(focusPos)
  obj.debugDrawProxy:drawNodeSphere(fifthwheelNodeCid, 0.15, getContrastColor(stringHash(fifthwheelKey), 150))
end

local function setFifthwheelIndicatorVisibility(visible)
  M.debugDraw = visible and debugDrawMethod or nil
  controller.cacheAllControllerFunctions()
  local cmdString = "for _,kingpin in ipairs(controller.getControllersByType('couplings/kingpin')) do kingpin.setKingpinVisibility(%q,%s) end"
  BeamEngine:queueAllObjectLua(string.format(cmdString, fifthwheelKey, visible))
end

local function toggleFifthwheelIndicatorVisibility()
  setFifthwheelIndicatorVisibility(M.debugDraw == nil)
end

local function detachFifthwheel()
  obj:detachCoupler(fifthwheelNodeCid, 0)
end

local function cacheKingpinData(objId, nodeId, distance)
  kingpinCache[objId] = kingpinCache[objId] or {}
  table.clear(kingpinCache[objId])
  kingpinCache[objId][nodeId] = kingpinCache[objId][nodeId] or {}
  kingpinCache[objId][nodeId].distance = distance
end

local function kingpinDataCallback(obj2Id, data)
  local position = obj:getPosition()
  local nodePostion = obj:getNodePosition(fifthwheelNodeCid)
  local distance = ((position + nodePostion) - data.nodePosition):length()
  kingpinRequestTimeouts[obj2Id] = nil
  cacheKingpinData(obj2Id, data.nodeId, distance)
end

local function requestKingpinData(obj2Id)
  --print("request kingpin data for id: " .. obj2Id)
  local kingpinCmd = string.format([[
        local kingpins = controller.getControllersByType("couplings/kingpin")
        for _, kingpin in ipairs(kingpins) do
          kingpin.sendDataToVehicle(%d, %q, %q)
        end
      ]], objectId, M.name, "fifthwheel_v2")
  obj:queueObjectLuaCommand(obj2Id, kingpinCmd)
  kingpinRequestTimeouts[obj2Id] = kingpinRequestTimeouts[obj2Id] or 1
end

local function updateGFX(dt)
  --print(state)
  --check what vehicles we are colliding with
  table.clear(isCollidingWithLookup)
  for _, id in ipairs(mapmgr.objectCollisionIds) do
    isCollidingWithLookup[id] = true
  end

  --check our kingpin request timeouts
  table.clear(kingpinTimeoutsToDelete)
  for obj2Id, _ in pairs(kingpinRequestTimeouts) do
    kingpinRequestTimeouts[obj2Id] = kingpinRequestTimeouts[obj2Id] - dt
    if kingpinRequestTimeouts[obj2Id] <= 0 then
      table.insert(kingpinTimeoutsToDelete, obj2Id)
      kingpinCache[obj2Id] = nil
    end
  end
  --delete expired timeouts
  for _, timeout in ipairs(kingpinTimeoutsToDelete) do
    kingpinRequestTimeouts[timeout] = nil
  end

  if state ~= couplerState.attached then
    collisionCheckTimer = collisionCheckTimer - dt
    if collisionCheckTimer <= 0 then
      for _, id in ipairs(mapmgr.objectCollisionIds) do
        requestKingpinData(id)
      end

      collisionCheckTimer = collisionCheckTimer + collisionCheckTime
    end
  else
    collisionCheckTimer = 0
  end

  if state == couplerState.detaching then
    if not isCollidingWithLookup[detachingKingpinObjId] and detachingKingpinObjId then
      requestKingpinData(detachingKingpinObjId)
    end
    --dump(kingpinCache[detachingKingpinObjId])
    if not detachingKingpinObjId or not detachingKingpinNodeCid or not kingpinCache[detachingKingpinObjId] or not kingpinCache[detachingKingpinObjId][detachingKingpinNodeCid] or kingpinCache[detachingKingpinObjId][detachingKingpinNodeCid].distance > 0.6 then
      --print("fully detached")
      state = couplerState.detached
      detachingKingpinObjId = nil
      detachingKingpinNodeCid = nil
    end
  end

  if state == couplerState.detached then
    local minDistance = math.huge
    local minDistanceObjId
    local minDistanceNodeCid
    for objId, nodeData in pairs(kingpinCache) do
      for nodeCid, data in pairs(nodeData) do
        if data.distance < minDistance then
          minDistanceObjId = objId
          minDistanceNodeCid = nodeCid
          minDistance = data.distance
        end
      end
    end

    if minDistanceObjId and minDistanceNodeCid and minDistance < fifthwheelCouplerRadius * 0.8 then
      --print("attach")
      obj:attachExternalCoupler(fifthwheelNodeCid, minDistanceObjId, minDistanceNodeCid, fifthwheelCouplerStrength, fifthwheelCouplerRadius, fifthwheelCouplerLockRadius, fifthWheelCouplerLatchSpeed, 0)
      state = couplerState.attaching
    end
  end

  electrics.values[attachmentStateElectricsName] = state == couplerState.attached and 1 or 0
end

local function onCouplerFound(nodeId, obj2id, obj2nodeId)
  --print("found")
end

local function onCouplerAttached(nodeId, obj2id, obj2nodeId, attachForce)
  if nodeId == fifthwheelNodeCid then
    --print("fully attached")
    state = couplerState.attached
    currentKingpinObjId = obj2id
    currentKingpinNodeCid = obj2nodeId
    local aggressionCoef = linearScale(attachForce, 0.1, 1, 0, 1)
    if fithwheelAttachSoundEvent then
      obj:playSFXOnceCT(fithwheelAttachSoundEvent, nodeId, fithwheelAttachSoundVolume, 0.5, aggressionCoef, 0)
    end
    extensions.couplings.couplingAttached(nodeId, obj2id, obj2nodeId)
  end

  setFifthwheelIndicatorVisibility(false)
end

local function onCouplerDetached(nodeId, obj2id, obj2nodeId, breakForce)
  --print(breakForce)
  if nodeId == fifthwheelNodeCid then
    state = couplerState.detaching
    detachingKingpinObjId = currentKingpinObjId
    detachingKingpinNodeCid = currentKingpinNodeCid
    currentKingpinObjId = nil
    currentKingpinNodeCid = nil
    if fithwheelDetachSoundEvent then
      obj:playSFXOnceCT(fithwheelDetachSoundEvent, nodeId, fithwheelDetachSoundVolume, 0.5, 0, 0)
    end
    extensions.couplings.couplingDetached(nodeId, obj2id, obj2nodeId)
  end
end

local function isAttached()
  return state == couplerState.attached
end

-- local function settingsChanged()
-- end

-- local function resetSounds(jbeamData)
-- end

local function reset(jbeamData)
  state = couplerState.detached
  kingpinCache = {}
  currentKingpinObjId = nil
  currentKingpinNodeCid = nil
  detachingKingpinObjId = nil
  detachingKingpinNodeCid = nil

  kingpinRequestTimeouts = {}
  kingpinTimeoutsToDelete = {}

  electrics.values[attachmentStateElectricsName] = 0
end

local function initSounds(jbeamData)
  fithwheelAttachSoundEvent = jbeamData.attachSoundEvent or "event:>Vehicle>Latches>Special>tailgate_01_close"
  fithwheelDetachSoundEvent = jbeamData.detachSoundEvent or "event:>Vehicle>Latches>Special>tailgate_01_open"
  fithwheelAttachSoundVolume = jbeamData.attachSoundEventVolume or 0.5
  fithwheelDetachSoundVolume = jbeamData.detachSoundEventVolume or 0.5
end

local function init(jbeamData)
  local fifthwheelNodeName = jbeamData.fifthwheelNode
  fifthwheelNodeCid = beamstate.nodeNameMap[fifthwheelNodeName]
  fifthwheelKey = jbeamData.fifthwheelKey or "fifthwheel_v2"

  fifthwheelCouplerStrength = v.data.nodes[fifthwheelNodeCid].couplerStrength or 10000000
  fifthwheelCouplerRadius = v.data.nodes[fifthwheelNodeCid].couplerRadius or 0.5
  fifthwheelCouplerLockRadius = v.data.nodes[fifthwheelNodeCid].couplerLockRadius or 0.025
  fifthWheelCouplerLatchSpeed = v.data.nodes[fifthwheelNodeCid].couplerLatchSpeed or 0.3

  attachmentStateElectricsName = jbeamData.attachmentStateElectricsName or (M.name .. "_attachmentState")

  state = couplerState.detached
  kingpinCache = {}
  currentKingpinObjId = nil
  currentKingpinNodeCid = nil
  detachingKingpinObjId = nil
  detachingKingpinNodeCid = nil

  electrics.values[attachmentStateElectricsName] = 0
end

M.init = init
M.initSounds = initSounds

M.reset = reset
--M.resetSounds = resetSounds

M.updateGFX = updateGFX

M.debugDraw = nil

M.onCouplerFound = onCouplerFound
M.onCouplerAttached = onCouplerAttached
M.onCouplerDetached = onCouplerDetached

M.isAttached = isAttached

M.kingpinDataCallback = kingpinDataCallback
M.detachFifthwheel = detachFifthwheel

M.setFifthwheelIndicatorVisibility = setFifthwheelIndicatorVisibility
M.toggleFifthwheelIndicatorVisibility = toggleFifthwheelIndicatorVisibility

return M
