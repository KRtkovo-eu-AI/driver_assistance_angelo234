-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

--- you can use this to turn of Just In Time compilation for debugging purposes:
vmType = 'objectpool'

package.path = 'lua/ge/?.lua;lua/gui/?.lua;lua/vehicle/?.lua;lua/common/?.lua;lua/common/libs/?/init.lua;lua/common/libs/luasocket/?.lua;lua/?.lua;core/scripts/?.lua;scripts/?.lua;?.lua'
package.cpath = ''
require('luaCore')

require("utils")
require("devUtils")
require("ve_utils")
require("mathlib")
float3 = vec3

print = function(...)
  log("A", "print", tostring(...))
end

local objPool = {}

function getObject(objId)
  return lpack.encode(objPool[objId])
end

function moveObject(objId)
  dump{'moveObject: ', objId}
  local objData = lpack.encode(objPool[objId])
  objPool[objId] = nil
  return objData
end

function setObject(objId, objDataStr)
  dump{'setObject: ', objId}
  objPool[objId] = lpack.decode(objDataStr)
end

function deleteObject(objId)
  objPool[objId] = nil
end

-- thread Lua VM
function initPool()
end

-- called in object Lua VM
function init(path, initData)
  local object
  local v = require("jbeam/stage2")
  if type(initData) == "string" and string.len(initData) > 0 then
    local state, initData = pcall(lpack.decode, initData)
    if state and type(initData) == "table" then
      if initData.vdata then
        object = v.loadVehicleStage2(initData)
      else
        log("E", "object", "unable to load object: invalid spawn data")
      end
    end
  else
    log("E", "object", "invalid initData: " .. tostring(type(initData)) .. ": " .. tostring(initData))
  end

  if not object then
    log("E", "loader", "object loading failed fatally")
    return false -- return false = unload lua
  end

  objPool[obj:getId()] = { v = object }
  return true
end

function onNodeCollision(id1, pos, normal, nodeVel, perpendicularVel, slipVec, slipVel, slipForce, normalForce, depth, materialId1, materialId2)
  --dump({"Node collision with object:", obj:getId(), counter})
end

function onVehicleReset(retainDebug)
end

function onBeamBroke(id, energy)
end

function onBeamDeformed(id, ratio)
end

-- Management machinery
objData = {}

function _setObj(objId, _obj)
  rawset(_G, 'obj', _obj)
  rawset(_G, 'objectId', objId)
  rawset(_G, 'objData', objPool[objId])
end

function __newIndexHandler(t, key, val)
  rawset(rawget(_G, 'objData'), key, val)
end

function __indexHandler(t, key)
  return rawget(rawget(_G, 'objData'), key)
end

setmetatable(_G, {
  __newindex = __newIndexHandler,
  __index = __indexHandler,
})
