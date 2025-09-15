--[[
This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
If a copy of the bCDDL was not distributed with this
file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
This module contains a set of functions which manipulate behaviours of vehicles.
]]

local M = {}
local mrad = math.rad

local function process(objID, vehicleObj, vehicle)
  profilerPushEvent('jbeam/mirror.process')

  if vehicle.mirrors ~= nil then
    for _, v in pairs(vehicle.mirrors) do
      if v.mesh then
        v.id = vehicleObj:addMirror(v.mesh,v.idRef or -1, v.id1 or -1, v.id2 or -1)
        if v.id < 0 then goto continue end
        local mirror = vehicleObj:getMirror(v.id)
        if not mirror then log("E","proc","getMirror failed!"); goto continue end
        if v.refBaseTranslation then
          mirror.offset = v.refBaseTranslation
        elseif v.offset then
          -- log("E","proc","Migrate "..dumps(v.mesh) .." baseTranslationGlobal="..dumps(v.offset + vehicle.nodes[v.idRef].pos) )
          mirror.offset = v.offset
        end
        if v.baseRotationGlobal then
          local q = quatFromEuler(mrad(v.baseRotationGlobal.x),mrad(v.baseRotationGlobal.y),mrad(v.baseRotationGlobal.z))
          mirror.normal = vec3(0,1,0):rotated(q)
          -- log("I","proc","Migrated "..dumps(v.mesh) .." baseRotationGlobal="..dumps(mirror.normal) )
        elseif v.normal then
          local q = vec3(0,1,0):getRotationTo(vec3(v.normal))
          local r = q:toEulerYXZ()
          log("E","proc","Migrate "..dumps(v.mesh) .." baseRotationGlobal="..dumps(math.deg(r.y)).."|"..dumps(math.deg(r.z)).."|"..dumps(math.deg(r.x)) )
          mirror.normal = v.normal
        end
        if v.offsetRotationGlobal then
          log("E","proc","offsetRotationGlobal is depracted, fix mesh normals "..dumps(v.mesh))
          local q = quatFromEuler(mrad(v.offsetRotationGlobal.x),mrad(v.offsetRotationGlobal.y),mrad(v.offsetRotationGlobal.z))
          mirror.offsetNormal = vec3(0,1,0):rotated(q)
          -- log("I","proc","Migrated "..dumps(v.mesh) .." offsetRotationGlobal="..dumps(mirror.offsetNormal) )
        elseif v.offsetNormal then
          log("E","proc","offsetNormal is depracted, fix mesh normals "..dumps(v.mesh))
          local q = vec3(0,1,0):getRotationTo(vec3(v.offsetNormal))
          local r = q:toEulerYXZ()
          log("E","proc","Migrate "..dumps(v.mesh) .." offsetRotationGlobal="..dumps(math.deg(r.y)).."|"..dumps(math.deg(r.z)).."|"..dumps(math.deg(r.x)) )
          mirror.offsetNormal = v.offsetNormal
        end
      else
        log("E","proc","probably old data!!!")
        dump(v)
      end
      ::continue::
    end
  end

  profilerPopEvent('jbeam/mirror.process')
end

M.process = process

return M