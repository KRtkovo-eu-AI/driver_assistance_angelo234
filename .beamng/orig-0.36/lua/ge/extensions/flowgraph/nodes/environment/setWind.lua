-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im = ui_imgui

local C = {}

C.name = 'Set Wind'
C.icon = "simobject_scatter_sky"
C.description = "Sets wind speed (either per vehicle or for all vehicles)."
C.category = 'once_p_duration'

C.pinSchema = {
  { dir = 'in', type = 'number', name = 'vehId', description = "Single vehicle id to apply wind for; leave this blank to use all vehicles." },
  { dir = 'in', type = 'number', name = 'windSpeed', description = "Wind speed, in metres per second." },
  { dir = 'in', type = 'vec3', name = 'windDir', description = "Wind direction vector." }
}

C.tags = {'environment', 'wind'}

function C:init()
  self:onNodeReset()
end

function C:_executionStopped()
  self:onNodeReset()
end

function C:onNodeReset()
  for _, veh in ipairs(getAllVehicles()) do
    veh:queueLuaCommand('obj:setWind(0, 0, 0)')
  end
end

local windVec = vec3()
function C:work()
  local windSpeed = self.pinIn.windSpeed.value or 0
  windVec:setFromTable(self.pinIn.windDir.value or {1, 0, 0})
  windVec:setScaled(windSpeed)
  for _, veh in ipairs(getAllVehicles()) do
    if not self.pinIn.vehId.value or self.pinIn.vehId.value == veh:getId() then
      veh:queueLuaCommand('obj:setWind('..string.format('%6f, %6f, %6f', windVec.x, windVec.y, windVec.z)..')')
    end
  end
end

return _flowgraph_createNode(C)
