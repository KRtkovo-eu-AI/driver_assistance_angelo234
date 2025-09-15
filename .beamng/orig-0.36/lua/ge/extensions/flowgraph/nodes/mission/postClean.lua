-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im = ui_imgui

local C = {}

C.name = 'Mission Post Clean'
C.color = im.ImVec4(0.13, 0.3, 0.64, 0.75)
C.description = "Cleans up after a mission."
C.category = 'once_p_duration'

C.pinSchema = {
  { dir = 'in', type = 'bool', name = 'keepVeh', description = 'If set, removes stashed player vehicle, keeps the vehicle given by vehId (or player vehicle), unfreezes it.' },
  { dir = 'in', type = 'bool', name = 'recoverInPlace', description = 'If set, also recovers the vehicle to keep in place.' , hidden=true, hardcoded=true,},
  { dir = 'in', type = 'bool', name = 'resetStartPos', description = 'If set, moved the players original vehicle back to where the mission was started, if that vehicle exists. Movement happens after the mission is stopped.' , hidden=true, hardcoded=true,},
  { dir = 'in', type = 'number', name = 'vehId', description = 'Defines the id of the vehicle to keep, uses current player vehicle Id if none given', hidden=true },
}
C.tags = { 'activity' }

function C:workOnce()
  local mission = self.mgr.activity
  if not mission then return end

  if self.pinIn.keepVeh.value then
    local veh
    if self.pinIn.vehId.value then
      veh = scenetree.findObjectById(self.pinIn.vehId.value)
    else
      veh = getPlayerVehicle(0)
    end
    local id = veh:getId()
    local playerId = self.mgr.modules.mission:getOriginalPlayerId()
    if id ~= playerId then
      self.mgr.modules.vehicle:setKeepVehicle(id, true)
      self.mgr.modules.mission:removeStashedPlayerVehicle()
    end

    if self.pinIn.recoverInPlace.value then
      local doRepair = true
      -- skip repair if the player uses their own vehicle in career
      if career_modules_inventory and career_modules_inventory.getCurrentVehicle() then
        doRepair = false
      end

      spawn.safeTeleport(veh, veh:getPosition(), quatFromDir(veh:getDirectionVector(), veh:getDirectionVectorUp()), nil, nil, nil, nil, doRepair)
    end

    core_vehicleBridge.executeAction(veh, 'setFreeze', false)
  end
  if self.pinIn.resetStartPos.value then
    mission.restoreStartingInfoSetup = true
  end

  if mission.setupModules.traffic.usePrevTraffic then -- prevents traffic from deactivating
    self.mgr.modules.traffic.keepTrafficState = true
  end
end

return _flowgraph_createNode(C)