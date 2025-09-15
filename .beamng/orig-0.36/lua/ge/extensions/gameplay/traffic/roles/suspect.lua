-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local C = {}

function C:init()
  self.personalityModifiers = {
    aggression = {offset = 0.2}
  }
  self.actions = {
    watchPolice = function ()
      self.state = 'wanted'
      self.flags.flee = nil
    end,
    fleePolice = function ()
      if self.veh.isAi then
        self.veh:setAiMode('flee')
        getObjectByID(self.veh.id):queueLuaCommand('controller.setFreeze(0)')
        getObjectByID(self.veh.id):queueLuaCommand('ai.setAggressionMode("off")')
        getObjectByID(self.veh.id):queueLuaCommand('ai.driveInLane("off")')
        getObjectByID(self.veh.id):queueLuaCommand('ai.setSpeedMode("off")')
        self:setAggression(0.8)
      end
      self.veh:modifyRespawnValues(800, 40) -- this strongly keeps the vehicle from respawning
      self.state = 'flee'
      self.flags.flee = 1
      self.flags.busy = 1
    end,
    arrest = function ()
      getObjectByID(self.veh.id):queueLuaCommand('controller.setFreeze(1)')
      self.flags.freeze = 1
      self.state = 'stop'
    end,
    clear = function ()
      getObjectByID(self.veh.id):queueLuaCommand('controller.setFreeze(0)')
      self.flags.freeze = nil
      self.flags.flee = nil
      self.flags.busy = nil
      self.state = 'none'
    end
  }

  self.pursuitMode = 2 -- default pursuit mode
end

function C:onRoleEnded()
  if gameplay_traffic.showMessages and be:getPlayerVehicleID(0) == self.targetId then
    gameplay_police.evadeVehicle(self.veh.id, true)
  end
end

function C:onTrafficTick(tickTime)
  local sightThreshold = self.veh.isAi and 0.25 or 1
  if self.state == 'wanted' and self.veh.pursuit.sightValue >= sightThreshold then
    local policeIds = {}
    for id, veh in pairs(gameplay_police.getPoliceVehicles()) do
      if not veh.role.flags.pursuit and not veh.role.flags.reset then -- only available police vehicles
        table.insert(policeIds, id)
      end
    end

    if policeIds[1] then
      gameplay_police.setPursuitMode(self.pursuitMode, self.veh.id, policeIds) -- police start chasing the wanted suspect
      local bestId = gameplay_police.getNearestPoliceVehicle(self.veh.id, true, true)
      self:setTarget(bestId or policeIds[1])
    end
  end
end

function C:onUpdate(dt, dtSim)
end

return function(...) return require('/lua/ge/extensions/gameplay/traffic/baseRole')(C, ...) end