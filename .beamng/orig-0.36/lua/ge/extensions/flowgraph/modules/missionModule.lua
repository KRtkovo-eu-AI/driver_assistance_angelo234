-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local C = {}
C.moduleOrder = -100 -- low first, high later

function C:init()
  self:clear()
end

function C:clear()
  self.processed = false
end

function C:processVehicles(params)
  local mission = self.mgr.activity
  if not mission then return end

  if self.processed then
    log("W", "", "Already processed mission vehicles!")
    return
  end
  if not params or not next(params) then return end

  if mission.setupData._hasStashedVehicles then
    log("I", "", "Mission manager has stashed vehicles, now using stash params for backwards compatibility")
  end

  -- here is some compatibility provided that may reactivate the player vehicle and traffic by using the params

  -- player vehicle
  local playerId = self:getOriginalPlayerId()
  if params.keepPlayer ~= nil and playerId then
    getObjectByID(playerId):setActive(params.keepPlayer and 1 or 0)
    mission.setupData.stashedVehicles[playerId] = not params.keepPlayer
    if params.keepPlayer then
      log("I", "", "Keeping player vehicle for mission")
    else
      log("I", "", "Hiding player vehicle for mission")
    end
  end

  -- traffic
  if params.keepTraffic ~= nil and (mission.setupModules.traffic.prevTraffic or mission.setupModules.traffic.prevParking) then
    if params.keepTraffic then
      gameplay_traffic.unfreezeState(mission.setupModules.traffic.prevTraffic, mission.setupModules.traffic.prevParking)
      mission.setupModules.traffic.prevTraffic, mission.setupModules.traffic.prevParking = nil, nil
      mission.setupModules.traffic.usePrevTraffic = true
    end
    if params.keepTraffic then
      log("I", "", "Keeping traffic for mission")
    else
      log("I", "", "Hiding traffic for mission")
    end
  end

  self.processed = true
end

function C:prepareVehicle(id)
  id = id or be:getPlayerVehicleID(0)
  local obj = getObjectByID(id)
  if obj then
    local tod = core_environment.getTimeOfDay()
    if tod and tod.time >= 0.225 and tod.time <= 0.775 then
      obj:queueLuaCommand("electrics.setLightsState(2)")
    end

    -- more actions can be set here depending on mission environment
  end
end

function C:getOriginalPlayerId()
  if self.mgr.activity and self.mgr.activity._startingInfo then
    return self.mgr.activity._startingInfo.vehId
  end
end

function C:removeStashedPlayerVehicle()
  local playerId = self:getOriginalPlayerId()
  if not playerId then return end

  self.mgr:logEvent("Removing stashed player vehicle", "I", "The stashed player vehicle will no longer be reactivated at the end of the project.")
  local pv = getObjectByID(playerId)
  if pv then
    if editor and editor.onRemoveSceneTreeObjects then
      editor.onRemoveSceneTreeObjects({playerId})
    end
    pv:delete()
  end
end

function C:missionHook(name, data)
  if self.mgr.activity.onMissionCustomEvent then
    self.mgr.activity:onMissionCustomEvent(name, data)
    self.mgr:logEvent("Mission hook trigger", "I", "Mission hook triggered with event: "..name)
  end
end

function C:executionStopped()
  local mission = self.mgr.activity
  if not mission then return end
  if mission._isOngoing then
    gameplay_missions_missionManager.stop(mission)
  end
  extensions.hook("onMissionFinished")
  guihooks.trigger('hotlappingReevaluateControlsEnabled')
end

function C:onClear()
end

function C:executionStarted()
  guihooks.trigger('hotlappingReevaluateControlsEnabled')
  self:clear()
end


return _flowgraph_createModule(C)