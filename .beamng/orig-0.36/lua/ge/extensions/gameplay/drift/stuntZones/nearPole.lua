-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local C = {}

local maxDist = 4

function C:clear()
  if self.activeData.veh then
    self.activeData.veh:delete()
  end
  self:clearMarker()
end

function C:clearMarker()
  if self.marker ~= nil then
    self.marker:clearMarkers()
    self.marker = nil
  end
end

function C:createMarker()
  local markerOffset = vec3(0, 0, 2.5)
  local veh = scenetree.findObjectById(self.activeData.veh:getId())
  if veh then
    local vehPos = veh:getPosition()
    self.marker = require('scenario/race_marker').createRaceMarker(true, "overhead")
    self.marker:setToCheckpoint({pos = vehPos + markerOffset, radius = 5, fadeNear = 1000, fadeFar = 0})
    self.marker:setMode('default')
  end
end

function C:resetData()
  self.activeData = {
    lastFrameHitPole = false,
    veh = nil,
    hitPole = false,
    available = true,
    resetFlag = false,

    lastFramePlDist = math.huge,
    minCornerDist = math.huge,
  }
end

function C:reset()
  local veh
  if self.activeData and self.activeData.veh then
    veh = self.activeData.veh
    self.activeData.veh:requestReset(RESET_PHYSICS)
    self.activeData.veh:resetBrokenFlexMesh()
  end
  self:resetData()

  if veh then self.activeData.veh = veh end

  core_jobsystem.create(function(job)
    job.sleep(0.1)  -- since reset a vehicle takes more than one frame, we need a hack. Otherwise the marker will remain at the pre-reset veh's position

    self:clearMarker()
    self:createMarker()
  end, 1)
end

function C:init(data)
  self.data = data
  self:resetData()

  -- spawn the pole
  local model = "delineator"
  local config = "vehicles/delineator/stripes.pc"
  local options = {config = config, paint = nil, licenseText = nil, vehicleName = generateObjectNameForClass('BeamNGVehicle', 'object_'), pos = self.data.zoneData.pos, rot = quat(0,0,0,1)}
  local spawningOptions = sanitizeVehicleSpawnOptions(model, options)
  spawningOptions.autoEnterVehicle = false
  self.activeData.veh = core_vehicles.spawnNewVehicle(model, spawningOptions)

  self:createMarker()
end

function C:accomplish()
  self:clearMarker()
  self.activeData.available = false

  local driftActiveData = gameplay_drift_drift.getDriftActiveData()

  extensions.hook("onAnyStuntZoneAccomplished", {
    stuntZoneId = self.data.id,
    subHookName = "onNearPoleAccomplished",
    subHookData =
    {
      currDegAngle = driftActiveData and driftActiveData.currDegAngle or 50, -- this safeguard is used in the drift debug imgui menu to test UI
      closeness = 1-(self.activeData.minCornerDist / maxDist),
      zoneData = {points = gameplay_drift_scoring.getStuntZoneBasePoints("nearPole")}
    }
  })
end

local cornerDist
local plDist
local currAngle
local scoredFlag
function C:detectStunt()
  scoredFlag = false
  for _, corner in ipairs(gameplay_drift_drift.getVehCorners()) do
    cornerDist = corner:distance(self.data.zoneData.pos)
    if cornerDist < maxDist then
      plDist = gameplay_drift_drift.getVehPos():distance(self.data.zoneData.pos)
      currAngle = gameplay_drift_drift.getDriftActiveData().currDegAngle

      if cornerDist < self.activeData.minCornerDist then
        self.activeData.minCornerDist  = cornerDist
      end

      if plDist > self.activeData.lastFramePlDist and not self.activeData.hitPole then -- we are moving away, so we score
        scoredFlag = true
        self:clearMarker()
        break
      end

      self.activeData.lastFramePlDist = plDist

    end
  end

  if scoredFlag then
    self:accomplish()
  end
end

function C:onUpdate(dtReal, dtSim)
  if self.marker then
    self.marker:update(dtReal, dtSim)
  end

  self.activeData.lastFrameHitPole = self.activeData.hitPole

  if self.activeData.veh and map then
    local vehMap = map.objects[gameplay_drift_drift.getVehId()]

    if vehMap then
      local cols = vehMap.objectCollisions
      if cols[self.activeData.veh:getId()] then
        self.activeData.hitPole = true
        self:clearMarker()
      end
    end
  end
end

function C:onVehicleDestroyed(vehId)
  if self.activeData.veh and vehId == self.activeData.veh:getId() then
    self.activeData.veh = nil
    gameplay_drift_stuntZones.clearStuntZone(self.data.id) -- make the stuntzone manager remove this stuntzone, not itself
  end
end

function C:isAvailable()
  return self.activeData.available
end


return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end