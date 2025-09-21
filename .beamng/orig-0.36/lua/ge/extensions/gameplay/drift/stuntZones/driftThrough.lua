-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local C = {}

local decalCount
local defaultDecalScale = vec3(0.35,0.35,3)
local t, cosX, sinY, x, y, data, invAmount, filledPerc, cooldownPerc


function C:reset()
  self.activeData = {
    isActive = false,
    currCooldown = 0,
    usedFlag = false,
  }
end

function C:init(data)
  self.data = data
  self:reset()
end

function C:accomplish()
  self.activeData.usedFlag = false
  self.activeData.currCooldown = self.data.zoneData.cooldown

  local driftActiveData = gameplay_drift_drift.getDriftActiveData()

  extensions.hook("onAnyStuntZoneAccomplished", {
    stuntZoneId = self.data.id,
    subHookName = "onDriftThroughAccomplished",
    subHookData =
    {
      currDegAngle = driftActiveData and driftActiveData.currDegAngle or 30, -- this safeguard is used in the drift debug imgui menu to test UI
      zoneData = {points = gameplay_drift_scoring.getStuntZoneBasePoints("driftThrough")}
    }
  })
end

function C:detectStunt()
  if self.activeData.usedFlag then
    self:accomplish()
  end
end

function C:isAvailable()
  return self.activeData.currCooldown <= 0
end

function C:isPlayerInside()
  local isInside = containsOBB_point(self.data.zoneData.pos, self.data.x, self.data.y, self.data.z, gameplay_drift_drift.getVehPos())
  if not isInside then
    self.activeData.usedFlag = true
  end
  return isInside
end

function C:onUpdate()
  self:sendDecals()
end

local a, b
local fwd = vec3()
local lerpVecA = vec3()
local lerpVecB = vec3()
local lerpVecC = vec3()
local lerpVecD = vec3()
local color
function C:sendDecals()
  if not gameplay_drift_stuntZones.getDrawLines() then return end

  cooldownPerc = 100 - self.activeData.currCooldown / self.data.zoneData.cooldown * 100

  if cooldownPerc >= 100 then
    debugDrawer:drawLineInstance(self.data.drawData.pointA, self.data.drawData.pointB, gameplay_drift_stuntZones.getLineThickness(self.data.drawData.pointA), gameplay_drift_stuntZones.getGreenColor())
    debugDrawer:drawLineInstance(self.data.drawData.pointC, self.data.drawData.pointD, gameplay_drift_stuntZones.getLineThickness(self.data.drawData.pointC), gameplay_drift_stuntZones.getGreenColor())
  else
    lerpVecA = lerp(self.data.drawData.pointA, self.data.drawData.pointB, cooldownPerc / 100)
    lerpVecB = lerp(self.data.drawData.pointC, self.data.drawData.pointD, cooldownPerc / 100)
    debugDrawer:drawLineInstance(self.data.drawData.pointA, lerpVecA, gameplay_drift_stuntZones.getLineThickness(lerpVecA), gameplay_drift_stuntZones.getWhiteColor())
    debugDrawer:drawLineInstance(self.data.drawData.pointC, lerpVecB, gameplay_drift_stuntZones.getLineThickness(lerpVecB), gameplay_drift_stuntZones.getWhiteColor())

    debugDrawer:drawLineInstance(self.data.drawData.pointB, lerpVecA, gameplay_drift_stuntZones.getLineThickness(lerpVecA), gameplay_drift_stuntZones.getRedColor())
    debugDrawer:drawLineInstance(self.data.drawData.pointD, lerpVecB, gameplay_drift_stuntZones.getLineThickness(lerpVecB), gameplay_drift_stuntZones.getRedColor())
  end
end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end