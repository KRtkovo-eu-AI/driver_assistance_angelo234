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
    totalDriftAngle = 0, -- donut counting will use that
    completedPerc = 0,
  }
end

function C:init(data)
  self.data = data
  self:reset()
end

function C:accomplish()
  self.activeData.totalDriftAngle = 0
  self.activeData.currCooldown = self.data.zoneData.cooldown

  extensions.hook("onAnyStuntZoneAccomplished", {
    stuntZoneId = self.data.id,
    subHookName = "onDonutDriftAccomplished",
    subHookData =
    {
      zoneData = {points = gameplay_drift_scoring.getStuntZoneBasePoints("donut")}
    }
  })
end

function C:detectStunt()
  self.activeData.totalDriftAngle = self.activeData.totalDriftAngle + gameplay_drift_drift.getAngleDiff()
  if gameplay_drift_general.getGeneralDebug() then
    debugDrawer:drawTextAdvanced(self.data.zoneData.pos, self.activeData.totalDriftAngle, ColorF(1,1,1,1), true, false, ColorI(0,0,0,255))
  end
  if self.activeData.totalDriftAngle >= 360 then
    self:accomplish()
  end
end

function C:isAvailable()
  return self.activeData.currCooldown <= 0
end

function C:isPlayerInside()
  local isInside = gameplay_drift_drift.getVehPos():distance(self.data.zoneData.pos) < self.data.zoneData.scl
  if not isInside then
    self.activeData.totalDriftAngle = 0
  end
  return isInside
end

function C:onUpdate()
  self:sendDecals()
end

local lastPos = vec3()
local pos = vec3()
local color
function C:sendDecals()
  if not gameplay_drift_stuntZones.getDrawLines() then return end

  filledPerc = self.activeData.totalDriftAngle / 360 * 100
  cooldownPerc = 100 - self.activeData.currCooldown / self.data.zoneData.cooldown * 100

  decalCount = self.data.drawData.decalCount
  invAmount = 1/decalCount

  for i = 0, decalCount do
    t = i*invAmount

    cosX = math.cos(math.rad(t * 360))
    sinY = math.sin(math.rad(t * 360))

    x = (self.data.zoneData.pos.x or 0) + ((self.data.zoneData.scl or 1) * cosX)
    y = (self.data.zoneData.pos.y or 0) + ((self.data.zoneData.scl or 1) * sinY)

    pos = vec3(x,y,self.data.zoneData.pos.z or 0)
    color = gameplay_drift_stuntZones.getDecalColor(cooldownPerc, filledPerc, t)

    if i > 0 then
      debugDrawer:drawLineInstance(pos, lastPos, gameplay_drift_stuntZones.getLineThickness(pos), color)
    end
    lastPos:set(pos)

  end
end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end