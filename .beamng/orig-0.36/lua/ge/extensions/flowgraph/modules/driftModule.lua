-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local C = {}
C.moduleOrder = 0 -- low first, high later
C.hooks = {'onDriftSpinout', 'onDriftCrash', 'onDonutDriftScored', 'onTightDriftScored', 'onDriftCompletedScored'}
C.dependencies = {'gameplay_drift_general'}

function C:resetModule()
  self.callbacks = {
    tight = {ttl = 0},
    donut = {ttl = 0},
    spinout = {ttl = 0},
    crash = {ttl = 0},
    tap = {ttl = 0},
    scored = {ttl = 0},
  }
end

function C:init()
  self:resetModule()
end

function C:onUpdate()
  for _, callbackData in pairs(self.callbacks) do
    if callbackData.ttl > 0 then
      callbackData.ttl = callbackData.ttl - 1
    end
  end
end


function C:getCallBacks()
  return self.callbacks
end

function C:addCallback(name, data)
  self.callbacks[name] = {
    ttl = 2,
    data = data
  }
end

function C:onDriftSpinout()
  self:addCallback("spinout")
end

function C:onDriftCrash()
  self:addCallback("crash")
end

function C:onTightDriftScored(score)
  self:addCallback("tight", {score = score})
end

function C:onDonutDriftScored(score)
  self:addCallback("donut", {score = score})
end

function C:onDriftCompletedScored(data)
  self:addCallback("scored", {addedScore = data.addedScore, cachedScore = data.cachedScore, combo = data.combo})
end

function C:onDonutZoneReactivated()
  gameplay_drift_drift.onDonutZoneReactivated()
end

return _flowgraph_createModule(C)