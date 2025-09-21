-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local logTag = ''
local rallyUtil = require('/lua/ge/extensions/gameplay/rally/util')

local C = {}

local steeringKey = 'steering'

function C:init(vehicle, missionDir)
  log('I', logTag, 'initializing cutCapture for vehicle='..vehicle:getId())
  self.vehicle = vehicle
  core_vehicleBridge.registerValueChangeNotification(self.vehicle, steeringKey)
  self.fname = rallyUtil.missionReccePath(missionDir, 'cuts.json')
  self.fname_transcripts = rallyUtil.missionReccePath(missionDir, 'transcripts.json')
  self.cut_id = 1
  log('I', logTag, 'cutCapture init fname='..self.fname)
end

function C:truncateCapturesFile()
  self.f = io.open(self.fname, "w")
  self.f:close()
end

function C:truncateTranscriptsFile()
  self.f = io.open(self.fname_transcripts, "w")
  self.f:close()
end

function C:writeCut(cutCapture)
  log('I', logTag, 'writeCut')
  self.f = io.open(self.fname, "a")
  local content = jsonEncode(cutCapture)
  self.f:write(content.."\n")
  self.f:close()
end

function C:capture()
  -- pos appears to be centered over the top center of the windshield.
  local vehPos = self.vehicle:getPosition()
  local vehRot = quatFromDir(self.vehicle:getDirectionVector(), self.vehicle:getDirectionVectorUp())
  local now = rallyUtil.getTime()

  local vInfo = {
    id = self.cut_id,
    ts = now,
    pos = { x = vehPos.x, y = vehPos.y, z = vehPos.z },
    quat = {x = vehRot.x, y = vehRot.y, z = vehRot.z, w = vehRot.w},
    steering = nil,
  }
  self.cut_id = self.cut_id + 1

  local steering = core_vehicleBridge.getCachedVehicleData(self.vehicle:getId(), steeringKey)
  if steering ~= nil then
    vInfo.steering = steering
  end

  self:writeCut(vInfo)
  return vInfo.id
end

-- function C:drawDebug()
--   -- local debugPos = self.last_time_pos
--   local debugPos = self.last_dist_pos
--
--   if debugPos then
--     debugDrawer:drawSphere(
--       debugPos,
--       4,
--       ColorF(0,1,1,0.5)
--     )
--   end
-- end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end

