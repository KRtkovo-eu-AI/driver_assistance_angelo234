-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui
local C = {}

C.name = 'On Crash Detected'

C.description = 'Trigger a flow when a tracked vehicle starts or ends a crash.'
C.color = ui_flowgraph_editor.nodeColors.crash
C.category = 'repeat_instant'
C.tags = {"crash", "vehicle", "track", "crashDetection"}
C.pinSchema = {
  { dir = 'out', type = 'flow', impulse = true, name = 'startCrash', description = "The beginning of the crash." },
  { dir = 'out', type = 'flow', impulse = true, name = 'endCrash', description = "The end of the crash." },
  { dir = 'out', type = 'number', name = 'vehId', description = "The id of the vehicle that crashed." },
}

function C:init()
  self:reset()
end

function C:_executionStopped()
  self:reset()
end

function C:reset()
  self.crashStartQueue = {} -- Queue system for multiple crash at the same frame
  self.crashEndQueue = {}
end

function C:work()
  if next(self.crashStartQueue) then
    local vehId = table.remove(self.crashStartQueue, 1)
    self.pinOut.startCrash.value = true
    self.pinOut.vehId.value = vehId
  else
    self.pinOut.startCrash.value = false
  end

  if next(self.crashEndQueue) then
    local vehId = table.remove(self.crashEndQueue, 1)
    self.pinOut.endCrash.value = true
    self.pinOut.vehId.value = vehId
  else
    self.pinOut.endCrash.value = false
  end
end

function C:onVehicleCrashStarted(crashStartData)
  table.insert(self.crashStartQueue, crashStartData.vehId)
end

function C:onVehicleCrashEnded(crashEndData)
  table.insert(self.crashEndQueue, crashEndData.vehId)
end

return _flowgraph_createNode(C)
