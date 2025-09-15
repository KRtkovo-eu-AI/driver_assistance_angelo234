-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im = ui_imgui

local C = {}

C.name = 'Activity Hook Trigger'
C.color = im.ImVec4(0.03,0.41,0.64,0.75)
C.description = "Triggers a hook with the given event name for an activity, for advanced usage."
C.todo = "This node is generally unused."
C.category = 'once_instant'

C.pinSchema = {}
C.tags = { 'activity' }
C.allowedManualPinTypes = {
  flow = false,
  string = true,
  number = true,
  bool = true,
  any = true,
  table = true,
  vec3 = true,
  quat = true,
  color = true
}

function C:init()
  self.data.eventName = "start"
  self.allowCustomInPins = true
  self.savePins = true
end

function C:workOnce()
  local pinData = {}
  for k, v in pairs(self.pinIn) do
    if k ~= 'flow' and k ~= 'reset' then
      pinData[k] = v.value
    end
  end

  self.mgr.modules.mission:missionHook(self.data.eventName, pinData)
end

return _flowgraph_createNode(C)