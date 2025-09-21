-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Stars Active'
C.color = im.ImVec4(0.03,0.41,0.64,0.75)
C.description = "Lets flow through if stars with specific keys are active. Only lets flow through if this mission is played in career."
C.category = 'repeat_instant'

C.pinSchema = {}

C.allowedManualPinTypes = {
  flow = true,
}


C.tags = {'activity', 'mission'}

function C:init()
  self.savePins = true
  self.allowCustomOutPins = true
end

function C:_executionStarted()
  self.hasRetrievedActiveStars = false
  self.activeStars = nil
end

function C:work()
  if self.pinIn.flow.value then

    -- only get the active stars once because its expensive
    if not self.activeStars then
      local mission = self.mgr.activity
      if not mission then return end
      local unflattenedSettings = {}
      for k, v in pairs(mission.lastUserSettings) do
        table.insert(unflattenedSettings, {key = k, value = v})
      end

      self.activeStars = gameplay_missions_missionScreen.getActiveStarsForUserSettings(mission.id, unflattenedSettings).starInfo
    end

    for _, pin in pairs(self.pinOut) do
      if pin.type == 'flow' then
        pin.value = self.activeStars[pin.name] and self.activeStars[pin.name].enabled or false
      end
    end
  else
    for _, pin in pairs(self.pinOut) do
      if pin.type == 'flow' then
        pin.value = false
      end
    end
  end
end


return _flowgraph_createNode(C)
