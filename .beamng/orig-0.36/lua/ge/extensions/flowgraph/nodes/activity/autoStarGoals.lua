-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Auto Star Goals'
C.color = im.ImVec4(0.03,0.41,0.64,0.75)
C.description = "Automatically creates goals for the tasklist app based on the active stars."
C.category = 'once_instant'

C.pinSchema = {
  { dir = 'in', type = 'bool', name = 'addMissionTitle', hardcoded = true, description = "If set, also adds the mission name as the tasklist header." }
}

C.allowedManualPinTypes = {
  flow = true,
}


C.tags = {'activity'}

function C:init()
end

local function tryBuildContext(label, data)
  if not label then return {} end
  local context = {}
  for key, value in pairs(data) do
    if type(value) == 'string' or type(value) == 'number' then
      context[key] = tostring(value)
    end
  end
  return context
end

function C:workOnce()
  guihooks.trigger("SetTasklistHeader", {
    label = self.mgr.activity.name
  })
  for _, star in ipairs(self.mgr.activity.careerSetup._activeStarCache.sortedStars) do
    if self.mgr.activity.careerSetup.starsActive[star] then
      local hook = "SetTasklistTask"
      local label = self.mgr.activity.starLabels[star] or "Missing Star Description"
      if type(label) == "string" then
        label = {
          txt = label,
          context = tryBuildContext(self.mgr.activity.starLabels[star], self.mgr.activity.missionTypeData),
        }
      elseif type(label) == "function" then
        label = label(self.mgr.activity)
      end
      guihooks.trigger(hook, {
        label = label,
        done = false,
        fail = false,
        active = true,
        id = star,
        type = "goal"
      }
    )
    end
  end
end




return _flowgraph_createNode(C)
