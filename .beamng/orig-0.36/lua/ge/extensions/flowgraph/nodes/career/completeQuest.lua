-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Complete Quest'
C.color = ui_flowgraph_editor.nodeColors.career
C.icon = ui_flowgraph_editor.nodeIcons.career
C.behaviour = { }
C.description = "Completes the quest that the flowgraph is associated with"
C.category = 'once_instant'
C.pinSchema = {
  { dir = 'in', type = 'bool', name = 'completeOnStop', description = 'If true, only completes the current quest once the manager is stopping. (this delays it by a frame)', hardCoded = true, default=true },
}


function C:workOnce()
  if career_career.isActive() and career_modules_questManager and self.mgr.quest then
    career_modules_questManager.completeQuest(self.mgr.quest.id)
  end
end

return _flowgraph_createNode(C)
