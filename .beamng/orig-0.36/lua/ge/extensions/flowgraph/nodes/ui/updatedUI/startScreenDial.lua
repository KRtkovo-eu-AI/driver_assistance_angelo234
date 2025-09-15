-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im = ui_imgui

local C = {}

C.name = 'StartScreen Drag Dial'
C.color = ui_flowgraph_editor.nodeColors.ui
C.description = 'Drag race dial setup.'
C.category = 'repeat_instant'

C.pinSchema = {
  { dir = 'in', type = 'flow', name = 'flow', description = '', chainFlow = true },
  { dir = 'out', type = 'flow', name = 'flow', description = '', chainFlow = true },
  { dir = 'out', type = 'table', name = 'dials', description = ''},
}

C.tags = { 'start', 'screen', 'intro', 'ui' }

function C:init()
  self.panel = {
    type = "dragDial",
    header = "Dial Setup",
    text = "Set up your dial. Racer who finishes closest to their dial without overshooting wins.",
    dials = {},
    pages = {
      main = true,
    }
  }
end

function C:work()
  self.pinOut.flow.value = self.pinIn.flow.value
  -- add it to the layout
  local dragData = gameplay_drag_general.getData()
  self.panel.dials = {}
  --dump(dragData)
  if not dragData then
    log("W", logTag, "No drag data found!")
    return
  end
  for racerId, racerData in pairs(dragData.racers) do
    table.insert(self.panel.dials,
    {
      label = racerData.isPlayable and "Player's Dial: Lane " .. racerData.lane or "Opponent's Dial: Lane " .. racerData.lane,
      key = racerData.isPlayable and "player" or "opponent",
      value = racerData.timers.dial.value or 12,
      disabled = not racerData.isPlayable,
      racerId = racerId
    })
  end
  self.mgr.modules.ui:addUIElement(self.panel)
  self.pinOut.dials.value = self.panel.dials
end

function C:onDialSetByDialPanel(dial)
  for _, d in ipairs(self.panel.dials) do
    if dial.label == d.label then
      d.value = dial.value
    end
  end
  self.pinOut.dials.value = self.panel.dials
end

return _flowgraph_createNode(C)
