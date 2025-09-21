-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Zone by Name'
C.description = 'Finds a single Zone by name.'
C.color = ui_flowgraph_editor.nodeColors.sites
C.pinSchema = {
  {dir = 'in', type = 'flow', name = 'flow', description = 'Inflow for this node.'},
  {dir = 'in', type = 'table', name = 'sitesData', tableType = 'sitesData', description = 'Sites data.'},
  {dir = 'in', type = 'string', name = 'zoneName', description = 'Name of the zone.'},
  {dir = 'out', type = 'flow', name = 'flow', description = 'Outflow from this node.'},
  {dir = 'out', type = 'bool', name = 'exists', description = 'True if the zone exists.', hidden = true},
  {dir = 'out', type = 'table', name = 'zone', tableType = 'zoneData', description = 'Zone data.'}
}

C.tags = {'scenario', 'sites'}

function C:init(mgr, ...)

end

function C:drawCustomProperties()
  if im.Button("Open Sites Editor") then
    if editor_sitesEditor then
      editor_sitesEditor.show()
    end
  end
  if editor_sitesEditor then
    local cZone = editor_sitesEditor.getCurrentZone()
    if cZone then
      im.Text("Currently selected Zone in editor:")
      im.Text(cZone.name)
      if im.Button("Hardcode to zoneName Pin") then
        self:_setHardcodedDummyInputPin(self.pinInLocal.zoneName, cZone.name)
      end
    end
  end
end

function C:_executionStarted()
  self._zone = nil
end

function C:work(args)
  if self.pinIn.flow.value then
    if self.pinIn.zoneName.value then
      local zone = self.pinIn.sitesData.value.zones.byName[self.pinIn.zoneName.value]
      if zone ~= self._zone then
        self._zone = zone
      end
    end
  end
  self.pinOut.zone.value = self._zone
  self.pinOut.flow.value = false
  if self._zone and not self._zone.missing then
    self.pinOut.flow.value = true
  end
  self.pinOut.exists.value = self.pinOut.flow.value
end

function C:drawMiddle(builder, style)
  builder:Middle()
  im.TextUnformatted((self._zone and (not self._zone.missing)) and self.pinIn.zoneName.value or "No zone!")
end

return _flowgraph_createNode(C)
