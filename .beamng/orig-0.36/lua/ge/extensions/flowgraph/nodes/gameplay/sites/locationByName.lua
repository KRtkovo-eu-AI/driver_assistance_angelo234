-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Location by Name'
C.description = 'Finds a single Location by name.'
C.color = ui_flowgraph_editor.nodeColors.sites
C.pinSchema = {
  {dir = 'in', type = 'flow', name = 'flow', description = 'Inflow for this node.'},
  {dir = 'in', type = 'table', name = 'sitesData', tableType = "sitesData", description = 'Sites data.'},
  {dir = 'in', type = 'string', name = 'locationName', description = 'Name of the location.'},
  {dir = 'out', type = 'flow', name = 'flow', description = 'Outflow from this node.'},
  {dir = 'out', type = 'bool', name = 'exists', description = 'True if the location exists.', hidden = true},
  {dir = 'out', type = 'table', name = 'location', tableType = 'locationData', description = 'Location data.'}
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
    local loc = editor_sitesEditor.getCurrentLocation()
    if loc then
      im.Text("Currently selected Location in editor:")
      im.Text(loc.name)
      if im.Button("Hardcode to locationName Pin") then
        self:_setHardcodedDummyInputPin(self.pinInLocal.locationName, loc.name)
      end
    end
  end
end

function C:_executionStarted()
  self._location = nil
end

function C:work(args)
  if self.pinIn.flow.value then
    if self.pinIn.locationName.value then
      local loc = self.pinIn.sitesData.value.locations.byName[self.pinIn.locationName.value]
      if loc ~= self._location then
        self._location = loc
        self.pinOut.flow.value = false
        if not self._location.missing then
          self.pinOut.location.value = self._location
          self.pinOut.flow.value = true
        end
      end
    end
  end
  self.pinOut.exists.value = self.pinOut.flow.value
end

function C:drawMiddle(builder, style)
  builder:Middle()
  im.TextUnformatted((self._location and (not self._location.missing)) and self.pinIn.locationName.value or "No location!")
end

return _flowgraph_createNode(C)
