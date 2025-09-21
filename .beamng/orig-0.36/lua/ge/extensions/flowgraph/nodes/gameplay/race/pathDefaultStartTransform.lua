-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Race Start Transform'
C.description = 'Gives the Start Positions Transform of a Path. Useful for creating custom triggers.'
C.category = 'repeat_instant'

C.color = im.ImVec4(1, 1, 0, 0.75)
C.pinSchema = {
  {dir = 'in', type = 'table', name = 'pathData', tableType = 'pathData', description = 'Data from the path for other nodes to process.'},
  {dir = 'in', type = {'string', 'number'}, name = 'name', description = 'Name of the start position to get; if none given, will use default.'},
  {dir = 'out', type = 'bool', name = 'existing', description = 'True if the start position was found.'},
  {dir = 'out', type = 'vec3', name = 'pos', description = 'The calculated position of this transform.'},
  {dir = 'out', type = 'vec3', name = 'origPos', hidden = true, description = 'The original position of this transform.'},
  {dir = 'out', type = 'quat', name = 'rot', description = 'The rotation of this transform.'},
  {dir = 'out', type = 'vec3', name = 'scl', description = 'The scale of this transform.'}
}

C.tags = {'scenario', 'race'}


function C:init(mgr, ...)
  self.path = nil
  self.clearOutPinsOnStart = false
  self.data.startBoxWidth = 2
  self.data.startBoxLength = 5
  self.data.startBoxHeight = 1.5
  self.data.useStringMatch = false
end

function C:_executionStopped()
  self.path = nil
end

function C:work(args)
  if self.path == nil and self.pinIn.pathData.value then
    self.path = self.pinIn.pathData.value

    local sp = self.path.startPositions.objects[self.path.defaultStartPosition]
    if self.pinIn.name.value then
      if type(self.pinIn.name.value) == 'string' then
        if self.data.useStringMatch then -- used in some special cases
          for _, v in ipairs(self.pinIn.pathData.value.startPositions.sorted) do
            if string.find(v.name, self.pinIn.name.value) then -- uses first result, if found
              sp = v
              break
            end
          end
        else
          sp = self.path:findStartPositionByName(self.pinIn.name.value)
        end
      elseif type(self.pinIn.name.value) == 'number' then
        sp = self.path.startPositions.objects[self.pinIn.name.value]
      end
    end
    self.pinOut.existing.value = false
    if sp == nil or sp.missing then return end
    
    local rot = sp.rot
    self.pinOut.rot.value = {rot.x, rot.y, rot.z, rot.w}
    self.pinOut.scl.value = {self.data.startBoxWidth, self.data.startBoxLength, self.data.startBoxHeight}
    
    local x, y, z = rot * vec3(1,0,0), rot * vec3(0,1,0), rot * vec3(0,0,1)
    self.pinOut.pos.value = vec3(sp.pos - (self.data.startBoxLength / 2) * y + (self.data.startBoxHeight / 2) * z):toTable()
    self.pinOut.origPos.value = sp.pos:toTable()
    self.pinOut.existing.value = not sp.missing
  end
end

return _flowgraph_createNode(C)
