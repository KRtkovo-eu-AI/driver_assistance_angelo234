-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui

local C = {}

C.name = 'Race End Transform'
C.description = 'Gives the End Pathnode Transform of a Path. Useful for creating custom triggers.'
C.category = 'repeat_instant'

C.color = im.ImVec4(1, 1, 0, 0.75)
C.pinSchema = {
  {dir = 'in', type = 'table', name = 'pathData', tableType = 'pathData', description = 'Data from the path for other nodes to process.'},
  {dir = 'out', type = 'flow', name = 'endExists', description = 'Outflow if the end node exists.'},
  {dir = 'out', type = 'flow', name = 'endDoesNotExist', description = 'Outflow if the end node does not exist.'},
  {dir = 'out', type = 'bool', name = 'existing', description = 'True if the transform was found'},
  {dir = 'out', type = 'vec3', name = 'pos', description = 'The position of this transform.'},
  {dir = 'out', type = 'vec3', name = 'dirVec', description = 'The direction vector of this transform.'},
  {dir = 'out', type = 'number', name = 'radius', description = 'The radius of this transform.'},
}

C.tags = {'scenario'}


function C:init(mgr, ...)
  self.path = nil
  self.clearOutPinsOnStart = false
end

function C:_executionStopped()
  self.path = nil
end

function C:work(args)
  self.pinOut.endExists.value = false
  self.pinOut.endDoesNotExist.value = false
  if self.path == nil and self.pinIn.pathData.value then
    self.path = self.pinIn.pathData.value

    local endNode
    if self.path.endNode and self.path.endNode ~= -1 and self.path.pathnodes.objects[self.path.endNode] then
      endNode = self.path.pathnodes.objects[self.path.endNode]
    elseif self.path.config.closed and self.path.startNode and self.path.startNode ~= -1 and self.path.pathnodes.objects[self.path.startNode] then
      endNode = self.path.pathnodes.objects[self.path.startNode]
    elseif self.path.endNode == -1 then
      log("W","","No end node found! It should be explicitely marked in the race editor." .. dumps(self.id).."!")
    end

    self.pinOut.existing.value = false

    if not endNode or endNode.missing then return end

    self.pinOut.existing.value = true
    self.pinOut.endExists.value = true
    self.pinOut.endDoesNotExist.value = false

    self.pinOut.pos.value = endNode.pos:toTable()
    self.pinOut.dirVec.value = endNode.hasNormal and endNode.normal:toTable() or {0, 0, 1}
    self.pinOut.radius.value = endNode.radius
  end
end

return _flowgraph_createNode(C)
