-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local im  = ui_imgui

local C = {}
C.windowDescription = 'Locations'

function C:init(sitesEditor, key)
  self.sitesEditor = sitesEditor
  self.key = key
  self.current = nil
end

function C:setSites(sites)
  self.sites = sites
  self.list = sites[self.key]
  self.current = nil
end

function C:select(loc)
  self.current = loc
  if self.current ~= nil then
    self:updateTransform()
  end
end

function C:hitTest(mouseInfo, objects)
  local minNodeDist = 4294967295
  local closestNode = nil
  for idx, node in pairs(objects) do
    local distNodeToCam = (node.pos - mouseInfo.camPos):length()
    local nodeRayDistance = (node.pos - mouseInfo.camPos):cross(mouseInfo.rayDir):length() / mouseInfo.rayDir:length()
    local sphereRadius = node.radius
    if nodeRayDistance <= sphereRadius then
      if distNodeToCam < minNodeDist then
        minNodeDist = distNodeToCam
        closestNode = node
      end
    end
  end
  return closestNode
end

function C:updateTransform()
  local transform = QuatF(0,0,0,0):getMatrix()
  transform:setPosition(self.current.pos)
  editor.setAxisGizmoTransform(transform)
end

local function updateTransformUndo(data)
  data.self.current = data.objects[data.id]
  if data.self.current then
    data.self.current.pos = data.old.pos
    data.self.current.radius = data.old.radius
    data.self:updateTransform()
  end
end

local function updateTransformRedo(data)
  data.self.current = data.objects[data.id]
  if data.self.current then
    data.self.current.pos = data.new.pos
    data.self.current.radius = data.new.radius
    data.self:updateTransform()
  end
end

function C:beginDrag()
  self._beginDragPos = vec3(editor.getAxisGizmoTransform():getColumn(3))
  self._beginDragRadius = math.max(0.1, self.current.radius)
end

function C:dragging()
  -- update/save our gizmo matrix
  if editor.getAxisGizmoMode() == editor.AxisGizmoMode_Translate then
    self.current.pos = vec3(editor.getAxisGizmoTransform():getColumn(3))
  elseif editor.getAxisGizmoMode() == editor.AxisGizmoMode_Scale then
    local scl = vec3(worldEditorCppApi.getAxisGizmoScale())
    if scl.x ~= 1 then
      scl = scl.x
    elseif scl.y ~= 1 then
      scl = scl.y
    elseif scl.z ~= 1 then
      scl = scl.z
    else
      scl = 1
    end
    self.current.radius = math.max(0.1, self._beginDragRadius * scl)
  end
end

function C:endDragging()
  local old = {pos = vec3(self._beginDragPos), radius = self._beginDragRadius}
  local new = {pos = vec3(self.current.pos), radius = self.current.radius}
  editor.history:commitAction("Update Transform of "..self.current.name,
    { self = self, objects = self.sites[self.key].objects, id = self.current.id, old = old, new = new },
    updateTransformUndo, updateTransformRedo)
end

local function createUndo(data)
  if not data.id and data.self.current then
    data.id = data.self.current.id
  end
  if data.id and data.list.objects[data.id] and not data.list.objects[data.id].missing then
    data.list:remove(data.id)
  end
end

local function createRedo(data)
  local loc = data.list:create()
  loc:set(data.pos, 3)
  if data.loc then
    loc:onDeserialized(data.loc)
  end
  data.loc = loc:onSerialize()
  data.id = loc.id
  data.self.current = loc
end

function C:create(pos)
  editor.history:commitAction("Create Location",
    { self = self, list = self.list, pos = pos },
    createUndo, createRedo)
  return self.current
end

function C:remove(loc)
  editor.history:commitAction("Remove Location",
    { self = self, list = self.list, loc = loc and loc:onSerialize(), id = loc and loc.id },
    createRedo, createUndo)
end

function C:drawElement(loc)
  if self.sitesEditor.allowGizmo() then
    self.current = loc
    editor.updateAxisGizmo(function() self:beginDrag() end, function() self:endDragging() end, function() self:dragging() end)
    editor.drawAxisGizmo()
  end
  local avail = im.GetContentRegionAvail()
end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end
