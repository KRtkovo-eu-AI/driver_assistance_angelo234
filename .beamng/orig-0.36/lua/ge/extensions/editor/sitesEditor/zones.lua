-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local im = ui_imgui

local snapToTerrain = true

local C = {}
C.windowDescription = 'Zones'

function C:init(sitesEditor, key)
  self.sitesEditor = sitesEditor
  self.key = key
  self.current = nil
  self.currentVertices = {}
end

function C:setSites(sites)
  self.sites = sites
  self.list = sites[self.key]
  self.current = nil
end

function C:select(zone)
  self.current = zone
  if self.current ~= nil then
    self:updateTransform()
  end
  self.currentVertices = {}
  self.currentPlane = nil
end

function C:findVert(mouseInfo, objects)
  local minNodeDist = 4294967295
  local closest = nil
  local clrF = ColorF(1, 1, 1, 0.75)
  local clrSelected = ColorF(0.91, 0.49, 0.24, 0.75)
  for idx, obj in pairs(objects) do
    local distNodeToCam = (obj.pos - mouseInfo.camPos):length()
    local nodeRayDistance = (obj.pos - mouseInfo.camPos):cross(mouseInfo.rayDir):length() / mouseInfo.rayDir:length()
    --local nodeRayDistance = (node.pos - camPos):cross(rayDir):length() / rayDir:length()
    local sphereRadius = (mouseInfo.camPos - obj.pos):length() / 40

    local selected = false
    for _, vertex in ipairs(self.currentVertices) do
      if vertex.index == idx then
        selected = true
        break
      end
    end
    if selected then
      debugDrawer:drawSphere(obj.pos, sphereRadius, clrSelected)
    else
      debugDrawer:drawSphere(obj.pos, sphereRadius, clrF)
    end
    --local sphereRadius = obj.radius
    if nodeRayDistance <= sphereRadius then
      if distNodeToCam < minNodeDist then
        minNodeDist = distNodeToCam
        closest = obj
      end
    end
  end
  return closest
end

function C:tryInsert(mouseInfo)
  if #self.current.vertices < 2 then
    return
  end
  local objs = {}

  for i, v in ipairs(self.current.vertices) do
    table.insert(objs, {
      pos = (v.pos + self.current.vertices[v.next].pos) / 2,
      radius = 3,
      orig = i
    })

  end
  local hit = self:findVert(mouseInfo, objs)
  if hit and mouseInfo.down then
    self.current:addVertex(hit.pos, hit.orig + 1)
  end

end

function C:input(mouseInfo)
  if not editor.isAxisGizmoHovered() and mouseInfo.down and not editor.keyModifiers.shift then
    local zoneHit = self:findZone(mouseInfo, self.sites[self.key].objects)
    if zoneHit and editor_sitesEditor then
      local mode = nil
      if editor.keyModifiers.ctrl then
        mode = "toggle"
      end
      editor_sitesEditor.selectZone(zoneHit.id, mode)
    end
  end

  if not self.current then
    return
  end

  if editor.keyModifiers.shift then
    -- adding
    if not editor.isAxisGizmoHovered() and mouseInfo.down then
      self.current:addVertex(mouseInfo._downPos)
    end
  elseif editor.keyModifiers.alt then
    self:tryInsert(mouseInfo)
  else
    -- selecting
    local hit = self:findVert(mouseInfo, self.current.vertices)
    if not editor.isAxisGizmoHovered() and mouseInfo.down then

      if editor.keyModifiers.ctrl then
        table.insert(self.currentVertices, hit)
      else
        self.currentVertices = { hit }
      end

      self.currentPlane = nil
      self:updateTransform()
    end
  end
end

function C:updateTransform()

  local transform = QuatF(0, 0, 0, 0):getMatrix()
  if tableSize(self.currentVertices) > 0 then

    local centroid = vec3(0, 0, 0)
    for _, vertex in ipairs(self.currentVertices) do
      centroid = centroid + vertex.pos
    end
    centroid = centroid / tableSize(self.currentVertices)
    transform:setPosition(centroid)
  end
  if self.currentPlane then
    local rotation
    if editor.getAxisGizmoAlignment() == editor.AxisGizmoAlignment_Local then
      local q = quatFromDir(quatFromEuler(0, math.pi / 2, 0) * self.currentPlane.normal, vec3(0, 0, 1))
      rotation = QuatF(q.x, q.y, q.z, q.w)
    else
      rotation = QuatF(0, 0, 0, 1)
    end
    transform = rotation:getMatrix()
    transform:setPosition(self.currentPlane.pos)
  end
  editor.setAxisGizmoTransform(transform)
end

function C:beginDrag()
  self._prevGizmoPos = vec3(editor.getAxisGizmoTransform():getColumn(3))

  self._prevVerticesPos = {}
  for _, vertex in ipairs(self.currentVertices) do
    self._prevVerticesPos[vertex.index] = vertex.pos
  end

  if self.currentPlane then
    self.beginDragRotation = deepcopy(quatFromDir(quatFromEuler(0, math.pi / 2, 0) * self.currentPlane.normal, vec3(0, 0, 1)))
  end
end

function C:dragging()
  -- update/save our gizmo matrix
  local posOffset = (vec3(editor.getAxisGizmoTransform():getColumn(3)) - self._prevGizmoPos) / 2

  if editor.getAxisGizmoMode() == editor.AxisGizmoMode_Translate then

    -- check for vertices
    if tableSize(self.currentVertices) > 0 then
      for i, vertex in ipairs(self.currentVertices) do
        local succ
        vertex.pos = vertex.pos + posOffset
        if snapToTerrain then
          vertex.pos, succ = self:dropToTerrain(vertex.pos + posOffset)
          if not succ then
            vertex.pos.z = self._prevVerticesPos[vertex.index].z
          end
          debugDrawer:drawLine((vertex.pos + posOffset + vec3(0, 0, -1000)), (vertex.pos + posOffset + vec3(0, 0, 1000)), ColorF(0, 0, 1, 1))
        end
      end
    end

    -- check for plane
    if self.currentPlane then
      self.currentPlane.pos = vec3(editor.getAxisGizmoTransform():getColumn(3))
    end
    self:updateTransform()

  elseif editor.getAxisGizmoMode() == editor.AxisGizmoMode_Rotate then

    -- check for vertices (only rotate if more than one selected)
    if tableSize(self.currentVertices) > 1 then
      local centroid = vec3(0, 0, 0)
      for _, vertex in ipairs(self.currentVertices) do
        centroid = centroid + vertex.pos
      end
      centroid = centroid / tableSize(self.currentVertices)

      for i, vertex in ipairs(self.currentVertices) do

        local gizmoTransform = editor.getAxisGizmoTransform()
        local rotation = QuatF(0, 0, 0, 1)
        rotation:setFromMatrix(gizmoTransform)

        local diff = self._prevVerticesPos[vertex.index] - centroid
        diff = diff:rotated(quat(rotation))
        vertex.pos = centroid + diff

      end
    end

    if self.currentPlane then
      local gizmoTransform = editor.getAxisGizmoTransform()
      local rotation = QuatF(0, 0, 0, 1)

      rotation:setFromMatrix(gizmoTransform)

      if editor.getAxisGizmoAlignment() == editor.AxisGizmoAlignment_Local then
        self.currentPlane.normal = quat(rotation) * vec3(0, 0, 1)
      else
        self.currentPlane.normal = self.beginDragRotation * quat(rotation) * vec3(0, 0, 1)
      end
    end
  end

  self._prevGizmoPos = vec3(editor.getAxisGizmoTransform():getColumn(3))
end

function C:dropToTerrain(pos)
  local p = vec3(pos)
  if core_terrain then
    p.z = (core_terrain.getTerrainHeight(p) or p.z)
    return p, true
  end
  return p, false
end

local function draggingUndo(actionData)
  for idx, pos in pairs(actionData.oldPos) do
    actionData.self.current.vertices[idx].pos = pos
  end
  actionData.self:updateTransform()
end

local function draggingRedo(actionData)
  for idx, pos in pairs(actionData.newPos) do
    actionData.self.current.vertices[idx].pos = pos
  end
  actionData.self:updateTransform()
end

function C:endDragging()
  -- undo action
  if snapToTerrain then
    for _, vertex in ipairs(self.currentVertices) do
      vertex.pos = self:dropToTerrain(vertex.pos)
    end
  end
  self.current:processVertices()

  local newVerticesPos = {}
  for _, vertex in ipairs(self.currentVertices) do
    newVerticesPos[vertex.index] = vertex.pos
  end

  --log("I","",dumps(self.currentVertices))
  editor.history:commitAction("Dragged Vertices", { oldPos = self._prevVerticesPos, newPos = newVerticesPos, self = self }, draggingUndo, draggingRedo)
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
  local zone = data.list:create()
  if data.id and data.list.objects[data.id] and not data.list.objects[data.id].missing then
    zone:onDeserialized(data.list.objects[data.id]:onSerialize())
  end
  data.zone = zone:onSerialize()
  data.id = zone.id
  data.self.current = zone
end

function C:create()
  editor.history:commitAction("Create Zone",
    { self = self, list = self.list },
    createUndo, createRedo)
  return self.current
end

function C:remove(zone)
  editor.history:commitAction("Remove Zone",
    { self = self, list = self.list, zone = zone and zone:onSerialize(), id = zone and zone.id },
    createRedo, createUndo)
end

function C:drawElement(zone)
  self.current = zone
  if (tableSize(self.currentVertices) > 0 or self.currentPlane) and self.sitesEditor.allowGizmo() then
    editor.updateAxisGizmo(function()
      self:beginDrag()
    end, function()
      self:endDragging()
    end, function()
      self:dragging()
    end)
    editor.drawAxisGizmo()
  end

  local avail = im.GetContentRegionAvail()
  im.Text("Properties")
  im.Text("Vertex Count: " .. (#self.current.vertices))
  im.Text("Area: %0.2fm²", self.current:zoneArea())
  if im.Button("Select all vertices") then
    self.currentVertices = self.current.vertices
    self:updateTransform()
  end
  if im.Button("Delete current selection") then
    for _, vertex in ipairs(self.currentVertices) do
      self.current:removeVertex(vertex.index)
    end
  end
  if im.IsItemHovered() then
    for _, vertex in ipairs(self.currentVertices) do
      debugDrawer:drawSphere(vertex.pos, 2, ColorF(1, 0, 0, 1))
    end
  end
  if im.Button("Auto Planes") then
    self.current:autoPlanes()
  end
  if im.Button("Divide") then
    self.current:makeHighResolutionFence()
  end

  local snapActive = im.BoolPtr(snapToTerrain)
  if im.Checkbox("Snap To Terrain", snapActive) then
    snapToTerrain = snapActive[0]
  end

  local topActive = im.BoolPtr(self.current.top.active)
  if im.Checkbox("Top Plane", topActive) then
    self.current.top.active = topActive[0]
  end
  if self.current.top.active then
    im.SameLine()
    if im.Button("Edit Top plane") then
      self.currentVertices = {}
      self.currentPlane = self.current.top
      self:updateTransform()
    end
  end
  local botActive = im.BoolPtr(self.current.bot.active)
  if im.Checkbox("Bot Plane", botActive) then
    self.current.bot.active = botActive[0]
  end
  if self.current.bot.active then
    im.SameLine()
    if im.Button("Edit Bot plane") then
      self.currentVertices = {}
      self.currentPlane = self.current.bot
      self:updateTransform()
    end
  end
  local pos
  local playerVehicle = getPlayerVehicle(0)
  if playerVehicle then
    pos = playerVehicle:getPosition()
  else
    pos = core_camera.getPosition()
  end
  zone:drawFence((pos), pos + vec3(1000, 0, 0), ColorI(255, 128, 128, 128))
  local prof = hptimer()
  local inside = self.current:containsPoint2D(pos)
  local time = prof:stop()

  im.Text("Inside: " .. dumps(inside))
  im.Text("Calc Time: " .. (time) .. " ms")
end

function C:findZone(mouseInfo, objects)
  local zone = nil
  local distance = math.huge
  local minDistance = math.huge
  for _, obj in pairs(objects) do
    if obj.vertices then
      distance = obj:rayIntersectsZone(mouseInfo.camPos, mouseInfo.rayDir)
      if distance < minDistance then
        minDistance = distance
        zone = obj
      end
    end
  end
  return zone
end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end
