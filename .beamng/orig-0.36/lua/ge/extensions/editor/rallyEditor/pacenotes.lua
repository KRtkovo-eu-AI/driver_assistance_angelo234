-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui
local logTag = ''
local waypointTypes = require('/lua/ge/extensions/gameplay/rally/notebook/waypointTypes')
local cc = require('/lua/ge/extensions/gameplay/rally/util/colors')
local rallyUtil = require('/lua/ge/extensions/gameplay/rally/util')
local Snaproad = require('/lua/ge/extensions/gameplay/rally/snaproad')
local Recce = require('/lua/ge/extensions/gameplay/rally/recce')
local RallyEnums = require('/lua/ge/extensions/gameplay/rally/enums')
local PacenoteForm = require('/lua/ge/extensions/editor/rallyEditor/pacenotes/pacenoteForm')
local RallyManager = require('/lua/ge/extensions/gameplay/rally/rallyManager')
local Driveline = require('/lua/ge/extensions/gameplay/rally/driveline')

-- pacenote form fields
-- local pacenoteNameText = im.ArrayChar(1024, "")
-- local playbackRulesText = im.ArrayChar(1024, "")

-- waypoint form fields
-- local waypointNameText = im.ArrayChar(1024, "")
-- local waypointPosition = im.ArrayFloat(3)
-- local waypointNormal = im.ArrayFloat(3)
-- local waypointRadius = im.FloatPtr(0)

local pacenotesSearchText = im.ArrayChar(1024, "")

local C = {}
C.windowDescription = 'Pacenotes'

local function selectWaypointUndo(data)
  data.self:selectWaypoint(data.old)
end
local function selectWaypointRedo(data)
  data.self:selectWaypoint(data.new)
end

local editModes = {
  editAll = 'edit_all',
  editAT = 'edit_at',
  editCorners = 'edit_corners',
}

function C:init(rallyEditor)
  self.rallyEditor = rallyEditor
  self.mouseInfo = {}

  self.wasWPSelected = false

  self.notes_valid = true
  self.validation_issues = {}

  self.pacenoteToolsState = {
    insertMode = false,
    snaproad = nil,
    mode = nil,
    search = nil,
    internal_lock = false,
    hover_wp_id = nil,
    shift = false,
    selected_pn_id = nil,
    selected_wp_id = nil,
    recent_selected_pn_id = nil,
    playbackLastCameraPos = nil,
    last_camera = {
      pos = nil,
      quat = nil
    }
  }

  self.pacenoteForm = PacenoteForm(self)
end

function C:isValid()
  return self.notes_valid and #self.validation_issues == 0
end

function C:setPath(path)
  self.path = path
end

function C:onEditModeActivate()
  self:selectPacenote(self.pacenoteToolsState.selected_pn_id)
end

function C:onEditModeDeactivate()
  self.rallyEditor.setFreeCam()
end

function C:getRacePath()
  return editor_raceEditor.getCurrentPath()
end

-- function C:selectionString()
--   local pn = self:selectedPacenote()
--   local wp = self:selectedWaypoint()
--   local text = {}
--   local mode = '--'
--   if pn and not pn.missing then
--     mode = 'P-'
--     local p_txt = '"' .. pn:noteTextForDrawDebug() .. '" ('.. pn.name ..')'
--     table.insert(text, p_txt)
--     if wp and not wp.missing then
--       mode = 'PW'
--       local w_txt = wp:selectionString()
--       table.insert(text, w_txt)
--     end
--   end
--   return text, mode
-- end

function C:selectedPacenote()
  if not self.path then return nil end
  if self.pacenoteToolsState.selected_pn_id then
    return self.path.pacenotes.objects[self.pacenoteToolsState.selected_pn_id]
  else
    return nil
  end
end

function C:selectedWaypoint()
  if not self:selectedPacenote() then return nil end
  if self.pacenoteToolsState.selected_wp_id then
    if self:selectedPacenote().pacenoteWaypoints then
      return self:selectedPacenote().pacenoteWaypoints.objects[self.pacenoteToolsState.selected_wp_id]
    else
      return nil
    end
  else
    return nil
  end
end

function C:loadSnaproad()
  local recce = Recce(self.path:getMissionDir())

  local snaproadType = editor.getPreference("rallyEditor.editing.preferredSnaproadType")

  if snaproadType == 'recce' then
    if recce:loadDrivelineAndCuts() then
      self.pacenoteToolsState.snaproad = self:loadSnaproadRecce(recce)
    else
      log('W', logTag, 'no recce driveline found for missionDir='..self.path:getMissionDir())
    end
  elseif snaproadType == 'route' then
    self.pacenoteToolsState.snaproad = self:loadSnaproadRoute(recce)
  end
end

function C:loadSnaproadRoute(recce)
  local missionId = self.path:missionId()
  local missionDir = self.path:getMissionDir()
  local missionName = rallyUtil.translatedMissionNameFromId(missionId)

  local styleData = recce.settings:getCornerCallStyle()

  log('D', logTag, string.format('loadSnaproadRoute missionId=%s missionDir=%s missionName=%s', missionId, missionDir, missionName))

  local rallyManager = RallyManager(missionDir, missionId, 100, 5)

  if not rallyManager:reload() then
    log('E', logTag, 'RallyManager reload failed for snaproad setup')
    return nil
  end

  -- local dr = rallyManager:getDrivelineRoute()
  local driveline = Driveline(missionDir)
  -- if not driveline:loadFromRoute(dr.routeStatic) then
  local snaproadPoints = rallyManager:getSnaproadPointsFromRoute()
  if not snaproadPoints then
    log('E', logTag, 'failed to get snaproad points from route')
    return nil
  end
  if not driveline:loadFromRoute(snaproadPoints) then
    log('E', logTag, 'failed to load driveline for route snaproad')
    return nil
  end
  return Snaproad(driveline, styleData, RallyEnums.drivelineMode.route)
end

function C:loadSnaproadRecce(recce)
  local styleData = recce.settings:getCornerCallStyle()
  return Snaproad(recce.driveline, styleData, RallyEnums.drivelineMode.recce)
end

-- called by RallyEditor when this tab is selected.
function C:selected()
  if not self.path then return end

  if not self.pacenoteToolsState.mode then
    self:cycleEditMode()
  end
  self:loadSnaproad()
  self:selectPacenote(self.pacenoteToolsState.selected_pn_id)

  editor.editModes.notebookEditMode.auxShortcuts[editor.AuxControl_Ctrl] = "Create new pacenote"
  editor.editModes.notebookEditMode.auxShortcuts[editor.AuxControl_Delete] = "Delete"
  -- force redraw of shortcutLegend window
  extensions.hook("onEditorEditModeChanged", nil, nil)
end

-- called by RallyEditor when this tab is unselected.
function C:unselect()
  if not self.path then return end

  self.rallyEditor.setFreeCam()

  editor.editModes.notebookEditMode.auxShortcuts[editor.AuxControl_Ctrl] = nil
  editor.editModes.notebookEditMode.auxShortcuts[editor.AuxControl_Delete] = nil
  -- force redraw of shortcutLegend window
  extensions.hook("onEditorEditModeChanged", nil, nil)
end

function C:setOrbitCameraToSelectedPacenote()
  if self:selectedPacenote() then
    core_camera.setByName(0, "pacenoteOrbit")
    core_camera.setRef(0, self:selectedPacenote():getPosForOrbitCamera())
  end
end

function C:selectPacenote(id)
  if not self.path then return end
  if not self.path.pacenotes then return end

  -- deselect waypoint if we are changing pacenotes.
  if self.pacenoteToolsState.selected_pn_id ~= id then
    self.pacenoteToolsState.selected_wp_id = nil
    if self.pacenoteToolsState.snaproad then
      self.pacenoteToolsState.snaproad:clearFilter()
    end
  end

  if not id then -- track the most recent selection
    if self.pacenoteToolsState.selected_pn_id then
      self.pacenoteToolsState.recent_selected_pn_id = self.pacenoteToolsState.selected_pn_id
    end
    self.pacenoteToolsState.playbackLastCameraPos = nil
  end

  self.pacenoteToolsState.selected_pn_id = id

  self.path:setAdjacentNotes(self.pacenoteToolsState.selected_pn_id)

  -- select the pacenote
  if id then
    local note = self.path.pacenotes.objects[id]
    -- pacenoteNameText = im.ArrayChar(1024, note.name)
    -- playbackRulesText = im.ArrayChar(1024, note.playback_rules)
    self.pacenoteForm:setPacenote(note)
    local startAtCs = self.pacenoteToolsState.mode == editModes.editCorners
    if self.pacenoteToolsState.snaproad then
      self.pacenoteToolsState.snaproad:setPartitionToPacenote(note, startAtCs)
    end
    -- self:setOrbitCameraToSelectedPacenote()
  else
    -- pacenoteNameText = im.ArrayChar(1024, "")
    -- playbackRulesText = im.ArrayChar(1024, "")
    self.pacenoteForm:setPacenote(nil)
    if self.pacenoteToolsState.snaproad then
      self.pacenoteToolsState.snaproad:clearPartition()
    end
    self.rallyEditor.setFreeCam()
  end
end

function C:selectWaypoint(id)
  if not self.path then return end
  self.pacenoteToolsState.selected_wp_id = id

  if id then
    local waypoint = self.path:getWaypoint(id)
    if waypoint then
      self:selectPacenote(waypoint.pacenote.id)
      -- waypointNameText = im.ArrayChar(1024, waypoint.name)
      self:updateGizmoTransform(id)
      if self.pacenoteToolsState.snaproad then
        self.pacenoteToolsState.snaproad:setFilter(waypoint)
        self.pacenoteToolsState.snaproad:setPartitionToFilter()
      end
    else
      log('E', logTag, 'expected to find waypoint with id='..id)
      if self.pacenoteToolsState.snaproad then
        self.pacenoteToolsState.snaproad:clearFilter()
        local startAtCs = self.pacenoteToolsState.mode == editModes.editCorners
        self.pacenoteToolsState.snaproad:setPartitionToPacenote(self:selectedPacenote(), startAtCs)
      end
    end
  else -- deselect waypoint
    -- waypointNameText = im.ArrayChar(1024, "")
    -- I think this fixes the bug where you cant click on a pacenote waypoint anymore.
    -- I think that was due to the Gizmo being present but undrawn, and the gizmo's mouseover behavior was superseding our pacenote hover.
    self:resetGizmoTransformToOrigin()

    if self.pacenoteToolsState.snaproad then
      self.pacenoteToolsState.snaproad:clearFilter()
      local startAtCs = self.pacenoteToolsState.mode == editModes.editCorners
      self.pacenoteToolsState.snaproad:setPartitionToPacenote(self:selectedPacenote(), startAtCs)
    end
  end
end

function C:deselect()
  if self:cameraPathIsPlaying() then return end

  -- since there are two levels of selection (waypoint+pacenote, pacenote),
  -- you must deselect twice to deselect everything.
  if self:selectedWaypoint() then
    self:selectWaypoint(nil)
  else
    self:selectPacenote(nil)
  end
end

function C:attemptToFixMapEdgeIssue()
  self:resetGizmoTransformToOrigin()
end

function C:resetGizmoTransformToOrigin()
  local rotation = QuatF(0,0,0,1)
  local transform = rotation:getMatrix()
  local pos = {0, 0, -1000} -- stick gizmo far away down the Z axis to hide it.
  transform:setPosition(pos)
  editor.setAxisGizmoTransform(transform)
  worldEditorCppApi.setAxisGizmoSelectedElement(-1)
  -- editor.drawAxisGizmo()
end

function C:updateGizmoTransform(index)
  if not self.rallyEditor.allowGizmo() then return end

  local wp = self.path:getWaypoint(index)
  if not wp then return end

  local rotation = QuatF(0,0,0,1)

  if editor.getAxisGizmoAlignment() == editor.AxisGizmoAlignment_Local then
    local q = quatFromDir(wp.normal, vec3(0,0,1))
    rotation = QuatF(q.x, q.y, q.z, q.w)
  else
    rotation = QuatF(0, 0, 0, 1)
  end

  local transform = rotation:getMatrix()
  transform:setPosition(wp.pos)
  editor.setAxisGizmoTransform(transform)
end

function C:beginDrag()
  if not self:selectedPacenote() then return end
  local wp = self:selectedPacenote().pacenoteWaypoints.objects[self.pacenoteToolsState.selected_wp_id]
  if not wp or wp.missing then return end

  self.beginDragNoteData = wp:onSerialize()

  if wp.normal then
    self.beginDragRotation = deepcopy(quatFromDir(wp.normal, vec3(0,0,1)))
  end

  self.beginDragRadius = wp.radius
end

function C:dragging()
  if not self:selectedPacenote() then return end
  local wp = self:selectedPacenote().pacenoteWaypoints.objects[self.pacenoteToolsState.selected_wp_id]
  if not wp or wp.missing then return end

  -- update/save our gizmo matrix
  if editor.getAxisGizmoMode() == editor.AxisGizmoMode_Translate then
    wp.pos = vec3(editor.getAxisGizmoTransform():getColumn(3))
  elseif editor.getAxisGizmoMode() == editor.AxisGizmoMode_Rotate then
    local gizmoTransform = editor.getAxisGizmoTransform()
    local rotation = QuatF(0,0,0,1)
    if wp.normal then
      rotation:setFromMatrix(gizmoTransform)
      if editor.getAxisGizmoAlignment() == editor.AxisGizmoAlignment_Local then
        wp.normal = quat(rotation)*vec3(0,1,0)
      else
        wp.normal = self.beginDragRotation * quat(rotation)*vec3(0,1,0)
      end
    end
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
    if scl < 0 then
      scl = 0
    end
    wp.radius = self.beginDragRadius * scl
  end
end

function C:endDragging()
  if not self:selectedPacenote() then return end
  local wp = self:selectedPacenote().pacenoteWaypoints.objects[self.pacenoteToolsState.selected_wp_id]
  if not wp or wp.missing then return end

  editor.history:commitAction("Manipulated Note Waypoint via Gizmo",
    {old = self.beginDragNoteData,
     new = wp:onSerialize(),
     index = self.pacenoteToolsState.selected_wp_id, self = self},
    function(data) -- undo
      local wp = self:selectedPacenote().pacenoteWaypoints.objects[data.index]
      wp:onDeserialized(data.old)
      data.self:selectWaypoint(data.index)
    end,
    function(data) --redo
      local wp = self:selectedPacenote().pacenoteWaypoints.objects[data.index]
      wp:onDeserialized(data.new)
      data.self:selectWaypoint(data.index)
    end
  )
end

function C:drawDebugEntrypoint(globalOpacity)
  local rallyDevToolsWindowOpen = self.rallyEditor.rallyDevToolsWindowOpen[0]

  if self.path and self.pacenoteToolsState.snaproad then
    if not rallyDevToolsWindowOpen and not self.pacenoteToolsState.snaproad:partitionAllEnabled() then
      self.path:drawDebugNotebook(self.pacenoteToolsState, globalOpacity)
    end
  end

  if self.pacenoteToolsState.snaproad then
    self.pacenoteToolsState.snaproad:setGlobalOpacity(globalOpacity)
    if not rallyDevToolsWindowOpen then
      self.pacenoteToolsState.snaproad:drawDebugSnaproad()
    end

    if not rallyDevToolsWindowOpen and self.pacenoteToolsState.snaproad:partitionAllEnabled() then
      self.path:drawDebugNotebookForPartitionedSnaproad(self.pacenoteToolsState, globalOpacity)
    end
  end

  if self.pacenoteToolsState.playbackLastCameraPos then
    local clr = cc.clr_purple
    local radius = cc.cam_last_pos_radius
    local alpha = cc.cam_last_pos_alpha
    debugDrawer:drawSphere(self.pacenoteToolsState.playbackLastCameraPos, radius, ColorF(clr[1],clr[2],clr[3],alpha))
  end
end

function C:drawDebugCameraPlaying()
  if self.pacenoteToolsState.snaproad then
    self.pacenoteToolsState.snaproad:drawDebugCameraPlaying()
  end
end

function C:handleMouseDown(hoveredWp)
  local autoCameraFocus = true

  if hoveredWp then
    local selectedPn = hoveredWp.pacenote
    if self:selectedPacenote() and self:selectedPacenote().id == selectedPn.id then
      -- print('if a pacenote is already selected and the clicked waypoint is in that pacenote.')
      self.simpleDragMouseOffset = self.mouseInfo._downPos - hoveredWp.pos
      self.beginSimpleDragNoteData = hoveredWp:onSerialize()

      -- if self:selectedWaypoint() then
      if self:selectedWaypoint() and self:selectedWaypoint().id ~= hoveredWp.id then
        -- self.pacenote_tools_state.internal_lock = true
        self:selectWaypoint(hoveredWp.id)
      elseif not self:selectedWaypoint() then
        if editor.keyModifiers.shift then
          -- self.pacenote_tools_state.internal_lock = true
          self:setOrbitCameraToSelectedPacenote()
        else
          -- self.pacenote_tools_state.internal_lock = true
          self:selectWaypoint(hoveredWp.id)
        end
      end
    elseif self:selectedPacenote() and self:selectedWaypoint() and self:selectedPacenote().id ~= selectedPn.id then
      -- print('if the selected waypoint is from a different pacenote than the clicked waypoint')
      self.simpleDragMouseOffset = self.mouseInfo._downPos - hoveredWp.pos
      -- if autoCameraFocus then
      --   self.pacenote_tools_state.internal_lock = true
      -- end
      self:selectPacenote(selectedPn.id)
      self:selectWaypoint(hoveredWp.id)
    elseif self:selectedPacenote() and self:selectedPacenote().id ~= selectedPn.id then
      -- print('if the selected pacenote is different than clicked waypoint')
      self.simpleDragMouseOffset = self.mouseInfo._downPos - hoveredWp.pos
      if autoCameraFocus and editor.keyModifiers.ctrl then
        self.pacenoteToolsState.internal_lock = true
      end
      self:selectPacenote(selectedPn.id)
      self:selectWaypoint(nil)
      if autoCameraFocus and editor.keyModifiers.shift then
        self:setOrbitCameraToSelectedPacenote()
      end
    elseif not self:selectedPacenote() then
      -- print('if no pacenote is selected')
      self:selectPacenote(selectedPn.id)
      self:selectWaypoint(nil)
      if autoCameraFocus and editor.keyModifiers.shift then
        self:setOrbitCameraToSelectedPacenote()
      end
    end
  else
    -- clear selection by clicking off waypoint.
    self:deselect()
  end

  -- if autoCameraFocus then
  --   self:setOrbitCameraToSelectedPacenote()
  -- end
end

function C:onPacenoteDrag(pn_sel)
  self.path:setAdjacentNotes(pn_sel.id)

  self:autofillDistanceCalls()

  local pn_prev = pn_sel.prevNote
  if pn_prev then
    pn_prev:refreshStructured()
  end

  pn_sel:refreshStructured()

  local pn_next = pn_sel.nextNote
  if pn_next then
    pn_next:refreshStructured()
  end
end

function C:handleMouseHold()
  local mouse_pos = self.mouseInfo._holdPos

  -- this sphere indicates the drag cursor
  -- debugDrawer:drawSphere((mouse_pos), 1, ColorF(1,1,0,1.0)) -- radius=1, color=yellow

  local wp_sel = self:selectedWaypoint()
  local pn_sel = self:selectedPacenote()

  if wp_sel and not wp_sel:isLocked() and not self.pacenoteToolsState.internal_lock then
    if self.mouseInfo.rayCast then
      local new_pos, normal_align_pos = self:wpPosForSimpleDrag(wp_sel, mouse_pos, self.simpleDragMouseOffset)
      if new_pos then
        local pn_sel = self:selectedPacenote()
        pn_sel:clearTodo()

        wp_sel.pos = new_pos
        self:onPacenoteDrag(pn_sel)

        if normal_align_pos then
          local rv = rallyUtil.calculateForwardNormal(new_pos, normal_align_pos)
          wp_sel.normal = vec3(rv.x, rv.y, rv.z)
        end

        if wp_sel:isCs() then
          local wp_at = pn_sel:getActiveFwdAudioTrigger()

          local point_cs = self.pacenoteToolsState.snaproad:closestSnapPoint(wp_sel.pos)
          local point_at = self.pacenoteToolsState.snaproad:closestSnapPoint(wp_at.pos, true)

          if point_cs.id <= point_at.id then
            point_at = self.pacenoteToolsState.snaproad:pointsBackwards(point_cs, 1)
            wp_at.pos = point_at.pos

            local normalVec = self.pacenoteToolsState.snaproad:forwardNormalVec(point_at)
            if normalVec then
              wp_at:setNormal(normalVec)
            end
          end
        end

      end
    end
  end
end

function C:handleMouseUp()
  self.pacenoteToolsState.internal_lock = false

  local wp_sel = self:selectedWaypoint()
  if wp_sel and not wp_sel.missing then
    editor.history:commitAction("Manipulated Note Waypoint via SimpleDrag",
      {
        self = self, -- the rallyEditor pacenotes tab
        pacenote_idx = self.pacenoteToolsState.selected_pn_id,
        wp_id = self.pacenoteToolsState.selected_wp_id,
        old = self.beginSimpleDragNoteData,
        new = wp_sel:onSerialize(),
        wasPWselection = self.wasWPSelected,
      },
      function(data) -- undo
        local notebook = data.self.path
        local pacenote = notebook.pacenotes.objects[data.pacenote_idx]
        local wp = pacenote.pacenoteWaypoints.objects[data.wp_id]
        wp:onDeserialized(data.old)
        data.self:selectWaypoint(data.wp_id)
      end,
      function(data) --redo
        local notebook = data.self.path
        local pacenote = notebook.pacenotes.objects[data.pacenote_idx]
        local wp = pacenote.pacenoteWaypoints.objects[data.wp_id]
        wp:onDeserialized(data.new)
        data.self:selectWaypoint(data.wp_id)
      end
    )
  end
end

function C:setHover(wp)
  self.pacenoteToolsState.hover_wp_id = nil
  self.pacenoteToolsState.shift = false

  if wp then
    self.pacenoteToolsState.hover_wp_id = wp.id

    if editor.keyModifiers.shift then
      self.pacenoteToolsState.shift = true
      if not self:selectedPacenote() or not self:selectedWaypoint() then
        local pos_rayCast = wp.pos
        local clr_txt = cc.clr_black
        local clr_bg = cc.clr_white
        -- debugDrawer:drawTextAdvanced(
        --   pos_rayCast,
        --   String("Lock Camera"),
        --   ColorF(clr_txt[1],clr_txt[2],clr_txt[3],1),
        --   true,
        --   false,
        --   ColorI(clr_bg[1]*255, clr_bg[2]*255, clr_bg[3]*255, 255)
        -- )
      end
    end
  end
end

function C:handleUnmodifiedMouseInteraction(hoveredWp)
  if self.mouseInfo.down then
    self:handleMouseDown(hoveredWp)
  elseif self.mouseInfo.hold then
    self:handleMouseHold()
  elseif self.mouseInfo.up then
    self:handleMouseUp()
  else
    self:setHover(hoveredWp)
  end
end

function C:handleMouseInput()
  if not self.mouseInfo.valid then return end

  -- handle positioning and drawing of the gizmo
  -- if self.pacenote_tools_state.drag_mode == dragModes.gizmo then
  --   self:updateGizmoTransform(self.pacenote_tools_state.selected_wp_id)
  --   editor.drawAxisGizmo()
  -- else
  --   self:resetGizmoTransformToOrigin()
  -- end
  self:resetGizmoTransformToOrigin()
  editor.updateAxisGizmo(function() self:beginDrag() end, function() self:endDragging() end, function() self:dragging() end)

  self.pacenoteToolsState.hover_wp_id = nil -- clear hover state

  -- There is a bug (in race tool as well) where if you start the game, open
  -- the world editor, and try to use the tool without having selected anything
  -- in Object Select mode (named "Manipulate Object(s)"), then the below line
  -- (which I copied from race tool) will cause the tool not to respond to
  -- mouse interactions.
  --
  -- The Line (which I have commented):
  -- if editor.isAxisGizmoHovered() then return end
  --
  -- Here's the underlying call that editor.isAxisGizmoHovered() uses from gizmo.lua:
  --
  --     -- Return true if the axis gizmo has any hovered elements (axes).
  --     local function isAxisGizmoHovered()
  --       return worldEditorCppApi.getAxisGizmoSelectedElement() ~= -1
  --     end
  --
  -- Turns out that worldEditorCppApi.getAxisGizmoSelectedElement() returns 6
  -- after a cold world editor start. Well, since I'm not using the gizmo for
  -- this tool, I'm just going to comment it and hope for the best.

  local hoveredWp = self:detectMouseHoverWaypoint()

  if editor.keyModifiers.ctrl then
    if not self:selectedPacenote() then
      self:createMouseDragPacenote()
    end
  elseif self.pacenoteToolsState.snaproad and self.pacenoteToolsState.snaproad:driveline() then
    if self.pacenoteToolsState.snaproad:partitionAllEnabled() then
      self.pacenoteToolsState.snaproad:clearAll()
    end
    self:handleUnmodifiedMouseInteraction(hoveredWp)
  end
end

function C:draw(mouseInfo)
  self.mouseInfo = mouseInfo
  if self.rallyEditor.allowGizmo() then
    self:handleMouseInput()
  end

  self:drawPacenotesList()
end

function C:debugDrawNewPacenote(pos_cs, pos_ce)
  local defaultRadius = self.rallyEditor.getPrefDefaultRadius()
  local radius = defaultRadius

  local alpha = cc.new_pacenote_cursor_alpha
  local clr_link = cc.new_pacenote_cursor_clr_link
  local clr_cs = cc.new_pacenote_cursor_clr_cs
  local clr_ce = cc.new_pacenote_cursor_clr_ce
  debugDrawer:drawSphere((pos_cs), radius, ColorF(clr_cs[1],clr_cs[2],clr_cs[3],alpha))
  debugDrawer:drawSphere((pos_ce), radius, ColorF(clr_ce[1],clr_ce[2],clr_ce[3],alpha))

  local fromHeight = radius * cc.new_pacenote_cursor_linkHeightRadiusShinkFactor
  local toHeight = radius * cc.new_pacenote_cursor_linkHeightRadiusShinkFactor
  debugDrawer:drawSquarePrism(
    pos_cs,
    pos_ce,
    Point2F(fromHeight, cc.new_pacenote_cursor_linkFromWidth),
    Point2F(toHeight, cc.new_pacenote_cursor_linkToWidth),
    ColorF(clr_link[1],clr_link[2],clr_link[3],alpha)
  )
end

function C:debugDrawNewPacenote2(pos_cs, pos_ce)
  local defaultRadius = self.rallyEditor.getPrefDefaultRadius()
  local radius = defaultRadius

  local alpha = cc.new_pacenote_cursor_alpha
  local clr_link = cc.new_pacenote_cursor_clr_link
  local clr_cs = cc.new_pacenote_cursor_clr_cs
  local clr_ce = cc.new_pacenote_cursor_clr_ce
  debugDrawer:drawSphere(pos_cs, radius, ColorF(clr_cs[1],clr_cs[2],clr_cs[3],alpha))
  debugDrawer:drawSphere(pos_ce, radius, ColorF(clr_ce[1],clr_ce[2],clr_ce[3],alpha))

  local fromHeight = radius * cc.new_pacenote_cursor_linkHeightRadiusShinkFactor
  local toHeight = radius * cc.new_pacenote_cursor_linkHeightRadiusShinkFactor
  debugDrawer:drawSquarePrism(
    pos_cs,
    pos_ce,
    Point2F(fromHeight, cc.new_pacenote_cursor_linkFromWidth),
    Point2F(toHeight, cc.new_pacenote_cursor_linkToWidth),
    ColorF(clr_link[1],clr_link[2],clr_link[3],alpha)
  )
end

function C:createMouseDragPacenote()
  if not self.path then return end
  if not self.mouseInfo.rayCast then return end
  if not self.pacenoteToolsState.snaproad then return end

  if not self.pacenoteToolsState.snaproad:partitionAllEnabled() then
    self.pacenoteToolsState.snaproad:partitionAllPacenotes(self.path)
    self.pacenoteToolsState.snaproad:setFilterToAllPartitions()
  end

  local txt = "Click to Create New Pacenote"

  local pos_rayCast = self.mouseInfo.rayCast.pos
  pos_rayCast = self.pacenoteToolsState.snaproad:closestSnapPos(pos_rayCast)

  -- local pos_ce = self.mouseInfo._holdPos
  -- if pos_ce then
  --   pos_ce = self.pacenote_tools_state.snaproad:closestSnapPos(pos_ce)
  -- end

  -- draw the cursor text
  local clr_txt = cc.clr_black
  local clr_bg = cc.clr_green
  debugDrawer:drawTextAdvanced(
    pos_rayCast,
    String(txt),
    ColorF(clr_txt[1],clr_txt[2],clr_txt[3],1),
    true,
    false,
    ColorI(clr_bg[1]*255, clr_bg[2]*255, clr_bg[3]*255, 255)
  )

  if self.mouseInfo.down then
    local pos_down = self.mouseInfo._downPos
    if pos_down then
      local point_cs = self.pacenoteToolsState.snaproad:closestSnapPoint(pos_down)
      if point_cs then
        local partition = point_cs.partition
        if not partition then
          log('W', logTag, 'found no snaproad partition for create')
          return
        end

        local pn = partition.pacenote_after
        local lastPacenote = self.path.pacenotes.sorted[#self.path.pacenotes.sorted]
        local sortOrder = 1
        if lastPacenote and not lastPacenote.missing then
          sortOrder = lastPacenote.sortOrder + 5 -- default to end of pacenotes list
        end
        if pn then
          sortOrder = pn.sortOrder - 0.5 -- go before the partition's pacenote_after
        end

        self.pacenoteToolsState.snaproad:setFilterPartitionPoint(point_cs)

        if point_cs.id == partition[1].id then
          if point_cs.next then
            point_cs = point_cs.next
          end
        end

        if point_cs.id == partition[#partition].id then
          if point_cs.prev then
            point_cs = point_cs.prev
          end
        end

        local defaultDistMeters = 10
        local point_ce = self.pacenoteToolsState.snaproad:distanceForwards(point_cs, defaultDistMeters)
        local point_at = self.pacenoteToolsState.snaproad:distanceBackwards(point_cs, defaultDistMeters)

        local newPacenote = self.path.pacenotes:create(nil, nil)
        newPacenote.sortOrder = sortOrder
        local wp_cs = newPacenote.pacenoteWaypoints:create('corner start', point_cs.pos)
        local wp_ce = newPacenote.pacenoteWaypoints:create('corner end', point_ce.pos)
        local wp_at = newPacenote.pacenoteWaypoints:create('audio trigger', point_at.pos)

        local normalVec = self.pacenoteToolsState.snaproad:forwardNormalVec(point_at)
        if normalVec then
          wp_at:setNormal(normalVec)
        end

        normalVec = self.pacenoteToolsState.snaproad:forwardNormalVec(point_cs)
        if normalVec then
          wp_cs:setNormal(normalVec)
        end

        normalVec = self.pacenoteToolsState.snaproad:forwardNormalVec(point_ce)
        if normalVec then
          wp_ce:setNormal(normalVec)
        end

        self.path.pacenotes:sort()
        self.path:cleanupPacenoteNames()
        self.pacenoteToolsState.snaproad:clearAll()
        self:autofillDistanceCalls()
        self:selectPacenote(newPacenote.id)
      end
    end
  end
end

-- figures out which pacenote to select with the mouse in the 3D scene.
function C:detectMouseHoverWaypoint()
  if not self.path then return end
  if not self.path.pacenotes then return end

  local min_note_dist = 4294967295
  local hover_wp = nil
  local selected_pacenote_i = -1
  local waypoints = {}
  local radius_factors = {}

  -- figure out which waypoints are available to select.
  for i, pacenote in ipairs(self.path.pacenotes.sorted) do
    -- if a pacenote is selected, then we can only select it's waypoints.
    if self:selectedPacenote() and self:selectedPacenote().id == pacenote.id then
      selected_pacenote_i = i
      for _,waypoint in ipairs(pacenote.pacenoteWaypoints.sorted) do
        if waypoint:isAt() and editor_rallyEditor.getPrefShowAudioTriggers() then
          table.insert(waypoints, waypoint)
        elseif (waypoint:isCs() or waypoint:isCe()) and not waypoint:isLocked() then
          table.insert(waypoints, waypoint)
        end
      end
    elseif not self:selectedPacenote() then
    -- if no waypoint is selected (ie at the PacenoteSelected mode), we can select any corner start.
      local waypoint = pacenote:getCornerStartWaypoint()
      table.insert(waypoints, waypoint)
    elseif not self:selectedWaypoint() then
    -- if no waypoint is selected (ie at the PacenoteSelected mode), we can select any corner start.
      local waypoint = pacenote:getCornerStartWaypoint()
      radius_factors[waypoint.id] = cc.pacenote_adjacent_radius_factor
      table.insert(waypoints, waypoint)
    end
  end

  -- add waypoints from the previous pacenote.
  if editor_rallyEditor.getPrefShowPreviousPacenote() then
    local prev_i = selected_pacenote_i - 1
    if prev_i > 0 and self:selectedWaypoint() then
      local pn_prev = self.path.pacenotes.sorted[prev_i]
      for _,waypoint in ipairs(pn_prev.pacenoteWaypoints.sorted) do
        if not waypoint:isLocked() then
          radius_factors[waypoint.id] = cc.pacenote_adjacent_radius_factor
          table.insert(waypoints, waypoint)
        end
      end
    end
  end

  -- add waypoints from the next pacenote.
  if editor_rallyEditor.getPrefShowNextPacenote() then
    local next_i = selected_pacenote_i + 1
    if next_i <= #self.path.pacenotes.sorted and self:selectedWaypoint() then
      local pn_next = self.path.pacenotes.sorted[next_i]
      for _,waypoint in ipairs(pn_next.pacenoteWaypoints.sorted) do
        if not waypoint:isLocked() then
          radius_factors[waypoint.id] = cc.pacenote_adjacent_radius_factor
          table.insert(waypoints, waypoint)
        end
      end
    end
  end

  -- of the available waypoints, figure out the closest one.
  for _, waypoint in ipairs(waypoints) do
    local distNoteToCam = (waypoint.pos - self.mouseInfo.camPos):length()
    local noteRayDistance = (waypoint.pos - self.mouseInfo.camPos):cross(self.mouseInfo.rayDir):length() / self.mouseInfo.rayDir:length()
    local sphereRadius =  waypoint.radius
    if radius_factors[waypoint.id] then
      sphereRadius = sphereRadius * radius_factors[waypoint.id]
    end
    if noteRayDistance <= sphereRadius then
      if distNoteToCam < min_note_dist then
        min_note_dist = distNoteToCam
        hover_wp = waypoint
      end
    end
  end

  return hover_wp
end

local function offsetMousePosWithTerrainZSnap(pos, offset)
  local newPos = pos - offset
  local rv = core_terrain.getTerrainHeight(pos)
  if rv then
    newPos.z = core_terrain.getTerrainHeight(pos)
  end
  return newPos
end

-- returns new position for the drag, and another position for orienting the normal perpendicularly.
function C:wpPosForSimpleDrag(wp, mousePos, mouseOffset)
  if self.pacenoteToolsState.snaproad then
    if self.mouseInfo.rayCast then
      local newPos = offsetMousePosWithTerrainZSnap(mousePos, mouseOffset)
      local newPoint = self.pacenoteToolsState.snaproad:closestSnapPoint(newPos)
      local prevPoint, currPoint, nextPoint = self.pacenoteToolsState.snaproad:normalAlignPoints(newPoint)
      if newPoint and nextPoint then
        return newPoint.pos, nextPoint.pos
      else
        return nil, nil
      end
    else
      return nil, nil
    end
  else
    log('W', logTag, 'wpPosForSimpleDrag hit the else when should no hit else')
    return nil, nil
  end
end

-- local function movePacenoteUndo(data)
--   data.self.path.pacenotes:move(data.index, -data.dir)
-- end
-- local function movePacenoteRedo(data)
--   data.self.path.pacenotes:move(data.index,  data.dir)
-- end
-- local function moveWaypointUndo(data)
--   data.self:selectedPacenote().pacenoteWaypoints:move(data.index, -data.dir)
-- end
-- local function moveWaypointRedo(data)
--   data.self:selectedPacenote().pacenoteWaypoints:move(data.index,  data.dir)
-- end

-- local function setPacenoteFieldUndo(data)
--   data.self.path.pacenotes.objects[data.index][data.field] = data.old
--   data.self.path:sortPacenotesByName()
-- end
-- local function setPacenoteFieldRedo(data)
--   data.self.path.pacenotes.objects[data.index][data.field] = data.new
--   data.self.path:sortPacenotesByName()
-- end

-- local function setWaypointFieldUndo(data)
--   data.self:selectedPacenote().pacenoteWaypoints.objects[data.index][data.field] = data.old
--   data.self:updateGizmoTransform(data.index)
-- end
-- local function setWaypointFieldRedo(data)
--   data.self:selectedPacenote().pacenoteWaypoints.objects[data.index][data.field] = data.new
--   data.self:updateGizmoTransform(data.index)
-- end

-- local function setWaypointNormalUndo(data)
--   local wp = data.self:selectedPacenote().pacenoteWaypoints.objects[data.index]
--   if wp then
--     wp:setNormal(data.old)
--   end
--   data.self:updateGizmoTransform(data.index)
-- end
-- local function setWaypointNormalRedo(data)
--   local wp = data.self:selectedPacenote().pacenoteWaypoints.objects[data.index]
--   if wp and not wp.missing then
--     wp:setNormal(data.new)
--   end
--   data.self:updateGizmoTransform(data.index)
-- end

function C:deleteSelected()
  if self:selectedPacenote() then
    self:deleteSelectedPacenote()
  end
end

function C:deleteSelectedPacenote(shouldSelect)
  if not self.path then return end

  shouldSelect = (shouldSelect == nil) and true

  local pn = self:selectedPacenote()
  local toSelect = pn.prevNote
  if not toSelect then
    toSelect = pn.nextNote
  end

  local notebook = self.path
  notebook.pacenotes:remove(self.pacenoteToolsState.selected_pn_id)

  if shouldSelect and toSelect then
    self:selectPacenote(toSelect.id)
    self:setOrbitCameraToSelectedPacenote()
  else
    self:selectPacenote(nil)
    self:setOrbitCameraToSelectedPacenote()
  end
end


function C:selectRecentPacenote()
  if not self:selectedPacenote() then
    if self.pacenoteToolsState.recent_selected_pn_id then
      self:selectPacenote(self.pacenoteToolsState.recent_selected_pn_id)
      return true
    end
  end
  return false
end

function C:selectPrevPacenote()
  if not self.path then return end
  if self:cameraPathIsPlaying() then return end

  if self:selectRecentPacenote() then return end

  local curr = self.path.pacenotes.objects[self.pacenoteToolsState.selected_pn_id]
  local sorted = self.path.pacenotes.sorted

  if curr and not curr.missing then
    local prev = nil
    for i = curr.sortOrder-1,1,-1 do
      local pacenote = sorted[i]
      if self:searchPacenoteMatchFn(pacenote) then
        prev = pacenote
        break
      end
    end

    -- wrap around: find the first usable one
    -- if not prev then
    --   for i = #sorted,1,-1 do
    --     local pacenote = sorted[i]
    --     if self:searchPacenoteMatchFn(pacenote) then
    --       prev = pacenote
    --       break
    --     end
    --   end
    -- end

    if prev then
      self:selectPacenote(prev.id)
    end
  else
    -- if no curr, that means no pacenote was selected, so then select the last one.
    for i = 1,#sorted do
      local pacenote = sorted[i]
      if self:searchPacenoteMatchFn(pacenote) then
        self:selectPacenote(pacenote.id)
        break
      end
    end
  end

  self:setOrbitCameraToSelectedPacenote()
end

function C:selectNextPacenote()
  if not self.path then return end
  if self:cameraPathIsPlaying() then return end

  if self:selectRecentPacenote() then return end

  local curr = self.path.pacenotes.objects[self.pacenoteToolsState.selected_pn_id]
  local sorted = self.path.pacenotes.sorted

  if curr and not curr.missing then
    local next = nil
    for i = curr.sortOrder+1,#sorted do
      local pacenote = sorted[i]
      if self:searchPacenoteMatchFn(pacenote) then
        next = pacenote
        break
      end
    end

    -- wrap around: find the first usable one
    -- if not next then
    --   for i = 1,#sorted do
    --     local pacenote = sorted[i]
    --     if self:searchPacenoteMatchFn(pacenote) then
    --       next = pacenote
    --       break
    --     end
    --   end
    -- end

    if next then
      self:selectPacenote(next.id)
    end
  else
    -- if no curr, that means no pacenote was selected, so then select the last one.
    for i = #sorted,1,-1 do
      local pacenote = sorted[i]
      if self:searchPacenoteMatchFn(pacenote) then
        self:selectPacenote(pacenote.id)
        break
      end
    end
  end

  self:setOrbitCameraToSelectedPacenote()
end

function C:refreshPacenotesTab()
  self:loadSnaproad()
  self:refreshPacenotes()
end

function C:insertMode()
  if self:cameraPathIsPlaying() then return end
  self.pacenoteToolsState.insertMode = true
end

function C:validate()
  self.validation_issues = {}
  self.notes_valid = true
  local invalid_notes_count = 0

  for _,note in ipairs(self.path.pacenotes.sorted) do
    note:validate()
    if not note:is_valid() then
      self.notes_valid = false
      invalid_notes_count = invalid_notes_count + 1
    end
  end

  if not self.notes_valid then
    table.insert(self.validation_issues, tostring(invalid_notes_count)..' pacenote(s) have issues')
  end

  local distance_call_issues = 0
  local did_add_first_issue = false

  local prev = nil
  for _,curr in ipairs(self.path.pacenotes.sorted) do
    if prev ~= nil then
      local prev_after = prev:getNoteFieldAfter()
      local curr_before = curr:getNoteFieldBefore()
      if prev_after == '' and curr_before == '' then
        if not did_add_first_issue then
          table.insert(self.validation_issues, 'missing distance call for '..prev.name..' -> '..curr.name..'. Use "#" if you want none.')
          did_add_first_issue = true
        end
        distance_call_issues = distance_call_issues + 1
      end
    end
    prev = curr
  end
  -- end

  if distance_call_issues >= 2 then
    table.insert(self.validation_issues, 'missing distance calls for '..(distance_call_issues-1)..' more pacenotes.')
  end
end

function C:deleteAllPacenotes()
  if not self.path then return end
  self.path:deleteAllPacenotes()
  self:selectPacenote(nil)
  -- self.pacenote_tools_state.snaproad:clearFilter()
  -- self.pacenote_tools_state.snaproad:clearPartition()
end

function C:drawPacenotesList()
  if not self.path then return end

  local notebook = self.path
  self:validate()

  if self:isValid() then
    im.HeaderText(tostring(#notebook.pacenotes.sorted).." Pacenotes")
  else
    im.HeaderText("[!] "..tostring(#notebook.pacenotes.sorted).." Pacenotes")
  end

  if im.Button("Refresh") then
    self:refreshPacenotesTab()
  end
  im.tooltip("Force a refresh of distance calls and punctuation.")

  im.SameLine()
  if im.Button("Select Closest to Vehicle") then
    local playerVehicle = getPlayerVehicle(0)
    if playerVehicle then
      local pacenotes = self.path:findNClosestPacenotes(playerVehicle:getPosition(), 1)
      if pacenotes and pacenotes[1] then
        self:selectPacenote(pacenotes[1].id)
        self:setOrbitCameraToSelectedPacenote()
      end
    end
  end

  im.SameLine()
  if im.Button("Delete All") then
    im.OpenPopup("Delete All")
  end
  im.tooltip("Delete all pacenotes from this notebook.")
  if im.BeginPopupModal("Delete All", nil, im.WindowFlags_AlwaysAutoResize) then
    im.Text("Delete all pacenotes?")
    im.Separator()
    if im.Button("Ok", im.ImVec2(120,0)) then
      self:deleteAllPacenotes()
      im.CloseCurrentPopup()
    end
    im.SameLine()
    if im.Button("Cancel", im.ImVec2(120,0)) then
      im.CloseCurrentPopup()
    end
    im.EndPopup()
  end

  im.SameLine()

  if im.Button("Mark All TODO") then
    self.path:markAllTodo()
  end
  im.SameLine()
  if im.Button("Mark Rest TODO") then
    self.path:markRestTodo(self:selectedPacenote())
  end
  im.SameLine()
  if im.Button("Mark All Done") then
    self.path:clearAllTodo()
  end

  -- local editModeLabel = '???'
  -- if self.pacenoteToolsState.mode == editModes.editAll then
  --   editModeLabel = "All"
  -- elseif self.pacenoteToolsState.mode == editModes.editCorners then
  --   editModeLabel = "Corners"
  -- elseif self.pacenoteToolsState.mode == editModes.editAT then
  --   editModeLabel = "Audio Triggers"
  -- end
  -- im.HeaderText("Edit Mode: "..)

  im.Spacing()
  im.Columns(2)

  -- if im.Button("Change Edit Mode") then
  --   self:cycleEditMode()
  -- end
  -- im.SameLine()
  -- im.Text("Mode: "..editModeLabel)


  -- im.HeaderText("Search")

  -- local editEnded = im.BoolPtr(false)

  -- -- im.SetNextItemWidth(200)
  -- editor.uiInputText("##SearchPn", pacenotesSearchText, nil, nil, nil, nil, editEnded)
  -- if editEnded[0] then
  --   self.pacenoteToolsState.search = ffi.string(pacenotesSearchText)

  --   if rallyUtil.trimString(self.pacenoteToolsState.search) == '' then
  --     self.pacenoteToolsState.search = nil
  --   end

  --   if self.pacenoteToolsState.search then
  --     log('D', logTag, 'searching pacenotes: '..self.pacenoteToolsState.search)
  --     local pn = self:selectedPacenote()
  --     if pn then
  --       if not self:pacenoteSearchMatches(pn) then
  --         self:selectNextPacenote()
  --         local pn2 = self:selectedPacenote()
  --         if pn2 and pn.id == pn2.id then
  --           self:selectPrevPacenote()
  --         end
  --       end
  --     end
  --   end
  -- end
  -- im.SameLine()
  -- if im.Button("X") then
  --   self.pacenoteToolsState.search = nil
  --   pacenotesSearchText = im.ArrayChar(1024, "")
  -- end

  -- if im.Button("Prev") then
  --     self:selectPrevPacenote()
  -- end
  -- im.SameLine()
  -- if im.Button("Next") then
  --     self:selectNextPacenote()
  -- end

  if self:isValid() then
    im.TextColored(cc.clr_no_error, "No issues")
  else
    local issuesHeading = "Issues (".. (#self.validation_issues) ..")"
    im.TextColored(cc.clr_error, issuesHeading)
    local issues = ""
    for _, issue in ipairs(self.validation_issues) do
      issues = issues..'- '..issue..'\n'
    end
    im.PushStyleColor2(im.Col_Text, cc.clr_error)
    im.tooltip(issues)
    im.PopStyleColor()
  end

  -- im.HeaderText("Selected Pacenote")
  -- im.BeginChild1("pacenotes", im.ImVec2(270*im.uiscale[0], 0), im.WindowFlags_ChildWindow)
  im.BeginChild1("pacenotes", nil, im.WindowFlags_ChildWindow)
  -- im.BeginChild1("pacenotes", nil, im.WindowFlags_ChildWindow)
  for i, note in ipairs(notebook.pacenotes.sorted) do
    if not note:is_valid() then
      im.PushStyleColor2(im.Col_Text, cc.clr_error)
    end
    if im.Selectable1(note:pacenoteTextForSelect(), note.id == self.pacenoteToolsState.selected_pn_id) then
      self:selectPacenote(note.id)
      self:setOrbitCameraToSelectedPacenote()
    end
    if not note:is_valid() then
      im.PopStyleColor()
    end

    local tooltipText = ''
    if self.path:useStructured() then
      tooltipText = dumps(note:noteOutputStructured())
    else
      tooltipText = note:getNoteFieldFreeform()
    end
    tooltipText = tooltipText..'\n\n'..note:triggerTypeAsText()
    if note:slowCornerAsText() ~= '' then
      tooltipText = tooltipText..'\n\n'..note:slowCornerAsText()
    end
    if note:is_valid() then
      im.tooltip(tooltipText)
    else
      im.tooltip("[!] Found "..(#note.validation_issues).." issue(s).\n"..tooltipText)
    end
  end
  im.EndChild() -- pacenotes child window
  im.SameLine()

  im.NextColumn()

  local pacenote = self.path.pacenotes.objects[self.pacenoteToolsState.selected_pn_id]
  self.pacenoteForm:draw()

  im.Columns(1)
end

function C:searchPacenoteMatchFn(pacenote)
  return pacenote and not pacenote.missing and self:pacenoteSearchMatches(pacenote)
end

function C:pacenoteSearchMatches(pacenote)
  if not self.pacenoteToolsState.search then return true end

  local searchPattern = rallyUtil.trimString(self.pacenoteToolsState.search)
  if searchPattern == '' then return true end

  return pacenote:matchesSearchPattern(searchPattern)
end

-- function C:handleNoteFieldEdit(note, language, subfield, buf)
--   local newVal = note.notes
--   local lang_data = newVal[language] or {}
--   local val = rallyUtil.trimString(ffi.string(buf))

--   if subfield == 'note' then
--     local last = note.id == self.path.pacenotes.sorted[#self.path.pacenotes.sorted].id
--     val = note:normalizeNoteText(language, last, false, val)
--     val = { freeform = val }
--   end

--   lang_data[subfield] = val
--   newVal[language] = lang_data

--   -- editor.history:commitAction("Change Notes of Pacenote",
--   --   {
--   --     index = self.pacenoteToolsState.selected_pn_id,
--   --     self = self,
--   --     old = note.notes,
--   --     new = newVal,
--   --     field = 'notes'
--   --   },
--   --   setPacenoteFieldUndo,
--   --   setPacenoteFieldRedo
--   -- )

--   -- self.path.pacenotes.objects[self.pacenoteToolsState.selected_pn_id].notes = newVal
--   self:selectedPacenote().notes = newVal
-- end

function C:cleanupPacenoteNames()
  if not self.path then return end

  editor.history:commitAction("Cleanup pacenote names",
    {
      self = self,
      notebook = self.path,
      old_pacenotes = deepcopy(self.path.pacenotes:onSerialize()),
    },
    function(data) -- undo
      data.notebook.pacenotes:onDeserialized(data.old_pacenotes, {})
    end,
    function(data) -- redo
      data.self:selectPacenote(nil)
      data.notebook:cleanupPacenoteNames()
    end
  )
end

function C:autofillDistanceCalls()
  if not self.path then return end
  -- log('D', logTag, 'autofilling distance calls')
  self.path:autofillDistanceCalls()
end

function C:refreshPacenotes()
  if not self.path then return end
  log('D', logTag, 'refreshing')

  self:cleanupPacenoteNames()
  self.path:refreshAllPacenotes()
  self:autofillDistanceCalls()
end

function C:placeVehicleAtPacenote()
  local pos, rot = self:selectedPacenote():vehiclePlacementPosAndRot()

  if pos and rot then
    local playerVehicle = getPlayerVehicle(0)
    if playerVehicle then
      spawn.safeTeleport(playerVehicle, pos, rot)
    end
  end
end

function C:insertNewPacenoteAfter(note)
  if not self.path then return end

  local pn_next = nil

  for i,pn in ipairs(self.path.pacenotes.sorted) do
    if pn.id == note.id then
      pn_next = i+1
    end
  end

  local _, numA = note:nameComponents()
  numA = tonumber(numA)
  local nextNum = numA

  if pn_next <= #self.path.pacenotes.sorted then
    local next_note = self.path.pacenotes.sorted[pn_next]
    if next_note then
      local _, numB = next_note:nameComponents()
      numB = tonumber(numB)
      nextNum = numA + ((numB - numA) / 2)
    end
  else
    nextNum = numA+1
  end

  -- local currId = self:selectedPacenote().id
  -- num = tonumber(num) + 0.01

  local newPacenote = self.path.pacenotes:create("Pacenote "..tostring(nextNum))
  self.path:sortPacenotesByName()
  -- self:cleanupPacenoteNames()
  -- self:selectPacenote(currId)
end

function C:selectNextWaypoint()
  if self:cameraPathIsPlaying() then return end

  -- if there's no selected PN, select the recent one.

  -- if not pn then
  --   if self.pacenote_tools_state.recent_selected_pn_id then
  --     self:selectPacenote(self.pacenote_tools_state.recent_selected_pn_id)
  --   end
  --   return
  -- end

  -- if self:selectRecentPacenote() then return end
  self:selectRecentPacenote()

  local pn = self:selectedPacenote()
  if not pn then return end

  local wp_sel = self:selectedWaypoint()

  if wp_sel then
    local wp_new = nil
    if wp_sel:isAt() then
      wp_new = pn:getCornerStartWaypoint()
    elseif wp_sel:isCs() then
      wp_new = pn:getCornerEndWaypoint()
    elseif wp_sel:isCe() then
      if editor_rallyEditor.getPrefShowAudioTriggers() then
        wp_new = pn:getActiveFwdAudioTrigger()
      else
      wp_new = pn:getCornerStartWaypoint()
      end
      -- if not wp_new then
      --   wp_new = pn:getCornerStartWaypoint()
      -- end
    end

    if wp_new and not wp_new:isLocked() then
      self:selectWaypoint(wp_new.id)
    end
  else
    local wp = nil
    if editor_rallyEditor.getPrefShowAudioTriggers() then
      wp = pn:getActiveFwdAudioTrigger()
    else
      wp = pn:getCornerStartWaypoint()
    end
    -- if not wp then
    --   wp = pn:getCornerStartWaypoint()
    -- end
    self:selectWaypoint(wp.id)
  end
end

function C:_moveSelectedWaypointHelper(fwd, steps)
  local wp = self:selectedWaypoint()
  if not wp then
    self:selectNextWaypoint()
    wp = self:selectedWaypoint()
    return
  end

  if self.pacenoteToolsState.snaproad then
    local pn = self:selectedPacenote()
    pn:clearTodo()
    pn:moveWaypointTowards(self.pacenoteToolsState.snaproad, wp, fwd, steps)
  end
end

function C:moveSelectedWaypointForward(steps)
  if self:cameraPathIsPlaying() then return end
  steps = steps or 1
  self:_moveSelectedWaypointHelper(true, steps)
end

function C:moveSelectedWaypointBackward(steps)
  if self:cameraPathIsPlaying() then return end
  steps = steps or 1
  self:_moveSelectedWaypointHelper(false, steps)
end

function C:cameraPathPlay()
  -- local snaproadType = editor.getPreference("rallyEditor.editing.preferredSnaproadType")
  -- if snaproadType == 'route' then
    -- TODO camera path playback not supported for route yet.
    -- return
  -- end

  if self.pacenoteToolsState.snaproad and self.pacenoteToolsState.snaproad:isRouteSourced() then
    return
  end

  if self:cameraPathIsPlaying() then
    self.pacenoteToolsState.snaproad:stopCameraPath()
    self.pacenoteToolsState.playbackLastCameraPos = core_camera.getPosition()
    core_camera.setPosition(0, self.pacenoteToolsState.last_camera.pos)
    core_camera.setRotation(0, self.pacenoteToolsState.last_camera.quat)
    self:selectPacenote(self:selectedPacenote().id)
  else
    self:selectWaypoint(nil)
    self.pacenoteToolsState.last_camera.pos = core_camera.getPosition()
    self.pacenoteToolsState.last_camera.quat = core_camera.getQuat()
    self.pacenoteToolsState.snaproad:playCameraPath()
  end
end

function C:cameraPathIsPlaying()
  return core_camera.getActiveCamName() == "path"
end

function C:toggleCornerCalls()
  local snaproadType = editor.getPreference("rallyEditor.editing.preferredSnaproadType")
  if snaproadType == 'route' then
    -- TODO corner calls not supported for route yet.
    return
  end

  if self.pacenoteToolsState.snaproad then
    self.pacenoteToolsState.snaproad:toggleCornerCalls()
  end
end

function C:setModeEditAll()
  self.pacenoteToolsState.mode = editModes.editAll
  editor_rallyEditor.setPrefLockWaypoints(false)
  editor_rallyEditor.setPrefShowAudioTriggers(true)
end

function C:setModeEditCorners()
  self.pacenoteToolsState.mode = editModes.editCorners
  editor_rallyEditor.setPrefLockWaypoints(false)
  editor_rallyEditor.setPrefShowAudioTriggers(false)
end

function C:setModeEditAudioTrigger()
  self.pacenoteToolsState.mode = editModes.editAT
  editor_rallyEditor.setPrefLockWaypoints(true)
  editor_rallyEditor.setPrefShowAudioTriggers(true)
end

function C:cycleEditMode()
  self:selectWaypoint(nil)

  if self.pacenoteToolsState.mode == editModes.editAll then
    self:setModeEditCorners()
  elseif self.pacenoteToolsState.mode == editModes.editCorners then
    self:setModeEditAudioTrigger()
  elseif self.pacenoteToolsState.mode == editModes.editAT then
    self:setModeEditAll()
  else
    self:setModeEditCorners()
  end

  local startAtCs = self.pacenoteToolsState.mode == editModes.editCorners
  if self.pacenoteToolsState.snaproad and self:selectedPacenote() then
    self.pacenoteToolsState.snaproad:setPartitionToPacenote(self:selectedPacenote(), startAtCs)
  end
end

function C:mergeSelectedWithPrevPacenote()
  local pn = self:selectedPacenote()
  if not pn then return end

  local pnPrev = pn.prevNote
  if not pnPrev then return end

  local currText = pn:getNoteFieldFreeform()
  local prevText = pnPrev:getNoteFieldFreeform()
  local mergedText = prevText..' '..currText

  pnPrev:setNoteFieldFreeform(mergedText)
  self:deleteSelectedPacenote(false)

  self:selectPacenote(pnPrev.id)
  self:setOrbitCameraToSelectedPacenote()
end

function C:mergeSelectedWithNextPacenote()
  local pn = self:selectedPacenote()
  if not pn then return end

  local pnNext = pn.nextNote
  if not pnNext then return end

  local currText = pn:getNoteFieldFreeform()
  local nextText = pnNext:getNoteFieldFreeform()
  local mergedText = currText..' '..nextText

  pnNext:setNoteFieldFreeform(mergedText)
  self:deleteSelectedPacenote(false)

  self:selectPacenote(pnNext.id)
  self:setOrbitCameraToSelectedPacenote()
end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end
