-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local im  = ui_imgui
local logTag = ''

local RallyUtil = require('/lua/ge/extensions/gameplay/rally/util')
local waypointTypes = require('/lua/ge/extensions/gameplay/rally/notebook/waypointTypes')
local kdTreeP3d = require('kdtreepoint3d')
local RallyEnums = require('/lua/ge/extensions/gameplay/rally/enums')
local Recce = require('/lua/ge/extensions/gameplay/rally/recce')
local cc = require('/lua/ge/extensions/gameplay/rally/util/colors')

local C = {}

local boolPtr

function C:init()
  self.drivelineMode = RallyEnums.drivelineMode.route

  self.nextPacenoteIdxForAudioTriggerTesting = nil
  self.nextPacenoteIdxForAudioTriggerTesting = nil

  self.recce = nil

  self.debug = {
    drawRacePath = false,
    drawRaceSplits = false,
    drawRaceAiRoute = false,
    drawRaceCurrentSeg = true,
    drawStartFinishLines = false,
    drawStopZone = false,
    drawNotebookPacenotes = false,
    drawDrivelineRoute = false,
    drawSelectedWaypoint = false,
    drawKdTreeClosestPacenoteWaypoint = false,
    drawKdTreeClosestRoutePoint = false,
    drawKdTreeNextPacenoteWp = false,
    drawDrivelineRouteStatic = false,
    useMouseRayCast = false,
    enableMouseRayCastMovement = false,
    drawPreRoutePoints = false,
    drawRoutePacenotes = false,
    drawRoutePacenoteText = false,
    drawRoutePathnodes = false,
    drawRoutePointI = false,
    drawRoutePointMetadata = false,
    drawRouteHiddenPathnodes = false,
    drawRouteNextPacenoteWpFromRecalc = false,
    drawRouteNextRacePathnodeFromRecalc = false,
    drawRouteCompletion = true,
    drawRouteShort = true,
    drawReccePacenotes = true,
    drawRecceDrivelinePoints = false
  }
end

function C:setDrivelineMode(drivelineMode)
  self.drivelineMode = drivelineMode
end

function C:raceDistanceKmString()
  local rm = gameplay_rally.getRallyManager()
  if not rm then return 'N/A' end
  local dr = rm:getDrivelineRoute()
  if dr then
    return dr:getRaceDistanceKmString()
  end
end

function C:calcClosestLineSegmentKD()
  if not self.selectedPacenote then return end
  if not self.kdTreeRaceAiPath then return end

  local item_id, dist = self.kdTreeRaceAiPath:findNearest(self.selectedPacenoteWaypoint.pos.x, self.selectedPacenoteWaypoint.pos.y, self.selectedPacenoteWaypoint.pos.z)
  self.kdState = {
    item_id = item_id,
    dist = dist,
  }
  self.lineSegState = {
    minDistSq = math.huge,
    fromPos = nil,
    toPos = nil,
  }

  local item = self.racePathAiPath[item_id]
  local prevItem = self.racePathAiPath[item_id-1]
  local nextItem = self.racePathAiPath[item_id+1]

  if prevItem then
    local prevDist = self.selectedPacenoteWaypoint.pos:squaredDistanceToLineSegment(prevItem, item)
    if prevDist < self.lineSegState.minDistSq then
      self.lineSegState.minDistSq = prevDist
      self.lineSegState.fromPos = prevItem
      self.lineSegState.toPos = item
    end
  end

  if nextItem then
    local nextDist = self.selectedPacenoteWaypoint.pos:squaredDistanceToLineSegment(item, nextItem)
    if nextDist < self.lineSegState.minDistSq then
      self.lineSegState.minDistSq = nextDist
      self.lineSegState.fromPos = item
      self.lineSegState.toPos = nextItem
    end
  end
end

function C:buildKdTreeRaceAiPath()
  local rm = gameplay_rally.getRallyManager()
  if not rm then
    log('E', logTag, 'failed to build kd tree race ai path, rally manager not found')
    return
  end
  local racePath = rm:getRacePath()
  if not racePath then
    log('E', logTag, 'failed to build kd tree race ai path, race path not found')
    return
  end
  racePath:autoConfig()
  local _aiPath, aiDetailedPath = racePath:getAiPath(true)

  self.racePathAiPath = {}

  -- for some reason, have to copy the pos to a new vec3
  for i, item in ipairs(aiDetailedPath) do
    self.racePathAiPath[i] = vec3(item.pos)
  end

  -- Initialize a new empty kdTree (the itemCount argument is optional for space pre-allocation of the items table)
  local kdT = kdTreeP3d.new(#self.racePathAiPath)

  -- Preload items: Populates the self.items table
  for id, item in pairs(self.racePathAiPath) do
    kdT:preLoad(id, item.x, item.y, item.z)
  end

  -- Build the tree: creates the tree from the preloaded items, i.e. it populates the self.tree table
  kdT:build()

  self.kdTreeRaceAiPath = kdT

  -- Range Query: Get all items within a querry box
  -- Two ways to range query the items in the tree. Both query function return iterators (to be used in for .. do constructs)

  -- 1) queries that are not nested
  -- for item_id in kdT:queryNotNested(query_xmin, query_ymin, query_zmin, query_xmax, query_ymax, query_zmax) do
    -- do something with item_id --
  -- end

  -- 2) for nested queries (also works for non nested queries but will be slower and create garbage)
  -- for item1 in kdT:query(query_xmin, query_ymin, query_zmin, query_xmax, query_ymax, query_zmax) do -- for all the items in the tree that intersect the querry area
    -- for item2 in kdT:query(item1_xmin, item1_ymin, item1_xmax, item2_ymax) do
      -- do stuff --
    -- end
  -- end

  -- Point query: get point in tree closest to a query point
  -- local item_id, dist = kdT:findNearest(query_x, query_y, query_z)

end

-- function C:buildKdTreePacenoteWaypoints()
--   local rm = gameplay_rally.getRallyManager()
--   local nb = rm:getNotebookPath()
--   local pacenotes = nb.pacenotes.sorted

--   self.pacenoteWaypoints = {}

--   -- for some reason, have to copy the pos to a new vec3
--   for i, pacenote in ipairs(pacenotes) do
--     local wpCs = pacenote:getCornerStartWaypoint()
--     local wpCe = pacenote:getCornerEndWaypoint()
--     table.insert(self.pacenoteWaypoints, vec3(wpCs.pos))
--     table.insert(self.pacenoteWaypoints, vec3(wpCe.pos))
--   end

--   -- Initialize a new empty kdTree (the itemCount argument is optional for space pre-allocation of the items table)
--   local kdT = kdTreeP3d.new(#self.pacenoteWaypoints)

--   -- Preload items: Populates the self.items table
--   for id, item in pairs(self.pacenoteWaypoints) do
--     kdT:preLoad(id, item.x, item.y, item.z)
--   end

--   -- Build the tree: creates the tree from the preloaded items, i.e. it populates the self.tree table
--   kdT:build()

--   self.kdTreePacenoteWaypoints = kdT
-- end

-- function C:calcClosestPacenoteWaypointKD()
--   if not self.kdTreePacenoteWaypoints then return end

--   local vehicle = getPlayerVehicle(0)
--   if not vehicle then return end
--   local vehiclePos = vehicle:getPosition()

--   local item_id, dist = self.kdTreePacenoteWaypoints:findNearest(vehiclePos.x, vehiclePos.y, vehiclePos.z)
--   self.kdStatePacenoteWaypoints = {
--     item_id = item_id,
--     dist = dist,
--   }
-- end

function C:buildKdTreeDrivelineRouteStatic()
  local rm = gameplay_rally.getRallyManager()
  if not rm then
    log('E', logTag, 'failed to build kd tree driveline route static, rally manager not found')
    return
  end

  local dr = rm:getDrivelineRoute()

  if not dr then
    log('E', logTag, 'failed to build kd tree driveline route static, driveline route not found')
    return
  end

  local route = dr.routeStatic
  self.routeWaypointsIndex = {}

  for i, item in ipairs(route.path) do
    self.routeWaypointsIndex[i] = item
  end

  local kdT = kdTreeP3d.new(#self.routeWaypointsIndex)

  for id, item in pairs(self.routeWaypointsIndex) do
    kdT:preLoad(id, item.pos.x, item.pos.y, item.pos.z)
  end

  kdT:build()
  self.kdTreeDrivelineRoute = kdT
end

function C:calcClosestDrivelineRouteKD()
  if not self.kdTreeDrivelineRoute then return end

  -- local vehicle = getPlayerVehicle(0)
  -- if not vehicle then return end
  -- local vehiclePos = vehicle:getPosition()

  local dr = gameplay_rally.getRallyManager():getDrivelineRoute()
  local pos = dr:getPosition()
  if not pos then return end

  local item_id, dist = self.kdTreeDrivelineRoute:findNearest(pos.x, pos.y, pos.z)
  self.kdStateDrivelineRoute = {
    item_id = item_id,
    dist = dist,
  }
end

function C:calcNextPacenoteWpForDrivelineRoutePoint()
  if not self.kdStateDrivelineRoute then return end

  local point = self.routeWaypointsIndex[self.kdStateDrivelineRoute.item_id]

  local nextWp = nil

  for i = self.kdStateDrivelineRoute.item_id, #self.routeWaypointsIndex do
    local pathPoint = self.routeWaypointsIndex[i]
    local metadata = pathPoint.metadata
    -- if metadata and metadata.pacenoteWaypoint then
    --   local wp = metadata.pacenoteWaypoint
    --     nextWp = wp
    --     break
    -- end
    if metadata then
      if metadata.wpCs then
        nextWp = metadata.wpCs
        break
      elseif metadata.wpCe then
        nextWp = metadata.wpCe
        break
      end
    end
  end

  self.nextPacenoteWp = nextWp
end

function C:getRace()
  local race = gameplay_rally.getRace()
  if not race then return nil end
  local path = race.path
  if path then
    local segments = path.segments.sorted
    if segments and segments[1] and not segments[1].missing and not segments[1]._rallyDebugColor then
      for i, seg in ipairs(segments) do
        -- seg._rallyDebugColor = rainbowColor(#segments, seg.sortOrder-1, 1)
        if i % 2 == 0 then
          seg._rallyDebugColor = {1,0.5,0}
        else
          seg._rallyDebugColor = {1,1,0}
        end
      end
    end
  end
  return race
end

function C:toggleMouseMovementCheckbox()
  self.debug.enableMouseRayCastMovement = not self.debug.enableMouseRayCastMovement
  self:toggleMouseLikeVehicleEnableMovement()
end

function C:toggleMouseLikeVehicleEnableMovement()
  local dr = gameplay_rally.getRallyManager():getDrivelineRoute()
  dr:enableTrackMouseLikeVehicleMovement(self.debug.enableMouseRayCastMovement)
end

function C:draw()
  local rm = gameplay_rally.getRallyManager()
  local missionName = '<none>'
  if rm then
    missionName = rm:getMissionName()
  end
  im.HeaderText("Mission: " .. missionName)

  if rm then
    local dr = rm:getDrivelineRoute()
    if dr then
      if dr and dr:isLoaded() then
        -- im.Text("driveline loaded.")
      else
        im.TextColored(im.ImVec4(1,0,0,1), "driveline load failed!")
      end
      im.Text("Race Distance: " .. dr:getRaceDistanceKmString())
    else
      im.Text("Race Distance: N/A")
    end
  end

  im.Text("gameplay_rally.getDebugLogging()=" .. tostring(gameplay_rally.getDebugLogging()))

  if rm and rm:getDrivelineMode() then
    local modeName = RallyEnums.drivelineModeNames[rm:getDrivelineMode()]
    im.Text("rallyManager.drivelineMode=" .. tostring(modeName))
  end

  im.Separator()

  -- im.SameLine()
  -- if im.Button("Driveline Route Recalc##debugDrivelineRouteRecalc") then
  --   local rm = gameplay_rally.getRallyManager()
  --   if rm then
  --     if not rm:getDrivelineRoute():recalculate() then
  --       log('E', logTag, 'failed to recalculate')
  --     end
  --   end
  -- end
  -- im.SameLine()

  if im.Button("RM Soft Reload##debugRmVhclSoftReload") then
    self:clearReccePoints()
    if rm then
      if not rm:softReload() then
        log('E', logTag, 'failed to softReload RallyManager')
      end
    end
  end
  im.SameLine()
  if im.Button("RM Full Reload##debugRmVhclReload") then
    self:clearReccePoints()
    if rm then
      rm:setDrivelineMode(self.drivelineMode)
      if not rm:reload() then
        log('E', logTag, 'failed to reload RallyManager')
      end
    end
  end

  im.Separator()

  if im.Button("Clear Visual Pacenotes##debugClearVisualPacenotes") then
    if rm then
      rm:triggerClearAllVisualPacenotes()
      rm:resetAudioQueue()
      self.nextPacenoteIdxForAudioTriggerTesting = nil
      self.nextPacenoteIdxForRemoveTesting = nil
    end
  end
  im.SameLine()

  if im.Button("Trigger Next Visual Note##triggerNextVisualNote") then
    if rm then
      local nb = rm:getNotebookPath()
      if nb then
        if not self.nextPacenoteIdxForAudioTriggerTesting then
          self.nextPacenoteIdxForAudioTriggerTesting = 1
        end
        local pacenote = nb.pacenotes.sorted[self.nextPacenoteIdxForAudioTriggerTesting]
        if pacenote and not pacenote.missing then
          rm:triggerShowVisualPacenote(pacenote)
          self.nextPacenoteIdxForAudioTriggerTesting = self.nextPacenoteIdxForAudioTriggerTesting + 1
        else
          log('E', logTag, 'failed to get pacenote, pacenote not found')
        end
      end
    end
  end
  im.SameLine()

  if im.Button("Remove Visual Note##removeNextVisualNote") then
    if rm then
      local nb = rm:getNotebookPath()
      if nb then
        if not self.nextPacenoteIdxForRemoveTesting then
          self.nextPacenoteIdxForRemoveTesting = 1
        end
        local pacenote = nb.pacenotes.sorted[self.nextPacenoteIdxForRemoveTesting]
        if pacenote and not pacenote.missing then
          rm:triggerClearVisualPacenote(pacenote)
          self.nextPacenoteIdxForRemoveTesting = self.nextPacenoteIdxForRemoveTesting + 1
        else
          log('E', logTag, 'failed to get pacenote, pacenote not found')
        end
      end
    end
  end

  im.Separator()

  im.Text("Race Debug")

  boolPtr = im.BoolPtr(self.debug.drawRacePath)
  if im.Checkbox("Pathnodes##debugDrawRacePath", boolPtr) then
    self.debug.drawRacePath = boolPtr[0]
  end
  im.SameLine()
  boolPtr = im.BoolPtr(self.debug.drawRaceSplits)
  if im.Checkbox("Splits##debugDrawRaceSplits", boolPtr) then
    self.debug.drawRaceSplits = boolPtr[0]
  end
  im.SameLine()
  boolPtr = im.BoolPtr(self.debug.drawRaceAiRoute)
  if im.Checkbox("AI Route##debugDrawRaceAiRoute", boolPtr) then
    self.debug.drawRaceAiRoute = boolPtr[0]
  end
  im.SameLine()

  local shouldBeDisabled = not self:getRace()
  if shouldBeDisabled then im.BeginDisabled() end
  boolPtr = im.BoolPtr(self.debug.drawRaceCurrentSeg)
  if im.Checkbox("Current Segment##debugDrawRaceCurrentSeg", boolPtr) then
    self.debug.drawRaceCurrentSeg = boolPtr[0]
  end
  if shouldBeDisabled then im.EndDisabled() end

  boolPtr = im.BoolPtr(self.debug.drawStartFinishLines)
  if im.Checkbox("Start and Finish Lines##debugDrawStartFinishLines", boolPtr) then
    self.debug.drawStartFinishLines = boolPtr[0]
  end
  im.SameLine()
  boolPtr = im.BoolPtr(self.debug.drawStopZone)
  if im.Checkbox("Stop Zone##debugDrawStopZone", boolPtr) then
    self.debug.drawStopZone = boolPtr[0]
  end

  im.Separator()

  im.Text("Notebook/Pacenotes Debug")

  boolPtr = im.BoolPtr(self.debug.drawNotebookPacenotes)
  if im.Checkbox("Pacenotes##debugDrawNotebookPacenotes", boolPtr) then
    self.debug.drawNotebookPacenotes = boolPtr[0]
  end
  im.SameLine()
  boolPtr = im.BoolPtr(self.debug.drawReccePacenotes)
  if im.Checkbox("Recce Pacenotes##debugDrawReccePacenotes", boolPtr) then
    self.debug.drawReccePacenotes = boolPtr[0]
  end

  im.Separator()

  im.Text("Driveline Debug")
  im.SameLine()

  im.Text(" | ")
  im.SameLine()

  im.SetNextItemWidth(100)
  -- local defaultMode = RallyEnums.drivelineModeNames[1]
  local drivelineMode = RallyEnums.drivelineModeNames[self.drivelineMode]

  if rm then
    if rmDrivelineMode then
      drivelineMode = RallyEnums.drivelineModeNames[rmDrivelineMode]
    end
  end

  if im.BeginCombo("Driveline Mode Override##drivelineMode", drivelineMode) then
    for _, mode in ipairs(RallyEnums.drivelineModeNames) do
      if im.Selectable1(mode, mode == drivelineMode) then
        self.drivelineMode = RallyEnums.drivelineMode[mode]
        if rm then
          self:clearReccePoints()
          rm:setDrivelineMode(self.drivelineMode)
          if not rm:reload() then
            log('E', logTag, 'failed to reload RallyManager')
          end
        end
      end
    end
    im.EndCombo()
  end

  boolPtr = im.BoolPtr(self.debug.drawDrivelineRoute)
  if im.Checkbox("Route##debugDrawDrivelineRoute", boolPtr) then
    self.debug.drawDrivelineRoute = boolPtr[0]
  end
  im.SameLine()
  boolPtr = im.BoolPtr(self.debug.drawDrivelineRouteStatic)
  if im.Checkbox("StaticRoute##debugDrawDrivelineRouteStatic", boolPtr) then
    self.debug.drawDrivelineRouteStatic = boolPtr[0]
  end
  im.SameLine()

  boolPtr = im.BoolPtr(self.debug.drawRouteShort)
  if im.Checkbox("Short##debugDrawRouteShort", boolPtr) then
    self.debug.drawRouteShort = boolPtr[0]
  end
  im.SameLine()

  boolPtr = im.BoolPtr(self.debug.drawPreRoutePoints)
  local shouldBeDisabled = not rm or not rm.drivelineRoute or not rm.drivelineRoute.preRoutePoints
  if shouldBeDisabled then
    im.BeginDisabled()
  end
  if im.Checkbox("PreRoute Points##debugPreRoutePoints", boolPtr) then
    self.debug.drawPreRoutePoints = boolPtr[0]
  end
  if shouldBeDisabled then
    im.EndDisabled()
    self.debug.drawPreRoutePoints = false
  end
  im.SameLine()

  boolPtr = im.BoolPtr(self.debug.drawRouteCompletion)
  if im.Checkbox("Completion %##debugDrawRouteCompletion", boolPtr) then
    self.debug.drawRouteCompletion = boolPtr[0]
  end

  boolPtr = im.BoolPtr(self.debug.drawRoutePacenotes)
  if im.Checkbox("Pacenotes##debugDrawRoutePacenotes", boolPtr) then
    self.debug.drawRoutePacenotes = boolPtr[0]
  end
  im.SameLine()

  boolPtr = im.BoolPtr(self.debug.drawRoutePacenoteText)
  if im.Checkbox("Pacenote Text##debugDrawRoutePacenoteText", boolPtr) then
    self.debug.drawRoutePacenoteText = boolPtr[0]
  end

  boolPtr = im.BoolPtr(self.debug.drawRouteNextPacenoteWpFromRecalc)
  if im.Checkbox("Next PacenoteWaypoint##debugDrawRouteNextPacenoteWp", boolPtr) then
    self.debug.drawRouteNextPacenoteWpFromRecalc = boolPtr[0]
  end
  im.SameLine()

  boolPtr = im.BoolPtr(self.debug.drawRouteNextRacePathnodeFromRecalc)
  if im.Checkbox("Next Race Pathnode##debugDrawRouteNextRacePathnode", boolPtr) then
    self.debug.drawRouteNextRacePathnodeFromRecalc = boolPtr[0]
  end

  boolPtr = im.BoolPtr(self.debug.drawRoutePathnodes)
  if im.Checkbox("Pathnodes##debugDrawRoutePathnodes", boolPtr) then
    self.debug.drawRoutePathnodes = boolPtr[0]
  end
  im.SameLine()

  boolPtr = im.BoolPtr(self.debug.drawRouteHiddenPathnodes)
  if im.Checkbox("Hidden Pathnodes##debugDrawRouteHiddenPathnodes", boolPtr) then
    self.debug.drawRouteHiddenPathnodes = boolPtr[0]
  end
  im.SameLine()

  boolPtr = im.BoolPtr(self.debug.drawRoutePointI)
  if im.Checkbox("Point Index##debugDrawRoutePointI", boolPtr) then
    self.debug.drawRoutePointI = boolPtr[0]
  end
  im.SameLine()

  boolPtr = im.BoolPtr(self.debug.drawRoutePointMetadata)
  if im.Checkbox("Point Metadata##debugDrawRoutePointMetadata", boolPtr) then
    self.debug.drawRoutePointMetadata = boolPtr[0]
  end

  boolPtr = im.BoolPtr(self.debug.drawRecceDrivelinePoints)
  if im.Checkbox("Recce Driveline Points##debugDrawRecceDrivelinePoints", boolPtr) then
    self.debug.drawRecceDrivelinePoints = boolPtr[0]
  end

  -- im.Separator()

  -- im.Text("kd-tree Tools")

  if im.CollapsingHeader1("KD-Tree Tools", im.TreeNodeFlags_DefaultClosed) then
    if self.selectedPacenote then
      local txt = '['..waypointTypes.shortenWaypointType(self.selectedPacenoteWaypoint.waypointType)..']'
      im.Text(string.format("Selected PacenoteWaypoint: %s%s", self.selectedPacenote.name, txt))
    else
      im.Text("Selected PacenoteWaypoint: <none>")
    end

    if im.Button("Build kd-tree for Race AI Path##debugBuildKdTreeRaceAiPath") then
      self:buildKdTreeRaceAiPath()
    end

    im.SameLine()

    if im.Button("<##debugSelectPrevPacenote") then
      if not self.selectedPacenote then
        if rm then
          local nb = rm:getNotebookPath()
          if nb then
            nb:setAllAdjacentNotes()
            self.selectedPacenote = nb.pacenotes.sorted[1]
            self.selectedPacenoteWaypoint = self.selectedPacenote:getCornerStartWaypoint()
          else
            log('E', logTag, 'failed to get notebook path, notebook path not found')
          end
        end
      end
      if self.selectedPacenote then
        if self.selectedPacenoteWaypoint:isCs() then
          if self.selectedPacenote.prevNote then
            self.selectedPacenote = self.selectedPacenote.prevNote
            self.selectedPacenoteWaypoint = self.selectedPacenote:getCornerEndWaypoint()
          end
        elseif self.selectedPacenoteWaypoint:isCe() then
          self.selectedPacenoteWaypoint = self.selectedPacenote:getCornerStartWaypoint()
        end
      end
      self:calcClosestLineSegmentKD()
    end

    im.SameLine()

    if im.Button(">##debugSelectNextPacenote") then
      if not self.selectedPacenote then
        if rm then
          local nb = rm:getNotebookPath()
          if nb then
            nb:setAllAdjacentNotes()
            self.selectedPacenote = nb.pacenotes.sorted[1]
            self.selectedPacenoteWaypoint = self.selectedPacenote:getCornerStartWaypoint()
          else
            log('E', logTag, 'failed to get notebook path, notebook path not found')
          end
        else
          log('E', logTag, 'failed to get notebook path, rally manager not found')
        end
      end
      if self.selectedPacenote then
        if self.selectedPacenoteWaypoint:isCs() then
          self.selectedPacenoteWaypoint = self.selectedPacenote:getCornerEndWaypoint()
        elseif self.selectedPacenoteWaypoint:isCe() then
          if self.selectedPacenote.nextNote then
            self.selectedPacenote = self.selectedPacenote.nextNote
            self.selectedPacenoteWaypoint = self.selectedPacenote:getCornerStartWaypoint()
          end
        end
      end
      self:calcClosestLineSegmentKD()
    end

    boolPtr = im.BoolPtr(self.debug.drawSelectedWaypoint)
    if im.Checkbox("PacenoteWaypoint & Closest Segment##debugDrawSelectedWaypoint", boolPtr) then
      self.debug.drawSelectedWaypoint = boolPtr[0]

      if not self.selectedPacenote then
        if rm then
          local nb = rm:getNotebookPath()
          if nb then
            nb:setAllAdjacentNotes()
            self.selectedPacenote = nb.pacenotes.sorted[1]
            self.selectedPacenoteWaypoint = self.selectedPacenote:getCornerStartWaypoint()
          else
            log('E', logTag, 'failed to get notebook path, notebook path not found')
          end
        else
          log('E', logTag, 'failed to get notebook path, rally manager not found')
        end
      end
      self:calcClosestLineSegmentKD()
    end

    -- im.Separator()

    if im.Button("Build kd-tree for Driveline Route##debugBuildKdTreeDrivelineRouteStatic") then
      self:buildKdTreeDrivelineRouteStatic()
    end

    boolPtr = im.BoolPtr(self.debug.drawKdTreeClosestRoutePoint)
    if im.Checkbox("Closest Route Point##debugDrawKdTreeClosestRoutePoint", boolPtr) then
      self.debug.drawKdTreeClosestRoutePoint = boolPtr[0]
      self:calcClosestDrivelineRouteKD()
    end
    im.SameLine()
    boolPtr = im.BoolPtr(self.debug.drawKdTreeNextPacenoteWp)
    if im.Checkbox("Next Pacenote Waypoint##debugDrawKdTreeNextPacenoteWp", boolPtr) then
      self.debug.drawKdTreeNextPacenoteWp = boolPtr[0]
      self:calcNextPacenoteWpForDrivelineRoutePoint()
    end
  end -- end KD-Tree Tools

  if im.CollapsingHeader1("Use Mouse as Vehicle", im.TreeNodeFlags_DefaultClosed) then
    boolPtr = im.BoolPtr(self.debug.useMouseRayCast)
    if im.Checkbox("Use Mouse Ray Cast##debugUseMouseRayCast", boolPtr) then
      self.debug.useMouseRayCast = boolPtr[0]
      local dr = gameplay_rally.getRallyManager():getDrivelineRoute()
      dr:setTrackMouseLikeVehicle(self.debug.useMouseRayCast)
    end
    im.SameLine()
    boolPtr = im.BoolPtr(self.debug.enableMouseRayCastMovement)
    if im.Checkbox("Enable Mouse Ray Cast Movement##debugEnableMouseRayCastMovement", boolPtr) then
      self.debug.enableMouseRayCastMovement = boolPtr[0]
      -- local dr = gameplay_rally.getRallyManager():getDrivelineRoute()
      -- dr:enableTrackMouseLikeVehicleMovement(self.debug.debugUseMouseRayCastEnableMovement)
      self:toggleMouseLikeVehicleEnableMovement()
    end
  end

  self:drawDebug()
end

function C:loadReccePoints()
  if self.recce then return end

  local rm = gameplay_rally.getRallyManager()
  if not rm then
    log('E', logTag, 'failed to load recce points, rally manager not found')
    return
  end

  local md = rm:getMissionDir()
  if not md then
    log('E', logTag, 'failed to load recce points, mission dir not found')
    return
  end

  local recce = Recce(md)
  if not recce:loadDrivelineAndCuts() then
    log('E', logTag, 'failed to load recce driveline and cuts for refresh')
  end

  self.recce = recce
end

function C:clearReccePoints()
  self.recce = nil
end

function C:drawDebug()
  if self.debug.drawRacePath then
    local rm = gameplay_rally.getRallyManager()
    if rm then
      local rp = rm:getRacePath()
      if rp then
        rp:drawDebug('normal')
      end
    end
  end

  if self.debug.drawRaceSplits then
    local rm = gameplay_rally.getRallyManager()
    if rm then
      local dr = rm:getDrivelineRoute()
      local pathnodes = rm:getRacePath().pathnodes.sorted
      for i, pathnode in ipairs(pathnodes) do
      local point = pathnode:getStaticRoutePoint()
      if pathnode.visible and point then
        local startDistToFinish = dr.routeStatic.path[1].distToTarget
        local finishLineDistOffset = dr:getFinishLineDistOffset()
        local distFromStart = startDistToFinish - point.distToTarget - finishLineDistOffset
        local distKm = distFromStart / 1000

        -- Get vehicle distance to this split
        local vehDistToFinish = dr.route.path[1].distToTarget
        local distToSplit = vehDistToFinish - point.distToTarget

        local distStr = string.format("split %.2fkm | %.1fm", distKm, distToSplit)
          debugDrawer:drawTextAdvanced(point.pos, distStr, ColorF(0,0,0,1), true, false, ColorI(255,128,0,255))
        end
      end
    end
  end

  if self.debug.drawRaceAiRoute then
    gameplay_rally.getRallyManager():getRacePath():drawAiRouteDebug()
  end

  if self.debug.drawRaceCurrentSeg then
    local race = self:getRace()
    local rm = gameplay_rally.getRallyManager()
    if rm and race then
      local currSegs = race.states[rm.vehicleTracker:getVehicleId()].currentSegments
      for i, segId in ipairs(currSegs) do
        local seg = race.path.segments.objects[segId]
        if seg and not seg.missing then
          local from = seg:getFrom()
          local to = seg:getTo()
          if from and to then
            local alpha = 0.3
            local fromName = from.name
            local toName = to.name

            if not from.visible then
              fromName = '('..fromName..')'
            end
            if not to.visible then
              toName = '('..toName..')'
            end

            local segClr = seg._rallyDebugColor
            local textFg = RallyUtil.getAppropriateTextColor(segClr)

            debugDrawer:drawSquarePrism(from.pos, to.pos, Point2F(2,4), Point2F(0,0), ColorF(segClr[1],segClr[2],segClr[3],alpha))
            debugDrawer:drawTextAdvanced(from.pos, String(string.format("%s [%s FROM]", fromName, seg.name)), textFg, true, false, ColorI(segClr[1]*255,segClr[2]*255,segClr[3]*255,255))
            debugDrawer:drawTextAdvanced(to.pos, String(string.format("%s [%s TO]", toName, seg.name)), textFg, true, false, ColorI(segClr[1]*255,segClr[2]*255,segClr[3]*255,255))

            -- debugDrawer:drawSphere(from.pos, from.radius, ColorF(1,0.5,0,alpha))
            -- debugDrawer:drawSphere(to.pos, to.radius, ColorF(1,0.5,0,alpha))

            local drawIntersectPlane = function(pn)
              if pn.hasNormal then
                local midWidth = pn.radius*2
                local side = pn.normal:cross(vec3(0,0,1)) *(pn.radius-pn.sidePadding.y - midWidth/2)
                -- debugDrawer:drawSquarePrism(
                --   pn.pos,
                --   (pn.pos + pn.radius * pn.normal),
                --   Point2F(1,pn.radius/2),
                --   Point2F(0,0),
                --   ColorF(1,0.5,0,alpha))
                debugDrawer:drawSquarePrism(
                  (pn.pos),
                  (pn.pos + 0.25 * pn.normal ),
                  Point2F(5,midWidth),
                  Point2F(0,0),
                  ColorF(segClr[1],segClr[2],segClr[3],alpha))
              end
            end
            drawIntersectPlane(from)
            drawIntersectPlane(to)
          end
        end
      end
    end
  end

  -- if self.debug.startPosition then
  --   local rp = gameplay_rally.getRallyManager():getRacePath()
  --   local defSpId = rp.defaultStartPosition
  --   local sp = rp.startPositions.objects[defSpId]
  --   if sp then

  --     sp:drawDebug()
  --   end
  -- end

  if self.debug.drawStartFinishLines then
    local rm = gameplay_rally.getRallyManager()
    if rm then
      local rp = rm:getRacePath()
      if rp then
        local defSpId = rp.defaultStartPosition
        local sp = rp.startPositions.objects[defSpId]
        local finish = rp.pathnodes.sorted[#rp.pathnodes.sorted]

        if sp then
          local midWidth = finish.radius*2 --- self.sidePadding.x - self.sidePadding.y

          local rot = sp.rot
          local normal = rot * vec3(0,-1,0) -- Forward vector from rotation

          debugDrawer:drawSquarePrism(
            sp.pos,
            (sp.pos + 0.25 * normal),
            Point2F(5,midWidth),
            Point2F(0,0),
            ColorF(1,0.5,0,0.6))
          debugDrawer:drawTextAdvanced(sp.pos,
            String('start'),
            ColorF(0,0,0,1), true, false,
            ColorI(255,128,0,255))
        end

        if finish then
          local midWidth = finish.radius*2 --- self.sidePadding.x - self.sidePadding.y
          -- local side = finish.normal:cross(vec3(0,0,1)) *(finish.radius-finish.sidePadding.y - midWidth/2)
          debugDrawer:drawSquarePrism(
            finish.pos,
            (finish.pos + 0.25 * finish.normal ),
            Point2F(5,midWidth),
            Point2F(0,0),
            ColorF(1,0.5,0,0.6))
          debugDrawer:drawTextAdvanced(finish.pos,
            String('finish'),
            ColorF(0,0,0,1), true, false,
            ColorI(255,128,0,255))
        end
      end
    end
  end

  if self.debug.drawStopZone then
    local rp = gameplay_rally.getRallyManager():getRacePath()
    local spStopZone = nil
    for _, sp in ipairs(rp.startPositions.sorted) do
      if sp.name == "STOP_ZONE" then
        spStopZone = sp
        break
      end
    end

    if spStopZone then
      debugDrawer:drawSphere(spStopZone.pos, 10, ColorF(1,0.5,0,0.5))
      debugDrawer:drawTextAdvanced(spStopZone.pos,
        String('STOP_ZONE'),
        ColorF(0,0,0,1), true, false,
        ColorI(255,128,0,255))
    end
  end

  if self.selectedPacenote and self.debug.drawSelectedWaypoint then
    -- debugDrawer:drawSphere(self.debugWaypoint.pos, 10, ColorF(1,0.5,0,0.5))
    local txt = self.selectedPacenote.name .. '['..waypointTypes.shortenWaypointType(self.selectedPacenoteWaypoint.waypointType)..']'
    local tagPos = vec3()
    tagPos:set(self.selectedPacenoteWaypoint.pos)
    tagPos.z = tagPos.z + 4
    local bottomPos = vec3(self.selectedPacenoteWaypoint.pos)
    bottomPos.z = bottomPos.z - 1
    debugDrawer:drawCylinder(bottomPos, tagPos, 0.3, ColorF(0,0,200/255.0,0.7))
    debugDrawer:drawTextAdvanced(tagPos,
      String(txt),
      ColorF(1,1,1,1), true, false,
      ColorI(0,0,200,255))

    if self.lineSegState and self.lineSegState.fromPos and self.lineSegState.toPos then
      debugDrawer:drawSphere(self.lineSegState.fromPos, 1.1, ColorF(1,0,1,0.8))
      debugDrawer:drawSphere(self.lineSegState.toPos, 1.1, ColorF(1,0,1,0.8))
      debugDrawer:drawSquarePrism(self.lineSegState.fromPos, self.lineSegState.toPos,
        Point2F(2,2), Point2F(2,2), ColorF(1,0,1,0.8))

      local xnorm = self.selectedPacenoteWaypoint.pos:xnormOnLine(self.lineSegState.fromPos, self.lineSegState.toPos)
      local projPos = lerp(self.lineSegState.fromPos, self.lineSegState.toPos, xnorm)

      debugDrawer:drawTextAdvanced(projPos,
        String(string.format("minSqDist to segment: %.2fm^2", self.lineSegState.minDistSq)),
        ColorF(0,0,0,1), true, false,
        ColorI(255,0,255,255))
    end

    if self.kdState and self.kdState.item_id and self.kdState.dist then
      local item = self.racePathAiPath[self.kdState.item_id]
      local dist = self.kdState.dist

      -- debugDrawer:drawSphere(item, 1.2, ColorF(0,1,1,0.8))
      debugDrawer:drawTextAdvanced(item,
        String(string.format("closest route point to wp %.2fm", dist)),
        ColorF(0,0,0,1), true, false,
        ColorI(255,0,255,255))
    end
  end

  if self.debug.drawNotebookPacenotes then
    local dr = gameplay_rally.getRallyManager():getDrivelineRoute()

    local function drawWp(wp, pacenoteName, pacenoteText, clr)
      local txtPos = vec3(wp.pos)
      local wpType = waypointTypes.shortenWaypointType(wp.waypointType)
      local clrFg = ColorF(0,0,0,1)
      local clrIBg = ColorI(clr[1] * 255, clr[2] * 255, clr[3] * 255, 255)
      debugDrawer:drawTextAdvanced(txtPos, String(string.format("%s[%s]%s", pacenoteName, wpType, pacenoteText)), clrFg, true, false, clrIBg)
    end

    for i, pacenote in ipairs(dr:getPacenotes()) do
      local pacenoteText = ''
      if true then
        pacenoteText = ' '..pacenote:noteOutputPreview()
      end
      drawWp(pacenote:getCornerStartWaypoint(), pacenote.name, pacenoteText, {0,1,0})
      drawWp(pacenote:getCornerEndWaypoint(), pacenote.name, pacenoteText, {1,0,0})
    end
  end

    local monochrome = false
    if self.debug.drawRaceAiRoute then
      monochrome = true
    end
    local rm = gameplay_rally.getRallyManager()
    if rm then
      local nb = rm:getNotebookPath()
      local dr = rm:getDrivelineRoute()
      local static = self.debug.drawDrivelineRouteStatic
      if dr then
        dr:drawDebugDrivelineRoute(
          self.debug.drawDrivelineRoute,
          self.debug.drawRoutePacenotes,
          #nb.pacenotes.sorted * 2,
          monochrome,
          static,
          self.debug.drawRouteHiddenPathnodes,
          self.debug.drawRoutePathnodes,
          self.debug.drawRoutePointI,
          self.debug.drawRoutePointMetadata,
          self.debug.drawRoutePacenoteText
        )
      end
    end

  if self.debug.drawKdTreeClosestPacenoteWaypoint then
    -- print('debug closest waypoint')
    if self.kdStatePacenoteWaypoints then
      -- print('kd state pacenote waypoints')
      local item = self.pacenoteWaypoints[self.kdStatePacenoteWaypoints.item_id]
      debugDrawer:drawSphere(item, 1.2, ColorF(0.5,0,1,0.8))
      debugDrawer:drawTextAdvanced(item,
        String(string.format("closest wp to veh %.2fm", self.kdStatePacenoteWaypoints.dist)),
        ColorF(1,1,1,1), true, false,
        ColorI(127,0,255,255))
    end
  end

  if self.debug.drawKdTreeClosestRoutePoint then
    if self.kdStateDrivelineRoute then
      local item = self.routeWaypointsIndex[self.kdStateDrivelineRoute.item_id]
      debugDrawer:drawSphere(item.pos, 1.2, ColorF(1,1,0,0.8))
      debugDrawer:drawTextAdvanced(item.pos,
        String(string.format("closest route point to veh %.2fm", self.kdStateDrivelineRoute.dist)),
        ColorF(0,0,0,1), true, false,
        ColorI(255,255,0,255))
    end
  end

  if self.debug.drawKdTreeNextPacenoteWp then
    if self.nextPacenoteWp then
      local wpType = waypointTypes.shortenWaypointType(self.nextPacenoteWp.waypointType)
      local pnName = self.nextPacenoteWp.pacenote.name
      local wpRp = self.nextPacenoteWp:getRoutePoint()
      debugDrawer:drawSphere(wpRp.pos, 1.2, ColorF(1,1,0,0.8))
      debugDrawer:drawTextAdvanced(wpRp.pos,
        String(string.format("next pacenoteWP: %s[%s]", pnName, wpType)),
        ColorF(0,0,0,1), true, false,
        ColorI(255,255,0,255))
    end
  end

  if self.debug.useMouseRayCast then
    local dr = gameplay_rally.getRallyManager():getDrivelineRoute()
    local pos = dr:getPosition()
    local speed = dr:getSpeed()
    if pos then
      debugDrawer:drawSphere(pos, 1.2, ColorF(0,0.5,0,0.8))
      if speed then
        debugDrawer:drawTextAdvanced(pos,
          String(string.format("%0.1f mph", speed * 2.23694)), -- convert m/s to mph
          ColorF(1,1,1,1), true, false,
          ColorI(0,127,0,255))
      end
    end
  end

  if self.debug.drawPreRoutePoints then
    local dr = gameplay_rally.getRallyManager():getDrivelineRoute()
    local preRoutePoints = dr.debugPreRoutePoints
    if preRoutePoints then
      for i, point in ipairs(preRoutePoints) do
        local clr = rainbowColor(#preRoutePoints, i, 1)
      debugDrawer:drawSphere(point.pos, 1.2, ColorF(clr[1], clr[2], clr[3], 0.8))
      debugDrawer:drawTextAdvanced(point.pos,
        String(string.format("pre_%d", i)),
        ColorF(i < 10 and 1 or 0, i < 10 and 1 or 0, i < 10 and 1 or 0, 1), true, false,
        ColorI(clr[1] * 255, clr[2] * 255, clr[3] * 255, 255))
      end
    end
  end

  local rm = gameplay_rally.getRallyManager()
  if rm then
    local dr = rm:getDrivelineRoute()
    if self.debug.drawRouteNextPacenoteWpFromRecalc and dr.nextPacenoteWpFromRecalc then
      local wpType = waypointTypes.shortenWaypointType(dr.nextPacenoteWpFromRecalc.waypointType)
      local pnName = dr.nextPacenoteWpFromRecalc.pacenote.name
      local wpRp = dr.nextPacenoteWpFromRecalc:getStaticRoutePoint()
      debugDrawer:drawSphere(wpRp.pos, 1.2, ColorF(1,1,0,0.8))
      debugDrawer:drawTextAdvanced(wpRp.pos,
        String(string.format("driveline.nextPacenoteWpFromRecalc: %s[%s]", pnName, wpType)),
        ColorF(0,0,0,1), true, false,
        ColorI(255,255,0,255))

      local pos = dr.lastRecalculateVehiclePos
      debugDrawer:drawSphere(pos, 1.2, ColorF(1,1,0,0.8))
      debugDrawer:drawTextAdvanced(pos,
        String(string.format("driveline.lastRecalculateVehiclePos")),
        ColorF(0,0,0,1), true, false,
        ColorI(255,255,0,255))

      local point = dr.debugNearestRecalcPoint
      if point then
        debugDrawer:drawSphere(point.pos, 1.2, ColorF(1,1,0,0.8))
        debugDrawer:drawTextAdvanced(point.pos,
          String(string.format("driveline.debugNearestRecalcPoint")),
          ColorF(0,0,0,1), true, false,
          ColorI(255,255,0,255))
      end

    end

    if self.debug.drawRouteNextRacePathnodeFromRecalc then
      local dr = rm:getDrivelineRoute()
      if dr.nextRacePathnodeFromRecalc then
        local wpRp = dr.nextRacePathnodeFromRecalc:getStaticRoutePoint()
        debugDrawer:drawSphere(wpRp.pos, 1.2, ColorF(1,1,0,0.8))
        debugDrawer:drawTextAdvanced(wpRp.pos,
          String(string.format("driveline.nextRacePathnodeFromRecalc: %s", dr.nextRacePathnodeFromRecalc.name)),
          ColorF(0,0,0,1), true, false,
          ColorI(255,255,0,255))
      end
    end
  end

  if self.debug.drawRouteCompletion then
    local rm = gameplay_rally.getRallyManager()
    if rm then
      local dr = rm:getDrivelineRoute()
      local pos = dr:getPosition()
      local completionData = dr:getRaceCompletionData()
      -- debugDrawer:drawTextAdvanced(pos,
      --   String(string.format("%dm", completionData.distM)),
      --   ColorF(1,1,1,1), true, false, ColorI(0,0,0,255))
      debugDrawer:drawTextAdvanced(pos,
        String(string.format("%.3fkm", completionData.distM / 1000)),
        ColorF(1,1,1,1), true, false, ColorI(0,0,0,255))
      debugDrawer:drawTextAdvanced(pos,
        String(string.format("%.1f%%", completionData.distPct * 100)),
        ColorF(1,1,1,1), true, false, ColorI(0,0,0,255))
    end
  end

  if self.debug.drawRouteShort then
    local rm = gameplay_rally.getRallyManager()
    if rm then
      local dr = rm:getDrivelineRoute()
      dr:drawDebugDrivelineRouteShort(100)
    end
  end

  if self.debug.drawReccePacenotes then
    local rm = gameplay_rally.getRallyManager()
    if rm then
      rm:drawPacenotesForDriving()
    end
  end

  if self.debug.drawRecceDrivelinePoints then
    local rm = gameplay_rally.getRallyManager()
    if rm then
      local dr = rm:getDrivelineRoute()
      local reccePoints = nil
      if dr and dr.recordedDriveline then
        reccePoints = dr.recordedDriveline.points
      else
        self:loadReccePoints()
        reccePoints = self.recce.driveline.points
      end
      if reccePoints then
        local clr = cc.snaproads_clr_recce
        for i, point in ipairs(reccePoints) do
          -- debugDrawer:drawCylinder(point.pos, point.pos + vec3(0,0,2), 0.3, ColorF(clr[1],clr[2],clr[3],0.8))
          -- debugDrawer:drawTextAdvanced(point.pos + vec3(0,0,2),
          --   String(string.format("recce_driveline_point_%d", i)),
          --   ColorF(1,1,1,1), true, false, ColorI(0,0,0,255))
          debugDrawer:drawSphere(point.pos, cc.snaproads_radius_recce, ColorF(clr[1], clr[2], clr[3], 0.8))
        end
      end
    end

    -- local finalPreRouteInput = dr.finalPreRouteInput
    -- for i, point in ipairs(finalPreRouteInput) do
    --   debugDrawer:drawCylinder(point.pos, point.pos + vec3(0,0,2), 0.1, ColorF(0.5,0,0.5,0.8))
    --   debugDrawer:drawTextAdvanced(point.pos + vec3(0,0,2),
    --     String(string.format("finalPreRouteInput_%d", i)),
    --     ColorF(1,1,1,1), true, false, ColorI(128,0,128,255))
    -- end

    -- local debugPreMergePath = dr.routeStatic.debugPath
    -- for i, point in ipairs(debugPreMergePath) do
    --   debugDrawer:drawCylinder(point.pos, point.pos + vec3(0,0,1.5), 0.2, ColorF(1,0.5,0.5,0.8))
    -- end

    -- local debugPostMergePath = dr.routeStatic.debugPostMergePath
    -- for i, point in ipairs(debugPostMergePath) do
    --   debugDrawer:drawCylinder(point.pos, point.pos + vec3(0,0,2), 0.1, ColorF(0,0.5,0.5,0.8))
    -- end

    -- local debugMergePathSample = dr.routeStatic.debugMergePathSample
    -- for i, point in ipairs(debugMergePathSample) do
    --   debugDrawer:drawCylinder(point.pos, point.pos + vec3(0,0,2), 0.1, ColorF(0,1,0.5,0.8))
    -- end

    -- local debugPostFixStartEnd1 = dr.routeStatic.debugPostFixStartEnd1
    -- for i, point in ipairs(debugPostFixStartEnd1) do
    --   debugDrawer:drawCylinder(point.pos, point.pos + vec3(0,0,2.5), 0.05, ColorF(1,1,0.5,0.8))
    -- end

    -- local debugPostFixStartEnd2 = dr.routeStatic.debugPostFixStartEnd2
    -- for i, point in ipairs(debugPostFixStartEnd2) do
    --   debugDrawer:drawCylinder(point.pos, point.pos + vec3(0,0,3.0), 0.025, ColorF(0.75,1,0.5,0.8))
    -- end

    -- if dr.routeStatic then
    --   local debugFixStartEnd_a = dr.routeStatic.debugFixStartEnd_a
    --   local debugFixStartEnd_b = dr.routeStatic.debugFixStartEnd_b
    --   local debugFixStartEnd_p = dr.routeStatic.debugFixStartEnd_p
    --   local debugFixStartEnd_xnorm = dr.routeStatic.debugFixStartEnd_xnorm

    --   if debugFixStartEnd_a and debugFixStartEnd_b and debugFixStartEnd_p then
    --     debugDrawer:drawCylinder(debugFixStartEnd_a.pos, debugFixStartEnd_a.pos + vec3(0,0,5.0), 0.01, ColorF(0,1,0,0.8))
    --     debugDrawer:drawCylinder(debugFixStartEnd_b.pos, debugFixStartEnd_b.pos + vec3(0,0,5.0), 0.01, ColorF(1,0,0,0.8))
    --     debugDrawer:drawCylinder(debugFixStartEnd_p.pos, debugFixStartEnd_p.pos + vec3(0,0,5.0), 0.01, ColorF(1,1,0,0.8))
    --     debugDrawer:drawTextAdvanced(debugFixStartEnd_p.pos,
    --       String(string.format("xnorm: %.2f", debugFixStartEnd_xnorm)),
    --       ColorF(0,0,0,1), true, false, ColorI(255,255,0,255))
    --   end
    -- end

  end
end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end