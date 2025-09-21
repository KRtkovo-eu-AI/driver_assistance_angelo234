-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- snaproad is used in the world editor to provide snap-to-road functionality.

local logTag = ''
local rallyUtil = require('/lua/ge/extensions/gameplay/rally/util')
local cc = require('/lua/ge/extensions/gameplay/rally/util/colors')
local RallyEnums = require('/lua/ge/extensions/gameplay/rally/enums')
local normals = require('/lua/ge/extensions/gameplay/rally/snaproad/normals')

local C = {}

local startingMinDist = 4294967295

function C:init(driveline, styleData, sourceType)
  self._driveline = driveline
  self.styleData = styleData
  self.sourceType = sourceType

  self.cameraPathPlayer = require('/lua/ge/extensions/gameplay/rally/cameraPathPlayer')(self)

  self.show_corner_calls = false

  self.filter = {
    enabled = false,
    points = {},
  }

  self.partition = {
    enabled = false,
    pacenote = nil,
    before_points = {},
    focus_points = {},
    after_points = {},
    corner_call_points = {},
  }

  self.partition_all_state = {
    enabled = false,
    notebook = nil,
    partitions = {},
    pacenote_partitions = {},
  }

  self.globalOpacity = 1.0
end

function C:isRecceSourced()
  return self.sourceType == RallyEnums.drivelineMode.recce
end

function C:isRouteSourced()
  return self.sourceType == RallyEnums.drivelineMode.route
end

function C:setGlobalOpacity(globalOpacity)
  self.globalOpacity = globalOpacity
end

local adjustHeight = function(pos, r)
  -- local newZ = core_terrain.getTerrainHeight(pos)
  -- return vec3(pos.x, pos.y, newZ-(r*0.25))
  return pos
end

function C:_drawDebugPoints(points, clr, alpha, radius, adjustH, globalOpacity)
  local clr = clr or self:_clrSnaproad()
  local alpha_shape = alpha or cc.snaproads_alpha
  local radius = radius or cc.snaproads_radius

  alpha_shape = alpha_shape * globalOpacity

  for _,point in ipairs(points) do
    local pos = point.pos

    local cached = point.cachedPacenotes
    if cached and cached.half then
      clr = cc.clr_blue
      radius = 0.75
    end

    if adjustH then
      pos = adjustHeight(pos, radius)
    end

    debugDrawer:drawSphere(
      pos,
      radius,
      ColorF(clr[1],clr[2],clr[3], alpha_shape)
    )
  end
end

function C:_drawDebugPartitionsAll()
  local points = self.partition_all_state.partitions
  if not points then return end
  local clr = cc.clr_green

  for i,partition in ipairs(points) do
    self:_drawDebugPoints(partition, clr, nil, nil, nil, self.globalOpacity)
  end

  points = self.partition_all_state.pacenote_partitions
  clr = cc.waypoint_clr_background

  for i,partition in ipairs(points) do
    self:_drawDebugPoints(partition, clr, nil, nil, nil, self.globalOpacity)
  end
end

function C:_clrSnaproad()
  if self:isRecceSourced() then
    return cc.snaproads_clr_recce
  elseif self:isRouteSourced() then
    return cc.snaproads_clr_route
  else
    return cc.snaproads_clr_recce
  end
end

function C:_drawDebugPartition(adjustH)
  local points = self.partition.focus_points
  local clr = nil

  if self.filter.enabled then
    clr = cc.clr_white
  else
    clr = self:_clrSnaproad()
  end

  if self.show_corner_calls then
    self:_drawDebugPoints(self.partition.corner_call_points.points_at_to_cs, clr, nil, nil, adjustH, self.globalOpacity)
    self:_drawDebugCornerCalls()
  else
    self:_drawDebugPoints(points, clr, nil, nil, adjustH, self.globalOpacity)
  end

  points = self.partition.before_points
  clr = cc.waypoint_clr_background
  self:_drawDebugPoints(points, clr, nil, nil, adjustH, self.globalOpacity)

  points = self.partition.after_points
  clr = cc.waypoint_clr_background
  self:_drawDebugPoints(points, clr, nil, nil, adjustH, self.globalOpacity)
end

function C:_drawDebugDefault(adjustH)
  local points = self:_allPoints()
  self:_drawDebugPoints(points, nil, nil, nil, adjustH, self.globalOpacity)
end

function C:drawDebugSnaproad()
  -- if self.filter.enabled then
    -- self:_drawDebugFilter()
  if self.partition.enabled then
    self:_drawDebugPartition()
  elseif self.partition_all_state.enabled then
    self:_drawDebugPartitionsAll()
  else
    self:_drawDebugDefault()
  end
end

function C:drawDebugRecceApp()
  if self.partition.enabled then
    self:_drawDebugPartition(true)
  else
    self:_drawDebugDefault(true)
  end
end

function C:drawDebugCameraPlaying()
  local pn = self.partition.pacenote
  if self.partition.enabled and pn then
    print('drawDebugCameraPlaying')
    local radius = 1.0
    local clr = cc.waypoint_clr_at
    local alpha_shape = cc.snaproads_alpha_driving

    clr = cc.waypoint_clr_cs
    debugDrawer:drawSphere(
      adjustHeight(pn:getCornerStartWaypoint().pos, radius),
      radius,
      ColorF(clr[1],clr[2],clr[3], alpha_shape)
    )

    clr = cc.waypoint_clr_ce
    debugDrawer:drawSphere(
      adjustHeight(pn:getCornerEndWaypoint().pos, radius),
      radius,
      ColorF(clr[1],clr[2],clr[3], alpha_shape)
    )

    clr = cc.clr_white
    radius = cc.snaproads_radius_driving
    local points = self.partition.focus_points

    for _,point in ipairs(points) do
      local pos = point.pos
      debugDrawer:drawSphere(
        adjustHeight(pos, radius),
        radius,
        ColorF(clr[1],clr[2],clr[3], alpha_shape)
      )
    end
  end
end

local function mylerp(a, b, t)
  return a + (b - a) * t
end

local function createGradient(steps)
  local gradient = {}

  for i = 1, steps do
    local t = (i - 1) / (steps - 1)  -- Normalize t to 0-1
    local r, g, b

    if t <= 0.5 then
      -- Interpolate between red and yellow
      r = 1
      g = mylerp(0, 1, t * 2)  -- Double t because it's only half the gradient
      b = 0
    else
      -- Interpolate between yellow and green
      r = mylerp(1, 0, (t - 0.5) * 2)  -- Adjust t and double for second half
      g = 1
      b = 0
    end

    table.insert(gradient, {r, g, b})
  end

  return gradient
end

function C:groupPointsByCornerCall(points)
  if not self.styleData then return {} end
  -- if editor_rallyEditor then
    -- local cornerAnglesStyle = capture_data.cornerAnglesStyle
    -- local corner_angles_data = editor_rallyEditor.getTranscriptsWindow():getCornerAngles(force_reload)
    -- local style_data = nil
    -- for _,style in ipairs(corner_angles_data.pacenoteStyles) do
    --   if style.name == cornerAnglesStyle then
    --     style_data = style
    --   end
    -- end

  local sortedAngles = {}
  for i,angle in ipairs(self.styleData.angles) do
    table.insert(sortedAngles, angle)
  end
  local function sortByAngleRev(a, b)
    return a.fromAngleDegrees > b.fromAngleDegrees
  end
  table.sort(sortedAngles, sortByAngleRev)

  local steps = #sortedAngles - 1  -- Number of color steps in the gradient
  -- subtract 1 for Center
  local gradientColors = createGradient(steps)
  for i,angle in ipairs(sortedAngles) do
    angle.color = gradientColors[i]
  end
  sortedAngles[#sortedAngles].color = cc.clr_white

  local subgroups = {{points = {}, label_point=-1, calc=nil}}

  for _,point in ipairs(points) do
    local subgroup_points = subgroups[#subgroups].points

    local angle_data, cornerCallStr, pct = rallyUtil.determineCornerCall(sortedAngles, point.steering)
    point.calc = {
      angle_pct = pct,
      angle_data = angle_data,
      cornerCallStr = cornerCallStr,
    }

    if #subgroup_points == 0 then
      table.insert(subgroup_points, point)
      subgroups[#subgroups].calc = point.calc
    elseif subgroup_points[#subgroup_points].calc.cornerCallStr ~= point.calc.cornerCallStr then
      table.insert(subgroups, {points={point}, label_point=-1, calc=point.calc})
    else
      table.insert(subgroup_points, point)
      subgroups[#subgroups].calc = point.calc
    end
  end

  for _,grp in ipairs(subgroups) do
    local label_i = round(#grp.points / 2)
    grp.label_point = grp.points[label_i]
  end

  return subgroups
end

function C:_drawDebugCornerCalls()
  local radius = cc.snaproads_radius
  local shapeAlpha = cc.snaproads_alpha
  local textAlpha = 1.0
  local clr = nil
  local clr_text_fg = cc.clr_black
  local label_point = nil
  local clr_text_bg = nil
  local calc = nil

  local groups = self.partition.corner_call_points.groups

  for _,grp in ipairs(groups) do
    for _,cap in ipairs(grp.points) do
      clr = cap.calc.angle_data.color
      local pos = vec3(cap.pos)
      debugDrawer:drawSphere(pos, radius, ColorF(clr[1],clr[2],clr[3],shapeAlpha))
    end

    label_point = grp.label_point
    calc = grp.calc
    clr_text_bg = calc.angle_data.color

    debugDrawer:drawTextAdvanced(
      vec3(label_point.pos),
      String(calc.cornerCallStr..' '),
      ColorF(clr_text_fg[1], clr_text_fg[2], clr_text_fg[3], textAlpha),
      true,
      false,
      ColorI(clr_text_bg[1]*255, clr_text_bg[2]*255, clr_text_bg[3]*255, textAlpha*255)
    )
  end
end

function C:driveline()
  return self._driveline
end

function C:_allPoints()
  if self:driveline() then
    return self:driveline().points
  else
    return {}
  end
end

function C:_operativePoints()
  if self.filter.enabled then
    return self.filter.points
  else
    return self:_allPoints()
  end
end

-- source_pos - should be a vec3
function C:closestSnapPoint(source_pos, useAllPoints)
  useAllPoints = useAllPoints or false
  local points = nil

  if useAllPoints then
    points = self:_allPoints()
  else
    points = self:_operativePoints()
  end

  local minDist = startingMinDist
  local closestPoint = nil

  for _,point in ipairs(points) do
    local pos = vec3(point.pos)
    local dist = (pos - source_pos):length()
    if dist < minDist then
      minDist = dist
      closestPoint = point
    end
  end

  return closestPoint
end

function C:closestSnapPos(source_pos)
  local point = self:closestSnapPoint(source_pos)
  if point then
    return point.pos
  else
    return source_pos
  end
end

-- function C:normalAlignPoints(point)
--   if not point then return nil, nil end
--
--   local fromPoint = nil
--   local toPoint = nil
--
--   if point.next then
--     fromPoint = point
--     toPoint = point.next
--   elseif point.prev then
--     toPoint = point.prev
--     fromPoint = point
--   else
--     toPoint = point + vec3(1,0,0)
--   end
--
--   return fromPoint, toPoint
-- end

function C:normalAlignPoints(point)
  return normals.normalAlignPoints(point)
end

-- function C:forwardNormalVec(point)
--   local fromPoint, toPoint = self:normalAlignPoints(point)
--   local normVec = rallyUtil.calculateForwardNormal(fromPoint.pos, toPoint.pos)
--   return vec3(normVec.x, normVec.y, normVec.z)
-- end

function C:forwardNormalVec(point)
  return normals.forwardNormalVec(point)
end

function C:pointsBackwards(fromPoint, steps, limitPoints)
  limitPoints = limitPoints or {}
  local toPoint = fromPoint

  for _ = 1,steps do
    local prevPoint = toPoint.prev

    for _,limitPoint in ipairs(limitPoints) do
      if prevPoint.id == limitPoint.id then
        return toPoint
      end
    end

    if prevPoint then
      toPoint = prevPoint
    end
  end
  return toPoint
end

function C:pointsForwards(fromPoint, steps, limitPoints)
  limitPoints = limitPoints or {}
  local toPoint = fromPoint

  for _ = 1,steps do
    local nextPoint = toPoint.next

    for _,lim in ipairs(limitPoints) do
      if nextPoint.id == lim.id then
        return toPoint
      end
    end

    if nextPoint then
      toPoint = nextPoint
    end
  end
  return toPoint
end

function C:distanceBackwards(fromPoint, meters, limitPoints)
  limitPoints = limitPoints or {}
  local toPoint = fromPoint
  local dist = 0

  while true do
    local prevPoint = toPoint.prev
    if prevPoint then
      dist = dist + vec3(toPoint.pos):distance(vec3(prevPoint.pos))

      for _,lim in ipairs(limitPoints) do
        if prevPoint.id == lim.id then
          return toPoint
        end
      end

      toPoint = prevPoint

      if dist > meters then
        return toPoint
      end
    else
      return toPoint
    end
  end
end

function C:distanceForwards(fromPoint, meters, limitPoints)
  limitPoints = limitPoints or {}
  local toPoint = fromPoint
  local dist = 0

  while true do
    local nextPoint = toPoint.next
    if nextPoint then
      dist = dist + vec3(toPoint.pos):distance(vec3(nextPoint.pos))

      for _,lim in ipairs(limitPoints) do
        if nextPoint.id == lim.id then
          -- break
          return toPoint
        end
      end

      toPoint = nextPoint

      if dist > meters then
        -- break
        return toPoint
      end
    else
      -- break
      return toPoint
    end
  end

  -- return toPoint
end

-- function C:closestPointBackwards(sourcePoint, notebook)
-- end

function C:firstSnapPoint()
  return self:_allPoints()[1]
end

function C:prevSnapPos(srcPos)
  return self:prevSnapPoint(srcPos).pos
end

function C:prevSnapPoint(srcPos)
  local snapPoint = self:closestSnapPoint(srcPos)
  if not snapPoint then return nil end

  local points = self:_operativePoints()

  -- check if we are at the beginning of the points list
  if snapPoint.id == points[1].id then
    return snapPoint
  elseif snapPoint.prev then
    local newPoint = snapPoint.prev
    return newPoint
  else
    return snapPoint
  end
end

function C:nextSnapPos(srcPos)
  return self:nextSnapPoint(srcPos).pos
end

function C:nextSnapPoint(srcPos)
  local snapPoint = self:closestSnapPoint(srcPos)
  if not snapPoint then return nil end

  local points = self:_operativePoints()

  -- check if we are at the end of the points list
  if snapPoint.id == points[#points].id then
    return snapPoint
  elseif snapPoint.next then
    local newPoint = snapPoint.next
    return newPoint
  else
    return snapPoint
  end
end

function C:setPartitionToPacenote(pn, startAtCs)
  if not pn or pn.missing then return end

  self.partition.pacenote = pn

  local pointStart = nil
  if startAtCs then
    pointStart = self:closestSnapPoint(pn:getCornerStartWaypoint().pos)
  else
    pointStart = self:closestSnapPoint(pn:getActiveFwdAudioTrigger().pos)
  end

  local pointCe = self:closestSnapPoint(pn:getCornerEndWaypoint().pos)

  local point_cs = pointStart
  local point_ce = pointCe

  local i_cs = point_cs.id
  local i_ce = point_ce.id
  local diff = i_ce - i_cs
  local half = round(diff / 2)
  local i_half = i_cs + half
  local point_half = self:_allPoints()[i_half]
  pn.halfpoint = point_half

  self:_partitionPoints(pointStart, pointCe)
end

function C:clearPartition()
  self.partition.enabled = false
  self.partition.pacenote = nil
  self.partition.before_points = {}
  self.partition.focus_points = {}
  self.partition.after_points = {}
end

function C:_partitionPoints(fromPoint, toPoint)
  if not (fromPoint and toPoint) then
    return
  end

  -- reset state
  self.partition.enabled = true
  self.partition.before_points = {}
  self.partition.focus_points = {}
  self.partition.after_points = {}
  self.partition.corner_call_points = {}
  self:_clearPointCachedPartitions()

  -- fill the focus points
  local currPoint = fromPoint
  table.insert(self.partition.focus_points, currPoint)

  while true do
    local nextPoint = currPoint.next

    if nextPoint then
      table.insert(self.partition.focus_points, nextPoint)
      nextPoint.partition = self.partition.focus_points

      if nextPoint.id == toPoint.id then
        break
      end

      currPoint = nextPoint
    else
      break
    end
  end

  -- maybe fill the corner call points
  if self.show_corner_calls and self.partition.pacenote then
    self.partition.corner_call_points = {
      points_at_to_cs = {},
      points_cs_to_ce = {},
      groups = {},
    }

    local wp_cs = self.partition.pacenote:getCornerStartWaypoint()
    local point_cs = self:closestSnapPoint(wp_cs.pos)
    for i,point in ipairs(self.partition.focus_points) do
      if point_cs and point.id < point_cs.id then
        table.insert(self.partition.corner_call_points.points_at_to_cs, point)
      else
        table.insert(self.partition.corner_call_points.points_cs_to_ce, point)
      end
    end

    local groups = self:groupPointsByCornerCall(self.partition.corner_call_points.points_cs_to_ce)
    self.partition.corner_call_points.groups = groups
  end

  -- fill the before points
  local points = self:_allPoints()
  local currPoint = points[1]
  local toPoint = self.partition.focus_points[1]
  table.insert(self.partition.before_points, currPoint)
  while true do
    local nextPoint = currPoint.next

    if nextPoint then
      if nextPoint.id == toPoint.id then
        break
      else
        table.insert(self.partition.before_points, nextPoint)
      end

      currPoint = nextPoint
    else
      break
    end
  end

  -- fill the after points
  local points = self:_allPoints()
  local currPoint = self.partition.focus_points[#self.partition.focus_points]
  if currPoint.next then
    local toPoint = points[#points]
    while true do
      local nextPoint = currPoint.next

      if nextPoint then
        table.insert(self.partition.after_points, nextPoint)
        if nextPoint.id == toPoint.id then
          break
        end
        currPoint = nextPoint
      else
        break
      end
    end
  end
end

function C:clearAll()
  self.partition_all_state.enabled = false
  self.partition_all_state.notebook = nil
  self.partition_all_state.partitions = {}
  self.partition_all_state.pacenote_partitions = {}
  self:clearFilter()
  self:_clearPointCachedPartitions()
end

function C:_clearPointCachedPartitions()
  for _,point in ipairs(self:_allPoints()) do
    point.partition = nil
  end
end

function C:partitionAllPacenotes(notebook)
  -- reset state
  self.partition_all_state.enabled = true
  self.partition_all_state.notebook = notebook
  self.partition_all_state.partitions = {}
  self.partition_all_state.pacenote_partitions = {}
  self:_clearPointCachedPartitions()

  local pn_partitions = {}
  local partitions = {}

  local pt_curr = self:_allPoints()[1]

  local partition = {}
  local pn_partition = {}

  for _,pn_curr in ipairs(notebook.pacenotes.sorted) do
    local wp_cs = pn_curr:getCornerStartWaypoint()
    local wp_ce = pn_curr:getCornerEndWaypoint()
    local pos_cs = wp_cs.pos
    local pos_ce = wp_ce.pos
    wp_cs._driveline_point = self:closestSnapPoint(pos_cs)
    wp_ce._driveline_point = self:closestSnapPoint(pos_ce)

    partition = {}
    pn_partition = {}

    while pt_curr.id < wp_cs._driveline_point.id do
      table.insert(partition, pt_curr)
      pt_curr.partition = partition
      pt_curr = pt_curr.next
    end

    while pt_curr and pt_curr.id <= wp_ce._driveline_point.id do
      table.insert(pn_partition, pt_curr)
      pt_curr = pt_curr.next
    end

    partition.pacenote_after = pn_curr
    table.insert(partitions, partition)
    table.insert(pn_partitions, pn_partition)
  end

  -- add points after the last pacenote to it's own partition
  partition = {}
  while pt_curr do
    table.insert(partition, pt_curr)
    pt_curr.partition = partition
    pt_curr = pt_curr.next
  end
  table.insert(partitions, partition)

  -- print('partitions:')
  -- for i,p in ipairs(partitions) do
  --   local s = ''
  --   for i,pt in ipairs(p) do
  --     s = s..', '..tostring(pt.id)
  --   end
  --   print(s)
  -- end

  self.partition_all_state.partitions = partitions
  self.partition_all_state.pacenote_partitions = pn_partitions
end

function C:clearFilter()
  self.filter.enabled = false
  self.filter.points = {}
end

function C:setFilterToAllPartitions()
  if not self.partition_all_state.enabled then return end

  self.filter.enabled = true
  self.filter.points = {}

  local partitions = self.partition_all_state.partitions

  for i,partition in ipairs(partitions) do
    for i,point in ipairs(partition) do
      table.insert(self.filter.points, point)
    end
  end
end

function C:setFilterPartitionPoint(point)
  if not self.partition_all_state.enabled then return end

  self.filter.enabled = true
  self.filter.points = {}

  local partition = point.partition

  for i,point in ipairs(partition) do
    table.insert(self.filter.points, point)
  end
end

function C:setFilter(wp)
  if not wp then return end
  -- if not wp then
  --   self.filter.enabled = false
  --   self.filter.points = {}
  --   self:clearPartition()
  --   return
  -- end

  -- self.filter.points = {}

  -- filtering modes:
  -- * AT is selected
  --   ->  back: cant go past prev AT
  --   ->  fwd:  cant go past self CS OR next note AT
  -- X CS is selected v1
  --   -> back: cant go past self AT OR cant go past prev CE
  --   -> fwd:  cant go past self CE
  -- * CS is selected v2 - moving CS back also moves AT back
  --   -> back: cant go past prev CE AND cant go past prev AT + 1
  --   -> fwd:  cant go past self CE
  -- * CE is selected
  --   -> back: cant go past self CS
  --   -> fwd:  cant go past next CS

  local notebook = wp.pacenote.notebook
  local pn_prev, pn_sel, pn_next = notebook:getAdjacentPacenoteSet(wp.pacenote.id)

  local limitBackPoint = nil
  local limitFwdPoint = nil

  if wp:isAt() then
    -- AT backwards movement
    if pn_prev then
      local wp_at_prev = pn_prev:getActiveFwdAudioTrigger()
      if wp_at_prev then
        local point = self:closestSnapPoint(wp_at_prev.pos, true)
        limitBackPoint = point
      end
    end

    -- AT forwards movement
    if pn_next then
      local wp_at_next = pn_next:getActiveFwdAudioTrigger()
      if wp_at_next then
        local point = self:closestSnapPoint(wp_at_next.pos, true)
        limitFwdPoint = point
      end
    end

    local wp_cs = pn_sel:getCornerStartWaypoint()
    if wp_cs then
      local point = self:closestSnapPoint(wp_cs.pos, true)
      if limitFwdPoint then
        if point.id < limitFwdPoint.id then
          limitFwdPoint = point
        end
      else
        limitFwdPoint = point
      end
    end

  elseif wp:isCs() then
    -- CS backwards movement
    -- local wp_at = pn_sel:getActiveFwdAudioTrigger()
    -- if wp_at then
    --   local point = self:closestSnapPoint(wp_at.pos, true)
    --   limitBackPoint = point
    -- end

    if pn_prev then
      local prev_wp_ce = pn_prev:getCornerEndWaypoint()
      if prev_wp_ce then
        local point = self:closestSnapPoint(prev_wp_ce.pos, true)
        if limitBackPoint then
          if point.id > limitBackPoint.id then
            limitBackPoint = point
          end
        else
          limitBackPoint = point
        end
      end

      -- local wp_at_prev = pn_prev:getActiveFwdAudioTrigger()
      -- if wp_at_prev then
      --   local point = self:closestSnapPoint(wp_at_prev.pos, true)
      --   point = self:pointsForwards(point, 1)
      --   limitBackPoint = point
      -- end
    end

    -- CS forwards movement
    local wp_ce = pn_sel:getCornerEndWaypoint()
    if wp_ce then
      local point = self:closestSnapPoint(wp_ce.pos, true)
      limitFwdPoint = point
    end
  elseif wp:isCe() then
    -- CE backwards movement
    local wp_cs = pn_sel:getCornerStartWaypoint()
    if wp_cs then
      local point = self:closestSnapPoint(wp_cs.pos, true)
      limitBackPoint = point
    end

    -- CE forwards movement
    if pn_next then
      local wp_cs_next = pn_next:getCornerStartWaypoint()
      if wp_cs_next then
        local point = self:closestSnapPoint(wp_cs_next.pos, true)
        limitFwdPoint = point
      end
    end
  end

  local unfilteredPoints = self:_allPoints()

  limitBackPoint = limitBackPoint or unfilteredPoints[1]
  local hitBackPos = false
  limitFwdPoint = limitFwdPoint or unfilteredPoints[#unfilteredPoints]

  self.filter.enabled = true
  self.filter.points = {}

  for _,point in ipairs(unfilteredPoints) do
    if not hitBackPos then
      if point == limitBackPoint then
        hitBackPos = true
      end
    elseif point == limitFwdPoint then
      break
    else
      table.insert(self.filter.points, point)
    end
  end

  -- self:_partitionPoints(self.filter.points[1], self.filter.points[#self.filter.points])
end

function C:setPartitionToFilter()
  if not self.filter.enabled then return end
  self:_partitionPoints(self.filter.points[1], self.filter.points[#self.filter.points])
end

function C:pointsForCameraPath()
  if self.partition.enabled then
    local points = self.partition.focus_points
    return points
  else
    return nil
  end
end

function C:playCameraPath()
  self.cameraPathPlayer:play()
end

function C:stopCameraPath()
  self.cameraPathPlayer:stop()
end

function C:partitionAllEnabled()
  return self.partition_all_state.enabled
end

function C:toggleCornerCalls()
  self.show_corner_calls = not self.show_corner_calls
  if self.partition.pacenote then
    self:setPartitionToPacenote(self.partition.pacenote)
  end
end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end

