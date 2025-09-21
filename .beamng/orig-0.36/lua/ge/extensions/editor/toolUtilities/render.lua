-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- This is a utility class for rendering splines with debugDraw. This is used across various spline-editing tools.

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- User constants.
local cullDist = 500.0 -- The distance to cull the rendering of splines, in meters.
local halfWidthForMeshSplineVis = 2.0 -- The half-width to be used for mesh spline layer wire-frame visualisations, in meters.
local elevationTolerance = 0.001 -- The tolerance for when the drop lines and elevation markups are displayed.
local layerBinormalSpacing = 10 -- The spacing of the binormal lines on layer rendering, in number of division points.
local lineJump = 5 -- The number of division points to jump between when drawing lines.
local normalLength = 5.0 -- The length of the normal line, in meters.
local numArcSegments = 16 -- The number of segments to use when drawing an arc.
local velocityGran = 10 -- The granularity of the velocity rendering.
local zExtra = 0.1 -- The extra z-offset to be used for the wire-frame rendering, in meters.
local zRayOffset = 0.05 -- The z-offset to add to the ribbon points when raycasting to the surface below.
local maxRaycastDist = 10000 -- The maximum distance to raycast for ribbon points.

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

local M = {}

-- Module dependencies.
local geom = require('editor/toolUtilities/geom')
local styleCore = require('editor/toolUtilities/style')
local util = require('editor/toolUtilities/util')
local dbgDraw = require('utils/debugDraw')

-- Module constants.
local abs, min, max, floor = math.abs, math.min, math.max, math.floor
local acos, sqrt = math.acos, math.sqrt
local style = styleCore.getStyle()
local cullDistSq = cullDist * cullDist
local numArcSegmentsInv = 1.0 / numArcSegments
local up, down = vec3(0, 0, 1), vec3(0, 0, -1)
local emptyTable = {}

-- Module state.
local finalPoints = {}
local latVec1, latVec2 = vec3(), vec3()
local bL, bR, tL, tR = vec3(), vec3(), vec3(), vec3()
local tmp1, tmp2, tmp3, tmp4 = vec3(), vec3(), vec3(), vec3()
local tmpA, tmpB = vec3(), vec3()
local tmpTgt, tmpDir = vec3(), vec3()
local tmpPoints = {}
for i = 1, numArcSegments + 1 do
  tmpPoints[i] = vec3()
end

-- Various functions to draw spheres.
local function drawSphereCulled(p, scale, col)
  local camPos = core_camera.getPosition()
  if p:squaredDistance(camPos) < cullDistSq then
    dbgDraw.drawSphere(p, sqrt(p:distance(camPos)) * scale, col)
  end
end
local function drawSphereNode(p) drawSphereCulled(p, style.sphere_node, style.color_node) end
local function drawSphereRib(p) drawSphereCulled(p, style.sphere_rib, style.color_rib_handle) end
local function drawSphereBar(p) drawSphereCulled(p, style.sphere_bar, style.color_bar_handle) end
local function drawSphereNodeDull(p) drawSphereCulled(p, style.sphere_node_dull, style.color_node_dull) end
local function drawSphereHighlight(p) drawSphereCulled(p, style.sphere_node_hover, style.color_highlight) end
local function drawGraphClearNode(p) drawSphereCulled(p, style.sphere_node, style.color_graph_clear_node) end
local function drawPathNode(p) drawSphereCulled(p, style.sphere_node, style.color_path_node) end
local function drawPathNodeBig(p) drawSphereCulled(p, style.big_node, style.color_path_node_big) end
local function drawSphereCursor(p) dbgDraw.drawSphere(p, sqrt(p:distance(core_camera.getPosition())) * style.sphere_mousePos, style.color_mousePos) end -- Not culled.

-- Various functions to draw lines between two points.
local function drawLineCulled(p0, p1, thickness, col)
  local camPos = core_camera.getPosition()
  if p0:squaredDistance(camPos) < cullDistSq and p1:squaredDistance(camPos) < cullDistSq then
    dbgDraw.drawLineInstance_MinArg(p0, p1, thickness, col)
  end
end
local function drawWireFrameLine(p0, p1) drawLineCulled(p0, p1, style.wire_thickness, style.color_layer_wire) end
local function drawRibLine(p0, p1) drawLineCulled(p0, p1, style.rib_thickness, style.color_rib_line) end
local function drawBarCeilingLine(p0, p1, v) drawLineCulled(p0, p1, style.drop_thickness, util.getBlueToRedColour(v, 0, 17)) end
local function drawSplineLineLinked(p0, p1) drawLineCulled(p0, p1, style.spline_thickness, style.color_spline_linked) end
local function drawActiveSegLine(p0, p1) drawLineCulled(p0, p1, style.wire_thickness, style.color_layer_wire) end
local function drawGroundLine(p0, p1) drawLineCulled(p0, p1, style.ground_thickness, style.color_ground) end
local function drawGroundLineDull(p0, p1) drawLineCulled(p0, p1, style.ground_thickness_dull, style.color_ground_dull) end
local function drawDropLine(p0, p1, v) drawLineCulled(p0, p1, style.drop_thickness, util.getBlueToRedColour(v, 0, 17)) end
local function drawDropLineThick(p0, p1, v) drawLineCulled(p0, p1, style.drop_thickness_thicker, util.getBlueToRedColour(v, 0, 17)) end
local function drawDropLineDull(p0, p1) drawLineCulled(p0, p1, style.drop_thickness_dull, style.color_drop_dull) end
local function drawNormalLine(p0, p1) drawLineCulled(p0, p1, style.normal_thickness, style.color_normal) end
local function drawRefNormalLine(p0, p1) drawLineCulled(p0, p1, style.normal_ref_thickness, style.color_ref_normal) end
local function drawNavLine(p0, p1) drawLineCulled(p0, p1, style.spline_thickness, style.color_nav) end
local function drawPathLine(p0, p1) drawLineCulled(p0, p1, style.spline_thickness, style.color_path) end
local function drawSplineLine(p0, p1) dbgDraw.drawLineInstance_MinArg(p0, p1, style.spline_thickness, style.color_spline) end -- Not culled.
local function drawSplineLineDull(p0, p1) dbgDraw.drawLineInstance_MinArg(p0, p1, style.spline_thickness_dull, style.color_spline_dull) end -- Not culled.
local function drawArcSegment(p0, p1) dbgDraw.drawLineInstance_MinArg(p0, p1, style.arc_seg_thickness, style.color_arc_seg) end -- Not culled.

-- Various functions to draw triangles, up to some culling distance.
local function drawTriCulled(a, b, c, col)
  local camPos = core_camera.getPosition()
  if a:squaredDistance(camPos) < cullDistSq and b:squaredDistance(camPos) < cullDistSq and c:squaredDistance(camPos) < cullDistSq then
    dbgDraw.drawTriSolid(a, b, c, col, true)
  end
end
local function drawTriNotSelectedSurface(a, b, c) drawTriCulled(a, b, c, style.color_not_selected_surf) end
local function drawTriActiveSurface(a, b, c) drawTriCulled(a, b, c, style.color_active_surf) end

-- Various functions to draw text markups, up to some culling distance.
local function drawMarkupCulled(pos, text)
  if pos:squaredDistance(core_camera.getPosition()) < cullDistSq then
    dbgDraw.drawTextAdvanced(pos, text, style.text_foreground, true, false, style.text_background)
  end
end
local function markupDrag(pos) drawMarkupCulled(pos, 'Drag To Move Node') end
local function markupAddNode(pos) drawMarkupCulled(pos, 'Click To Add Node Here') end
local function markupInsertNode(pos) drawMarkupCulled(pos, 'Click To Insert Node Here') end
local function markupStartNode(pos) drawMarkupCulled(pos, '[START]') end
local function markupEndNode(pos) drawMarkupCulled(pos, '[END]') end
local function markupAdjustWidth(pos) drawMarkupCulled(pos, 'Drag To Adjust Width') end
local function markupAdjustBar(pos) drawMarkupCulled(pos, 'Drag To Adjust Height') end
local function markupWidthDisplay(pos, w) drawMarkupCulled(pos, string.format('Width = %.2f m', w)) end
local function markupAddPolygonNode(pos) drawMarkupCulled(pos, 'Click To Add Node Here. Double Click To Finish.') end
local function markupStart(pos) drawMarkupCulled(pos, 'Start') end
local function markupEnd(pos) drawMarkupCulled(pos, 'End') end
local function markupAddPairLeft(pos) drawMarkupCulled(pos, 'Add Node Pair: Click To Add 1st Node Here') end
local function markupPairFirstNode(pos) drawMarkupCulled(pos, 'Pair - First Node') end
local function markupPairSecondNode(pos) drawMarkupCulled(pos, 'Left Click To Add 2nd Node Here, Right Click To Cancel Pair') end
local function markupNode1(pos) drawMarkupCulled(pos, 'Node [1]') end
local function markupNode2(pos) drawMarkupCulled(pos, 'Node [2]') end
local function markupNode3(pos) drawMarkupCulled(pos, 'Node [3]') end
local function markupActiveSurf(pos) drawMarkupCulled(pos, 'Active Surface (2D)') end
local function markupVolume(pos) drawMarkupCulled(pos, 'Active Volume (3D)') end
local function markupElevation(pos, elev) drawMarkupCulled(pos, string.format('Elevation = %.2f m', elev)) end
local function markupTwistAngle(pos, angleDeg) drawMarkupCulled(pos, string.format('Twist Angle = %.2f deg', angleDeg)) end
local function markupVelocity(pos, vel, isBarsLimit, unitsStr)drawMarkupCulled(pos, string.format('%s = %.2f %s', isBarsLimit and 'Limit' or 'Velocity', vel, unitsStr)) end
local function markupGraphNodeHover(pos) drawMarkupCulled(pos, 'Click To Add To Path (Or Remove If Already In Path)') end
local function markupGraphFreeSpace(pos) drawMarkupCulled(pos, 'Click On NavGraph Node To Add To Path') end
local function markupPathNode(pos, i) drawMarkupCulled(pos, string.format('Path Node [%d]', i)) end

-- Function to draw a circular arc, to some granularity.
local function drawArc(center, a, b, axis, twistAngle)
  local distFromCen = normalLength * 0.97
  local cenX, cenY, cenZ = center.x, center.y, center.z
  tmpA:set(a.x - cenX, a.y - cenY, a.z - cenZ) -- Vector from center to a
  tmpA:normalize()
  tmpB:set(b.x - cenX, b.y - cenY, b.z - cenZ) -- Vector from center to b
  tmpB:normalize()
  local angle = acos(tmpA:dot(tmpB)) -- The angle between the two vectors.
  local step = -sign2(twistAngle) * angle * numArcSegmentsInv
  for i = 1, numArcSegments + 1 do
    local p = geom.rotateVecAroundAxis(tmpA, axis, step * (i - 1))
    tmpPoints[i]:set(cenX + p.x * distFromCen, cenY + p.y * distFromCen, cenZ + p.z * distFromCen)
  end
  local midIdx = floor(numArcSegments * 0.5)
  for i = 1, numArcSegments do
    drawArcSegment(tmpPoints[i], tmpPoints[i + 1]) -- Draw the arc segments.
    if i == midIdx then
      markupTwistAngle(tmpPoints[i], twistAngle)
    end
  end
end

-- Draw the rib points.
local function drawRibPoints(ribs)
  for j = 1, #ribs, 2 do
    local p0, p1 = ribs[j], ribs[j + 1]
    drawSphereRib(p0)
    drawSphereRib(p1)
    drawRibLine(p0, p1)
  end
end

-- Draw the bar points.
local function drawBarPoints(bars)
  if #bars > 1 then
    for j = 1, #bars do
      local pBar = bars[j]
      drawSphereBar(pBar)
      tmp1:set(pBar.x, pBar.y, pBar.z + zExtra)
      local d = castRayStatic(tmp1, down, maxRaycastDist) -- Vertical raycast to set z.
      tmp1.z = tmp1.z - d + zRayOffset
      local elevation = pBar.z - tmp1.z
      drawDropLineThick(pBar, tmp1, elevation)
    end
  end
end

-- Handles the rendering of splines.
local function handleSplineRendering(splines, splineIdx, nodeIdx, isGizmoActive, isUseRot, isShapeLocked, isShowElevation)
  -- Render each spline, in turn.
  for i = 1, #splines do
    local spline = splines[i]
    if spline.isEnabled then -- Only render enabled splines.
      local nodes, ribs, bars, divPoints = spline.nodes, spline.ribPoints or emptyTable, spline.barPoints or emptyTable, spline.divPoints
      if spline.isLink then
        -- CASE #1: This is a linked spline, so draw the line segments in a special colour.
        for j = 1, #divPoints - 1 do
          drawSplineLineLinked(divPoints[j], divPoints[j + 1])
        end
      elseif i == splineIdx then
        -- CASE #2: This is the selected spline, so draw the nodes and line segments in bold colours.
        for j = 1, #nodes do
          local node = nodes[j]
          drawSphereNode(node)
          tmp2:set(node.x, node.y, node.z)
          tmp1:set(node.x, node.y, node.z + zExtra)
          local d = castRayStatic(tmp1, down, maxRaycastDist) -- Vertical raycast to set z.
          tmp2.z = tmp1.z - d + zRayOffset
          local elevation = tmp2.z - tmp1.z
          drawDropLineThick(tmp2, tmp1, elevation)
          if isShowElevation and isGizmoActive and editor.getAxisGizmoMode() == editor.AxisGizmoMode_Translate then
            if abs(elevation) < 0.01 then elevation = 0.0 end -- Stops flickering text between 0 and -0 when changing node elevations.
            markupElevation(tmp2, elevation)
          end
          if isGizmoActive and isUseRot and editor.getAxisGizmoMode() == editor.AxisGizmoMode_Rotate then
            if not isShapeLocked or j == nodeIdx then
              local p1, p2 = nodes[max(j - 1, 1)], nodes[min(j + 1, #nodes)]
              tmpTgt:set(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z)
              tmpTgt:normalize()
              local normal = spline.nmls[j]
              local twistAngle = geom.signedAngleAroundAxis(up, normal, tmpTgt)
              tmp1:set(node.x + normal.x * normalLength, node.y + normal.y * normalLength, node.z + normal.z * normalLength) -- The true normal.
              drawNormalLine(node, tmp1)
              local dp = up:dot(tmpTgt)
              tmp2:set(up.x - tmpTgt.x * dp, up.y - tmpTgt.y * dp, up.z - tmpTgt.z * dp)
              tmp2:normalize()
              tmp2:set(node.x + tmp2.x * normalLength, node.y + tmp2.y * normalLength, node.z + tmp2.z * normalLength) -- The reference normal (where angle = 0).
              drawRefNormalLine(node, tmp2)
              tmp3:set(node.x - tmp1.x, node.y - tmp1.y, node.z - tmp1.z)
              tmp4:set(node.x - tmp2.x, node.y - tmp2.y, node.z - tmp2.z)
              tmpDir:set(tmp3:cross(tmp4))
              tmpDir:normalize()
              drawArc(node, tmp1, tmp2, -sign(twistAngle) * tmpDir, twistAngle)
            end
          end
        end

        -- Draw the extra node handles.
        drawRibPoints(ribs) -- Rib points (appear at either side of the node, and control width-related properties).
        drawBarPoints(bars) -- Bar points (appear above the node and control height-related properties).

        -- Draw the spline.
        local numPts = #divPoints
        for j = 1, numPts - 1, lineJump do -- Do not draw every division point, to improve performance.
          local p0, p1 = divPoints[j], divPoints[min(j + lineJump, numPts)]
          tmp1:set(p0.x, p0.y, p0.z + zExtra)
          local d = castRayStatic(tmp1, down, maxRaycastDist) -- Vertical raycast to set z.
          tmp1.z = tmp1.z - d + zRayOffset
          tmp2:set(p1.x, p1.y, p1.z + zExtra)
          local d = castRayStatic(tmp2, down, maxRaycastDist) -- Vertical raycast to set z.
          tmp2.z = tmp2.z - d + zRayOffset
          drawSplineLine(p0, p1) -- Draw the 3D point-to-point line on the spline.
          drawGroundLine(tmp1, tmp2) -- Draw the ground line (3D line projected onto the surface below).
          local elev = p0.z - tmp1.z
          if elev > elevationTolerance then
            drawDropLine(p0, tmp1, elev) -- Draw the vertical drop line (ground height - spline height).
          end
        end
      else
        -- CASE #3: This is an unselected spline, so draw the nodes and line segments in dull colours.
        for j = 1, #nodes do
          drawSphereNodeDull(nodes[j])
        end
        local numPts = #divPoints
        for j = 1, numPts - 1, lineJump do -- Do not draw every division point, to improve performance.
          local p0, p1 = divPoints[j], divPoints[min(j + lineJump, numPts)]
          tmp1:set(p0.x, p0.y, p0.z + zExtra)
          local d = castRayStatic(tmp1, down, maxRaycastDist) -- Vertical raycast to set z.
          tmp1.z = tmp1.z - d + zRayOffset
          tmp2:set(p1.x, p1.y, p1.z + zExtra)
          local d = castRayStatic(tmp2, down, maxRaycastDist) -- Vertical raycast to set z.
          tmp2.z = tmp2.z - d + zRayOffset
          drawSplineLineDull(p0, p1) -- Draw the 3D point-to-point line on the spline.
          drawGroundLineDull(tmp1, tmp2) -- Draw the ground line (3D line projected onto the surface below).
          local elev = p0.z - tmp1.z
          if abs(elev) > elevationTolerance then
            drawDropLineDull(p0, tmp1) -- Draw the vertical drop line (ground height - spline height).
          end
        end
      end
    end
  end

  -- Render the selected node highlight, if everything is valid.
  if splineIdx and nodeIdx and splines[splineIdx] then
    local spline = splines[splineIdx]
    if nodeIdx <= #spline.nodes and spline.isEnabled and not spline.isLink then
      drawSphereHighlight(spline.nodes[nodeIdx])
    end
  end
end

-- Renders the wire-frame for the given layer.
-- [Layer - The layer to render.]
-- [Spline - The spline which contains the layer.]
local function renderLayer(layer, spline)
  table.clear(finalPoints)
  local divPoints, divWidths, binormals = spline.divPoints, spline.divWidths, spline.binormals
  local layerPosition = layer.position
  for j = 1, #divPoints do
    finalPoints[j] = divPoints[j] + binormals[j] * layerPosition * divWidths[j] * 0.5 -- Apply the lateral offset.
    tmp1:set(finalPoints[j].x, finalPoints[j].y, finalPoints[j].z + zExtra)
    local d = castRayStatic(tmp1, down, maxRaycastDist) -- Vertical raycast to set z.
    finalPoints[j].z = tmp1.z - d + zRayOffset
  end

  -- If the layer is not to span the entire track length, then we need to get the start and end indices of the division points.
  local startIdx, endIdx = 1, #finalPoints
  if not layer.isTrackLength and #spline.discMap >= layer.maxNodeIdx then
    startIdx, endIdx = spline.discMap[layer.minNodeIdx], spline.discMap[layer.maxNodeIdx]
  end

  -- Draw the wire-frame for the selected layer, to required specification.
  if layer.isTrackWidth and not layer.isLink then
    -- CASE #1: Width-track layer.
    for j = startIdx, endIdx - 1 do
      local jPlusOne = j + 1
      local lat1, lat2 = binormals[j], binormals[jPlusOne] -- The two corresponding binormal vectors.
      local wHalf1, wHalf2 = divWidths[j] * 0.5, divWidths[jPlusOne] * 0.5
      latVec1:set(lat1.x * wHalf1, lat1.y * wHalf1, lat1.z * wHalf1)
      latVec2:set(lat2.x * wHalf2, lat2.y * wHalf2, lat2.z * wHalf2)
      local p1, p2 = finalPoints[j], finalPoints[jPlusOne] -- The two div points.
      local p1X, p1Y, p1Z, p2X, p2Y, p2Z = p1.x, p1.y, p1.z, p2.x, p2.y, p2.z
      local latVec1X, latVec1Y, latVec1Z, latVec2X, latVec2Y, latVec2Z = latVec1.x, latVec1.y, latVec1.z, latVec2.x, latVec2.y, latVec2.z
      bL:set(p1X - latVec1X, p1Y - latVec1Y, p1Z - latVec1Z) -- The quadrilateral corner points.
      bR:set(p1X + latVec1X, p1Y + latVec1Y, p1Z + latVec1Z)
      tL:set(p2X - latVec2X, p2Y - latVec2Y, p2Z - latVec2Z)
      tR:set(p2X + latVec2X, p2Y + latVec2Y, p2Z + latVec2Z)
      drawWireFrameLine(bL, tL) -- Draw the wire-frame side lines.
      drawWireFrameLine(bR, tR)
      if j % layerBinormalSpacing == 0 then
        drawWireFrameLine(bL, bR) -- Draw the wire-frame front and back lines, but at the required spacing.
      end
    end
  else
    -- CASE #2: Fixed-width layer.
    for j = startIdx, endIdx - 1 do
      local jPlusOne = j + 1
      local lat1, lat2 = binormals[j], binormals[jPlusOne] -- The two corresponding binormal vectors.
      local layerHalfWidth = layer.isLink and halfWidthForMeshSplineVis or layer.width * 0.5 -- Special default fixed width for linked splines.
      latVec1:set(lat1.x * layerHalfWidth, lat1.y * layerHalfWidth, lat1.z * layerHalfWidth)
      latVec2:set(lat2.x * layerHalfWidth, lat2.y * layerHalfWidth, lat2.z * layerHalfWidth)
      local p1, p2 = finalPoints[j], finalPoints[jPlusOne] -- The back and front points for this line segment.
      local p1X, p1Y, p1Z, p2X, p2Y, p2Z = p1.x, p1.y, p1.z, p2.x, p2.y, p2.z
      local latVec1X, latVec1Y, latVec1Z, latVec2X, latVec2Y, latVec2Z = latVec1.x, latVec1.y, latVec1.z, latVec2.x, latVec2.y, latVec2.z
      bL:set(p1X - latVec1X, p1Y - latVec1Y, p1Z - latVec1Z) -- The quadrilateral corner points.
      bR:set(p1X + latVec1X, p1Y + latVec1Y, p1Z + latVec1Z)
      tL:set(p2X - latVec2X, p2Y - latVec2Y, p2Z - latVec2Z)
      tR:set(p2X + latVec2X, p2Y + latVec2Y, p2Z + latVec2Z)
      drawWireFrameLine(bL, tL) -- Draw the wire-frame side lines.
      drawWireFrameLine(bR, tR)
      if j % layerBinormalSpacing == 0 then
        drawWireFrameLine(bL, bR) -- Draw the wire-frame front and back lines, but at the required spacing.
      end
    end
  end
end

-- Renders the wire-frame for width.
local function renderWidth(spline)
  if not spline then
    return -- No spline to render.
  end

  local divPoints, divWidths, binormals = spline.divPoints, spline.divWidths, spline.binormals
  if not spline.isLink then
    for j = 1, #divPoints - 1 do
      local jPlusOne = j + 1
      local lat1, lat2 = binormals[j], binormals[jPlusOne] -- The two corresponding binormal vectors.
      local wHalf1, wHalf2 = divWidths[j] * 0.5, divWidths[jPlusOne] * 0.5
      latVec1:set(lat1.x * wHalf1, lat1.y * wHalf1, lat1.z * wHalf1)
      latVec2:set(lat2.x * wHalf2, lat2.y * wHalf2, lat2.z * wHalf2)
      local p1, p2 = divPoints[j], divPoints[jPlusOne] -- The two div points.
      local p1X, p1Y, p1Z, p2X, p2Y, p2Z = p1.x, p1.y, p1.z, p2.x, p2.y, p2.z
      local latVec1X, latVec1Y, latVec1Z, latVec2X, latVec2Y, latVec2Z = latVec1.x, latVec1.y, latVec1.z, latVec2.x, latVec2.y, latVec2.z
      bL:set(p1X - latVec1X, p1Y - latVec1Y, p1Z - latVec1Z) -- The quadrilateral corner points.
      bR:set(p1X + latVec1X, p1Y + latVec1Y, p1Z + latVec1Z)
      tL:set(p2X - latVec2X, p2Y - latVec2Y, p2Z - latVec2Z)
      tR:set(p2X + latVec2X, p2Y + latVec2Y, p2Z + latVec2Z)
      tmp1:set(bL.x, bL.y, bL.z + zExtra)
      local d = castRayStatic(tmp1, down, maxRaycastDist) -- Vertical raycast to set z.
      bL.z = tmp1.z - d + zRayOffset
      tmp2:set(bR.x, bR.y, bR.z + zExtra)
      d = castRayStatic(tmp2, down, maxRaycastDist) -- Vertical raycast to set z.
      bR.z = tmp2.z - d + zRayOffset
      tmp3:set(tL.x, tL.y, tL.z + zExtra)
      d = castRayStatic(tmp3, down, maxRaycastDist) -- Vertical raycast to set z.
      tL.z = tmp3.z - d + zRayOffset
      tmp4:set(tR.x, tR.y, tR.z + zExtra)
      d = castRayStatic(tmp4, down, maxRaycastDist) -- Vertical raycast to set z.
      tR.z = tmp4.z - d + zRayOffset
      drawWireFrameLine(bL, tL) -- Draw the wire-frame side lines.
      drawWireFrameLine(bR, tR)
      if j % layerBinormalSpacing == 0 then
        drawWireFrameLine(bL, bR) -- Draw the wire-frame front and back lines, but at the required spacing.
      end
      drawTriNotSelectedSurface(bR, bL, tL)
      drawTriNotSelectedSurface(bR, tL, tR)
    end
  end
end

-- Renders the velocities for the given spline.
-- [spline - The spline to render the velocities for.]
-- [isBarsLimit - Whether the velocities are limits, or actual values.]
-- [unitsInt - The units to render the velocities in.]
local function renderVelocities(spline, isBarsLimit, unitsInt)
  if not spline or #spline.nodes < 2 then
    return -- No spline or too short to render.
  end

  -- Render the bar ceiling spline.
  local valsMs = isBarsLimit and spline.velLimits or spline.vels
  local barScale = geom.getBarScale()
  local pts, vals = geom.catmullRomNodesWidthsOnly(spline.barPoints, valsMs, velocityGran)
  for j = 1, #pts - 1 do
    local p0, p1 = pts[j], pts[j + 1]
    tmp1:set(p0.x, p0.y, p0.z + zExtra)
    local d = castRayStatic(tmp1, down, maxRaycastDist) -- Vertical raycast to set z.
    tmp1.z = tmp1.z - d + zRayOffset
    tmp2:set(p0.x, p0.y, tmp1.z + vals[j] * barScale)

    tmp3:set(p1.x, p1.y, p1.z + zExtra)
    local d = castRayStatic(tmp3, down, maxRaycastDist) -- Vertical raycast to set z.
    tmp3.z = tmp3.z - d + zRayOffset
    tmp4:set(p1.x, p1.y, tmp3.z + vals[j + 1] * barScale)

    local elev = tmp2.z - tmp1.z
    drawDropLine(tmp2, tmp1, elev)
    drawBarCeilingLine(tmp2, tmp4, elev)
  end

  -- Render the velocity markups.
  for i = 1, #spline.barPoints do
    local finalVel, units = valsMs[i], 'm/s'
    if unitsInt == 1 then
      finalVel, units = util.msToMph(valsMs[i]), 'mph'
    elseif unitsInt == 2 then
      finalVel, units = util.msToKph(valsMs[i]), 'kph'
    end
    markupVelocity(spline.barPoints[i], finalVel, isBarsLimit, units)
  end
end

-- Renders the velocities for the given graph path.
-- [spline - The spline to render the velocities for.]
-- [unitsInt - The units to render the velocities in.]
local function renderVelocitiesGraph(spline, unitsInt)
  if not spline or #spline.graphNodes < 2 then
    return -- No spline or too short to render.
  end

  -- Render the bar ceiling spline.
  local vels = spline.vels
  local barScale = geom.getBarScale()
  local pts, vals = geom.catmullRomNodesWidthsOnly(spline.barPoints, vels, velocityGran)
  for j = 1, #pts - 1 do
    local p0, p1 = pts[j], pts[j + 1]
    tmp1:set(p0.x, p0.y, p0.z + zExtra)
    local d = castRayStatic(tmp1, down, maxRaycastDist) -- Vertical raycast to set z.
    tmp1.z = tmp1.z - d + zRayOffset
    tmp2:set(p0.x, p0.y, tmp1.z + vals[j] * barScale)

    tmp3:set(p1.x, p1.y, p1.z + zExtra)
    local d = castRayStatic(tmp3, down, maxRaycastDist) -- Vertical raycast to set z.
    tmp3.z = tmp3.z - d + zRayOffset
    tmp4:set(p1.x, p1.y, tmp3.z + vals[j + 1] * barScale)

    local elev = tmp2.z - tmp1.z
    drawDropLine(tmp2, tmp1, elev)
    drawBarCeilingLine(tmp2, tmp4, elev)
  end

  -- Render the velocity markups.
  for i = 1, #spline.barPoints do
    local finalVel, units = vels[i], 'm/s'
    if unitsInt == 1 then
      finalVel, units = util.msToMph(vels[i]), 'mph'
    elseif unitsInt == 2 then
      finalVel, units = util.msToKph(vels[i]), 'kph'
    end
    markupVelocity(spline.barPoints[i], finalVel, false, units)
  end
end

-- Renders the start node markup for the given spline.
local function renderStartEndMarkups(splines)
  for i = 1, #splines do
    local nodes = splines[i].nodes
    if #nodes > 1 then
      markupStartNode(nodes[1])
      markupEndNode(nodes[#nodes])
    end
  end
end

-- Renders the given ribbon.
-- [Ribbons are splines which are placed in L-R pairs, and have segments comprising of adjacent 8-point boxes.]
-- [They can have active surfaces (top or bottom), and closest segments. They are used for dynamic audio emitter placement, for example.]
-- [ribbon - The ribbon to render.]
-- [pMouse - The current mouse position.]
-- [isSelectedRibbon - Whether the ribbon is selected, or not.]
-- [selectedNodeIdx - The index of the selected ribbon node, if it exists.]
-- [placedLeftNode - The left node position, if it has been placed by the user.]
-- [bestCursorSeg - The index of the closest segment, if it exists.]
-- [isDragging - Whether the node is being dragged, or not.]
local function handleRibbonRendering(ribbons, selectedRibbonIdx, selectedNodeIdx, placedLeftNode, bestCursorSeg, isDragging)
  local pMouse = util.mouseOnMapPos()

  -- Iterate over each ribbon, in turn.
  for i = 1, #ribbons do
    local ribbon = ribbons[i]

    -- If the user has placed a left node for a pair, then render it.
    if placedLeftNode then
      local pLeft = placedLeftNode.p
      drawSphereNode(pLeft) -- Draw the left node.
      drawSphereHighlight(pLeft) -- Highlight the left node.
      drawSplineLineDull(pLeft, pMouse) -- Line from the left node to the mouse position.
    end

    -- Render the ribbon nodes.
    local nodes = ribbon.nodes
    local numNodes = #nodes
    if numNodes < 1 then
      return -- Only render ribbons if there is at least one node.
    end

    -- Render the ribbon nodes for all enabled ribbons.
    local isSelectedRibbon = i == selectedRibbonIdx
    if ribbon.isEnabled then
      local sphereFn = isSelectedRibbon and drawSphereNode or drawSphereNodeDull
      for j = 1, numNodes do
        sphereFn(nodes[j])
      end
    end

    -- Highlight the selected node, on the selected ribbon.
    if isSelectedRibbon and ribbon.isEnabled then
      if nodes[selectedNodeIdx] then
        drawSphereHighlight(nodes[selectedNodeIdx])
      end
    end

    -- Render the segments of this ribbon.
    if #nodes > 3 then -- We can only render segments if there are at least four nodes.
      local isTopActive, isUpRibbon, isAmbient, isQuadAndVolume = ribbon.isTopActive, ribbon.isUpRibbon, ribbon.isAmbient, ribbon.isQuadAndVolume
      local isActiveSurfTop = true
      local depths, numSegs = ribbon.depths, ribbon.numSegs
      for j = 1, numSegs do
        -- Choose the appropriate styling, depending on whether a surface is active/not active, or ribbon is selected/not selected.
        local lineFn, triFn1, triFn2 = drawSplineLine, drawTriNotSelectedSurface, drawTriNotSelectedSurface
        if not isAmbient then
          if isTopActive then
            if isUpRibbon then
              triFn1 = drawTriActiveSurface -- The ribbon is directed upwards, and has the top surface active.
              isActiveSurfTop = true
            else
              triFn2 = drawTriActiveSurface -- The ribbon is directed upwards, and has the bottom surface active.
              isActiveSurfTop = false
            end
          else
            if isUpRibbon then
              triFn2 = drawTriActiveSurface -- The ribbon is directed downwards, and has the bottom surface active.
              isActiveSurfTop = false
            else
              triFn1 = drawTriActiveSurface -- The ribbon is directed downwards, and has the top surface active.
              isActiveSurfTop = true
            end
          end
        end
        if isQuadAndVolume then
          triFn1, triFn2 = drawTriActiveSurface, drawTriActiveSurface -- Override # 1: 2D quad and volume ribbons with top and bottom surfaces the same.
        end
        if not isDragging and j == bestCursorSeg then
          lineFn = drawActiveSegLine -- Override # 2: Highlight the closest segment in a special colour.
        end
        if not isSelectedRibbon or not ribbon.isEnabled then
          lineFn, triFn1, triFn2 = drawSplineLineDull, drawTriNotSelectedSurface, drawTriNotSelectedSurface -- Override # 3: Unselected/disabled ribbons are drawn in dull colors.
        end

        -- The active surface (either top or bottom).
        local twoSegIdx = j * 2
        local i1, i2, i3, i4 = twoSegIdx - 1, twoSegIdx, twoSegIdx + 1, twoSegIdx + 2
        local lB_B, lF_B, rB_B, rF_B = nodes[i1], nodes[i2], nodes[i3], nodes[i4]
        lineFn(lB_B, lF_B)
        lineFn(rB_B, rF_B)
        lineFn(lB_B, rB_B)
        lineFn(lF_B, rF_B)
        triFn1(lB_B, lF_B, rB_B)
        triFn1(lB_B, rB_B, lF_B)
        triFn1(lF_B, rF_B, rB_B)
        triFn1(lF_B, rB_B, rF_B)

        -- Cache 'active surface' markup point 1.
        if j == 1 then
          tmp1:set((lB_B.x + rF_B.x) * 0.5, (lB_B.y + rF_B.y) * 0.5, (lB_B.z + rF_B.z) * 0.5)
        end

        -- The non-active surface (the other surface).
        local d1, d2, d3, d4 = depths[i1], depths[i2], depths[i3], depths[i4]
        if not isUpRibbon then
          d1, d2, d3, d4 = -d1, -d2, -d3, -d4
        end
        bL:set(lB_B.x, lB_B.y, lB_B.z - d1)
        bR:set(lF_B.x, lF_B.y, lF_B.z - d2)
        tL:set(rB_B.x, rB_B.y, rB_B.z - d3)
        tR:set(rF_B.x, rF_B.y, rF_B.z - d4)
        lineFn(bL, bR)
        lineFn(tL, tR)
        lineFn(bL, tL)
        lineFn(bR, tR)
        triFn2(bL, bR, tL)
        triFn2(bL, tL, bR)
        triFn2(bR, tR, tL)
        triFn2(bR, tL, tR)

        -- The vertical side lines.
        lineFn(lB_B, bL)
        lineFn(lF_B, bR)
        lineFn(rB_B, tL)
        lineFn(rF_B, tR)

        -- Cache 'active surface' markup point 1.
        if j == 1 then
          tmp2:set((bL.x + tR.x) * 0.5, (bL.y + tR.y) * 0.5, (bL.z + tR.z) * 0.5)
        end
      end

      -- Markup the active surface/volume, if appropriate.
      if isQuadAndVolume or isAmbient then
        tmp3:set((tmp1.x + tmp2.x) * 0.5, (tmp1.y + tmp2.y) * 0.5, (tmp1.z + tmp2.z) * 0.5)
        markupVolume(tmp3)
      elseif isActiveSurfTop then
        markupActiveSurf(tmp1)
      else
        markupActiveSurf(tmp2)
      end
    end
  end
end

-- Renders the nav graph.
local function renderNavGraph(navGraph)
  if not navGraph then
    return -- No nav graph to render.
  end

  -- Render the sphere-to-sphere ribbons.
  local lines = navGraph.lines
  for i = 1, #lines do
    local line = lines[i]
    local pL, pR, qL, qR, cL, cR = line.pL, line.pR, line.qL, line.qR, line.cL, line.cR
    drawTriNotSelectedSurface(pR, pL, qR) -- Ribbon surface.
    drawTriNotSelectedSurface(pL, qL, qR)
    drawNavLine(cL, cR) -- Center line between the two spheres.
  end

  -- Render the nav graph nodes.
  local nodes = navGraph.nodes
  for _, v in pairs(nodes) do
    drawGraphClearNode(v)
  end
end

-- Renders the given graph path.
local function renderGraphPath(path, graphData)
  local graphNodes = graphData.nodes
  local pathLength = #path
  for i = 1, pathLength do
    drawPathNode(graphNodes[path[i]])
  end
  for i = 1, pathLength - 1 do
    local p0, p1 = graphNodes[path[i]], graphNodes[path[i + 1]]
    drawPathLine(p0, p1)
  end
end

-- Renders the chosen navGraph nodes with special indicators.
local function renderChosenNodes(nodes, graphData)
  local graphNodes = graphData.nodes
  for i = 1, #nodes do
    local p = graphNodes[nodes[i]]
    drawPathNodeBig(p) -- Larger sphere to show it is part of the selection and not just the path. Thus it can be removed.
    markupPathNode(p, i) -- Markup to index the node on screen.
  end
end


-- Public interface.
M.drawSphereCulled =                                    drawSphereCulled
M.drawSphereCursor =                                    drawSphereCursor
M.drawSphereNode =                                      drawSphereNode
M.drawSphereRib =                                       drawSphereRib
M.drawSphereNodeDull =                                  drawSphereNodeDull
M.drawSphereHighlight =                                 drawSphereHighlight

M.drawLineCulled =                                      drawLineCulled
M.drawSplineLine =                                      drawSplineLine
M.drawSplineLineDull =                                  drawSplineLineDull
M.drawWireFrameLine =                                   drawWireFrameLine
M.drawRibLine =                                         drawRibLine
M.drawSplineLineLinked =                                drawSplineLineLinked
M.drawDropLine =                                        drawDropLine
M.drawDropLineThick =                                   drawDropLineThick
M.drawDropLineDull =                                    drawDropLineDull
M.drawNormalLine =                                      drawNormalLine
M.drawRefNormalLine =                                   drawRefNormalLine
M.drawGroundLine =                                      drawGroundLine
M.drawGroundLineDull =                                  drawGroundLineDull
M.drawArcSegment =                                      drawArcSegment

M.drawTriCulled =                                       drawTriCulled
M.drawTriNotSelectedSurface =                           drawTriNotSelectedSurface
M.drawTriActiveSurface =                                drawTriActiveSurface

M.markupDrag =                                          markupDrag
M.markupAddNode =                                       markupAddNode
M.markupInsertNode =                                    markupInsertNode
M.markupStartNode =                                     markupStartNode
M.markupEndNode =                                       markupEndNode
M.markupAdjustWidth =                                   markupAdjustWidth
M.markupAdjustBar =                                     markupAdjustBar
M.markupWidthDisplay =                                  markupWidthDisplay
M.markupVelocity =                                      markupVelocity
M.markupAddPolygonNode =                                markupAddPolygonNode
M.markupStart =                                         markupStart
M.markupEnd =                                           markupEnd
M.markupAddPairLeft =                                   markupAddPairLeft
M.markupPairFirstNode =                                 markupPairFirstNode
M.markupPairSecondNode =                                markupPairSecondNode
M.markupNode1 =                                         markupNode1
M.markupNode2 =                                         markupNode2
M.markupNode3 =                                         markupNode3
M.markupActiveSurf =                                    markupActiveSurf
M.markupVolume =                                        markupVolume
M.markupGraphFreeSpace =                                markupGraphFreeSpace
M.markupGraphNodeHover =                                markupGraphNodeHover

M.drawArc =                                             drawArc

M.drawRibPoints =                                       drawRibPoints
M.drawBarPoints =                                       drawBarPoints

M.handleSplineRendering =                               handleSplineRendering
M.renderLayer =                                         renderLayer
M.renderWidth =                                         renderWidth
M.renderVelocities =                                    renderVelocities
M.renderVelocitiesGraph =                               renderVelocitiesGraph
M.renderStartEndMarkups =                               renderStartEndMarkups

M.renderRibbon =                                        handleRibbonRendering

M.renderNavGraph =                                      renderNavGraph
M.renderGraphPath =                                     renderGraphPath
M.renderChosenNodes =                                   renderChosenNodes

return M