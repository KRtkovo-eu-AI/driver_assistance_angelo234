-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- This is a utility class for managing the drawing of a user-defined polygon on the map.

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- User constants.
local isMouseMoveTolSq = 0.0001 -- The tolerance for determining if the mouse is moving, in squared meters per frame.
local timeUntilTextAppears = 1.0 -- The time it takes for the text to appear when adding a new node, in seconds.
local lineSegGran = 10 -- The number of segments to use for the line segments of the polygon, in meters.
local lineRaiseHeight = 0.1 -- The height by which the polygon edges are raised above the terrain, in meters.

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

local M = {}

-- Module dependencies.
local geom = require('editor/toolUtilities/geom')
local render = require('editor/toolUtilities/render')
local gizmo = require('editor/toolUtilities/gizmo')
local util = require('editor/toolUtilities/util')

-- Module constants.
local im = ui_imgui
local min, max = math.min, math.max
local delKeyIdx, altKeyIdx = im.GetKeyIndex(im.Key_Delete), im.GetKeyIndex(im.Key_ModAlt)
local lineSegGranInv = 1.0 / lineSegGran

-- Module state.
local polygon = {}
local lastMousePos = vec3(0, 0, 0)
local selectedNodeIdx = 1
local hasDeletePressedRecently, lastAltDown = false, false
local isGizmoActive = false
local dragStatePre, dragNodeIdx = nil, nil
local tmpTable = { { nodes = nil } }
local markupTimer, markupTime = hptimer(), 0.0
local tmp1, tmp2, tmp3, tmp4 = vec3(0, 0, 0), vec3(0, 0, 0), vec3(0, 0, 0), vec3(0, 0, 0)


-- Deep copies the polygon.
local function deepCopyPolygon()
  local numNodes = #polygon
  local copy = table.new(numNodes, 0)
  for i = 1, numNodes do
    copy[i] = vec3(polygon[i])
  end
  return copy
end

-- Undo/redo callbacks for polygon edits.
local function undoPolyEdit(data) polygon = data.old end
local function redoPolyEdit(data) polygon = data.new end

-- Returns the current polygon.
local function getPolygon() return polygon end

-- Checks if the polygon is valid.
local function isPolygonValid() return #polygon > 2 end

-- Clears the polygon.
local function clearPolygon()
  local preState = deepCopyPolygon()
  table.clear(polygon)
  editor.history:commitAction("Clear Polygon", { old = preState, new = deepCopyPolygon() }, undoPolyEdit, redoPolyEdit)
end

-- Renders the polygon.
local function renderPolygon()
  -- Render the nodes.
  local numNodes = #polygon
  for i = 1, numNodes do
    render.drawSphereNode(polygon[i])
  end

  -- Render the start/end markups, if there are at least two nodes.
  if numNodes > 1 then
    render.markupStart(polygon[1])
    render.markupEnd(polygon[numNodes])
  end

  -- Render the edges.
  for i = 1, numNodes - 1 do
    local p1, p2 = polygon[i], polygon[i + 1]
    tmp1:set(p1.x, p1.y, p1.z)
    tmp2:set(p2.x, p2.y, p2.z)
    for j = 1, lineSegGran do
      tmp3:set(lerp(tmp1, tmp2, (j - 1) * lineSegGranInv)) -- Interpolate to increase line clarity on scene
      tmp4:set(lerp(tmp1, tmp2, j * lineSegGranInv))
      tmp3.z = core_terrain.getTerrainHeight(tmp3) + lineRaiseHeight
      tmp4.z = core_terrain.getTerrainHeight(tmp4) + lineRaiseHeight
      render.drawSplineLine(tmp3, tmp4)
    end
  end

  -- Render the closure edge (the line segment between the first and last node, indicating how the polygon will complete/close).
  if numNodes > 2 then
    local p1, p2 = polygon[1], polygon[numNodes]
    tmp1:set(p1.x, p1.y, p1.z + lineRaiseHeight)
    tmp2:set(p2.x, p2.y, p2.z + lineRaiseHeight)
    render.drawSplineLineDull(tmp1, tmp2)
  end

  -- Render the selected node highlight.
  if selectedNodeIdx and selectedNodeIdx > 0 and selectedNodeIdx <= numNodes then
    render.drawSphereHighlight(polygon[selectedNodeIdx])
  end
end

-- Handles the lifecycle of the user polygon. Calling this function allows the user to draw a polygon on the map with the mouse.
local function handleUserPolygon(doubleClickCallback)
  -- Render the mouse position.
  local mousePos = util.mouseOnMapPos()
  render.drawSphereCursor(mousePos)

  -- Render the polygon.
  renderPolygon()

  -- Manage the toggling of the gizmo on/off using the ALT key.
  local isAltDown = im.IsKeyDown(altKeyIdx)
  if isAltDown and isAltDown ~= lastAltDown then
    isGizmoActive = not isGizmoActive
  end
  lastAltDown = isAltDown

  -- Manage the translation gizmo.
  if isGizmoActive then
    tmpTable[1].nodes = polygon
    gizmo.handleGizmo(false, 1, selectedNodeIdx, tmpTable, true, false, deepCopyPolygon, undoPolyEdit, redoPolyEdit)
  end

  -- If the mouse is moving, then reset the markup delay timer.
  local isMouseMoving = mousePos:squaredDistance(lastMousePos) > isMouseMoveTolSq
  if isMouseMoving then
    markupTime = timeUntilTextAppears
  end

  -- Handle the mouse events.
  if util.isMouseHoveringOverTerrain() then

    -- Handle the double click event.
    if im.IsMouseDoubleClicked(0) and #polygon > 2 then
      if doubleClickCallback then
        doubleClickCallback(deepCopyPolygon()) -- Call the callback if it exists.
      end
      table.clear(polygon) -- Clear the polygon.
      selectedNodeIdx = 1
      return
    end

    -- Handle 'end drag' events.
    if not im.IsMouseDown(0) then
      if dragStatePre then
        editor.history:commitAction('Drag Node', { old = dragStatePre, new = deepCopyPolygon() }, undoPolyEdit, redoPolyEdit)
        dragStatePre = nil
      end
      dragNodeIdx = nil -- Keep this empty any time the mouse button is not held down.
    end

    -- Handle any active dragging events.
    if dragNodeIdx then
      if isMouseMoving then
        polygon[dragNodeIdx] = mousePos
      end
      lastMousePos = mousePos
      return -- Return to prevent any checking any other mouse events in this frame.
    end

    -- CASE #1: Check if the mouse is over a polygon node.
    tmpTable[1].nodes = polygon
    local isOverNode, _, nodeIdx = geom.isMouseOverNode(tmpTable)
    if isOverNode then
      render.drawSphereHighlight(polygon[nodeIdx]) -- Draw a highlight when the mouse is over a node.
      render.markupDrag(polygon[nodeIdx]) -- Draw a special markup when the mouse is over a node.
      if im.IsMouseClicked(0) then
        selectedNodeIdx = nodeIdx
        dragNodeIdx = nodeIdx -- Start dragging when mouse btn is held over a highlighted node.
        dragStatePre = deepCopyPolygon()
        lastMousePos = mousePos
        return -- Return to prevent any checking any other mouse events in this frame.
      end
    else
      -- CASE #2: Check if the mouse is over a polygon edge (but not a node).
      local isOverEdge, idxLower = geom.isMouseOverPolyline(polygon, mousePos)
      if isOverEdge then
        render.drawSphereHighlight(mousePos) -- Draw a highlight when the mouse is over a polygon edge.
        render.markupInsertNode(mousePos) -- Draw a special markup when the mouse is over a polygon edge.
        if im.IsMouseClicked(0) then
          local polygonPre = deepCopyPolygon()
          table.insert(polygon, idxLower + 1, vec3(mousePos)) -- Insert the new node at the correct intermediate position.
          editor.history:commitAction("Insert Node", { old = polygonPre, new = deepCopyPolygon() }, undoPolyEdit, redoPolyEdit)
          markupTime = timeUntilTextAppears
          selectedNodeIdx = idxLower + 1
          dragNodeIdx = idxLower + 1 -- Start dragging so user can drag straight away after inserting the node.
          dragStatePre = deepCopyPolygon()
          lastMousePos = mousePos
          return -- Return to prevent any checking any other mouse events in this frame.
        end
      else
        -- CASE #3: The mouse is not over a node or edge, so check for the adding of new nodes.
        if not isOverNode and markupTime < 0.0 then
          render.markupAddPolygonNode(mousePos) -- Show the delayed 'add node' markup.
        end
        if im.IsMouseClicked(0) then
          local polygonPre = deepCopyPolygon()
          polygon[#polygon + 1] = vec3(mousePos)
          editor.history:commitAction("Add Node", { old = polygonPre, new = deepCopyPolygon() }, undoPolyEdit, redoPolyEdit)
          selectedNodeIdx = #polygon
          markupTime = timeUntilTextAppears
          dragNodeIdx = #polygon -- Start dragging so user can drag straight away after inserting the node.
          dragStatePre = deepCopyPolygon()
          lastMousePos = mousePos
          return -- Return to prevent any checking any other mouse events in this frame.
        end
      end
    end
  end

  -- Handle node deletion using the delete key.
  if im.IsKeyDown(delKeyIdx) then
    if not hasDeletePressedRecently and selectedNodeIdx > 0 and selectedNodeIdx <= #polygon then
      local polygonPre = deepCopyPolygon()
      table.remove(polygon, selectedNodeIdx)
      selectedNodeIdx = max(1, min(#polygon, selectedNodeIdx))
      editor.history:commitAction("Delete Node", { old = polygonPre, new = deepCopyPolygon() }, undoPolyEdit, redoPolyEdit)
      hasDeletePressedRecently = true
    end
  else
    hasDeletePressedRecently = false -- Only allows one time use of the delete key.
  end

  -- Update the last mouse position.
  lastMousePos = mousePos

  -- Manage event timers.
  markupTime = max(-1.0, markupTime - markupTimer:stopAndReset() * 0.001)
end


-- Public interface.
M.getPolygon =                                          getPolygon
M.isPolygonValid =                                      isPolygonValid
M.clearPolygon =                                        clearPolygon

M.handleUserPolygon =                                   handleUserPolygon

return M