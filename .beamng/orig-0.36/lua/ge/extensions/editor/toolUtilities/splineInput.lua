-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- This is a utility class for handling mouse and keyboard events across various spline-editing tools.

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- User constants.
local isMouseMoveTolSq = 0.0001 -- The tolerance for determining if the mouse is moving, in squared meters per frame.
local heightSensitivity = 1 -- The sensitivity of the height adjustment, in meters per pixel.
local minSplineWidth, maxSplineWidth = 3.0, 200.0 -- The minimum and maximum widths for a spline, in meters.
local minSplineHeight, maxSplineHeight = 0.0, 70.0 -- The minimum and maximum heights for a spline, in meters.
local defaultSplineVel, defaultSplineVelLimit = 13.5, 70.0 -- The default velocity and velocity limit for a spline node, in meters per second.
local timeUntilTextAppears = 1.0 -- The time it takes for the text to appear when adding a new node, in seconds.

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

local M = {}

-- Module dependencies.
local geom = require('editor/toolUtilities/geom')
local render = require('editor/toolUtilities/render')
local gizmo = require('editor/toolUtilities/gizmo')
local util = require('editor/toolUtilities/util')

-- Module constants.
local im = ui_imgui
local min, max, ceil = math.min, math.max, math.ceil
local up = vec3(0, 0, 1)
local altKeyIdx = im.GetKeyIndex(im.Key_ModAlt)
local ctrlKeyIdx = im.GetKeyIndex(im.Key_ModCtrl)
local delKeyIdx = im.GetKeyIndex(im.Key_Delete)
local cKeyIdx = im.GetKeyIndex(im.Key_C)
local vKeyIdx = im.GetKeyIndex(im.Key_V)

-- Module state.
local mouseVel2D, mouseLast, mouseLastRawY = vec3(), vec3(), 0.0
local binVec = vec3()
local lastAltDown, hasDeletePressedRecently = false, false
local dragSplineIdx, dragNodeIdx, dragStatePre, isDragRib, isDragBar, dragRibIsFirstHandle = nil, nil, nil, nil, nil, nil
local ctrlCProfile = nil
local markupTimer, markupTime, restTimer, restTime = hptimer(), 0.0, hptimer(), 0.0
local tmpTable = {}


-- Handles the user input events for spline-editing tools.
-- Splines are user-editable polylines along the centerline of a variable width. They can be used to create roads, paths, etc.
-- [Splines - The collection of splines to handle events for.]
-- [Out - A table which contains the following common fields (will be updated as the user interacts with the splines):]
  -- [SelSplineIdx - The index of the selected spline.]
  -- [SelNodeIdx - The index of the selected node.]
  -- [SelLayerIdx - The index of the selected layer. NOTE: This is only used for the 'decal placement' case.]
  -- [IsGizmoActive - A flag which indicates whether the gizmo is active.]
-- [IsMoveSingleNodeRelative - A flag which indicates whether single selected nodes should be moved relative with the mouse velocity, or directly to the mouse position.]
-- [IsRotEnabled - A flag which indicates whether the rotation gizmo is enabled.]
-- [IsConformToTerrain - A flag which indicates whether the spline should be conform to the terrain, or not. Used for vertical gizmo control.]
-- [UseRibs - A flag which indicates whether to use ribs (handles for width adjustment).]
-- [UseBars - A flag which indicates whether to use bars (handles for height adjustment).]
-- [IsBarsLimits - A flag which indicates whether the bars are limits (true) or velocities (false).]
-- [UseCopyPaste - A flag which indicates whether to use the copy/paste profile feature.]
-- [UseGizmo - A flag which indicates whether to use gizmo.]
-- [IsLockShape - A flag which indicates whether the shape of the spline is locked, or not.]
-- [DefaultSplineWidth - The default width for a spline when adding a new node, in meters.]
-- [DeepCopyFunct - A function which deep copies a spline.]
-- [CopyProfileFunct - A function which copies a profile.]
-- [PasteProfileFunct - A function which pastes a profile.]
-- [RibStartDragCallback - A function which is called when the user starts dragging a rib.]
-- [AfterEndDragCallback - A function which is called when the user ends dragging.]
-- [UndoFunct - The undo callback function for a single spline edit.]
-- [RedoFunct - The redo callback function for a single spline edit.]
local function handleSplineEvents(splines, out, isMoveSingleNodeRelative, isRotEnabled, isConformToTerrain, useRibs, useBars, isBarsLimits, useCopyPaste, useGizmo, isLockShape, defaultSplineWidth, deepCopyFunct, copyProfileFunct, pasteProfileFunct, afterEndDragCallback, undoFunct, redoFunct)
  -- Update the mouse position and velocity, and cache the current mouse state.
  local mouseRawY = im.GetMousePos().y -- The current raw mouse y-position (2D).
  local mousePos = util.mouseOnMapPos() -- The current mouse position on the map (3D).
  mouseVel2D:set(mousePos.x - mouseLast.x, mousePos.y - mouseLast.y, 0.0) -- The 2D mouse velocity (XY).
  if mouseVel2D:squaredLength() > isMouseMoveTolSq then
    restTime = timeUntilTextAppears -- Reset the rest time when the mouse is moving.
  end

  -- If there are no splines or not an enabled selected spline, just update the mouse position and leave immediately.
  local selSpline = splines[out.spline]
  if #splines < 1 or not selSpline or not selSpline.isEnabled or selSpline.isLink then
    mouseLastRawY = mouseRawY
    mouseLast = mousePos
    return
  end

  -- Scene-only events (when mouse is hovering over the terrain).
  if util.isMouseHoveringOverTerrain() then
    -- Draw the mouse cursor.
    render.drawSphereCursor(mousePos)

    -- Handle 'end drag' events.
    if not im.IsMouseDown(0) then
      if dragStatePre then
        editor.history:commitAction("Drag", { old = dragStatePre, new = deepCopyFunct(splines[dragSplineIdx]) }, undoFunct, redoFunct)
        dragStatePre = nil
        if afterEndDragCallback then
          afterEndDragCallback(selSpline) -- Call the callback function after the drag has ended.
        end
      end
      isDragRib, isDragBar, dragSplineIdx, dragNodeIdx, dragRibIsFirstHandle = nil, nil, nil, nil, nil
    end

    -- Handle any active dragging events.
    if dragSplineIdx then
      if isDragBar then
        -- Bar dragging events.
        local vals = selSpline.vels -- The velocities of the spline nodes.
        if isBarsLimits then
          vals = selSpline.velLimits -- The speed limits.
        end
        local delta = (mouseLastRawY - mouseRawY) * heightSensitivity -- The vertical mouse velocity.
        if isLockShape then
          for i = 1, #vals do
            vals[i] = max(minSplineHeight, min(maxSplineHeight, vals[i] + delta)) -- Move all bars by the relative amount.
          end
        else
          vals[dragNodeIdx] = max(minSplineHeight, min(maxSplineHeight, vals[dragNodeIdx] + delta)) -- Move bar of the selected node by rel. amount.
        end
        selSpline.isDirty = true
      else
        if isDragRib then
          -- Rib dragging events.
          local ribPoints = selSpline.ribPoints
          local dragNodeIdxTimesTwo = dragNodeIdx * 2
          local p1, p2 = ribPoints[dragNodeIdxTimesTwo], ribPoints[dragNodeIdxTimesTwo - 1]
          render.markupWidthDisplay(p1, selSpline.widths[dragNodeIdx]) -- Draw a special markup to display the width of the selected node, as it is dragged.
          binVec:set(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z)
          binVec:normalize()
          local handleSign = dragRibIsFirstHandle and -1 or 1 -- The sign of the handle to adjust, so that pulling outward increases the width.
          local delta = handleSign * mouseVel2D:dot(binVec) -- The mouse velocity vector projected onto the rib binormal.
          local widths = selSpline.widths
          if isLockShape then
            for i = 1, #widths do
              widths[i] = max(minSplineWidth, min(maxSplineWidth, widths[i] + delta)) -- Move all ribs by the relative amount.
            end
          else
            widths[dragNodeIdx] = max(minSplineWidth, min(maxSplineWidth, widths[dragNodeIdx] + delta)) -- Move rib of the selected node by rel. amount.
          end
          selSpline.isDirty = true
        else
          -- Node dragging events.
          local spline = splines[dragSplineIdx]
          local nodes = spline.nodes
          if isLockShape then
            for i = 1, #nodes do
              nodes[i] = nodes[i] + mouseVel2D
            end
          else
            if isMoveSingleNodeRelative then
              nodes[dragNodeIdx] = nodes[dragNodeIdx] + mouseVel2D -- Move the selected node RELATIVELY (by the mouse velocity).
            else
              nodes[dragNodeIdx] = mousePos -- Move the selected node DIRECTLY to the mouse position.
            end
          end
          spline.isDirty = true
        end
        mouseLastRawY = mouseRawY
        mouseLast = mousePos
        return
      end
    end

    -- Handle 'add node' and 'start dragging' events.
    local isMouseOverHandle = false
    local isOverNode, hoverSplineIdx, hoverNodeIdx = geom.isMouseOverNode(splines)
    isMouseOverHandle = isMouseOverHandle or isOverNode
    if isOverNode and splines[hoverSplineIdx].isEnabled then
      -- 'Hover-Over-Node' events.
      if not splines[hoverSplineIdx].isLink then
        render.drawSphereHighlight(splines[hoverSplineIdx].nodes[hoverNodeIdx]) -- Draw a highlight when the mouse is over a node.
        if not dragSplineIdx and not im.IsMouseDown(0) and markupTime < 0.0 then
          render.markupDrag(splines[hoverSplineIdx].nodes[hoverNodeIdx]) -- Draw a special markup when the mouse is over a node.
        end
        if im.IsMouseClicked(0) then
          out.spline, out.node = hoverSplineIdx, hoverNodeIdx -- Update the selected spline index/node index when left clicking on a new node.
          dragStatePre = deepCopyFunct(splines[hoverSplineIdx])
          isDragRib, dragSplineIdx, dragNodeIdx = false, hoverSplineIdx, hoverNodeIdx -- Start dragging when mouse btn is held over a highlighted node.
        end
      end
    else
      -- Handle 'hover-over-rib' events.
      if useRibs then
        local isOverRib, ribSplineIdx, ribIdx = geom.isMouseOverRib(splines)
        isMouseOverHandle = isMouseOverHandle or isOverRib
        if isOverRib and splines[ribSplineIdx].isEnabled and out.spline == ribSplineIdx then
          -- 'Hover-Over-Rib' events.
          local idx2 = ribIdx % 2 == 0 and ribIdx - 1 or ribIdx + 1 -- The index of the other rib point at the same node.
          local ribPts = splines[ribSplineIdx].ribPoints
          render.drawSphereHighlight(ribPts[ribIdx]) -- Draw left rib hightlight.
          render.drawSphereHighlight(ribPts[idx2]) -- Draw right rib highlight.
          if not dragSplineIdx then
            render.markupAdjustWidth(ribPts[ribIdx]) -- Draw a special markup when the mouse is over a rib.
          end
          if im.IsMouseClicked(0) then
            local nodeIdx = ceil(ribIdx * 0.5) -- Map: Rib index -> node index.
            dragStatePre = deepCopyFunct(splines[ribSplineIdx])
            isDragRib, dragSplineIdx, dragNodeIdx = true, ribSplineIdx, nodeIdx
            dragRibIsFirstHandle = ribIdx % 2 == 0
            mouseLastRawY = mouseRawY
            mouseLast = mousePos
            out.spline, out.node = ribSplineIdx, nodeIdx
            return
          end
        end
      end

      -- Handle 'hover-over-bar' events.
      if useBars then
        local isOverBar, barSplineIdx, barNodeIdx = geom.isMouseOverBar(splines)
        isMouseOverHandle = isMouseOverHandle or isOverBar
        if isOverBar and splines[barSplineIdx].isEnabled and out.spline == barSplineIdx then
          -- 'Hover-Over-Bar' events.
          local barPts = splines[barSplineIdx].barPoints
          render.drawSphereHighlight(barPts[barNodeIdx]) -- Draw bar hightlight.
          if not dragSplineIdx then
            render.markupAdjustBar(barPts[barNodeIdx]) -- Draw a special markup when the mouse is over a bar.
          end
          if im.IsMouseClicked(0) then
            dragStatePre = deepCopyFunct(splines[barSplineIdx])
            isDragBar, dragSplineIdx, dragNodeIdx = true, barSplineIdx, barNodeIdx
            mouseLastRawY = mouseRawY
            mouseLast = mousePos
            out.spline, out.node = barSplineIdx, barNodeIdx
            return
          end
        end
      end

      local isOverSpline, idxLower = geom.isMouseOverSpline(selSpline, mousePos)
      local isMouseOverHandleButNotSpline = isMouseOverHandle
      isMouseOverHandle = isMouseOverHandle or isOverSpline
      if isOverSpline and selSpline.isEnabled then
        -- 'Hover-Over-Spline' events.
        if not selSpline.isLink then
          render.drawSphereHighlight(mousePos) -- Draw a highlight when the mouse is over the selected spline.
          if not isMouseOverHandleButNotSpline and not dragSplineIdx and not im.IsMouseDown(0) and markupTime < 0.0 then
            render.markupInsertNode(mousePos) -- Draw a special markup when the mouse is over the spline.
          end
          if im.IsMouseClicked(0) then
            local splinePre = deepCopyFunct(selSpline)
            local tableIdx = idxLower + 1
            table.insert(selSpline.nodes, tableIdx, vec3(mousePos)) -- Insert the new node at the correct intermediate position.
            local lerpWidth = (selSpline.widths[tableIdx - 1] + selSpline.widths[tableIdx]) * 0.5
            table.insert(selSpline.widths, tableIdx, lerpWidth) -- lerp the width.
            local lerpNormal = lerp(selSpline.nmls[tableIdx - 1], selSpline.nmls[tableIdx], 0.5)
            table.insert(selSpline.nmls, tableIdx, lerpNormal) -- lerp the normal.
            if useBars then
              local lerpVel = lerp(selSpline.vels[tableIdx - 1], selSpline.vels[tableIdx], 0.5)
              table.insert(selSpline.vels, tableIdx, lerpVel)
              local lerpVelLimit = lerp(selSpline.velLimits[tableIdx - 1], selSpline.velLimits[tableIdx], 0.5)
              table.insert(selSpline.velLimits, tableIdx, lerpVelLimit)
            end
            out.node = tableIdx
            markupTime = timeUntilTextAppears
            selSpline.isDirty = true
            editor.history:commitAction("Insert Node", { old = splinePre, new = deepCopyFunct(selSpline) }, undoFunct, redoFunct)
          end
        end
      else
        -- 'Hover-Over-Free-Space' events.
        if restTime < 0.0 and not isMouseOverHandle then
          render.markupAddNode(mousePos) -- Draw a special markup when the mouse is over free space.
        end
        if im.IsMouseClicked(0) and selSpline.isEnabled then
          local statePre = deepCopyFunct(selSpline)
          local selNodes, selWidths, selNmls, selVels, selVelLimits = selSpline.nodes, selSpline.widths, selSpline.nmls, selSpline.vels, selSpline.velLimits
          if #selNodes > 1 then -- Mouse not over spline, but spline has > 2 nodes.
            if  mousePos:squaredDistance(selNodes[1]) < mousePos:squaredDistance(selNodes[#selNodes]) then
              table.insert(selNodes, 1, vec3(mousePos)) -- If closest node is at the start, add the new node here.
              table.insert(selWidths, 1, selWidths[1]) -- Use the width of the first node.
              table.insert(selNmls, 1, vec3(selNmls[1] or up)) -- Set the normal to same as first node.
              if useBars then
                table.insert(selVels, 1, selVels[1])
                table.insert(selVelLimits, 1, selVelLimits[1])
              end
              out.node = 1
            else -- Otherwise, add the new node to the end.
              table.insert(selNodes, vec3(mousePos))
              table.insert(selWidths, selWidths[#selWidths]) -- Use the width of the last node.
              table.insert(selNmls, vec3(selNmls[#selNmls])) -- Set the normal to same as last node.
              if useBars then
                table.insert(selVels, selVels[#selVels])
                table.insert(selVelLimits, selVelLimits[#selVelLimits])
              end
              out.node = #selNodes
            end
          else -- Mouse not over spline, and spline has < 2 nodes. Just add the new node at the end.
            selNodes[#selNodes + 1] = vec3(mousePos)
            table.insert(selWidths, #selWidths > 0 and selWidths[#selWidths] or defaultSplineWidth) -- Use the width of the last node.
            table.insert(selNmls, vec3(selNmls[#selNmls] or up)) -- Set the normal to same as last node.
            if useBars then
              table.insert(selVels, selVels[#selVels] or defaultSplineVel)
              table.insert(selVelLimits, selVelLimits[#selVelLimits] or defaultSplineVelLimit)
            end
            out.node = #selNodes
          end
          markupTime = timeUntilTextAppears
          selSpline.isDirty = true
          editor.history:commitAction("Add Node", { old = statePre, new = deepCopyFunct(selSpline) }, undoFunct, redoFunct)
        end
      end
    end
  end

  -- Handle node deletion using the delete key.
  if not selSpline.isLink and selSpline.isEnabled then
    if im.IsKeyDown(delKeyIdx) then
      if not hasDeletePressedRecently and out.node > 0 and out.node <= #selSpline.nodes then
        local splinePre = deepCopyFunct(selSpline)
        table.remove(selSpline.nodes, out.node)
        table.remove(selSpline.widths, out.node)
        table.remove(selSpline.nmls, out.node)
        out.node = max(1, min(#selSpline.nodes, out.node))
        selSpline.isDirty = true
        editor.history:commitAction("Delete Node", { old = splinePre, new = deepCopyFunct(selSpline) }, undoFunct, redoFunct)
        hasDeletePressedRecently = true
      end
    else
      hasDeletePressedRecently = false -- Only allows one time use of the delete key.
    end
  end

  -- If requested, manage the ALT key for toggling the gizmo on/off.
  if useGizmo then
    local isAltDown = im.IsKeyDown(altKeyIdx)
    if isAltDown and isAltDown ~= lastAltDown then
      out.isGizmoActive = not out.isGizmoActive
    end
    if out.isGizmoActive then -- Handle the gizmo for translation, if it is active.
      gizmo.handleGizmo(isRotEnabled, out.spline, out.node, splines, isConformToTerrain, isLockShape, deepCopyFunct, undoFunct, redoFunct)
    end
    lastAltDown = isAltDown
  end

  -- Check if the user is attempting to copy/paste a profile.
  if useCopyPaste then
    local isCtrlDown, isCDown, isVDown = im.IsKeyDown(ctrlKeyIdx), im.IsKeyDown(cKeyIdx), im.IsKeyDown(vKeyIdx)
    if isCtrlDown and isCDown and out.spline then
      ctrlCProfile = copyProfileFunct(splines[out.spline])
    end
    if isCtrlDown and isVDown and ctrlCProfile and out.spline then
      local splinePre = copyProfileFunct(splines[out.spline])
      pasteProfileFunct(splines[out.spline], ctrlCProfile)
      editor.history:commitAction("Paste Profile", { old = splinePre, new = deepCopyFunct(splines[out.spline]) }, undoFunct, redoFunct)
    end
  end

  -- Manage the markup event timers.
  -- [Timers run from some positive value when set, then decrement until they reach -1.0. Events are triggered when the timers drop below 0.0]
  markupTime = max(-1.0, markupTime - markupTimer:stopAndReset() * 0.001)
  restTime = max(-1.0, restTime - restTimer:stopAndReset() * 0.001)

  -- Update the mouse position data.
  mouseLastRawY = mouseRawY
  mouseLast = mousePos
end

-- Handles the user input events for spline-editing tools.
-- Splines are user-editable polylines along the centerline of a variable width. They can be used to create roads, paths, etc.
-- [SelSpline - The selected spline.]
-- [Nodes - The collection of navigation graph nodes to handle events for.]
-- [DeepCopyFunct - A function which deep copies a spline.]
-- [UndoFunct - The undo callback function for a single spline edit.]
-- [RedoFunct - The redo callback function for a single spline edit.]
local function handleNavGraphEvents(selSpline, nodes, deepCopyFunct, undoFunct, redoFunct)
  -- Update the mouse position and velocity, and cache the current mouse state.
  local mouseRawY = im.GetMousePos().y -- The current raw mouse y-position (2D).
  local mousePos = util.mouseOnMapPos() -- The current mouse position on the map (3D).
  mouseVel2D:set(mousePos.x - mouseLast.x, mousePos.y - mouseLast.y, 0.0) -- The 2D mouse velocity (XY).
  if mouseVel2D:squaredLength() > isMouseMoveTolSq then
    restTime = timeUntilTextAppears -- Reset the rest time when the mouse is moving.
  end

  -- Scene-only events (when mouse is hovering over the terrain).
  if util.isMouseHoveringOverTerrain() then
    -- Draw the mouse cursor.
    render.drawSphereCursor(mousePos)

    -- Handle 'end bar drag' events.
    if not im.IsMouseDown(0) then
      if dragStatePre then
        editor.history:commitAction("Drag Bar", { old = dragStatePre, new = deepCopyFunct(selSpline) }, undoFunct, redoFunct)
        dragStatePre = nil
      end
      isDragBar, dragNodeIdx = nil, nil
    end

    -- Handle any active bar dragging events.
    if isDragBar then
      local vals = selSpline.vels -- The velocities of the spline nodes.
      local delta = (mouseLastRawY - mouseRawY) * heightSensitivity -- The vertical mouse velocity.
      vals[dragNodeIdx] = max(minSplineHeight, min(maxSplineHeight, vals[dragNodeIdx] + delta)) -- Move bar of the selected node by rel. amount.
      selSpline.isDirty = true
    end

    -- Handle 'add node' and 'start dragging' events.
    local isMouseOverHandle = false
    local isOverNode, hoverNodeKey = geom.isMouseOverGraphNode(nodes)
    isMouseOverHandle = isMouseOverHandle or isOverNode
    if isOverNode then
      -- 'Hover-Over-Node' events.
      render.drawSphereHighlight(nodes[hoverNodeKey]) -- Draw a highlight when the mouse is over a graph node.
      if not im.IsMouseDown(0) and markupTime < 0.0 then
        render.markupGraphNodeHover(nodes[hoverNodeKey]) -- Draw a special markup when the mouse is over a graph node.
      end
      if im.IsMouseClicked(0) then
        local statePre = deepCopyFunct(selSpline)
        local doesContain, idx = util.doesPathContainNode(selSpline.graphNodes, hoverNodeKey)
        if doesContain then
          table.remove(selSpline.graphNodes, idx) -- Path contains the node already, so remove it.
          table.remove(selSpline.vels, idx) -- Remove the velocity for the node.
        else
          table.insert(selSpline.graphNodes, hoverNodeKey) -- Path does not contain the node, so add it (to end).
          table.insert(selSpline.vels, defaultSplineVel) -- Add a default velocity for the new node.
        end
        selSpline.isDirty = true
        editor.history:commitAction("Change Path", { old = statePre, new = deepCopyFunct(selSpline) }, undoFunct, redoFunct)
        markupTime = timeUntilTextAppears
      end
    else
      tmpTable[1] = selSpline
      local isOverBar, _, barNodeIdx = geom.isMouseOverBar(tmpTable)
      isMouseOverHandle = isMouseOverHandle or isOverBar
      if isOverBar then
        -- 'Hover-Over-Bar' events.
        local barPts = selSpline.barPoints
        render.drawSphereHighlight(barPts[barNodeIdx]) -- Draw bar hightlight.
        if not dragSplineIdx then
          render.markupAdjustBar(barPts[barNodeIdx]) -- Draw a special markup when the mouse is over a bar.
        end
        if im.IsMouseClicked(0) then
          dragStatePre = deepCopyFunct(selSpline)
          isDragBar, dragNodeIdx = true, barNodeIdx
          mouseLastRawY = mouseRawY
          mouseLast = mousePos
          return
        end
      else
        -- 'Hover-Over-Free-Space' events.
        if restTime < 0.0 and not isMouseOverHandle then
          render.markupGraphFreeSpace(mousePos) -- Draw a special markup when the mouse is over free space.
        end
      end
    end
  end

  -- Manage the markup event timers.
  -- [Timers run from some positive value when set, then decrement until they reach -1.0. Events are triggered when the timers drop below 0.0]
  markupTime = max(-1.0, markupTime - markupTimer:stopAndReset() * 0.001)
  restTime = max(-1.0, restTime - restTimer:stopAndReset() * 0.001)

  -- Update the mouse position data.
  mouseLastRawY = mouseRawY
  mouseLast = mousePos
end


-- Public interface.
M.handleSplineEvents =                                  handleSplineEvents
M.handleNavGraphEvents =                                handleNavGraphEvents

return M