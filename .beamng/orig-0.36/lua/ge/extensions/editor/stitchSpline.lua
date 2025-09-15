-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

-- User constants.
local diskFileName = "stitchSplineData.splineTools.json" -- The name of the file to save/load the stitch spline data to/from.

local defaultSplineWidth = 10.0 -- The default width for a spline when adding a new node, in meters.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

local M = {}
local logTag = 'stitchSpline'

-- External modules.
local splineMgr = require('editor/stitchSpline/splineMgr')
local pop = require('editor/stitchSpline/populate')
local import = require('editor/stitchSpline/import')
local input = require('editor/toolUtilities/splineInput')
local render = require('editor/toolUtilities/render')
local poly = require('editor/toolUtilities/polygon')
local skeleton = require('editor/toolUtilities/skeleton')
local util = require('editor/toolUtilities/util')
local style = require('editor/toolUtilities/style')
local meshAuditionMgr = require('editor/toolUtilities/meshAuditionMgr')

-- Module constants.
local im = ui_imgui
local min, max = math.min, math.max
local toolWinName, toolWinSize = 'stitchSpline', im.ImVec2(300, 700)
local defaultParams = splineMgr.getDefaultSliderParams()
local cols = style.getImguiCols('crystal')
local vec24, vec36 = im.ImVec2(24, 24), im.ImVec2(36, 36)

-- Module state.
local isStitchSplineActive = false
local isGizmoActive = false
local isDrawPolygon = false
local selectedSplineIdx = 1
local selectedNodeIdx = 1
local selectedMeshIdx = 1
local meshTarget = 'pole'
local sliderPreEditState = nil
local out = { spline = selectedSplineIdx, node = selectedNodeIdx, isGizmoActive = isGizmoActive }
local isLockShape = false


-- Callback for when a static mesh is selected in the mesh audition manager.
local function onMeshSelected(auditionMesh, path)
  -- Get the bounding box data from the given audition mesh.
  local box = auditionMesh:getObjBox()
  local worldBox = auditionMesh:getWorldBox()
  local center = auditionMesh:getPosition()
  local minExtents, maxExtents = worldBox.minExtents, worldBox.maxExtents
  local extents = box:getExtents()

  -- Store the mesh box data in the currently-selected spline.
  local splines = splineMgr.getStitchSplines()
  local selSpline = splines[selectedSplineIdx]
  local statePre = splineMgr.deepCopyStitchSpline(selSpline)
  if meshTarget == 'pole' then
    selSpline.poleMeshPath = path
    selSpline.poleMeshName = path:match("([^/]+)$")
    selSpline.pole_boxXLeft_Center, selSpline.pole_boxXRight_Center = center.x - minExtents.x, maxExtents.x - center.x
    selSpline.pole_boxYLeft_Center, selSpline.pole_boxYRight_Center = center.y - minExtents.y, maxExtents.y - center.y
    selSpline.pole_boxZLeft_Center, selSpline.pole_boxZRight_Center = center.z - minExtents.z, maxExtents.z - center.z
    selSpline.pole_extentsL_Center, selSpline.pole_extentsW_Center, selSpline.pole_extentsZ_Center = extents.x, extents.y, extents.z
  elseif meshTarget == 'wire' then
    selSpline.wireMeshPath = path
    selSpline.wireMeshName = path:match("([^/]+)$")
    selSpline.wire_boxXLeft_Center, selSpline.wire_boxXRight_Center = center.x - minExtents.x, maxExtents.x - center.x
    selSpline.wire_boxYLeft_Center, selSpline.wire_boxYRight_Center = center.y - minExtents.y, maxExtents.y - center.y
    selSpline.wire_boxZLeft_Center, selSpline.wire_boxZRight_Center = center.z - minExtents.z, maxExtents.z - center.z
    selSpline.wire_extentsL_Center, selSpline.wire_extentsW_Center, selSpline.wire_extentsZ_Center = extents.x, extents.y, extents.z
  end
  pop.tryRemove(selSpline) -- Remove all the static meshes related to this stitch spline from the scene, since the mesh has changed.
  selSpline.isDirty = true
  editor.history:commitAction("Select Static Mesh", { old = statePre, new = splineMgr.deepCopyStitchSpline(selSpline) }, splineMgr.singleSplineEditUndo, splineMgr.singleSplineEditRedo)
end

-- Handles the main tool window.
local function handleMainToolWindowUI()
  if editor.beginWindow(toolWinName, "Stitch Spline###21344", im.WindowFlags_NoCollapse) then
    local icons = editor.icons
    local stitchSplines = splineMgr.getStitchSplines()
    selectedSplineIdx = max(1, min(#stitchSplines, selectedSplineIdx)) -- Ensure the selected spline index is within bounds.

    -- Top buttons row.
    im.Columns(6, "topMasterButtonsRow", false)
    im.SetColumnWidth(0, 39)
    im.SetColumnWidth(1, 39)
    im.SetColumnWidth(2, 39)
    im.SetColumnWidth(3, 39)
    im.SetColumnWidth(4, 39)
    im.SetColumnWidth(5, 39)
    im.PushStyleVar2(im.StyleVar_FramePadding, im.ImVec2(2, 2))
    im.PushStyleVar2(im.StyleVar_ItemSpacing, im.ImVec2(4, 2))

    -- 'Add New stitch spline' button.
    if editor.uiIconImageButton(icons.bSpline, vec36, cols.blueB, nil, nil, 'addNewStitchSplineBtn') then
      local statePre = splineMgr.deepCopyStitchSplineState()
      splineMgr.addNewStitchSpline()
      selectedSplineIdx = #stitchSplines
      editor.history:commitAction("Add New stitch spline", { old = statePre, new = splineMgr.deepCopyStitchSplineState() }, splineMgr.transSplineEditUndo, splineMgr.transSplineEditRedo)
    end
    im.tooltip('Add a new stitch spline.')
    im.SameLine()
    im.NextColumn()

    -- 'Import From Bitmap Mask' button.
    if editor.uiIconImageButton(icons.floppyDiskPlus, vec36, cols.blueB, nil, nil, 'importFromBitmapMaskBtn') then
      extensions.editor_fileDialog.openFile(
        function(data)
          if data.filepath then
            local paths = skeleton.getPathsFromPng(data.filepath)
            if #paths > 0 then
              local preState = splineMgr.deepCopyStitchSplineState()
              splineMgr.convertPathsToStitchSplines(paths)
              editor.history:commitAction("Import Stitch Splines From Bitmap", { old = preState, new = splineMgr.deepCopyStitchSplineState() }, splineMgr.transSplineEditUndo, splineMgr.transSplineEditRedo)
            end
          end
        end,
        {{"PNG",".png"}},
        false,
        "/")
    end
    im.tooltip('Import stitch splines from a bitmap mask.')
    im.SameLine()
    im.NextColumn()

    -- 'Draw A Selection Polygon' button.
    local btnCol = isDrawPolygon and cols.blueB or cols.blueD
    if editor.uiIconImageButton(icons.rounded_corner, vec36, btnCol, nil, nil, 'drawPolygonBtn') then
      isDrawPolygon = not isDrawPolygon
      poly.clearPolygon() -- Ensure there is no residual polygon left over from the previous usage.
    end
    im.tooltip(isDrawPolygon and 'Click to stop drawing a selection polygon.' or 'Click to draw a selection polygon, to convert scene objects to a Stitch Spline.')
    im.SameLine()
    im.NextColumn()

    -- 'Remove All stitch splines' button.
    if #stitchSplines > 0 then
      if editor.uiIconImageButton(icons.trashBin2, vec36, cols.blueB, nil, nil, 'removeAllStitchSplinesBtn') then
        local statePre = splineMgr.deepCopyStitchSplineState()
        splineMgr.removeAllStitchSplines(false)
        selectedSplineIdx = 1
        editor.history:commitAction("Remove All stitch splines", { old = statePre, new = splineMgr.deepCopyStitchSplineState() }, splineMgr.transSplineEditUndo, splineMgr.transSplineEditRedo)
      end
      im.tooltip('Remove all (enabled and not linked) stitch splines from the session.')
    else
      im.Dummy(vec36)
    end
    im.SameLine()
    im.NextColumn()

    -- 'Lock Shape' toggle button.
    local selSpline = stitchSplines[selectedSplineIdx]
    if #stitchSplines > 0 and selSpline and selSpline.isEnabled and not selSpline.isLink then
      local btnCol = isLockShape and cols.blueB or cols.blueD
      if editor.uiIconImageButton(icons.roadGuideArrowSolid, vec36, btnCol, nil, nil, 'lockShapeBtn') then
        isLockShape = not isLockShape
      end
      im.tooltip((isLockShape and 'Unlock the shape of the stitch spline (to move nodes separately)' or 'Lock the shape of the stitch spline (to move nodes rigidly)'))
    else
      im.Dummy(vec36)
    end
    im.SameLine()
    im.NextColumn()

    -- Toggle gizmo on/off button.
    if #stitchSplines > 0 then
      local btnCol, btnIcon = cols.blueD, icons.gizmosOutline
      if isGizmoActive then btnCol, btnIcon = cols.blueB, icons.gizmosSolid end
      if editor.uiIconImageButton(btnIcon, vec36, btnCol, nil, nil, 'toggleGizmoOnOffBtn') then
        isGizmoActive = not isGizmoActive
      end
      im.tooltip('Switch the translational gizmo ' .. (isGizmoActive and 'off' or 'on') .. ' (can also press ALT to toggle).')
    else
      im.Dummy(vec36)
    end
    im.NextColumn()
    im.PopStyleVar(2)
    im.Columns(1)
    im.Separator()

    -- Stitch splines list.
    if #stitchSplines > 0 then
      im.TextColored(cols.greenB, "Stitch Splines:")
      im.PushItemWidth(-1)
      if im.BeginListBox('', im.ImVec2(-1, 180)) then
        im.Columns(5, "splineListBoxColumns", true)
        im.SetColumnWidth(0, 30)
        im.SetColumnWidth(1, 180)
        im.SetColumnWidth(2, 35)
        im.SetColumnWidth(3, 35)
        im.SetColumnWidth(4, 35)
        im.PushStyleVar2(im.StyleVar_FramePadding, im.ImVec2(4, 2))
        im.PushStyleVar2(im.StyleVar_ItemSpacing, im.ImVec2(4, 2))
        local wCtr = 41225
        for i = 1, #stitchSplines do
          local spline = stitchSplines[i]
          local flag = i == selectedSplineIdx
          if im.Selectable1("###" .. tostring(wCtr), flag, bit.bor(im.SelectableFlags_SpanAllColumns, im.SelectableFlags_AllowItemOverlap)) then
            selectedSplineIdx = i
          end
          wCtr = wCtr + 1
          im.SameLine()
          im.NextColumn()

          im.PushItemWidth(180)
          local splineNamePtr = im.ArrayChar(32, spline.name)
          if spline.isLink then
            im.TextColored(cols.dullWhite, spline.name)
            im.tooltip('This stitch spline is linked to a Road Group. To edit or remove it, first unlink it from within the Road Group Editor.')
          elseif not spline.isEnabled then
            im.TextColored(cols.dullWhite, spline.name)
            im.tooltip('This stitch spline is disabled. To edit or remove it, first enable it.')
          else
            if im.InputText("###" .. tostring(wCtr), splineNamePtr, 32) then
              spline.name = ffi.string(splineNamePtr)
              if spline.sceneTreeFolderId then
                local folder = scenetree.findObjectById(spline.sceneTreeFolderId)
                if folder then
                  local preState = splineMgr.deepCopyStitchSpline(spline)
                  folder:setName(spline.name)
                  editor.refreshSceneTreeWindow()
                  spline.isDirty = true -- Ensures the mesh names are updated in the scene tree.
                  local postState = splineMgr.deepCopyStitchSpline(spline)
                  preState.isUpdateSceneTree = true
                  postState.isUpdateSceneTree = true
                  editor.history:commitAction("Edit stitch spline name", { old = preState, new = postState }, splineMgr.singleSplineEditUndo, splineMgr.singleSplineEditRedo)
                end
              end
            end
            im.tooltip('Edit the stitch spline name.')
            if im.IsItemActive() then
              selectedSplineIdx = i
            end
          end
          im.PopItemWidth()
          wCtr = wCtr + 1
          im.SameLine()
          im.NextColumn()

          -- 'Remove Selected Stitch Spline' button.
          if spline.isEnabled and not spline.isLink then
            if editor.uiIconImageButton(icons.trashBin2, vec24, cols.blueB, nil, nil, 'removeSpline') then
              local statePre = splineMgr.deepCopyStitchSplineState()
              splineMgr.removeStitchSpline(i)
              if selectedSplineIdx > i then
                selectedSplineIdx = selectedSplineIdx - 1
              end
              selectedSplineIdx = max(1, min(#stitchSplines, selectedSplineIdx))
              editor.history:commitAction("Remove stitch spline", { old = statePre, new = splineMgr.deepCopyStitchSplineState() }, splineMgr.transSplineEditUndo, splineMgr.transSplineEditRedo)
              return
            end
            im.tooltip('Remove this stitch spline from the session.')
          else
            im.Dummy(vec24)
          end
          im.SameLine()
          im.NextColumn()

          -- 'Lock/Unlock Stitch Spline' button.
          if not spline.isLink then
            local btnCol = spline.isEnabled and cols.blueB or cols.blueD
            local btnIcon = spline.isEnabled and icons.lock or icons.lock_open
            if editor.uiIconImageButton(btnIcon, vec24, btnCol, nil, nil, 'lockUnlockStitchSplineToggleBtn') then
              local statePre = splineMgr.deepCopyStitchSplineState()
              spline.isEnabled = not spline.isEnabled
              pop.tryRemove(spline) -- Remove the stitch spline from the population, before rebuilding the collision mesh.
              spline.isDirty = true
              selectedSplineIdx = i
              editor.history:commitAction("Toggle stitch spline Lock", { old = statePre, new = splineMgr.deepCopyStitchSplineState() }, splineMgr.transSplineEditUndo, splineMgr.transSplineEditRedo)
            end
            im.tooltip((spline.isEnabled and 'Disable' or 'Enable') .. ' this stitch spline.')
          else
            im.Dummy(vec24)
          end
          im.SameLine()
          im.NextColumn()

          -- 'Burn To Scene' button.
          if spline.isEnabled then
            if editor.uiIconImageButton(icons.whatshot, vec24, cols.redB, nil, nil, 'burnToSceneBtn') then
              splineMgr.burnToScene(spline.id)
              return
            end
            im.tooltip("Burn the selected stitch spline to the scene. Warning: This will remove the stitch spline from the editor, and can not be undone.")
          else
            im.Dummy(vec24)
          end
          im.NextColumn()
          im.Separator()
        end
        im.PopStyleVar(2)
        im.EndListBox()
      end
      im.Separator()

      -- Buttons underneath the stitch splines list box.
      im.Columns(5, "buttonsUnderneathListBox", false)
      im.SetColumnWidth(0, 40)
      im.SetColumnWidth(1, 40)
      im.SetColumnWidth(2, 40)
      im.SetColumnWidth(3, 40)
      im.SetColumnWidth(4, 40)

      im.PushStyleVar2(im.StyleVar_FramePadding, im.ImVec2(2, 2))
      im.PushStyleVar2(im.StyleVar_ItemSpacing, im.ImVec2(4, 2))

      -- 'Go To Selected Spline' button.
      if selSpline.nodes and #selSpline.nodes > 1 then
        if editor.uiIconImageButton(icons.cameraFocusTopDown, vec36, cols.blueB, nil, nil, 'goToSelectedSplineBtn') then
          util.goToSpline(selSpline.nodes)
        end
        im.tooltip('Go to this stitch spline (move camera).')
      else
        im.Dummy(vec36)
      end
      im.SameLine()
      im.NextColumn()

      -- 'Conform To Terrain' button.
      if selSpline and selSpline.isEnabled and not selSpline.isLink then
        local btnCol = selSpline.isConformToTerrain and cols.blueB or cols.blueD
        if editor.uiIconImageButton(icons.lineToTerrain, vec36, btnCol, nil, nil, 'conformToTerrainBtn') then
          local statePre = splineMgr.deepCopyStitchSpline(selSpline)
          selSpline.isConformToTerrain = not selSpline.isConformToTerrain
          editor.history:commitAction("Conform To Terrain", { old = statePre, new = splineMgr.deepCopyStitchSpline(selSpline) }, splineMgr.singleSplineEditUndo, splineMgr.singleSplineEditRedo)
        end
        im.tooltip((selSpline.isConformToTerrain and 'Unconform' or 'Conform') .. ' the selected stitch spline to the terrain.')
      else
        im.Dummy(vec36)
      end
      im.SameLine()
      im.NextColumn()

      -- 'Split stitch spline' button.
      if selSpline and selSpline.isEnabled and not selSpline.isLink and selSpline.nodes[selectedNodeIdx] and #selSpline.nodes > 2 and selectedNodeIdx > 1 and selectedNodeIdx < #selSpline.nodes then
        if editor.uiIconImageButton(icons.content_cut, vec36, cols.blueB, nil, nil, 'splitStitchSplineBtn') then
          local statePre = splineMgr.deepCopyStitchSplineState()
          splineMgr.splitStitchSpline(selectedSplineIdx, selectedNodeIdx)
          selectedSplineIdx = #stitchSplines
          editor.history:commitAction("Split New stitch spline", { old = statePre, new = splineMgr.deepCopyStitchSplineState() }, splineMgr.transSplineEditUndo, splineMgr.transSplineEditRedo)
        end
        im.tooltip('Splits the selected stitch spline into two, at the selected node.')
      else
        im.Dummy(vec36)
      end
      im.SameLine()
      im.NextColumn()

      -- Save template button.
      if selSpline.isEnabled then
        if editor.uiIconImageButton(icons.floppyDisk, vec36, nil, nil, nil, 'saveTemplateBtn') then
          extensions.editor_fileDialog.saveFile(
            function(data)
              local preState = splineMgr.deepCopyStitchSplineState()
              jsonWriteFile(data.filepath, splineMgr.copyStitchSplineProfile(selSpline), true)
              editor.history:commitAction("Save Stitch Spline Template", { old = preState, new = splineMgr.deepCopyStitchSplineState() }, splineMgr.transSplineEditUndo, splineMgr.transSplineEditRedo)
            end,
            {{"JSON",".json"}},
            false,
            "/",
            "File already exists.\nDo you want to overwrite the file?")
        end
        im.tooltip('Saves the template of the selected stitch spline to disk.')
      else
        im.Dummy(vec36)
      end
      im.SameLine()
      im.NextColumn()

      -- Load template button.
      if selSpline.isEnabled then
        if editor.uiIconImageButton(icons.roadFolder, vec36, cols.dullWhite, nil, nil, 'loadTemplateBtn') then
          extensions.editor_fileDialog.openFile(
            function(data)
              local preState = splineMgr.deepCopyStitchSplineState()
              splineMgr.pasteStitchSplineProfile(selSpline, jsonReadFile(data.filepath))
              editor.history:commitAction("Load Stitch Spline Template", { old = preState, new = splineMgr.deepCopyStitchSplineState() }, splineMgr.transSplineEditUndo, splineMgr.transSplineEditRedo)
            end,
            {{"JSON",".json"}},
            false,
            "/")
        end
        im.tooltip('Sets the selected stitch spline to a template loaded from disk.')
      else
        im.Dummy(vec36)
      end
      im.NextColumn()
      im.PopStyleVar(2)
      im.Separator()
      im.Columns(1)

      -- If the selected spline is disabled, don't show anything further.
      if not selSpline or not selSpline.isEnabled then
        return
      end

      -- Pole component selection panel.
      im.TextColored(cols.greenB, "Components:")
      im.Columns(3, "staticMeshSelectionColumns", false)
      im.SetColumnWidth(0, 70)
      im.SetColumnWidth(1, 40)
      im.Text('Poles:')
      im.SameLine()
      im.NextColumn()
      if editor.uiIconImageButton(icons.youtube_searched_for, vec24, cols.blueB, nil, nil, 'selectPoleComponentMatBtn') then
        meshTarget = 'pole'
        meshAuditionMgr.addMeshToAudition(selectedMeshIdx, nil)
      end
      im.tooltip('Select a new static mesh for the pole component.')
      im.SameLine()
      im.NextColumn()
      im.SetCursorPosY(im.GetCursorPosY() + max(0, (vec24.y - im.GetTextLineHeight()) * 0.5))
      im.Text(('[' ..selSpline.poleMeshName .. ']') or '[Not Set]')
      im.tooltip('The currently-selected static mesh for the pole component.')
      im.NextColumn()

      -- Wire component selection panel.
      im.Text('Wires:')
      im.SameLine()
      im.NextColumn()
      if editor.uiIconImageButton(icons.youtube_searched_for, vec24, cols.blueB, nil, nil, 'selectWireComponentMatBtn') then
        meshTarget = 'wire'
        meshAuditionMgr.addMeshToAudition(selectedMeshIdx, nil)
      end
      im.tooltip('Select a new static mesh for the wire component.')
      im.SameLine()
      im.NextColumn()
      im.SetCursorPosY(im.GetCursorPosY() + max(0, (vec24.y - im.GetTextLineHeight()) * 0.5))
      im.Text(('[' ..selSpline.wireMeshName .. ']') or '[Not Set]')
      im.tooltip('The currently-selected static mesh for the wire component.')
      im.NextColumn()

      -- Spline component properties.
      im.Columns(1)
      im.Separator()
      im.TextColored(cols.greenB, "Properties:")
      im.PushItemWidth(-1)
      im.PushStyleVar1(im.StyleVar_GrabMinSize, 20)
      im.Columns(2, 'spacingAndJitterCols', false)
      im.SetColumnWidth(0, 30)

      -- Spacing slider.
      if selSpline.spacing ~= defaultParams.spacing then
        if editor.uiIconImageButton(icons.star_border, vec24, cols.blueB, nil, nil, 'resetSpacingBtn') then
          local preEditState = splineMgr.deepCopyStitchSpline(selSpline)
          selSpline.spacing = defaultParams.spacing
          selSpline.isDirty = true
          editor.history:commitAction("Reset Spacing", { old = preEditState, new = splineMgr.deepCopyStitchSpline(selSpline) }, splineMgr.singleSplineEditUndo, splineMgr.singleSplineEditRedo)
        end
        im.tooltip("Reset to default")
      else
        im.Dummy(vec24)
      end
      im.SameLine()
      im.NextColumn()
      im.PushItemWidth(-1)
      local tmpPtr = im.FloatPtr(selSpline.spacing)
      if im.SliderFloat('###5756', tmpPtr, 5.0, 100.0, "Spacing (m) = %.2f") then
        selSpline.spacing = tmpPtr[0]
        selSpline.isDirty = true
      end
      im.tooltip('Set the longitudinal spacing between each pole component, in meters.')
      if im.IsItemActivated() then
        sliderPreEditState = splineMgr.deepCopyStitchSpline(selSpline)
      end
      if im.IsItemDeactivatedAfterEdit() then
        editor.history:commitAction("Adjust Spacing", { old = sliderPreEditState, new = splineMgr.deepCopyStitchSpline(selSpline) }, splineMgr.singleSplineEditUndo, splineMgr.singleSplineEditRedo)
      end
      im.PopItemWidth()
      im.NextColumn()

      -- Sag slider.
      if selSpline.sag ~= defaultParams.sag then
        if editor.uiIconImageButton(icons.star_border, vec24, cols.blueB, nil, nil, 'resetSagBtn') then
          local preEditState = splineMgr.deepCopyStitchSpline(selSpline)
          selSpline.sag = defaultParams.sag
          selSpline.isDirty = true
          editor.history:commitAction("Reset Sag", { old = preEditState, new = splineMgr.deepCopyStitchSpline(selSpline) }, splineMgr.singleSplineEditUndo, splineMgr.singleSplineEditRedo)
        end
        im.tooltip("Reset to default")
      else
        im.Dummy(vec24)
      end
      im.SameLine()
      im.NextColumn()
      im.PushItemWidth(-1)
      local tmpPtr = im.FloatPtr(selSpline.sag)
      if im.SliderFloat('###5316', tmpPtr, 0.0, 5.0, "Sag (m) = %.2f") then
        selSpline.sag = tmpPtr[0]
        selSpline.isDirty = true
      end
      im.tooltip('Set the sag of the wire component, in meters.')
      if im.IsItemActivated() then
        sliderPreEditState = splineMgr.deepCopyStitchSpline(selSpline)
      end
      if im.IsItemDeactivatedAfterEdit() then
        editor.history:commitAction("Adjust Sag", { old = sliderPreEditState, new = splineMgr.deepCopyStitchSpline(selSpline) }, splineMgr.singleSplineEditUndo, splineMgr.singleSplineEditRedo)
      end
      im.PopItemWidth()
      im.NextColumn()

      -- Jitter forward slider.
      if selSpline.jitterForward ~= defaultParams.jitterForward then
        if editor.uiIconImageButton(icons.star_border, vec24, cols.blueB, nil, nil, 'resetJitterForwardBtn') then
          local preEditState = splineMgr.deepCopyStitchSpline(selSpline)
          selSpline.jitterForward = defaultParams.jitterForward
          selSpline.isDirty = true
          editor.history:commitAction("Reset Pitch Jitter", { old = preEditState, new = splineMgr.deepCopyStitchSpline(selSpline) }, splineMgr.singleSplineEditUndo, splineMgr.singleSplineEditRedo)
        end
        im.tooltip("Reset to default")
      else
        im.Dummy(vec24)
      end
      im.SameLine()
      im.NextColumn()
      im.PushItemWidth(-1)
      local tmpPtr = im.FloatPtr(selSpline.jitterForward)
      if im.SliderFloat('###5757', tmpPtr, 0.0, 0.2, "Pitch Jitter = %.3f") then
        selSpline.jitterForward = tmpPtr[0]
        selSpline.isDirty = true
      end
      im.tooltip('Set the amount of random jitter to apply to the pole components, around the local Y-axis (pitch) .')
      if im.IsItemActivated() then
        sliderPreEditState = splineMgr.deepCopyStitchSpline(selSpline)
      end
      if im.IsItemDeactivatedAfterEdit() then
        editor.history:commitAction("Adjust Pitch Jitter", { old = sliderPreEditState, new = splineMgr.deepCopyStitchSpline(selSpline) }, splineMgr.singleSplineEditUndo, splineMgr.singleSplineEditRedo)
      end
      im.PopItemWidth()
      im.NextColumn()

      -- Jitter right slider.
      if selSpline.jitterRight ~= defaultParams.jitterRight then
        if editor.uiIconImageButton(icons.star_border, vec24, cols.blueB, nil, nil, 'resetJitterRightBtn') then
          local preEditState = splineMgr.deepCopyStitchSpline(selSpline)
          selSpline.jitterRight = defaultParams.jitterRight
          selSpline.isDirty = true
          editor.history:commitAction("Reset Yaw Jitter", { old = preEditState, new = splineMgr.deepCopyStitchSpline(selSpline) }, splineMgr.singleSplineEditUndo, splineMgr.singleSplineEditRedo)
        end
        im.tooltip("Reset to default")
      else
        im.Dummy(vec24)
      end
      im.SameLine()
      im.NextColumn()
      im.PushItemWidth(-1)
      local tmpPtr = im.FloatPtr(selSpline.jitterRight)
      if im.SliderFloat('###5758', tmpPtr, 0.0, 0.2, "Yaw Jitter = %.3f") then
        selSpline.jitterRight = tmpPtr[0]
        selSpline.isDirty = true
      end
      im.tooltip('Set the amount of random jitter to apply to the pole components, around the local X-axis (yaw).')
      if im.IsItemActivated() then
        sliderPreEditState = splineMgr.deepCopyStitchSpline(selSpline)
      end
      if im.IsItemDeactivatedAfterEdit() then
        editor.history:commitAction("Adjust Yaw Jitter", { old = sliderPreEditState, new = splineMgr.deepCopyStitchSpline(selSpline) }, splineMgr.singleSplineEditUndo, splineMgr.singleSplineEditRedo)
      end
      im.PopItemWidth()
      im.NextColumn()

      -- Jitter up slider.
      if selSpline.jitterUp ~= defaultParams.jitterUp then
        if editor.uiIconImageButton(icons.star_border, vec24, cols.blueB, nil, nil, 'resetJitterUpBtn') then
          local preEditState = splineMgr.deepCopyStitchSpline(selSpline)
          selSpline.jitterUp = defaultParams.jitterUp
          selSpline.isDirty = true
          editor.history:commitAction("Reset Roll Jitter", { old = preEditState, new = splineMgr.deepCopyStitchSpline(selSpline) }, splineMgr.singleSplineEditUndo, splineMgr.singleSplineEditRedo)
        end
        im.tooltip("Reset to default")
      else
        im.Dummy(vec24)
      end
      im.SameLine()
      im.NextColumn()
      im.PushItemWidth(-1)
      local tmpPtr = im.FloatPtr(selSpline.jitterUp)
      if im.SliderFloat('###5759', tmpPtr, 0.0, 0.2, "Roll Jitter = %.3f") then
        selSpline.jitterUp = tmpPtr[0]
        selSpline.isDirty = true
      end
      im.tooltip('Set the amount of random jitter to apply to the pole components, around the local Z-axis (roll).')
      if im.IsItemActivated() then
        sliderPreEditState = splineMgr.deepCopyStitchSpline(selSpline)
      end
      if im.IsItemDeactivatedAfterEdit() then
        editor.history:commitAction("Adjust Roll Jitter", { old = sliderPreEditState, new = splineMgr.deepCopyStitchSpline(selSpline) }, splineMgr.singleSplineEditUndo, splineMgr.singleSplineEditRedo)
      end
      im.PopItemWidth()
      im.NextColumn()
      im.PopStyleVar()
      im.Columns(1)
    end
  end
end

-- Callback for when the user has finished drawing a selection polygon.
local function onSelectionPolygonComplete(polygon)
  import.importFromPolygon(polygon)
  isDrawPolygon = false
end

-- World editor main callback.
local function onEditorGui()
  -- Ensure all stitch splines are updated, even if this tool is not active.
  -- [This ensures any linked stitch splines are also updated.]
  splineMgr.updateDirtyStitchSplines()

  -- If this tool is not active, leave without doing anything further.
  if not isStitchSplineActive then
    return
  end

  -- Handle the main tool window UI.
  handleMainToolWindowUI()

  -- Handle the mesh audition and selection.
  meshAuditionMgr.handleMeshAuditionAndSelection(meshTarget, onMeshSelected)

  -- Handle the front end if the user is drawing a selection polygon.
  if isDrawPolygon then
    poly.handleUserPolygon(onSelectionPolygonComplete)
    return -- Don't do anything further if the user is drawing a selection polygon.
  end

  -- Handle the mouse and keyboard events.
  local splines = splineMgr.getStitchSplines()
  local isConformToTerrain = selectedSplineIdx and splines[selectedSplineIdx] and splines[selectedSplineIdx].isConformToTerrain
  out.spline, out.node, out.isGizmoActive = selectedSplineIdx, selectedNodeIdx, isGizmoActive
  input.handleSplineEvents(
    splines,
    out,
    false, false, isConformToTerrain, false, false, false, true, true, isLockShape,
    defaultSplineWidth,
    splineMgr.deepCopyStitchSpline,
    splineMgr.copyStitchSplineProfile, splineMgr.pasteStitchSplineProfile,
    nil,
    splineMgr.singleSplineEditUndo, splineMgr.singleSplineEditRedo)
  selectedSplineIdx, selectedNodeIdx, isGizmoActive = out.spline, out.node, out.isGizmoActive

  -- Render the stitch splines with debugDraw.
  render.handleSplineRendering(splines, selectedSplineIdx, selectedNodeIdx, isGizmoActive, false, isLockShape, true)
end

-- Called when the tool mode icon is pressed.
local function onActivate()
  isDrawPolygon = false
  poly.clearPolygon() -- Ensure there is no residual polygon left over from the previous usage.
  editor.clearObjectSelection()
  editor.showWindow(toolWinName)
  isStitchSplineActive = true
end

-- Called when the tool is exited.
local function onDeactivate()
  isDrawPolygon = false
  poly.clearPolygon() -- Ensure there is no residual polygon left over from the previous usage.
  meshAuditionMgr.leaveAuditionView()
  editor.hideWindow(toolWinName)
  isStitchSplineActive = false
end

-- Writes the stitch splines to a file.
local function writeStitchSplinesToFile(filename, splines)
  local file = io.open(filename, "w")
  if not file then
    log('E', logTag, "Could not open stitch spline file for writing: " .. filename)
    return
  end

  for _, spline in ipairs(splines) do
    local nodeStrs, nmlStrs, widthStrs = {}, {}, {}

    for _, n in ipairs(spline.nodes or {}) do
      nodeStrs[#nodeStrs + 1] = string.format('{"x":%.10g,"y":%.10g,"z":%.10g}', n.x, n.y, n.z)
    end

    for _, n in ipairs(spline.nmls or {}) do
      nmlStrs[#nmlStrs + 1] = string.format('{"x":%.10g,"y":%.10g,"z":%.10g}', n.x, n.y, n.z)
    end

    for _, w in ipairs(spline.widths or {}) do
      widthStrs[#widthStrs + 1] = string.format("%.10g", w)
    end

    local jsonLine = string.format(
      '{"id":"%s","name":"%s","isLink":%d,"linkId":%s,"isEnabled":%d,"spacing":%.10g,"sag":%.10g,"isConformToTerrain":%d,' ..
      '"jitterForward":%.10g,"jitterRight":%.10g,"jitterUp":%.10g,' ..
      '"poleMeshPath":"%s","poleMeshName":"%s","pole_extentsW_Center":%.10g,"pole_extentsL_Center":%.10g,"pole_extentsZ_Center":%.10g,' ..
      '"pole_boxXLeft_Center":%.10g,"pole_boxXRight_Center":%.10g,"pole_boxYLeft_Center":%.10g,"pole_boxYRight_Center":%.10g,"pole_boxZLeft_Center":%.10g,"pole_boxZRight_Center":%.10g,' ..
      '"wireMeshPath":"%s","wireMeshName":"%s","wire_extentsW_Center":%.10g,"wire_extentsL_Center":%.10g,"wire_extentsZ_Center":%.10g,' ..
      '"wire_boxXLeft_Center":%.10g,"wire_boxXRight_Center":%.10g,"wire_boxYLeft_Center":%.10g,"wire_boxYRight_Center":%.10g,"wire_boxZLeft_Center":%.10g,"wire_boxZRight_Center":%.10g,' ..
      '"nodes":[%s],"nmls":[%s],"widths":[%s]}',
      spline.id, spline.name,
      spline.isLink and 1 or 0,
      spline.linkId and string.format('"%s"', spline.linkId) or "null",
      spline.isEnabled and 1 or 0,
      spline.spacing, spline.sag, spline.isConformToTerrain and 1 or 0,
      spline.jitterForward, spline.jitterRight, spline.jitterUp,
      spline.poleMeshPath, spline.poleMeshName,
      spline.pole_extentsW_Center, spline.pole_extentsL_Center, spline.pole_extentsZ_Center,
      spline.pole_boxXLeft_Center, spline.pole_boxXRight_Center,
      spline.pole_boxYLeft_Center, spline.pole_boxYRight_Center,
      spline.pole_boxZLeft_Center, spline.pole_boxZRight_Center,
      spline.wireMeshPath, spline.wireMeshName,
      spline.wire_extentsW_Center, spline.wire_extentsL_Center, spline.wire_extentsZ_Center,
      spline.wire_boxXLeft_Center, spline.wire_boxXRight_Center,
      spline.wire_boxYLeft_Center, spline.wire_boxYRight_Center,
      spline.wire_boxZLeft_Center, spline.wire_boxZRight_Center,
      table.concat(nodeStrs, ","), table.concat(nmlStrs, ","), table.concat(widthStrs, ",")
    )

    file:write(jsonLine .. "\n")
  end

  file:close()
end

-- Reads the stitch splines from a file.
local function readStitchSplinesFromFile(filename)
  local file = io.open(filename, "r")
  if not file then
    return {}
  end

  local splines = {}
  for line in file:lines() do
    local ok, data = pcall(jsonDecode, line)
    if ok and data then
      table.insert(splines, splineMgr.deserializeStitchSpline(data, true))
    end
  end

  file:close()
  return splines
end

-- Called once on level save.
local function onEditorBeforeSaveLevel()
  local levelName = getCurrentLevelIdentifier()
  if levelName then
    local splines = splineMgr.getStitchSplines()
    if #splines < 1 then
      return
    end
    local filepath = "levels/" .. levelName .. "/stitchSplineData/" .. diskFileName
    writeStitchSplinesToFile(filepath, splines)
  end
end

-- Called when the World Editor is initialised.
local function loadSessionFromLevelDirectory()
  local levelName = getCurrentLevelIdentifier()
  if levelName then
    local filepath = "levels/" .. levelName .. "/stitchSplineData/" .. diskFileName
    local splines = readStitchSplinesFromFile(filepath)
    splineMgr.setStitchSplines(splines)
  end
end

-- Validates the selection for the scenetree right click menu.
local function validateSceneTreeRightClickMenuSelection(node)
  if node then
    local validCtr = 0
    for _, objId in ipairs(editor.selection.object) do
      local obj = scenetree.findObjectById(objId)
      if obj and obj:getClassName() == "TSStatic" and string.find(obj.shapeName, "pole") then
        validCtr = validCtr + 1
        if validCtr >= 2 then -- Need at least two poles to convert to a stitch spline.
          return true
        end
      end
    end
  end
  return false
end

-- Called on scenetree right click menu, if validated.
local function processSceneTreeRightClickMenuSelection(node)
  if node then
    import.convertTSStatics2StitchSpline(editor.selection.object)
  end
end

-- Called once when the extension is loaded.
local function onExtensionLoaded(serializedData)
  if not serializedData then
    loadSessionFromLevelDirectory() -- Load the stitch splines from the session file in the level directory.
  end
end

-- Called upon world editor initialization.
local function onEditorInitialized()
  editor.editModes.stitchSplineEditMode = {
    displayName = "Stitch Spline",
    onUpdate = nop,
    onActivate = onActivate,
    onDeactivate = onDeactivate,
    icon = editor.icons.link,
    iconTooltip = "StitchSpline",
    auxShortcuts = {},
    hideObjectIcons = true }
  editor.registerWindow(toolWinName, toolWinSize)
  meshAuditionMgr.registerWindow()

  -- Set up the scenetree right click menu for importing.
  editor.addExtendedSceneTreeObjectMenuItem({
    title = "Convert to Stitch Spline",
    extendedSceneTreeObjectMenuItems = processSceneTreeRightClickMenuSelection,
    validator = validateSceneTreeRightClickMenuSelection })
end

-- Called when leaving the map. We need to remove all stitch splines.
local function onClientEndMission()
  splineMgr.removeAllStitchSplines(true) -- We remove all stitch splines (even disabled), so we don't have to worry about bad TSStatic pointers post-load.
end

-- Serialise callback.
local function onSerialize()
  meshAuditionMgr.leaveAuditionView()
  local stitchSplines = splineMgr.getStitchSplines()
  local numSplines = #stitchSplines
  local stitchSplinesSer, ctr = table.new(numSplines, 0), 1
  for i = 1, numSplines do
    stitchSplinesSer[ctr] = splineMgr.serializeStitchSpline(stitchSplines[i])
    ctr = ctr + 1
  end
  splineMgr.removeAllStitchSplines(true)
  return stitchSplinesSer
end

-- Deserialise callback.
local function onDeserialized(data)
  if data and #data > 0 then
    local stitchSplines = splineMgr.getStitchSplines()
    table.clear(stitchSplines)
    for i = 1, #data do
      local spline = splineMgr.deserializeStitchSpline(data[i], true)
      stitchSplines[#stitchSplines + 1] = spline
    end
    selectedSplineIdx = max(1, min(#stitchSplines, selectedSplineIdx))

    -- Update the spline map.
    util.computeIdToIdxMap(stitchSplines, splineMgr.getSplineMap())
  end
end


-- Public interface.
M.onEditorGui =                                           onEditorGui
M.onExtensionLoaded =                                     onExtensionLoaded
M.onEditorInitialized =                                   onEditorInitialized
M.onClientEndMission =                                    onClientEndMission
M.onEditorBeforeSaveLevel =                               onEditorBeforeSaveLevel

M.onSerialize =                                           onSerialize
M.onDeserialized =                                        onDeserialized

return M