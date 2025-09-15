-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- local M = {}

-- local api = extensions.editor_api_dynamicDecals
-- local uiControls = extensions.ui_liveryEditor_controls
-- local uiDecals = extensions.ui_liveryEditor_layers_decals
-- local uiLayers = extensions.ui_liveryEditor_layers
-- local uiCursor = extensions.ui_liveryEditor_layers_cursor
-- local uiMaterialTool = extensions.ui_liveryEditor_tools_material

-- local active = false
-- local reapplyActive = false
-- local applyActive = false
-- local allowApply = true
-- local activeLayerUid
-- local lastLayerPosition
-- local highlightTimer = nil

-- local appliedLayers = {}

-- local setup = function()
--   M.appliedLayers = {}
-- end

-- local setReapplyActive = function(value)
--   M.reapplyActive = value
--   M.editModeStateChanged()
--   guihooks.trigger("liveryEditor_OnEditMode_ReapplyChanged", M.reapplyActive)
-- end

-- local requestReapply = function()
--   M.setReapplyActive(true)
--   M.setAllowApply(true)
--   local layer = api.getLayerByUid(M.activeLayerUid)
--   layer.enabled = false
--   api.setLayer(layer, true)

--   -- set decal properties based on active layer
--   uiCursor.setCursorProperties(layer)
--   uiCursor.notifyListeners()
-- end

-- local cancelReapply = function()
--   M.setReapplyActive(false)
--   local layer = api.getLayerByUid(M.activeLayerUid)
--   layer.enabled = true
--   api.setLayer(layer, true)

--   if layer.type == api.layerTypes.decal then
--     uiDecals.notifyListeners(layer)
--   end

--   M.setAllowApply(false)
-- end

-- M.getActiveLayer = function()
--   if not M.activeLayerUid then
--     return nil
--   else
--     return uiLayers.getLayerByUid(M.activeLayerUid)
--   end
-- end

-- M.apply = function()
--   if M.active and M.allowApply then
--     api.addDecal()
--   end
-- end

-- M.onApply = function(layer)
--   if M.isContinuousApply() and not M.isReapplyActive() and not M.isDuplicateActiveLayer then
--     uiCursor.notifyListeners()
--   else
--     M.setActiveLayer(layer.uid)
--     if M.isDuplicateActiveLayer then
--       M.isDuplicateActiveLayer = false
--       M.requestReapply()
--     else
--       M.setAllowApply(false)
--       M.setApplyActive(false)
--       M.setReapplyActive(false)
--     end
--   end
-- end

-- M.toggleRequestApply = function()
--   if M.applyActive then
--     M.cancelRequestApply()
--   else
--     M.requestApply()
--   end
-- end

-- M.requestApply = function()
--   M.setAllowApply(true)
--   M.setApplyActive(true)
--   uiCursor.notifyListeners()
--   M.setActiveLayer(nil)
-- end

-- M.cancelRequestApply = function()
--   M.setAllowApply(false)
--   M.setApplyActive(false)

--   -- set active layer as top layer
--   if M.appliedLayers and #M.appliedLayers > 0 then
--     for k, v in ipairs(M.appliedLayers) do
--       M.setActiveLayer(v, true)
--       break
--     end
--   end
-- end

-- M.setAllowApply = function(value)
--   M.allowApply = value
--   local color = api.getDecalColor():toTable()
--   if M.allowApply then
--     local isUseMousePos = api.isUseMousePos()
--     if isUseMousePos then
--       uiControls.useMouseProjection()
--     else
--       uiControls.useCursorProjection()
--     end
--     color[4] = 1
--     api.disableDecalHighlighting()
--   else
--     color[4] = 0
--   end
--   api.setDecalColor(Point4F.fromTable(color))
--   M.editModeStateChanged()
-- end

-- M.saveChanges = function(params)
--   M.setActive(false)
--   M.setAllowApply(false)

--   if (params.grouped) then
--     local group = api.addLinkedSet()
--     local index = 1
--     local lastPos = M.lastLayerPosition

--     for k, v in ipairs(M.appliedLayers) do
--       api.moveLayer(lastPos, nil, index, group.uid)
--       index = index + 1
--       lastPos = lastPos - 1
--     end

--     M.appliedLayers = {group.uid}
--   end

--   M.editModeStateChanged()
-- end

-- M.cancelChanges = function()
--   -- get last layer position
--   local lastUid = M.appliedLayers[1]
--   local lastLayer = uiLayers.getLayerByUid(lastUid)
--   local lastPos = lastLayer.order

--   for key, value in ipairs(M.appliedLayers) do
--     api.removeLayer(lastPos, nil)
--     lastPos = lastPos - 1
--   end

--   M.lastLayerPosition = nil
--   M.setActiveLayer(nil)
--   M.setAppliedLayers(nil)
-- end

-- M.activate = function()
--   pushActionMap("liveryEditorEditMode")
--   M.setActive(true)
--   M.setAppliedLayers({})
--   M.toggleRequestApply()
--   M.editModeStateChanged()
-- end

-- M.deactivate = function()
--   M.setActive(false)
--   M.reapplyActive = false
--   M.applyActive = false
--   M.setActiveLayer(nil)
--   M.setAllowApply(false)
--   M.setAppliedLayers(nil)
--   M.editModeStateChanged()
--   popActionMap("liveryEditorEditMode")
-- end

-- M.setActive = function(value)
--   M.active = value
--   api.setEnabled(value)
--   guihooks.trigger("liveryEditor_EditMode_OnActiveStatusChanged", M.active)
--   M.editModeStateChanged()
-- end

-- M.setApplyActive = function(value)
--   M.applyActive = value
--   M.editModeStateChanged()
--   guihooks.trigger("liveryEditor_EditMode_OnRequestApplyChanged", M.applyActive)
-- end

-- M.isReapplyActive = function()
--   return M.reapplyActive
-- end

-- M.isApplyActive = function()
--   return M.applyActive
-- end

-- M.isContinuousApply = function()
--   return api.isUseMousePos()
-- end

-- M.getAppliedLayersData = function()
--   if not M.appliedLayers then
--     return nil
--   end

--   local data = {}
--   for key, value in ipairs(M.appliedLayers) do
--     table.insert(data, uiLayers.getLayerByUid(value))
--   end
--   return data
-- end

-- M.setAppliedLayers = function(data)
--   M.appliedLayers = data
--   M.editModeStateChanged()
--   guihooks.trigger("liveryEditor_EditMode_OnAppliedLayersUpdated", M.getAppliedLayersData())
-- end

-- M.setActiveLayer = function(layerUid)
--   M.activeLayerUid = layerUid

--   if M.activeLayerUid then
--     local layer = api.getLayerByUid(M.activeLayerUid)
--     uiDecals.notifyListeners(api.getLayerByUid(layer.uid))
--     if api.getHighlightedLayer() then
--       api.highlightLayerByUid(M.activeLayerUid)
--     end
--   else
--     api.disableDecalHighlighting()
--   end

--   M.editModeStateChanged()
--   guihooks.trigger("liveryEditor_EditMode_OnActiveLayerChanged", M.activeLayerUid)
-- end

-- M.toggleHighlightActive = function()
--   if M.reapplyActive or applyActive then
--     return
--   end
--   local curr = api.getHighlightedLayer()
--   if M.activeLayerUid and (not curr or curr.uid ~= M.activeLayerUid) then
--     api.highlightLayerByUid(M.activeLayerUid)
--   else
--     api.disableDecalHighlighting()
--   end
-- end

-- M.setActiveLayerDirection = function(direction)
--   if not M.appliedLayers or #M.appliedLayers == 0 then
--     log("W", "", "Cannot change active layer. No applied layers found.")
--     return
--   end

--   if M.isApplyActive() or M.isReapplyActive() then
--     return
--   end

--   local newActiveLayerUid

--   if not M.activeLayerUid then
--     if direction == 1 then
--       newActiveLayerUid = M.appliedLayers[0]
--     else
--       newActiveLayerUid = #M.appliedLayers
--     end
--   else
--     local index

--     for k, v in ipairs(M.appliedLayers) do
--       if v == M.activeLayerUid then
--         index = k
--         break
--       end
--     end

--     if index == 1 and direction == -1 or index == #M.appliedLayers and direction == 1 then
--       return
--     else
--       newActiveLayerUid = M.appliedLayers[index + direction]
--     end
--   end

--   M.setActiveLayer(newActiveLayerUid, true)
-- end

-- M.duplicateActiveLayer = function()
--   local activeLayer = M.getActiveLayer()
--   if not activeLayer then
--     log("W", "", "Cannot duplicate layer, no active layer found")
--     return
--   end

--   M.isDuplicateActiveLayer = true
--   api.duplicateLayer(activeLayer.order, activeLayer.parentUid)
-- end

-- M.removeAppliedLayer = function(layerUid)
--   local index = #M.appliedLayers
--   for k, v in ipairs(M.appliedLayers) do
--     if v == layerUid then
--       index = k
--     end
--   end

--   if layerUid == M.activeLayerUid then
--     -- set next top layer as active
--     local newActiveLayerUid = #M.appliedLayers - 1

--     if index < #M.appliedLayers then
--       newActiveLayerUid = index + 1
--     end

--     M.setActiveLayer(M.appliedLayers[newActiveLayerUid])
--   end

--   local uiLayer = uiLayers.getLayerByUid(layerUid)

--   local newAppliedLayers = shallowcopy(M.appliedLayers)
--   table.remove(newAppliedLayers, index)
--   M.setAppliedLayers(newAppliedLayers)

--   api.removeLayer(uiLayer.order, nil)
-- end

-- M.resetCursorProperties = function(params)
--   uiCursor.resetProperties(params)
--   if M.isApplyActive then
--     uiCursor.notifyListeners()
--   end
-- end

-- M.editModeStateChanged = function()
--   local data = {
--     appliedLayers = M.appliedLayers,
--     activeLayerUid = M.activeLayerUid,
--     active = M.active,
--     applyActive = M.applyActive,
--     reapplyActive = M.reapplyActive
--   }
--   extensions.hook("liveryEditor_editMode_onStateChanged", data)
-- end

-- M.appliedLayers = appliedLayers
-- M.activeLayerUid = activeLayerUid
-- M.active = active
-- M.applyActive = applyActive
-- M.reapplyActive = reapplyActive
-- M.allowApply = allowApply
-- M.lastLayerPosition = lastLayerPosition
-- M.setup = setup
-- M.requestReapply = requestReapply
-- M.cancelReapply = cancelReapply
-- M.setReapplyActive = setReapplyActive
-- M.isDuplicateActiveLayer = false

-- -- External hooks. Do not call!
-- M.liveryEditor_OnLayerAdded = function(layer)
--   dump("liveryEditor_OnLayerAdded > editMode.active", M.active)
--   if not M.active or layer.type == api.layerTypes.linkedSet then
--     return
--   end

--   -- remove old or reapplied layer from applied layers table
--   if M.isReapplyActive() then
--     local oldUid = M.activeLayerUid
--     local index

--     for k, v in ipairs(M.appliedLayers) do
--       if v == oldUid then
--         index = k
--       end
--     end

--     local order = uiLayers.getLayerByUid(oldUid).order

--     -- move new layer to old layers position
--     api.moveLayer(layer.order, nil, order, nil)

--     -- delete old layer which after insert will be at original order + 1
--     api.removeLayer(order + 1)

--     M.appliedLayers[index] = layer.uid
--   else
--     M.lastLayerPosition = layer.order
--     table.insert(M.appliedLayers, 1, layer.uid)
--   end

--   guihooks.trigger("liveryEditor_EditMode_OnAppliedLayersUpdated", M.getAppliedLayersData())
--   M.onApply(layer)
-- end

-- M.liveryEditor_onLayersUpdated = function(layerUid)
--   if not M.active then
--     return
--   end

--   if M.appliedLayers and #M.appliedLayers > 0 then
--     if layerUid == M.activeLayerUid then
--       api.disableDecalHighlighting()
--     end

--     for k, v in ipairs(M.appliedLayers) do
--       if v == layerUid then
--         guihooks.trigger("liveryEditor_EditMode_OnAppliedLayersUpdated", M.getAppliedLayersData())
--         break
--       end
--     end
--   end
-- end

-- M.liveryEditor_OnLayerDeleted = function(layerUid)
--   if not M.active then
--     return
--   end
-- end

-- M.onUpdate = function(dtReal, dtSim)
--   -- dump("update", {dtReal, dtSim})
--   -- if M.highlightTimer then
--   --   M.highlightTimer = M.highlightTimer + dtReal
--   -- end
--   -- if M.highlightTimer and M.highlightTimer >= 5 then
--   --   dump("HEEEEERREEEEE")
--   --   if api.getHighlightedLayer() then
--   --     api.disableDecalHighlighting()
--   --   end
--   --   M.highlightTimer = nil
--   -- end
--   -- if M.highlightTimer and M.highlightTimer > 0 then
--   --   M.highlightTimer = M.highlightTimer - dtReal
--   -- elseif api.getHighlightedLayer() then
--   --   -- api.disableDecalHighlighting()
--   -- end
-- end

-- return M
