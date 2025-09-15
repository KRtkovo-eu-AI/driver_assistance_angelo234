-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

local api = extensions.editor_api_dynamicDecals
local uiTools = extensions.ui_liveryEditor_tools
local uiSelectionApi = extensions.ui_liveryEditor_selection
local uiLayersApi = extensions.ui_liveryEditor_layers

M.moveOrderUp = function()
  -- return uiTools.doOperation(function(layer)
  local layerUid = uiSelectionApi.getFirstSelectedLayer()
  -- local layer = uiLayersApi.getLayerByUid(layerUid)
  -- return M.moveOrderUpById(layer.uid)
  return M.moveOrderUpById(layerUid)
  -- end)
end

M.moveOrderDown = function()
  -- return uiTools.doOperation(function(layer)
  local layerUid = uiSelectionApi.getFirstSelectedLayer()
  return M.moveOrderDownById(layerUid)
  -- end)
end

M.changeOrderToTop = function()
  -- return uiTools.doOperation(function(layer)
  local layerUid = uiSelectionApi.getFirstSelectedLayer()
  local uiLayer = uiLayersApi.getLayerByUid(layerUid)
  local newOrder = uiLayer.siblingCount
  api.moveLayer(uiLayer.order, uiLayer.parentUid, newOrder, uiLayer.parentUid)
  return newOrder
  -- end)
end

M.changeOrderToBottom = function()
  -- return uiTools.doOperation(function(layer)
  local layerUid = uiSelectionApi.getFirstSelectedLayer()
  local uiLayer = uiLayersApi.getLayerByUid(layerUid)
  local newOrder = 2
  api.moveLayer(uiLayer.order, uiLayer.parentUid, newOrder, uiLayer.parentUid)
  return newOrder
  -- end)
end

M.setOrder = function(order)
  -- uiTools.doOperation(function(layer, order)
  local layerUid = uiSelectionApi.getFirstSelectedLayer()
  local uiLayer = uiLayersApi.getLayerByUid(layerUid)
  return api.moveLayer(uiLayer.order, uiLayer.parentUid, order, uiLayer.parentUid)
  -- end, order)
end

M.moveOrderUpById = function(layerUid)
  local uiLayer = uiLayersApi.getLayerByUid(layerUid)
  -- local siblingsCount = uiLayersApi.getChildrenCount(uiLayer.parentUid)

  if uiLayer.order == uiLayer.siblingCount then
    log("W", "", "Unable to move layer " .. uiLayer.uid .. " up with order " .. uiLayer.order)
    return
  end

  local newOrder = uiLayer.order + 1

  api.moveLayer(uiLayer.order, uiLayer.parentUid, newOrder, uiLayer.parentUid)
  return newOrder
end

M.moveOrderDownById = function(layerUid)
  local uiLayer = uiLayersApi.getLayerByUid(layerUid)
  -- local siblingsCount = uiLayersApi.getChildrenCount(uiLayer.parentUid)

  if uiLayer.order == 2 then
    log("W", "", "Unable to move layer " .. uiLayer.uid .. " down with order " .. uiLayer.order)
    return
  end

  local newOrder = uiLayer.order - 1
  api.moveLayer(uiLayer.order, uiLayer.parentUid, newOrder, uiLayer.parentUid)
  return newOrder
end

M.changeOrder = function(oldOrder, oldParentUid, newOrder, newParentUid)
  if oldParentUid == "" then
    oldParentUid = nil
  end
  if newParentUid == "" then
    newParentUid = nil
  end
  api.moveLayer(oldOrder, oldParentUid, newOrder, newParentUid)
end

M.groupLayers = function()
  local selectedLayerUids = uiSelectionApi.getSelectedLayers()

  -- create linked group layer here
  -- get layer with shallowest level and use its parent as group's parent
  local shallowestLevel
  local parentUid
  for k, layerUid in ipairs(selectedLayerUids) do
    local uiLayer = uiLayersApi.getLayerByUid(layerUid)
    local level = uiLayer.path and #uiLayer.path or 0

    if not shallowestLevel or level < shallowestLevel then
      shallowestLevel = level
      parentUid = uiLayer.parentUid
    end
  end

  local newGroup = api.addLinkedSet({parentUid})

  -- move selected layers to this group
  local newOrder = 1
  for k, layerUid in ipairs(selectedLayerUids) do
    local uiLayer = uiLayersApi.getLayerByUid(layerUid)
    api.moveLayer(uiLayer.order, uiLayer.parentUid, newOrder, newGroup.uid)
    newOrder = newOrder + 1
  end

  -- select new group
  uiSelectionApi.select(newGroup.uid)
end

M.ungroupLayer = function()
  uiTools.doOperation(function(layer)
    local uiLayer = uiLayersApi.getLayerByUid(layer.uid)

    -- Need to store layer ids first instead of directly calling moveLayer
    -- to avoid getting reference and the latest data after moving layer
    local childLayerUids = {}

    for k, childLayer in ipairs(uiLayer.children) do
      table.insert(childLayerUids, childLayer.uid)
    end

    local layerIndex = uiLayer.order
    for k, layerUid in ipairs(childLayerUids) do
      local layer = uiLayersApi.getLayerByUid(layerUid)
      api.moveLayer(layer.order, layer.parentUid, layerIndex, uiLayer.parentUid)
      layerIndex = layerIndex + 1
    end

    uiLayer = uiLayersApi.getLayerByUid(layer.uid)
    api.removeLayer(uiLayer.order, uiLayer.parentUid)
  end)
end

return M
