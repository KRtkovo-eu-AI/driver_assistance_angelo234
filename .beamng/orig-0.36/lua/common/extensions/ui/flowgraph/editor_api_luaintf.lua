-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local fge = Engine.fge -- shortcut to prevent lookups all the time
local im = ui_imgui

return function(M)
  function M.PtrToId(obj)
    return obj:id()
  end

  function M.NodeIdPtr()
    return fge.NodeId(0)
  end

  function M.LinkIdPtr()
    return fge.LinkId(0)
  end

  function M.PinIdPtr()
    return fge.PinId(0)
  end

  -- enums
  M.StyleColor_Bg = fge.enum.StyleColor_Bg
  M.StyleColor_Grid = fge.enum.StyleColor_Grid
  M.StyleColor_NodeBg = fge.enum.StyleColor_NodeBg
  M.StyleColor_NodeBorder = fge.enum.StyleColor_NodeBorder
  M.StyleColor_HovNodeBorder = fge.enum.StyleColor_HovNodeBorder
  M.StyleColor_SelNodeBorder = fge.enum.StyleColor_SelNodeBorder
  M.StyleColor_NodeSelRect = fge.enum.StyleColor_NodeSelRect
  M.StyleColor_NodeSelRectBorder = fge.enum.StyleColor_NodeSelRectBorder
  M.StyleColor_HovLinkBorder = fge.enum.StyleColor_HovLinkBorder
  M.StyleColor_SelLinkBorder = fge.enum.StyleColor_SelLinkBorder
  M.StyleColor_HighlightLinkBorder = fge.enum.StyleColor_HighlightLinkBorder
  M.StyleColor_LinkSelRect = fge.enum.StyleColor_LinkSelRect
  M.StyleColor_LinkSelRectBorder = fge.enum.StyleColor_LinkSelRectBorder
  M.StyleColor_PinRect = fge.enum.StyleColor_PinRect
  M.StyleColor_PinRectBorder = fge.enum.StyleColor_PinRectBorder
  M.StyleColor_Flow = fge.enum.StyleColor_Flow
  M.StyleColor_FlowMarker = fge.enum.StyleColor_FlowMarker
  M.StyleColor_GroupBg = fge.enum.StyleColor_GroupBg
  M.StyleColor_GroupBorder = fge.enum.StyleColor_GroupBorder
  M.StyleColor_Count = fge.enum.StyleColor_Count

  M.StyleVar_NodePadding = fge.enum.StyleVar_NodePadding
  M.StyleVar_NodeRounding = fge.enum.StyleVar_NodeRounding
  M.StyleVar_NodeBorderWidth = fge.enum.StyleVar_NodeBorderWidth
  M.StyleVar_HoveredNodeBorderWidth = fge.enum.StyleVar_HoveredNodeBorderWidth
  M.StyleVar_SelectedNodeBorderWidth = fge.enum.StyleVar_SelectedNodeBorderWidth
  M.StyleVar_PinRounding = fge.enum.StyleVar_PinRounding
  M.StyleVar_PinBorderWidth = fge.enum.StyleVar_PinBorderWidth
  M.StyleVar_LinkStrength = fge.enum.StyleVar_LinkStrength
  M.StyleVar_SourceDirection = fge.enum.StyleVar_SourceDirection
  M.StyleVar_TargetDirection = fge.enum.StyleVar_TargetDirection
  M.StyleVar_ScrollDuration = fge.enum.StyleVar_ScrollDuration
  M.StyleVar_FlowMarkerDistance = fge.enum.StyleVar_FlowMarkerDistance
  M.StyleVar_FlowSpeed = fge.enum.StyleVar_FlowSpeed
  M.StyleVar_FlowDuration = fge.enum.StyleVar_FlowDuration
  M.StyleVar_FlowMarkerSize = fge.enum.StyleVar_FlowMarkerSize
  M.StyleVar_PivotAlignment = fge.enum.StyleVar_PivotAlignment
  M.StyleVar_PivotSize = fge.enum.StyleVar_PivotSize
  M.StyleVar_PivotScale = fge.enum.StyleVar_PivotScale
  M.StyleVar_PinCorners = fge.enum.StyleVar_PinCorners
  M.StyleVar_PinRadius = fge.enum.StyleVar_PinRadius
  M.StyleVar_PinArrowSize = fge.enum.StyleVar_PinArrowSize
  M.StyleVar_PinArrowWidth = fge.enum.StyleVar_PinArrowWidth
  M.StyleVar_GroupRounding = fge.enum.StyleVar_GroupRounding
  M.StyleVar_GroupBorderWidth = fge.enum.StyleVar_GroupBorderWidth

  M.PinKind_Input = fge.enum.PinKind_Input
  M.PinKind_Output = fge.enum.PinKind_Output

  M.FlowDirection_Forward = fge.enum.FlowDirection_Forward
  M.FlowDirection_Backward = fge.enum.FlowDirection_Backward

  M.IconType_Flow = fge.enum.IconType_Flow
  M.IconType_Circle = fge.enum.IconType_Circle
  M.IconType_Square = fge.enum.IconType_Square
  M.IconType_Grid = fge.enum.IconType_Grid
  M.IconType_RoundSquare = fge.enum.IconType_RoundSquare
  M.IconType_Diamond = fge.enum.IconType_Diamond

  M.Dirty_None       = 0x00000000
  M.Dirty_Navigation = 0x00000001
  M.Dirty_Position   = 0x00000002
  M.Dirty_Size       = 0x00000004
  M.Dirty_Selection  = 0x00000008
  M.Dirty_AddNode    = 0x00000010
  M.Dirty_RemoveNode = 0x00000020
  M.Dirty_User       = 0x00000040

  -- functions
  M.ctx = nil
  if fge.GetCurrentEditor() ~= nil then
    M.ctx = fge.GetCurrentEditor()
  end

  M.SetCurrentEditor = function(ctx)
    M.ctx = ctx
    fge.SetCurrentEditor(ctx)
  end

  M.GetCurrentEditor = fge.GetCurrentEditor
  M.CreateEditor = fge.CreateEditor
  M.DestroyEditor = fge.DestroyEditor

  M.GetStyle = fge.GetStyle
  M.GetStyleColorName = fge.GetStyleColorName
  M.PushStyleColor = fge.PushStyleColor
  M.PopStyleColor = function(count)
    if count == nil then count = 1 end
    fge.PopStyleColor(count)
  end
  M.PushStyleVar1 = fge.PushStyleVar1
  M.PushStyleVar2 = fge.PushStyleVar2
  M.PushStyleVar4 = fge.PushStyleVar4
  M.PopStyleVar = function(count)
    if count == nil then count = 1 end
    fge.PopStyleVar(count)
  end
  M.Begin = function(id, size, readOnly)
    if size == nil then size = im.ImVec2(0, 0) end
    if readOnly == nil then readOnly = false end
    local result = fge.Begin(id, size, readOnly)
    if im then
      local io = im.GetIO(io)
      im.SetWindowFontScale(1/io.FontGlobalScale)
      M.oldImguiScale = im.uiscale[0]
      im.uiscale[0] = 1
    end
    return result
  end
  M.End = function()
    local result = fge.End()
    if im then im.uiscale[0] = M.oldImguiScale end
    im.SetWindowFontScale(1)

    return result
  end
  M.BeginNode = fge.BeginNode
  M.BeginPin = fge.BeginPin
  M.PinRect = fge.PinRect
  M.PinPivotRect = fge.PinPivotRect
  M.PinPivotSize = fge.PinPivotSize
  M.PinPivotScale = fge.PinPivotScale
  M.PinPivotAlignment = fge.PinPivotAlignment
  M.EndPin = fge.EndPin
  M.Group = fge.Group
  M.SetGroupingDisabled = fge.SetGroupingDisabled
  M.EndNode = fge.EndNode
  M.BeginGroupHint = fge.BeginGroupHint
  M.GetGroupMin = fge.GetGroupMin
  M.GetGroupMax = fge.GetGroupMax
  M.GetHintForegroundDrawList = fge.GetHintForegroundDrawList
  M.GetHintBackgroundDrawList = fge.GetHintBackgroundDrawList
  M.EndGroupHint = fge.EndGroupHint
  M.GetNodeBackgroundDrawList = fge.GetNodeBackgroundDrawList
  M.Link = function(id, startPinId, endPinId, color, thickness, isShortcut, shortCutLabel)
    if color == nil then color = im.ImVec4(1, 1, 1, 1) end
    if thickness == nil then thickness = 1.0 end
    if isShortcut == nil then isShortcut = false end
    return fge.Link(id, startPinId, endPinId, color, thickness, isShortcut, shortCutLabel)
  end
  M.Flow = function(linkId, direction)
    if direction == nil then direction = M.FlowDirection_Forward end
    fge.Flow(linkId, direction)
  end
  M.BeginCreate = function(color, thickness)
    if color == nil then color = im.ImVec4(1, 1, 1, 1) end
    if thickness == nil then thickness = 1.0 end
    return fge.BeginCreate(color, thickness)
  end
  M.QueryNewLink1 = fge.QueryNewLink1
  M.QueryNewLink2 = function(startId, endId, color, thickness)
    if thickness == nil then thickness = 1.0 end
    return fge.QueryNewLink2(startId, endId, color, thickness)
  end
  M.QueryNewNode1 = fge.QueryNewNode1
  M.QueryNewNode2 = function(pinId, color, thickness)
    if thickness == nil then thickness = 1.0 end
    return fge.QueryNewNode2(pinId, color, thickness)
  end
  M.AcceptNewItem1 = fge.AcceptNewItem1
  M.AcceptNewItem2 = function(color, thickness)
    if thickness == nil then thickness = 1.0 end
    return fge.AcceptNewItem2(color, thickness)
  end
  M.RejectNewItem1 = fge.RejectNewItem1
  M.RejectNewItem2 = function(color, thickness)
    if thickness == nil then thickness = 1.0 end
    return fge.RejectNewItem2(color, thickness)
  end
  M.EndCreate = fge.EndCreate
  M.BeginDelete = fge.BeginDelete
  M.QueryDeletedLink = fge.QueryDeletedLink
  M.QueryDeletedNode = fge.QueryDeletedNode
  M.AcceptDeletedItem = function(deleteDependencies)
    if deleteDependencies == nil then deleteDependencies = true end
    return fge.AcceptDeletedItem(deleteDependencies)
  end
  M.RejectDeletedItem = fge.RejectDeletedItem
  M.EndDelete = fge.EndDelete
  M.SetNodePosition = fge.SetNodePosition
  M.GetNodePosition = fge.GetNodePosition
  M.GetNodeSize = fge.GetNodeSize
  M.CenterNodeOnScreen = fge.CenterNodeOnScreen
  M.RestoreNodeState = fge.RestoreNodeState
  M.Suspend = fge.Suspend
  M.Resume = fge.Resume
  M.IsSuspended = fge.IsSuspended
  M.IsActive = fge.IsActive
  M.HasSelectionChanged = fge.HasSelectionChanged
  M.GetSelectedObjectCount = fge.GetSelectedObjectCount
  M.GetSelectedNodes = function(nodes, size)
    log('E', 'editor', 'Deprecated function \'GetSelectedNodes\'. Use \'GetSelectedNodeIds\' instead.')
    local nodeIds = M.GetSelectedNodeIds()
    for idx, id in ipairs(nodeIds) do
      nodes[idx - 1] = id
    end
    return #nodeIds
  end
  M.GetSelectedLinks = function(links, size)
    log('E', 'editor', 'Deprecated function \'GetSelectedLinks\'. Use \'GetSelectedLinkIds\' instead.')
    local linkIds = M.GetSelectedLinkIds()
    for idx, id in ipairs(linkIds) do
      links[idx - 1] = id
    end
    return #linkIds
  end
  M.GetSelectedNodeIds = fge.GetSelectedNodeIds
  M.GetSelectedLinkIds = fge.GetSelectedLinkIds
  M.ClearSelection = fge.ClearSelection
  M.SelectNode = function(nodeId, append)
    if nodeId == nil then nodeId = 0 end
    if append == nil then append = false end
    fge.SelectNode(nodeId, append)
  end
  M.SelectLink = function(linkId, append)
    if linkId == nil then linkId = 0 end
    if append == nil then append = false end
    fge.SelectLink(linkId, append)
  end
  M.DeselectNode = fge.DeselectNode
  M.DeselectLink = fge.DeselectLink
  M.DeleteNode = fge.DeleteNode
  M.DeleteLink = fge.DeleteLink
  M.NavigateToContent = function(duration)
    if duration == nil then duration = -1.0 end
    fge.NavigateToContent(duration)
  end
  M.NavigateToSelection = function(zoomIn, duration)
    if zoomIn == nil then zoomIn = false end
    if duration == nil then duration = -1.0 end
    fge.NavigateToSelection(zoomIn, duration)
  end
  M.ShowNodeContextMenu = fge.ShowNodeContextMenu
  M.ShowPinContextMenu = fge.ShowPinContextMenu
  M.ShowLinkContextMenu = fge.ShowLinkContextMenu
  M.ShowBackgroundContextMenu = fge.ShowBackgroundContextMenu
  M.EnableShortcuts = fge.EnableShortcuts
  M.AreShortcutsEnabled = fge.AreShortcutsEnabled
  M.BeginShortcut = fge.BeginShortcut
  M.AcceptCut = fge.AcceptCut
  M.AcceptCopy = fge.AcceptCopy
  M.AcceptPaste = fge.AcceptPaste
  M.AcceptDuplicate = fge.AcceptDuplicate
  M.AcceptCreateNode = fge.AcceptCreateNode
  M.GetActionContextSize = fge.GetActionContextSize
  M.GetActionContextNodes = function(nodes, size)
    log('E', 'editor', 'Deprecated function \'GetActionContextNodes\'. Use \'GetActionContextNodeIds\' instead.')
    local nodeIds = M.GetActionContextNodeIds()
    for idx, id in ipairs(nodeIds) do
      nodes[idx - 1] = id
    end
    return #nodeIds
  end
  M.GetActionContextLinks = function(links, size)
    log('E', 'editor', 'Deprecated function \'GetActionContextLinks\'. Use \'GetActionContextLinkIds\' instead.')
    local linkIds = M.GetActionContextLinkIds()
    for idx, id in ipairs(linkIds) do
      links[idx - 1] = id
    end
    return #linkIds
  end
  M.GetActionContextNodeIds = fge.GetActionContextNodeIds
  M.GetActionContextLinkIds = fge.GetActionContextLinkIds
  M.EndShortcut = fge.EndShortcut
  M.GetCurrentZoom = fge.GetCurrentZoom
  M.GetDoubleClickedNode = fge.GetDoubleClickedNode
  M.GetDoubleClickedPin = fge.GetDoubleClickedPin
  M.GetDoubleClickedLink = fge.GetDoubleClickedLink
  M.IsBackgroundClicked = fge.IsBackgroundClicked
  M.IsBackgroundDoubleClicked = fge.IsBackgroundDoubleClicked
  M.PinHadAnyLinks = fge.PinHadAnyLinks
  M.GetScreenSize = fge.GetScreenSize
  M.ScreenToCanvas = fge.ScreenToCanvas
  M.CanvasToScreen = fge.CanvasToScreen
  M.GetVisibleCanvasBounds = fge.getVisibleBounds
  M.DrawIcon = fge.DrawIcon
  M.Icon = fge.Icon
  M.setDebugEnabled = fge.getDebugEnabled
  M.getDebugEnabled = fge.getDebugEnabled

  M.getViewState = fge.getViewState
  M.setViewState = fge.setViewState

  M.FindLinkAt = fge.FindLinkAt
  M.GetHotObjectId = fge.GetHotObjectId

  M.GetDirtyReason = fge.GetDirtyReason
  M.ClearDirty = fge.ClearDirty
end