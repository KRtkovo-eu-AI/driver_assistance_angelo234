-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = 'editor_roadEditor'
local actionMapName = "RoadEditor"
local editModeName = "Edit Road"
local roadRiverGui = extensions.editor_roadRiverGui
local im = ui_imgui
local roadTemplatesActive = false

local u_32_max_int = 4294967295
local xVector = vec3(1,0,0)
local yVector = vec3(0,1,0)
local zVector = vec3(0,0,1)

-- Hovered Node info is a pair of RoadID and NodeID
local hoveredNodeInfo = {}
local hoveredRoadID = nil
local lastHoveredRoadID = nil
local selectedRoadsIds  = {}

local selectedNodes = {}

local hoveredRoadsIDs = {}
local hoveredRoadsIndex = 1

local duplicatedRoadIDs = {}

local tempNodeIndexes = {}

-- Used as reference start point for multi-road drawing
local addTempNodeStartPos = nil

local mouseButtonHeldOnNode = false
local oldNodeWidth = nil

local templateDialogOpen = im.BoolPtr(false)

-- Params for setting width of node by dragging the mouse after creating it
local dragMouseStartPos = vec3(0,0,0)
local dragNodesStartPositions = {}

-- Params for Rectangle Selection of Nodes on Selected Roads
local rectSelectDragMouseStartPos = vec3(0,0,0)
local rectSelectDragMouseLastPos = vec3(0,0,0)
local isRectSelecting = false

-- Param for dragging nodes
local dragStartPosition = nil

local useTemplate = im.BoolPtr(false)

local fieldsCopy = nil
local differentValuesColor = im.ImVec4(1, 0.2, 0, 1)

local lastMousePos = im.ImVec2(0,0)
local roadMaterialTagString = "RoadAndPath"
local roadNotSelectableErrorWindowName = "roadNotSelectableErrorWindowName"
local roadNotSelectableErrorWindowTitle = "Road Select Error"
local shouldShowFuseOption = false

local function showAIModeText()
  local vm = GFXDevice.getVideoMode()
  local w, h = vm.width, vm.height
  local windowAspectRatio = w/h

  local pos = core_camera.getPosition()
  local q = core_camera.getQuat()
  local dist = 10
  local fovRadians = (core_camera.getFovRad() or 60)
  local x, y, z = q * xVector, q * yVector, q * zVector

  local center = pos + y*dist
  local height =  (math.tan(fovRadians/2) * dist)
  local width = (height * windowAspectRatio)

  local textPos = center - x*width/3 + z*height/3
  textPos = textPos + x*0.05 - z*0.05
  debugDrawer:drawTextAdvanced(textPos, String("Only AI roads selectable"), ColorF(0,0,0,1), false, false, ColorI(0,0,0,255), false, false)
end

local function indexOf(list, value)
  for index, v in ipairs(list) do
    if value == v then
      return index
    end
  end
  return nil
end

local function selectNodesRange(first, last, roadID)
  selectedNodes[roadID] = {}
  for i = first, last do
    table.insert(selectedNodes[roadID], i)
  end
end

local function onSelectAll()
  if not tableIsEmpty(selectedRoadsIds) then
    for _, roadID in ipairs(selectedRoadsIds) do
      local road = scenetree.findObjectById(roadID)
      if road then
        selectNodesRange(0, road:getNodeCount()-1, roadID)
      end
    end
  end
end

local function isNodeSelected(roadId, nodeId)
  if not selectedNodes[roadId] then return false end
  for _, id in ipairs(selectedNodes[roadId]) do
    if nodeId == id then return true end
  end
  return false
end

local function deselectNode(roadId, nodeId)
  if not selectedNodes[roadId] then return end
  for _, id in ipairs(selectedNodes[roadId]) do
    if nodeId == id then
      local nodeIndex = indexOf(selectedNodes[roadId], id)
      table.remove(selectedNodes[roadId], nodeIndex)
      break
    end
  end
end

local function isRoadSelected(roadID)
  for _, selectedRoadID in ipairs(selectedRoadsIds) do
    if selectedRoadID == roadID then
      return true
    end
  end
  return false
end

local function selectNode(roadId, id, selectMode)
  if roadId == nil then
    selectedNodes = {}
    return
  end

  if selectMode then
    if selectMode == editor.SelectMode_Add then
      selectedNodes[roadId] = selectedNodes[roadId] or {}
      table.insert(selectedNodes[roadId], id)
    end
  else
    if editor.keyModifiers.ctrl then
      if isNodeSelected(roadId, id) then
        deselectNode(roadId, id)
      else
        selectedNodes[roadId] = selectedNodes[roadId] or {}
        table.insert(selectedNodes[roadId], id)
      end
    elseif editor.keyModifiers.shift then
      local selNodes = selectedNodes[roadId]
      if not tableIsEmpty(selNodes) then
        local lastSelNode = selNodes[#selNodes]
        selectNodesRange(math.min(lastSelNode, id), math.max(lastSelNode, id), roadId)
      else
        selectedNodes[roadId] = selectedNodes[roadId] or {}
        table.insert(selectedNodes[roadId], id)
      end
    else
      selectedNodes = {}
      selectedNodes[roadId] = {}
      table.insert(selectedNodes[roadId], id)
    end
  end
end

local function selectNodes(nodeIDsTbl)
  selectedNodes = {}
  for roadID, arrayNodeIDs in pairs(nodeIDsTbl) do
    if not arrayNodeIDs then goto continue end
    selectedNodes[roadID] = {}
    for _, nodeID in ipairs(arrayNodeIDs) do
      table.insert(selectedNodes[roadID], nodeID)
    end
    ::continue::
  end
end

local function deleteNode(road, nodeID)
  editor.deleteRoadNode(road:getID(), nodeID)
  editor_roadUtils.reloadDecorations(road)
  editor_roadUtils.reloadDecals(road)
  editor_roadUtils.updateChildRoads(road, nodeID)
end

local function setNodeWidth(road, nodeID, width, safeStartWidth)
  editor.setNodeWidth(road, nodeID, width)
  editor.updateRoadVertices(road)
end

-- Paste Fields
local function pasteActionUndo(actionData)
  editor.pasteFields(actionData.oldFields, actionData.roadId)
end

local function pasteActionRedo(actionData)
  editor.pasteFields(actionData.newFields, actionData.roadId)
end

local function pasteFieldsAM()
  if tableSize(selectedRoadsIds) == 1 and fieldsCopy then
    editor.history:commitAction("PasteRoad", {oldFields = editor.copyFields(selectedRoadsIds[1]), newFields = deepcopy(fieldsCopy), roadId = selectedRoadsIds[1]}, pasteActionUndo, pasteActionRedo)
  end
end

-- Set all Nodes Width
local function setNodesWidthActionUndo(actionData)
  for roadID, nodeWidthsTbl in pairs(actionData.oldWidths) do
    local road = scenetree.findObjectById(roadID)
    if road then
      for nodeID, oldWidth in pairs(nodeWidthsTbl) do
        editor.setNodeWidth(road, nodeID, oldWidth)
      end
    end
  end
end

local function setNodesWidthActionRedo(actionData)
  for roadID, nodeWidthsTbl in pairs(actionData.newWidths) do
    local road = scenetree.findObjectById(roadID)
    if road then
      for nodeID, newWidth in pairs(nodeWidthsTbl) do
        editor.setNodeWidth(road, nodeID, newWidth)
      end
    end
  end
end

-- Position Node
local function positionNodeActionUndo(actionData)
  for roadID, nodes in pairs(actionData.roadAndNodeIDs) do
    local road = scenetree.findObjectById(roadID)
    if not road then goto continue end
    for _, nodeID in ipairs(nodes) do
      local roadNodePositions = actionData.oldPositions[roadID]
      if roadNodePositions and roadNodePositions[nodeID] then
        editor.setNodePosition(road, nodeID, roadNodePositions[nodeID])
      end
    end
    ::continue::
  end
end

local function positionNodeActionRedo(actionData)
  for roadID, nodes in pairs(actionData.roadAndNodeIDs) do
    local road = scenetree.findObjectById(roadID)
    if not road then goto continue end
    for _, nodeID in ipairs(nodes) do
      local roadNodePositions = actionData.newPositions[roadID]
      if roadNodePositions and roadNodePositions[nodeID] then
        editor.setNodePosition(road, nodeID, roadNodePositions[nodeID])
      end
    end
    ::continue::
  end
end

local function setNodePosition(road, nodeID, position)
  editor.setNodePosition(road, nodeID, position)
  editor.updateRoadVertices(road)
end

-- Insert Node
local function insertNodeActionUndo(actionData)
  -- Loop the nodes from back to front
  for roadID, nodeInfo in pairs(actionData.roadInfos) do
    local road = scenetree.findObjectById(roadID)
    deleteNode(road, nodeInfo.index)
  end
end

local function insertNodeActionRedo(actionData)
  for roadID, nodeInfo in pairs(actionData.roadInfos) do
    editor.addRoadNode(roadID, nodeInfo)
  end
end

local function insertNode(road, position, width, index, withUndo)
  local nodeInfo = {pos = position, width = width, index = index}
  if withUndo then
    local roadInfoTbl = {}
    roadInfoTbl[road:getID()] = nodeInfo
    return editor.history:commitAction("InsertRoadNode", {roadInfos = roadInfoTbl}, insertNodeActionUndo, insertNodeActionRedo)
  else
    return editor.addRoadNode(road:getID(), nodeInfo)
  end
end

local function deleteSelectionActionRedo(actionData)
  for roadID, _ in pairs(actionData.roadInfos) do
    editor.deleteRoad(roadID)
  end

  -- Firstly, sort nodeInfos table in descending index order then delete the nodes.
  -- Otherwise we will face invalid index issues because of deleting during iteration.
  for _, nodeInfos in pairs(actionData.nodeInfos) do
    if nodeInfos then
      table.sort(nodeInfos, function(nodeInfo1, nodeInfo2)
        return (nodeInfo1.index or "") > (nodeInfo2.index or "")
      end)
    end
  end

  for roadID, nodeInfos in pairs(actionData.nodeInfos) do
    for _, nodeInfo in ipairs(nodeInfos) do
      local road = scenetree.findObjectById(roadID)
      deleteNode(road, nodeInfo.index)
    end
  end
end

local function deleteSelectionActionUndo(actionData)
  for roadID, roadInfo in pairs(actionData.roadInfos) do
    SimObject.setForcedId(roadID)
    editor.createRoad(actionData.nodes[roadID], roadInfo)
    editor.selectObjectById(roadID, editor.SelectMode_Add)
  end

  -- Firstly, sort nodeInfos table in ascending index order then add the nodes.
  -- Previous nodes must have been added so that current node index will be valid.
  for _, nodeInfos in pairs(actionData.nodeInfos) do
    if nodeInfos then
      table.sort(nodeInfos, function(nodeInfo1, nodeInfo2)
        return (nodeInfo1.index or "") < (nodeInfo2.index or "")
      end)
    end
  end

  for roadID, nodeInfos in pairs(actionData.nodeInfos) do
    editor.selectObjectById(roadID, editor.SelectMode_Add)
    for _, nodeInfo in ipairs(nodeInfos) do
      editor.addRoadNode(roadID, nodeInfo)
    end
  end
end

-- Delete Node
local deleteNodeActionUndo = insertNodeActionRedo
local deleteNodeActionRedo = insertNodeActionUndo

-- Create Road
local function createRoadActionUndo(actionData)
  editor.deleteRoad(actionData.roadID)
  editor.clearObjectSelection()
end

local function createRoadActionRedo(actionData)
  if actionData.roadID then
    SimObject.setForcedId(actionData.roadID)
  end
  actionData.roadID = editor.createRoad(actionData.nodes, actionData.roadInfo)
  editor.selectObjectById(actionData.roadID)
end

-- Duplicate Road
local function duplicateRoadActionUndo(actionData)
  for _, roadID in ipairs(actionData.arrayRoadIDs) do
    editor.deleteRoad(roadID)
    local roadIndex = indexOf(duplicatedRoadIDs, roadID)
    table.remove(duplicatedRoadIDs, roadIndex)
  end
  editor.clearObjectSelection()
end

local function duplicateRoadActionRedo(actionData)
  local newRoadIDs = {}
  for index, nodes in ipairs(actionData.nodes) do
    if actionData.arrayRoadIDs then
      SimObject.setForcedId(actionData.arrayRoadIDs[index])
    end
    local newRoadID = editor.createRoad(nodes, actionData.roadInfos[index])
    editor.selectObjectById(newRoadID, editor.SelectMode_Add)
    table.insert(duplicatedRoadIDs, newRoadID)
    table.insert(newRoadIDs, newRoadID)
  end

  actionData.arrayRoadIDs = {}

  for _, newRoadID in ipairs(newRoadIDs) do
    table.insert(actionData.arrayRoadIDs, newRoadID)
  end
end

-- Delete Road
local deleteRoadActionUndo = createRoadActionRedo
local deleteRoadActionRedo = createRoadActionUndo

-- Split Road
local function splitRoadActionUndo(actionData)
  if tableIsEmpty(actionData.originalRoadAndNodeIDs) then return end
  local roadIndex = 1
  for originalRoadID, nodeID in pairs(actionData.originalRoadAndNodeIDs) do
    local originalRoad = scenetree.findObjectById(originalRoadID)
    deleteNode(originalRoad, originalRoad:getNodeCount() - 1)
    local newRoad = scenetree.findObjectById(actionData.newRoadIDs[roadIndex])
    -- Loop through all the nodes
    for _, node in ipairs(editor.getNodes(newRoad)) do
      insertNode(originalRoad, node.pos, node.width, u_32_max_int)
    end
    roadIndex = roadIndex + 1
  end

  for _, roadID in pairs(actionData.newRoadIDs) do
    editor.deleteRoad(roadID)
  end

  for roadID, _ in pairs(actionData.originalRoadAndNodeIDs) do
    editor.selectObjectById(roadID, editor.SelectMode_Add)
  end

end

local function splitRoadActionRedo(actionData)
  if tableIsEmpty(actionData.originalRoadAndNodeIDs) then return end
  local newRoadIds = {}
  local roadIndex = 1
  for originalRoadID, nodeID in pairs(actionData.originalRoadAndNodeIDs) do
    local originalRoad = scenetree.findObjectById(originalRoadID)
    if originalRoad then
      local newRoadNodes = {}
      -- Loop through all the nodes
      for id, node in ipairs(editor.getNodes(originalRoad)) do
        if (id - 1) == nodeID then
          table.insert(newRoadNodes, node)
        elseif (id - 1) > nodeID then
          table.insert(newRoadNodes, node)
          deleteNode(originalRoad, nodeID + 1)
        end
      end

      if actionData.newRoadIDs then
        SimObject.setForcedId(actionData.newRoadIDs[roadIndex])
      end
       local roadID = editor.createRoad(newRoadNodes, editor.copyFields(originalRoad:getID()))
       table.insert(newRoadIds, roadID)
      roadIndex = roadIndex + 1
    end
  end
  actionData.newRoadIDs = deepcopy(newRoadIds)
end

local function splitRoads(roadAndNodeIDs)
  for roadID, nodeID in pairs(roadAndNodeIDs) do
    local road = scenetree.findObjectById(roadID)
    if nodeID == 0 or nodeID == road:getNodeCount() - 1 then
      editor.logError("Can't split at the end of road "..tostring(road:getID()))
      return
    end
  end
  -- Split the road and return the id of the new road
  editor.history:commitAction("SplitRoad",
    {originalRoadAndNodeIDs = roadAndNodeIDs}, splitRoadActionUndo, splitRoadActionRedo)
end

local function fuseRoadsActionUndo(actionData)
  if tableIsEmpty(actionData.fuseRoadsInfo) then return end

  editor.deleteRoad(actionData.fuseRoadsInfo[1].roadID)

  SimObject.setForcedId(actionData.fuseRoadsInfo[1].roadID)
  editor.createRoad(actionData.road1Nodes, actionData.road1Fields)

  SimObject.setForcedId(actionData.fuseRoadsInfo[2].roadID)
  editor.createRoad(actionData.road2Nodes, actionData.road2Fields)

  for _, nodeInfo in pairs(actionData.fuseRoadsInfo) do
    editor.selectObjectById(nodeInfo.roadID, editor.SelectMode_Add)
  end
end

local function fuseRoadsActionRedo(actionData)
  if tableIsEmpty(actionData.fuseRoadsInfo) then return end
  local road1Data = actionData.fuseRoadsInfo[1]
  local road2Data = actionData.fuseRoadsInfo[2]

  local road1 = scenetree.findObjectById(road1Data.roadID)
  local road2 = scenetree.findObjectById(road2Data.roadID)

  local insertIndex = (road1Data.fuseNodeIndex == 1) and 0 or u_32_max_int
  local road2Nodes = editor.getNodes(road2)
  local numNodes = road2:getNodeCount()
  if road2Data.fuseNodeIndex == 1 then
    for _, node in ipairs(road2Nodes) do
      insertNode(road1, node.pos, node.width, insertIndex)
    end
  else
    for index = 1, tableSize(road2Nodes) do
      local node = road2Nodes[numNodes - index + 1]
      insertNode(road1, node.pos, node.width, insertIndex)
    end
  end

  editor.deleteRoad(road2Data.roadID)
  editor.selectObjectById(road1Data.roadID)
end

local function fuseRoads()
  if tableSize(selectedRoadsIds) ~= 2 then return end
  local selectedRoad1 = scenetree.findObjectById(selectedRoadsIds[1])
  local selectedRoad2 = scenetree.findObjectById(selectedRoadsIds[2])
  if not selectedRoad1 or not selectedRoad2 then return end
  local nodesRoad1 = editor.getNodes(selectedRoad1)
  local nodesRoad2 = editor.getNodes(selectedRoad2)
  local edgeNodes1 = {1, selectedRoad1:getNodeCount()}
  local edgeNodes2 = {1, selectedRoad2:getNodeCount()}

  local minDist = math.huge
  local fuseIndex1 = nil
  local fuseIndex2 = nil
  for _, nodeIndex1 in ipairs(edgeNodes1) do
    for _, nodeIndex2 in ipairs(edgeNodes2) do
      local distVec = nodesRoad1[nodeIndex1].pos - nodesRoad2[nodeIndex2].pos
      local dist = distVec:length()
      if minDist > dist then
        minDist = dist
        fuseIndex1 = nodeIndex1
        fuseIndex2 = nodeIndex2
      end
    end
  end

  local fuseRoadInfoTbl = {{roadID = selectedRoadsIds[1], fuseNodeIndex = fuseIndex1}, {roadID = selectedRoadsIds[2], fuseNodeIndex = fuseIndex2}}
  editor.history:commitAction("FuseRoads",
    {fuseRoadsInfo = fuseRoadInfoTbl, road1Nodes = nodesRoad1, road2Nodes = nodesRoad2,
      road1Fields = editor.copyFields(selectedRoadsIds[1]), road2Fields = editor.copyFields(selectedRoadsIds[2])}, fuseRoadsActionUndo, fuseRoadsActionRedo)
end

local function setAsDefault(decalRoadId)
end

local function templateDialog()
  --TODO: convert to editor.beginWindow/endWindow
  if templateDialogOpen[0] then
    im.Begin("Templates", templateDialogOpen, 0)
      for i=1, #editor_roadUtils.getMaterials() do
        im.PushID1(string.format('template_%d', i))
        if im.ImageButton(editor_roadUtils.getMaterials()[i].texId, im.ImVec2(128, 128), im.ImVec2Zero, im.ImVec2One, 1, im.ImColorByRGB(0,0,0,255).Value, im.ImColorByRGB(255,255,255,255).Value) then
          templateDialogOpen[0] = false
          editor.setDynamicFieldValue(selectedRoadId, "template", editor_roadUtils.getRoadTemplateFiles()[i])
          editor_roadUtils.reloadTemplates()
        end
        if im.IsItemHovered() then
          im.BeginTooltip()
          im.PushTextWrapPos(im.GetFontSize() * 35.0)
          im.TextUnformatted(string.format("%d x %d", editor_roadUtils.getMaterials()[i].size.x, editor_roadUtils.getMaterials()[i].size.y ))
          im.TextUnformatted(string.format("%s", editor_roadUtils.getRoadTemplateFiles()[i] ))
          im.PopTextWrapPos()
          im.EndTooltip()
        end
        im.PopID()
        if i%4 ~= 0 then im.SameLine() end
      end
    im.End()
  end
end

local function isAnyNodeSelected(roadID)
  local anyNodeSelected = false
  if roadID then
    if selectedNodes and selectedNodes[roadID] then
      if not tableIsEmpty(selectedNodes[roadID]) then
        anyNodeSelected = true
      end
    end
  else
    if tableIsEmpty(selectedNodes) then return false end
    for _, arrayNodes in pairs(selectedNodes) do
      if arrayNodes and not tableIsEmpty(arrayNodes) then
        anyNodeSelected = true
        break
      end
    end
  end
  return anyNodeSelected
end

local function getSelectedSingleNodeInRoad(roadID)
  local nodeId = -1
  if tableIsEmpty(selectedNodes) then return nodeId end
  local selectedNodesInRoad = selectedNodes[roadID]
  if selectedNodesInRoad then
    nodeId = (tableSize(selectedNodesInRoad) == 1) and selectedNodesInRoad[1] or -1
  end
  return nodeId
end

local function getRoadTempNodeIndex(roadID)
  local tempNodeIndex = -1
  if not roadID or tableIsEmpty(tempNodeIndexes) then
    return tempNodeIndex
  end
  tempNodeIndex = tempNodeIndexes[roadID] and tempNodeIndexes[roadID] or -1

  return tempNodeIndex
end

local editingPos = false
local nodePosition = im.ArrayFloat(3)

local editingWidth = false
local nodeWidth = im.FloatPtr(0)

local widthSliderEditEnded = im.BoolPtr(false)

local function onEditorInspectorHeaderGui(inspectorInfo)
  if not editor.editMode or (editor.editMode.displayName ~= editModeName) then
    return
  end

  local isAllSameFunc = function(valArray)
    local isAllSame = true
    if tableIsEmpty(valArray) then return false end
    local fieldVal = valArray[1]
    for _, val in ipairs(valArray) do
      if val ~= fieldVal then
        isAllSame = false
        break
      end
    end
    return isAllSame
  end

  local allSelectedNodePositionsArray = {}
  local allSelectedNodeWidthsArray = {}
  local nodePosVisible = nil
  local nodeWidthVisible = nil
  for roadID, nodesTbl in pairs(selectedNodes) do
    local selectedRoad = scenetree.findObjectById(roadID)
    if selectedRoad then
      for _, nodeID in ipairs(nodesTbl) do
        local selNodePos = selectedRoad:getNodePosition(nodeID)
        local selNodeWidth = selectedRoad:getNodeWidth(nodeID)
        table.insert(allSelectedNodePositionsArray, selNodePos)
        table.insert(allSelectedNodeWidthsArray, selNodeWidth)
        if not nodePosVisible then nodePosVisible = selNodePos end
        if not nodeWidthVisible then nodeWidthVisible = selNodeWidth end
      end
    end
  end

  local nodePositionsAllSame = isAllSameFunc(allSelectedNodePositionsArray)
  local nodeWidthsAllSame = isAllSameFunc(allSelectedNodeWidthsArray)

  local selectedRoad = scenetree.findObjectById(selectedRoadsIds[1])
  if selectedRoad and #selectedRoadsIds == 1 then
    useTemplate[0] = (selectedRoad:getField("useTemplate", "") == "true")
    if roadTemplatesActive then
      im.SameLine()
      if im.Checkbox("Use Template", useTemplate) then
        editor.setDynamicFieldValue(selectedRoad:getID(), "useTemplate", tostring(useTemplate[0]))
      end
    end

    -- The button to open the template window with
    if useTemplate[0] then
      local materialName = selectedRoad:getField("Material", "")
      local matIndex = indexOf(editor_roadUtils.getMaterialNames(), materialName)
      local texID = 0
      if matIndex then
        if im.ImageButton(editor_roadUtils.getMaterials()[matIndex].texId, im.ImVec2(128, 128), im.ImVec2Zero, im.ImVec2One, 1, im.ImColorByRGB(0,0,0,255).Value, im.ImColorByRGB(255,255,255,255).Value) then
          templateDialogOpen[0] = true
        end
      else
        if im.Button("Change Template") then
          templateDialogOpen[0] = true
        end
      end
    end
    templateDialog()
    im.Text(string.format("Road Length: %0." .. editor.getPreference("ui.general.floatDigitCount") .. "f m", selectedRoad:getField("debugRoadLength", "")))
  end

  -- Display node properties of selected node
  if (isAnyNodeSelected()) then
    im.BeginChild1("node", im.ImVec2(0, 130), true)
    im.Text("Node Properties")
    local positionSliderEditEnded = im.BoolPtr(false)
    -- Create the field for node position
    if not editingPos then
      local pos = nodePositionsAllSame and nodePosVisible or vec3(0, 0, 0)
      nodePosition[0] = pos.x
      nodePosition[1] = pos.y
      nodePosition[2] = pos.z
    end

    if not nodePositionsAllSame then
      im.PushStyleColor2(im.Col_Text, differentValuesColor)
    end

    if editor.uiDragFloat3("Node Position", nodePosition, 0.2, -1000000000, 100000000, "%0." .. editor.getPreference("ui.general.floatDigitCount") .. "f", 1, positionSliderEditEnded) then
      editingPos = true
    end

    if not nodePositionsAllSame then
      im.PopStyleColor()
    end

    if positionSliderEditEnded[0] then
      local oldPositionsTbl = {}
      local newPositionsTbl = {}

      for roadID, nodes in pairs(selectedNodes) do
        newPositionsTbl[roadID] = {}
        oldPositionsTbl[roadID] = {}

        local oldPosTbl = oldPositionsTbl[roadID]
        local road = scenetree.findObjectById(roadID)
        for _, nodeID in ipairs(nodes) do
          oldPosTbl[nodeID] = road:getNodePosition(nodeID)
        end

        local newPosTbl = newPositionsTbl[roadID]
        for _, nodeID in ipairs(nodes) do
          newPosTbl[nodeID] = vec3(nodePosition[0], nodePosition[1], nodePosition[2])
        end
      end

      local nodeIDsTbl = deepcopy(selectedNodes)
      editor.history:commitAction("PositionRoadNode", {roadAndNodeIDs = nodeIDsTbl, oldPositions = oldPositionsTbl, newPositions = newPositionsTbl}, positionNodeActionUndo, positionNodeActionRedo)
      dragStartPosition = nil
      editingPos = false
    end
    -- Create the field for node width
    if not editingWidth then
      local width = nodeWidthsAllSame and nodeWidthVisible or 0
      nodeWidth[0] = width
    end
    if not nodeWidthsAllSame then
      im.PushStyleColor2(im.Col_Text, differentValuesColor)
    end
    widthSliderEditEnded[0] = false
    if editor.uiInputFloat("Node Width", nodeWidth, 0.1, 1.0, "%0." .. editor.getPreference("ui.general.floatDigitCount") .. "f", nil, widthSliderEditEnded) then
      editingWidth = true
    end
    if not nodeWidthsAllSame then
      im.PopStyleColor()
    end

    if widthSliderEditEnded[0] then
      local oldWidths = {}
      local newWidths = {}
      for roadID, nodes in pairs(selectedNodes) do
        oldWidths[roadID] = {}
        newWidths[roadID] = {}

        local oldWidthsTbl = oldWidths[roadID]
        local road = scenetree.findObjectById(roadID)
        for _, nodeID in ipairs(nodes) do
          oldWidthsTbl[nodeID] = road:getNodeWidth(nodeID)
        end

        local newWidthsTbl = newWidths[roadID]
        for _, nodeID in ipairs(nodes) do
          newWidthsTbl[nodeID] = nodeWidth[0]
        end
      end
      if not tableIsEmpty(oldWidths) then
        editor.history:commitAction("SetRoadNodesWidth", {oldWidths = oldWidths, newWidths = newWidths}, setNodesWidthActionUndo, setNodesWidthActionRedo)
      end
      editingWidth = false
    end

    if positionSliderEditEnded[0] or widthSliderEditEnded[0] then
      editor_roadUtils.updateChildRoads(selectedRoad)
      editor_roadUtils.reloadDecorations(selectedRoad)
      editor_roadUtils.reloadDecals(selectedRoad)
    end

    local roadAndNodeIDsTbl = {}
    for _, roadID in ipairs(selectedRoadsIds) do
      local selectedSingleNode = getSelectedSingleNodeInRoad(roadID)
      if selectedSingleNode == -1 then
        roadAndNodeIDsTbl = {}
        break
      else
        roadAndNodeIDsTbl[roadID] = selectedSingleNode
      end
    end

    if not tableIsEmpty(roadAndNodeIDsTbl) then
      local buttonTextSuffix = tableIsEmpty(roadAndNodeIDsTbl) and "" or "s"
      if im.Button("Split Road"..buttonTextSuffix, im.ImVec2(0,0)) then
        splitRoads(roadAndNodeIDsTbl)
      end
    end
    im.EndChild()
  end

  if shouldShowFuseOption then
    im.BeginChild1("RoadOps", im.ImVec2(0, 130), true)
    im.Text("Road Operations")
    if im.Button("Fuse Roads", im.ImVec2(0,0)) then
      fuseRoads()
    end
    im.EndChild()
  end
end

local function isNodeSelected(roadID, nodeID)
 return selectedNodes[roadID] and indexOf(selectedNodes[roadID], nodeID) ~= nil or false
end

local function showNodes(road)
  for index, node in ipairs(editor.getNodes(road)) do
    local pos = node.pos
    local tempNodeIndex = getRoadTempNodeIndex(road:getID())
    if editor.getPreference("roadEditor.general.dragWidth") and index - 1 == tempNodeIndex then
      if road:getNodeCount() == 1 then
        debugDrawer:drawSphere(pos, road:getNodeWidth(0)/2, roadRiverGui.highlightColors.nodeTransparent, false)
      end
      debugDrawer:drawTextAdvanced(pos, String("Road Width: " .. string.format("%.2f", road:getNodeWidth(tempNodeIndex)) .. ". Change width by dragging."), ColorF(1.0,1.0,1.0,1), true, false, ColorI(0, 0, 0, 128))
    end

    local sphereRadius = (core_camera.getPosition() - pos):length() * roadRiverGui.nodeSizeFactor
    if isNodeSelected(road:getID(), index - 1) and isRoadSelected(road:getID()) then
      debugDrawer:drawSphere(pos, sphereRadius, roadRiverGui.highlightColors.selectedNode, false)
    elseif hoveredNodeInfo[road:getID()] and hoveredNodeInfo[road:getID()] == (index - 1) then
      debugDrawer:drawSphere(pos, sphereRadius, roadRiverGui.highlightColors.hoveredNode, false)
    else
      debugDrawer:drawSphere(pos, sphereRadius, roadRiverGui.highlightColors.node, false)
    end
  end
end

local function showRoad(road, roadColor)
  local edgeCount = road:getEdgeCount()
  local duplicatedRoadIndex = indexOf(duplicatedRoadIDs, road:getID())

  -- Loop through the points and draw the lines
  for index = 0, edgeCount - 1 do
    local currentLeftEdge = road:getLeftEdgePosition(index)
    local currentMiddleEdge = road:getMiddleEdgePosition(index)
    local currentRightEdge = road:getRightEdgePosition(index)
    local borderLineWidth = editor.getPreference("gizmos.general.lineThicknessScale") * 4
    local midLineWidth = editor.getPreference("gizmos.general.lineThicknessScale") * 2
    debugDrawer:drawLineInstance(currentLeftEdge, currentMiddleEdge, midLineWidth, roadColor, false)
    debugDrawer:drawLineInstance(currentMiddleEdge, currentRightEdge, midLineWidth, roadColor, false)

    if index < edgeCount - 1 then
      debugDrawer:drawLineInstance(currentLeftEdge, road:getLeftEdgePosition(index+1), borderLineWidth, roadColor, false)
      debugDrawer:drawLineInstance(currentMiddleEdge, road:getMiddleEdgePosition(index+1), midLineWidth, roadColor, false)
      debugDrawer:drawLineInstance(currentRightEdge, road:getRightEdgePosition(index+1), borderLineWidth, roadColor, false)
    end

    if duplicatedRoadIndex and index < edgeCount - 1 then
      local duplicatedColor = color(roadRiverGui.highlightColors.duplicated.r * 255,
                                     roadRiverGui.highlightColors.duplicated.g * 255,
                                     roadRiverGui.highlightColors.duplicated.b * 255,
                                     roadRiverGui.highlightColors.duplicated.a * 255)
      debugDrawer:drawTriSolid(
        currentRightEdge,
        currentLeftEdge,
        road:getLeftEdgePosition(index+1),
        duplicatedColor, false)

      debugDrawer:drawTriSolid(
        road:getLeftEdgePosition(index+1),
        road:getRightEdgePosition(index+1),
        currentRightEdge,
        duplicatedColor, false)
    end
  end

  -- Only show nodes of selected road
  local roadID = road:getID()
  if isRoadSelected(roadID) then
    local selectedRoad = scenetree.findObjectById(roadID)
    if selectedRoad then
      showNodes(selectedRoad)
    end
  end
end

local function finishRoad()
  for _, roadID in ipairs(selectedRoadsIds) do
    local selectedRoad = scenetree.findObjectById(roadID)
    local roadTempNodeIndex = getRoadTempNodeIndex(roadID)
    if selectedRoad and roadTempNodeIndex ~= -1 then
      deleteNode(selectedRoad, roadTempNodeIndex)
      selectNode(roadID, nil)
      if selectedRoad:getNodeCount() <= 1 then
        editor.deleteRoad(roadID)
        editor.clearObjectSelection()
      end
    end
  end
  tempNodeIndexes = {}
  mouseButtonHeldOnNode = false
end

local function drawFrustumRect(frustum)
  local topLeftFrustum = vec3(frustum:getNearLeft() * 2, frustum:getNearDist() * 2, frustum:getNearTop() * 2)
  local topRightFrustum = vec3(frustum:getNearRight() * 2, frustum:getNearDist() * 2, frustum:getNearTop() * 2)
  local bottomLeftFrustum = vec3(frustum:getNearLeft() * 2, frustum:getNearDist() * 2, frustum:getNearBottom() * 2)
  local bottomRightFrustum = vec3(frustum:getNearRight() * 2, frustum:getNearDist() * 2, frustum:getNearBottom() * 2)

  local pos = core_camera.getPosition()
  local q = core_camera.getQuat()
  local topLeftWorld, bottomRightWorld = (q * topLeftFrustum) + pos, (q * bottomRightFrustum) + pos
  local topRightWorld, bottomLeftWorld = (q * topRightFrustum) + pos, (q * bottomLeftFrustum) + pos

  -- Draw the selection rectangle
  debugDrawer:drawLine(topLeftWorld, topRightWorld, ColorF(1, 0, 0, 1))
  debugDrawer:drawLine(topRightWorld, bottomRightWorld, ColorF(1, 0, 0, 1))
  debugDrawer:drawLine(bottomRightWorld, bottomLeftWorld, ColorF(1, 0, 0, 1))
  debugDrawer:drawLine(bottomLeftWorld, topLeftWorld, ColorF(1, 0, 0, 1))
end

local function onUpdate()
  local rayCastHit
  if core_forest.getForestObject() then core_forest.getForestObject():disableCollision() end
  local rayCast = cameraMouseRayCast()
  if core_forest.getForestObject() then core_forest.getForestObject():enableCollision() end

  if rayCast then rayCastHit = rayCast.pos end
  local mousePos = im.GetMousePos()

  local mouseMoved = true
  if mousePos.x == lastMousePos.x and mousePos.y == lastMousePos.y then
    mouseMoved = false
  end
  lastMousePos = mousePos

  hoveredNodeInfo = {}
  hoveredRoadID = nil
  local camPos = core_camera.getPosition()

  if tableIsEmpty(selectedRoadsIds) then
    templateDialogOpen[0] = false
    selectNode(nil)
  end

  local checkNonselectedRoads = true
  local roadIsHovered = false
  local isRectSelectKeyCombinationActive = editor.keyModifiers.shift and editor.keyModifiers.ctrl
  if not editor.keyModifiers.alt and not isRectSelectKeyCombinationActive and not mouseButtonHeldOnNode and
     not im.IsWindowHovered(im.HoveredFlags_AnyWindow) and not im.IsAnyItemHovered() then
    -- Check the selected road first
    if not tableIsEmpty(selectedRoadsIds) then
      -- Check if a node is hovered over
      local ray = getCameraMouseRay()
      local rayDir = ray.dir
      local minNodeDist = u_32_max_int

      for _, roadID in ipairs(selectedRoadsIds) do
        local road = scenetree.findObjectById(roadID)
        if not road then goto continue end
        for i, node in ipairs(editor.getNodes(road)) do
          local distNodeToCam = (node.pos - camPos):length()
          if distNodeToCam < minNodeDist then
            local nodeRayDistance = (node.pos - camPos):cross(rayDir):length() / rayDir:length()
            local sphereRadius = (camPos - node.pos):length() * roadRiverGui.nodeSizeFactor
            if nodeRayDistance <= sphereRadius then
              hoveredNodeInfo[roadID] = i - 1
              hoveredRoadID = roadID
              roadIsHovered = true
              checkNonselectedRoads = false
              minNodeDist = distNodeToCam
              break
            end
          end
        end
        ::continue::
      end
    end
  end

  if roadIsHovered then
    table.insert(hoveredRoadsIDs, hoveredRoadID)
  end

  -- Mouse Cursor Handling
  if rayCastHit then
    local focusPoint = rayCastHit
    local focusPointP3F = focusPoint
    local cursorColor = roadRiverGui.highlightColors.cursor
    if editor.keyModifiers.alt then
      -- Hovers somewhere else than the selected road
      if not tableIsEmpty(selectedRoadsIds) then
        local addNodeRefPoint = vec3(0, 0, 0)
        local numNodes = 0

        --Calculate mean pos of selected heading/trailing single nodes
        for _, roadId in ipairs(selectedRoadsIds) do
          local selRoad = scenetree.findObjectById(roadId)
          local selectedNode = getSelectedSingleNodeInRoad(roadId)
          if selectedNode == -1 then goto continue end
          local roadTempNodeIndex = getRoadTempNodeIndex(roadId)
          if selRoad and roadTempNodeIndex == -1 and selRoad:containsPoint(focusPointP3F) ~= selectedNode then
            if selectedNode == 0 and selRoad:getNodeCount() > 1 then
              addNodeRefPoint = addNodeRefPoint + selRoad:getNodePosition(selectedNode + 1)
              numNodes = numNodes + 1
            elseif selectedNode == selRoad:getNodeCount() - 1 then
              addNodeRefPoint = addNodeRefPoint + selRoad:getNodePosition(selectedNode)
              numNodes = numNodes + 1
            end
          end
          ::continue::
        end

        if numNodes > 0 then
          addTempNodeStartPos = vec3(addNodeRefPoint.x/numNodes, addNodeRefPoint.y/numNodes, addNodeRefPoint.z/numNodes)
        end

        for _, roadId in ipairs(selectedRoadsIds) do
          local selRoad = scenetree.findObjectById(roadId)
          local selectedNode = getSelectedSingleNodeInRoad(roadId)
          if selectedNode == -1 then goto continue end
          local roadTempNodeIndex = getRoadTempNodeIndex(roadId)
          if selRoad and roadTempNodeIndex == -1 and selRoad:containsPoint(focusPointP3F) ~= selectedNode then
            if selectedNode == 0 and selRoad:getNodeCount() > 1 then
              -- Add Node at the beginning
              local prevNodePos = selRoad:getNodePosition(selectedNode)
              local diff = focusPoint - addTempNodeStartPos
              tempNodeIndexes[roadId] = insertNode(selRoad, prevNodePos + diff, selRoad:getNodeWidth(selectedNode), 0)
            elseif selectedNode == selRoad:getNodeCount() - 1 then
              -- Add Node at the end
              local prevNodePos = selRoad:getNodePosition(selectedNode)
              local diff = focusPoint - addTempNodeStartPos
              tempNodeIndexes[roadId] = insertNode(selRoad, prevNodePos + diff, selRoad:getNodeWidth(selRoad:getNodeCount()-1), u_32_max_int)
            end
          end
          ::continue::
        end
      end
      if not addTempNodeStartPos then addTempNodeStartPos = focusPoint end
      cursorColor = roadRiverGui.highlightColors.createModeCursor
    end

    -- Debug cursor
    --[[if not im.IsMouseDown(1) then
      debugDrawer:drawSphere(focusPoint, 0.5, cursorColor)
    end]]

    -- Highlight hovered road
    local hoveredRoadsIDsCopy = hoveredRoadsIDs
    hoveredRoadsIDs = {}
    if not editor.keyModifiers.alt and not isRectSelectKeyCombinationActive and not mouseButtonHeldOnNode and not im.IsWindowHovered(im.HoveredFlags_AnyWindow) and not im.IsAnyItemHovered() then

      -- Check the selected roads first
      for _, roadID in ipairs(selectedRoadsIds) do
        local road = scenetree.findObjectById(roadID)
        if road and road:containsPoint(focusPointP3F) ~= -1 or hoveredNodeInfo[roadID] then
          table.insert(hoveredRoadsIDs, roadID)
          checkNonselectedRoads = false
        end
      end

      -- Then check the other roads
      if checkNonselectedRoads then
        local aiRoadsSelectable = editor.getPreference("roadEditor.general.aiRoadsSelectable")
        local nonAiRoadsSelectable = editor.getPreference("roadEditor.general.nonAiRoadsSelectable")
        for roadID, _ in pairs(editor.getAllRoads()) do
          if scenetree.decalRoadContainsPoint(roadID, focusPointP3F) then
            local road = scenetree.findObjectById(roadID)
            local drivability = road.drivability
            if ((drivability > 0 and aiRoadsSelectable) or (drivability <= 0 and nonAiRoadsSelectable)) and not road:isHidden() then
              table.insert(hoveredRoadsIDs, roadID)
            end
          end
        end
      end

      -- If the selected road is one of the hovered roads, always choose it
      local selectedRoadIndex = nil
      for _, roadID in ipairs(selectedRoadsIds) do
        local selectedHoveredRoadIndex = indexOf(hoveredRoadsIDs, roadID)
        if selectedHoveredRoadIndex then
          selectedRoadIndex = selectedHoveredRoadIndex
          break
        end
      end

      if selectedRoadIndex then
        hoveredRoadsIndex = selectedRoadIndex
      -- If the set of hovered roads has changed, use the last hovered road if possible, or else number 1
      elseif not setEqual(hoveredRoadsIDs, hoveredRoadsIDsCopy) then
        local oldIndex = indexOf(hoveredRoadsIDs, lastHoveredRoadID)
        if oldIndex then
          hoveredRoadsIndex = oldIndex
        else
          hoveredRoadsIndex = 1
        end
      end

      -- Set the hoveredRoad with the hoveredRoadsIndex
      hoveredRoadID = hoveredRoadsIDs[hoveredRoadsIndex]
      -- Color the hovered roads
      for _, roadID in ipairs(hoveredRoadsIDs) do
        if roadID == selectedRoadIndex then
          -- This gets colored later
          break
        elseif roadID == hoveredRoadID then
          local road = scenetree.findObjectById(roadID)
          if road then
            showRoad(road, Prefab.getPrefabByChild(road) and roadRiverGui.highlightColors.hoverSelectNotAllowed or roadRiverGui.highlightColors.hover)
          end
        else
          local road = scenetree.findObjectById(roadID)
          if road then
            showRoad(road, Prefab.getPrefabByChild(road) and roadRiverGui.highlightColors.lightHoverSelectNotAllowed or roadRiverGui.highlightColors.lightHover)
          end
        end
      end
    end

    if editor.keyModifiers.alt then
      local drawingOnSelectedRoad = false
      local tempNodeAvailable = false
      for _, roadID in ipairs(selectedRoadsIds) do
        local selectedRoad = scenetree.findObjectById(roadID)
        local roadTempNodeIndex = getRoadTempNodeIndex(roadID)
        if selectedRoad and selectedRoad:containsPoint(focusPointP3F) ~= -1 and roadTempNodeIndex == -1 then
          debugDrawer:drawSphere(focusPointP3F, (camPos - focusPoint):length() / 40, roadRiverGui.highlightColors.node, false)
          debugDrawer:drawTextAdvanced(focusPointP3F, "Insert node here.", ColorF(1.0,1.0,1.0,1), true, false, ColorI(0, 0, 0, 128))
          drawingOnSelectedRoad = true
        elseif roadTempNodeIndex ~= -1 then
          tempNodeAvailable = true
        end
      end

      if not drawingOnSelectedRoad and not tempNodeAvailable then
        debugDrawer:drawSphere(focusPointP3F, editor.getPreference("roadEditor.general.defaultWidth") / 2, roadRiverGui.highlightColors.nodeTransparent, false)
        debugDrawer:drawTextAdvanced(focusPointP3F, String("Road Width: " .. string.format("%.2f", editor.getPreference("roadEditor.general.defaultWidth")) .. (editor.getPreference("roadEditor.general.dragWidth") and ". Change width by dragging." or "")), ColorF(1.0,1.0,1.0,1), true, false, ColorI(0, 0, 0, 128))
      end
    end

    local activeRoadsIDs = selectedRoadsIds
    -- Mouse button has been released
    if mouseButtonHeldOnNode and im.IsMouseReleased(0) then
      if editor.keyModifiers.alt then
        local nodeIDsTbl = {}
        local roadInfosTbl = {}
        for _, activeRoadID in ipairs(activeRoadsIDs) do
          local tempNodeIndex = getRoadTempNodeIndex(activeRoadID)
          local activeRoad = scenetree.findObjectById(activeRoadID)
          if tempNodeIndex == -1 or not activeRoad then goto continue end
          -- Add new node to selectedRoad
          nodeIDsTbl[activeRoadID] = deepcopy({tempNodeIndex})
          tempNodeIndexes[activeRoadID] = nil
          local selectedNode = getSelectedSingleNodeInRoad(activeRoadID)
          if activeRoad:getNodeCount() > 2 then
            -- Undo action for placed node
            local nodeInfo = {pos = activeRoad:getNodePosition(tempNodeIndex), width = activeRoad:getNodeWidth(tempNodeIndex), index = tempNodeIndex}
            roadInfosTbl[activeRoadID] = nodeInfo
          elseif activeRoad:getNodeCount() == 2 then
            -- Undo whole road for 2 nodes
            local roadInfo = {nodes = editor.getNodes(activeRoad), roadInfo = editor.copyFields(activeRoadID), roadID = activeRoadID}
            editor.history:commitAction("CreateRoad", roadInfo, createRoadActionUndo, createRoadActionRedo, true)
            editor.selectObjectById(roadInfo.roadID)
          end
          editor.setPreference("roadEditor.general.defaultWidth", activeRoad:getNodeWidth(tempNodeIndex))
          ::continue::
        end
        if not tableIsEmpty(roadInfosTbl) then
          editor.history:commitAction("InsertRoadNode", {roadInfos = roadInfosTbl}, insertNodeActionUndo, insertNodeActionRedo, true)
        end
          selectNodes(nodeIDsTbl)
      else
        --If drag length is above threshold then register PositionRoadNode action
        if dragStartPosition then
          local cursorPosImVec = im.GetMousePos()
          local cursorPos = vec3(cursorPosImVec.x, cursorPosImVec.y, 0)
          local dragLength = (dragStartPosition - cursorPos):length()
          if dragLength > 5 then
            local oldPositionsTbl = deepcopy(dragNodesStartPositions)
            local newPositionsTbl = {}

            for roadID, nodes in pairs(selectedNodes) do
              if not newPositionsTbl[roadID] then
                newPositionsTbl[roadID] = {}
              end
              local posTbl = newPositionsTbl[roadID]
              local selectedRoad = scenetree.findObjectById(roadID)
              for _, nodeID in ipairs(nodes) do
                posTbl[nodeID] = selectedRoad:getNodePosition(nodeID)
              end
            end

            local nodeIDsTbl = deepcopy(selectedNodes)
            editor.history:commitAction("PositionRoadNode", {roadAndNodeIDs = nodeIDsTbl, oldPositions = oldPositionsTbl, newPositions = newPositionsTbl}, positionNodeActionUndo, positionNodeActionRedo)
          end
        end

        if roadTemplatesActive then
          for _, roadID in pairs(selectedRoadsIds) do
            local selectedRoad = scenetree.findObjectById(roadID)
            if selectedRoad then
              editor_roadUtils.updateChildRoads(selectedRoad)
              editor_roadUtils.reloadDecorations(selectedRoad)
              editor_roadUtils.reloadDecals(selectedRoad)
            end
          end
        end
      end

      mouseButtonHeldOnNode = false
      dragMouseStartPos = nil
      dragStartPosition = nil
      dragNodesStartPositions = {}
    end

    -- The mouse button is down
    if mouseButtonHeldOnNode and im.IsMouseDown(0) and mouseMoved then
      local cursorPosImVec = im.GetMousePos()
      local cursorPos = vec3(cursorPosImVec.x, cursorPosImVec.y, 0)
      -- Set the width of the node by dragging
      if editor.keyModifiers.alt then
        if editor.getPreference("roadEditor.general.dragWidth") then
          for _, roadID in ipairs(selectedRoadsIds) do
            local selectedRoad = scenetree.findObjectById(roadID)
            local roadTempNodeIndex = getRoadTempNodeIndex(roadID)
            if selectedRoad and roadTempNodeIndex ~= -1 then
              local width = math.max(oldNodeWidth + (cursorPos.x - dragMouseStartPos.x) / 10.0, 0)
              setNodeWidth(selectedRoad, roadTempNodeIndex, width)
            end
          end
        end
      -- Put the grabbed node(s) on the position of the cursor, dont move the node if it is close enough to the old position
      elseif not (dragMouseStartPos and (dragMouseStartPos - cursorPos):length() <= 5) then
        if not dragStartPosition then
          dragStartPosition = focusPoint
        end
        local diff = focusPoint - dragStartPosition
        for roadID, nodes in pairs(selectedNodes) do
          for _, nodeID in ipairs(nodes) do
            local nodePosTbl = dragNodesStartPositions[roadID]
            if nodePosTbl[nodeID] then
              local selectedRoad = scenetree.findObjectById(roadID)
              setNodePosition(selectedRoad, nodeID,  nodePosTbl[nodeID] + vec3(diff.x, diff.y, 0))
            end
          end
        end
      end
    end

    -- Create temporary node position to show where the next one will be
    if editor.keyModifiers.alt and not mouseButtonHeldOnNode and mouseMoved then
      for _, roadID in ipairs(selectedRoadsIds) do
        local selectedRoad = scenetree.findObjectById(roadID)
        local roadTempNodeIndex = getRoadTempNodeIndex(roadID)
        -- Calculate the pos of temp node by adding diff to the pos of the previous node
        if selectedRoad and roadTempNodeIndex  ~= -1 then
          local prevNodeIndex = (roadTempNodeIndex == 0) and 1 or roadTempNodeIndex-1
          local prevNodePos = selectedRoad:getNodePosition(prevNodeIndex)
          local diff = focusPoint - addTempNodeStartPos
          setNodePosition(selectedRoad, roadTempNodeIndex, prevNodePos + diff)
        end
      end
    end

    -- Mouse click on map
    if im.IsMouseClicked(0) and not (im.IsAnyItemHovered() or im.IsWindowHovered(im.HoveredFlags_AnyWindow)) then
      if editor.keyModifiers.alt then
        -- Clicked while in create mode
        local startNewRoad = true
        local selectedRoad = nil
        for _, roadID in ipairs(selectedRoadsIds) do
          selectedRoad = scenetree.findObjectById(roadID)
          if selectedRoad then
            local nodeIdx = selectedRoad:containsPoint(focusPoint)
            local roadTempNodeIndex = getRoadTempNodeIndex(roadID)
            -- Clicked into the selected road
            if nodeIdx ~= -1 and roadTempNodeIndex == -1 then
              -- Interpolate width of two adjacent nodes
              local w0 = selectedRoad:getNodeWidth(nodeIdx)
              local w1 = selectedRoad:getNodeWidth(nodeIdx + 1)
              local avgWidth = (w0 + w1) * 0.5
              insertNode(selectedRoad, focusPoint, avgWidth, nodeIdx + 1, true)
              selectNode(nodeIdx + 1)
              startNewRoad = false
            elseif roadTempNodeIndex  ~= -1 then
              -- Clicked outside of the selected road
              mouseButtonHeldOnNode = true
              startNewRoad = false
              oldNodeWidth = selectedRoad:getNodeWidth(roadTempNodeIndex)
            end
          end
        end

        if startNewRoad then
          -- Create new road
          local newRoadID = editor.createRoad({{pos = focusPoint, width = editor.getPreference("roadEditor.general.defaultWidth")}}, {overObjects = editor.getPreference("roadEditor.general.overObjects")})
          if fieldsCopy then
            editor.pasteFields(fieldsCopy, newRoadID)
          end
          editor.selectObjectById(newRoadID)
          selectedRoad = scenetree.findObjectById(newRoadID)
          -- If the mouse button is held down, change the width of the created node
          mouseButtonHeldOnNode = true
          tempNodeIndexes[newRoadID] = 0
          oldNodeWidth = selectedRoad:getNodeWidth(tempNodeIndexes[newRoadID])
        end
      end
    end

    if (not tableIsEmpty(tempNodeIndexes)) and (not editor.keyModifiers.alt) then
      finishRoad()
    end
  end

  if im.IsMouseClicked(0) and not (im.IsAnyItemHovered() or im.IsWindowHovered(im.HoveredFlags_AnyWindow) or editor_inspector.comboMenuOpen) then
    dragMouseStartPos = vec3(im.GetMousePos().x, im.GetMousePos().y, 0)
    if not editor.keyModifiers.alt and not isRectSelectKeyCombinationActive then
      local roadTempNodeIndex = getRoadTempNodeIndex(hoveredRoadID)
      -- Clicked on a hovered road
      if hoveredRoadID and roadTempNodeIndex == -1 then
        local roadObj = scenetree.findObjectById(hoveredRoadID)
        if tableIsEmpty(hoveredNodeInfo) then
          if(not Prefab.getPrefabByChild(roadObj)) then
            local selectMode = nil
            local ctrlDown = editor.keyModifiers.ctrl
            if ctrlDown then selectMode = editor.SelectMode_Toggle end
            -- Add road to selection
            editor.selectObjectById(hoveredRoadID, selectMode)
          else
            -- Get the top level prefab
            local hoveredObject = roadObj
            local prefab
            repeat
              prefab = Prefab.getPrefabByChild(hoveredObject)
              if prefab then
                hoveredObject = prefab
              end
            until not prefab

            if hoveredObject and editor.isObjectSelectable(hoveredObject) then
              local selectMode = nil
              local ctrlDown = editor.keyModifiers.ctrl
              if ctrlDown then selectMode = editor.SelectMode_Toggle end
              editor.selectObjectById(hoveredObject:getID(), selectMode)
            end
            editor.openModalWindow(roadNotSelectableErrorWindowName)
          end
        else
          -- Check if a node was clicked
          selectNode(hoveredRoadID, hoveredNodeInfo[hoveredRoadID])
          mouseButtonHeldOnNode = true
        end
      elseif not tableIsEmpty(selectedRoadsIds) and tableIsEmpty(hoveredNodeInfo) and not isRectSelectKeyCombinationActive then
        selectNode(nil)
        editor.clearObjectSelection()
      end
    end

    for roadID, nodes in pairs(selectedNodes) do
      local selectedRoad = scenetree.findObjectById(roadID)
      if not dragNodesStartPositions[roadID] then
        dragNodesStartPositions[roadID] = {}
      end
      local nodePosTbl = dragNodesStartPositions[roadID]
      for _, id in pairs(nodes) do
        nodePosTbl[id] = selectedRoad:getNodePosition(id)
      end
    end
  end

if isRectSelectKeyCombinationActive and im.IsMouseDragging(0) and not isRectSelecting then
  isRectSelecting = true
  rectSelectDragMouseStartPos = im.GetMousePos()
elseif isRectSelecting and (not isRectSelectKeyCombinationActive or im.IsMouseReleased(0)) then
  isRectSelecting = false
end

if isRectSelecting then
  local delta = im.GetMouseDragDelta(0)
  local topLeft2I = editor.screenToClient(Point2I(rectSelectDragMouseStartPos.x, rectSelectDragMouseStartPos.y))
  local topLeft = vec3(topLeft2I.x, topLeft2I.y, 0)
  local bottomRight = (topLeft + vec3(delta.x, delta.y, 0))
  local rect = {topLeft = topLeft, bottomRight = bottomRight}

  local viewportSizeIm = im.GetMainViewport().Size
  local viewportSize = vec3(viewportSizeIm.x, viewportSizeIm.y, 0)
  local viewFrustum = Engine.sceneGetCameraFrustum()
  local rectFrustum = Frustum(
                      false,
                      viewFrustum:getNearLeft() * (viewportSize.x/2 - rect.topLeft.x)/(viewportSize.x/2),
                      viewFrustum:getNearRight() * (rect.bottomRight.x - viewportSize.x/2)/(viewportSize.x/2),
                      viewFrustum:getNearTop() * (viewportSize.y/2 - rect.topLeft.y)/(viewportSize.y/2),
                      viewFrustum:getNearBottom() * (rect.bottomRight.y - viewportSize.y/2)/(viewportSize.y/2),
                      viewFrustum:getNearDist(),
                      viewFrustum:getFarDist(),
                      viewFrustum:getCameraCenterOffset(),
                      viewFrustum:getTransform())
  drawFrustumRect(rectFrustum)

  for _, roadID in ipairs(selectedRoadsIds) do
    local selectedRoad = scenetree.findObjectById(roadID)
    if selectedRoad then
      local nodesInRect = selectedRoad:getNodesFrustum(rectFrustum)
      selectedNodes[roadID] = {}
      for _, nodeID in ipairs(nodesInRect) do
        if isRoadSelected(roadID) then
          selectNode(roadID, nodeID, editor.SelectMode_Add)
        end
      end
    end
  end
end

  -- Highlight selected roads
  if not tableIsEmpty(selectedRoadsIds) then
    local numSelectedRoads = tableSize(selectedRoadsIds)
    for i = 1, numSelectedRoads do
      local selectedRoad = scenetree.findObjectById(selectedRoadsIds[i])
      if not selectedRoad then goto continue end
      showRoad(selectedRoad, roadRiverGui.highlightColors.selected)
      if selectedRoad.drivability > 0 then
        -- Draw an arrow representing the navgraph direction
        local edgeCount = selectedRoad:getEdgeCount()
        if edgeCount > 1 then
          local i1 = selectedRoad.flipDirection and edgeCount - 1 or 0
          local i2 = selectedRoad.flipDirection and edgeCount - 2 or 1
          local pos = selectedRoad:getMiddleEdgePosition(i1)
          local dir = (selectedRoad:getMiddleEdgePosition(i2) - pos):normalized()
          debugDrawer:drawSquarePrism(pos, pos + dir * 1.5, Point2F(0.5, 0.75), Point2F(0.5, 0), roadRiverGui.highlightColors.selectedNode)
        end
      end
      ::continue::
      shouldShowFuseOption = (numSelectedRoads == 2)
    end
  end
  lastHoveredRoadID = hoveredRoadID
end


local function onExtensionLoaded()
  log('D', logTag, "initialized")
end


local function onPreRender()
  if not editor.editMode or (editor.editMode.displayName ~= editModeName) then
    return
  end
end


local function onActivate()
  log('I', logTag, "onActivate")
  roadTemplatesActive = editor.getPreference("roadTemplates.general.loadTemplates")
  editor.initializeLevelRoadsVertices()
  M.onEditorObjectSelectionChanged()
end

local function onDeactivate()
  finishRoad()
end

-- These methods are for the action map to call
local function copySettingsAM()
  local selectedRoad
  if #selectedRoadsIds == 1 then
    selectedRoad = scenetree.findObjectById(selectedRoadsIds[1])
  end

  if selectedRoad then
    fieldsCopy = editor.copyFields(selectedRoadsIds[1])
    local nodeWidthsTotal = 0
    for _, nodeInfo in ipairs(editor.getNodes(selectedRoad)) do
      nodeWidthsTotal = nodeWidthsTotal + nodeInfo.width
    end
    local averageNodeWidth = nodeWidthsTotal / tableSize(editor.getNodes(selectedRoad))
    editor.setPreference("roadEditor.general.defaultWidth", averageNodeWidth)
  end
end

local function onDeleteSelection()
  local roadInfos = {}
  local nodeInfos = {}
  local nodes = {}
  for _, roadID in ipairs(selectedRoadsIds) do
    local selectedRoad = scenetree.findObjectById(roadID)
    if selectedRoad and not isAnyNodeSelected(roadID) then
      roadInfos[roadID] = editor.copyFields(roadID)
      nodes[roadID] = editor.getNodes(selectedRoad)
    end
  end

  for roadID, nodeIDs in pairs(selectedNodes) do
    local selectedRoad = scenetree.findObjectById(roadID)
    if selectedRoad and selectedNodes[roadID] then
      nodeInfos[roadID] = {}
      nodes[roadID] = {}
      local numSelectedNodesInRoad = tableSize(selectedNodes[roadID])
      -- If there are 2 or more unselected nodes then we just delete the selected nodes
      if (selectedRoad:getNodeCount() - numSelectedNodesInRoad) >= 2 then
        for _, nodeID in ipairs(nodeIDs) do
            local selNodePos = selectedRoad:getNodePosition(nodeID)
            local selNodeWidth = selectedRoad:getNodeWidth(nodeID)
            local nodeInfo = {pos = selNodePos, width = selNodeWidth, index = nodeID}
            table.insert(nodeInfos[roadID], nodeInfo)
        end
      -- If there are less than 2 unselected nodes then we delete the road
      else
        roadInfos[roadID] = editor.copyFields(roadID)
        nodes[roadID] = editor.getNodes(selectedRoad)
      end
    end
  end
  editor.history:commitAction("DeleteSelection", {roadInfos = roadInfos, nodeInfos = nodeInfos, nodes = nodes}, deleteSelectionActionUndo, deleteSelectionActionRedo)
  selectNode(nil)
end

local function cycleHoveredRoadsAM(value)
  local numberOfHoveredRoads = table.getn(hoveredRoadsIDs)
  if numberOfHoveredRoads == 0 then return end
  if value == 1 then
    hoveredRoadsIndex = ((hoveredRoadsIndex % numberOfHoveredRoads) + 1)
  elseif value == 0 then
    hoveredRoadsIndex = (((hoveredRoadsIndex - 2) % numberOfHoveredRoads) + 1)
  end
end

local function defaultWidthSlider()
  local defaultWidthPtr = im.FloatPtr(editor.getPreference("roadEditor.general.defaultWidth"))
  if im.InputFloat("##Default Width", defaultWidthPtr, 0.1, 0.5, "%0." .. editor.getPreference("ui.general.floatDigitCount") .. "f") then
    editor.setPreference("roadEditor.general.defaultWidth", defaultWidthPtr[0])
  end
end

local function onToolbar()
  im.Text("Default Width")
  im.SameLine()
  im.PushItemWidth(im.uiscale[0] * 130)
  defaultWidthSlider()

  im.SameLine()
  local aiRoadsPtr = im.BoolPtr(editor.getPreference("roadEditor.general.aiRoadsSelectable"))
  if im.Checkbox("AI roads selectable", aiRoadsPtr) then
    editor.setPreference("roadEditor.general.aiRoadsSelectable", aiRoadsPtr[0])
  end
  im.tooltip("Make roads that are used by AI hoverable and clickable")

  im.SameLine()
  local nonAiRoadsPtr = im.BoolPtr(editor.getPreference("roadEditor.general.nonAiRoadsSelectable"))
  if im.Checkbox("non-AI roads selectable", nonAiRoadsPtr) then
    editor.setPreference("roadEditor.general.nonAiRoadsSelectable", nonAiRoadsPtr[0])
  end
  im.tooltip("Make roads that are not used by AI hoverable and clickable")

  im.SameLine()
  local overObjectsPtr = im.BoolPtr(editor.getPreference("roadEditor.general.overObjects"))
  if im.Checkbox("Over Objects", overObjectsPtr) then
    editor.setPreference("roadEditor.general.overObjects", overObjectsPtr[0])
  end
  im.tooltip("Make roads that go over static objects too")

  if editor.beginModalWindow(roadNotSelectableErrorWindowName, roadNotSelectableErrorWindowTitle, im.WindowFlags_AlwaysAutoResize + im.WindowFlags_NoScrollbar) then
    im.Text("Cannot select Road!")
    im.TextColored(im.ImVec4(1, 1, 0, 1), "Select and edit not allowed when road is inside packed prefab!")
    if im.Button("OK") then
      editor.closeModalWindow(roadNotSelectableErrorWindowName)
    end
  end
  editor.endModalWindow()
end

local function onEditorRegisterPreferences(prefsRegistry)
  prefsRegistry:registerCategory("roadEditor")
  prefsRegistry:registerSubCategory("roadEditor", "general", nil,
  {
    -- {name = {type, default value, desc, label (nil for auto Sentence Case), min, max, hidden, advanced, customUiFunc, enumLabels}}
    {defaultWidth = {"float", 10}},
    {dragWidth = {"bool", false, "Change the width of newly placed nodes by clicking and dragging the mouse cursor."}},
    {aiRoadsSelectable = {"bool", true, "Controls whether ai roads should be selectable in the decal road editor"}},
    {nonAiRoadsSelectable = {"bool", true, "Controls whether non-ai roads should be selectable in the decal road editor"}},
    {overObjects = {"bool", false, "Controls whether roads go over static objects too"}},
    -- hidden
    {columnSizes = {"table", {29, 53, 300, 145, 97, 280}, "", nil, nil, nil, true}}
  })
end

local function onDuplicate()
  if not editor.isViewportFocused() then return end
  local duplicatedRoadIds = {}
  for _, roadID in ipairs(selectedRoadsIds) do
    local selectedRoad = scenetree.findObjectById(roadID)
    if selectedRoad then
      table.insert(duplicatedRoadIds, roadID);
    end
  end

  local arrayRoadInfos = {}
  local arrayNodes = {}
  for _, roadID in ipairs(duplicatedRoadIds) do
    local selectedRoad = scenetree.findObjectById(roadID)
    if selectedRoad then
      table.insert(arrayNodes, editor.getNodes(selectedRoad))
      table.insert(arrayRoadInfos, editor.copyFields(roadID))
    end
  end

  if not tableIsEmpty(duplicatedRoadIds) then
    editor.history:commitAction("DuplicateRoad", {nodes = arrayNodes, roadInfos = arrayRoadInfos}, duplicateRoadActionUndo, duplicateRoadActionRedo)
  end
end

local function onEditorObjectSelectionChanged()
  if not editor.editMode or (editor.editMode.displayName ~= editModeName) then
    return
  end
  if (not tableIsEmpty(tempNodeIndexes)) and tableIsEmpty(selectedRoadsIds) then
    finishRoad()
  end
  table.clear(selectedRoadsIds)
  for i = 1, tableSize(editor.selection.object) do
    local selectedObject = scenetree.findObjectById(editor.selection.object[i])
    if selectedObject and selectedObject:getClassName() == "DecalRoad" then
      table.insert(selectedRoadsIds, editor.selection.object[i])
    end
  end

  local tempSelectedNodes = {}
  for _, roadID in ipairs(selectedRoadsIds) do
    if selectedNodes[roadID] and not tableIsEmpty(selectedNodes[roadID]) then
      tempSelectedNodes[roadID] = deepcopy(selectedNodes[roadID])
    end
  end
  selectedNodes = tempSelectedNodes
end

local function customDecalRoadMaterialsFilter(materialSet)
  local retSet = {}
  for i = 0, materialSet:size() - 1 do
    local material = materialSet:at(i)
    for tagId = 0, 2 do
      local tag = material:getField("materialTag", tostring(tagId))
      if string.lower(tag) == string.lower(roadMaterialTagString) then
        table.insert(retSet, material)
      end
    end
  end
  return retSet
end

local function onEditorInitialized()
  editor.editModes.roadEditMode =
  {
    displayName = editModeName,
    onActivate = onActivate,
    onDeactivate = onDeactivate,
    onDeleteSelection = onDeleteSelection,
    onUpdate = onUpdate,
    onToolbar = onToolbar,
    actionMap = actionMapName,
    onCopy = copySettingsAM,
    onPaste = pasteFieldsAM,
    onDuplicate = onDuplicate,
    onSelectAll = onSelectAll,
    icon = editor.icons.create_road_decal,
    iconTooltip = "Decal Road Editor",
    auxShortcuts = {},
    hideObjectIcons = true
  }

  editor.editModes.roadEditMode.auxShortcuts[bit.bor(editor.AuxControl_LMB, editor.AuxControl_Alt)] = "Create road / Add node"
  editor.editModes.roadEditMode.auxShortcuts[bit.bor(editor.AuxControl_Ctrl, editor.AuxControl_LMB)] = "Select Multiple Roads"
  editor.editModes.roadEditMode.auxShortcuts[bit.bor(editor.AuxControl_Ctrl, editor.AuxControl_Shift, editor.AuxControl_LMB_Drag)] = "Rectangle Select Nodes"
  editor.editModes.roadEditMode.auxShortcuts[editor.AuxControl_Copy] = "Copy road properties"
  editor.editModes.roadEditMode.auxShortcuts[editor.AuxControl_Paste] = "Paste road properties"
  editor.editModes.roadEditMode.auxShortcuts[editor.AuxControl_Duplicate] = "Duplicate road"

  editor.registerCustomFieldInspectorFilter("DecalRoad", "Material", customDecalRoadMaterialsFilter)
  editor.registerModalWindow(roadNotSelectableErrorWindowName, im.ImVec2(600, 400))

  editor_roadUtils.reloadTemplates()
end

M.onPreRender = onPreRender
M.onEditorInitialized = onEditorInitialized
M.onExtensionLoaded = onExtensionLoaded
M.onEditorInspectorHeaderGui = onEditorInspectorHeaderGui
M.onEditorRegisterPreferences = onEditorRegisterPreferences
M.onEditorObjectSelectionChanged = onEditorObjectSelectionChanged

M.cycleHoveredRoadsAM = cycleHoveredRoadsAM

return M