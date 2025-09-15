-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = 'editor_scene_tree'
local imgui = ui_imgui
local sceneTreeWindowNamePrefix = "scenetree"
local nameFilterText = ""
local editEnded = imgui.BoolPtr(false)
local comboIndex = imgui.IntPtr(0)
local inputTextValue = imgui.ArrayChar(500)
local iconSize = imgui.ImVec2(20, 20)
local nodeIconColor = imgui.ImColorByRGB(255,255,0,255)
local nodeTextColor = imgui.ImColorByRGB(255,255,255,255)
local selectedNodeIconColor = imgui.ImColorByRGB(0,255,255,255)
local selectedNodeTextColor = imgui.ImColorByRGB(0,255,255,255)
local selectedObjectNodeIconColor = imgui.ImColorByRGB(0,255,255,255)
local objectNodeIconColor = imgui.ImColorByRGB(180, 120, 0, 255)
local transparentColor = imgui.ImVec4(0,0,0,0)
local defaultObjectNodeIcon = nil -- inited in onEditorInitialized
local objectClassIcons = nil -- inited in onEditorInitialized
local dragDropBGColor = imgui.GetColorU322(imgui.ImVec4(1, 1, 1, 0.25), 1)
local guiInstancer = require("editor/api/guiInstancer")()
local objectHistoryActions = require("editor/api/objectHistoryActions")()
local socket = require("socket")
local editingNodeName = nil
local deleteNodes = false
local selectedNodePathNodes = {}

local SelectMode_Range = 0

local MaxGroupNestingLevel = 30

local hasDragDropPayload = false
local clickedOnNode

local nodeIdToOpen = nil
local onClickSelected = false
local nodeWasDblClicked

local mouseDragRange
local dragSelectionList = {}

-- vars for virtual scrolling
local entrySize

local searchTypesComboItems

local SearchMode_Name = 0
local SearchMode_ID = 1
local SearchMode_PersistentID = 2
local SearchMode_Class = 3
local SearchMode_All = 4
local searchNodeMode = SearchMode_All
local searchMatches = { bit.lshift(1, 0), bit.lshift(1, 1), bit.lshift(1, 2), bit.lshift(1, 3), bit.lshift(1, 4)} -- displayname, name, id, persistentId, class

local searchRange = -1
local searchRangeTimer = hptimer()
local searchRangeTime = -1

local cameraPositionCache
local showGroups = true -- shows/hides groups. Used for search results
local searchResults = {}
local searchResultsMode = false
local prefabSaveFolder = "/"
local currentSceneTreeInstanceIndex = nil

-- Registered extended scene tree object menu items
local extendedSceneTreeObjectMenuItems = {}


local function getRootGroup()
  if editor.getPreference("ui.general.showCompleteSceneTree") then return Sim.getRootGroup() end
  return scenetree.MissionGroup
end

local function getNodeName(object)
  if not object or not object["getName"] or not object["getClassName"] then return "<unsupported>" end
  if object:getName() == "" then
    return object:getClassName()
  else
    return object:getName()
  end
end

local function getNameOrInternalName(object)
  if not object or not object["getName"] or not object["getClassName"] then return "<unsupported>" end
  if (object:getName() == "" or object:getName() == nil) and object["getInternalName"] and object:getInternalName() ~= "" and object:getInternalName() ~= nil then
    return object:getInternalName()
  else
    return getNodeName(object)
  end
end

local function getNodeDisplayName(object)
  if not object or not object["getName"] or not object["getClassName"] then return "<unsupported>" end
  local displayName
  local className = object:getClassName()
  if className == 'TSStatic' then
    -- TSStatics will trail with the shapeName
    local shapeName = object.shapeName
    if shapeName then
      local _, shapeNameRes, _ = path.split(shapeName)
      displayName = shapeNameRes
    end
  elseif className == 'DecalRoad' then
    -- DecalRoads will trail with the material name
    displayName = object.material
  elseif editor.getPreference("ui.general.showInternalName") then
    return getNameOrInternalName(object)
  else
    -- otherwise, use the actual name, internal name or classname
    return getNodeName(object)
  end

  -- if we have a trailing string, the prefix will be the name
  -- we do custom checking for names and internal names here, so we only use Name or InternalName as a prefix, not the ClassName.
  if object:getName() ~= "" and object:getName() ~= nil then
    return object:getName() .. " (" .. displayName .. ")"
  elseif editor.getPreference("ui.general.showInternalName") and object["getInternalName"] and object:getInternalName() ~= "" and object:getInternalName() ~= nil then
    return object:getInternalName() .. " (" .. displayName .. ")"
  end
  return displayName
end

local function getObjectNodeIcon(className)
  local iconName = objectClassIcons[className]

  if not iconName then
    return defaultObjectNodeIcon
  else
    return editor.icons[iconName]
  end
end

local function getGroupNodeIcon(node)
  local object = scenetree.findObjectById(node.id)
  if object and object:getField("unpacked_prefab", "") == "1" then
    return editor.icons[objectClassIcons["unpacked_prefab"]]
  end
  if node.open or node.openOnSearch then
    return editor.icons.folder_open
  end
  return editor.icons.folder
end

local function getSceneTreeSelectedGroup(instance)
  if instance and #instance.selectedNodes ~= 0 then
    if #instance.selectedNodes == 1 then
      if instance.selectedNodes[1].isGroup then return scenetree.findObjectById(instance.selectedNodes[1].id) end
    end
    if instance.selectedNodes[1].parent then return scenetree.findObjectById(instance.selectedNodes[1].parent.id) end
  end
  return scenetree.MissionGroup
end

local function getNodeSize(instance, node)
  local size = 0
  if node then
    if not instance.rootNodeSizeCache then
      node.listIndex = nil
    end
    if not node.hidden then
      size = 1
      if not instance.rootNodeSizeCache then
        node.listIndex = instance.listIndex
        instance.listIndex = instance.listIndex + 1
      end
      if node.isGroup and (not showGroups or node.open or node.openOnSearch) and node.children then
        for orderIndex, childId in ipairs(node.renderChildrenOrder) do
          local child = node.children[childId]
          if child then
            size = size + getNodeSize(instance, child)
          end
        end
      end
    end
  end
  return size
end

local function findNodeById(instance, parentNode, objId)
  if not objId then
    return
  end
  if not parentNode then
    parentNode = instance.rootNode
  end

  if parentNode.id == objId then
    return parentNode
  end

  if parentNode.children then
    local node = parentNode.children[objId]
    if node then
      return node
    end
    for _, node in pairs(parentNode.children) do
      local child = findNodeById(instance, node, objId)
      if child then
        return child
      end
    end
  end
end

local function findNodeByObject(instance, parentNode, object)
  if not object then
    return
  end
  local objId = object:getID()
  return findNodeById(instance, parentNode, objId)
end

local function getRootNodeSize(instance)
  if not instance.rootNodeSizeCache then
    instance.listIndex = 1
    instance.rootNodeSizeCache = getNodeSize(instance, instance.rootNode)
  end
  return instance.rootNodeSizeCache
end

local function getIsExpandable(className)
  local expandableClasses = {"SimGroup", "SimSet"}
  for i,v in ipairs(expandableClasses) do
    if v == className then
      return true
    end
  end
  return false
end

local function cacheGroupNodeInternal(instance, node, groupsToChildren, nestingLevel)
  if not node then node = instance.rootNode end
  if not node then return end
  if not node.size then node.size = 0 end
  if nestingLevel > MaxGroupNestingLevel then
    editor.logError("Scene tree depth too high, probably cyclic group reference")
  end

  if (node.isGroup or node.isExpandable) then
    node.children = node.children or {}
    node.renderChildrenOrder = node.renderChildrenOrder or {}
  end

  local childrenIds = groupsToChildren[node.id]

  if childrenIds then
    for _, objId in ipairs(childrenIds) do
      local object = scenetree.findObjectById(objId)
      if object then
        local child = node.children[objId]
        if not child then
          local className = object:getClassName() or ""
          local isGroup = object:isSubClassOf("SimSet") or object:isSubClassOf("SimGroup")
          local isExpandable = getIsExpandable(className)
          child = {
            id = objId,
            order = order,--TODO we need to save order in undo, so we know where to place the node
            name = getNodeName(object),
            displayName = getNodeDisplayName(object),
            className = className,
            icon = getObjectNodeIcon(className, object),
            open = false,
            selected = false,
            isGroup = isGroup,
            isExpandable = isExpandable,
            parent = node,
            renderOrderIndex = #node.renderChildrenOrder + 1
          }
          node.children[objId] = child
          -- table.insert(node.renderChildrenOrder, objId)
          local childIndex = arrayFindValueIndex(node.renderChildrenOrder, objId)
          if not childIndex then
            table.insert(node.renderChildrenOrder, objId)
          -- else
          --   log('E','', node.id..': Duplicated child index detected child = '..objId..' (site 2)')
          end
        end

        -- also cache this node if its a group
        -- this happens if the newly added group missed the addition of its children objects (in the case of prefab packing for example)
        if child.isGroup or child.isExpandable then
          if child.isExpandable and not child.isGroup then
            local nodeChildSceneData = object:getScenetreeData()
            local dataCount = #nodeChildSceneData
            if dataCount > 0 then
              for index = 1, dataCount, 2 do
                local objId = nodeChildSceneData[index]
                local objGroupId = nodeChildSceneData[index + 1]
                if not groupsToChildren[objGroupId] then
                  groupsToChildren[objGroupId] = {}
                end
                table.insert(groupsToChildren[objGroupId], objId)
              end
            end
          end
          cacheGroupNodeInternal(instance, child, groupsToChildren, nestingLevel + 1)
        end
      end
    end
  else
    if node.children then
      for childId,child in pairs(node.children) do
        cacheGroupNodeInternal(instance, child, groupsToChildren, nestingLevel + 1)
      end
    end
  end

  if node.children == nil then
    node.size = 1
  end
  if node.parent then
    node.parent.size = node.parent.size + node.size
  end
end

local function cacheGroupNode(instance, node, addObjectIds, nestingLevel)
  local groupsToChildren = {}
  local addedObjectCount = addObjectIds and #addObjectIds or 0
  if addedObjectCount > 0 then
    for index = 1, addedObjectCount, 2 do
      local objId = addObjectIds[index]
      local objGroupId = addObjectIds[index + 1]
      if not groupsToChildren[objGroupId] then
        groupsToChildren[objGroupId] = {}
      end
      table.insert(groupsToChildren[objGroupId], objId)
    end
  end

  cacheGroupNodeInternal(instance, node, groupsToChildren, nestingLevel)
end

local function removeNodeByObjectId(node, objId)
  if node.id == objId then
    if node.parent then
      local renderOrderIndex = node.parent.children[objId].renderOrderIndex
      table.remove(node.parent.renderChildrenOrder or {}, renderOrderIndex)
      node.parent.children[objId] = nil
      return
    end
  end

  if node.children then
    for _, child in pairs(node.children) do
      removeNodeByObjectId(child, objId)
    end
  end
end

local function removeNodesByObjectIds(instance, objectIds)
  if not instance.rootNode then return end
  if not objectIds or tableIsEmpty(objectIds) then return end
  local count = #objectIds
  for i = 1, count, 2 do
    local id = objectIds[i]
    local groupId = objectIds[i + 1]
    removeNodeByObjectId(instance.rootNode, id)
  end
end

local function deleteSelectedNodes(instance)
  if not instance.selectedNodes or tableIsEmpty(instance.selectedNodes) then return end
  for _, node in ipairs(instance.selectedNodes) do
    if node.parent then
      node.parent.children[node.id] = nil
      node = nil
    end
  end
  instance.selectedNodes = {}
end

local function applyFilterRecursive(instance, node)
  if not node then node = rootNode end
  if not node then return end

  node.hidden = false
  node.openOnSearch = nil
  node.filterResult = 0
  local passed = false

  if nameFilterText ~= "" then
    -- searching
    if searchNodeMode == SearchMode_Name then
      if imgui.ImGuiTextFilter_PassFilter(instance.nameFilter, node.displayName) then
        node.filterResult = searchMatches[1]
      end
      if imgui.ImGuiTextFilter_PassFilter(instance.nameFilter, node.name) then
        node.filterResult = node.filterResult + searchMatches[2]
      end
      passed = node.filterResult ~= 0
    elseif searchNodeMode == SearchMode_ID then
      if imgui.ImGuiTextFilter_PassFilter(instance.nameFilter, tostring(node.id)) then
        node.filterResult = searchMatches[3]
      end
      passed = node.filterResult ~= 0
    elseif searchNodeMode == SearchMode_PersistentID then
      if imgui.ImGuiTextFilter_PassFilter(instance.nameFilter, tostring(node.persistentId)) then
        node.filterResult = searchMatches[4]
      end
      passed = node.filterResult ~= 0
    elseif searchNodeMode == SearchMode_Class then
      if imgui.ImGuiTextFilter_PassFilter(instance.nameFilter, node.className) then
        node.filterResult = searchMatches[5]
      end
      passed = node.filterResult ~= 0
    elseif searchNodeMode == SearchMode_All then
      if imgui.ImGuiTextFilter_PassFilter(instance.nameFilter, node.displayName) then
        node.filterResult = searchMatches[1]
      end
      if imgui.ImGuiTextFilter_PassFilter(instance.nameFilter, node.name) then
        node.filterResult = node.filterResult + searchMatches[2]
      end
      if imgui.ImGuiTextFilter_PassFilter(instance.nameFilter, tostring(node.id)) then
        node.filterResult = node.filterResult + searchMatches[3]
      end
      if imgui.ImGuiTextFilter_PassFilter(instance.nameFilter, tostring(node.persistentId)) then
        node.filterResult = node.filterResult + searchMatches[4]
      end
      if imgui.ImGuiTextFilter_PassFilter(instance.nameFilter, node.className) then
        node.filterResult = node.filterResult + searchMatches[5]
      end
      passed = node.filterResult ~= 0
    end
  else
    -- not searching
    passed = true
  end

  if passed and searchRange > 0 then
    local object = scenetree.findObjectById(node.id)
    node.cameraDistance = math.huge
    if object and type(object.getPosition) == 'function' then
      local nodePos = object:getPosition()
      node.cameraDistance = math.abs((cameraPositionCache - nodePos):length())
      passed = node.cameraDistance < searchRange
    else
      passed = false
    end
  else
    node.cameraDistance = nil
  end

  if passed then
    table.insert(searchResults, node)
  end

  if passed and nameFilterText ~= "" then
    if node.isGroup then
      node.hidden = false
      if node.parent then node.parent.openOnSearch = true end
    elseif node.parent then
      node.hidden = false
      node.parent.hidden = false
      node.parent.openOnSearch = true
    end
  end
  if not passed then
    node.hidden = true
  end

  if node.children and tableSize(node.children) then
    for _, child in pairs(node.children) do
      applyFilterRecursive(instance, child)
    end
  end

  if node.openOnSearch then
    if node.parent then
      node.parent.hidden = false
      node.parent.openOnSearch = true
    end
  end
end

local function applyFilter(instance, node)
  searchResults = {}
  cameraPositionCache = core_camera.getPosition()
  nameFilterText = ffi.string(imgui.TextFilter_GetInputBuf(instance.nameFilter))
  applyFilterRecursive(instance, node)
  instance.rootNodeSizeCache = nil

  -- now sort the results
  if searchRange > 0 then
    table.sort(searchResults, function(n1, n2)
      return n1.cameraDistance < n2.cameraDistance
    end)
  end

  searchResultsMode = searchRange > 0 or nameFilterText ~= ''
end

local function refreshNodeCache(instance)
  applyFilter(instance, instance.rootNode)
end

local function updateBreadcrumbPath(node)
  local function recursiveWalkParents(parent)
    if parent.name == "MissionGroup" or parent.name == "RootGroup" then return end
    table.insert(selectedNodePathNodes, parent)
    if parent.parent then
      recursiveWalkParents(parent.parent)
    end
  end
  selectedNodePathNodes = {}
  recursiveWalkParents(node)
end

local function selectNode(instance, node, selectMode)
  if not node then return end
  if not selectMode then selectMode = editor.SelectMode_New end
  if selectMode == SelectMode_Range then
    if #instance.selectedNodes > 0 then
      instance.selectionRange = {}
      for i = math.min(instance.lastSelectedIndex, node.listIndex), math.max(instance.lastSelectedIndex, node.listIndex) do
        instance.selectionRange[i] = true
      end
    else
      instance.lastSelectedIndex = node.listIndex
      selectMode = editor.SelectMode_New
    end
    instance.currentListIndex = node.listIndex
  elseif selectMode == editor.SelectMode_New then
    instance.lastSelectedIndex = node.listIndex
    instance.currentListIndex = node.listIndex
  end

  editor.selectObjectById(node.id, selectMode)
  editor.updateObjectSelectionAxisGizmo()

  updateBreadcrumbPath(node)

  if node.name ~= "MissionGroup" then
    local current = node
    while current.parent and current.parent.name ~= "MissionGroup" do
      current.parent.open = true
      current = current.parent
    end
  end

  --TODO: jump to new selection in the other unlocked scene tree window instances
end

local function createSimGroupActionUndo(actionData)
  local sceneTreeInstance = guiInstancer.instances[currentSceneTreeInstanceIndex]
  removeNodeByObjectId(sceneTreeInstance.rootNode, actionData.objectID)
  sceneTreeInstance.selectedNodes = {}
  local obj = scenetree.findObjectById(actionData.objectID)
  if obj then obj:deleteObject() end
end

local function createSimGroupActionRedo(actionData)
  local sceneTreeInstance = guiInstancer.instances[currentSceneTreeInstanceIndex]
  if actionData.objectID then
    SimObject.setForcedId(actionData.objectID)
  end
  local grp = worldEditorCppApi.createObject("SimGroup")
  grp:registerObject("")

  if actionData.addToRoot or not actionData.groupParentID then
    scenetree.MissionGroup:addObject(grp)
  else
    local parentObject = scenetree.findObjectById(actionData.groupParentID)
    if parentObject then
      parentObject:addObject(grp)
    end
  end

  local grpNode = findNodeByObject(sceneTreeInstance, nil, grp)
  selectNode(sceneTreeInstance, grpNode)
  sceneTreeInstance.scrollToNode = true
  actionData.objectID = grpNode.id
  actionData.grp = grp
end

local function getChangeOrderActionData(instance, newGroup)
  local objects = {}
  local placeholderObjects = {}
  local oldGroups = {}
  for _, node in ipairs(instance.selectedNodes) do
    local object = scenetree.findObjectById(node.id)
    if object then
      table.insert(objects, node.id)
      local nextObject = object:getGroup():getObject(object:getGroup():getObjectIndex(object) + 1)
      if nextObject then
        table.insert(placeholderObjects, nextObject:getID())
      end
      table.insert(oldGroups, node.parent.id)
    end
  end
  return {objects = objects, placeholderObjects = placeholderObjects, oldGroups = oldGroups, newGroup = newGroup}
end

local function getNodeOpenStatus(node, res)
  if not res then res = {} end
  res[node.id] = node.open
  if node.children then
    for _, child in pairs(node.children) do
      getNodeOpenStatus(child, res)
    end
  end
  return res
end

local function applyNodeOpenStatus(node, openStatus)
  node.open = openStatus[node.id]
  if node.children then
    for _, child in pairs(node.children) do
      applyNodeOpenStatus(child, openStatus)
    end
  end
end

local function recacheAllNodes(keepOpenStatus)
  local rootGrp = getRootGroup()
  local incomingObjectIds = {}
  if rootGrp then
    incomingObjectIds = rootGrp:getScenetreeData()
  end

  for index, instance in pairs(guiInstancer.instances) do
    local openStatus
    if keepOpenStatus and instance.rootNode then
      openStatus = getNodeOpenStatus(instance.rootNode)
    end
    instance.rootNode = nil
    if rootGrp then
      instance.rootNode = {
        id = rootGrp:getID(),
        object = rootGrp,
        className = rootGrp:getClassName(),
        upcastedObject = Sim.upcast(rootGrp),
        open = true,
        selected = false,
        isGroup = true,
        isExpandable = true,
        parent = nil,
        name = getNodeName(rootGrp),
        displayName = getNodeDisplayName(rootGrp),
        children = nil,
        renderChildrenOrder = nil
      }
    end

    cacheGroupNode(instance, instance.rootNode, incomingObjectIds, 0)
    instance.rootNodeSizeCache = nil
    if openStatus and instance.rootNode then
      applyNodeOpenStatus(instance.rootNode, openStatus)
    end
  end
end

local function openNode(node)
  if node.parent then
    node.parent.open = true
    for index, instance in pairs(guiInstancer.instances) do
      instance.rootNodeSizeCache = nil
    end
    openNode(node.parent)
  end
end

local function updateNodeSelection(instance, node)
  if not editor.selection or not editor.selection.object or not node then return end
  if tableContains(editor.selection.object, node.id) then
    table.insert(instance.selectedNodes, node)
    node.selected = true
    openNode(node)
    if not instance.noScrollToSelection then
      instance.scrollToNode = editor.selection.object[1]
    end
  end
  if node.children then
    for _, child in pairs(node.children) do
      updateNodeSelection(instance, child)
    end
  end
end

local function changeNodeName(node, name)
  local searchedObj = scenetree.findObject(name)
  -- if its a different object, but has same name with out new name, then error
  if searchedObj and searchedObj:getID() ~= node.id then
    local msg = "'" .. name .. "' already exists in the scene, please choose another name"
    editor.logWarn(msg)
    editor.showNotification(msg)
    editor.setStatusBar(msg, function() if imgui.Button("Close##duplicate") then editor.hideStatusBar() end end)
    return
  end
  -- check new name if valid
  if SimObject.isNameValid(name) == false then
    local msg = "Cannot rename scene node to '" .. name .. "', bad format (cannot start with %, a digit 0-9 or /, cannot be a class name)"
    editor.logError(msg)
    editor.showNotification(msg)
    editor.setStatusBar(msg, function() if imgui.Button("Close##duplicate") then editor.hideStatusBar() end end)
    return
  end
  objectHistoryActions.changeObjectFieldWithUndo({node.id}, "name", name, 0)
  node.name = name
end

local function onEditorObjectSelectionChanged()
  for index, instance in pairs(guiInstancer.instances) do
    if instance.selectedNodes then
      for _, node in pairs(instance.selectedNodes) do
        node.selected = false
      end
    end
    instance.selectedNodes = {}
    if instance.rootNode then
      updateNodeSelection(instance, instance.rootNode)
      getRootNodeSize(instance)
    end
    instance.noScrollToSelection = nil
    if not clickedOnNode and #instance.selectedNodes > 0 then
      instance.lastSelectedIndex = instance.selectedNodes[#instance.selectedNodes].listIndex
    end
  end

  -- Apply last nodes' new name
  if editingNodeName then
    local newName = ffi.string(inputTextValue)
    local object = scenetree.findObjectById(editingNodeName.id)
    if object then
      changeNodeName(editingNodeName, newName)
    end
    editingNodeName = nil
  end
end

-- Change Ordering
local function changeOrderActionUndo(actionData)
  if not actionData.objects then
    return
  end
  for i = #actionData.objects, 1, -1 do
    local object = scenetree.findObjectById(actionData.objects[i])
    if object then
      local oldGroup = scenetree.findObjectById(actionData.oldGroups[i])
      if oldGroup then
        oldGroup:add(object)
        if actionData.placeholderObjects[i] then
          local placeholderObject = scenetree.findObjectById(actionData.placeholderObjects[i])
          if placeholderObject then
            oldGroup:reorderChild(object, placeholderObject)
          end
        end
      end
    end
  end
  onEditorObjectSelectionChanged()
end

local function changeOrderActionRedo(actionData)
  if not actionData.objects then
    editor.selectObjects({actionData.newGroup}, editor.SelectMode_New)
    return
  end

  -- Add to New Group
  local newGroup = scenetree.findObjectById(actionData.newGroup)
  if not newGroup then return end
  if actionData.destObject then
    local destObject = scenetree.findObjectById(actionData.destObject)
    for i = 1, #actionData.objects do
      local object = scenetree.findObjectById(actionData.objects[i])
      if object then
        newGroup:add(object)
        if destObject then
          newGroup:reorderChild(object, destObject)
        end
      end
    end
  else
    for i = #actionData.objects, 1, -1 do
      local object = scenetree.findObjectById(actionData.objects[i])
      if object and newGroup.add and newGroup.bringToFront then
        newGroup:add(object)
        newGroup:bringToFront(object)
      end
    end
  end

  -- Remove from Old group
  if actionData.oldGroups then
    for index, oldGroupId in ipairs(actionData.oldGroups) do
      if oldGroupId ~= actionData.newGroup then
        local group = scenetree.findObjectById(oldGroupId)
        if group then
          for _, instance in pairs(guiInstancer.instances) do
            local groupNode = findNodeByObject(instance, nil, group)
            if groupNode then
              local childId = actionData.objects[index]
              groupNode.children[childId] = nil
              local childIndex = arrayFindValueIndex(groupNode.renderChildrenOrder, childId)
              if childIndex then
                table.remove(groupNode.renderChildrenOrder, childIndex)
              end
            end
          end
        end
      end
    end
  end

  onEditorObjectSelectionChanged()
end

local function createSimGroupRedo(actionData)
  local objId = objectHistoryActions.createObjectRedo({name = "", className = "SimGroup", objectId = actionData.newGroup, parentId = actionData.parentId})
  if not actionData.newGroup then
    actionData.newGroup = objId
  end
  changeOrderActionRedo(actionData)
end

local function createSimGroupUndo(actionData)
  changeOrderActionUndo(actionData)
  objectHistoryActions.createObjectUndo({objectId = actionData.newGroup})
end

local function getNodeLevelRec(node, level)
  if node.parent then
    level = level + 1
    return getNodeLevelRec(node.parent, level)
  end
  return level
end

local function getNodeLevel(node)
  return getNodeLevelRec(node, 0)
end

local function getHighestNode(nodes)
  local levelHighestNode = math.huge
  local highestNode
  for i, node in ipairs(nodes) do
    local level = getNodeLevel(node)
    if level < levelHighestNode then
      levelHighestNode = level
      highestNode = node
    end
  end
  return highestNode
end

local function addNewGroupToSceneTree(instance, groupParentID, addSelectedObjects)
  if not groupParentID then
    local selNode = getHighestNode(instance.selectedNodes)
    if selNode then
      local object = scenetree.findObjectById(selNode.id)
      if selNode.isGroup and not addSelectedObjects then
        if object then
          groupParentID = object:getID()
        end
      else
        if object then
          local parent = object:getGroup()
          if parent then
            groupParentID = parent:getID()
          end
        end
      end
    end
  end

  if not groupParentID then
    groupParentID = scenetree.MissionGroup:getID()
  end

  local actionInfo = addSelectedObjects and getChangeOrderActionData(instance) or {}
  actionInfo.parentId = groupParentID
  editor.history:commitAction("CreateGroup", actionInfo, createSimGroupUndo, createSimGroupRedo)
  editor.setDirty()

  return scenetree.findObjectById(actionInfo.newGroup)
end

local function addNewGroupToSceneTreeFromSelection(instance)
  local grp = addNewGroupToSceneTree(instance, nil, true)
  if not grp then return end
  return grp
end

local function toggleNode(node)
  if node.isExpandable then
    node.open = not node.open
    editingNodeName = nil
    for index, instance in pairs(guiInstancer.instances) do
      instance.rootNodeSizeCache = nil
    end
  end
end

local function editNodeName(node)
  editingNodeName = node
  node.setFocus = true
  if node.name then
    ffi.copy(inputTextValue, node.name)
  end
end

local function nodeIsInTheSelection(instance, node)
  for i = 1, tableSize(instance.selectedNodes) do
    if instance.selectedNodes[i].id == node.id then return true end
  end
  return false
end

local function moveSelectionIndex(up)
  for index, instance in pairs(guiInstancer.instances) do
    if instance.focused and instance.currentListIndex then
      instance.newListIndex = instance.currentListIndex + (up and -1 or 1)
    end
  end
end

local function isGroupChildOfSelection(instance, group)
  local parent = group.parent
  if parent then
    for _, node in pairs(instance.selectedNodes) do
      if parent.id == node.id then
        return true
      end
    end
    return isGroupChildOfSelection(instance, parent)
  end
  return false
end

local function refreshAllNodes(incomingObjectIds)
  for _, instance in pairs(guiInstancer.instances) do
    cacheGroupNode(instance, instance.rootNode, incomingObjectIds, 0)
    instance.rootNodeSizeCache = nil
  end
end

local function sortGroupNode(instance, node, recursive)
  local object = scenetree.findObjectById(node.id)
  if node.isGroup and object then
    object:sortByName(false, false)
    -- we clear the children list so it will recreate it sorted
    node.children = nil
    node.renderChildrenOrder = nil
    local incomingObjectIds = object:getScenetreeData()
    cacheGroupNode(instance, node, incomingObjectIds, 0)

    if recursive then
      for _, child in pairs(node.children) do
        sortGroupNode(instance, child, recursive)
      end
    end
  end
end

local function collapseNode(node)
  if not node.children then return end
  for _, child in pairs(node.children) do
    child.open = false
    collapseNode(child)
  end
end

local function collapseAllSceneTree(instance)
  collapseNode(instance.rootNode)
end

--TODO: check if we can do the scene tree populate directly with no Lua tables
local function removeObjectFromSet(object, simset)
  for _, instance in pairs(guiInstancer.instances) do
    removeNodesByObjectIds(instance, {object:getID()})
  end
  refreshAllNodes()
  for index, instance in pairs(guiInstancer.instances) do
    refreshNodeCache(instance)
  end
end

local function reorderGroupChildren(group, groupNode)
  if not group then return end
  if not groupNode then return end

  local groupSceneData = group:getScenetreeData()
  local dataCount = #groupSceneData
  table.clear(groupNode.renderChildrenOrder)
  for index = 1, dataCount, 2 do
    local objId = groupSceneData[index]
    local child = groupNode.children[objId]
    if child then
      local childIndex = arrayFindValueIndex(groupNode.renderChildrenOrder, childId)
      if not childIndex then
        child.renderOrderIndex = #groupNode.renderChildrenOrder + 1
        table.insert(groupNode.renderChildrenOrder, objId)
      -- else
      --   log('E','', groupNode.id..': Duplicated child index detected child = '..child.id..' (site 1)')
      end
    end
  end
end

local function reorderGroups(data)
  local groupProcessed = {}
  local dataCount = #data
  for batchIndex = 2, dataCount, 2 do
    local groupId = data[batchIndex]
    if not groupProcessed[groupId] then
      groupProcessed[groupId] = true
      local group = scenetree.findObjectById(groupId)
      for _, instance in pairs(guiInstancer.instances) do
        local groupNode = findNodeByObject(instance, nil, group)
        reorderGroupChildren(group, groupNode)
      end
    end
  end
end

local function submitTransactions(transactions)
  local opTypeToString = {"None", "ClearSet", "AddObject", "RemoveObject", "AssignObjectName", "ReorderObject"}
  local opClearSet      = 1
  local opAddObject     = 2
  local opRemoveObject  = 3
  local opAssignName    = 4
  local opReorderObject = 5

  local getNextOperationData = function(transactions, curIndex)
    local op = nil
    local nextIndex = nil
    if curIndex < #transactions then
      op = {type = transactions[curIndex], objId = transactions[curIndex + 1], groupId = transactions[curIndex + 2]}
      nextIndex = curIndex + 3
    end
    return op, nextIndex
  end

  local submitBatchedOperation = function(operation, batchedData)
    -- log('I','','Submitting batch '..opTypeToString[operation + 1]..' batchedData = '..dumps(batchedData))
    if operation == opClearSet then
      recacheAllNodes(true) -- improve this. Maybe we can remove that set alone. instead of recaching all nodes
    elseif operation == opAddObject then
      if tableSize(batchedData) > 0 then
        refreshAllNodes(batchedData)
      end
    elseif operation == opRemoveObject then
      for _, instance in pairs(guiInstancer.instances) do
        removeNodesByObjectIds(instance, batchedData)
      end
      refreshAllNodes()
    elseif operation == opAssignName then
    elseif operation == opReorderObject then
      reorderGroups(batchedData)
    end

    for index, instance in pairs(guiInstancer.instances) do
      refreshNodeCache(instance)
    end

    table.clear(batchedData)
  end

  local op, curIndex = getNextOperationData(transactions, 1)
  local currentBatchData = {}
  local currentBatchOp = op and op.type or nil
  while op do
    if currentBatchOp ~= op.type then
      submitBatchedOperation(currentBatchOp, currentBatchData)
    end
    -- log('I','','Op = '..opTypeToString[op.type + 1]..' objId = '..tostring(op.objId)..' groupId = '..op.groupId)
    currentBatchOp = op.type
    table.insert(currentBatchData, op.objId)
    table.insert(currentBatchData, op.groupId)
    op, curIndex = getNextOperationData(transactions, curIndex)
  end
  if currentBatchOp then
    submitBatchedOperation(currentBatchOp, currentBatchData)
  end

  for _, instance in pairs(guiInstancer.instances) do
    if instance.rootNode then
      updateNodeSelection(instance, instance.rootNode)
      getRootNodeSize(instance)
    end
  end
end

local function selectChildrenRecursive(instance, parent, objectIDs)
  for _, node in pairs(parent.children) do
    if node.isGroup then
      selectChildrenRecursive(instance, node, objectIDs)
    else
      table.insert(objectIDs, node.id)
    end
  end
end

local function selectChildren(instance, node)
  local objectIDs = {}
  selectChildrenRecursive(instance, node, objectIDs)
  editor.selectObjects(objectIDs, editor.SelectMode_New)
end

local function rangesIntersect(r1, r2)
  return r1.min < r2.max and r1.max > r2.min
end

local function setFieldRec(node, v, objectIDs)
  table.insert(objectIDs, node.id)
  for _, n in pairs(node.children or {}) do
    setFieldRec(n, v, objectIDs)
  end
end

local function boolFieldButton(instance, node, field, iconOn, iconOff)
  local object = scenetree.findObjectById(node.id)
  if not object then return end
  local value = object:getField(field, 0) == "1"
  local icon = value and iconOn or iconOff
  if editor.uiIconImageButton(icon, iconSize, iconColor, "", nil, nil, iconColor, node.textBG, activateOnRelease) then
    local newValue = not value
    local objectIDs = {}
    table.insert(objectIDs, node.id)
    if node.selected then
      for _, n in ipairs(instance.selectedNodes) do
        setFieldRec(n, newValue, objectIDs)
      end
    end
    for _, n in pairs(node.children or {}) do
      setFieldRec(n, newValue, objectIDs)
    end
    objectHistoryActions.changeObjectFieldWithUndo(objectIDs, field, tostring(newValue), 0)
    editor.setDirty()
    return true
  end
end

local disableHoverColor
local drewDragSeparator

local function nodeSelectable(instance, node, icon, iconColor, iconSize, selectionColor, label, textColor, triggerOnRelease, highlightText)
  editor.uiIconImage(icon, iconSize, iconColor)
  imgui.SameLine()
  imgui.PushStyleColor2(imgui.Col_Header, selectionColor)
  if node.selected or node.dragSelected then
    imgui.PushStyleColor2(imgui.Col_HeaderHovered, selectionColor)
  elseif hasDragDropPayload and (disableHoverColor or not node.isGroup) then
    imgui.PushStyleColor2(imgui.Col_HeaderHovered, imgui.ImVec4(0,0,0,0))
  end
  imgui.Selectable1("##" .. label, node.selected or node.dragSelected, imgui.SelectableFlags_SpanAllColumns)
  imgui.SetItemAllowOverlap()
  local selectableHovered = imgui.IsItemHovered()
  if node.selected or node.dragSelected then
    imgui.PopStyleColor()
  elseif hasDragDropPayload and (disableHoverColor or not node.isGroup) then
    imgui.PopStyleColor()
  end
  imgui.PopStyleColor()

  if imgui.ImGuiTextFilter_IsActive(instance.nameFilter) then
    imgui.tooltip("ID: " .. node.id .. " Parent: " .. (node.parent.name or "<none>"))
  end

  imgui.SameLine()

  local object = scenetree.findObjectById(node.id)
  local textCol = textColor

  if object and object.locked then
    local lockedObjectTextColor = editor.getPreference("ui.general.lockedObjectTextColor")
    textCol = imgui.ImVec4(lockedObjectTextColor.r, lockedObjectTextColor.g, lockedObjectTextColor.b, lockedObjectTextColor.a)
  end

  editor.uiHighlightedText(label, highlightText, textCol)
  local textHovered = imgui.IsItemHovered()

  if hasDragDropPayload then
    -- Check if hovering between items
    local mousePosY = imgui.GetMousePos().y
    local itemRect = {min = imgui.GetItemRectMin(), max = imgui.GetItemRectMax()}
    local middlePoint = itemRect.min.y - imgui.GetStyle().FramePadding.y/2
    if mousePosY < middlePoint + entrySize/5 and mousePosY > middlePoint - entrySize/5 and imgui.IsWindowHovered(imgui.HoveredFlags_RootAndChildWindows) then
      local p1 = imgui.ImVec2(imgui.GetWindowPos().x, itemRect.min.y - imgui.GetStyle().FramePadding.y/2)
      local winSize = imgui.GetWindowSize()
      local p2 = imgui.ImVec2(imgui.GetWindowPos().x + winSize.x*2, p1.y)
      local dl = imgui.GetWindowDrawList()
      imgui.ImDrawList_AddLine(dl, p1, p2, imgui.GetColorU322(imgui.ImVec4(1,1,1,1)), 3)

      local parent = scenetree.findObjectById(node.parent.id)
      if imgui.IsMouseReleased(0) and parent and not node.parent.selected and not isGroupChildOfSelection(instance, parent) then
        local objects = {}
        local placeholderObjects = {}
        local oldGroups = {}
        for _, node in ipairs(instance.selectedNodes) do
          local object = scenetree.findObjectById(node.id)
          local parent = scenetree.findObjectById(node.parent.id)
          if object and parent then
            table.insert(objects, node.id)
            local nextObject = parent:getObject(parent:getObjectIndex(object) + 1)
            if nextObject then
              table.insert(placeholderObjects, nextObject:getID())
            end
            table.insert(oldGroups, node.parent.id)
          end
        end
        local actionInfo = {objects = objects, placeholderObjects = placeholderObjects, oldGroups = oldGroups, newGroup = node.parent.id, destObject = node.id}
        editor.history:commitAction("ChangeOrder", actionInfo, changeOrderActionUndo, changeOrderActionRedo)
        editor.setDirty()
      end
      drewDragSeparator = true
    elseif selectableHovered then
      if imgui.IsMouseReleased(0) then
        if node.isGroup and not node.hidden and not disableHoverColor then
          if not node.selected and not isGroupChildOfSelection(instance, node) then
            local actionInfo = getChangeOrderActionData(instance, node.id)
            editor.history:commitAction("ChangeOrder", actionInfo, changeOrderActionUndo, changeOrderActionRedo)
            editor.setDirty()
            nodeIdToOpen = node.id
          end
        end
      end
    end
  end

  imgui.TableNextColumn()
  -- ==============================
  -- following are the buttons for the table

  -- hide/unhide, check if it has an isHidden method
  if object and object.isHidden then
    if boolFieldButton(instance, node, "hidden", editor.icons.visibility_off, editor.icons.visibility) then
      clickedOnNode = true
    end
    if imgui.IsItemHovered() then selectableHovered = false end
    if object.isLocked then imgui.SameLine() end
  end

  -- lock/unlock, check if it has an isLocked method
  if object and object.isLocked then
    if boolFieldButton(instance, node, "locked", editor.icons.lock, editor.icons.lock_open) then
      clickedOnNode = true
    end
    if imgui.IsItemHovered() then selectableHovered = false end
  end

  if selectableHovered then
    if imgui.IsMouseDoubleClicked(0) then
      node.renameRequestTime = nil
      nodeWasDblClicked = true
      if node.isGroup then
        toggleNode(node)
      else
        editor.fitViewToSelectionSmooth()
      end
    end
    if imgui.IsMouseClicked(0) then
      clickedOnNode = true
    end
    if imgui.IsMouseDragging(0) then
      if clickedOnNode and not mouseDragRange and (triggerOnRelease or textHovered) then
        hasDragDropPayload = true
      end
      node.renameRequestTime = nil
    end
    if imgui.IsMouseClicked(1) then
      if not node.isGroup then
        imgui.SetWindowFocus1()
      end
    end
    if imgui.IsMouseReleased(1) then
      imgui.OpenPopup("##sceneItemPopupMenu"..node.id)
    end

    if triggerOnRelease then
      if imgui.IsMouseReleased(0) and not imgui.IsMouseDragging(0) then
        return true
      else
        return false
      end
    elseif imgui.IsMouseClicked(0) then
      return true
    end
  end
end

local deleteNodes = false
local hideSelectionClicked = false
local showSelectionClicked = false
local lockSelectionClicked = false
local unlockSelectionClicked = false
local objectRemoved = false

local function renderSceneGroup(instance, node, selectMode)
  if not showGroups then return end
  local icon = getGroupNodeIcon(node)
  local iconColor = node.overrideIconColor or imgui.GetStyleColorVec4(imgui.Col_Text)
  local textColor = node.overrideTextColor
  local selectionColor = imgui.GetStyleColorVec4(imgui.Col_ButtonActive)

  local arrowIcon = node.open and editor.icons.keyboard_arrow_down or editor.icons.keyboard_arrow_right
  imgui.PushStyleColor2(imgui.Col_Button, transparentColor)
  if editor.uiIconImageButton(arrowIcon, iconSize, nil, nil, nil, nil, selectionColor) then
    toggleNode(node)
  end
  imgui.PopStyleColor()
  imgui.SameLine()

  local nodeLabel = node.displayName

  if node.filterResult then
    if bit.band(node.filterResult, searchMatches[2]) ~= 0 and node.name ~= node.displayName then
      nodeLabel = nodeLabel .. ' [name: ' .. node.name .. ']'
    end
    if bit.band(node.filterResult, searchMatches[3]) ~= 0 then
      nodeLabel = nodeLabel .. ' [id: ' .. tostring(node.id) .. ']'
    end
    if bit.band(node.filterResult, searchMatches[4]) ~= 0 then
      nodeLabel = nodeLabel .. ' [class: ' .. node.className .. ']'
    end
  end

  if node.selected then
    if nodeSelectable(instance, node, icon, iconColor, iconSize, selectionColor, nodeLabel, nil, not onClickSelected, nameFilterText) then
      if (onClickSelected or clickedOnNode) and not hasDragDropPayload then
        if tableSize(editor.selection.object) == 1 and not nodeWasDblClicked then
          node.renameRequestTime = socket.gettime()
        end
        -- just reset selection to this one
        if not editor.editingObjectName then
          selectNode(instance, node, selectMode)
        else
          editor.postNameChangeSelectObjectId = node.id
        end
        nodeWasDblClicked = nil
      end
    end
  else
    if nodeSelectable(instance, node, icon, iconColor, iconSize, selectionColor, nodeLabel, textColor, nil, nameFilterText) then
      onClickSelected = true
      if not editor.editingObjectName then
        selectNode(instance, node, selectMode)
      else
        editor.postNameChangeSelectObjectId = node.id
      end
    end
  end

  node.textBG = nil

  if imgui.BeginPopup("##sceneItemPopupMenu"..node.id) then
    if not nodeIsInTheSelection(instance, node) then
      selectNode(instance, node, editor.SelectMode_New)
    end
    if imgui.Selectable1("Select Children") then
      if tableSize(instance.selectedNodes) == 1 then
        local parentNode = instance.selectedNodes[1]
        selectChildren(instance, parentNode)
      end
    end
    if imgui.Selectable1("New Group") then
      local grp = addNewGroupToSceneTree(instance)
      if grp then
        editor.selectObjectById(grp:getID())
      else
        editor.logError("Cannot add new group to scene tree")
      end
    end
    if imgui.Selectable1("Duplicate Selection") then
      editor.duplicate()
    end
    if imgui.Selectable1("Delete Selection") then
      if not tableIsEmpty(instance.selectedNodes) then
        deleteNodes = true
      end
    end
    if imgui.Selectable1("Put Into New Group") then
      local grp = addNewGroupToSceneTreeFromSelection(instance)
      local grpNode = findNodeByObject(instance, nil, grp)
      selectNode(instance, grpNode)
    end
    imgui.Separator()
    if not tableIsEmpty(instance.selectedNodes) then
      if imgui.Selectable1("Hide Selection") then
        hideSelectionClicked = true
      end
      if imgui.Selectable1("Show Selection") then
        showSelectionClicked = true
      end
    end
    if not tableIsEmpty(instance.selectedNodes) then
      if imgui.Selectable1("Lock Selection") then
        lockSelectionClicked = true
      end
      if imgui.Selectable1("Unlock Selection") then
        unlockSelectionClicked = true
      end
    end
    imgui.Separator()
    if imgui.Selectable1("Sort Group") then
      sortGroupNode(instance, node)
    end
    if imgui.Selectable1("Sort Group Recursive") then
      sortGroupNode(instance, node, true)
    end
    imgui.Separator()
    local object = scenetree.findObjectById(node.id)
    if object and node.className ~= "Prefab" then
      if imgui.Selectable1("Pack Prefab") then
        --TODO: check JSON save, load, replace cs to json save load
        if object:getField("prefab_filename", "") ~= "" then
          local prefab = editor.createPrefabFromObjectSelection(object:getField("prefab_filename", ""), object:getField("prefab_name", ""))
          local prefabNode = findNodeByObject(instance, nil, prefab)
          selectNode(instance, prefabNode)
          imgui.EndPopup()
          objectRemoved = true
          return
        else
          extensions.editor_fileDialog.saveFile(function(data)
            local prefab = editor.createPrefabFromObjectSelection(data.filepath, nil, "auto")
            local prefabNode = findNodeByObject(instance, nil, prefab)
            selectNode(instance, prefabNode) end,
            {{"Prefab Files (JSON)",".prefab.json"}}, false,
              FS:directoryExists(prefabSaveFolder) and prefabSaveFolder or "/")
          imgui.EndPopup()
          objectRemoved = true
          return
        end
      end
    end
    imgui.Separator()
    if imgui.Selectable1("Collapse Parent Group") then
      local parentNode = node.parent
      if parentNode then parentNode.open = false end
    end
    if imgui.Selectable1("Collapse All Scene Tree") then
      collapseAllSceneTree(instance)
    end

    imgui.EndPopup()
  end
end

local function renderSceneNode(instance, node, selectMode, overrideTextColor, overrideIconColor)
  if node.hidden then return end

  local iconColor = overrideIconColor or imgui.GetStyleColorVec4(imgui.Col_Text)
  local selectionColor = imgui.GetStyleColorVec4(imgui.Col_ButtonActive)
  local activateOnRelease = node.selected and not onClickSelected
  local textColor = overrideTextColor
  imgui.Spacing()
  imgui.SameLine()

  local nodeLabel = node.displayName

  if node.filterResult then
    if bit.band(node.filterResult, searchMatches[2]) ~= 0 and node.name ~= node.displayName then
      nodeLabel = nodeLabel .. ' [name: ' .. node.name .. ']'
    end
    if bit.band(node.filterResult, searchMatches[3]) ~= 0 then
      nodeLabel = nodeLabel .. ' [id: ' .. tostring(node.id) .. ']'
    end
    if bit.band(node.filterResult, searchMatches[4]) ~= 0 then
      nodeLabel = nodeLabel .. ' [class: ' .. node.className .. ']'
    end
  end

  if nodeSelectable(instance, node, node.icon or defaultObjectNodeIcon, iconColor, iconSize, selectionColor, nodeLabel, textColor, activateOnRelease, nameFilterText) then
    if (not activateOnRelease or clickedOnNode) and not hasDragDropPayload then
      if node.selected and not (ctrlDown or shiftDown) then
        if tableSize(editor.selection.object) == 1 and not nodeWasDblClicked then
          node.renameRequestTime = socket.gettime()
        end
        if not editor.editingObjectName then
          selectNode(instance, node, selectMode)
        else
          editor.postNameChangeSelectObjectId = node.id
        end
        nodeWasDblClicked = nil
      else
        onClickSelected = true
        if not editor.editingObjectName then
          selectNode(instance, node, selectMode)
        else
          editor.postNameChangeSelectObjectId = node.id
        end
      end
      instance.noScrollToSelection = true
    end
  end

  if searchRange > 0 and node.cameraDistance then
    imgui.TableNextColumn()
    imgui.TextUnformatted(string.format('%0.1f', node.cameraDistance) .. 'm')
  end

  if imgui.BeginPopup("##sceneItemPopupMenu"..node.id) then
    if not nodeIsInTheSelection(instance, node) then
      selectNode(instance, node, editor.SelectMode_New)
    end
    if imgui.Selectable1("Duplicate Selection") then
      if not tableIsEmpty(instance.selectedNodes) then
        editor.duplicate()
      end
    end
    if imgui.Selectable1("Delete Selection") then
      if not tableIsEmpty(instance.selectedNodes) then
        deleteNodes = true
      end
    end
    if not tableIsEmpty(instance.selectedNodes) then
      if imgui.Selectable1("Hide Selection") then
        hideSelectionClicked = true
      end
      if imgui.Selectable1("Show Selection") then
        showSelectionClicked = true
      end
    end
    if not tableIsEmpty(instance.selectedNodes) then
      if imgui.Selectable1("Lock Selection") then
        lockSelectionClicked = true
      end
      if imgui.Selectable1("Unlock Selection") then
        unlockSelectionClicked = true
      end
    end
    imgui.Separator()
    if imgui.Selectable1("Put Into New Group") then
      local grp = addNewGroupToSceneTreeFromSelection(instance)
      local grpNode = findNodeByObject(instance, nil, grp)
      selectNode(instance, grpNode)
    end
    if imgui.Selectable1("Pack Into Prefab") then
      extensions.editor_fileDialog.saveFile(function(data)
        local prefab = editor.createPrefabFromObjectSelection(data.filepath, nil, "auto")
        local prefabNode = findNodeByObject(instance, nil, prefab)
        selectNode(instance, prefabNode) end,
        {{"Prefab Files (JSON)",".prefab.json"}}, false,
          FS:directoryExists(prefabSaveFolder) and prefabSaveFolder or "/")
    end
    if node.className == "Prefab" then
      if imgui.Selectable1("Unpack Prefab") then
        local groups = editor.explodeSelectedPrefab()
        if tableSize(groups) then
          if groups[1] then
            local grpNode = findNodeByObject(instance, nil, groups[1])
            selectNode(instance, grpNode)
            -- node is now nil/invalid since it was deleted by unpacking, assign group node
            node = grpNode
          end
        end
        objectRemoved = true
      end
    end
    imgui.Separator()
    if imgui.Selectable1("Collapse Parent Group") then
      local parentNode = node.parent
      if parentNode then parentNode.open = false end
    end
    if imgui.Selectable1("Collapse All Scene Tree") then
      collapseAllSceneTree(instance)
    end

    if imgui.Selectable1("Inspect in new Window") then
      editor.addInspectorInstance(editor.selection)
    end
    if imgui.IsItemHovered() then imgui.SetTooltip("New Inspector Window for the selected object(s)") end

    --  Extended menu items generation
    --  Items are "registered" via the `editor.addExtendedSceneTreeObjectMenuItem` method
    --  They are displayed in a "More >" submenu.
    if #extendedSceneTreeObjectMenuItems > 0 then
      --  Constructs valid custom items
      local validCustomMenuItems = {}
      for _, item in ipairs(extendedSceneTreeObjectMenuItems) do
        local validator = item.validator or function(obj) return true end
        if validator(node) then
          table.insert(validCustomMenuItems, item)
        end
      end

      if #validCustomMenuItems > 0 then
        imgui.Separator()
        local generateExtendedSceneTreeObjectMenuItems = function(items)
          for _, item in ipairs(items) do
            if item.title and imgui.Selectable1(item.title) and item.extendedSceneTreeObjectMenuItems then
              item.extendedSceneTreeObjectMenuItems(node)
            end
          end
        end
        -- Generates the "More >" submenu
        if imgui.BeginMenu("More", imgui_true) then
          generateExtendedSceneTreeObjectMenuItems(validCustomMenuItems)
          imgui.EndMenu()
        end
      end
    end

    imgui.EndPopup()
  end
end

local function renderSceneTreeGui(instance, node, recursiveDisplay, overrideIconColor, oveTextColor)
  if not node then return end
  if not node.id then return end
  local selectMode = editor.SelectMode_New

  local ctrlDown = editor.keyModifiers.ctrl
  local shiftDown = editor.keyModifiers.shift
  local altDown = editor.keyModifiers.alt

  if ctrlDown then selectMode = editor.SelectMode_Toggle end
  if altDown then selectMode = editor.SelectMode_Remove end
  if shiftDown then selectMode = SelectMode_Range end

  -- skip root node from showing in the scene tree
  if node ~= instance.rootNode then
    if instance.newListIndex and node.listIndex == instance.newListIndex then
      selectNode(instance, node, selectMode)
      instance.newListIndex = nil
      if imgui.GetCursorPosY() + entrySize > (imgui.GetScrollY() + imgui.GetWindowHeight()) or imgui.GetCursorPosY() < imgui.GetScrollY() then
        imgui.SetScrollY(imgui.GetCursorPosY() - imgui.GetWindowHeight()/2)
      end
    end
    if instance.scrollToNode and instance.scrollToNode == node.id then
      if not node.hidden then
        if imgui.GetCursorPosY() > (imgui.GetScrollY() + imgui.GetWindowHeight()) or imgui.GetCursorPosY() + entrySize < imgui.GetScrollY() then
          imgui.SetScrollY((node.listIndex or 1) * entrySize - imgui.GetWindowHeight()/2)
        end
      end
      instance.scrollToNode = nil
    end

    if instance.selectionRange then
      if instance.selectionRange[node.listIndex] then
        if not instance.objectsToSelect then instance.objectsToSelect = {} end
        table.insert(instance.objectsToSelect, node.id)
        instance.selectionRange[node.listIndex] = nil
      end
      if tableIsEmpty(instance.selectionRange) then
        instance.selectionRange = nil
        if not editor.editingObjectName then
          editor.selectObjects(instance.objectsToSelect)
        else
          if instance.objectsToSelect and #instance.objectsToSelect then
            editor.postNameChangeSelectObjectId = instance.objectsToSelect[1]
          end
        end
        instance.objectsToSelect = nil
      end
    end

    local skipGui = false
    if not node.hidden and imgui.GetCursorPosY() + entrySize < imgui.GetScrollY() then
      imgui.SetCursorPosY(imgui.GetCursorPosY() + entrySize)
      skipGui = true
    end

    if imgui.GetCursorPosY() > (imgui.GetScrollY() + imgui.GetWindowHeight()) and not instance.scrollToNode then
      imgui.SetCursorPosY(instance.scenetreeSize)
      return
    end

    if not skipGui and not node.hidden then
      imgui.TableNextRow()
      imgui.TableNextColumn()

      if nodeIdToOpen and nodeIdToOpen == node.id then
        node.open = true
        nodeIdToOpen = nil
      end

      -- Turn the name into a text field for name editing
      if editingNodeName == node then
        local icon = node.icon
        if node.isGroup then
          icon = getGroupNodeIcon(node)
          local arrowIcon = node.open and editor.icons.keyboard_arrow_down or editor.icons.keyboard_arrow_right
          imgui.PushStyleColor2(imgui.Col_Button, transparentColor)
          editor.uiIconImageButton(arrowIcon, iconSize, iconColor, nil, nil, nil, iconColor)
          imgui.PopStyleColor()
          imgui.SameLine()
        end
        editor.uiIconImageButton(icon, iconSize, imgui.GetStyleColorVec4(imgui.Col_Text))
        imgui.SameLine()
        if node.setFocus then
          imgui.SetKeyboardFocusHere()
        end
        editor.uiInputText("", inputTextValue, imgui.ArraySize(inputTextValue), imgui.InputTextFlags_AutoSelectAll, nil, nil, editEnded)
        if editEnded[0] or (not imgui.IsItemActive() and not node.setFocus) then
          local newName = ffi.string(inputTextValue)
          local object = scenetree.findObjectById(node.id)
          if object then
            changeNodeName(node, newName)
          end
          editingNodeName = nil
        end
        if node.setFocus then
          node.setFocus = nil
        end
      elseif node.isGroup and not node.hidden then
        renderSceneGroup(instance, node, selectMode)
      else
        renderSceneNode(instance, node, selectMode, overrideIconColor, overrideTextColor)
      end

      if objectRemoved then node = nil objectRemoved = false end
      if not node then return end

      if tableSize(editor.selection.object) == 1
          and node.renameRequestTime
          and not imgui.IsMouseDown(0)
          and (socket.gettime() - node.renameRequestTime) > imgui.GetIO().MouseDoubleClickTime
          and node.name ~= "MissionGroup" then
        editNodeName(node)
        node.renameRequestTime = nil
      end

      node.dragSelected = false
      if mouseDragRange and node.listIndex then
        dragSelectionList[node.listIndex] = nil
        local itemRectRange = {min = imgui.GetItemRectMin().y, max = imgui.GetItemRectMax().y}
        itemRectRange.min = itemRectRange.min - 2
        itemRectRange.max = itemRectRange.max + 2
        if rangesIntersect(mouseDragRange, itemRectRange) then
          dragSelectionList[node.listIndex] = true
          node.dragSelected = true
        end
      end
    end
  end -- end skip root node if

  if recursiveDisplay and (node.isGroup or node.isExpandable) and (not showGroups or node.open or node.openOnSearch) then
    if showGroups and node ~= instance.rootNode then imgui.Indent() end

    for orderIndex, childId in ipairs(node.renderChildrenOrder) do
      local child = node.children[childId]
      if child then
        child.overrideIconColor =  node.overrideIconColor
        child.overrideTextColor =  node.overrideTextColor
        child.renderOrderIndex = orderIndex
        renderSceneTreeGui(instance, child, recursiveDisplay, node.overrideIconColor, node.overrideTextColor)
      else
        table.remove(node.renderChildrenOrder, orderIndex)
      end
    end

    if showGroups and node ~= instance.rootNode then imgui.Unindent() end
  end

  if hasDragDropPayload then
    if imgui.IsKeyDown(imgui.GetKeyIndex(imgui.Key_Escape)) then
      hasDragDropPayload = false
    else
      imgui.SetMouseCursor(2) -- ResizeAll cursor
    end
  end

  if hideSelectionClicked then
    editor.hideObjectSelection()
  end

  if showSelectionClicked then
    editor.showObjectSelection()
  end

  if lockSelectionClicked then
    editor.lockObjectSelection()
  end

  if unlockSelectionClicked then
    editor.unlockObjectSelection()
  end

  if deleteNodes then
    editor.deleteSelectedObjects()
  end

  deleteNodes = false
  hideSelectionClicked = false
  showSelectionClicked = false
  lockSelectionClicked = false
  unlockSelectionClicked = false
end

local function addNewSceneTreeInstance()
  local index = guiInstancer:addInstance()
  local wndName = sceneTreeWindowNamePrefix .. index
  guiInstancer.instances[index].locked = true -- will not scroll to the new selection, will stay at its scroll position
  guiInstancer.instances[index].nameFilter = imgui.ImGuiTextFilter()
  guiInstancer.instances[index].selectedNodes = {}
  guiInstancer.instances[index].windowName = wndName
  recacheAllNodes()
  editor.registerWindow(wndName, imgui.ImVec2(300, 500))
  editor.showWindow(wndName)
end

local function openSceneTree()
  --TODO: this will force 1 instance only of the scene tree
  if tableSize(guiInstancer.instances) == 1 then return end
  addNewSceneTreeInstance()
end

local function onEditorGui()
  drewDragSeparator = false
  entrySize = round(math.max(imgui.CalcTextSize("W").y, iconSize.y * imgui.uiscale[0]) + imgui.GetStyle().FramePadding.y + 1) + 4
  for index, instance in pairs(guiInstancer.instances) do
    currentSceneTreeInstanceIndex = index
    local wndName = instance.windowName
    imgui.PushStyleColor2(imgui.Col_Button, imgui.ImVec4(0,0,0,0))

    if not editor.isWindowVisible(wndName) then
      guiInstancer:removeInstance(index)
      editor.unregisterWindow(wndName)
    elseif editor.beginWindow(wndName, "SceneTree##" .. index) then
      -- SceneTree toolbar
      local filterTypeComboWidth = 100
      local style = imgui.GetStyle()
      local searchRangeIconWidth = 24
      local helpIconWidth = 24
      local searchFilterWidth = imgui.GetContentRegionAvailWidth() - (filterTypeComboWidth + 2 * (searchRangeIconWidth + helpIconWidth) * imgui.uiscale[0] + 2 * style.ItemSpacing.x)

      if editor.uiIconImageButton(editor.icons.create_new_folder, imgui.ImVec2(24, 24)) then
        addNewGroupToSceneTree(instance)
      end
      if imgui.IsItemHovered() then imgui.SetTooltip("New subgroup (folder) in the selected group") end
      imgui.SameLine()
      imgui.PushID1("SceneSearchFilter")
      if editor.uiInputSearchTextFilter("##nodeNameSearchFilter", instance.nameFilter, searchFilterWidth, nil, nil, editEnded) then
        if ffi.string(imgui.TextFilter_GetInputBuf(instance.nameFilter)) == "" then
          imgui.ImGuiTextFilter_Clear(instance.nameFilter)
          if instance.rootNode then
            instance.noScrollToSelection = false
            updateNodeSelection(instance, instance.rootNode)
          end
        end
        refreshNodeCache(instance)
      end
      imgui.PopID()
      if imgui.IsItemHovered() then imgui.SetTooltip("Search text") end
      imgui.SameLine()
      imgui.PushItemWidth(filterTypeComboWidth)
      comboIndex[0] = searchNodeMode
      if imgui.Combo1("##filterType", comboIndex, searchTypesComboItems) then
        searchNodeMode = comboIndex[0]
      end
      if imgui.IsItemHovered() then imgui.SetTooltip("Search filter mode") end
      imgui.PopItemWidth()

      imgui.SameLine()

      local bgColor = nil
      if searchRange > 0 then bgColor = imgui.GetStyleColorVec4(imgui.Col_ButtonActive) end
      if editor.uiIconImageButton(editor.icons.wifi_tethering, imgui.ImVec2(searchRangeIconWidth, searchRangeIconWidth), nil, nil, bgColor) then
        if searchRange > 0 then
          searchRange = -1
        else
          searchRange = 200
        end
        showGroups = searchRange < 0
        applyFilter(instance, instance.rootNode)
      end
      if imgui.IsItemHovered() then imgui.SetTooltip("Only show near objects") end

      imgui.SameLine()

      editor.uiHelpButton("Scene Tree", "world_editor/windows/scenetree/")

      local maxTreeHeight = imgui.GetContentRegionAvail().y - entrySize - (imgui.GetStyle().FramePadding.y * 2 + imgui.GetStyle().ItemInnerSpacing.y + 2 * imgui.GetStyle().ItemSpacing.y) - 5

      if #selectedNodePathNodes == 0 then
        imgui.Text("<no selection>")
      end
      for i = #selectedNodePathNodes, 1, -1 do
        if imgui.SmallButton(selectedNodePathNodes[i].displayName) then
          if i > 1 then
            instance.scrollToNode = selectedNodePathNodes[i].id
          else
            instance.scrollToNode = selectedNodePathNodes[i].id
            selectNode(instance, selectedNodePathNodes[i])
          end
        end
        if i > 1 then imgui.SameLine() imgui.Text(">") imgui.SameLine() end
      end

      imgui.BeginChild1("Scene Tree Child", imgui.ImVec2(0, searchResultsMode and maxTreeHeight or 0), false)
      if searchResultsMode then
        instance.scenetreeSize = #searchResults * entrySize
        instance.rootNodeSizeCache = instance.scenetreeSize
        instance.listIndex = 1
      else
        instance.scenetreeSize = getRootNodeSize(instance) * entrySize
      end

      -- Renders alternate rows on all window
      local tableFlags = bit.bor(imgui.TableFlags_ScrollY, imgui.TableFlags_BordersV, imgui.TableFlags_BordersOuterH, imgui.TableFlags_Resizable, imgui.TableFlags_RowBg, imgui.TableFlags_NoBordersInBody)

      local colCount = 2
      if searchRange > 0 then colCount = colCount + 1 end

      if imgui.BeginTable('##scenetreetable', colCount, tableFlags) then
        -- The first column will use the default _WidthStretch when ScrollX is Off and _WidthFixed when ScrollX is On
        local textBaseWidth = imgui.CalcTextSize('A').x
        imgui.TableSetupScrollFreeze(0, 1) -- Make top row always visible
        imgui.TableSetupColumn('Tree', imgui.TableColumnFlags_NoHide)
        if searchRange > 0 then
          imgui.TableSetupColumn('Distance', imgui.TableColumnFlags_WidthFixed, textBaseWidth * 6)
        end
        imgui.TableSetupColumn('Controls', imgui.TableColumnFlags_WidthFixed, textBaseWidth * 6)
        imgui.TableHeadersRow()

        --  SceneTree list
        if searchResultsMode then
          -- refreshNodeCache every half second
          searchRangeTime = searchRangeTime + searchRangeTimer:stopAndReset()
          if searchRangeTime > 500 then
            searchRangeTime = math.fmod(searchRangeTime, 500)
            refreshNodeCache(instance)
          end

          for li, n in ipairs(searchResults) do
            n.listIndex = li
            instance.listIndex = instance.listIndex + 1
            renderSceneTreeGui(instance, n, false)
          end
        else
          renderSceneTreeGui(instance, instance.rootNode, true)
        end

        imgui.EndTable()
      end

      if imgui.IsMouseClicked(0) and imgui.IsWindowHovered(imgui.HoveredFlags_RootAndChildWindows) then
        if not clickedOnNode and not editor.keyModifiers.ctrl then
          editor.clearObjectSelection()
        end
        local mousePos = imgui.GetMousePos()
        if mousePos.x < imgui.GetWindowPos().x + imgui.GetWindowWidth() - 16 then
          instance.mouseDragStartPos = mousePos
          instance.mouseDragStartScrollY = imgui.GetScrollY()
        end
      end

      if imgui.IsMouseDragging(0) and instance.mouseDragStartPos and not hasDragDropPayload then
        if not editor.keyModifiers.ctrl then
          editor.clearObjectSelection()
        end
        local mouseDragEndPos = imgui.GetMousePos()
        local scrollYDiff = imgui.GetScrollY() - instance.mouseDragStartScrollY
        mouseDragRange = {min = math.min(instance.mouseDragStartPos.y - scrollYDiff, mouseDragEndPos.y),
                          max = math.max(instance.mouseDragStartPos.y - scrollYDiff, mouseDragEndPos.y)}

        local localMouseDragStartPos = imgui.ImVec2(instance.mouseDragStartPos.x, instance.mouseDragStartPos.y - scrollYDiff)
        local winPos = imgui.GetWindowPos()
        local winSize = imgui.GetWindowSize()

        if mouseDragEndPos.y < winPos.y then
          imgui.SetScrollY(imgui.GetScrollY() - 10)
        end
        if mouseDragEndPos.y > winPos.y + winSize.y then
          imgui.SetScrollY(imgui.GetScrollY() + 10)
        end

        imgui.ImDrawList_AddRect(imgui.GetWindowDrawList(), localMouseDragStartPos, mouseDragEndPos, imgui.GetColorU322(imgui.ImVec4(1, 1, 0, 1)))
      end

      if imgui.IsMouseReleased(0) and instance.mouseDragStartPos then
        if not hasDragDropPayload then
          local maxIndex = -1
          local minIndex = math.huge
          instance.selectionRange = {}
          for nodeListIndex, _ in pairs(dragSelectionList) do
            instance.selectionRange[nodeListIndex] = true
            if nodeListIndex > maxIndex then maxIndex = nodeListIndex end
            if nodeListIndex < minIndex then minIndex = nodeListIndex end
          end
          if not tableIsEmpty(dragSelectionList) then
            instance.lastSelectedIndex = minIndex
            instance.currentListIndex = maxIndex
          end
        end
        instance.noScrollToSelection = true
        instance.mouseDragStartPos = nil
        dragSelectionList = {}
      end

      imgui.EndChild()

      -- footer
      if searchResultsMode then
        editor.uiIconImage(editor.icons.find_in_page, imVec24x24)
        imgui.SameLine()
        --imgui.Dummy(imgui.ImVec2(5, imgui.GetStyle().ItemSpacing.y))
        local label = tostring(#searchResults) .. ' matches'
        if searchRange > 0 then
          label = label .. ' in ' .. string.format('%g', searchRange) .. 'm'
        end
        imgui.TextUnformatted(label)
      end
    end
    editor.endWindow()
    imgui.PopStyleColor()
  end

  if imgui.IsMouseReleased(0) then
    onClickSelected = false
    clickedOnNode = false
    mouseDragRange = nil
  end

  if hasDragDropPayload then
    selectedNodeTextColor = imgui.ImColorByRGB(0,255,255,150)
    if imgui.IsMouseReleased(0) then
      selectedNodeTextColor = imgui.ImColorByRGB(0,255,255,255)
      hasDragDropPayload = false
    end
  end

  -- disable hover coloring when the cursor is between two items
  disableHoverColor = drewDragSeparator
end

local function onExtensionLoaded()
  log('D', logTag, "initialized")
  local searchTypesComboItemsTbl = {"By Name", "By ID", "By Persistent ID", "By Class", "All"}
  searchTypesComboItems = imgui.ArrayCharPtrByTbl(searchTypesComboItemsTbl)

  editor.addExtendedSceneTreeObjectMenuItem = function(item)
    -- Expected item format:
    -- {
    -- title = string                                               -- required menu item title
    -- onExtendedSceneTreeObjectMenuItemSelected = function(node)   -- function to applu extension behavior on sceneTree
    -- validator = function(node) or nil                            -- optional function to check if menu applicable to sceneTree node
    -- }
    -- No validation for now
    table.insert(extendedSceneTreeObjectMenuItems, item)
  end
end

local function onWindowMenuItem()
  openSceneTree()
end

local function onEditorLoadGuiInstancerState(state)
  guiInstancer:deserialize("scenetreeInstances", state)
  recacheAllNodes()
  for key, instance in pairs(guiInstancer.instances) do
    instance.nameFilter = imgui.ImGuiTextFilter()
    instance.selectedNodes = {}
    instance.windowName = sceneTreeWindowNamePrefix .. key
    editor.registerWindow(instance.windowName, imgui.ImVec2(300, 500))
  end
end

local function onEditorSaveGuiInstancerState(state)
  local instancesCopy = deepcopy(guiInstancer.instances)
  for key, instance in pairs(guiInstancer.instances) do
    instance.nameFilter = nil
    instance.selectedNodes = nil
    instance.rootNode = nil
    instance.scenetreeSize = nil
    instance.currentListIndex = nil
    instance.newListIndex = nil
  end
  guiInstancer:serialize("scenetreeInstances", state)
  guiInstancer.instances = instancesCopy
end

local function onEditorActivated()
  onEditorObjectSelectionChanged()
end

local function onEditorAfterOpenLevel()
  recacheAllNodes()
  for index, instance in pairs(guiInstancer.instances) do
    instance.selectedNodes = {}
    imgui.ImGuiTextFilter_Clear(instance.nameFilter)
    applyFilter(instance, instance.rootNode)
  end
end

local function onEditorInitialized()
  defaultObjectNodeIcon = editor.icons.stop
  objectClassIcons = worldEditorCppApi.getObjectClassIcons()
  recacheAllNodes()
  for index, instance in pairs(guiInstancer.instances) do
    instance.selectedNodes = {}
  end

  editor.removeObjectFromSet = removeObjectFromSet
  editor.submitTransactions = submitTransactions
  editor.getSelectedSceneTreeNodes = function()
    if tableSize(guiInstancer.instances) then
      --TODO: remove the "0" key, was a wrong decision to use 0-based indices
      if guiInstancer.instances["0"] then
        return guiInstancer.instances["0"].selectedNodes
      else
        return guiInstancer.instances[tostring(guiInstancer.nextInstanceIndex - 1)].selectedNodes
      end
    end
  end
  editor.getSceneTreeSelectedGroup = getSceneTreeSelectedGroup
  editor.refreshSceneTreeWindow = function () recacheAllNodes() end
  editor.addWindowMenuItem("Scene Tree", onWindowMenuItem, nil, true)
  editor.hideAllSceneTreeInstances = function()
    for _, wnd in pairs(guiInstancer.instances) do
      editor.hideWindow(wnd.windowName)
    end
  end
  editor.showAllSceneTreeInstances = function()
    for _, wnd in pairs(guiInstancer.instances) do
      editor.showWindow(wnd.windowName)
    end
  end
  if path.split(getMissionFilename()) then
    prefabSaveFolder = path.split(getMissionFilename()).."art/prefabs/"
  end
end

local function onWindowGotFocus(windowName)
  for index, instance in pairs(guiInstancer.instances) do
    if windowName == sceneTreeWindowNamePrefix .. index then
      instance.focused = true
      pushActionMap("SceneTree")
      return
    end
  end
end

local function onWindowLostFocus(windowName)
  local allLostFocus = true
  for index, instance in pairs(guiInstancer.instances) do
    if windowName == sceneTreeWindowNamePrefix .. index then
      instance.focused = false
    end
    if instance.focused then
      allLostFocus = false
    end
  end
  if allLostFocus then
    popActionMap("SceneTree")
  end
end

local itemCount = 0
local function setOrder(object)
  local obj = Sim.upcast(object)
  table.insert(editor.orderTable, obj:getId())
  itemCount = itemCount + 1
  local isGroup = obj:isSubClassOf("SimSet") or obj:isSubClassOf("SimGroup")
  if isGroup then
    local count = obj:size() - 1
    for i = 0, count do
      local child = obj:at(i)
      setOrder(child)
    end
  end
end

local function onEditorBeforeSaveLevel()
  itemCount = 0
  local rootGrp = Sim.findObject("MissionGroup")
  editor.orderTable = {}
  if rootGrp then
    setOrder(rootGrp)
  end
end

local function onEditorObjectAdded()
  recacheAllNodes(true)
  for index, instance in pairs(guiInstancer.instances) do
    refreshNodeCache(instance)
  end
end

--TODO: check if we can do the scene tree populate directly with no Lua tables
local function refreshNodeNames(objectIds)
  if not objectIds then return end
  for index, instance in pairs(guiInstancer.instances) do
    local renamer = function(func, node, objectIds)
      if tableContains(objectIds, node.id) then
        local object = scenetree.findObjectById(node.id)
        node.name = getNodeName(object)
        node.displayName = getNodeDisplayName(object)
      end
      if node.isGroup then
        for _, child in pairs(node.children) do
          func(func, child, objectIds)
        end
      end
    end
    if instance and instance.rootNode then
      renamer(renamer, instance.rootNode, objectIds)
    end
  end
end

local function onEditorInspectorFieldChanged(selectedIds)
  refreshNodeNames(selectedIds)
end

local function closeAllInstances()
  for index, instance in pairs(guiInstancer.instances) do
    local wndName = instance.windowName
    guiInstancer:removeInstance(index)
    editor.unregisterWindow(wndName)
  end
end

local function debugNode(id, level)
  local object = scenetree.findObjectById(id)
  if object then
    for index, instance in pairs(guiInstancer.instances) do
      local node = findNodeByObject(instance, nil, object)
      if node then
        log('I','',tostring(id)..' node = '..dumpsz(node, level))
      end
    end
  end
end

M.onEditorInitialized = onEditorInitialized
M.onEditorActivated = onEditorActivated
M.onEditorGui = onEditorGui
M.onEditorSaveGuiInstancerState = onEditorSaveGuiInstancerState
M.onEditorLoadGuiInstancerState = onEditorLoadGuiInstancerState
M.onExtensionLoaded = onExtensionLoaded
M.onEditorAfterOpenLevel = onEditorAfterOpenLevel
M.onEditorObjectSelectionChanged = onEditorObjectSelectionChanged
M.onEditorToolWindowGotFocus = onWindowGotFocus
M.onEditorToolWindowLostFocus = onWindowLostFocus
M.onEditorBeforeSaveLevel = onEditorBeforeSaveLevel
M.onEditorObjectAdded = onEditorObjectAdded
M.onEditorInspectorFieldChanged = onEditorInspectorFieldChanged

M.moveSelectionIndex = moveSelectionIndex
M.refreshNodeNames = refreshNodeNames
M.refreshAllNodes = refreshAllNodes
M.recacheAllNodes = recacheAllNodes
M.closeAllInstances = closeAllInstances
M.openSceneTree = openSceneTree
M.debugNode = debugNode
return M