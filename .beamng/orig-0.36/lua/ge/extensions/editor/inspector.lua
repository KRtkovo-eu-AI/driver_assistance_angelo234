-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = "editor_inspector"
local ffi = require("ffi")
local guiInstancer = require("editor/api/guiInstancer")()
local valueInspector = require("editor/api/valueInspector")()
local objectHistoryActions = require("editor/api/objectHistoryActions")()
local imgui = ui_imgui

local inspectorWindowNamePrefix = "inspector"
local maxGroupCount = 500
local lockedInspectorColor = imgui.ImVec4(1, 0.8, 0, 1)
local differentValuesColor = imgui.ImVec4(1, 0.2, 0, 1)
local inspectorTypeHandlers = {}
local inspectorFieldModifiers = {}
local collapseGroups = {}
local arrayHeaderBgColor = imgui.ImVec4(0.04, 0.15, 0.1, 1)
local headerMenus = {
  {
    groupName = "Transform",
    open = false,
    pos = nil
  }
}
local groundCoverUVWindowName = "Ground Cover UV Editor"
local groundCoverUVTypeIndex = nil
local groundCoverUVInitialValue = ""
local groundCoverUVHandleColorIndex = 1
local groundCoverUVBgColorIndex = 1
local draggingGCUVHandleSq = nil
local groundCoverUVDragPos = nil
local groundCoverUVDragStartLocalDelta = imgui.ImVec2(0, 0)
local groundCoverUVHandlesOnDragStart = nil
local handleSquareSize = math.ceil(imgui.GetFontSize())
local groundCoverUVHandle_enum = {
  topLeft = 1,
  topCenter = 2,
  topRight = 3,
  middleLeft = 4,
  middleCenter = 5,
  middleRight = 6,
  bottomLeft = 7,
  bottomCenter = 8,
  bottomRight = 9
}

local groundCoverUVHandles = {
  [groundCoverUVHandle_enum.topLeft] = {imgui.ImVec2(0, 0), imgui.ImVec2(0, 0)},
  [groundCoverUVHandle_enum.topCenter] = {imgui.ImVec2(0, 0.5), imgui.ImVec2(0, 0)},
  [groundCoverUVHandle_enum.topRight] = {imgui.ImVec2(0, 1), imgui.ImVec2(0, 0)},

  [groundCoverUVHandle_enum.middleLeft] = {imgui.ImVec2(0, 0.5), imgui.ImVec2(0, 0)},
  [groundCoverUVHandle_enum.middleCenter] = {imgui.ImVec2(0.5, 0.5), imgui.ImVec2(0, 0)},
  [groundCoverUVHandle_enum.middleRight] = {imgui.ImVec2(1, 0.5), imgui.ImVec2(0, 0)},

  [groundCoverUVHandle_enum.bottomLeft] = {imgui.ImVec2(0, 1), imgui.ImVec2(0, 0)},
  [groundCoverUVHandle_enum.bottomCenter] = {imgui.ImVec2(0.5, 1), imgui.ImVec2(0, 0)},
  [groundCoverUVHandle_enum.bottomRight] = {imgui.ImVec2(1, 1), imgui.ImVec2(0, 0)},
}

local groundCoverUVal = imgui.FloatPtr(0.0)
local groundCoverVVal = imgui.FloatPtr(0.0)
local groundCoverWVal = imgui.FloatPtr(1.0)
local groundCoverHVal = imgui.FloatPtr(1.0)

local groundCoverUVHandleColors = {
  {
    name = "Color 1",
    color = imgui.ImVec4(1, 0, 0, 1)
  }, {
    name = "Color 2",
    color = imgui.ImVec4(0, 1, 0, 1)
  }, {
    name = "Color 3",
    color = imgui.ImVec4(0, 0, 1, 1)
  }
}

local groundCoverUVBgColors = {
  {
    name = "Color 1",
    color = imgui.ImVec4(0, 0, 0, 0)
  }, {
    name = "Color 2",
    color = imgui.ImVec4(0, 0, 0, 1)
  }, {
    name = "Color 3",
    color = imgui.ImVec4(1, 1, 1, 1)
  }
}

local function checkEditorDirtyFlag()
  if editor.getObjectSelection and not editor.dirty then
    for k, v in ipairs(editor.getObjectSelection()) do
      local obj = scenetree.findObjectById(v)
      -- we call inspectUpdate so the objects compute internal things and return true if inspector needs to be refreshed
      -- but inspector is refreshing its UI continuously, so that bool is not really needed, but the call is kept
      -- for that internal update some objects might need
      if obj and obj.inspectUpdate and (obj:inspectUpdate() or obj:isEditorDirty()) then
        -- something changed, dirty editor, needs save level
        editor.setDirty()
      end
    end
  end
end

local function createInspectorContext()
  return {
    newFieldName = imgui.ArrayChar(1024),
    matchedFilterStaticFields = true,
    inspectorCurrentFieldNames = {},
    firstObjectId = nil,
    firstObjectFieldValues = {},
    editEnded = imgui.BoolPtr(false),
    inputTextValue = imgui.ArrayChar(valueInspector.inputTextShortStringMaxSize),
    fields = {}
  }
end

-- this is the shared context, used by all inspector non locked instances
-- locked inspectors will use a custom context for each of the locked instances
local sharedCtx = createInspectorContext()

local function addInspectorInstance(selection)
  -- note: idx is a string key, not a number, because it gets serialized to json as key
  local idx = guiInstancer:addInstance()
  guiInstancer.instances[idx].selection = deepcopy(selection)
  guiInstancer.instances[idx].previousSelectedIds = {}
  guiInstancer.instances[idx].fieldNameFilter = imgui.ImGuiTextFilter()

  -- if this instance is locked, also create a new context for that inspector instance
  -- needed so that instance can handle the locked selection fields editing undisturbed by the
  -- current selection and other shared inspectors
  if selection then
    guiInstancer.instances[idx].ctx = createInspectorContext()
  end

  local wndName = inspectorWindowNamePrefix .. tostring(idx)
  editor.registerWindow(wndName, imgui.ImVec2(300, 500))
  editor.showWindow(wndName)
  return idx
end

local function openInspector()
  if not tableIsEmpty(guiInstancer.instances) then return end
  addInspectorInstance()
end

local function closeInspectorInstance(idx)
  local wndName = inspectorWindowNamePrefix .. tostring(idx)
  editor.unregisterWindow(wndName)
  guiInstancer:removeInstance(idx)
end

local function getInspectorInstances()
  return guiInstancer.instances
end

local function getInspectorTypeHandlers()
  return inspectorTypeHandlers
end

local function registerInspectorTypeHandler(typeName, guiCallback)
  inspectorTypeHandlers[typeName] = {
    typeName = typeName,
    guiCallback = guiCallback
  }
end

local function unregisterInspectorTypeHandler(typeName)
  inspectorTypeHandlers[typeName] = nil
end

local function registerInspectorFieldModifier(uniqueName, callback)
  inspectorFieldModifiers[uniqueName] = {
    callback = callback
  }
end

local function unregisterInspectorFieldModifier(uniqueName)
  inspectorFieldModifiers[uniqueName] = nil
end

local function setMultiSelectionFieldValue(selectedIds, fieldName, fieldValue, arrayIndex, editEnded)
  if editEnded == nil then editEnded = true end
  if editEnded then
    editor.history:beginTransaction("ChangeFieldValue")
    objectHistoryActions.changeObjectFieldWithUndo(selectedIds, fieldName, fieldValue, arrayIndex)
    editor.history:endTransaction()
  else
    for i = 1, tableSize(selectedIds) do
      editor.setFieldValue(selectedIds[i], fieldName, fieldValue, arrayIndex)
    end
  end
  if editEnded then
    editor.setDirty()
  end
end

local function setMultiSelectionFieldWithOldValues(selectedIds, fieldName, fieldValue, oldValues, arrayIndex, editEnded)
  if editEnded == nil then editEnded = true end
  if editEnded then
    objectHistoryActions.changeObjectFieldWithOldValues(selectedIds, fieldName, fieldValue, oldValues, arrayIndex)
  else
    for i = 1, tableSize(selectedIds) do
      editor.setFieldValue(selectedIds[i], fieldName, fieldValue, arrayIndex)
    end
  end
  if editEnded then
    editor.setDirty()
  end
end

local function setMultiSelectionDynamicFieldValue(selectedIds, fieldName, fieldValue, arrayIndex, editEnded)
  if editEnded == nil then editEnded = true end
  if editEnded then
    editor.history:beginTransaction("ChangeDynamicFieldValue")
    objectHistoryActions.changeObjectDynFieldWithUndo(selectedIds, fieldName, fieldValue, arrayIndex)
    editor.history:endTransaction()
  else
    for i = 1, tableSize(selectedIds) do
      editor.setDynamicFieldValue(selectedIds[i], fieldName, fieldValue, arrayIndex)
    end
  end
  if editEnded then
    editor.setDirty()
  end
end

-- callback for the value inspector copy paste menu
local function pasteFieldValue(fieldName, fieldValue, arrayIndex, customData)
  setMultiSelectionFieldValue(valueInspector.selectedIds, fieldName, fieldValue, arrayIndex)
  editor.updateObjectSelectionAxisGizmo()
end

local function resetFieldValue(fieldName, fieldType)
    local fieldVal = ""
    if fieldType == "Point3F" or fieldType == "vec3" or fieldType == "MatrixPosition" then
      if string.lower(fieldName) == "scale" then
        fieldVal = "1 1 1"
      else
        fieldVal = "0 0 0"
      end
    elseif fieldType == "MatrixRotation" then
      fieldVal = "0 0 0 0"
    elseif fieldType == "EulerRotation" then
      fieldVal = "0 0 0"
    else
      assert(false,"resetFieldValue not yet implemented for type " .. fieldType)
    end

    setMultiSelectionFieldValue(valueInspector.selectedIds, fieldName, fieldVal, 0)
    editor.updateObjectSelectionAxisGizmo()
end

local function getIndeterminateFlagsForFieldValues(fieldInfo, value1, value2)
  local flags = 0
  local elementCount = 1
  local fieldType = fieldInfo.type

  -- some values might be nil (not existent in the first object fields table for example)
  if nil == value1 then value1 = "" end
  if nil == value2 then value2 = "" end

  local valTbl1 = stringToTable(value1)
  local valTbl2 = stringToTable(value2)

  if fieldType == "Point2F" or fieldType == "vec2" or fieldType == "Point2I" then
    elementCount = 2
  elseif fieldType == "Point3F" or fieldType == "vec3" or fieldType == "MatrixPosition" or fieldType == "EulerRotation" then
    elementCount = 3
  elseif fieldType == "Point4F" or fieldType == "vec4" or fieldType == "EaseF" or fieldType == "RectF" or fieldType == "ColorF" or fieldType == "ColorI" then
    elementCount = 4
  end

  for i = 1, elementCount do
    if valTbl1[i] ~= valTbl2[i] then
      flags = bit.bor(flags, bit.lshift(1, i - 1))
    end
  end

  return flags
end

local function objectInspectorGui(inspectorInfo)
  valueInspector.selectedIds = nil

  -- if we have a locked inspector, ctx will be valid, else use the shared context
  local ctx = inspectorInfo.ctx or sharedCtx

  if inspectorInfo.selection then
    inspectorInfo.selection.object = editor.removeInvalidObjects(inspectorInfo.selection.object)
    valueInspector.selectedIds = inspectorInfo.selection.object
  else
    editor.selection.object = editor.removeInvalidObjects(editor.selection.object)
    valueInspector.selectedIds = editor.selection.object
    ctx.inspectorCurrentFieldNames = {}
  end

  if not valueInspector.selectedIds or 0 == tableSize(valueInspector.selectedIds) then
    imgui.Text("No selection")
    return
  end

  if tableSize(valueInspector.selectedIds) > 1 then
    imgui.Text(tostring(tableSize(valueInspector.selectedIds)) .. " selected object(s)")
  end

  if not setEqual(valueInspector.selectedIds, inspectorInfo.previousSelectedIds) then
    imgui.ClearActiveID()
    inspectorInfo.previousSelectedIds = valueInspector.selectedIds
  end

  local firstObj = scenetree.findObjectById(valueInspector.selectedIds[1])

  if firstObj then
    if firstObj.getClassName then
      valueInspector.selectionClassName = firstObj:getClassName()
    end
  else
    editor.logError("Object with this ID does not exists in the scene: " .. tostring(valueInspector.selectedIds[1]))
    return
  end

  ctx.firstObjectFieldValues = {}
  valueInspector.differentValuesFieldFlags = {}
  ctx.firstObjectId = valueInspector.selectedIds[1]

  -- find common fields in all the selected objects
  local commonFields = {}

  for fldName, field in pairs(ctx.fields) do
    if field.useCount == tableSize(valueInspector.selectedIds) then
      if nil == tableFindKey(commonFields, fldName) then
        commonFields[fldName] = field
      end
      if not field.isArray then
        ctx.firstObjectFieldValues[fldName] = editor.getFieldValue(valueInspector.selectedIds[1], fldName)
      end
    end
  end

  -- if we have multi selection, remove the name field, cant rename all objects at once the same name
  if tableSize(editor.selection.object) > 1 then commonFields["name"] = nil end

  -- make it our main fields list again
  ctx.fields = commonFields
  -- now find the fields with different values across the selected objects
  -- these will show as blank values in their field edit widgets
  local fieldVal = nil
  for key, val in pairs(ctx.fields) do
    -- we start at the second object, since the first one we keep as reference
    for i = 2, tableSize(valueInspector.selectedIds) do
      if not val.isArray then
        fieldVal = editor.getFieldValue(valueInspector.selectedIds[i], key)
        if ctx.firstObjectFieldValues[key] ~= fieldVal then
          if nil == valueInspector.differentValuesFieldFlags[key] then valueInspector.differentValuesFieldFlags[key] = 0 end
          valueInspector.differentValuesFieldFlags[key] = bit.bor(valueInspector.differentValuesFieldFlags[key], getIndeterminateFlagsForFieldValues(val, ctx.firstObjectFieldValues[key], fieldVal))
          --TODO: should break if all elements of the field are indeterminate
        end
      end
    end
  end

  -- if we got no fields, just return
  if not ctx.fields then
    return
  end

  local sortedFields = {}

  -- set the sorted fields array and sort the array fields
  for _, field in pairs(ctx.fields) do
    if field.type == "beginArray" then
      -- sort the array fields by ID in a new array table
      field.sortedFields = {}
      for _, fld in pairs(field.fields) do table.insert(field.sortedFields, fld) end
      table.sort(field.sortedFields, function(a, b) return a.id < b.id end)
    end
    table.insert(sortedFields, field)
  end

  -- sort by id value (order of declaration in the C++ vector, the id is the index in the fields vector)
  table.sort(sortedFields, function(a, b) return a.id < b.id end)

  local groupedSortedFields = {}

  if editor.uiInputSearchTextFilter("##fieldNameSearchFilter", inspectorInfo.fieldNameFilter, 200, nil, ctx.editEnded) then
    if ffi.string(imgui.TextFilter_GetInputBuf(inspectorInfo.fieldNameFilter)) == "" then
      imgui.ImGuiTextFilter_Clear(inspectorInfo.fieldNameFilter)
    end
  end

  -- put the fields in a grouped table
  local groupIndex = 1
  local groupIndexLUT = {} -- a look up table with the order index for each group

  for i = 1, tableSize(sortedFields) do
    local val = sortedFields[i]

    if val then
      for k, v in pairs(inspectorFieldModifiers) do
        if v.callback then
          local ret = v.callback(val, valueInspector.selectionClassName)
          if ret then val = ret end
        end
      end
    end

    if val and not val.hideInInspector then
      -- only gather field names for unlocked inspectors
      if not inspectorInfo.selection then
        ctx.inspectorCurrentFieldNames[val.name] = true
      end
      -- add group table if not existing in the LUT
      if val.groupName and not groupIndexLUT[val.groupName] then
        groupIndexLUT[val.groupName] = groupIndex
        groupedSortedFields[groupIndex] = {
          groupName = val.groupName,
          isExpanded = true, -- TODO: get from val.groupExpand from C++?
          fields = {}
        }
        groupIndex = groupIndex + 1
      end
      if groupIndexLUT[val.groupName] then
        -- render colored label indicators for Transform group
        if val.groupName == "Transform" then
          val.coloredLabelIndicator = true
        end
        table.insert(groupedSortedFields[groupIndexLUT[val.groupName]].fields, val)
      end
    end
  end

  -- general and transform groups come first
  local general = groupedSortedFields[groupIndexLUT["Ungrouped"]] or {}
  local xform = groupedSortedFields[groupIndexLUT["Transform"]] or {}
  local fieldIndent = 15

  local function displayFields(fields)
    for _, val in ipairs(fields) do
      -- simple field
      if not val.isArray and not val.hidden then
        local fieldLabel = val.name
        local valChanger = editor.findCustomFieldLabelChanger(val.name, valueInspector.selectionClassName)

        if valChanger then
          fieldLabel = valChanger.callback(val.name, valueInspector.selectionClassName)
        end

        if val.elementCount == 1 then
          val.value = editor.getFieldValue(valueInspector.selectedIds[1], val.name, 0)
          valueInspector:valueEditorGui(val.name, val.value or "", 0, fieldLabel, val.fieldDocs, val.type, val.typeName, val, pasteFieldValue, nil, valueInspector.differentValuesFieldFlags[val.name] or 0)
        else
          local customFieldEditor = editor.findCustomFieldEditor(val.name, valueInspector.selectionClassName)
          if customFieldEditor and customFieldEditor.useArray then
            valueInspector:valueEditorGui(val.name, val.value or "", 0, fieldLabel, val.fieldDocs, val.type, val.typeName, val, pasteFieldValue, nil, valueInspector.differentValuesFieldFlags[val.name] or 0)
          else
            local nodeFlags = imgui.TreeNodeFlags_DefaultClosed
            imgui.PushStyleColor2(imgui.Col_Header, arrayHeaderBgColor)
            if imgui.CollapsingHeader1(val.name, nodeFlags) then
              imgui.Indent(fieldIndent)
              for i = 0, val.elementCount - 1 do
                local value = editor.getFieldValue(valueInspector.selectedIds[#valueInspector.selectedIds], val.name, i)
                valueInspector:valueEditorGui(val.name, value or "", i, fieldLabel .. "["..tostring(i).."]", val.fieldDocs, val.type, val.typeName, val, pasteFieldValue, nil, valueInspector.differentValuesFieldFlags[val.name] or 0)
              end
              imgui.Unindent(fieldIndent)
              imgui.Separator()
            end
            imgui.PopStyleColor()
          end
        end
      -- if its and array of fields
      elseif val.isArray then
        local fieldLabel = val.arrayName
        local valChanger = editor.findCustomFieldLabelChanger(val.arrayName, valueInspector.selectionClassName)

        if valChanger then
          fieldLabel = valChanger.callback(val.arrayName, valueInspector.selectionClassName)
        end

        local nodeFlags = imgui.TreeNodeFlags_DefaultClosed
        imgui.PushStyleColor2(imgui.Col_Header, arrayHeaderBgColor)
        if imgui.CollapsingHeader1(fieldLabel, nodeFlags) then
          imgui.Indent(fieldIndent)
          for i = 0, val.elementCount - 1 do
            imgui.PushID1(val.arrayName .. "_ARRAY_ITEMS_" .. i)
            if imgui.CollapsingHeader1("[" .. tostring(i) .. "]", nodeFlags) then
              for _, arrayField in ipairs(val.sortedFields) do
                if not arrayField.hidden then
                  arrayField.value = editor.getFieldValue(valueInspector.selectedIds[#valueInspector.selectedIds], arrayField.name, i)
                  if arrayField.name == "billboardUVs" and arrayField.typeName == "TypeRectUV" then
                    arrayField.arrayIndex = i
                    arrayField.objID = ctx.firstObjectId
                  end
                  valueInspector:valueEditorGui(arrayField.name, arrayField.value or "", i, arrayField.name, arrayField.fieldDocs, arrayField.type, arrayField.typeName, arrayField, pasteFieldValue, nil, valueInspector.differentValuesFieldFlags[arrayField.name] or 0)
                end
              end
            end
            imgui.PopID()
          end
          imgui.Unindent(fieldIndent)
          imgui.Separator()
        end
        imgui.PopStyleColor()
      end
    end
  end

  local function setHeaderMenu(groupName)
    for _, headerMenu in ipairs(headerMenus) do
      if string.lower(headerMenu.groupName) == string.lower(groupName) then
        if imgui.Button("...") then
          if headerMenu.open then
            headerMenu.open = false
          else
            headerMenu.open = true
          end
          headerMenu.pos = imgui.ImVec2(imgui.GetMousePos().x - 150 * editor.getPreference("ui.general.scale"), imgui.GetMousePos().y + 10)
        end
        break
      end
    end
  end

  local function getFieldType(fieldName, fields)
    for _, field in ipairs(fields) do
      if string.lower(field.name) == string.lower(fieldName) then
        return field.type
      end
    end
    return nil
  end

  local function headerMenu(groupName, fields)
    local menu = nil
    local menuFound = false
    for _, headerMenu in ipairs(headerMenus) do
      if string.lower(headerMenu.groupName) == string.lower(groupName) then
        menuFound = true
        menu = headerMenu
        break
      end
    end
    if not menuFound then return end
    if menu.open then
      imgui.SetNextWindowPos(menu.pos)
      imgui.Begin(groupName.."HeaderMenu", nil, imgui.WindowFlags_NoCollapse + imgui.WindowFlags_AlwaysAutoResize + imgui.WindowFlags_NoResize + imgui.WindowFlags_NoTitleBar)
      if groupName == "Transform" then
        local posFieldType = getFieldType("position", fields)
        if not posFieldType then
          imgui.BeginDisabled()
        end
        if imgui.Button("Reset Position") then
          resetFieldValue("position", posFieldType)
          menu.open = false
        end
        if not posFieldType then
          imgui.EndDisabled()
        end

        local rotFieldType = getFieldType("rotation", fields)
        if not rotFieldType then
          imgui.BeginDisabled()
        end
        if imgui.Button("Reset Rotation") then
          resetFieldValue("rotation", rotFieldType)
          menu.open = false
        end
        if not rotFieldType then
          imgui.EndDisabled()
        end

        local scaleFieldType = getFieldType("scale", fields)
        if not scaleFieldType then
          imgui.BeginDisabled()
        end
        if imgui.Button("Reset Scale") then
          resetFieldValue("scale", scaleFieldType)
          menu.open = false
        end
        if not scaleFieldType then
          imgui.EndDisabled()
        end

        if not rotFieldType or not scaleFieldType then
          imgui.BeginDisabled()
        end
        if imgui.Button("Reset Rotation & Scale") then
          resetFieldValue("rotation", rotFieldType)
          resetFieldValue("scale", scaleFieldType)
          menu.open = false
        end
        if not rotFieldType or not scaleFieldType then
          imgui.EndDisabled()
        end
      end
      if not imgui.IsWindowFocused() then
        menu.open = false
      end
      imgui.End()
    end
  end

  local function collapsingHeaderMenu(groupName, fields)
    local groupHeaderMenuFound = false
    for _, val in ipairs(headerMenus) do
      if string.lower(val.groupName) == string.lower(groupName) then
        groupHeaderMenuFound = true
        break
      end
    end
    if not groupHeaderMenuFound then
      return
    end
    imgui.SameLine(imgui.GetWindowWidth() - 40 * editor.getPreference("ui.general.scale"));
    imgui.SetItemAllowOverlap()
    setHeaderMenu(groupName)
    headerMenu(groupName, fields)
  end

  local function displayGroup(groupName, fields, ctx)
    local nodeFlags = imgui.TreeNodeFlags_DefaultOpen
    if not fields then return end
    -- check if any of its fields are filtered
    local passFilter = false
    for _, val in ipairs(fields) do
      if imgui.ImGuiTextFilter_PassFilter(inspectorInfo.fieldNameFilter, val.name) then
        passFilter = true
        val.hidden = false
      else
        val.hidden = true
      end
    end
    if not passFilter then return end
    ctx.matchedFilterStaticFields = true
    local res = imgui.CollapsingHeader1(groupName, nodeFlags)
    collapsingHeaderMenu(groupName, fields)
    if res then
      displayFields(fields)
    end
  end

  --
  -- Static fields
  --
  ctx.matchedFilterStaticFields = true
  -- a bit of info about the selection
  if general and general.fields then
    if valueInspector.selectedIds and #valueInspector.selectedIds == 1 and valueInspector.selectedIds[1] ~= 0 then
      local firstId = valueInspector.selectedIds[1]
      local obj = scenetree.findObjectById(firstId)
      if obj then
        local textColor = imgui.GetStyleColorVec4(imgui.Col_Text)
        imgui.TextUnformatted("Class:") imgui.SameLine() imgui.TextColored(textColor, valueInspector.selectionClassName)

        if imgui.GetIO().KeyCtrl then
          imgui.tooltip(obj:getClassDocString())
        end

        if #valueInspector.selectedIds == 1 then
          imgui.SameLine()
          imgui.Text("    ")
          imgui.SameLine()
          imgui.TextUnformatted("ID:") imgui.SameLine() imgui.TextColored(textColor, tostring(obj:getId()))
          imgui.SameLine()
          if imgui.Button("Copy ID") then
            setClipboard(tostring(obj:getId()))
          end
          imgui.SameLine()

          local grp = obj:getGroup()
          if grp then
            imgui.TextUnformatted("Parent:") imgui.SameLine() imgui.TextColored(textColor, tostring(grp:getName()))
          end
        end
      end
    end
    displayGroup("General", general.fields, ctx)
  end

  if xform and xform.fields then
    displayGroup("Transform", xform.fields, ctx)
  end

  -- display the groups and their fields
  for _, group in ipairs(groupedSortedFields) do
    if group ~= general and group ~= xform then
      displayGroup(group.groupName, group.fields, ctx)
    end
  end

  if not ctx.matchedFilterStaticFields then
    imgui.PushStyleColor2(imgui.Col_Text, imgui.ImVec4(1, 0.5, 0, 1))
    imgui.Text("<No search matches>")
    imgui.PopStyleColor()
  end

  --
  -- Dynamic fields
  --
  local dynFields = {}
  -- if only one object selected, then just get its dynamic fields
  if #valueInspector.selectedIds == 1 then
    if ctx.firstObjectId == 0 then
      dynFields = {}
    else
      dynFields = editor.getDynamicFields(ctx.firstObjectId)
    end
  else
    -- if multiselection, then find all the common dynamic fields to the selection
    local dynFieldUsage = {}
    -- count the usage for each dynamic field of each object
    for i = 1, #valueInspector.selectedIds do
      local objDynFields = editor.getDynamicFields(valueInspector.selectedIds[i])
      for j = 1, #objDynFields do
        local fieldName = objDynFields[j]
        if not dynFieldUsage[fieldName] then
          dynFieldUsage[fieldName] = 1
        else
          dynFieldUsage[fieldName] = dynFieldUsage[fieldName] + 1
        end
      end
    end
    -- add the dynamic field whom usage is equal to the selection count
    -- meaning that is used by every object in the selection so its common to all
    -- otherwise we skip it, since it would not make sense when not common
    for key, val in pairs(dynFieldUsage) do
      if val == #valueInspector.selectedIds then
        table.insert(dynFields, key)
      end
    end
  end

  -- show the dynamic fields editors
  if dynFields ~= nil and imgui.CollapsingHeader1("Dynamic Fields") then
    local arrayIndex = 0
    -- if multiselection and no common dynamic fields
    if #dynFields == 0 and #valueInspector.selectedIds > 1 then
        imgui.TextUnformatted("No common dynamic fields")
    else
      local fieldValue = ""
      local passedFilter = false
      for i = 1, #dynFields do
        if imgui.ImGuiTextFilter_PassFilter(inspectorInfo.fieldNameFilter, dynFields[i]) then
          passedFilter = true
          fieldValue = editor.getFieldValue(ctx.firstObjectId, dynFields[i])
          if fieldValue ~= nil then
            ffi.copy(ctx.inputTextValue, fieldValue)
          end
          imgui.PushID1("FIELDS_COL")
          imgui.Columns(2, "FieldsColumn")
          imgui.Text(dynFields[i])
          imgui.NextColumn()
          local fieldNameId = "##" .. dynFields[i]
          -- if dynamic field value is changed and the value it's not empty string then update it
          if editor.uiInputText(fieldNameId, ctx.inputTextValue, imgui.ArraySize(ctx.inputTextValue), nil, nil, nil, ctx.editEnded) and ctx.editEnded[0] and ffi.string(ctx.inputTextValue) ~= "" then
            fieldValue = ffi.string(ctx.inputTextValue)
            setMultiSelectionDynamicFieldValue(valueInspector.selectedIds, dynFields[i], fieldValue, arrayIndex)
          end
          imgui.SameLine()
          imgui.PushID4(i)
          -- delete dynamic field button
          if imgui.Button("X") then
            -- just set to empty string will delete it
            setMultiSelectionDynamicFieldValue(valueInspector.selectedIds, dynFields[i], "", arrayIndex)
          end
          imgui.PopID()
          imgui.Columns(1)
          imgui.PopID()
        end
      end
      if #dynFields > 0 and not passedFilter then
        imgui.PushStyleColor2(imgui.Col_Text, imgui.ImVec4(1, 0.5, 0, 1))
        imgui.Text("<No search matches>")
        imgui.PopStyleColor()
      end
    end

    local wantsToAddField = false

    imgui.Text("Add new field named:")
    if imgui.InputText("##newDynField", ctx.newFieldName, imgui.ArraySize(ctx.newFieldName), imgui.InputTextFlags_EnterReturnsTrue) then
      wantsToAddField = true
    end
    imgui.SameLine()
    if imgui.Button("Add") then
      wantsToAddField = true
    end

    if wantsToAddField then
      local fieldValue = ffi.string(ctx.newFieldName)
      setMultiSelectionDynamicFieldValue(valueInspector.selectedIds, fieldValue, "0", arrayIndex)
      ffi.copy(ctx.newFieldName, "")
    end
  end
end

local function inspectorHasField(fieldName)
  return sharedCtx.inspectorCurrentFieldNames[fieldName] ~= nil
end

local function registerApi()
  editor.addInspectorInstance = addInspectorInstance
  editor.closeInspectorInstance = closeInspectorInstance
  editor.getInspectorInstances = getInspectorInstances
  editor.registerInspectorTypeHandler = registerInspectorTypeHandler
  editor.unregisterInspectorTypeHandler = unregisterInspectorTypeHandler
  editor.getInspectorTypeHandlers = getInspectorTypeHandlers
  editor.registerInspectorFieldModifier = registerInspectorFieldModifier
  editor.unregisterInspectorFieldModifier = unregisterInspectorFieldModifier
  editor.inspectorHasField = inspectorHasField
end

local function onExtensionLoaded()
  for i = 1, maxGroupCount do
    table.insert(collapseGroups, i, imgui.BoolPtr(false))
  end

  registerApi()
end

local function drawUVHandle(imageStartCursorPos, imageStartAbsCursorPos, imageSize)
  local u_pixel = groundCoverUVal[0] * imageSize.x
  local v_pixel = groundCoverVVal[0] * imageSize.y
  local width_pixel = groundCoverWVal[0] * imageSize.x
  local height_pixel = groundCoverHVal[0] * imageSize.y
  local handleColor = imgui.GetColorU322(groundCoverUVHandleColors[groundCoverUVHandleColorIndex].color)

  local p_Min = imgui.ImVec2(imgui.GetWindowPos().x + imageStartCursorPos.x + u_pixel + imgui.GetStyle().ChildBorderSize, imgui.GetWindowPos().y + imageStartCursorPos.y + v_pixel + imgui.GetStyle().ChildBorderSize)
  local p_Max = imgui.ImVec2(p_Min.x + width_pixel, p_Min.y + height_pixel)

  imgui.ImDrawList_AddRect(imgui.GetWindowDrawList(), p_Min, p_Max, handleColor, nil, nil, 2)

  local p1_HandleSqTopLeft = imgui.ImVec2(p_Min.x, p_Min.y)
  local p2_HandleSqTopLeft = imgui.ImVec2(p_Min.x + handleSquareSize, p_Min.y + handleSquareSize)
  groundCoverUVHandles[groundCoverUVHandle_enum.topLeft] = {p1_HandleSqTopLeft, p2_HandleSqTopLeft}

  local p1_HandleSqTopMiddle = imgui.ImVec2(p_Min.x + width_pixel/2 - handleSquareSize/2, p_Min.y)
  local p2_HandleSqTopMiddle = imgui.ImVec2(p_Min.x + width_pixel/2 + handleSquareSize/2, p_Min.y + handleSquareSize)
  groundCoverUVHandles[groundCoverUVHandle_enum.topCenter] = {p1_HandleSqTopMiddle, p2_HandleSqTopMiddle}

  local p1_HandleSqTopRight = imgui.ImVec2(p_Min.x + width_pixel - handleSquareSize, p_Min.y)
  local p2_HandleSqTopRight = imgui.ImVec2(p_Min.x + width_pixel, p_Min.y + handleSquareSize)
  groundCoverUVHandles[groundCoverUVHandle_enum.topRight] = {p1_HandleSqTopRight, p2_HandleSqTopRight}

  local p1_HandleSqMiddleLeft = imgui.ImVec2(p_Min.x, p_Min.y + height_pixel/2 - handleSquareSize/2)
  local p2_HandleSqMiddleLeft = imgui.ImVec2(p_Min.x + handleSquareSize, p_Min.y + height_pixel/2 + handleSquareSize/2)
  groundCoverUVHandles[groundCoverUVHandle_enum.middleLeft] = {p1_HandleSqMiddleLeft, p2_HandleSqMiddleLeft}

  local p1_HandleSqCenter = imgui.ImVec2(p_Min.x + width_pixel/2 - handleSquareSize/2, p_Min.y + height_pixel/2 - handleSquareSize/2)
  local p2_HandleSqCenter = imgui.ImVec2(p_Min.x + width_pixel/2 + handleSquareSize/2, p_Min.y + height_pixel/2 + handleSquareSize/2)
  groundCoverUVHandles[groundCoverUVHandle_enum.middleCenter] = {p1_HandleSqCenter, p2_HandleSqCenter}

  local p1_HandleSqMiddleRight = imgui.ImVec2(p_Min.x + width_pixel - handleSquareSize, p_Min.y + height_pixel/2 - handleSquareSize/2)
  local p2_HandleSqMiddleRight = imgui.ImVec2(p_Min.x + width_pixel, p_Min.y + height_pixel/2 + handleSquareSize/2)
  groundCoverUVHandles[groundCoverUVHandle_enum.middleRight] = {p1_HandleSqMiddleRight, p2_HandleSqMiddleRight}

  local p1_HandleSqBottomLeft = imgui.ImVec2(p_Min.x, p_Min.y + height_pixel - handleSquareSize)
  local p2_HandleRectBottomLeft = imgui.ImVec2(p_Min.x + handleSquareSize, p_Min.y + height_pixel)
  groundCoverUVHandles[groundCoverUVHandle_enum.bottomLeft] = {p1_HandleSqBottomLeft, p2_HandleRectBottomLeft}

  local p1_HandleSqBottomMiddle = imgui.ImVec2(p_Min.x + width_pixel/2 - handleSquareSize/2, p_Min.y + height_pixel - handleSquareSize)
  local p2_HandleSqBottomMiddle = imgui.ImVec2(p_Min.x + width_pixel/2 + handleSquareSize/2, p_Min.y + height_pixel)
  groundCoverUVHandles[groundCoverUVHandle_enum.bottomCenter] = {p1_HandleSqBottomMiddle, p2_HandleSqBottomMiddle}

  local p1_HandleSqBottomRight = imgui.ImVec2(p_Min.x + width_pixel - handleSquareSize, p_Min.y + height_pixel - handleSquareSize)
  local p2_HandleSqBottomRight = imgui.ImVec2(p_Min.x + width_pixel, p_Min.y + height_pixel)
  groundCoverUVHandles[groundCoverUVHandle_enum.bottomRight] = {p1_HandleSqBottomRight, p2_HandleSqBottomRight}

  imgui.ImDrawList_AddRectFilled(imgui.GetWindowDrawList(), p1_HandleSqTopLeft, p2_HandleSqTopLeft, handleColor)
  imgui.ImDrawList_AddRectFilled(imgui.GetWindowDrawList(), p1_HandleSqTopMiddle, p2_HandleSqTopMiddle, handleColor)
  imgui.ImDrawList_AddRectFilled(imgui.GetWindowDrawList(), p1_HandleSqTopRight, p2_HandleSqTopRight, handleColor)

  imgui.ImDrawList_AddRectFilled(imgui.GetWindowDrawList(), p1_HandleSqMiddleLeft, p2_HandleSqMiddleLeft, handleColor)
  imgui.ImDrawList_AddRectFilled(imgui.GetWindowDrawList(), p1_HandleSqCenter, p2_HandleSqCenter, handleColor)
  imgui.ImDrawList_AddRectFilled(imgui.GetWindowDrawList(), p1_HandleSqMiddleRight, p2_HandleSqMiddleRight, handleColor)

  imgui.ImDrawList_AddRectFilled(imgui.GetWindowDrawList(), p1_HandleSqBottomLeft, p2_HandleRectBottomLeft, handleColor)
  imgui.ImDrawList_AddRectFilled(imgui.GetWindowDrawList(), p1_HandleSqBottomMiddle, p2_HandleSqBottomMiddle, handleColor)
  imgui.ImDrawList_AddRectFilled(imgui.GetWindowDrawList(), p1_HandleSqBottomRight, p2_HandleSqBottomRight, handleColor)

  local updateGroundCoverUVHandle = function ()
    groundCoverUVDragPos = imgui.GetMousePos()
    local xConstraint = nil
    local yConstraint = nil
    if draggingGCUVHandleSq == groundCoverUVHandle_enum.topLeft then
      xConstraint = {imageStartAbsCursorPos.x, groundCoverUVHandlesOnDragStart[groundCoverUVHandle_enum.bottomRight][1].x - 2*handleSquareSize}
      yConstraint = {imageStartAbsCursorPos.y, groundCoverUVHandlesOnDragStart[groundCoverUVHandle_enum.bottomRight][1].y - 2*handleSquareSize}
    elseif draggingGCUVHandleSq == groundCoverUVHandle_enum.topCenter then
      xConstraint = {nil, nil}
      yConstraint = {imageStartAbsCursorPos.y, groundCoverUVHandlesOnDragStart[groundCoverUVHandle_enum.bottomRight][1].y - 2*handleSquareSize}
    elseif draggingGCUVHandleSq == groundCoverUVHandle_enum.topRight then
      xConstraint = {groundCoverUVHandlesOnDragStart[groundCoverUVHandle_enum.topLeft][1].x + 2*handleSquareSize, imageStartAbsCursorPos.x + imageSize.x - handleSquareSize + groundCoverUVDragStartLocalDelta.x + 2}
      yConstraint = {imageStartAbsCursorPos.y, groundCoverUVHandlesOnDragStart[groundCoverUVHandle_enum.bottomRight][1].y - 2*handleSquareSize + groundCoverUVDragStartLocalDelta.y}
    elseif draggingGCUVHandleSq == groundCoverUVHandle_enum.middleLeft then
      xConstraint = {imageStartAbsCursorPos.x, groundCoverUVHandlesOnDragStart[groundCoverUVHandle_enum.topRight][1].x - 2*handleSquareSize}
      yConstraint = {nil, nil}
    elseif draggingGCUVHandleSq == groundCoverUVHandle_enum.middleCenter then
      local width = groundCoverWVal[0] * imageSize.x
      local height = groundCoverHVal[0] * imageSize.y
      local x1 = imageStartAbsCursorPos.x + width/2 + groundCoverUVDragStartLocalDelta.x
      local x2 = imageStartAbsCursorPos.x + imageSize.x - width/2 + groundCoverUVDragStartLocalDelta.x
      local y1 = imageStartAbsCursorPos.y + height/2 + groundCoverUVDragStartLocalDelta.y
      local y2 = imageStartAbsCursorPos.y + imageSize.x - height/2 + groundCoverUVDragStartLocalDelta.y
      xConstraint = {x1, x2}
      yConstraint = {y1, y2}
    elseif draggingGCUVHandleSq == groundCoverUVHandle_enum.middleRight then
      xConstraint = {groundCoverUVHandlesOnDragStart[groundCoverUVHandle_enum.topLeft][2].x + 2*handleSquareSize, imageStartAbsCursorPos.x + imageSize.x + 1}
      yConstraint = {nil, nil}
    elseif draggingGCUVHandleSq == groundCoverUVHandle_enum.bottomLeft then
      xConstraint = {imageStartAbsCursorPos.x, groundCoverUVHandlesOnDragStart[groundCoverUVHandle_enum.bottomRight][1].x - 2*handleSquareSize}
      yConstraint = {groundCoverUVHandlesOnDragStart[groundCoverUVHandle_enum.topLeft][2].y + 2*handleSquareSize, imageStartAbsCursorPos.y + imageSize.y + 1}
    elseif draggingGCUVHandleSq == groundCoverUVHandle_enum.bottomCenter then
      xConstraint = {nil, nil}
      yConstraint = {groundCoverUVHandlesOnDragStart[groundCoverUVHandle_enum.topRight][2].y + 2*handleSquareSize, imageStartAbsCursorPos.y - handleSquareSize + imageSize.y + 1}
    elseif draggingGCUVHandleSq == groundCoverUVHandle_enum.bottomRight then
      xConstraint = {groundCoverUVHandlesOnDragStart[groundCoverUVHandle_enum.topLeft][2].x + 2*handleSquareSize, imageStartAbsCursorPos.x + imageSize.x + 2}
      yConstraint = {groundCoverUVHandlesOnDragStart[groundCoverUVHandle_enum.topLeft][2].y + 2*handleSquareSize, imageStartAbsCursorPos.y + imageSize.y + 1}
    end
    local constraints = {xConstraint, yConstraint}
    local sqNewPos = imgui.GetMousePos()

    local checkBoundaries = function(newPos, uvHandleConstraints)
      local xConstraint = uvHandleConstraints[1]
      local yConstraint = uvHandleConstraints[2]

      if xConstraint[1] ~= xConstraint[2] then
        if newPos.x < xConstraint[1] then
          newPos.x = xConstraint[1]
        elseif newPos.x > xConstraint[2] then
          newPos.x = xConstraint[2]
        end
      elseif xConstraint[1] and xConstraint[2] and xConstraint[1] == xConstraint[2] then
        newPos.x = math.huge
      end

      if yConstraint[1] ~= yConstraint[2] then
        if newPos.y < yConstraint[1] then
          newPos.y = yConstraint[1]
        elseif newPos.y > yConstraint[2] then
          newPos.y = yConstraint[2]
        end
      elseif yConstraint[1] and yConstraint[2] and yConstraint[1] == yConstraint[2] then
        newPos.y = math.huge
      end
    end

    if draggingGCUVHandleSq == groundCoverUVHandle_enum.topLeft then
      sqNewPos.x = sqNewPos.x - groundCoverUVDragStartLocalDelta.x
      sqNewPos.y = sqNewPos.y - groundCoverUVDragStartLocalDelta.y
      checkBoundaries(sqNewPos, constraints)
      groundCoverUVHandles[draggingGCUVHandleSq][1] = sqNewPos
      local relNewPos = imgui.ImVec2(sqNewPos.x - imageStartAbsCursorPos.x, sqNewPos.y - imageStartAbsCursorPos.y)
      groundCoverUVal[0] = (relNewPos.x)/imageSize.x
      groundCoverVVal[0] = (relNewPos.y)/imageSize.y
      local width =  groundCoverUVHandlesOnDragStart[groundCoverUVHandle_enum.topRight][2].x - sqNewPos.x - 1
      local height =  groundCoverUVHandlesOnDragStart[groundCoverUVHandle_enum.bottomRight][2].y - sqNewPos.y - 1
      groundCoverWVal[0] = width/imageSize.x
      groundCoverHVal[0] = height/imageSize.y
    elseif draggingGCUVHandleSq == groundCoverUVHandle_enum.topCenter then
      sqNewPos.y = sqNewPos.y - groundCoverUVDragStartLocalDelta.y
      sqNewPos.x = sqNewPos.x + groundCoverUVDragStartLocalDelta.x
      checkBoundaries(sqNewPos, constraints)
      groundCoverUVHandles[draggingGCUVHandleSq][1] = sqNewPos
      local relNewPos = imgui.ImVec2(sqNewPos.x - imageStartAbsCursorPos.x, sqNewPos.y - imageStartAbsCursorPos.y)
      groundCoverVVal[0] = relNewPos.y/imageSize.y
      local height = groundCoverUVHandlesOnDragStart[groundCoverUVHandle_enum.bottomRight][2].y - sqNewPos.y - 1
      groundCoverHVal[0] = height/imageSize.y
    elseif draggingGCUVHandleSq == groundCoverUVHandle_enum.topRight then
      sqNewPos.y = sqNewPos.y - groundCoverUVDragStartLocalDelta.y
      sqNewPos.x = sqNewPos.x - groundCoverUVDragStartLocalDelta.x
      checkBoundaries(sqNewPos, constraints)
      groundCoverUVHandles[draggingGCUVHandleSq][1] = sqNewPos
      local relNewPos = imgui.ImVec2(sqNewPos.x - imageStartAbsCursorPos.x, sqNewPos.y - imageStartAbsCursorPos.y)
      groundCoverVVal[0] = relNewPos.y/imageSize.y
      local width = sqNewPos.x + handleSquareSize - groundCoverUVDragStartLocalDelta.x - groundCoverUVHandlesOnDragStart[groundCoverUVHandle_enum.topLeft][1].x - 1
      local height = groundCoverUVHandlesOnDragStart[groundCoverUVHandle_enum.bottomRight][2].y - sqNewPos.y - 1
      groundCoverWVal[0] = width/imageSize.x
      groundCoverHVal[0] = height/imageSize.y
    elseif draggingGCUVHandleSq == groundCoverUVHandle_enum.middleLeft then
      sqNewPos.y = sqNewPos.y - groundCoverUVDragStartLocalDelta.y
      sqNewPos.x = sqNewPos.x - groundCoverUVDragStartLocalDelta.x
      checkBoundaries(sqNewPos, constraints)
      groundCoverUVHandles[draggingGCUVHandleSq][1] = sqNewPos
      local relNewPos = imgui.ImVec2(sqNewPos.x - imageStartAbsCursorPos.x, sqNewPos.y - imageStartAbsCursorPos.y)
      groundCoverUVal[0] = relNewPos.x/imageSize.x
      local width = groundCoverUVHandlesOnDragStart[groundCoverUVHandle_enum.topRight][2].x - sqNewPos.x - 1
      groundCoverWVal[0] = width/imageSize.x
    elseif draggingGCUVHandleSq == groundCoverUVHandle_enum.middleCenter then
      sqNewPos.y = sqNewPos.y + groundCoverUVDragStartLocalDelta.y
      sqNewPos.x = sqNewPos.x + groundCoverUVDragStartLocalDelta.x
      checkBoundaries(sqNewPos, constraints)
      groundCoverUVHandles[draggingGCUVHandleSq][1].x = (sqNewPos.x == math.huge and groundCoverUVHandles[draggingGCUVHandleSq][1].x or sqNewPos.x)
      groundCoverUVHandles[draggingGCUVHandleSq][1].y = (sqNewPos.y == math.huge and groundCoverUVHandles[draggingGCUVHandleSq][1].y or sqNewPos.y)
      local relNewPos = imgui.ImVec2(sqNewPos.x - imageStartAbsCursorPos.x, sqNewPos.y - imageStartAbsCursorPos.y)
      if sqNewPos.x ~= math.huge then
        local width = groundCoverWVal[0]*imageSize.x
        local uValue = (relNewPos.x - (width)/2 - groundCoverUVDragStartLocalDelta.x)/imageSize.x
        groundCoverUVal[0] = uValue < 0 and 0 or uValue
      end
      if sqNewPos.y ~= math.huge then
        local height = groundCoverHVal[0]*imageSize.y
        local vValue = (relNewPos.y - (height)/2 - groundCoverUVDragStartLocalDelta.y)/imageSize.y
        groundCoverVVal[0] = vValue < 0 and 0 or vValue
      end
    elseif draggingGCUVHandleSq == groundCoverUVHandle_enum.middleRight then
      sqNewPos.x = sqNewPos.x - groundCoverUVDragStartLocalDelta.x
      checkBoundaries(sqNewPos, constraints)
      groundCoverUVHandles[draggingGCUVHandleSq][1] = sqNewPos
      local width = sqNewPos.x - groundCoverUVHandlesOnDragStart[groundCoverUVHandle_enum.topLeft][1].x
      groundCoverWVal[0] = width/imageSize.x
    elseif draggingGCUVHandleSq == groundCoverUVHandle_enum.bottomLeft then
      sqNewPos.y = sqNewPos.y + groundCoverUVDragStartLocalDelta.x
      sqNewPos.x = sqNewPos.x - groundCoverUVDragStartLocalDelta.x
      checkBoundaries(sqNewPos, constraints)
      groundCoverUVHandles[draggingGCUVHandleSq][1] = sqNewPos
      local relNewPos = imgui.ImVec2(sqNewPos.x - imageStartAbsCursorPos.x, sqNewPos.y - imageStartAbsCursorPos.y)
      groundCoverUVal[0] = relNewPos.x/imageSize.x
      local width = groundCoverUVHandlesOnDragStart[groundCoverUVHandle_enum.topRight][2].x - sqNewPos.x - 1
      local height = sqNewPos.y - groundCoverUVHandlesOnDragStart[groundCoverUVHandle_enum.topRight][1].y
      groundCoverWVal[0] = width/imageSize.x
      groundCoverHVal[0] = height/imageSize.y
    elseif draggingGCUVHandleSq == groundCoverUVHandle_enum.bottomCenter then
      sqNewPos.y = sqNewPos.y - groundCoverUVDragStartLocalDelta.y
      sqNewPos.x = sqNewPos.x - groundCoverUVDragStartLocalDelta.x
      checkBoundaries(sqNewPos, constraints)
      groundCoverUVHandles[draggingGCUVHandleSq][1] = sqNewPos
      local height = sqNewPos.y - groundCoverUVHandlesOnDragStart[groundCoverUVHandle_enum.topRight][1].y + handleSquareSize
      groundCoverHVal[0] = height/imageSize.y
    elseif draggingGCUVHandleSq == groundCoverUVHandle_enum.bottomRight then
      sqNewPos.y = sqNewPos.y + groundCoverUVDragStartLocalDelta.y
      sqNewPos.x = sqNewPos.x + groundCoverUVDragStartLocalDelta.x
      checkBoundaries(sqNewPos, constraints)
      groundCoverUVHandles[draggingGCUVHandleSq][1] = sqNewPos
      local width = sqNewPos.x - groundCoverUVHandlesOnDragStart[groundCoverUVHandle_enum.topLeft][1].x - 1
      local height = sqNewPos.y - groundCoverUVHandlesOnDragStart[groundCoverUVHandle_enum.topLeft][1].y
      groundCoverWVal[0] = width/imageSize.x
      groundCoverHVal[0] = height/imageSize.y
    end
  end

  if draggingGCUVHandleSq ~= nil then
    if imgui.IsMouseDragging(0) then
      updateGroundCoverUVHandle()
    end
  else
    local hoveredHandleSquare = nil
    for index, val in ipairs(groundCoverUVHandles) do
      if imgui.IsMouseHoveringRect(val[1], val[2]) then
        hoveredHandleSquare = index
      end
    end
    if hoveredHandleSquare ~= nil and imgui.IsMouseDown(0) then
      draggingGCUVHandleSq = hoveredHandleSquare
      groundCoverUVDragPos = imgui.GetMousePos()
      groundCoverUVHandlesOnDragStart = deepcopy(groundCoverUVHandles)
      local groundCoverUVDragStartLocalDeltaX = groundCoverUVDragPos.x - groundCoverUVHandlesOnDragStart[draggingGCUVHandleSq][1].x
      local groundCoverUVDragStartLocalDeltaY = groundCoverUVDragPos.y - groundCoverUVHandlesOnDragStart[draggingGCUVHandleSq][1].y
      groundCoverUVDragStartLocalDelta = imgui.ImVec2(groundCoverUVDragStartLocalDeltaX, groundCoverUVDragStartLocalDeltaY)
    end
  end
  if imgui.IsMouseReleased(0) then
    draggingGCUVHandleSq = nil
  end
end

local function groundCoverUVWindow(customData, retTbl)
  local typeIndex = customData.arrayIndex
  if typeIndex ~= groundCoverUVTypeIndex then return end
  if imgui.BeginPopupModal(groundCoverUVWindowName, nil) then
    local availableSize = imgui.GetContentRegionAvail()
    if typeIndex == groundCoverUVTypeIndex then
      local fontSize = math.ceil(imgui.GetFontSize())
      local menuBarHeight = 2*imgui.GetStyle().FramePadding.y + fontSize
      local uvMainPanelHeight = imgui.GetWindowSize().y - (2*menuBarHeight - 6 + 3*imgui.GetStyle().WindowPadding.y + imgui.GetStyle().FramePadding.x + 2*imgui.GetStyle().ChildBorderSize)

      if imgui.BeginChild1("ColorMapColumn", imgui.ImVec2(0, uvMainPanelHeight), true, imgui.WindowFlags_NoScrollWithMouse) then
        imgui.Columns(2, "MainColumn")
        imgui.SetColumnWidth(0, availableSize.x * 0.75)

        local textStartCursorPos = imgui.GetCursorPos()
        imgui.SetCursorPos(imgui.ImVec2(fontSize*2, fontSize*2))

        local availableImageSize = imgui.GetContentRegionAvail()
        local imageSize = math.min(availableImageSize.x, availableImageSize.y) - 6*imgui.GetStyle().ChildBorderSize
        local size = imgui.ImVec2(imageSize, imageSize)

        local groundCoverId = valueInspector.selectedIds[#valueInspector.selectedIds]
        local groundCover = scenetree.findObjectById(groundCoverId)
        local groundCoverMaterialName = groundCover:getField("Material", "")
        local groundCoverMaterial = scenetree.findObject(groundCoverMaterialName)
        local texturePath = "/core/art/warnMat.dds"
        local texture = nil

        if groundCoverMaterial ~= nil then
          local colorMap = groundCoverMaterial:getField("colorMap", 0)
          texturePath = (colorMap == "" and "/core/art/missingTexture.dds" or colorMap)
        end

        if texturePath ~= ""then
          texturePath = (string.find(texturePath, "/") ~= nil and texturePath or (groundCoverMaterial:getPath() .. texturePath))
          texture = editor.getTempTextureObj(texturePath)
        end
        if texture and texture.size.x ~= 0 and texture.size.y ~= 0 then
          local x = imageSize * texture.size.x / texture.size.y
          local y = imageSize
          local mul = 1
          if x > availableImageSize.x then
            mul = availableImageSize.x /x
          end
          size.x = x * mul
          size.y = y * mul
        end

        local windowPos = imgui.GetWindowPos()
        local imageStartCursorPos = imgui.GetCursorPos()
        local imageStartAbsCursorPos = imgui.GetCursorScreenPos()
        local drawlist = imgui.GetWindowDrawList()
        local coloredBG_StartPos_X = windowPos.x + imageStartCursorPos.x + imgui.GetStyle().ChildBorderSize
        local coloredBG_StartPos_Y = windowPos.y + imageStartCursorPos.y + imgui.GetStyle().ChildBorderSize

        local p1_BG = imgui.ImVec2(coloredBG_StartPos_X, coloredBG_StartPos_Y)
        local p2_BG = imgui.ImVec2(coloredBG_StartPos_X + size.x, coloredBG_StartPos_Y + size.y)
        imgui.ImDrawList_AddRectFilled(drawlist, p1_BG, p2_BG, imgui.GetColorU322(groundCoverUVBgColors[groundCoverUVBgColorIndex].color))

        imgui.SetCursorPos(imageStartCursorPos)
        imgui.Image(texture.tex:getID(), size, nil, nil, nil, editor.color.white.Value)

        drawUVHandle(imageStartCursorPos, imageStartAbsCursorPos, size)
        imgui.SetCursorPos(imgui.ImVec2(textStartCursorPos.x + fontSize/2, fontSize))
        imgui.TextUnformatted("0.0")
        imgui.SameLine()

        imgui.SetCursorPosX(fontSize*2 + (size.x)/2)
        imgui.TextUnformatted("U")
        imgui.SameLine()

        imgui.SetCursorPosX(size.x - (imgui.CalcTextSize("1.0").x) + 2*fontSize)
        imgui.TextUnformatted("1.0")

        imgui.SetCursorPos(imgui.ImVec2(textStartCursorPos.x + fontSize, fontSize*2 + size.y / 2))
        imgui.TextUnformatted("V")

        imgui.SetCursorPos(imgui.ImVec2(textStartCursorPos.x + fontSize/2, size.y + fontSize))
        imgui.TextUnformatted("1.0")

        imgui.NextColumn()
        if imgui.BeginChild1("UVValuesColumn", nil, true) then
          local cursorPos = imgui.GetCursorPos()
          local uvValueWidgetWidth = imgui.CalcTextSize("Height").x

          imgui.TextUnformatted("Type ".."[".. tostring(groundCoverUVTypeIndex) .."]")

          imgui.TextUnformatted("U: ")
          imgui.SameLine()
          imgui.SetCursorPosX(cursorPos.x + uvValueWidgetWidth + 2*imgui.GetStyle().FramePadding.x)
          editor.uiInputFloat("##input" .. tostring(typeIndex).."U", groundCoverUVal, 0.1, 1.0, "%0.5f", imgui.InputTextFlags_EnterReturnsTrue)

          imgui.TextUnformatted("V: ")
          imgui.SameLine()
          imgui.SetCursorPosX(cursorPos.x + uvValueWidgetWidth + 2*imgui.GetStyle().FramePadding.x)
          editor.uiInputFloat("##input" .. tostring(typeIndex).."V", groundCoverVVal, 0.1, 1.0, "%0.5f", imgui.InputTextFlags_EnterReturnsTrue)

          imgui.TextUnformatted("Width: ")
          imgui.SameLine()
          imgui.SetCursorPosX(cursorPos.x + uvValueWidgetWidth + 2*imgui.GetStyle().FramePadding.x)
          editor.uiInputFloat("##input" .. tostring(typeIndex).."W", groundCoverWVal, 0.1, 1.0, "%0.5f", imgui.InputTextFlags_EnterReturnsTrue)

          imgui.TextUnformatted("Height: ")
          imgui.SameLine()
          imgui.SetCursorPosX(cursorPos.x + uvValueWidgetWidth + 2*imgui.GetStyle().FramePadding.x)
          editor.uiInputFloat("##input" .. tostring(typeIndex).."H", groundCoverHVal, 0.1, 1.0, "%0.5f", imgui.InputTextFlags_EnterReturnsTrue)

          imgui.SetCursorPosX(cursorPos.x + uvValueWidgetWidth + 2*imgui.GetStyle().FramePadding.x)
          if imgui.Button("Reset") then
            local vec = stringToTable(groundCoverUVInitialValue)
            if vec[1] == nil then vec[1] = "0" end
            if vec[2] == nil then vec[2] = "0" end
            if vec[3] == nil then vec[3] = "0" end
            if vec[4] == nil then vec[4] = "0" end
            groundCoverUVal[0] = tonumber(vec[1])
            groundCoverVVal[0] = tonumber(vec[2])
            groundCoverWVal[0] = tonumber(vec[3])
            groundCoverHVal[0] = tonumber(vec[4])
          end
        end
        imgui.EndChild()
      end
      imgui.EndChild()
      imgui.Columns(1)

      local uvValueWidgetWidth = 2*imgui.GetContentRegionAvailWidth()/16
      imgui.PushItemWidth(uvValueWidgetWidth)

      imgui.TextUnformatted("Handle Color: ")
      imgui.SameLine()

      local windowPos = imgui.GetWindowPos()
      local cursorPos = imgui.GetCursorPos()

      local p1_HandleColorLabel = imgui.ImVec2(windowPos.x + cursorPos.x, windowPos.y + cursorPos.y + imgui.GetStyle().FramePadding.y)
      local p2_HandleColorLabel = imgui.ImVec2(p1_HandleColorLabel.x + fontSize, p1_HandleColorLabel.y + fontSize)

      imgui.ImDrawList_AddRectFilled(imgui.GetWindowDrawList(), p1_HandleColorLabel, p2_HandleColorLabel, imgui.GetColorU322(groundCoverUVHandleColors[groundCoverUVHandleColorIndex].color))
      imgui.ImDrawList_AddRect(imgui.GetWindowDrawList(), imgui.ImVec2(p1_HandleColorLabel.x - 1, p1_HandleColorLabel.y - 1), imgui.ImVec2(p2_HandleColorLabel.x + 1, p2_HandleColorLabel.y + 1), imgui.GetColorU322(imgui.ImVec4(0, 0, 0, 1)), nil, nil, 1)

      imgui.SetCursorPosX(imgui.GetCursorPos().x + fontSize + 2)
      if imgui.BeginCombo("##handleColor", groundCoverUVHandleColors[groundCoverUVHandleColorIndex].name) then
        local windowPos = imgui.GetWindowPos()
        local comboCursorPos = imgui.GetCursorPos()
        local drawlist = imgui.GetWindowDrawList()
        local coloredBG_StartPos_X = windowPos.x + comboCursorPos.x
        local coloredBG_EndPos_X = coloredBG_StartPos_X + fontSize - 4*imgui.GetStyle().ChildBorderSize
        local coloredBG_StartPos_Y = windowPos.y + comboCursorPos.y - imgui.GetStyle().ChildBorderSize

        for index, val in ipairs(groundCoverUVHandleColors) do
          local p1_BG = imgui.ImVec2(coloredBG_StartPos_X, coloredBG_StartPos_Y + 4*imgui.GetStyle().ChildBorderSize)
          local p2_BG = imgui.ImVec2(coloredBG_EndPos_X, coloredBG_StartPos_Y + fontSize - imgui.GetStyle().ChildBorderSize)

          imgui.ImDrawList_AddRectFilled(drawlist, p1_BG, p2_BG, imgui.GetColorU322(val.color))
          imgui.ImDrawList_AddRect(drawlist, imgui.ImVec2(p1_BG.x - 1, p1_BG.y - 1), imgui.ImVec2(p2_BG.x + 1, p2_BG.y + 1), imgui.GetColorU322(imgui.ImVec4(0, 0, 0, 1)), nil, nil, 1)

          imgui.SetCursorPosX(imgui.GetCursorPos().x + fontSize)
          if imgui.Selectable1(val.name, false) then
            groundCoverUVHandleColorIndex = index
          end
          coloredBG_StartPos_Y = coloredBG_StartPos_Y + fontSize + (2*imgui.GetStyle().ChildBorderSize)
        end
        imgui.EndCombo()
      end

      imgui.SameLine()
      imgui.SetCursorPosX(imgui.GetCursorPos().x + fontSize)
      imgui.TextUnformatted("Background Color: ")
      imgui.SameLine()

      local cursorPosBg = imgui.GetCursorPos()
      local p1_bgColorLabel = imgui.ImVec2(windowPos.x + cursorPosBg.x, windowPos.y + cursorPosBg.y + imgui.GetStyle().FramePadding.y)
      local p2_bgColorLabel = imgui.ImVec2(p1_bgColorLabel.x + fontSize, p1_bgColorLabel.y + fontSize)

      imgui.ImDrawList_AddRectFilled(imgui.GetWindowDrawList(), p1_bgColorLabel, p2_bgColorLabel, imgui.GetColorU322(groundCoverUVBgColors[groundCoverUVBgColorIndex].color))
      imgui.ImDrawList_AddRect(imgui.GetWindowDrawList(), imgui.ImVec2(p1_bgColorLabel.x - 1, p1_bgColorLabel.y - 1), imgui.ImVec2(p2_bgColorLabel.x + 1, p2_bgColorLabel.y + 1), imgui.GetColorU322(imgui.ImVec4(0, 0, 0, 1)), nil, nil, 1)

      imgui.SetCursorPosX(cursorPosBg.x + fontSize + 2)
      if imgui.BeginCombo("##backgroundColor", groundCoverUVBgColors[groundCoverUVBgColorIndex].name) then
        local windowPos = imgui.GetWindowPos()
        local cursorPos = imgui.GetCursorPos()
        local drawlist = imgui.GetWindowDrawList()
        local coloredBG_StartPos_X = windowPos.x + cursorPos.x
        local coloredBG_EndPos_X = coloredBG_StartPos_X + fontSize - 4*imgui.GetStyle().ChildBorderSize
        local coloredBG_StartPos_Y = windowPos.y + cursorPos.y - imgui.GetStyle().ChildBorderSize

        for index, val in ipairs(groundCoverUVBgColors) do
          local p1_BG = imgui.ImVec2(coloredBG_StartPos_X, coloredBG_StartPos_Y + 4*imgui.GetStyle().ChildBorderSize)
          local p2_BG = imgui.ImVec2(coloredBG_EndPos_X, coloredBG_StartPos_Y + fontSize - imgui.GetStyle().ChildBorderSize)

          imgui.ImDrawList_AddRectFilled(drawlist, p1_BG, p2_BG, imgui.GetColorU322(val.color))
          imgui.ImDrawList_AddRect(drawlist, imgui.ImVec2(p1_BG.x - 1, p1_BG.y - 1), imgui.ImVec2(p2_BG.x + 1, p2_BG.y + 1), imgui.GetColorU322(imgui.ImVec4(0, 0, 0, 1)), nil, nil, 1)

          imgui.SetCursorPosX(imgui.GetCursorPos().x + fontSize)
          if imgui.Selectable1(val.name, false) then
            groundCoverUVBgColorIndex = index
          end
          coloredBG_StartPos_Y = coloredBG_StartPos_Y + fontSize + (2*imgui.GetStyle().ChildBorderSize)
        end
        imgui.EndCombo()
      end

      imgui.PopItemWidth()
      imgui.SameLine()
      imgui.SetCursorPosX(availableSize.x * 0.75 + 2*imgui.GetStyle().FramePadding.x)
      if imgui.Button("OK") then
        local fieldVal =
        tostring(groundCoverUVal[0]) .. " " .. tostring(groundCoverVVal[0]) .. " " .. tostring(groundCoverWVal[0] .. " " .. tostring(groundCoverHVal[0]))
        if(fieldVal ~= retTbl.fieldVal) then
          retTbl.valueChanged = true
          retTbl.fieldVal = fieldVal
        end
        imgui.CloseCurrentPopup()
      end

      imgui.SameLine()
      if imgui.Button("Cancel") then
        imgui.CloseCurrentPopup()
      end
    end
    imgui.EndPopup()
  end
end

local function onEditorGui()
  if guiInstancer.instances then
    for key, inspectorInfo in pairs(guiInstancer.instances) do
      local wndName = inspectorWindowNamePrefix .. key

      if not editor.isWindowVisible(wndName) then
        editor.closeInspectorInstance(key)
      end

      if editor.beginWindow(wndName, "Inspector##" .. key, imgui.WindowFlags_AlwaysVerticalScrollbar) then
        if inspectorInfo.selection then
          if editor.uiIconImageButton(editor.icons.lock, imgui.ImVec2(24, 24)) then
            inspectorInfo.selection = nil
            inspectorInfo.ctx = nil
          end
          if imgui.IsItemHovered() then imgui.SetTooltip("Unlock Inspector Window") end
        elseif editor.uiIconImageButton(editor.icons.lock_open, imgui.ImVec2(24, 24)) and (not tableIsEmpty(editor.selection)) then
          inspectorInfo.selection = deepcopy(editor.selection)
          inspectorInfo.ctx = createInspectorContext()
          inspectorInfo.ctx.fields = deepcopy(sharedCtx.fields)
        end
        imgui.tooltip("Lock this Inspector to the currently selected object(s)")
        imgui.SameLine()
        local numKeys = 0
        if editor.uiIconImageButton(editor.icons.fiber_new, imgui.ImVec2(24, 24)) then
          editor.addInspectorInstance()
        end
        if imgui.IsItemHovered() then imgui.SetTooltip("New Inspector Window") end
        if inspectorInfo.selection then
          imgui.SameLine()
          imgui.PushStyleColor2(imgui.Col_Text, lockedInspectorColor)
          imgui.Text("[Locked]")
          imgui.PopStyleColor()
        else
          -- first lets check if we have multiple selection types
          for _, val in pairs(editor.selection) do
            if not tableIsEmpty(val) then
              numKeys = numKeys + 1
              if numKeys == 2 then
                break
              end
            end
          end
        end
        imgui.SameLine()
        editor.uiHelpButton("Inspector", "world_editor/windows/inspector/")
        if numKeys == 2 then
          imgui.Text("Multiple types selected:")
          for className, val in pairs(editor.selection) do
            imgui.Text(#val .. " " .. className .. "(s)")
          end
        else
          -- allow various tools to render custom specific UI in the header of the object inspector window
          extensions.hook("onEditorInspectorHeaderGui", inspectorInfo)
          -- inspector has multiple view types, like object inspector, editor settings, asset properties etc.
          -- so we provide a function for the current mode, the default is object inspector objectInspectorGui function
          for typeName, typeHandler in pairs(inspectorTypeHandlers) do
            -- if we have a locked inspector, use its selection
            if inspectorInfo.selection ~= nil then
              -- if we found this type to have something selected, show ui
              if inspectorInfo.selection[typeHandler.typeName] ~= nil then
                if typeHandler.guiCallback then
                  typeHandler.guiCallback(inspectorInfo)
                  break -- stop at first viable type handler, just show this type inspector ui
                end
              end
            -- else use the editor selection
            elseif editor.selection[typeName] ~= nil then
              if typeHandler.guiCallback then
                typeHandler.guiCallback(inspectorInfo)
                break -- stop at first viable type handler, just show this type inspector ui
              end
            end
          end
        end
      else
        if not editor.isWindowVisible(wndName) then
          editor.closeInspectorInstance(key)
        end
      end
      editor.endWindow()
    end
  end
  checkEditorDirtyFlag()
end

local function onWindowMenuItem()
  openInspector()
end

local function onEditorActivated()
  valueInspector:initializeTables()
  sharedCtx = createInspectorContext()
  M.onEditorObjectSelectionChanged()
end

local function onEditorDeactivated()
end

local function onEditorLoadGuiInstancerState(state)
  guiInstancer:deserialize("inspectorInstances", state)
  for key, val in pairs(guiInstancer.instances) do
    editor.registerWindow(inspectorWindowNamePrefix .. tostring(key), imgui.ImVec2(300, 500))
    val.fieldNameFilter = imgui.ImGuiTextFilter()
  end
end

local function onEditorSaveGuiInstancerState(state)
  guiInstancer:serialize("inspectorInstances", state)
end

local metallicValuePtrs = {}
metallicValuePtrs[0] = imgui.FloatPtr(0)
metallicValuePtrs[1] = imgui.FloatPtr(0)
metallicValuePtrs[2] = imgui.FloatPtr(0)
metallicValuePtrs[3] = imgui.FloatPtr(0)

local metallicLabels = {}
metallicLabels[0] = "Metallic"
metallicLabels[1] = "Roughness"
metallicLabels[2] = "Clearcoat"
metallicLabels[3] = "Cc Roughness"

local function customVehicleMetallicFieldEditor(objectIds, fieldValue, fieldName, fieldLabel, fieldDesc, fieldType, fieldTypeName, customData, pasteCallback, contextMenuUI)
  if imgui.CollapsingHeader1(fieldName) then
    local floatFormat = "%0." .. editor.getPreference("ui.general.floatDigitCount") .. "f"
    for colorIndex = 0, 2 do
      imgui.Text("Paint " .. colorIndex+1)
      local fieldValue = editor.getFieldValue(valueInspector.selectedIds[#valueInspector.selectedIds], fieldName, colorIndex)
      local metallicValues = stringToTable(fieldValue)
      metallicValuePtrs[0][0] = tonumber(metallicValues[1])
      metallicValuePtrs[1][0] = tonumber(metallicValues[2])
      metallicValuePtrs[2][0] = tonumber(metallicValues[3])
      metallicValuePtrs[3][0] = tonumber(metallicValues[4])
      for propertyIndex = 0, 3 do
        imgui.PushItemWidth(imgui.GetContentRegionAvailWidth() - imgui.CalcTextSize(metallicLabels[3]).x)
        if editor.getPreference("ui.general.useSlidersInInspector") then
          editor.uiDragFloat(metallicLabels[propertyIndex] .. "##" .. colorIndex, metallicValuePtrs[propertyIndex], 0.1, 0, 1, floatFormat, nil, sharedCtx.editEnded)
        else
          editor.uiInputFloat(metallicLabels[propertyIndex] .. "##" .. colorIndex, metallicValuePtrs[propertyIndex], 0.1, 0.5, floatFormat, nil, sharedCtx.editEnded)
        end

        if sharedCtx.editEnded[0] then
          objectHistoryActions.changeObjectFieldWithUndo({valueInspector.selectedIds[#valueInspector.selectedIds]}, fieldName, metallicValuePtrs[0][0] .. " " .. metallicValuePtrs[1][0] .. " " .. metallicValuePtrs[2][0] .. " " .. metallicValuePtrs[3][0], colorIndex)
        end
      end
    end
  end
end

local function customDecalDataRowsColsLabelChanger(fieldName, objectClass)
  if fieldName == "texRows" then return "texCols" end
  if fieldName == "texCols" then return "texRows" end
end

local function drawGroundCoverUVIndicators(windowPos, cursorPos, widgetWidth)
  local drawlist = imgui.GetWindowDrawList()
  local coloredBGHeight = math.ceil(imgui.GetFontSize())
  local textWidth = imgui.CalcTextSize("W").x
  local textPadding = imgui.GetFontSize() - textWidth
  local coloredBGWidth = math.ceil(imgui.GetFontSize()) - textPadding/2 - 4

  local coloredBG_StartPos_Y = windowPos.y + cursorPos.y - imgui.GetScrollY() + 3
  local coloredBG_EndPos_Y = coloredBG_StartPos_Y + coloredBGHeight - 6
  local textPadding_Y = coloredBG_StartPos_Y - 4

  local coloredBG_U_StartPos_X = windowPos.x + cursorPos.x + 4 * math.ceil(imgui.uiscale[0])
  local coloredBG_U_EndPos_X = coloredBG_U_StartPos_X + coloredBGWidth

  local coloredBG_V_StartPos_X = coloredBG_U_StartPos_X + (widgetWidth/4)
  local coloredBG_V_EndPos_X = coloredBG_V_StartPos_X + coloredBGWidth

  local coloredBG_W_StartPos_X = coloredBG_U_StartPos_X + (widgetWidth/4)*2
  local coloredBG_W_EndPos_X = coloredBG_W_StartPos_X + coloredBGWidth

  local coloredBG_H_StartPos_X = coloredBG_U_StartPos_X + (widgetWidth/4)*3
  local coloredBG_H_EndPos_X = coloredBG_H_StartPos_X + coloredBGWidth

  local labelBGColor = imgui.GetColorU322(imgui.ImVec4(0.2, 0.2, 0.2, 1.0))

  local p1_U_BG = imgui.ImVec2(coloredBG_U_StartPos_X, coloredBG_StartPos_Y)
  local p2_U_BG = imgui.ImVec2(coloredBG_U_EndPos_X, coloredBG_EndPos_Y)
  local p1_U_Text = imgui.ImVec2(coloredBG_U_StartPos_X + textPadding/4 - 2, textPadding_Y)

  local p1_V_BG = imgui.ImVec2(coloredBG_V_StartPos_X, coloredBG_StartPos_Y)
  local p2_V_BG = imgui.ImVec2(coloredBG_V_EndPos_X, coloredBG_EndPos_Y)
  local p1_V_Text = imgui.ImVec2(coloredBG_V_StartPos_X + textPadding/4, textPadding_Y)

  local p1_W_BG = imgui.ImVec2(coloredBG_W_StartPos_X, coloredBG_StartPos_Y)
  local p2_W_BG = imgui.ImVec2(coloredBG_W_EndPos_X, coloredBG_EndPos_Y)
  local p1_W_Text = imgui.ImVec2(coloredBG_W_StartPos_X + textPadding/4, textPadding_Y)

  local p1_H_BG = imgui.ImVec2(coloredBG_H_StartPos_X, coloredBG_StartPos_Y)
  local p2_H_BG = imgui.ImVec2(coloredBG_H_EndPos_X, coloredBG_EndPos_Y)
  local p1_H_Text = imgui.ImVec2(coloredBG_H_StartPos_X + textPadding/4, textPadding_Y)

  local labelTextColor = imgui.GetColorU322(imgui.ImVec4(1.0, 1.0, 1.0, 1.0))

  imgui.ImDrawList_AddRectFilled(drawlist, p1_U_BG, p2_U_BG, labelBGColor)
  imgui.ImDrawList_AddText1(drawlist, p1_U_Text, labelTextColor, "U", nil)

  imgui.ImDrawList_AddRectFilled(drawlist, p1_V_BG, p2_V_BG, labelBGColor)
  imgui.ImDrawList_AddText1(drawlist, p1_V_Text, labelTextColor, "V", nil)

  imgui.ImDrawList_AddRectFilled(drawlist, p1_W_BG, p2_W_BG, labelBGColor)
  imgui.ImDrawList_AddText1(drawlist, p1_W_Text, labelTextColor, "W", nil)

  imgui.ImDrawList_AddRectFilled(drawlist, p1_H_BG, p2_H_BG, labelBGColor)
  imgui.ImDrawList_AddText1(drawlist, p1_H_Text, labelTextColor, "H", nil)
end

local billboardUVValue = imgui.ArrayFloat(4)
local function customGroundCoverBillBoardUVsFieldEditor(objectIds, fieldValue, fieldName, fieldLabel, fieldDesc, fieldType, fieldTypeName, customData, pasteCallback, contextMenuUI)
  local uvVec = stringToTable(fieldValue)
  if uvVec[1] == nil then uvVec[1] = "0" end
  if uvVec[2] == nil then uvVec[2] = "0" end
  if uvVec[3] == nil then uvVec[3] = "0" end
  if uvVec[4] == nil then uvVec[4] = "0" end

  billboardUVValue[0] = tonumber(uvVec[1])
  billboardUVValue[1] = tonumber(uvVec[2])
  billboardUVValue[2] = tonumber(uvVec[3])
  billboardUVValue[3] = tonumber(uvVec[4])

  local windowPos = imgui.GetWindowPos()
  local cursorPos = imgui.GetCursorPos()

  local buttonSize = imgui.GetFontSize() / imgui.uiscale[0]
  local uvValueWidgetWidth = imgui.GetContentRegionAvailWidth() - buttonSize * imgui.uiscale[0]

  imgui.PushStyleVar2(imgui.StyleVar_FramePadding, imgui.ImVec2(imgui.GetFontSize(), 0))
  imgui.PushItemWidth(uvValueWidgetWidth)
  editor.uiInputFloat4("##groundcoverUV", billboardUVValue, nil, imgui.InputTextFlags_ReadOnly, nil)
  imgui.PopStyleVar()
  imgui.PopItemWidth()
  imgui.SameLine()
  local prevCursorPos = imgui.ImVec2(imgui.GetCursorPos().x - imgui.GetStyle().FramePadding.x / 2, imgui.GetCursorPos().y)

  imgui.InvisibleButton("openUVEditorButton", imgui.ImVec2(buttonSize * imgui.uiscale[0], buttonSize * imgui.uiscale[0]))
  local uvButtonBGColor = imgui.GetStyleColorVec4(imgui.Col_Button)
  if imgui.IsItemHovered() then
    if imgui.IsMouseDown(0) then
      uvButtonBGColor = imgui.GetStyleColorVec4(imgui.Col_ButtonActive)
      groundCoverUVTypeIndex = customData.arrayIndex
      groundCoverUVInitialValue = fieldValue
      local vec = stringToTable(fieldValue)
      if vec[1] == nil then vec[1] = "0" end
      if vec[2] == nil then vec[2] = "0" end
      if vec[3] == nil then vec[3] = "0" end
      if vec[4] == nil then vec[4] = "0" end
      groundCoverUVal[0] = tonumber(vec[1])
      groundCoverVVal[0] = tonumber(vec[2])
      groundCoverWVal[0] = tonumber(vec[3])
      groundCoverHVal[0] = tonumber(vec[4])
      imgui.SetWindowSize2(groundCoverUVWindowName, imgui.ImVec2(800, 600))
      imgui.OpenPopup(groundCoverUVWindowName)
    else
      uvButtonBGColor = imgui.GetStyleColorVec4(imgui.Col_ButtonHovered)
    end
  end

  imgui.SetCursorPos(prevCursorPos)
  editor.uiIconImageButton(editor.icons.crop_free, imgui.ImVec2(buttonSize, buttonSize), nil, nil, uvButtonBGColor)

  drawGroundCoverUVIndicators(windowPos, cursorPos, uvValueWidgetWidth)

  local retTbl = {valueChanged = false, fieldVal = fieldValue}
  if groundCoverUVTypeIndex ~= nil then
    groundCoverUVWindow(customData, retTbl)
  end
  if retTbl.valueChanged then
    return {fieldValue = retTbl.fieldVal, editEnded = true}
  end
end

local function onEditorRegisterApi()
  editor.checkEditorDirtyFlag = checkEditorDirtyFlag
end

local function onEditorInitialized()
  valueInspector:reinitializeTables()
  registerInspectorTypeHandler("object", objectInspectorGui)
  editor.addWindowMenuItem("Inspector", onWindowMenuItem, nil, true)
  valueInspector.inspectorName = "mainInspector"
  valueInspector.addTypeToTooltip = true -- shows type in field description tooltip
  -- delete the various material/texture thumbs from previous editor show
  valueInspector:deleteTexObjs()
  -- set the value callback func, called when the edited value was changed in the value editor widgets
  valueInspector.setValueCallback = function(fieldName, fieldValue, arrayIndex, customData, editEnded)
    if customData.startValues then
      setMultiSelectionFieldWithOldValues(valueInspector.selectedIds, fieldName, fieldValue, customData.startValues, arrayIndex, editEnded, valueInspector)
    else
      setMultiSelectionFieldValue(valueInspector.selectedIds, fieldName, fieldValue, arrayIndex, editEnded)
    end
  end

  editor.registerCustomFieldInspectorEditor("BeamNGVehicle", "metallicPaintData", customVehicleMetallicFieldEditor, true)
  editor.registerCustomFieldInspectorEditor("GroundCover", "billboardUVs", customGroundCoverBillBoardUVsFieldEditor, true)
  editor.registerCustomFieldLabelChanger("DecalData", "texRows", customDecalDataRowsColsLabelChanger)
  editor.registerCustomFieldLabelChanger("DecalData", "texCols", customDecalDataRowsColsLabelChanger)
end

local function onEditorObjectSelectionChanged()
  if editor.pickingLinkTo and #editor.selection.object then
    -- local pid = Sim.findObjectById(editor.selection.object[1]):getOrCreatePersistentID() -- Note: 11/06/2024 - Only TerrainMaterial and DecalRoad still have persistent id. Persistent Id will soon be no more.
    -- editor.pickingLinkTo.child:setField("linkToParent", 0, pid)
    editor.selection.object = deepcopy(editor.pickingLinkTo.selectionObjectIds)
    editor.pickingLinkTo = nil
    return
  end

  table.clear(sharedCtx.fields)

  groundCoverUVTypeIndex = nil

  -- get all fields from all selected objects
  for i = 1, tableSize(editor.selection.object) do
    local objFields = editor.getFields(editor.selection.object[i])
    if objFields then
      for fldName, field in pairs(objFields) do
        if sharedCtx.fields[fldName] == nil then
          sharedCtx.fields[fldName] = field
          sharedCtx.fields[fldName].useCount = 1
        else
          sharedCtx.fields[fldName].useCount = sharedCtx.fields[fldName].useCount + 1
        end
      end
    end
  end
  valueInspector:setSimSetWindowFieldId(nil)
end

local function onEditorAfterOpenLevel()
  -- we do this to update the datablock lists to show in dropdowns
  valueInspector:initializeTables(true)
end

M.onEditorGui = onEditorGui
M.onEditorRegisterApi = onEditorRegisterApi
M.onEditorAfterOpenLevel = onEditorAfterOpenLevel
M.onExtensionLoaded = onExtensionLoaded
M.onEditorInitialized = onEditorInitialized
M.onEditorActivated = onEditorActivated
M.onEditorDeactivated = onEditorDeactivated
M.onEditorLoadGuiInstancerState = onEditorLoadGuiInstancerState
M.onEditorSaveGuiInstancerState = onEditorSaveGuiInstancerState
M.onEditorObjectSelectionChanged = onEditorObjectSelectionChanged

return M
