-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = "editor_dynamicDecals_layerStack"
local im = ui_imgui

-- reference to the editor tool, set in setup()
local tool = nil
-- reference to the dynamics decal api
local api = nil
local fill = nil
local textureFill = nil
local gizmo = nil
local selection = nil
local helper = nil
local docs = nil

local dragging = false
local layerDropHeight = 10

local maxLayerDepth = 2

-- copy/paste layer maks
local layerMaskCopyData = nil

-- drag'n'drop
local layerDragDropType = "DynDecalLayerDragDrop"
local payloadSize = "char[64]"

-- highlight button hovering
local permLayerHighlight = nil      -- type: layer
local layerHoverStateA = {}        -- type: layer
local layerHoverStateB = {}        -- type: layer
local layerMaskHoverStateA = {}    -- type: layer
local layerMaskHoverStateB = {}    -- type: layer

local function removeLayer(index, uid, parentUid)
  if editor.selection["dynamicDecalLayer"] and editor.selection["dynamicDecalLayer"][uid] then
    editor.selection["dynamicDecalLayer"][uid] = nil
    api.projectDynamicDecals = true
  end
  api.removeLayer(index, parentUid)

  if tool.getCurrentMaskEditingLayerUid() == uid then
    tool.setCurrentMaskEditingLayerUid(nil)
  end
end

-- includes the two icon buttons (to the right of the name text input field) to move the layers up and down
local function moveLayerGui(k, guiId, layer, parentUid, parentStack)
  if editor.getPreference("dynamicDecalsTool.layerStack.topToBottomLayerStructure") then
    if k == #parentStack then im.BeginDisabled() end
    if editor.uiIconImageButton(editor.icons.keyboard_arrow_up, im.ImVec2(tool.getIconSize(), tool.getIconSize()), nil, nil, nil, string.format("moveup##%s_%d", guiId, k)) then
      api.moveLayer(k, parentUid, k+1, parentUid)
    end
    im.tooltip("Move layer up")
    if k == #parentStack then im.EndDisabled() end

    im.SameLine()
    if k == 1 then im.BeginDisabled() end
    if editor.uiIconImageButton(editor.icons.keyboard_arrow_down, im.ImVec2(tool.getIconSize(), tool.getIconSize()), nil, nil, nil, string.format("movedown##%s_%d", guiId, k)) then
      api.moveLayer(k, parentUid, k-1, parentUid)
    end
    im.tooltip("Move layer down")
    if k == 1 then im.EndDisabled() end
  else
    if k == 1 then im.BeginDisabled() end
    if editor.uiIconImageButton(editor.icons.keyboard_arrow_up, im.ImVec2(tool.getIconSize(), tool.getIconSize()), nil, nil, nil, string.format("moveup##%s_%d", guiId, k)) then
      api.moveLayer(k, parentUid, k-1, parentUid)
    end
    im.tooltip("Move layer up")
    if k == 1 then im.EndDisabled() end

    im.SameLine()
    if k == #parentStack then im.BeginDisabled() end
    if editor.uiIconImageButton(editor.icons.keyboard_arrow_down, im.ImVec2(tool.getIconSize(), tool.getIconSize()), nil, nil, nil, string.format("movedown##%s_%d", guiId, k)) then
      api.moveLayer(k, parentUid, k+1, parentUid)
    end
    im.tooltip("Move layer down")
    if k == #parentStack then im.EndDisabled() end
  end
end

local function layerElementDragDropTargetDebug(colorTbl)
  local wpos = im.GetWindowPos()
  local cpos = im.GetCursorPos()
  local p1 = im.ImVec2(wpos.x + cpos.x, wpos.y + cpos.y - tool.getMainScrollY())
  local p2 = im.ImVec2(wpos.x + cpos.x + im.GetContentRegionAvailWidth(), wpos.y + cpos.y + layerDropHeight - tool.getMainScrollY())
  im.ImDrawList_AddRect(im.GetWindowDrawList(), p1, p2, im.GetColorU322(editor.getTempImVec4_TableTable(colorTbl or {1,1,1,1})))
end

local function layerDragDropTarget(name, layer, guiId, to, toParentUid, id, additionalCheckFn, dbgColor)
  if dragging then
    if editor.getPreference("dynamicDecalsTool.general.debug") then
      im.TextUnformatted(string.format("%s_%s_%s_%d", name, layer.uid, guiId, id))
      layerElementDragDropTargetDebug(dbgColor or {1,1,1,1})
    end
    im.BeginChild1(string.format("%s_%s_%s_%d", name, layer.uid, guiId, id), im.ImVec2(0, layerDropHeight), true)
    im.EndChild()
    if im.BeginDragDropTarget() then
      local payload = im.AcceptDragDropPayload(layerDragDropType)
      if payload~=nil then
        assert(payload.DataSize == ffi.sizeof(payloadSize))
        local data = jsonDecode(ffi.string(payload.Data))
        local from = data.from
        local fromParentUid = data.fromParentUid
        if additionalCheckFn then
          if editor.getPreference("dynamicDecalsTool.general.debug") then
            print(string.format("layerElementDragDropTargetDebug before\nfrom: %d\nfromParentUid: %s\nto: %d\ntoParentUid: %s", from or -1, fromParentUid or "nil", to or -1, toParentUid or "nil"))
          end
          from, fromParentUid, to, toParentUid = additionalCheckFn(from, fromParentUid, to, toParentUid)
          if editor.getPreference("dynamicDecalsTool.general.debug") then
            print(string.format("layerElementDragDropTargetDebug after\nfrom: %d\nfromParentUid: %s\nto: %d\ntoParentUid: %s", from or -1, fromParentUid or "nil", to or -1, toParentUid or "nil"))
          end
        end
        if editor.getPreference("dynamicDecalsTool.general.debug") then
          print(string.format("drag drop: %s, from: %d, fromParentUid: %s, to: %d, toParentUid: %s", name, from or -1, fromParentUid or "nil", to or -1, toParentUid or "nil"))
        end
        if (from ~= to) or (fromParentUid ~= toParentUid) then
          api.moveLayer(from, fromParentUid, to, toParentUid)
        end
      end
      im.EndDragDropTarget()
    end
  end
end

local function hasMirrorableChildren(layer)
  if not layer.children then return false end
  for _, child in ipairs(layer.children) do
    if child.type == api.layerTypes.decal or child.type == api.layerTypes.path or child.type == api.layerTypes.brushStroke then
      return true
    end
    if hasMirrorableChildren(child) then
      return true
    end
  end
  return false
end

local function layerElement(k, layer, guiId, parentUid, parentStack, layerLevel)
  editor.uiIconImageButton(editor.icons.menu, im.ImVec2(tool.getIconSize(), tool.getIconSize()), nil, nil, nil, string.format("%s_%s_%s", "layerdrag##", guiId, layer.uid))
  im.tooltip("LMB + drag to move layer")
  if im.BeginDragDropSource(im.DragDropFlags_SourceAllowNullID) then
    dragging = true
    local payload = ffi.new(payloadSize)
    ffi.copy(payload, jsonEncode({from = k, fromParentUid = parentUid}), ffi.sizeof(payloadSize))
    im.SetDragDropPayload(layerDragDropType, payload, ffi.sizeof(payloadSize))
    im.TextUnformatted(layer.name)
    im.EndDragDropSource()
  end

  -- drag drop target: drag icon
  if layerLevel < maxLayerDepth then
    if im.BeginDragDropTarget() then
      local payload = im.AcceptDragDropPayload(layerDragDropType)
      if payload~=nil then
        assert(payload.DataSize == ffi.sizeof(payloadSize))
        local data = jsonDecode(ffi.string(payload.Data))
        local from = data.from
        local fromParentUid = data.fromParentUid
        local to = nil
        local toParentUid = layer.uid
        if (from ~= to) or (fromParentUid ~= toParentUid) then
          api.moveLayer(from, fromParentUid, to, toParentUid)
        end
      end
      im.EndDragDropTarget()
    end
  end
  im.SameLine()

  local indentedCursorPosX = im.GetCursorPosX()
  local layerIconColorData = editor.getPreference("dynamicDecalsTool.layerStack.layerTypeIconColor")
  editor.uiIconImageButton(
    layer.type == api.layerTypes.decal and editor.icons.local_florist or
    layer.type == api.layerTypes.fill and editor.icons.format_color_fill or
    layer.type == api.layerTypes.textureFill and editor.icons.texture or
    layer.type == api.layerTypes.group and editor.icons.layers or
    layer.type == api.layerTypes.brushStroke and editor.icons.forest_paint or
    layer.type == api.layerTypes.path and editor.icons.simobject_path or
    layer.type == api.layerTypes.linkedSet and editor.icons.link or editor.icons.symbol_exclamation,
    im.ImVec2(tool.getIconSize(), tool.getIconSize()),
    layer.type == api.layerTypes.decal and editor.getTempImVec4_TableTable(layerIconColorData.decal) or
    layer.type == api.layerTypes.fill and editor.getTempImVec4_TableTable(layerIconColorData.fill) or
    layer.type == api.layerTypes.textureFill and editor.getTempImVec4_TableTable(layerIconColorData.textureFill) or
    layer.type == api.layerTypes.group and editor.getTempImVec4_TableTable(layerIconColorData.group) or
    layer.type == api.layerTypes.brushStroke and editor.getTempImVec4_TableTable(layerIconColorData.brushStroke) or
    layer.type == api.layerTypes.path and editor.getTempImVec4_TableTable(layerIconColorData.path) or
    layer.type == api.layerTypes.linkedSet and editor.getTempImVec4_TableTable(layerIconColorData.linkedSet or {1.0, 1.0, 1.0, 1.0}) or editor.getTempImVec4_TableTable({1,1,1,1})
  )
  im.tooltip(
    layer.type == api.layerTypes.decal and "Decal Layer" or
    layer.type == api.layerTypes.fill and "Fill Layer" or
    layer.type == api.layerTypes.textureFill and "Texture Fill Layer" or
    layer.type == api.layerTypes.group and "Group Layer" or
    layer.type == api.layerTypes.path and "Path Layer" or
    layer.type == api.layerTypes.brushStroke and "Brush Stroke Layer" or
    layer.type == api.layerTypes.linkedSet and "Linked Set Layer" or "!Layer"
  )
  im.SameLine()

  if editor.getPreference("dynamicDecalsTool.general.debug") then
    im.TextUnformatted(string.format("%d", k))
    im.SameLine()
  end

  im.PushItemWidth(im.GetContentRegionAvailWidth() - 4 * tool.getIconSize() - 5 * im.GetStyle().ItemSpacing.x)
  editor.uiInputText(
    string.format("%s_%s_%s", "##LayerName", guiId, layer.uid),
    editor.getTempCharPtr(layer.name),
    nil,
    im.InputTextFlags_AutoSelectAll,
    nil,
    nil,
    editor.getTempBool_BoolBool(false)
  )
  if editor.getTempBool_BoolBool() == true then
    local layerCopy = deepcopy(layer)
    layerCopy.name = editor.getTempCharPtr()
    api.setLayer(layerCopy, false)
  end

  im.PopItemWidth()
  im.SameLine()
  moveLayerGui(k, guiId, layer, parentUid, parentStack)

  im.SameLine()
  im.SetCursorPosX(im.GetCursorPosX() + (im.GetContentRegionAvailWidth() - (tool.getIconSize() * im.uiscale[0])))
  if editor.uiIconImageButton(layer.enabled and editor.icons.visibility or editor.icons.visibility_off, im.ImVec2(tool.getIconSize(), tool.getIconSize()), nil, nil, nil, string.format("##enabled_%s_%s", guiId, layer.uid)) then
    layer.enabled = not layer.enabled
    api.setLayer(layer, true)
  end
  im.tooltip("Toggle layer visibility")

  im.SetCursorPosX(indentedCursorPosX)

  local selectionData = editor.selection["dynamicDecalLayer"]

  -- TODO: Use input action instead of im.IsKeyDown(im.GetKeyIndex(im.Key_ReservedForModCtrl)) as 'Ctrl' modifier
  if selectionData and selectionData[layer.uid] then
    if editor.uiIconImageButton(editor.icons.near_me, im.ImVec2(tool.getIconSize(), tool.getIconSize()), editor.color.beamng.Value, nil, nil, string.format("select##%s_%s", guiId, layer.uid)) then
      selection.deselectLayer(im.IsKeyDown(im.GetKeyIndex(im.Key_ReservedForModCtrl)) and layer.uid or nil)
    end
    im.tooltip("Unselect layer")
  else
    if editor.uiIconImageButton(editor.icons.near_me, im.ImVec2(tool.getIconSize(), tool.getIconSize()), nil, nil, nil, string.format("select##%s_%s", guiId, layer.uid)) then
      selection.selectLayer(layer.uid, im.IsKeyDown(im.GetKeyIndex(im.Key_ReservedForModCtrl)))
    end
    im.tooltip("Select layer")
  end

  im.SameLine()
  if editor.uiIconImageButton(editor.icons.delete, im.ImVec2(tool.getIconSize(), tool.getIconSize()), nil, nil, nil, string.format("remove##%s_%s", guiId, layer.uid)) then
    removeLayer(k, layer.uid, parentUid)
  end
  im.tooltip("Remove layer")

  im.SameLine()
  if editor.uiIconImageButton(editor.icons.content_copy, im.ImVec2(tool.getIconSize(), tool.getIconSize()), nil, nil, nil, string.format("duplicate##%s_%s", guiId, layer.uid)) then
    api.duplicateLayer(k, parentUid)
  end
  im.tooltip("Duplicate layer")

  if layer.type == api.layerTypes.decal or layer.type == api.layerTypes.path or layer.type == api.layerTypes.brushStroke or hasMirrorableChildren(layer) then
    im.SameLine()
    if editor.uiIconImageButton(editor.icons.content_copy, im.ImVec2(tool.getIconSize(), tool.getIconSize()), nil, nil, nil, string.format("duplicateAndMirror##%s_%s", guiId, layer.uid)) then
      api.duplicateAndMirrorLayer(k, parentUid, true) -- args: id, parentUid, mirrorChildren
    end
    im.tooltip("Duplicate And Mirror layer")
  end

  -- LAYER MASK EDITING + COPY/PASTE
  local layerMaskContextMenuPopupName = string.format("CopyLayerMask_%s_%d_%d", layer.uid, layerLevel, k)
  if im.BeginPopup(layerMaskContextMenuPopupName) then
    if not layer.mask then im.BeginDisabled() end
    if im.Button("Copy Layer Mask") then
      layerMaskCopyData = deepcopy(layer.mask)
      editor.logInfo(string.format("%s: %s", logTag, "Copied layer mask"))
      im.CloseCurrentPopup()
    end
    if not layer.mask then im.EndDisabled() end

    if not layerMaskCopyData then im.BeginDisabled() end
    if im.Button(layer.mask and "Replace Layer Mask" or "Paste Layer Mask") then
      for _, maskLayer in ipairs(layerMaskCopyData.layers) do
        maskLayer.uid = api.getRandomUid()
      end
      local layerCopy = deepcopy(layer)
      layerCopy.mask = deepcopy(layerMaskCopyData)
      api.setLayer(layerCopy, true)
      editor.logInfo(string.format("%s: %s", logTag, layer.mask and "Replaced Layer Mask" or "Pasted Layer Mask"))
      im.CloseCurrentPopup()
    end
    if not layerMaskCopyData then im.EndDisabled() end

    if not layerMaskCopyData then im.BeginDisabled() end
    if im.Button("Append Layer Mask") then
      local layerCopy = deepcopy(layer)
      for _, maskLayer in ipairs(layerMaskCopyData.layers) do
        maskLayer.uid = api.getRandomUid()
        table.insert(layerCopy.mask.layers, maskLayer)
      end
      api.setLayer(layerCopy, true)
      editor.logInfo(string.format("%s: %s", logTag, "Appended Layer Mask"))
      im.CloseCurrentPopup()
    end
    if not layerMaskCopyData then im.EndDisabled() end

    im.Separator()
    if not layer.mask then im.BeginDisabled() end
    if im.Button("Export layer mask") then
      -- api.exportLayerMask(layer, string.format("%sexport/masks/", tool.directoryPath), "niceexport", "png")
      im.CloseCurrentPopup()
      editor_fileDialog.saveFile(
        function(data)
          local dir, file, ext = path.split(data.filepath)
          file = string.sub(file, 1, #file - (#ext + 1))
          api.exportLayerMask(layer, dir, file, ext)
        end,
        {{"Any files", "*"},{"PNG",".png"},{"DDS",".dds"}},
        false,
        string.format("%sexport/masks/", tool.directoryPath),
        "File already exists.\nOverwrite?"
      )
    end
    if not layer.mask then im.EndDisabled() end
    im.EndPopup()
  end

  if tool.getCurrentMaskEditingLayerUid() and tool.getCurrentMaskEditingLayerUid() == layer.uid then
    im.SameLine()
    if editor.uiIconImageButton(editor.icons.ab_asset_material, im.ImVec2(tool.getIconSize(), tool.getIconSize()), editor.color.beamng.Value, nil, nil, string.format("disable_mask##%s_%s", guiId, layer.uid)) then
      tool.setCurrentMaskEditingLayerUid(nil)
    end
    if im.IsItemClicked(1) then
      im.OpenPopup(layerMaskContextMenuPopupName)
    end
    im.tooltip("Disable mask editing\nRMB to open layer mask context menu")
  else
    im.SameLine()
    if editor.uiIconImageButton(editor.icons.ab_asset_material, im.ImVec2(tool.getIconSize(), tool.getIconSize()), nil, nil, nil, string.format("enable_mask##%s_%s", guiId, layer.uid)) then
      tool.setCurrentMaskEditingLayerUid(layer.uid)
    end
    if im.IsItemClicked(1) then
      im.OpenPopup(layerMaskContextMenuPopupName)
    end
    im.tooltip("Enable mask editing\nRMB to open layer mask context menu")
  end

  -- LAYER HIGHLIGHTING
  if layer.type == api.layerTypes.decal then
    im.SameLine()
    local isHighlighted = (api.getHighlightedLayer() == layer)
    if editor.uiIconImageButton(
      isHighlighted and editor.icons.simobject_pointlight or editor.icons.lightbulb_outline,
      im.ImVec2(tool.getIconSize(), tool.getIconSize()),
      isHighlighted and editor.color.beamng.Value or nil, nil, nil, string.format("highlightLayerButton##%s_%s", guiId, layer.uid)
    ) then

      if not permLayerHighlight or layer ~= permLayerHighlight then
        permLayerHighlight = layer
      else
        permLayerHighlight = nil
        api.disableDecalHighlighting()
      end
    end
    im.tooltip("Hover to highlight decal.\nLMB to permanently highlight decal")
    if im.IsItemHovered() then
      layerHoverStateA[guiId] = layer
    end
  end

  -- DEBUG
  if editor.getPreference("dynamicDecalsTool.general.debug") then
    im.SameLine()
    if im.Button(string.format("dump##%s", layer.uid)) then
      dump(layer)
    end
    im.SameLine()
    im.TextUnformatted(layer.uid)
  end

  if layer.status then
    if layer.status.missingTextureFiles then
      im.SameLine()
      editor.uiIconImageButton(editor.icons.warning, im.ImVec2(tool.getIconSize(), tool.getIconSize()), editor.color.warning.Value, nil, nil, string.format("missingTextureFiles", guiId, layer.uid))
      im.tooltip("Missing texture files: " .. dumps(layer.status.missingTextureFiles))
    end

    if layer.status.missingFontFile then
      im.SameLine()
      editor.uiIconImageButton(editor.icons.warning, im.ImVec2(tool.getIconSize(), tool.getIconSize()), editor.color.warning.Value, nil, nil, string.format("missingFontFile", guiId, layer.uid))
      im.tooltip("Missing font file: " .. layer.status.missingFontFile.path)
    end

    im.SameLine()
    local text = string.format("%.4fs", layer.status.bakingTime or -99)
    local textWidth = im.CalcTextSize(text).x
    im.SetCursorPosX(im.GetCursorPosX() + im.GetContentRegionAvailWidth() - textWidth)
    im.TextColored(editor.color.warning.Value, text)
  end

  -- LAYER MASK
  if layer.mask then
    im.SetCursorPosX(indentedCursorPosX)
    if im.Checkbox(string.format("##%s_%s_%s", layer.uid, guiId, "layerMaskEnabled"), editor.getTempBool_BoolBool(layer.mask.enabled)) then
      if editor.getTempBool_BoolBool() then
        local layerCopy = deepcopy(layer)
        layerCopy.mask.enabled = true
        api.setLayer(layerCopy, true)
      else
        local layerCopy = deepcopy(layer)
        layerCopy.mask.enabled = false
        api.setLayer(layerCopy, true)
      end
    end
    im.tooltip(layer.mask.enabled and "Disable layer mask" or "Enable layer mask")
    im.SameLine()
    if editor.uiIconImageButton(editor.icons.delete, im.ImVec2(tool.getIconSize(), tool.getIconSize()), nil, nil, nil, string.format("%s_%s", layer.uid, "removeLayerMask")) then
      local layerCopy = deepcopy(layer)
      layerCopy.mask = nil
      api.setLayer(layerCopy, true)
    end
    im.tooltip("Remove layer mask")
    im.SameLine()
    if im.TreeNodeEx1(string.format("Layer Mask##%s_%s", layer.uid, guiId)) then
      for k, maskLayer in ipairs(layer.mask.layers) do
        im.SetCursorPosX(indentedCursorPosX)
        local indentWidth = 2 * im.GetStyle().IndentSpacing
        im.Indent(indentWidth)
        if im.Checkbox(string.format("##%s_%s_%s_%d", layer.uid, guiId, "layerMaskLayerEnabled", k), editor.getTempBool_BoolBool(maskLayer.enabled)) then
          local layerCopy = deepcopy(layer)
          layerCopy.mask.layers[k].enabled = editor.getTempBool_BoolBool()
          api.setLayer(layerCopy, true)
        end
        im.SameLine()

        if selectionData and selectionData[maskLayer.uid] then
          if editor.uiIconImageButton(editor.icons.near_me, im.ImVec2(tool.getIconSize(), tool.getIconSize()), editor.color.beamng.Value, nil, nil, string.format("select##%s_%s", guiId, layer.uid)) then
            selection.deselectLayer(maskLayer.uid)
          end
          im.tooltip("Unselect layer mask layer")
        else
          if editor.uiIconImageButton(editor.icons.near_me, im.ImVec2(tool.getIconSize(), tool.getIconSize()), nil, nil, nil, string.format("select##%s_%s", guiId, layer.uid)) then
            selection.selectLayer(maskLayer.uid)
          end
          im.tooltip("Select layer mask layer")
        end
        im.SameLine()

        if editor.uiIconImageButton(editor.icons.delete, im.ImVec2(tool.getIconSize(), tool.getIconSize()), nil, nil, nil, string.format("%s_%d_%s", layer.uid, k, "removeLayerMaskLayer")) then
          local layerCopy = deepcopy(layer)
          table.remove(layerCopy.mask.layers, k)
          if #layerCopy.mask.layers == 0 then
            layerCopy.mask = nil
          end
          api.setLayer(layerCopy, true)
        end
        im.tooltip("Remove layer mask layer")
        im.SameLine()

        im.TextUnformatted(string.format("type: %s", api.layerTypesMap[maskLayer.type]))

        if editor.getPreference("dynamicDecalsTool.general.debug") then
          im.SameLine()
          if im.Button(string.format("Dump##%s_%s_%s_%d", layer.uid, guiId, "layerMaskLayerDumbButton", k)) then
            dump(maskLayer)
          end
        end

        im.SameLine()
        local isHighlighted = (api.getHighlightedLayer() == maskLayer)
        if editor.uiIconImageButton(
          isHighlighted and editor.icons.simobject_pointlight or editor.icons.lightbulb_outline,
          im.ImVec2(tool.getIconSize(), tool.getIconSize()),
          isHighlighted and editor.color.beamng.Value or nil, nil, nil, string.format("highlightLayerMaskButton##%s_%s_%s_%d", layer.uid, guiId, "layerMaskLayerHighlightButton", k)) then

          if not permLayerHighlight or maskLayer ~= permLayerHighlight then
            permLayerHighlight = maskLayer
          else
            permLayerHighlight = nil
            api.disableDecalHighlighting()
          end
        end
        im.tooltip("Hover to highlight layer mask decal.\nLMB to permanently highlight layer mask decal")
        if im.IsItemHovered() then
          layerMaskHoverStateA[guiId] = maskLayer
        end

        im.Unindent(indentWidth)
      end
      im.TreePop()
    end
    im.tooltip("RMB to open layer mask context menu")
    if im.IsItemClicked(1) then
      im.OpenPopup(layerMaskContextMenuPopupName)
    end
  end

  -- drag drop target: first child
  if (layerLevel < maxLayerDepth) and (layer.children and #layer.children == 0) then
    im.Indent()
    -- name, layer, guiId, to, toParentUid, id, additionalCheckFn, dbgColor
    layerDragDropTarget("first_child", layer, guiId, 1, layer.uid, k, nil, {1,0,0,1})
    im.Unindent()
  end

  if layer.children and #layer.children > 0 then
    if im.TreeNodeEx1(string.format("Children##%s_%s", layer.uid, guiId), im.TreeNodeFlags_DefaultOpen) then

      if editor.getPreference("dynamicDecalsTool.layerStack.topToBottomLayerStructure") then
        for k = #layer.children, 1, -1 do

          -- drag drop target: before child layer
          -- name, layer, guiId, to, toParentUid, id, additionalCheckFn, dbgColor
          layerDragDropTarget("child_before", layer, guiId, k, layer.uid, k, function(from, fromParentUid, to, toParentUid)
            if ((fromParentUid == toParentUid) and (from > to)) or (fromParentUid ~= toParentUid) then to = to + 1 end
            return from, fromParentUid, to, toParentUid
          end, {1,1,0,1})

          layerElement(k, layer.children[k], guiId, layer.uid, layer.children, layerLevel + 1)
        end

        -- drag drop target: after last child element
        -- name, layer, guiId, to, toParentUid, id, additionalCheckFn, dbgColor
        layerDragDropTarget("child_after_last", layer, guiId, 1, layer.uid, k, nil, {0,1,1,1})

      else -- BOTTOM TO TOP LAYER STRUCTURE [CHILDREN]
        for k, child in ipairs(layer.children) do

          -- drag drop target: before child layer
          -- name, layer, guiId, to, toParentUid, id, additionalCheckFn, dbgColor
          layerDragDropTarget("child_before", layer, guiId, k, layer.uid, k, function(from, fromParentUid, to, toParentUid)
            if ((fromParentUid == toParentUid) and (from < to)) or (fromParentUid ~= toParentUid) then to = to - 1 end
            return from, fromParentUid, to, toParentUid
          end, {1,1,0,1})

          layerElement(k, child, guiId, layer.uid, layer.children, layerLevel + 1)

          -- drag drop target: after last child element
          -- name, layer, guiId, to, toParentUid, id, additionalCheckFn, dbgColor
          layerDragDropTarget("child_before", layer, guiId, nil, layer.uid, 0, nil, {0,1,1,1})

        end
      end
      im.TreePop()
    end
  end

  -- Disabled for now since undo/redo won't work properly here due to the fact that we work on the reference of the layer
  -- directly hence the 'from' state will represent the new value already.
  -- It works flawlessly when using the button to display the data in the inspector since we create a deepcopy of the object beforehand.
  -- if im.TreeNode1("Inspect##" .. k) then
  --   inspectLayerGui(layer, "layerStack")
  --   im.TreePop()
  -- end
  if not parentUid then
    im.Separator()
  end
end

local function sectionGui(guiId)
  local vehicleObj = getPlayerVehicle(0)
  layerHoverStateA[guiId] = nil
  layerMaskHoverStateA[guiId] = nil

  if permLayerHighlight and not api.getHighlightedLayer() then
    api.highlightLayer(permLayerHighlight)
  end

  if im.BeginPopup("ClearLayerStackPopup") then
    im.TextColored(editor.color.warning.Value , "Do you really want to clear the baked textures? This will also wipe the layer stack!")
    if im.Button("Cancel") then
      im.CloseCurrentPopup()
    end
    im.SameLine()
    if im.Button("Ok") then
      selection.deselectLayer()
      api.clearLayerStack()
      im.CloseCurrentPopup()
    end
    im.EndPopup()
  end

  if im.Button("Add Fill Layer") then
    fill.openAddLayerWindow()
  end
  im.SameLine()
  if im.GetContentRegionAvailWidth() <= (im.GetStyle().ItemSpacing.x + 2*im.GetStyle().FramePadding.x + im.CalcTextSize("Add Texture Fill Layer").x - im.GetStyle().WindowPadding.x) then
    im.NewLine()
  end

  if im.Button("Add Texture Fill Layer") then
    textureFill.openAddLayerWindow()
  end
  im.SameLine()
  if im.GetContentRegionAvailWidth() <= (im.GetStyle().ItemSpacing.x + 2*im.GetStyle().FramePadding.x + im.CalcTextSize("Add Group").x - im.GetStyle().WindowPadding.x) then
    im.NewLine()
  end

  if im.Button("Add Group") then
    api.addGroup()
  end
  im.tooltip("Adds an empty group layer to the stack")
  im.SameLine()
  if im.GetContentRegionAvailWidth() <= (im.GetStyle().ItemSpacing.x + 2*im.GetStyle().FramePadding.x + im.CalcTextSize("Add Linked Set Layer").x - im.GetStyle().WindowPadding.x) then
    im.NewLine()
  end

  if im.Button("Add Linked Set Layer") then
    api.addLinkedSet()
  end
  im.tooltip("Add Linked Set Layer")

  im.Separator()
  if im.Button("Clear Layer Stack") then
    im.OpenPopup("ClearLayerStackPopup")
  end

  im.SameLine()
  local layerCountText = string.format("Layer count: %d", api.getLayerCount())

  if im.GetContentRegionAvailWidth() < im.CalcTextSize(layerCountText).x then
    im.NewLine()
    im.TextUnformatted(layerCountText)
  else
    editor.uiTextUnformattedRightAlign(layerCountText)
  end

  im.Separator()

  if editor.getPreference("dynamicDecalsTool.layerStack.topToBottomLayerStructure") then
    local topLayerText = "↓ Top Layer ↓"
    local topLayerTextSize = im.CalcTextSize(topLayerText)
    im.SetCursorPosX(im.GetCursorPosX() +  im.GetContentRegionAvailWidth() / 2 - topLayerTextSize.x / 2)
    im.TextUnformatted(topLayerText)

    im.Separator()

    local layerStackCount = #api.getLayerStack()
    for k = layerStackCount, 1, -1 do
      local layer = api.getLayerById(k)

      -- drag drop target: before layer
      -- name, layer, guiId, to, toParentUid, id, additionalCheckFn, dbgColor
      layerDragDropTarget("before", layer, guiId, k, nil, k,
        function(from, fromParentUid, to, toParentUid)
          if ((fromParentUid == toParentUid) and (from > to)) or (fromParentUid ~= toParentUid) then to = to + 1 end
          return from, fromParentUid, to, toParentUid
        end,
        {0,0,1,1}
      )

      layerElement(k, layer, guiId, nil, api.getLayerStack(), 0)
    end

    -- drag drop target: after last layer
    -- name, layer, guiId, to, toParentUid, id, additionalCheckFn, dbgColor
    layerDragDropTarget("after_last", {uid = "XXXX-XXXX"}, guiId, 1, nil, 0, nil, {1,0,1,1})

    im.Separator()
    local bottomLayerText = "↑ Bottom Layer ↑"
    local bottomLayerTextSize = im.CalcTextSize(bottomLayerText)
    im.SetCursorPosX(im.GetCursorPosX() +  im.GetContentRegionAvailWidth() / 2 - bottomLayerTextSize.x / 2)
    im.TextUnformatted(bottomLayerText)

  else -- BOTTOM TO TOP LAYER STRUCTURE
    local bottomLayerText = "↓ Bottom Layer ↓"
    local bottomLayerTextSize = im.CalcTextSize(bottomLayerText)
    im.SetCursorPosX(im.GetCursorPosX() +  im.GetContentRegionAvailWidth() / 2 - bottomLayerTextSize.x / 2)
    im.TextUnformatted(bottomLayerText)
    im.Separator()

    for k, layer in ipairs(api.getLayerStack()) do

      -- drag drop target: before layer
      if dragging then
        if editor.getPreference("dynamicDecalsTool.general.debug") then
          im.TextUnformatted(tostring(k))
          layerElementDragDropTargetDebug({1,0,0,1})
        end
        im.BeginChild1(string.format("##%s_%s_%s", guiId, layer.uid, "before"), im.ImVec2(0, layerDropHeight), true)
        im.EndChild()
        if im.BeginDragDropTarget() then
          local payload = im.AcceptDragDropPayload(layerDragDropType)
          if payload~=nil then
            assert(payload.DataSize == ffi.sizeof(payloadSize))
            local data = jsonDecode(ffi.string(payload.Data))
            local from = data.from
            local fromParentUid = data.fromParentUid
            local to = k
            local toParentUid = nil
            if editor.getPreference("dynamicDecalsTool.general.debug") then
              print(string.format("drag drop: before 1, from: %d, fromParentUid: %s, to: %d, toParentUid: %s", from or -1, fromParentUid or "nil", to or -1, toParentUid or "nil"))
            end
            -- the dragged layer is above the dropped layer, we need to alter the logic a bit
            if ((fromParentUid == toParentUid) and (from < to)) or (fromParentUid ~= toParentUid) then to = to - 1 end
            if editor.getPreference("dynamicDecalsTool.general.debug") then
              print(string.format("drag drop: before 2, from: %d, fromParentUid: %s, to: %d, toParentUid: %s", from or -1, fromParentUid or "nil", to or -1, toParentUid or "nil"))
            end
            if (from ~= to) or (fromParentUid ~= toParentUid) then
              api.moveLayer(from, fromParentUid, to, toParentUid)
            end
          end
          im.EndDragDropTarget()
        end
      end

      layerElement(k, layer, guiId, nil, api.getLayerStack(), 0)
    end

    -- drag drop target: after last layer
    if dragging then
      if editor.getPreference("dynamicDecalsTool.general.debug") then
        im.TextUnformatted(tostring(1))
        layerElementDragDropTargetDebug({0,1,0,1})
      end
      im.BeginChild1(string.format("##%s_%s_%s", guiId, "XXXX-XXXX", "after_last"), im.ImVec2(0, layerDropHeight), true)
      im.EndChild()
      if im.BeginDragDropTarget() then
        local payload = im.AcceptDragDropPayload(layerDragDropType)
        if payload~=nil then
          assert(payload.DataSize == ffi.sizeof(payloadSize))
          local data = jsonDecode(ffi.string(payload.Data))
          local from = data.from
          local fromParentUid = data.fromParentUid
          local to = #api.getLayerStack()
          local toParentUid = nil
          if editor.getPreference("dynamicDecalsTool.general.debug") then
            print(string.format("drag drop: after_last, from: %d, fromParentUid: %s, to: %d, toParentUid: %s", from or -1, fromParentUid or "nil", to or -1, toParentUid or "nil"))
          end
          if (from ~= to) or (fromParentUid ~= toParentUid) then
            api.moveLayer(from, fromParentUid, to, toParentUid)
          end
        end
        im.EndDragDropTarget()
      end
    end

    im.Separator()
    local topLayerText = "↑ Top Layer ↑"
    local topLayerTextSize = im.CalcTextSize(topLayerText)
    im.SetCursorPosX(im.GetCursorPosX() +  im.GetContentRegionAvailWidth() / 2 - topLayerTextSize.x / 2)
    im.TextUnformatted(topLayerText)
  end

  if dragging and im.IsMouseReleased(0) then
    dragging = false
  end

  -- using tables with guiId as keys for layerHoverStateA etc. otherwise sections and windows would incorrectly set the state next frame
  if layerHoverStateA[guiId] and layerHoverStateA[guiId] ~= layerHoverStateB[guiId] then -- user started hovering layer highlight button
    api.highlightLayer(layerHoverStateA[guiId])
  elseif layerHoverStateB[guiId] and layerHoverStateB[guiId] ~= layerHoverStateA[guiId] then -- user stoppedhovering layer highlight button
    api.disableDecalHighlighting()
  end
  layerHoverStateB[guiId] = layerHoverStateA[guiId]

  if layerMaskHoverStateA[guiId] and layerMaskHoverStateA[guiId] ~= layerMaskHoverStateB[guiId] then -- user started hovering layer mask highlight button
    api.highlightLayer(layerMaskHoverStateA[guiId])
  elseif layerMaskHoverStateB[guiId] and layerMaskHoverStateB[guiId] ~= layerMaskHoverStateA[guiId] then -- user stoppedhovering layer mask highlight button
    api.disableDecalHighlighting()
  end
  layerMaskHoverStateB[guiId] = layerMaskHoverStateA[guiId]
end

local function layerIconColorPrefGui()
  if im.TreeNodeEx1("Layer Icon Colors", im.TreeNodeFlags_DefaultOpen) then
    local data = editor.getPreference("dynamicDecalsTool.layerStack.layerTypeIconColor")
    for name, color in pairs(data) do
      if im.ColorEdit4("##layerIconColor_" .. tostring(name), editor.getTempFloatArray4_TableTable(color), im.flags(im.ColorEditFlags_NoInputs, im.ColorEditFlags_AlphaPreview)) then
        data[name] = editor.getTempFloatArray4_TableTable()
        editor.setPreference("dynamicDecalsTool.layerStack.layerTypeIconColor", data)
      end
      im.SameLine()

      im.TextUnformatted(helper.splitAndCapitalizeCamelCase(name) .. " Layer")
    end
    im.TreePop()
  end
end

local function registerEditorPreferences(prefsRegistry)
  prefsRegistry:registerSubCategory("dynamicDecalsTool", "layerStack", nil, {
    {topToBottomLayerStructure = {"bool", true, "Whether the tool displays the layers in top-to-bottom order or bottom-to-top order."}},
    {layerTypeIconColor = {"table",
      {
        ["decal"] = {1.0, 0.0, 0.0, 1.0},
        ["fill"] = {0.0, 1.0, 0.0, 1.0},
        ["textureFill"] = {0.0, 0.0, 1.0, 1.0},
        ["group"] = {1.0, 1.0, 0.0, 1.0},
        ["brushStroke"] = {0.0, 1.0, 1.0, 1.0},
        ["path"] = {1.0, 0.0, 1.0, 1.0},
        ["linkedSet"] = {1.0, 0.5, 0.0, 1.0},
      }, "", nil, nil, nil, nil, nil, function(cat, subCat, item)
        layerIconColorPrefGui()
    end}},
  })
end

local function editorPreferenceValueChanged(path, value)

end

local function documentationGui(docsSection)
  im.PushTextWrapPos(im.GetContentRegionAvailWidth())
  im.TextUnformatted([[
The Layer Stack is a vital component, providing an organized overview of all layers used in your design.

It displays each layer, allowing you to easily select, toggle visibility, duplicate, add and edit masks and rearrange layers.
The layer type and name are clearly displayed, streamlining the process of managing your livery composition.
With this intuitive interface, you can effortlessly navigate and control the layer hierarchy to achieve your desired visual effects.
]])
  im.PopTextWrapPos()
end

local function setup(tool_in)
  tool = tool_in
  api = extensions.editor_api_dynamicDecals
  fill = extensions.editor_dynamicDecals_layerTypes_fill
  textureFill = extensions.editor_dynamicDecals_layerTypes_textureFill
  gizmo = extensions.editor_dynamicDecals_gizmo
  selection = extensions.editor_dynamicDecals_selection
  helper = extensions.editor_dynamicDecals_helper
  docs = extensions.editor_dynamicDecals_docs

  -- tool.registerSection("Decal Stack / Layers", sectionGui, 140, true, {size = im.ImVec2(320, 640)}, {
  --   {icon = editor.icons.help_outline, tooltip = "Docs", fn = function() docs.selectSection("Layer Stack") end},
  -- })
  tool.registerSection("Decal Stack / Layers", sectionGui, 140, true, nil, {
    {icon = editor.icons.help_outline, tooltip = "Docs", fn = function() docs.selectSection("Layer Stack") end},
  })
  docs.register({section = {"Layer Stack"}, guiFn = documentationGui})
end

local function registerLayerElementGui()

end

M.registerLayerElementGui = registerLayerElementGui

M.registerEditorPreferences = registerEditorPreferences
M.editorPreferenceValueChanged = editorPreferenceValueChanged
M.setup = setup

return M