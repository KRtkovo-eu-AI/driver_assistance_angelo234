-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local imgui = Engine.imgui -- shortcut to prevent lookups all the time
local sb = require('string.buffer')

return function(M)
  M.uiscale = {[0] = 1}

  --default values
  M.plotLineColor = 4288453788
  M.plotLineBgColor = 2323270185

  --=== struct ImVec2 ===
  function M.ImVec2(x, y)
    return imgui.ImVec2(x, y)
  end

  function M.ImVec2Ptr(x, y)
    return M.ImVec2(x, y)
  end
  --===
  --=== struct ImVec4 ===
  function M.ImVec4(x, y, z, w)
    return imgui.ImVec4(x, y, z, w)
  end

  function M.ImVec4Ptr(x, y, z, w)
    return M.ImVec4(x, y, z, w)
  end
  --===

  function M.Bool(x) return x end
  function M.BoolPtr(x)
    if x == 0 then x = false end
    if x == 1 then x = true end
    return {[0] = x}
  end
  function M.Int(x) return x end
  function M.IntPtr(x) return {[0] = x} end
  function M.Float(x) return x end
  function M.FloatPtr(x) return {[0] = x} end
  function M.Double(x) return x end
  function M.DoublePtr(x) return {[0] = x} end
  function M.ArrayChar(len, val)
    local buf = sb.new()
    if val then
      buf:put(val)
      len = len - #buf
    end
    if len > 0 then
      buf:reserve(len)
      buf:commit(len)
    end
    return buf
  end
  function M.ArrayInt(size)
    local x = table.new(size - 1, 0)
    for i = 0, size - 1 do
      x[i] = 0
    end
    return x
  end
  function M.ArrayFloat(size)
    local x = table.new(size - 1, 0)
    for i = 0, size - 1 do
      x[i] = 0
    end
    return x
  end

  function M.BoolTrue() return true end
  function M.BoolFalse() return false end

  -- custom constructors
  function M.ImGuiTextFilter(default_filter)
    local res = imgui.ImGuiTextFilter()
    -- res.default_filter = default_filter
    return res
  end

  function M.ImGuiTextFilterPtr(default_filter)
    local res = imgui.ImGuiTextFilter()
    -- res[0].default_filter = default_filter
    return res
  end

  function M.ImDrawList(shared_data)
    local res = imgui.ImDrawList()
    res.shared_data = shared_data
    return res
  end

  function M.ImDrawListPtr(shared_data)
    local res = imgui.ImDrawList()
    res[0].shared_data = shared_data
    return res
  end

  function M.ArrayFloatByTbl(tbl)
    return tbl
  end

  function M.TableToArrayFloat(tbl)
    return tbl
  end

  function M.ImColorByRGB(r, g, b, a)
    local res = imgui.ImColor(r, g, b, a)
    local sc = 1/255
    res.Value = M.ImVec4(r * sc, g * sc, b * sc, (a or 255) * sc)
    return res
  end

  function M.Begin(string_name, bool_p_open, ImGuiWindowFlags_flags)
    -- bool_p_open is optional and can be nil
    if ImGuiWindowFlags_flags == nil then ImGuiWindowFlags_flags = M.WindowFlags_NoFocusOnAppearing end
    if string_name == nil then log("E", "", "Parameter 'string_name' of function 'Begin' cannot be nil, as the c type is 'const char *'") ; return end
    return imgui.Begin(string_name, bool_p_open, ImGuiWindowFlags_flags)
  end

  --
  function M.InputText(label, buf, buf_size, flags, callback, user_data, editEnded)
    if not buf_size then buf_size = 2^32 end
    if not flags then flags = 0 end

    return imgui.InputText(label, buf, buf_size, flags, nil, user_data)
  end

  function M.InputTextMultiline(label, buf, buf_size, size, flags, callback, user_data)
    if not buf_size then buf_size = 2^32 end
    if not size then size = M.ImVec2(0,0) end
    if not flags then flags = 0 end

    return imgui.InputTextMultiline(label, buf, buf_size, size, flags, nil, user_data)
  end

  function M.Combo1(label, current_item, items, items_count, popup_max_height_in_items)
    if popup_max_height_in_items == nil then popup_max_height_in_items = -1 end
    if items_count == nil then items_count = M.GetLengthArrayCharPtr(items) end
    return imgui.Combo1(label, current_item, items, items_count, popup_max_height_in_items)
  end

  function M.PushFont2(index)
    return imgui.PushFont2(index)
  end

  function M.PushFont3(uniqueId)
    return imgui.PushFont3(uniqueId)
  end

  function M.TextGlyph(unicode)
    return imgui.TextGlyph(unicode)
  end

  function M.IoFontsGetCount()
    return imgui.IoFontsGetCount()
  end

  function M.IoFontsGetName(index)
    return imgui.IoFontsGetName(index)
  end

  function M.SetDefaultFont(index)
    return imgui.SetDefaultFont(index)
  end

  function M.OpenPopup(string_str_id, ImGuiPopupFlags_popup_flags)
    return M.OpenPopup1(string_str_id, ImGuiPopupFlags_popup_flags)
  end

  -- GetKeyIndex() is a no-op since 1.87
  function M.GetKeyIndex(index)
    return index
  end

  function M.SetDragDropPayload(string_type, void_data, size_t_sz, ImGuiCond_cond)
    if ImGuiCond_cond == nil then ImGuiCond_cond = 0 end
    if string_type == nil then log("E", "", "Parameter 'string_type' of function 'SetDragDropPayload' cannot be nil, as the c type is 'const char *'") ; return end
    if void_data == nil then log("E", "", "Parameter 'void_data' of function 'SetDragDropPayload' cannot be nil, as the c type is 'const char *'") ; return end
    return imgui.SetDragDropPayload(string_type, void_data, size_t_sz, ImGuiCond_cond)
  end

  function M.GetImGuiIO_FontAllowUserScaling() return imgui.GetIO().FontAllowUserScaling end

  function M.ImGuiIO_FontGlobalScale(io, value) return io.FontGlobalScale end

  function M.ImTextureHandler(path)
    local res = imgui.ImTextureHandler()
    res:setID(path)
    return res
  end

  function M.ImTextureHandlerIsCached(path)
    return imgui.ImTextureHandler.isCached(path)
  end

  -- HELPER
  function M.ArraySize(arr) return #arr end
  function M.GetLengthArrayFloat(array) return #array end
  function M.GetLengthArrayCharPtr(array) return #array end
  function M.ArrayCharPtrByTbl(tbl) return tbl end

  -- WRAPPER
  -- Context creation and access
  function M.GetMainContext() return imgui.GetMainContext() end

  -- Helper functions
    -- Imgui Helper

  function M.ShowHelpMarker(desc, sameLine)
    if sameLine == true then M.SameLine() end
    M.TextDisabled("(?)")
    if M.IsItemHovered() then
      M.BeginTooltip()
      M.PushTextWrapPos(M.GetFontSize() * 35.0)
      M.TextUnformatted(desc)
      M.PopTextWrapPos()
      M.EndTooltip()
    end
  end

  function M.tooltip(message)
    if M.IsItemHovered() then
      M.SetTooltip(message)
    end
  end

  function M.ImGuiTextFilter(default_filter)
    local res = imgui.ImGuiTextFilter()
    --res.default_filter = default_filter
    return res
  end

  function M.DockBuilderDockWindow(window_name, node_id) imgui.DockBuilderDockWindow(window_name, node_id) end

  function M.DockBuilderAddNode2(node_id, flags)
    if not flags then flags = 0 end
    return imgui.DockBuilderAddNode2(node_id, flags)
  end

  function M.DockBuilderSplitNode(node_id, split_dir, size_ratio_for_node_at_dir, out_id_dir, out_id_other) return imgui.DockBuilderSplitNode(node_id, split_dir, size_ratio_for_node_at_dir, out_id_dir, out_id_other) end

  function M.DockBuilderFinish(node_id) imgui.DockBuilderFinish(node_id) end

  function M.BeginDisabled(disable)
    if disable == nil then disable = true end
    imgui.BeginDisabled(disable)
  end

  function M.EndDisabled()
    imgui.EndDisabled()
  end

  function M.TextFilter_GetInputBuf(filter)
    return filter:GetInputBuf()
  end

  function M.TextFilter_SetInputBuf(filter, text)
    return filter:SetInputBuf(text)
  end

  function M.getMonitorIndex()
    return imgui.getMonitorIndex()
  end

  function M.getCurrentMonitorSize()
    local vec2 = M.ImVec2(0, 0)
    imgui.getCurrentMonitorSize(vec2)
    return vec2
  end

  function M.loadIniSettingsFromDisk(filename)
    imgui.LoadIniSettingsFromDisk(filename)
  end

  function M.ClearActiveID()
    imgui.ClearActiveID()
  end

  function M.saveIniSettingsToDisk(filename)
    imgui.SaveIniSettingsToDisk(filename)
  end

  -- Wrapper function, this was removed in the latest imgui version
  function M.GetContentRegionAvailWidth()
     return M.GetContentRegionAvail().x
  end

  local matchColor = M.ImVec4(1,0.5,0,1)
  function M.HighlightText(label, highlightText)
    M.PushStyleVar2(M.StyleVar_ItemSpacing, M.ImVec2(0, 0))
    if highlightText == "" then
      M.TextColored(matchColor,label)
    else
      local pos1 = 1
      local pos2 = 0
      local labelLower = label:lower()
      local highlightLower = highlightText:lower()
      local highlightLowerLen = string.len(highlightLower) - 1
      for i = 0, 6 do -- up to 6 matches overall ...
        pos2 = labelLower:find(highlightLower, pos1, true)
        if not pos2 then
          M.Text(label:sub(pos1))
          break
        elseif pos1 < pos2 then
          M.Text(label:sub(pos1, pos2 - 1))
          M.SameLine()
        end

        local pos3 = pos2 + highlightLowerLen
        M.TextColored(matchColor, label:sub(pos2, pos3))
        M.SameLine()
        pos1 = pos3 + 1
      end
    end
    M.PopStyleVar()
  end

  function M.HighlightSelectable(label, highlightText, selected)
    local cursor = M.GetCursorPos()
    local width = M.GetContentRegionAvailWidth()
    M.BeginGroup()
    local x = M.GetCursorPosX()
    M.HighlightText(label, highlightText)
    local spacing = M.GetStyle().ItemSpacing
    M.SameLine()
    M.Dummy(M.ImVec2(width - M.GetCursorPosX(), 1))
    M.EndGroup()
    if M.IsItemHovered() then

      local itemSize = M.GetItemRectSize()
      M.ImDrawList_AddRectFilled(M.GetWindowDrawList(), M.ImVec2(cursor.x + M.GetWindowPos().x - 2,
                        cursor.y + M.GetWindowPos().y + (spacing.y/2) - 2 - M.GetScrollY()),
                        M.ImVec2(cursor.x + M.GetWindowPos().x + itemSize.x + (spacing.y/2),
                        cursor.y + M.GetWindowPos().y + itemSize.y + 2 - M.GetScrollY()),
                        M.GetColorU321(M.IsAnyMouseDown() and M.Col_HeaderActive or M.Col_HeaderHovered), 1, 1)

    elseif selected then
      local itemSize = M.GetItemRectSize()
      M.ImDrawList_AddRectFilled(M.GetWindowDrawList(), M.ImVec2(cursor.x + M.GetWindowPos().x - 2,
                        cursor.y + M.GetWindowPos().y + (spacing.y/2) - 2 - M.GetScrollY()),
                        M.ImVec2(cursor.x + M.GetWindowPos().x + itemSize.x + (spacing.y/2),
                        cursor.y + M.GetWindowPos().y + itemSize.y + 2 - M.GetScrollY()),
                        M.GetColorU321(M.Col_Header), 1, 1)
    end
  end

  local headerDefaultColor = M.ImVec4(1,0.6,0,8,0.75)
  function M.HeaderText(text, color)
    color = color or headerDefaultColor
    M.PushFont3("cairo_regular_medium")
    M.TextColored(color, text)
    M.PopFont()
  end

  -- # TODO: deprecated
  function M.SetItemAllowOverlap()
    imgui.SetItemAllowOverlap()
  end

  -- Knobs
  function M.Knob(string_label, floatPtr_value, float_v_min, float_v_max, float_speed, string_format, ImGuiKnobVariant_variant, float_size, ImGuiKnobFlags_flags, int_steps)
    if float_v_min == nil then float_v_min = 0 end
    if float_v_max == nil then float_v_max = 1 end
    if float_speed == nil then float_speed = 0 end
    if string_format == nil then string_format = "%.3f" end
    if ImGuiKnobVariant_variant == nil then ImGuiKnobVariant_variant = ImGuiKnobVariant_Tick end
    if float_size == nil then float_size = 0 end
    if ImGuiKnobFlags_flags == nil then ImGuiKnobFlags_flags = 0 end
    if int_steps == nil then int_steps = 10 end
    return imgui.Knob(string_label, floatPtr_value, float_v_min, float_v_max, float_speed, string_format, ImGuiKnobVariant_variant, float_size, ImGuiKnobFlags_flags, int_steps)
  end

  function M.KnobInt(string_label, intPtr_value, int_v_min, int_v_max, float_speed, string_format, ImGuiKnobVariant_variant, float_size, ImGuiKnobFlags_flags, int_steps)
    if int_v_min == nil then int_v_min = 0 end
    if int_v_max == nil then int_v_max = 1 end
    if float_speed == nil then float_speed = 0 end
    if string_format == nil then string_format = "%.3f" end
    if ImGuiKnobVariant_variant == nil then ImGuiKnobVariant_variant = ImGuiKnobVariant_Tick end
    if float_size == nil then float_size = 0 end
    if ImGuiKnobFlags_flags == nil then ImGuiKnobFlags_flags = 0 end
    if int_steps == nil then int_steps = 10 end
    return imgui.KnobInt(string_label, floatPtr_value, float_v_min, float_v_max, float_speed, string_format, ImGuiKnobVariant_variant, float_size, ImGuiKnobFlags_flags, int_steps)
  end

  M.KnobFlags_NoTitle = imgui.KnobFlags.NoTitle
  M.KnobFlags_NoInput = imgui.KnobFlags.NoInput
  M.KnobFlags_ValueTooltip = imgui.KnobFlags.ValueTooltip
  M.KnobFlags_DragHorizontal = imgui.KnobFlags.DragHorizontal

  M.KnobVariant_Tick = imgui.KnobVariant.Tick
  M.KnobVariant_Dot = imgui.KnobVariant.Dot
  M.KnobVariant_Wiper = imgui.KnobVariant.Wiper
  M.KnobVariant_WiperOnly = imgui.KnobVariant.WiperOnly
  M.KnobVariant_WiperDot = imgui.KnobVariant.WiperDot
  M.KnobVariant_Stepped = imgui.KnobVariant.Stepped
  M.KnobVariant_Space = imgui.KnobVariant.Space

end -- return end add things above, not below

