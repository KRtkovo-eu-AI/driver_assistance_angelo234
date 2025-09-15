-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = "editor_dynamicDecals_widgets"
local im = ui_imgui

-- reference to the editor tool, set in setup()
local tool = nil
local api = nil
local helper = nil

local highlightedWidgets = {}

local function lockRatioButton(property, widgetId)
  if property.lockRatio ~= nil then
    if editor.uiIconImageButton(editor.icons.lock, tool.getIconSizeVec2(), property.lockRatio and editor.color.beamng.Value or nil, nil, nil, string.format("LockRatioButton_%s", widgetId)) then
      property.lockRatio = not property.lockRatio
    end
    im.tooltip("Lock ratio")
    im.SameLine()
  end
end

-- BOOL
local function boolCheckboxWidget(value, property, guiId, editEnded, widgetId, widgetOptions)
  local changed = false

  local baseProp = property
  local ended = false

  if baseProp.default ~= nil then
    if editor.uiIconImageButton(editor.icons.refresh, tool.getIconSizeVec2(), nil, nil, nil, string.format("%s_resetButton", widgetId)) then
      property.value = baseProp.default
      changed = true
      ended = true
    end
    im.tooltip(string.format("Reset to default: %s", dumps(baseProp.default)))
    im.SameLine()
  end

  if im.Checkbox(widgetId, editor.getTempBool_BoolBool(value)) then
    local newVal = editor.getTempBool_BoolBool()
    property.value = newVal
    changed = true
  end
  ended = ended or im.IsItemDeactivatedAfterEdit()

  if editEnded then
    editEnded[0] = ended
  end

  if changed == true then
    return true
  end
end

local function defaultButton(widgetId, cbFunc, tooltipMsg)
  if editor.uiIconImageButton(editor.icons.refresh, tool.getIconSizeVec2(), nil, nil, nil, string.format("%s_resetButton", widgetId)) then
    cbFunc()
  end
  if tooltipMsg then
    im.tooltip(tooltipMsg)
  end
  im.SameLine()
end

-- INT
local function intInputWidget(value, property, guiId, editEnded, widgetId, widgetOptions)
  local changed = false

  local baseProp = property
  local ended = false

  if baseProp.default then
    if editor.uiIconImageButton(editor.icons.refresh, tool.getIconSizeVec2(), nil, nil, nil, string.format("%s_resetButton", widgetId)) then
      property.value = baseProp.default
      changed = true
      ended = true
    end
    if baseProp.options then
      im.tooltip(string.format("Reset to default: %d (%s)", baseProp.default, baseProp.options[baseProp.default + 1]))
    else
      im.tooltip(string.format("Reset to default: %s", dumps(baseProp.default)))
    end
    im.SameLine()
  end

  im.PushItemWidth(im.GetContentRegionAvailWidth())
  if im.InputInt(widgetId, editor.getTempInt_NumberNumber(baseProp.getMod and baseProp.getMod(value) or value), 1, 10) then
    local newVal = editor.getTempInt_NumberNumber()
    if newVal < baseProp.min then newVal = baseProp.min end
    if newVal > baseProp.max then newVal = baseProp.max end
    property.value = baseProp.setMod and baseProp.setMod(newVal) or newVal
    changed = true
  end
  ended = ended or im.IsItemDeactivatedAfterEdit()
  im.PopItemWidth()

  if editEnded then
    editEnded[0] = ended
  end

  if changed == true then
    return true
  end
end

local function intSliderWidget(value, property, guiId, editEnded, widgetId, widgetOptions)
  local changed = false

  local baseProp = property
  local ended = false

  if baseProp.default then
    if editor.uiIconImageButton(editor.icons.refresh, tool.getIconSizeVec2(), nil, nil, nil, string.format("%s_resetButton", widgetId)) then
      property.value = baseProp.default
      changed = true
      ended = true
    end
    if baseProp.options then
      im.tooltip(string.format("Reset to default: %d (%s)", baseProp.default, baseProp.options[baseProp.default + 1]))
    else
      im.tooltip(string.format("Reset to default: %s", dumps(baseProp.default)))
    end
    im.SameLine()
  end

  im.PushItemWidth(im.GetContentRegionAvailWidth())
  if im.SliderInt(widgetId, editor.getTempInt_NumberNumber(baseProp.getMod and baseProp.getMod(value) or value), baseProp.min, baseProp.max) then
    local newVal = editor.getTempInt_NumberNumber()
    property.value = baseProp.setMod and baseProp.setMod(newVal) or newVal
    changed = true
  end
  ended = ended or im.IsItemDeactivatedAfterEdit()
  if editor.getPreference("dynamicDecalsTool.general.debug") then
    im.tooltip(widgetId)
  end
  im.PopItemWidth()

  if editEnded then
    editEnded[0] = ended
  end

  if changed == true then
    return true
  end
end

local function intDragWidget(value, property, guiId, editEnded, widgetId, widgetOptions)
  local changed = false

  local baseProp = property
  local ended = false

  if baseProp.default then
    if editor.uiIconImageButton(editor.icons.refresh, tool.getIconSizeVec2(), nil, nil, nil, string.format("%s_resetButton", widgetId)) then
      property.value = baseProp.default
      changed = true
      ended = true
    end
    if baseProp.options then
      im.tooltip(string.format("Reset to default: %d (%s)", baseProp.default, baseProp.options[baseProp.default + 1]))
    else
      im.tooltip(string.format("Reset to default: %s", dumps(baseProp.default)))
    end
    im.SameLine()
  end

  im.PushItemWidth(im.GetContentRegionAvailWidth())
  if im.DragInt(widgetId, editor.getTempInt_NumberNumber(baseProp.getMod and baseProp.getMod(value) or value), baseProp.dragSpeed, baseProp.min, baseProp.max) then
    local newVal = editor.getTempInt_NumberNumber()
    property.value = baseProp.setMod and baseProp.setMod(newVal) or newVal
    changed = true
  end
  ended = ended or im.IsItemDeactivatedAfterEdit()
  if editor.getPreference("dynamicDecalsTool.general.debug") then
    im.tooltip(widgetId)
  end
  im.PopItemWidth()

  if editEnded then
    editEnded[0] = ended
  end

  if changed == true then
    return true
  end
end

local function intComboWidget(value, property, guiId, editEnded, widgetId, widgetOptions)
  local changed = false

  local baseProp = property
  local ended = false

  if baseProp.default then
    if editor.uiIconImageButton(editor.icons.refresh, tool.getIconSizeVec2(), nil, nil, nil, string.format("%s_resetButton", widgetId)) then
      property.value = baseProp.default
      changed = true
      ended = true
    end
    if baseProp.options then
      im.tooltip(string.format("Reset to default: %d (%s)", baseProp.default, baseProp.options[baseProp.default + 1]))
    else
      im.tooltip(string.format("Reset to default: %s", dumps(baseProp.default)))
    end
    im.SameLine()
  end

  local options = ""
  if baseProp.options then
    for _, option in ipairs(baseProp.options) do
      options = options .. option .. "\0"
    end
    options = options .. "\0"
  else
    for i = baseProp.min, baseProp.max do
      options = options .. tostring(i) .. "\0"
    end
    options = options .. "\0"
  end

  im.PushItemWidth(im.GetContentRegionAvailWidth() + (widgetOptions.widthMod and widgetOptions.widthMod or 0))
  if im.Combo2(widgetId, editor.getTempInt_NumberNumber(baseProp.getMod and baseProp.getMod(value) or value), options) then
    local newVal = editor.getTempInt_NumberNumber()
    property.value = baseProp.setMod and baseProp.setMod(newVal) or newVal
    changed = true
  end
  ended = ended or im.IsItemDeactivatedAfterEdit()
  if editor.getPreference("dynamicDecalsTool.general.debug") then
    im.tooltip(widgetId)
  end
  im.PopItemWidth()

  if editEnded then
    editEnded[0] = ended
  end

  if changed == true then
    if editEnded then
      editEnded[0] = true
    end
    return true
  end
end

-- FLOAT
local function floatInputWidget(value, property, guiId, editEnded, widgetId, widgetOptions)
  local changed = false

  local baseProp = property
  local ended = false

  if baseProp.default then
    if editor.uiIconImageButton(editor.icons.refresh, tool.getIconSizeVec2(), nil, nil, nil, string.format("%s_resetButton", widgetId)) then
      property.value = baseProp.default
      changed = true
      ended = true
    end
    im.tooltip(string.format("Reset to default: %s", dumps(baseProp.default)))
    im.SameLine()
  end

  im.PushItemWidth(im.GetContentRegionAvailWidth() + (widgetOptions.widthMod or 0))
  if im.InputFloat(widgetId, editor.getTempFloat_NumberNumber(baseProp.getMod and baseProp.getMod(value) or value), 1, 10, baseProp.format) then
    local newVal = editor.getTempFloat_NumberNumber()
    if newVal < baseProp.min then newVal = baseProp.min end
    if newVal > baseProp.max then newVal = baseProp.max end
    property.value = baseProp.setMod and baseProp.setMod(newVal) or newVal
    changed = true
  end
  ended = ended or im.IsItemDeactivatedAfterEdit()
  if editor.getPreference("dynamicDecalsTool.general.debug") then
    im.tooltip(widgetId)
  end
  im.PopItemWidth()

  if editEnded then
    editEnded[0] = ended
  end

  if changed == true then
    return true
  end
end

local function floatSliderWidget(value, property, guiId, editEnded, widgetId, widgetOptions)
  local changed = false

  local baseProp = property
  local ended = false

  if baseProp.default then
    if editor.uiIconImageButton(editor.icons.refresh, tool.getIconSizeVec2(), nil, nil, nil, string.format("%s_resetButton", widgetId)) then
      property.value = baseProp.default
      changed = true
      ended = true
    end
    im.tooltip(string.format("Reset to default: %s", dumps(baseProp.default)))
    im.SameLine()
  end

  im.PushItemWidth(im.GetContentRegionAvailWidth() + (widgetOptions.widthMod or 0))
  if im.SliderFloat(widgetId, editor.getTempFloat_NumberNumber(baseProp.getMod and baseProp.getMod(value) or value), baseProp.min, baseProp.max, baseProp.format) then
    local newVal = editor.getTempFloat_NumberNumber()
    property.value = baseProp.setMod and baseProp.setMod(newVal) or newVal
    changed = true
  end
  ended = ended or im.IsItemDeactivatedAfterEdit()
  if editor.getPreference("dynamicDecalsTool.general.debug") then
    im.tooltip(widgetId)
  end
  im.PopItemWidth()

  if editEnded then
    editEnded[0] = ended
  end

  if changed == true then
    return true
  end
end

local function floatDragWidget(value, property, guiId, editEnded, widgetId, widgetOptions)
  local changed = false

  local baseProp = property
  local ended = false

  if baseProp.default then
    if editor.uiIconImageButton(editor.icons.refresh, tool.getIconSizeVec2(), nil, nil, nil, string.format("%s_resetButton", widgetId)) then
      property.value = baseProp.default
      changed = true
      ended = true
    end
    im.tooltip(string.format("Reset to default: %s", dumps(baseProp.default)))
    im.SameLine()
  end
  im.PushItemWidth(im.GetContentRegionAvailWidth() + (widgetOptions.widthMod or 0))
  if im.DragFloat(widgetId, editor.getTempFloat_NumberNumber(baseProp.getMod and baseProp.getMod(value) or value), baseProp.dragSpeed, baseProp.min, baseProp.max, baseProp.format) then
    local newVal = editor.getTempFloat_NumberNumber()
    property.value = baseProp.setMod and baseProp.setMod(newVal) or newVal
    changed = true
  end
  ended = ended or im.IsItemDeactivatedAfterEdit()
  if editor.getPreference("dynamicDecalsTool.general.debug") then
    im.tooltip(widgetId)
  end
  im.PopItemWidth()

  if editEnded then
    editEnded[0] = ended
  end

  if changed == true then
    return true
  end
end

-- POINT2F
local function point2FInputWidget(value, property, guiId, editEnded, widgetId, widgetOptions)
  local changed = false

  local baseProp = property
  local ended = false

  if baseProp.default then
    if editor.uiIconImageButton(editor.icons.refresh, tool.getIconSizeVec2(), nil, nil, nil, string.format("%s_resetButton", widgetId)) then
      property.value = shallowcopy(baseProp.default)
      changed = true
      ended = true
    end
    im.tooltip(string.format("Reset to default: %s", dumps(baseProp.default)))
    im.SameLine()
  end

  lockRatioButton(property, widgetId)

  local width = (im.GetContentRegionAvailWidth() - im.GetStyle().ItemSpacing.x) / 2
  im.PushItemWidth(width)
  if im.InputFloat(string.format("%s_x", widgetId), editor.getTempFloat_NumberNumber(baseProp.getMod and baseProp.getMod(value[1]) or value[1]), 1, 10, baseProp.format) then
    local newVal = editor.getTempFloat_NumberNumber()
    if newVal < baseProp.min[1] then newVal = baseProp.min[1] end
    if newVal > baseProp.max[1] then newVal = baseProp.max[1] end
    property.value[1] = baseProp.setMod and baseProp.setMod(newVal) or newVal
    if property.lockRatio then
      property.value[2] = property.value[1]
    end
    changed = true
  end
  ended = ended or im.IsItemDeactivatedAfterEdit()
  if editor.getPreference("dynamicDecalsTool.general.debug") then
    im.tooltip(string.format("%s_x", widgetId))
  end
  im.PopItemWidth()

  im.SameLine()
  im.PushItemWidth(width)
  if im.InputFloat(string.format("%s_y", widgetId), editor.getTempFloat_NumberNumber(baseProp.getMod and baseProp.getMod(value[2]) or value[2]), 1, 10, baseProp.format) then
    local newVal = editor.getTempFloat_NumberNumber()
    if newVal < baseProp.min[2] then newVal = baseProp.min[2] end
    if newVal > baseProp.max[2] then newVal = baseProp.max[2] end
    property.value[2] = baseProp.setMod and baseProp.setMod(newVal) or newVal
    if property.lockRatio then
      property.value[1] = property.value[2]
    end
    changed = true
  end
  ended = ended or im.IsItemDeactivatedAfterEdit()
  if editor.getPreference("dynamicDecalsTool.general.debug") then
    im.tooltip(string.format("%s_y", widgetId))
  end
  im.PopItemWidth()

  if editEnded then
    editEnded[0] = ended
  end

  if changed == true then
    return true
  end
end

local function point2FSliderWidget(value, property, guiId, editEnded, widgetId, widgetOptions)
  local changed = false

  local baseProp = property
  local ended = false

  if baseProp.default then
    if editor.uiIconImageButton(editor.icons.refresh, tool.getIconSizeVec2(), nil, nil, nil, string.format("%s_resetButton", widgetId)) then
      property.value = shallowcopy(baseProp.default)
      changed = true
      ended = true
    end
    im.tooltip(string.format("Reset to default: %s", dumps(baseProp.default)))
    im.SameLine()
  end

  lockRatioButton(property, widgetId)

  local width = (im.GetContentRegionAvailWidth() - im.GetStyle().ItemSpacing.x) / 2
  im.PushItemWidth(width)
  if im.SliderFloat(string.format("%s_x", widgetId), editor.getTempFloat_NumberNumber(baseProp.getMod and baseProp.getMod(value[1]) or value[1]), baseProp.min and baseProp.min[1] or nil, baseProp.max and baseProp.max[1] or nil, baseProp.format) then
    local newVal = editor.getTempFloat_NumberNumber()
    property.value[1] = baseProp.setMod and baseProp.setMod(newVal) or newVal
    property.value[2] = value[2]
    if property.lockRatio then
      property.value[2] = property.value[1]
    end
    changed = true
  end
  ended = ended or im.IsItemDeactivatedAfterEdit()
  if editor.getPreference("dynamicDecalsTool.general.debug") then
    im.tooltip(string.format("%s_x", widgetId))
  end
  im.PopItemWidth()

  im.SameLine()
  im.PushItemWidth(width)
  if im.SliderFloat(string.format("%s_y", widgetId), editor.getTempFloat_NumberNumber(baseProp.getMod and baseProp.getMod(value[2]) or value[2]), baseProp.min and baseProp.min[2] or nil, baseProp.max and baseProp.max[2] or nil, baseProp.format) then
    local newVal = editor.getTempFloat_NumberNumber()
    property.value[1] = value[1]
    property.value[2] = baseProp.setMod and baseProp.setMod(newVal) or newVal
    if property.lockRatio then
      property.value[1] = property.value[2]
    end
    changed = true
  end
  ended = ended or im.IsItemDeactivatedAfterEdit()
  if editor.getPreference("dynamicDecalsTool.general.debug") then
    im.tooltip(string.format("%s_y", widgetId))
  end
  im.PopItemWidth()

  if editEnded then
    editEnded[0] = ended
  end

  if changed == true then
    return true
  end
end

local function point2FDragWidget(value, property, guiId, editEnded, widgetId, widgetOptions)
  local changed = false

  local baseProp = property
  local ended = false

  if baseProp.default then
    if editor.uiIconImageButton(editor.icons.refresh, tool.getIconSizeVec2(), nil, nil, nil, string.format("%s_resetButton", widgetId)) then
      property.value = shallowcopy(baseProp.default)
      changed = true
      ended = true
    end
    im.tooltip(string.format("Reset to default: %s", dumps(baseProp.default)))
    im.SameLine()
  end

  lockRatioButton(property, widgetId)

  local width = (im.GetContentRegionAvailWidth() - im.GetStyle().ItemSpacing.x) / 2
  im.PushItemWidth(width)
  if im.DragFloat(string.format("%s_x", widgetId), editor.getTempFloat_NumberNumber(baseProp.getMod and baseProp.getMod(value[1]) or value[1]), baseProp.dragSpeed, baseProp.min[1], baseProp.max[1], baseProp.format) then
    local newVal = editor.getTempFloat_NumberNumber()
    property.value[1] = baseProp.setMod and baseProp.setMod(newVal) or newVal
    if property.lockRatio then
      property.value[2] = property.value[1]
    end
    changed = true
  end
  ended = ended or im.IsItemDeactivatedAfterEdit()
  if editor.getPreference("dynamicDecalsTool.general.debug") then
    im.tooltip(string.format("%s_x", widgetId))
  end
  im.PopItemWidth()

  im.SameLine()
  im.PushItemWidth(width)
  if im.DragFloat(string.format("%s_y", widgetId), editor.getTempFloat_NumberNumber(baseProp.getMod and baseProp.getMod(value[2]) or value[2]), baseProp.dragSpeed, baseProp.min[2], baseProp.max[2], baseProp.format) then
    local newVal = editor.getTempFloat_NumberNumber()
    property.value[2] = baseProp.setMod and baseProp.setMod(newVal) or newVal
    if property.lockRatio then
      property.value[1] = property.value[2]
    end
    changed = true
  end
  ended = ended or im.IsItemDeactivatedAfterEdit()
  if editor.getPreference("dynamicDecalsTool.general.debug") then
    im.tooltip(string.format("%s_y", widgetId))
  end
  im.PopItemWidth()

  if editEnded then
    editEnded[0] = ended
  end

  if changed == true then
    return true
  end
end

-- POINT3F
local function point3FSliderWidget(value, property, guiId, editEnded, widgetId, widgetOptions)
  local changed = false

  local baseProp = property
  local ended = false

  if baseProp.default then
    if editor.uiIconImageButton(editor.icons.refresh, tool.getIconSizeVec2(), nil, nil, nil, string.format("%s_resetButton", widgetId)) then
      property.value = shallowcopy(baseProp.default)
      changed = true
      ended = true
    end
    im.tooltip(string.format("Reset to default: %s", dumps(baseProp.default)))
    im.SameLine()
  end

  lockRatioButton(property, widgetId)

  local width = (im.GetContentRegionAvailWidth() - 2 * im.GetStyle().ItemSpacing.x + (widgetOptions.widthMod and widgetOptions.widthMod or 0)) / 3
  im.PushItemWidth(width)
  if im.SliderFloat(string.format("%s_x", widgetId), editor.getTempFloat_NumberNumber(baseProp.getMod and baseProp.getMod(value[1]) or value[1]), baseProp.min and baseProp.min[1] or nil, baseProp.max and baseProp.max[1] or nil, baseProp.format) then
    local newVal = editor.getTempFloat_NumberNumber()
    property.value[1] = baseProp.setMod and baseProp.setMod(newVal) or newVal
    property.value[2] = value[2]
    property.value[3] = value[3]
    if property.lockRatio then
      property.value[2] = property.value[1]
      property.value[3] = property.value[1]
    end
    changed = true
  end
  ended = ended or im.IsItemDeactivatedAfterEdit()
  if editor.getPreference("dynamicDecalsTool.general.debug") then
    im.tooltip(string.format("%s_x", widgetId))
  end
  im.PopItemWidth()

  im.SameLine()
  im.PushItemWidth(width)
  if im.SliderFloat(string.format("%s_y", widgetId), editor.getTempFloat_NumberNumber(baseProp.getMod and baseProp.getMod(value[2]) or value[2]), baseProp.min and baseProp.min[2] or nil, baseProp.max and baseProp.max[2] or nil, baseProp.format) then
    local newVal = editor.getTempFloat_NumberNumber()
    property.value[1] = value[1]
    property.value[2] = baseProp.setMod and baseProp.setMod(newVal) or newVal
    property.value[3] = value[3]
    if property.lockRatio then
      property.value[1] = property.value[2]
      property.value[3] = property.value[2]
    end
    changed = true
  end
  ended = ended or im.IsItemDeactivatedAfterEdit()
  if editor.getPreference("dynamicDecalsTool.general.debug") then
    im.tooltip(string.format("%s_y", widgetId))
  end
  im.PopItemWidth()

  im.SameLine()
  im.PushItemWidth(width)
  if im.SliderFloat(string.format("%s_z", widgetId), editor.getTempFloat_NumberNumber(baseProp.getMod and baseProp.getMod(value[3]) or value[3]), baseProp.min and baseProp.min[3] or nil, baseProp.max and baseProp.max[3] or nil, baseProp.format) then
    local newVal = editor.getTempFloat_NumberNumber()
    property.value[1] = value[1]
    property.value[2] = value[2]
    property.value[3] = baseProp.setMod and baseProp.setMod(newVal) or newVal
    if property.lockRatio then
      property.value[1] = property.value[3]
      property.value[2] = property.value[3]
    end
    changed = true
  end
  ended = ended or im.IsItemDeactivatedAfterEdit()
  if editor.getPreference("dynamicDecalsTool.general.debug") then
    im.tooltip(string.format("%s_z", widgetId))
  end
  im.PopItemWidth()

  if editEnded then
    editEnded[0] = ended
  end

  if changed == true then
    return true
  end
end

local function point3FColorWidget(value, property, guiId, editEnded, widgetId, widgetOptions)
  local changed = false

  local baseProp = property
  local ended = false

  if baseProp.default then
    if editor.uiIconImageButton(editor.icons.refresh, tool.getIconSizeVec2(), nil, nil, nil, string.format("%s_resetButton", widgetId)) then
      property.value = shallowcopy(baseProp.default)
      changed = true
      ended = true
    end
    im.tooltip(string.format("Reset to default: %s", dumps(baseProp.default)))
    im.SameLine()
  end

  im.PushItemWidth(im.GetContentRegionAvailWidth())
  if im.ColorEdit3(widgetId, editor.getTempFloatArray3_TableTable(value)) then
    property.value = editor.getTempFloatArray3_TableTable()
    changed = true
  end
  ended = ended or im.IsItemDeactivatedAfterEdit()
  im.PopItemWidth()

  if editEnded then
    editEnded[0] = ended
  end

  if changed == true then
    return true
  end
end

-- POINT4F
local function point4FSliderWidget(value, property, guiId, editEnded, widgetId, widgetOptions)
  local changed = false

  local baseProp = property
  local ended = false

  if baseProp.default then
    if editor.uiIconImageButton(editor.icons.refresh, tool.getIconSizeVec2(), nil, nil, nil, string.format("%s_resetButton", widgetId)) then
      property.value = shallowcopy(baseProp.default)
      changed = true
      ended = true
    end
    im.tooltip(string.format("Reset to default: %s", dumps(baseProp.default)))
    im.SameLine()
  end

  lockRatioButton(property, widgetId)

  local width = (im.GetContentRegionAvailWidth() - 3 * im.GetStyle().ItemSpacing.x) / 4
  im.PushItemWidth(width)
  if im.SliderFloat(string.format("%s_x", widgetId), editor.getTempFloat_NumberNumber(baseProp.getMod and baseProp.getMod(value[1]) or value[1]), baseProp.min and baseProp.min[1] or nil, baseProp.max and baseProp.max[1] or nil, baseProp.format) then
    local newVal = editor.getTempFloat_NumberNumber()
    property.value[1] = baseProp.setMod and baseProp.setMod(newVal) or newVal
    if property.lockRatio then
      property.value[2] = property.value[1]
      property.value[3] = property.value[1]
      property.value[4] = property.value[1]
    end
    changed = true
  end
  ended = ended or im.IsItemDeactivatedAfterEdit()
  if editor.getPreference("dynamicDecalsTool.general.debug") then
    im.tooltip(string.format("%s_x", widgetId))
  end
  im.PopItemWidth()

  im.SameLine()
  im.PushItemWidth(width)
  if im.SliderFloat(string.format("%s_y", widgetId), editor.getTempFloat_NumberNumber(baseProp.getMod and baseProp.getMod(value[2]) or value[2]), baseProp.min and baseProp.min[2] or nil, baseProp.max and baseProp.max[2] or nil, baseProp.format) then
    local newVal = editor.getTempFloat_NumberNumber()
    property.value[2] = baseProp.setMod and baseProp.setMod(newVal) or newVal
    if property.lockRatio then
      property.value[1] = property.value[2]
      property.value[3] = property.value[2]
      property.value[4] = property.value[2]
    end
    changed = true
  end
  ended = ended or im.IsItemDeactivatedAfterEdit()
  if editor.getPreference("dynamicDecalsTool.general.debug") then
    im.tooltip(string.format("%s_y", widgetId))
  end
  im.PopItemWidth()

  im.SameLine()
  im.PushItemWidth(width)
  if im.SliderFloat(string.format("%s_z", widgetId), editor.getTempFloat_NumberNumber(baseProp.getMod and baseProp.getMod(value[3]) or value[3]), baseProp.min and baseProp.min[3] or nil, baseProp.max and baseProp.max[3] or nil, baseProp.format) then
    local newVal = editor.getTempFloat_NumberNumber()
    property.value[3] = baseProp.setMod and baseProp.setMod(newVal) or newVal
    if property.lockRatio then
      property.value[1] = property.value[3]
      property.value[2] = property.value[3]
      property.value[4] = property.value[3]
    end
    changed = true
  end
  ended = ended or im.IsItemDeactivatedAfterEdit()
  if editor.getPreference("dynamicDecalsTool.general.debug") then
    im.tooltip(string.format("%s_z", widgetId))
  end
  im.PopItemWidth()

  im.SameLine()
  im.PushItemWidth(width)
  if im.SliderFloat(string.format("%s_w", widgetId), editor.getTempFloat_NumberNumber(baseProp.getMod and baseProp.getMod(value[4]) or value[4]), baseProp.min and baseProp.min[4] or nil, baseProp.max and baseProp.max[4] or nil, baseProp.format) then
    local newVal = editor.getTempFloat_NumberNumber()
    property.value[4] = baseProp.setMod and baseProp.setMod(newVal) or newVal
    if property.lockRatio then
      property.value[1] = property.value[4]
      property.value[2] = property.value[4]
      property.value[3] = property.value[4]
    end
    changed = true
  end
  ended = ended or im.IsItemDeactivatedAfterEdit()
  if editor.getPreference("dynamicDecalsTool.general.debug") then
    im.tooltip(string.format("%s_w", widgetId))
  end
  im.PopItemWidth()

  if editEnded then
    editEnded[0] = ended
  end

  if changed == true then
    return true
  end
end

local function point4FColorWidget(value, property, guiId, editEnded, widgetId, widgetOptions)
  local changed = false

  local baseProp = property
  local ended = false

  if baseProp.default then
    if editor.uiIconImageButton(editor.icons.refresh, tool.getIconSizeVec2(), nil, nil, nil, string.format("%s_resetButton", widgetId)) then
      property.value = shallowcopy(baseProp.default)
      changed = true
      ended = true
    end
    im.tooltip(string.format("Reset to default: %s", dumps(baseProp.default)))
    im.SameLine()
  end

  im.PushItemWidth(im.GetContentRegionAvailWidth())
  if im.ColorEdit4(widgetId, editor.getTempFloatArray4_TableTable(value)) then
    property.value = editor.getTempFloatArray4_TableTable()
    changed = true
  end
  ended = ended or im.IsItemDeactivatedAfterEdit()
  im.PopItemWidth()

  if editEnded then
    editEnded[0] = ended
  end

  if changed == true then
    return true
  end
end

local function point4FColorWidgetWithPresets(value, property, guiId, editEnded, widgetId, widgetOptions)
  local changed = false

  local baseProp = property
  local ended = false

  if baseProp.default then
    if editor.uiIconImageButton(editor.icons.refresh, tool.getIconSizeVec2(), nil, nil, nil, string.format("%s_resetButton", widgetId)) then
      property.value = shallowcopy(baseProp.default)
      changed = true
      ended = true
    end
    im.tooltip(string.format("Reset to default: %s", dumps(baseProp.default)))
    im.SameLine()
  end





  if im.BeginPopup(string.format("%s_Point4FColorPopup", widgetId)) then
    im.SetNextItemWidth(256)
    if im.ColorPicker4(string.format("%s_Point4FColorPicker", widgetId), editor.getTempFloatArray4_TableTable(value), nil, nil) then
      value = editor.getTempFloatArray4_TableTable()
      changed = true
    end

    im.TextColored(editor.color.beamng.Value, "Presets")
    local btnSize = math.ceil(im.GetFontSize())
    local availableWidth = 256 + im.GetStyle().WindowPadding.x * 2
    local numOfPresetsInOneRow = availableWidth / (btnSize + im.GetStyle().ItemSpacing.x)
    local i = 0
    for k, col in ipairs(data) do
      i = i + 1
      if im.ColorButton(string.format("##pres_%s_%d", guiId, k), im.ImVec4(col[1],col[2],col[3],col[4]), nil, im.ImVec2(btnSize, btnSize)) then
        value = shallowcopy(col)
        changed = true
      end
      im.SameLine()
      if i >= numOfPresetsInOneRow then
        i = 1
        im.NewLine()
      end
    end

    im.EndPopup()
  end

  if im.ColorButton(string.format("%s_openPopupColorButton", widgetId), im.ImVec4(value[1],value[2],value[3],value[4]), nil, nil) then
    im.OpenPopup(string.format("%s_Point4FColorPopup", widgetId))
  end





  if editEnded then
    editEnded[0] = ended
  end

  if changed == true then
    return true
  end
end

-- STRING
local function stringInputWidget(value, property, guiId, editEnded, widgetId, widgetOptions)
  local changed = false

  local baseProp = property
  local ended = false

  if baseProp.default then
    if editor.uiIconImageButton(editor.icons.refresh, tool.getIconSizeVec2(), nil, nil, nil, string.format("%s_resetButton", widgetId)) then
      property.value = baseProp.default
      changed = true
      ended = true
    end
    im.tooltip(string.format("Reset to default: %s", dumps(baseProp.default)))
    im.SameLine()
  end

  im.PushItemWidth(im.GetContentRegionAvailWidth())
  if im.InputText(widgetId, editor.getTempCharPtr(value), nil, im.InputTextFlags_AutoSelectAll) then
    property.value = editor.getTempCharPtr()
    changed = true
  end
  ended = ended or im.IsItemDeactivatedAfterEdit()
  im.PopItemWidth()

  if editEnded then
    editEnded[0] = ended
  end

  if changed == true then
    return true
  end
end

local function stringInputMultilineWidget(value, property, guiId, editEnded, widgetId, widgetOptions)
  local changed = false

  local baseProp = property
  local ended = false

  if baseProp.default then
    if editor.uiIconImageButton(editor.icons.refresh, tool.getIconSizeVec2(), nil, nil, nil, string.format("%s_resetButton", widgetId)) then
      property.value = baseProp.default
      changed = true
      ended = true
    end
    im.tooltip(string.format("Reset to default: %s", dumps(baseProp.default)))
    im.SameLine()
  end

  if im.InputTextMultiline(widgetId, editor.getTempCharPtr(value), nil, im.ImVec2(im.GetContentRegionAvailWidth(), 120)) then
    property.value = editor.getTempCharPtr()
    changed = true
  end
  ended = ended or im.IsItemDeactivatedAfterEdit()

  if editEnded then
    editEnded[0] = ended
  end

  if changed == true then
    return true
  end
end

local function fileFileWidget(value, property, guiId, editEnded, widgetId, widgetOptions)
  local changed = false

  local baseProp = property
  local ended = false

  if baseProp.default then
    if editor.uiIconImageButton(editor.icons.refresh, tool.getIconSizeVec2(), nil, nil, nil, string.format("%s_resetButton", widgetId)) then
      property.value = baseProp.default
      changed = true
      ended = true
    end
    im.tooltip(string.format("Reset to default: %s", dumps(baseProp.default)))
    im.SameLine()
  end

  im.PushItemWidth(im.GetContentRegionAvailWidth() - (im.GetStyle().ItemSpacing.x + tool.getIconSize()))
  im.InputText(widgetId, editor.getTempCharPtr(value), nil, im.InputTextFlags_ReadOnly)
  im.PopItemWidth()

  im.SameLine()
  if editor.uiIconImageButton(editor.icons.folder, tool.getIconSizeVec2(), nil, nil, nil, string.format("%s_openFileDialogButton", widgetId)) then
    local openPath = baseProp.defaultDir
    local dir, fileName, fileExt = path.split(value)
    if #fileExt > 0 then
      openPath = dir
    end

    local fileTypes = {{"Any files", "*"}}
    if baseProp.fileTypes then
      for _, type in ipairs(baseProp.fileTypes) do
        table.insert(fileTypes, type)
      end
    end

    editor_fileDialog.openFile(
      function(data)
        property.value = data.filepath
        -- little hacky to add a member to the property object, but the 'changed' attribute is always false and won't notify the caller that the property has changed
        property.changed = true
      end,
      fileTypes,
      false,
      openPath,
      true
    )
  end

  if editEnded then
    editEnded[0] = ended
  end

  if changed == true or property.changed then
    property.changed = nil
    if editEnded then
      editEnded[0] = true
    end
    return true
  end
end

local function colorIColorWidget(value, property, guiId, editEnded, widgetId, widgetOptions)
  local changed = false

  local baseProp = property
  local ended = false

  if baseProp.default then
    if editor.uiIconImageButton(editor.icons.refresh, tool.getIconSizeVec2(), nil, nil, nil, string.format("%s_resetButton", widgetId)) then
      property.value = shallowcopy(baseProp.default)
      changed = true
      ended = true
    end
    im.tooltip(string.format("Reset to default: %s", dumps(baseProp.default)))
    im.SameLine()
  end

  im.PushItemWidth(im.GetContentRegionAvailWidth())
  local val = {value[1] / 255, value[2] / 255, value[3] / 255, value[4] / 255}
  if im.ColorEdit4(widgetId, editor.getTempFloatArray4_TableTable(val)) then
    local newVal = editor.getTempFloatArray4_TableTable()
    property.value = {newVal[1] * 255, newVal[2] * 255, newVal[3] * 255, newVal[4] * 255}
    changed = true
  end
  ended = ended or im.IsItemDeactivatedAfterEdit()
  im.PopItemWidth()

  if editEnded then
    editEnded[0] = ended
  end

  if changed == true then
    return true
  end
end

local function textureFileWidget(value, property, guiId, editEnded, widgetId, widgetOptions)
  local changed = false

  local baseProp = property
  local ended = false

  if baseProp.default then
    if editor.uiIconImageButton(editor.icons.refresh, tool.getIconSizeVec2(), nil, nil, nil, string.format("%s_resetButton", widgetId)) then
      property.value = baseProp.default
      changed = true
      ended = true
    end
    im.tooltip(string.format("Reset to default: %s", dumps(baseProp.default)))
    im.SameLine()
  end

  im.PushItemWidth(im.GetContentRegionAvailWidth() - (im.GetStyle().ItemSpacing.x + tool.getIconSize()))
  im.InputText(widgetId, editor.getTempCharPtr(value), nil, im.InputTextFlags_ReadOnly)
  helper.imageTooltip(value)
  im.PopItemWidth()

  im.SameLine()
  if editor.uiIconImageButton(editor.icons.folder, tool.getIconSizeVec2(), nil, nil, nil, string.format("%s_openFileDialogButton", widgetId)) then
    local openPath = baseProp.defaultDir
    local dir, fileName, fileExt = path.split(value)
    if #fileExt > 0 then
      openPath = dir
    end

    local fileTypes = {{"Any files", "*"}}
    if baseProp.fileTypes then
      for _, type in ipairs(baseProp.fileTypes) do
        table.insert(fileTypes, type)
      end
    end

    editor_fileDialog.openFile(
      function(data)
        property.value = data.filepath
        -- little hacky to add a member to the property object, but the 'changed' attribute is always false and won't notify the caller that the property has changed
        property.changed = true
      end,
      fileTypes,
      false,
      openPath,
      true
    )
  end

  if editEnded then
    editEnded[0] = ended
  end

  if changed == true or property.changed then
    property.changed = nil
    if editEnded then
      editEnded[0] = true
    end
    return true
  end
end

local function textureImageButtonWidget(value, property, guiId, editEnded, widgetId, widgetOptions)
  local changed = false

  local baseProp = property
  local ended = false

  if baseProp.default then
    if editor.uiIconImageButton(editor.icons.refresh, tool.getIconSizeVec2(), nil, nil, nil, string.format("%s_resetButton", widgetId)) then
      property.value = baseProp.default
      changed = true
      ended = true
    end
    im.tooltip(string.format("Reset to default: %s", dumps(baseProp.default)))
    im.SameLine()
  end

  local inputPosX = im.GetCursorPosX()
  im.PushItemWidth(im.GetContentRegionAvailWidth() - (im.GetStyle().ItemSpacing.x + tool.getIconSize()))
  im.InputText(widgetId, editor.getTempCharPtr(value), nil, im.InputTextFlags_ReadOnly)
  im.PopItemWidth()

  im.SameLine()

  if editor.uiIconImageButton(editor.icons.folder, tool.getIconSizeVec2(), nil, nil, nil, string.format("%s_openFileDialogButton", widgetId)) then
    local openPath = baseProp.defaultDir
    local dir, fileName, fileExt = path.split(value)
    if #fileExt > 0 then
      openPath = dir
    end

    local fileTypes = {{"Any files", "*"}}
    if baseProp.fileTypes then
      for _, type in ipairs(baseProp.fileTypes) do
        table.insert(fileTypes, type)
      end
    end

    editor_fileDialog.openFile(
      function(data)
        property.value = data.filepath
        -- little hacky to add a member to the property object, but the 'changed' attribute is always false and won't notify the caller that the property has changed
        property.changed = true
      end,
      fileTypes,
      false,
      openPath,
      true
    )
  end

  local img = editor.getTempTextureObj(value)
  local imgWidthSetting = editor.getPreference("dynamicDecalsTool.widgets.texturePreviewSize")
  local imgWidth = imgWidthSetting > im.GetContentRegionAvailWidth() and im.GetContentRegionAvailWidth() or imgWidthSetting
  local imgHeight = img.path == "/" and imgWidth or imgWidth * img.size.y / img.size.x

  -- align with InputText widget
  -- im.SetCursorPosX(inputPosX)

  -- right align
  im.SetCursorPosX(im.GetContentRegionAvailWidth() - (imgWidth))

  local windowPos = im.GetWindowPos()
  local scrollX = im.GetScrollX()
  local scrollY = im.GetScrollY()
  im.ImDrawList_AddRect(
    im.GetWindowDrawList(),
    im.ImVec2(im.GetCursorPosX() + windowPos.x - scrollX - 1, im.GetCursorPosY() + windowPos.y - scrollY - 1),
    im.ImVec2(im.GetCursorPosX() + imgWidth + windowPos.x - scrollX + 3, im.GetCursorPosY() + imgHeight + windowPos.y - scrollY + 2),
    im.GetColorU322(editor.color.grey.Value),
    nil,
    nil,
    1
  )


  if im.ImageButton(string.format("%s_imageButton", widgetId), img.texId, im.ImVec2(imgWidth, imgHeight), im.ImVec2(0,0), im.ImVec2(1,1)) then
    local openPath = baseProp.defaultDir
    local dir, fileName, fileExt = path.split(value)
    if #fileExt > 0 then
      openPath = dir
    end

    local fileTypes = {{"Any files", "*"}}
    if baseProp.fileTypes then
      for _, type in ipairs(baseProp.fileTypes) do
        table.insert(fileTypes, type)
      end
    end

    editor_fileDialog.openFile(
      function (data)
        property.value = data.filepath
        -- little hacky to add a member to the property object, but the 'changed' attribute is always false and won't notify the caller that the property has changed
        property.changed = true
      end,
      fileTypes,
      false,
      openPath,
      true
    )
  end
  if im.BeginDragDropTarget() then
    local payload = im.AcceptDragDropPayload("DynDecalTextureDrapDrop")
    if payload~=nil then
      assert(payload.DataSize == 256)
      local path = ffi.string(payload.Data)
      property.value = path
      changed = true
    end
    im.EndDragDropTarget()
  end

  if editEnded then
    editEnded[0] = ended
  end

  if changed == true or property.changed then
    property.changed = nil
    if editEnded then
      editEnded[0] = true
    end
    return true
  end
end

local function multiColorColorGradientWidget(value, property, guiId, editEnded, widgetId, widgetOptions)
  local changed = false

  local baseProp = property
  local ended = false

  if baseProp.default then
    if editor.uiIconImageButton(editor.icons.refresh, tool.getIconSizeVec2(), nil, nil, nil, string.format("%s_resetButton", widgetId)) then
      property.value = baseProp.default
      changed = true
      ended = true
    end
    im.tooltip(string.format("Reset to default: %s", dumps(baseProp.default)))
    im.SameLine()
  end

  local gradientColorTopLeft = {value[1][1]/255, value[1][2]/255, value[1][3]/255, value[1][4]/255}
  local gradientColorTopRight = {value[2][1]/255, value[2][2]/255, value[2][3]/255, value[2][4]/255}
  local gradientColorBottomLeft = {value[3][1]/255, value[3][2]/255, value[3][3]/255, value[3][4]/255}
  local gradientColorBottomRight = {value[4][1]/255, value[4][2]/255, value[4][3]/255, value[4][4]/255}
  local gradientColorTopLeftU32 = im.GetColorU322(editor.getTempImVec4_TableTable(gradientColorTopLeft))
  local gradientColorTopRightU32 = im.GetColorU322(editor.getTempImVec4_TableTable(gradientColorTopRight))
  local gradientColorBottomLeftU32 = im.GetColorU322(editor.getTempImVec4_TableTable(gradientColorBottomLeft))
  local gradientColorBottomRightU32 = im.GetColorU322(editor.getTempImVec4_TableTable(gradientColorBottomRight))

  local prefSize = editor.getPreference("dynamicDecalsTool.widgets.multiColorWidgetWidth")
  local maxSize = im.GetContentRegionAvailWidth() - (2 * im.GetStyle().ItemSpacing.x + 2 * math.ceil(im.GetFontSize()))
  local size = prefSize > maxSize and maxSize or prefSize

  local cursorPos = im.GetCursorPos()
  im.SetCursorPos(im.ImVec2(cursorPos.x, cursorPos.y + size - 20))
  if editor.uiColorEdit4(string.format("%s_gradientColorBottomLeft", widgetId), editor.getTempFloatArray4_TableTable(gradientColorBottomLeft), im.flags(im.ColorEditFlags_AlphaPreview, im.ColorEditFlags_NoInputs), editor.getTempBool_BoolBool(false)) then
    local val = editor.getTempFloatArray4_TableTable()
    property.value[1] = {value[1][1], value[1][2], value[1][3], value[1][4]}
    property.value[2] = {value[2][1], value[2][2], value[2][3], value[2][4]}
    property.value[4] = {value[4][1], value[4][2], value[4][3], value[4][4]}

    property.value[3] = {val[1] * 255, val[2] * 255, val[3] * 255, val[4] * 255}
    changed = true
  end
  if editor.getTempBool_BoolBool() then
    ended = true
  end

  im.SetCursorPos(cursorPos)
  if editor.uiColorEdit4(string.format("%s_gradientColorTopLeft", widgetId), editor.getTempFloatArray4_TableTable(gradientColorTopLeft), im.flags(im.ColorEditFlags_AlphaPreview, im.ColorEditFlags_NoInputs), editor.getTempBool_BoolBool(false)) then
    local val = editor.getTempFloatArray4_TableTable()
    property.value[2] = {value[2][1], value[2][2], value[2][3], value[2][4]}
    property.value[3] = {value[3][1], value[3][2], value[3][3], value[3][4]}
    property.value[4] = {value[4][1], value[4][2], value[4][3], value[4][4]}

    property.value[1] = {val[1] * 255, val[2] * 255, val[3] * 255, val[4] * 255}
    changed = true
  end
  if editor.getTempBool_BoolBool() then
    ended = true
  end
  im.SameLine()
  local windowPos = im.GetWindowPos()
  cursorPos = im.GetCursorPos()
  local scrollPosX = im.GetScrollX()
  local scrollPosY = im.GetScrollY()
  im.ImDrawList_AddRectFilledMultiColor(
    im.GetWindowDrawList(),
    im.ImVec2(windowPos.x + cursorPos.x - scrollPosX, windowPos.y + cursorPos.y - scrollPosY),
    im.ImVec2(windowPos.x + cursorPos.x + size - scrollPosX, windowPos.y + cursorPos.y + size - scrollPosY),
    gradientColorTopLeftU32,
    gradientColorTopRightU32,
    gradientColorBottomRightU32,
    gradientColorBottomLeftU32
  )
  -- adding an invisible button so the imgui cursor is at the right location
  im.InvisibleButton("GradientButton", im.ImVec2(size, size))
  im.SameLine()

  cursorPos = im.GetCursorPos()
  im.SetCursorPos(im.ImVec2(cursorPos.x, cursorPos.y + size - 20))
  if editor.uiColorEdit4(string.format("%s_gradientColorBottomRight", widgetId), editor.getTempFloatArray4_TableTable(gradientColorBottomRight), im.flags(im.ColorEditFlags_AlphaPreview, im.ColorEditFlags_NoInputs), editor.getTempBool_BoolBool(false)) then
    local val = editor.getTempFloatArray4_TableTable()
    property.value[1] = {value[1][1], value[1][2], value[1][3], value[1][4]}
    property.value[2] = {value[2][1], value[2][2], value[2][3], value[2][4]}
    property.value[3] = {value[3][1], value[3][2], value[3][3], value[3][4]}

    property.value[4] = {val[1] * 255, val[2] * 255, val[3] * 255, val[4] * 255}
    changed = true
  end
  if editor.getTempBool_BoolBool() then
    ended = true
  end

  im.SetCursorPos(cursorPos)
  if editor.uiColorEdit4(string.format("%s_gradientColorTopRight", widgetId), editor.getTempFloatArray4_TableTable(gradientColorTopRight), im.flags(im.ColorEditFlags_AlphaPreview, im.ColorEditFlags_NoInputs), editor.getTempBool_BoolBool(false)) then
    local val = editor.getTempFloatArray4_TableTable()
    property.value[1] = {value[1][1], value[1][2], value[1][3], value[1][4]}
    property.value[3] = {value[3][1], value[3][2], value[3][3], value[3][4]}
    property.value[4] = {value[4][1], value[4][2], value[4][3], value[4][4]}

    property.value[2] = {val[1] * 255, val[2] * 255, val[3] * 255, val[4] * 255}
    changed = true
  end
  if editor.getTempBool_BoolBool() then
    ended = true
  end
  im.SetCursorPos(im.ImVec2(cursorPos.x, cursorPos.y + size + im.GetStyle().ItemSpacing.y))
  im.NewLine()

  if editEnded then
    editEnded[0] = ended
  end

  if changed == true then
    return true
  end
end

M.draw = function(value, property, guiId, editEnded, widgetOptions)
  local widgetId = string.format("##%s_%s", guiId, property.id)
  local propType = property.type or api.propertiesMap[property.id].type
  local isHighlighted = highlightedWidgets[widgetId]

  -- Widget highlighting
  if isHighlighted and highlightedWidgets[widgetId].setScroll then
    im.SetScrollHereY()
    highlightedWidgets[widgetId].setScroll = false
  end
  local startCursorPos = im.GetCursorPos()

  im.TextUnformatted(property.name or api.propertiesMap[property.id].name)
  local description = property.description or api.propertiesMap[property.id].description
  if editor.getPreference("dynamicDecalsTool.general.debug") then
    if #description > 0 then
      im.tooltip(string.format("%s\n%s", widgetId, description))
    else
      im.tooltip(widgetId)
    end
  else
    if #description > 0 then
      im.tooltip(description)
    end
  end
  im.SameLine()

  if api.widgetTypes[propType] then
    if im.BeginPopup(string.format("WidgetTypePopup_%s", widgetId)) then
      for name, id in pairs(api.widgetTypes[propType]) do
        if im.Selectable1(name, id == (property.widgetType or 0)) then
          property.widgetType = id
        end
      end
      im.EndPopup()
    end

    if editor.uiIconImageButton(editor.icons.fg_type_diamond_2, tool.getIconSizeVec2(), nil, nil, nil, string.format("WidgetTypeOpenPopupButton_", widgetId)) then
      im.OpenPopup(string.format("WidgetTypePopup_%s", widgetId))
    end
    im.tooltip("Widget Type")

    im.SameLine()
  end

  -- BOOL
  if propType == api.types.bool then
    if property.widgetType then
      if property.widgetType == api.widgetTypes[api.types.int].Input then
        if boolCheckboxWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
      else -- default or not existing
        if boolCheckboxWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
      end
    else
      -- default
      if boolCheckboxWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
    end

  -- INTEGER
  elseif propType == api.types.int then
    if property.widgetType then
      if property.widgetType == api.widgetTypes[api.types.int].Input then
        if intInputWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
      elseif property.widgetType == api.widgetTypes[api.types.int].Slider then
        if intSliderWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
      elseif property.widgetType == api.widgetTypes[api.types.int].Drag then
        if intDragWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
      elseif property.widgetType == api.widgetTypes[api.types.int].Combo then
        if intComboWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
      else -- default or not existing
        if intInputWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
      end
    else
      -- default
      if intInputWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
    end

  -- FLOAT
  elseif propType == api.types.float then
    if property.widgetType then
      if property.widgetType == api.widgetTypes[api.types.float].Input then
        if floatInputWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
      elseif property.widgetType == api.widgetTypes[api.types.float].Slider then
        if floatSliderWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
      elseif property.widgetType == api.widgetTypes[api.types.float].Drag then
        if floatDragWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
      else -- default or not existing
        if floatSliderWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
      end
    else
      -- default
      if floatSliderWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
    end

  -- POINT2F
  elseif propType == api.types.Point2F then
    if property.widgetType then
      if property.widgetType == api.widgetTypes[api.types.Point2F].Input then
        if point2FInputWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
      elseif property.widgetType == api.widgetTypes[api.types.Point2F].Slider then
        if point2FSliderWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
      elseif property.widgetType == api.widgetTypes[api.types.Point2F].Drag then
        if point2FDragWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
      else -- default or not existing
        if point2FSliderWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
      end
    else
      -- default
      if point2FSliderWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
    end

  -- POINT3F
  elseif propType == api.types.Point3F then
    if property.widgetType then
      if property.widgetType == api.widgetTypes[api.types.Point3F].Slider then
        if point3FSliderWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
      elseif property.widgetType == api.widgetTypes[api.types.Point3F].Color then
        if point3FColorWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
      else -- default or not existing
        if point3FSliderWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
      end
    else
      -- default
      if point3FSliderWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
    end

  -- POINT4F
  elseif propType == api.types.Point4F then
    if property.widgetType then
      if property.widgetType == api.widgetTypes[api.types.Point4F].Slider then
        if point4FSliderWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
      elseif property.widgetType == api.widgetTypes[api.types.Point4F].Color then
        if point4FColorWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
      else -- default or not existing
        if point4FSliderWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
      end
    else
      -- default
      if point4FSliderWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
    end

  -- STRING
  elseif propType == api.types.string then
    if property.widgetType then
      if property.widgetType == api.widgetTypes[api.types.string].Input then
        if stringInputWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
      elseif property.widgetType == api.widgetTypes[api.types.string].InputMultiline then
        if stringInputMultilineWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
      else -- default or not existing
        if stringInputWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
      end
    else
      -- default
      if stringInputWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
    end

  -- TEXTURE
  elseif propType == api.types.Texture then
    if property.widgetType then
      if property.widgetType == api.widgetTypes[api.types.Texture].File then
        if textureFileWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
      elseif property.widgetType == api.widgetTypes[api.types.Texture].ImageButton then
        if textureImageButtonWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
      else -- default or not existing
        if textureImageButtonWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
      end
    else
      -- default
      if textureImageButtonWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
    end

  -- FILE
  elseif propType == api.types.File then
    if property.widgetType then
      if property.widgetType == api.widgetTypes[api.types.File].File then
        if fileFileWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
      else -- default or not existing
        if fileFileWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
      end
    else
      -- default
      if fileFileWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
    end

  -- ColorI
  elseif propType == api.types.ColorI then
    if property.widgetType then
      if property.widgetType == api.widgetTypes[api.types.ColorI].Color then
        if colorIColorWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
      else -- default or not existing
        if colorIColorWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
      end
    else
      -- default
      if colorIColorWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
    end

  -- MultiColor
  elseif propType == api.types.MultiColor then
    if property.widgetType then
      if property.widgetType == api.widgetTypes[api.types.MultiColor].ColorGradient then
        if multiColorColorGradientWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
      else -- default or not existing
        if multiColorColorGradientWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
      end
    else
      -- default
      if multiColorColorGradientWidget(value, property, guiId, editEnded, widgetId, widgetOptions or {}) then return true end
    end

  else
    im.TextColored(editor.color.error.Value, api.propertiesMap[property.id].name)
  end

  -- Widget highlighting
  if isHighlighted then
    local endCursorPos = im.GetCursorPos()
    local wpos = im.GetWindowPos()
    local cpos = startCursorPos
    local scrollX = im.GetScrollX()
    local scrollY = im.GetScrollY()
    local style = im.GetStyle()
    local p1 = im.ImVec2(wpos.x + startCursorPos.x - scrollX - style.ItemSpacing.x / 2, wpos.y + startCursorPos.y - scrollY)
    local p2 = im.ImVec2(wpos.x + endCursorPos.x - scrollX + im.GetContentRegionAvailWidth() + style.ItemSpacing.x / 2, wpos.y + endCursorPos.y - scrollY)
    im.ImDrawList_AddRect(im.GetWindowDrawList(), p1, p2, im.GetColorU322(editor.color.beamng.Value), nil, nil, 2)
  end
end

M.highlight = function(id, time)
  highlightedWidgets = {}
  highlightedWidgets[id] = {time = time, setScroll = true}
end

local function editModeUpdate(dtReal, dtSim, dtRaw)
  for id, _ in pairs(highlightedWidgets) do
    highlightedWidgets[id].time = highlightedWidgets[id].time - dtReal
    if highlightedWidgets[id].time < 0 then
      highlightedWidgets[id] = nil
    end
  end
end

local function setup(tool_in)
  tool = tool_in
  api = extensions.editor_api_dynamicDecals
  helper = extensions.editor_dynamicDecals_helper

  tool.registerEditorOnUpdateFn("widgets", editModeUpdate)
end

local function registerEditorPreferences(prefsRegistry)
  prefsRegistry:registerSubCategory("dynamicDecalsTool", "widgets", nil, {
    {texturePreviewSize = {"float", 128, "Max width of the decal texture thumbnails", nil, 32, 512}},
    {multiColorWidgetWidth = {"float", 128, "Width of the color gradient preview", nil, 32, 512}},
  })
end

local function editorPreferenceValueChanged(path, value)

end

M.defaultButton = defaultButton

M.registerEditorPreferences = registerEditorPreferences
M.editorPreferenceValueChanged = editorPreferenceValueChanged
M.setup = setup

return M