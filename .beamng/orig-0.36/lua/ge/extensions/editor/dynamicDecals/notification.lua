-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = "editor_dynamicDecals_notification"
local im = ui_imgui

local dynamicDecal_notification_windowName = "Dynamic Decals Tool - Notifications"

-- reference to the editor tool, set in setup()
local tool = nil
-- reference to the dynamics decal api
local api = nil

local levels = {
  log = 1,
  warning = 2,
  error = 3
}
local colors = {
  [1] = im.ImColorByRGB(255,255,255,255), -- log
  [2] = im.ImColorByRGB(255,204,0,255), -- warning
  [3] = im.ImColorByRGB(255,0,0,255), -- error
}

local popupTitle = "Dynamic Decals - Notifications"
local notifications = {}
local dirty = false

local function onGui()
  if editor.beginWindow(dynamicDecal_notification_windowName, "Dynamic Decals - Notifications") then
    if tableSize(notifications) > 0 then
      local style = im.GetStyle()
      im.BeginChild1("DynamicDecals_Notification_NotificationsChild", im.ImVec2(0, im.GetContentRegionAvail().y - (math.ceil(im.GetFontSize()) + 2*style.ItemSpacing.y)), true)
      for sectionName, sectionData in pairs(notifications) do

        if im.CollapsingHeader1(string.format("%s##NotificationSection", sectionName), im.TreeNodeFlags_DefaultOpen) then
          for k, notification in ipairs(sectionData) do
            if editor.uiIconImageButton(editor.icons.delete, im.ImVec2(tool.getIconSize(), tool.getIconSize()), nil, nil, nil, string.format("%s_%d", sectionName, k)) then
              table.remove(notifications[sectionName], k)
              if #notifications[sectionName] == 0 then notifications[sectionName] = nil end
            end
            im.tooltip("Remove notification")
            im.SameLine()
            local msgtype = type(notification.msg)
            if msgtype == 'string' then
              im.TextColored(colors[notification.level].Value, string.format("%s - %s", notification.title, notification.msg))
            elseif msgtype == 'function' then
              notification.msg()
            end
          end
        end
      end
      im.EndChild()

      if im.Button("Remove all") then
        notifications = {}
      end
      im.SameLine()
      if im.Button("Close") then
        editor.hideWindow(dynamicDecal_notification_windowName)
      end
    else
      editor.hideWindow(dynamicDecal_notification_windowName)
    end
  end
  editor.endWindow()

  if dirty then
    editor.showWindow(dynamicDecal_notification_windowName)
    if editor.isWindowVisible(dynamicDecal_notification_windowName) then
      dirty = false
    end
  end
end

local function registerEditorPreferences(prefsRegistry)
  -- prefsRegistry:registerSubCategory("dynamicDecalsTool", "moduleName", nil, {

  -- })
end

local function editorPreferenceValueChanged(path, value)

end

local function setup(tool_in)
  tool = tool_in
  api = extensions.editor_api_dynamicDecals

  tool.registerOnEditorGuiFn("notification", onGui)

  editor.registerWindow(dynamicDecal_notification_windowName, im.ImVec2(450, 650), nil, nil, nil, true)
end

local function add(section, title, msg, level)
  if not notifications[section] then notifications[section] = {} end
  table.insert(notifications[section], {section=section, title=title, msg=msg, level=level or levels.log})
  dirty = true
end

M.add = add
M.levels = levels

M.onGui = onGui
M.registerEditorPreferences = registerEditorPreferences
M.editorPreferenceValueChanged = editorPreferenceValueChanged
M.setup = setup

return M