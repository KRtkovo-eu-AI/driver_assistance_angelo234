-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = 'editor_scene_tree'
local toolWindowName = "forestEditorView"
local imgui = ui_imgui
-- local selectedItems = {}

local function onEditorInitialized()
  editor.registerWindow(toolWindowName, imgui.ImVec2(220,320), nil, true)
end

local function onEditorActivated()

end

-- local function selectForestItemData(item, addToSelection)
--   log('I','','selecting item: '..dumps(item))
--   editor.selection = {}
--   if addToSelection then
--     item.selected = not item.selected
--   else
--     for _, entry in ipairs(selectedItems) do
--       entry.selected = false
--     end
--     selectedItems = {}
--     item.selected = true
--   end

--   if item.selected then
--     table.insert(selectedItems, item)
--     -- --
--     -- -- Use the forest to find the corresponding forest item
--     -- --
--     local forest = scenetree.findObject("theForest")
--     if not forest then
--       return
--     end
--     local forestItem = forest:getData():findItem(item.key)

--     local forestItems = {forestItem}
--     -- editor_forestEditor.selectForestItems(forestItems, addToSelection)
--     dump(editor_forestEditor.getVar())
--   else
--     for i, entry in ipairs(selectedItems) do
--       if entry.key == item.key then
--         table.remove(selectedItems, i)
--         break
--       end
--     end
--   end
-- end

local function forestItemDataTreeNode(item)
  if not item then return end
  local obj = scenetree.findObjectById(item.id)
  if not obj then return end
  local fontSize = math.ceil(imgui.GetFontSize())
  local buttonColor_active = imgui.GetStyleColorVec4(imgui.Col_ButtonActive)
  local buttonColor_inactive = imgui.GetStyleColorVec4(imgui.Col_Button)
  local style = imgui.GetStyle()
  editor.uiIconImage(editor.icons.forest_brushelement, imgui.ImVec2(fontSize, fontSize))
  imgui.SameLine()
  local cPos = imgui.GetCursorPos()
  imgui.PushStyleColor2(imgui.Col_Button, (item.selected == true) and buttonColor_active or buttonColor_inactive)
  --TODO: make sure to make a better nil check for internalName
  if imgui.Button("##_button_FID_" .. tostring(item.name), imgui.ImVec2(imgui.GetContentRegionAvailWidth(), fontSize)) then
    -- add to selection if ctrl is held
    selectForestItemData(item, editor.keyModifiers.ctrl)
  end
  imgui.PopStyleColor()
  imgui.SetCursorPos(imgui.ImVec2(cPos.x + style.FramePadding.x, cPos.y))
  imgui.TextUnformatted(obj.name)
end

local forestItemSortByNameFunc = function(a, b)
  local aObj = scenetree.findObjectById(a.id)
  local bObj = scenetree.findObjectById(b.id)
  return string.lower(aObj.internalName or "unnamed_"..a.id) < string.lower(bObj.internalName or "unnamed_"..b.id)
end

local forestItemsInfo = {}

local function onEditorGui()
  local numberForestItems = #forestItemsInfo
  if editor.beginWindow(toolWindowName, "Forest View") then
      imgui.BeginChild1("##scrollingregion")
      imgui.Columns(1)
      for i = 1, numberForestItems do
        forestItemDataTreeNode(forestItemsInfo[i])
      end
      imgui.EndChild()
      -- imgui.TreePop()
    -- end
    editor.endWindow()
  end
end

local function populateForestItems()
 local forestObject = core_forest.getForestObject()
  if forestObject then
    forestObject = Sim.upcast(forestObject)
  end

 local forest = scenetree.findObject("theForest")
  if not forest then
    return
  end
  local forestItems = forest:getData():getItems()
  if not forestItems then
    return
  end

  local numberForestItems = #forestItems
  for i = 1, numberForestItems do
    local key = forestItems[i]:getKey()
    local data = forestItems[i]:getData()
    local name = data:getName()
    local obj = scenetree.findObject(name)
    if obj then
      local id = obj:getId()
      table.insert(forestItemsInfo, {key = key, id = id, name = (obj.internalName or "unnamed").."_"..id.."_"..i, selected = false})
    end
  end
  table.sort(forestItemsInfo, forestItemSortByNameFunc)
  -- log('I','','populateForestItems: '..dumps(forestItemsInfo))
end

local function onEditorEditModeActivated(newEditMode)
  -- log('I','','onEditorEditModeActivated called: '..dumpsz(newEditMode, 2))
  if newEditMode.displayName == 'Edit Forest' or newEditMode.actionMap == 'forestTools' then
    editor_sceneTree.closeAllInstances()
    editor.showWindow("forestEditorView")
    populateForestItems()
  end
end

local function onEditorEditModeDeactivated(oldEditMode)
  -- log('I','','onEditorEditModeDeactivated called: '..dumpsz(oldEditMode, 2))
  if oldEditMode.displayName == 'Edit Forest' or oldEditMode.actionMap == 'forestTools' then
    editor_sceneTree.openSceneTree()
    editor.showWindow("forestEditorView", false)
  end
end



M.onEditorInitialized = onEditorInitialized
M.onEditorActivated = onEditorActivated
M.onEditorGui = onEditorGui
-- M.onEditorEditModeActivated = onEditorEditModeActivated
-- M.onEditorEditModeDeactivated = onEditorEditModeDeactivated

return M
