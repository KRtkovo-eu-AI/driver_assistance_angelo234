-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local logTag = "editor_dynamicDecals_docs"
local im = ui_imgui

-- reference to the editor tool, set in setup()
local tool = nil
local helper = nil

local windowName = "Vehicle Livery Editor - Documentation"
local lateSetupTimer = 2
local docsSectionsBase = {children = {}}
local docsSections = {children = {}}
local setColumnWidth = true
local currentSection = nil
local currentSectionId = ""
local filter = im.ImGuiTextFilter()

local first = true
local inputActionInfo = {}

local setScroll = false

local function checkSection(section, id)
  if section.name then

    local selected = currentSection == section
    if selected then

      if setScroll then
        setScroll = false
        im.SetScrollHereY()
      end

      local wpos = im.GetWindowPos()
      local cPos = im.GetCursorPos()
      local itemInnerSpacing = im.GetStyle().ItemInnerSpacing.x
      local itemSpacing = im.GetStyle().ItemSpacing
      local scrollY = im.GetScrollY()
      im.ImDrawList_AddRectFilled(
        im.GetWindowDrawList(),
        im.ImVec2(wpos.x + cPos.x - itemInnerSpacing - itemSpacing.x, wpos.y + cPos.y - itemSpacing.y/2 - scrollY),
        im.ImVec2(wpos.x + cPos.x - itemInnerSpacing, wpos.y + cPos.y + im.GetFontSize() + itemSpacing.y - scrollY),
        im.GetColorU322(editor.color.beamng.Value)
      )
    end

    if section.order then
      im.PushStyleColor2(im.Col_Text, editor.color.lightblue.Value)
    end
    if first and not section.order then
      im.Separator()
      first = false
    end

    if im.Selectable1(string.format("%s##%s", section.name, id), selected) then
      currentSection = section
      currentSectionId = id
    end
    if section.order then
      im.PopStyleColor(1)
    end
  end

  if not section.children then return end
  im.Indent()
  for k, v in pairs(section.children) do
    checkSection(v, string.format("%s/%s", id, k))
  end
  im.Unindent()
end

local function filterChildren(section, parentSection, index)
  for i = #section.children, 1, -1 do
    local child = section.children[i]
    if child.children then
      filterChildren(child, section, i)
    else
      if not im.ImGuiTextFilter_PassFilter(filter, child.name) then
        table.remove(section.children, i)
      end
    end
  end

  -- There's no child in this section anymore. Check if the section itself passes the filter, else remove it from its parent.
  if #section.children == 0 and parentSection and not im.ImGuiTextFilter_PassFilter(filter, section.name) then
    table.remove(parentSection.children, index)
  end
end

local function filterDocs()
  docsSections = deepcopy(docsSectionsBase)
  filterChildren(docsSections)
end

local function onGui()
  if editor.beginWindow(windowName, windowName) then
    -- TITLE
    im.PushFont3("cairo_regular_medium")
    local titleSize = im.CalcTextSize(windowName)
    im.SetCursorPosX(im.GetCursorPosX() + im.GetContentRegionAvailWidth() / 2 - titleSize.x / 2)
    im.TextColored(editor.color.beamng.Value, windowName)
    im.PopFont()
    im.SameLine()
    im.Separator()

    im.Columns(2, "DynDecal_Docs_Columns")
    if setColumnWidth then
      im.SetColumnWidth(0, im.GetWindowWidth() / 4)
      setColumnWidth = false
    end


    -- TABLE OF CONTENTS
    if im.BeginChild1("TreeChild", nil, true) then
      if editor.uiInputSearchTextFilter("docsfilter", filter, im.GetContentRegionAvailWidth()) then
        filterDocs()
      end
      im.Separator()

      im.PushStyleVar1(im.StyleVar_IndentSpacing, editor.getPreference("dynamicDecalsTool.docs.indentSpacing"))
      first = true
      checkSection(docsSections, "")
      im.PopStyleVar(1)
    end
    im.EndChild()
    im.NextColumn()


    -- DOCS CONTENT
    if im.BeginChild1("ContentChild", nil, true) then
      if currentSection then
        -- HEADER
        im.PushFont3("cairo_regular_medium")
        local titleSize = im.CalcTextSize(currentSection.name)
        im.SetCursorPosX(im.GetCursorPosX() + im.GetContentRegionAvailWidth() / 2 - titleSize.x / 2)
        im.TextColored(editor.color.beamng.Value, currentSection.name)
        im.PopFont()
        im.Separator()

        if currentSection.fn then
          currentSection.fn(currentSection)
        end
      end
    end
    im.EndChild()


    im.NextColumn()
    im.Columns(1, "DynDecal_Docs_Columns")
  end
  editor.endWindow()
end

local function sortFn(a, b)
  if a.order and not b.order then return true end
  if not a.order and b.order then return false end
  if a.order and b.order then return a.order < b.order end
  return string.lower(a.name) < string.lower(b.name)
end

local function sortSectionChildren(section)
  if not section.children then return end
  table.sort(section.children, sortFn)
  for _, child in ipairs(section.children) do
    sortSectionChildren(child)
  end
end

local function image(path, overrideWidth, title, subtitle)
  local img = editor.getTempTextureObj(path)
  local ratio = img.size.y / img.size.x
  local sizeX = overrideWidth or im.GetContentRegionAvailWidth()
  local sizeY = sizeX * ratio

  im.Image(
    img.tex:getID(),
    im.ImVec2(sizeX, sizeY),
    nil, nil, nil,
    editor.color.white.Value
  )
  if title then
    im.tooltip(title)
  end
  if subtitle then
    im.TextColored(editor.color.grey.Value, subtitle)
  end
end

local function verticalSpacing()
  im.Dummy(im.ImVec2(0, editor.getPreference("dynamicDecalsTool.docs.verticalSpacing")))
end

M.introductionGui = function(docsSection)
  im.PushTextWrapPos(im.GetContentRegionAvailWidth())
  helper.textUnformattedCentered("Welcome to the world of skin customization in BeamNG!")
  helper.textUnformattedCentered(string.format("v %d.%d.%d", tool.version[1], tool.version[2], tool.version[2]))

  im.TextUnformatted([[

We're delighted to introduce you to our skin creation tool, a work-in-progress tool designed to give you complete freedom in personalizing your vehicles.

Our tool offers an array of features, let's dive into what you can expect:

* Decal Customization: With our tool, you can place decals on your vehicles, making them truly yours. Choose from a diverse collection of preloaded decal textures or import your own designs. Adjust the size, rotation, color, and more to achieve the perfect look.
* Layer System: Get ready to explore endless possibilities with our versatile layer system. Create decal layers, path layers (ideal for text or intricate designs following curves), fill layers, texture fill layers (to fill shapes with captivating patterns), brush stroke layers, and group layers.
  * Layer Masks: Enhance your designs with layer masks. These masks provide you with greater control and flexibility in shaping your decals and compositions.
* SDF support: The tool supports SDF (Signed Distance Field) technology, ensuring your decals and text appear crisp and sharp. Add colored outlines, edge feathering, and other fine details to take your designs to the next level.
* Save, Share, and Export: Once you've crafted your perfect skin, save it for future use or share it with fellow enthusiasts. You can export your designs as skin to seamlessly incorporate them into BeamNG. Alternatively, you can export the raw textures, allowing you to make fine adjustments and further refine your designs using third-party raster image editing tools.
]])

  im.TextColored(editor.color.beamng.Value, "Please keep in mind that this tool is work-in-progress. We're actively working to enhance and refine the experience based on your feedback.")
  im.TextUnformatted("We can't wait to see the incredible liveries you create and share with the community!")

  verticalSpacing()
  if im.Button("Dynamic Decals Thread [Link]", im.ImVec2(im.GetContentRegionAvailWidth(), 0)) then openWebBrowser("https://www.beamng.com/threads/experimental-dynamic-decals.95559/") end
  verticalSpacing()

  im.TextUnformatted([[
Get ready to rev up your creativity and make your vehicles stand out on the track!
  ]])
  im.PopTextWrapPos()
end

local function gettingStartedGui(docsSection)
  im.PushTextWrapPos(im.GetContentRegionAvailWidth())

  im.TextUnformatted([[
This guide helps you getting started with this tool.
]])

  im.PopTextWrapPos()
end

local function controlsGui(docsSection)
  -- local spacing = im.ImVec2(1, im.GetStyle().ItemSpacing.y * 2)

  if im.BeginTable("Decal Layer Properties Table", 3, im.flags(im.TableFlags_Resizable, im.TableFlags_Hideable, im.TableFlags_RowBg)) then
    im.TableSetupColumn('Control')
    im.TableSetupColumn('Title')
    im.TableSetupColumn('Description')
    im.TableHeadersRow()
    for _, inputAction in ipairs(inputActionInfo) do
      im.TableNextColumn()
      im.TextUnformatted(inputAction.controlCap)
      im.TableNextColumn()
      im.TextUnformatted(inputAction.title)
      im.TableNextColumn()
      im.TextUnformatted(inputAction.desc)
    end

    im.EndTable()
  end
end

local function lateSetup()
  -- docs section sorting
  sortSectionChildren(docsSections)
  docsSectionsBase = deepcopy(docsSections)
  -- select the very first section of the docs by default
  M.selectSection(docsSections.children[1].name, true)
  -- dump(docsSections)
end

local function editModeUpdate(dtReal, dtSim, dtRaw)
  if not lateSetupTimer then return end
  if lateSetupTimer >= 0 then
    lateSetupTimer = lateSetupTimer - 1
  else
    lateSetup()
    lateSetupTimer = nil
  end
end

local function setup(tool_in)
  tool = tool_in
  helper = extensions.editor_dynamicDecals_helper

  editor.registerWindow(windowName, im.ImVec2(640, 640))

  tool.registerOnEditorGuiFn("docs", onGui)
  tool.registerEditorOnUpdateFn("docs", editModeUpdate)

  M.register({section = {"Introduction"}, guiFn = M.introductionGui, order = 0})
  M.register({section = {"Getting Started"}, guiFn = gettingStartedGui, order = 10})
  M.register({section = {"Controls"}, guiFn = controlsGui, order = 20})

  -- Get input actions for the tool
  inputActionInfo = {}
  for _,device in ipairs(extensions.core_input_bindings.bindings) do
    if device.devname == "keyboard0" or device.devname == "mouse0" then
      for _, binding in ipairs(device.contents.bindings) do
        local actionmap = extensions.core_input_actions.getActiveActions()[binding.action].actionMap
        if actionmap and actionmap == "dynamicDecals" then
          local action = extensions.core_input_actions.getActiveActions()[binding.action]
          table.insert(inputActionInfo, {control = binding.control, controlCap = helper.capitalizeWords(binding.control), title = action.title, desc = action.desc})
        end
      end
    end
  end
  table.sort(inputActionInfo, function(a,b) return string.lower(a.control) < string.lower(b.control) end)
end

local function registerEditorPreferences(prefsRegistry)
  prefsRegistry:registerSubCategory("dynamicDecalsTool", "docs", nil, {
    {indentSpacing = {"int", 8, "Indent spacing of the items in the TOC", nil, 0, 64}},
    {verticalSpacing = {"int", 16, "Vertical spacing within the docs", nil, 0, 64}},
  })
end

local function editorPreferenceValueChanged(path, value)

end

local function doesEntryExist(sec, name)
  if not sec.children then return end
  for _, child in ipairs(sec.children) do
    if child.name == name then
      return child
    end
  end
  return nil
end

M.register = function(data)
  if type(data.section) ~= 'table' then
    editor.logWarn(logTag .. " - register(): section must be of type 'table'")
    return
  end

  local cur = docsSections

  for k, sectionName in ipairs(data.section) do
    if k ~= #data.section then
      local s = doesEntryExist(cur, sectionName)
      if s then
        cur = s
      else
        if not cur.children then cur.children = {} end
        local newSection = {name = sectionName, order = data.order, parent = cur}
        table.insert(cur.children, 1, newSection)
        cur = newSection
      end

    else -- last tbl entry
      local s = doesEntryExist(cur, sectionName)
      if s then
        s.fn = data.guiFn
      else
        if not cur.children then cur.children = {} end
        table.insert(cur.children, {name = sectionName, fn = data.guiFn, order = data.order, parent = cur})
      end
    end
  end
end

local function selectSectionInChildren(section, name)
  if not section.children then return end
  for _, child in ipairs(section.children) do
    if child.name == name then
      setScroll = true
      currentSection = child
      return
    end
    selectSectionInChildren(child, name)
  end
end

local function searchSectionInChildren(curSection, sections, index)
  local name = sections[index]
  if #sections == index then
    selectSectionInChildren(curSection, name)
  else
    for _, child in ipairs(curSection.children) do
      if child.name == name then
        curSection = child
        searchSectionInChildren(curSection, sections, index + 1)
      end
    end
  end
end

M.selectSection = function(section, doNotOpenWindow)
  if not doNotOpenWindow then
    M.showWindow()
  end

  if type(section) == "string" then
    im.ImGuiTextFilter_Clear(filter)
    docsSections = deepcopy(docsSectionsBase)
    selectSectionInChildren(docsSections, section)
  elseif type(section) == "table" then
    if #section == 0 then return end
    searchSectionInChildren(docsSections, section, 1)
  end
end

M.showWindow = function()
  sortSectionChildren(docsSections)
  docsSectionsBase = deepcopy(docsSections)

  editor.showWindow(windowName)
end

M.hideWindow = function()
  editor.hideWindow(windowName)
end

M.isWindowVisible = function()
  return editor.isWindowVisible(windowName)
end

M.image = image
M.verticalSpacing = verticalSpacing

M.registerEditorPreferences = registerEditorPreferences
M.editorPreferenceValueChanged = editorPreferenceValueChanged
M.setup = setup

return M