-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local imguiUtils = require('ui/imguiUtils')
local jbeamIO = require('jbeam/io')
local jsonAST = require('json-ast')
local im = ui_imgui

local wndName = 'Part Text View'
M.menuEntry = wndName

local tableFlags = bit.bor(im.TableFlags_ScrollX, im.TableFlags_ScrollY, im.TableFlags_RowBg, im.TableFlags_Borders)

local clipper = nil

local initTextView = true
local maxLineLength = 1
local lineToScrollTo = -1
local editNodeIdx

local textColorDefault = im.GetColorU322(im.ImVec4(1, 1, 1, 1))
local nodeColorDefault = im.GetColorU322(im.ImVec4(1, 1, 1, 0.1))
local nodeColorHighlight = im.GetColorU322(im.ImVec4(1, 1, 1, 0.5))
local nodeColorHighlightRed = im.GetColorU322(im.ImVec4(1, 1, 0.5, 0.2))

local scrollToSelection = im.BoolPtr(true)

local nodeEditTextInput = im.ArrayChar(256)
local nodeEditDoubleInput = im.DoublePtr(0)

local colorTable = {
  ['string'] = im.ImVec4(0.31, 0.73, 1, 1),
  string_single = im.ImVec4(0.31, 0.73, 1, 1),
  comment = im.ImVec4(0.42, 0.6, 0.29, 1),
  comment_multiline = im.ImVec4(0.42, 0.6, 0.29, 1),
  list_begin = im.ImVec4(0.95, 0.84, 0.06, 1),
  list_end = im.ImVec4(0.95, 0.84, 0.06, 1),
  object_begin = im.ImVec4(0.85, 0.39, 0.63, 1),
  object_end = im.ImVec4(0.85, 0.39, 0.63, 1),
}

--local p = LuaProfiler("Part_Text_View")

local function save()
  writeFile(vEditor.astFilename, jsonAST.stringify(vEditor.ast.ast))
  log('I', '', 'Wrote file: '.. tostring(vEditor.astFilename))
end

local function _editNode(nodeIdx, node)
  local nodeType = node[1]
  editNodeIdx = nodeIdx
  log('I', '', '_editNode: ' .. tostring(nodeIdx))
  nodeEditTextInput[0] = 0
  nodeEditDoubleInput[0] = 0
  if nodeType == 'string' or nodeType == 'string_single' then
    nodeEditTextInput = im.ArrayChar(256, node[2])
  elseif nodeType == 'number' then
    nodeEditDoubleInput[0] = node[2]
  elseif nodeType == 'bool' then
    node[2] = not node[2]
    return
  end
  editor.openModalWindow("editASTNode")
end

local tempTbl = {}

local function _renderNode(nodeIdx, node)
  tempTbl[1] = node

  local text = jsonAST.stringifyNodes(tempTbl)
  local color = colorTable[node[1]]
  if color then
    im.PushStyleColor2(im.Col_Text, color)
  end
  im.PushID1(tostring(nodeIdx))
  im.TextUnformatted(text)
  if im.IsItemHovered() and im.IsMouseDoubleClicked(0) then
    _editNode(nodeIdx, node)
  end
  im.PopID()
  if color then
    im.PopStyleColor()
  end

  local nodeColor = nodeColorDefault
  if im.IsItemHovered() then
    nodeColor = nodeColorHighlight
    local rMin = im.GetItemRectMin()
    local rMax = im.GetItemRectMax()
    im.ImDrawList_AddRect(
      im.GetWindowDrawList(),
      rMin,
      rMax,
      nodeColor,
      0,
      nil,
      2
    )
    --if im.IsItemClicked(0) then
    --  node[1] = 'string'
    --  node[2] = 'Hello world :D'
    --end
  end

  local nodeSelected = vEditor.selectedASTNodeMap ~= nil and vEditor.selectedASTNodeMap[nodeIdx]

  if nodeSelected then
    local rMin = im.GetItemRectMin()
    local rMax = im.GetItemRectMax()
    im.ImDrawList_AddRectFilled(
      im.GetWindowDrawList(),
      rMin,
      rMax,
      nodeColorHighlightRed,
      0,
      nil,
      0
    )
  end

  return im.GetItemRectSize().x
end

local function _renderASTLines(clipper)
  local fontSize = im.GetFontSize()
  local numLines = #vEditor.ast.transient.linesIndexes

  -- Only on initializing the text view render all lines,
  -- in order to calculate the max horizontal scroll size
  local lineStart = initTextView and 1 or clipper.DisplayStart + 1
  local lineEnd = initTextView and numLines or clipper.DisplayEnd + 1

  im.ImGuiListClipper_Begin(clipper, numLines, fontSize)
  im.ImGuiListClipper_Step(clipper)
  for lineNo = lineStart, lineEnd, 1 do
    if lineNo > numLines then
      break
    end
    local nodeIdx = vEditor.ast.transient.linesIndexes[lineNo]
    -- render line number
    im.TableNextRow()
    im.TableNextColumn()
    im.TextUnformatted(tostring(lineNo))
    im.TableNextColumn()

    local lineLength = 0
    -- now render the line itself
    while nodeIdx <= #vEditor.ast.ast.nodes do
      local node = vEditor.ast.ast.nodes[nodeIdx]
      local nodeHierarchy = vEditor.ast.transient.hierarchy[nodeIdx]
      local nodeType = node[1]
      if nodeType == 'newline' or nodeType == 'newline_windows' then
        break
      end
      local nodeWidth = _renderNode(nodeIdx, node)
      lineLength = lineLength + nodeWidth
      im.SameLine()
      nodeIdx = nodeIdx + 1
    end
    if initTextView then
      maxLineLength = math.max(maxLineLength, lineLength)
    end
  end
  im.ImGuiListClipper_End(clipper)
  --dump{"we only rendered these lines: ", 'clipper.DisplayStart = ', clipper.DisplayStart + 1, 'clipper.DisplayEnd = ', clipper.DisplayEnd + 1, 'lineCount =', #vEditor.ast.transient.linesIndexes}
end

local columnFlags = bit.bor(im.TableColumnFlags_NoHide, im.TableFlags_ScrollX)
local oldPartName, oldResult, success

local function onEditorGui()
  if not vEditor.vehicle or not vEditor.vehData or not vEditor.selectedPart then return end

  local partName = vEditor.selectedPart
  local ioCtx = vEditor.vehData.ioCtx

  -- On part selected changed
  if oldPartName ~= partName then
    oldPartName = partName
    success, oldResult = pcall(function() return {jbeamIO.getPart(ioCtx, partName)} end)
    if not success then return end

    initTextView = true
  end

  if not oldResult then return end
  local part = oldResult[1]
  local jbeamFilename = oldResult[2]

  if editor.beginWindow(wndName, wndName, im.WindowFlags_MenuBar) then
    if im.BeginMenuBar() then
      if im.MenuItem1("Reload") then
        vEditor.ast = nil
      end
      if im.MenuItem1("Save") then
        save()
      end
      if im.MenuItem1("Close") then
        vEditor.selectedPart = nil
        return
      end
      if im.MenuItem1("Delete") then
        FS:removeFile(jbeamFilename)
        ast = nil
      end
      im.Checkbox("Scroll to selection", scrollToSelection)
      im.TextUnformatted(tostring(vEditor.selectedPart) .. ' - ')
      if jbeamFilename then
        im.TextUnformatted(tostring(jbeamFilename))
      end
      im.EndMenuBar()
    end

    if jbeamFilename then
      im.SameLine()
      if im.Button('explore') then
        Engine.Platform.exploreFolder(jbeamFilename)
      end
      local stat = FS:stat(jbeamFilename)
      if stat then
        im.SameLine()
        im.TextUnformatted('File times: ')
        im.SameLine()
        if stat.modtime ~= stat.createtime then
          im.TextUnformatted('created: ' .. os.date("%x %H:%M", stat.createtime) .. ' - modified: ' .. os.date("%x %H:%M", stat.modtime))
        else
          im.TextUnformatted('created: ' .. os.date("%x %H:%M", stat.createtime))
        end
      end
    end

    if vEditor.ast and vEditor.ast.transient.linesIndexes then
      im.PushStyleVar2(im.StyleVar_ItemSpacing, im.ImVec2(0, 2))
      im.PushFont2(1) -- 1= monospace? PushFont3("cairo_semibold_large")

      --im.SetNextWindowContentSize(im.ImVec2(maxLineLength, 0))

      if initTextView then
        maxLineLength = 1
        clipper = im.ImGuiListClipper()
      end

      -- Scroll to selected node(s) on first selection
      local scrollToSel = scrollToSelection[0]
      if scrollToSel and vEditor.scrollToNode then
        local numSelectedLines = 0
        local sumSelectedLineNums = 0

        for nodeIdx, v in pairs(vEditor.selectedASTNodeMap) do
          numSelectedLines = numSelectedLines + 1
          sumSelectedLineNums = sumSelectedLineNums + vEditor.ast.transient.nodeIdxToLineNum[nodeIdx]
        end

        vEditor.scrollToNode = false
        lineToScrollTo = sumSelectedLineNums / numSelectedLines
      end

      -- Render text view
      if im.BeginTable('astTable', 2, tableFlags) then
        im.TableSetupColumn('', columnFlags, 0)
        im.TableSetupColumn('', columnFlags, maxLineLength)

        im.TableSetupScrollFreeze(1, 0) -- Make line numbers always visible

        _renderASTLines(clipper)

        if lineToScrollTo ~= -1 then
          local itemPosY = clipper.StartPosY + clipper.ItemsHeight * lineToScrollTo
          im.SetScrollFromPosY(itemPosY)
          lineToScrollTo = -1
        end

        if initTextView then
          initTextView = false
        end
        im.EndTable()
      end

      im.PopFont()
      im.PopStyleVar()
    end
  end
  editor.endWindow()

  if editor.beginModalWindow("editASTNode", "Edit Node") and editNodeIdx then
    local node = vEditor.ast.ast.nodes[editNodeIdx]
    local nodeHierarchy = vEditor.ast.transient.hierarchy[editNodeIdx]
    local nodeType = node[1]
    im.TextUnformatted(tostring(nodeType) .. ' - ' .. tostring(editNodeIdx))
    im.TextUnformatted('Raw node data: ' .. dumps(node))
    if nodeType == 'string' or nodeType == 'string_single' then
      im.InputText('##nodeEditTextInput', nodeEditTextInput)
    elseif nodeType == 'number' then
      im.InputScalar('##nodeEditScalarInput', im.DataType_Double, nodeEditDoubleInput)
    elseif nodeType == 'bool' then
      im.Checkbox('##nodeEditBoolInput', nodeEditBoolInput)
    end
    --im.SetKeyboardFocusHere(0)
    im.Separator()
    if im.Button("Apply") then
      if nodeType == 'string' or nodeType == 'string_single' then
        node[2] = ffi.string(nodeEditTextInput)
      elseif nodeType == 'number' then
        node[2] = nodeEditDoubleInput[0]
      end
      editor.closeModalWindow("editASTNode")
      editNodeIdx = nil
    end
    im.SameLine()
    if im.Button("Cancel") then
      editor.closeModalWindow("editASTNode")
      editNodeIdx = nil
    end
  end
  editor.endModalWindow()
end

local function open()
  editor.showWindow(wndName)
end

local function onEditorInitialized()
  editor.registerWindow(wndName, im.ImVec2(500,400))
  editor.registerModalWindow("editASTNode")
end

M.onEditorGui = onEditorGui
M.open = open
M.onEditorInitialized = onEditorInitialized

return M