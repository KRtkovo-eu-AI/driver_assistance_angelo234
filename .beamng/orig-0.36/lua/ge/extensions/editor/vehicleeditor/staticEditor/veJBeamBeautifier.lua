-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local im = extensions.ui_imgui
local jbeamTableSchema = require('jbeam/tableSchema')
local jbeamIO = require('jbeam/io')
local jsonAST = require('json-ast')
local jsonDebug = require('jsonDebug')

local wndName = "JBeam Beautifier"
M.menuEntry = "JBeam Beautifier"

local windowOpen = im.BoolPtr(false)

local sectionsToBeautifyTblPtr = {
  {'beams', im.BoolPtr(false)},
  {'nodes', im.BoolPtr(true)},
  {'quads', im.BoolPtr(false)},
  {'slots', im.BoolPtr(false)},
  {'slots2', im.BoolPtr(false)},
  {'triangles', im.BoolPtr(true)},
}

local alignHeaderEnabledPtr = im.BoolPtr(true)
local roundingEnabledPtr = im.BoolPtr(false)
local decimalPlacesPtr = im.IntPtr(3)

local directoryToBeautifyPtr = im.ArrayChar(1024, '')

local function jsonDebugDecode(content, context)
  local state, data, warnings = xpcall(function() return jsonDebug.decode(content, context) end, debug.traceback)
  if state == false then
    log('E', "jsonDecode", "unable to decode JSON: "..tostring(context))
    log('E', "jsonDecode", "JSON decoding error: "..tostring(data))
    return nil
  end
  for _, warning in ipairs(warnings) do
    log('W', 'jsonDecode', warning)
  end
  return data, warnings
end

local function isLastElementInArray(astNodes, currASTIdx, endASTIdx)
  for i = currASTIdx + 1, endASTIdx do
    local astNode = astNodes[i]
    if astNode[1] ~= 'array_delimiter' and astNode[1] ~= 'space' and astNode[1] ~= 'tab' then
      return false
    end
  end
  return true
end

local function doesArraySpanMultipleLines(astNodes, startASTIdx, endASTIdx)
  for i = startASTIdx, endASTIdx do
    local astNode = astNodes[i]
    if astNode[1] == 'newline' or astNode[1] == 'newline_windows' then
      return true
    end
  end
  return false
end

local function getSectionsWidthPerColumn(astData)
  local partsSectionsColumnWidths = {}
  local partsSectionsMaxColumnWidths = {}

  local astNodes = astData.ast.nodes
  local transientData = astData.transient
  local luaDataRaw = transientData.luaDataRaw
  local hierarchy = transientData.hierarchy

  for partName, part in pairs(luaDataRaw) do
    if partName ~= '__astNodeIdx' then
      partsSectionsMaxColumnWidths[partName] = {}
      partsSectionsColumnWidths[partName] = {}

      for _, sectionToBeautifyData in ipairs(sectionsToBeautifyTblPtr) do
        local sectionName, toBeautify = sectionToBeautifyData[1], sectionToBeautifyData[2][0]
        if toBeautify then
          local section = part[sectionName]
          if section then
            partsSectionsMaxColumnWidths[partName][sectionName] = {}
            partsSectionsColumnWidths[partName][sectionName] = {}

            local sectionMaxColumnWidths = partsSectionsMaxColumnWidths[partName][sectionName]
            local sectionColumnWidths = partsSectionsColumnWidths[partName][sectionName]

            for rowIdx, rowData in ipairs(section) do
              if alignHeaderEnabledPtr[0] or rowIdx > 1 then
                local rowStartASTNodeIdx = rowData.__astNodeIdx
                local rowStartASTNode = astNodes[rowStartASTNodeIdx]
                if rowStartASTNode[1] == 'list_begin' then
                  local rowASTNodeIdxs = hierarchy[rowStartASTNodeIdx]
                  local rowASTNodeIdxsSize = #rowASTNodeIdxs
                  local rowASTNodeIdxStart, rowASTNodeIdxEnd = rowASTNodeIdxs[1], rowASTNodeIdxs[#rowASTNodeIdxs]

                  if not doesArraySpanMultipleLines(astNodes, rowASTNodeIdxStart, rowASTNodeIdxEnd) then
                    sectionColumnWidths[rowIdx] = {}
                    local i = 1
                    local inArrDictLevel = 0
                    local arrDictStart = -1

                    for k, astIdx in ipairs(rowASTNodeIdxs) do
                      if k ~= rowASTNodeIdxsSize then
                        local astNode = astNodes[astIdx]
                        local astNodeType = astNode[1]

                        if inArrDictLevel == 1 then
                          local arrDictNodes = {}
                          for j = arrDictStart, astIdx - 1 do
                            table.insert(arrDictNodes, astNodes[j])
                          end
                          local columnWidth = #jsonAST.stringifyNodes(arrDictNodes)
                          sectionMaxColumnWidths[i] = math.max(sectionMaxColumnWidths[i] or 0, columnWidth)
                          sectionColumnWidths[rowIdx][i] = columnWidth
                          i = i + 1
                          inArrDictLevel = 0
                        end

                        if astNodeType == 'list_begin' or astNodeType == 'object_begin' then
                          arrDictStart = astIdx
                          inArrDictLevel = inArrDictLevel + 1
                        end

                        if inArrDictLevel == 0 then
                          if astNodeType ~= 'array_delimiter' and astNodeType ~= 'space' and astNodeType ~= 'tab' then
                            local columnWidth = 0
                            if astNodeType == 'number' then
                              if roundingEnabledPtr[0] then
                                astNode[2] = roundNear(astNode[2], 10^-decimalPlacesPtr[0])
                                astNode[3] = decimalPlacesPtr[0]
                              end
                              if sign2(astNode[2]) == -1 then
                                columnWidth = columnWidth - 1
                              end
                            end
                            columnWidth = columnWidth + #jsonAST.stringifyNode(astNode)
                            sectionMaxColumnWidths[i] = math.max(sectionMaxColumnWidths[i] or 0, columnWidth)
                            sectionColumnWidths[rowIdx][i] = columnWidth
                            i = i + 1
                          end
                        end
                      end
                    end
                  end
                end
              end
            end
          end
        end
      end
    end
  end

  return partsSectionsColumnWidths, partsSectionsMaxColumnWidths
end

local function beautifySections(astData, partsSectionsColumnWidths, partsSectionsMaxColumnWidths)
  local astNodes = astData.ast.nodes
  local transientData = astData.transient
  local luaDataRaw = transientData.luaDataRaw
  local hierarchy = transientData.hierarchy

  local offset = 0

  local astIdxToPart = {}
  for partName, part in pairs(luaDataRaw) do
    if partName ~= '__astNodeIdx' then
      astIdxToPart[part.__astNodeIdx] = partName
    end
  end

  for _, partASTIdx in ipairs(tableKeysSorted(astIdxToPart)) do
    local partName = astIdxToPart[partASTIdx]
    local part = luaDataRaw[partName]

    local astIdxToSection = {}
    for _, sectionToBeautifyData in ipairs(sectionsToBeautifyTblPtr) do
      local sectionName, toBeautify = sectionToBeautifyData[1], sectionToBeautifyData[2][0]
      if toBeautify then
        local section = part[sectionName]
        if section then
          astIdxToSection[section.__astNodeIdx] = sectionName
        end
      end
    end

    for _, sectionASTIdx in ipairs(tableKeysSorted(astIdxToSection)) do
      local sectionName = astIdxToSection[sectionASTIdx]
      local section = part[sectionName]

      local sectionMaxColumnWidths = partsSectionsMaxColumnWidths[partName][sectionName]
      local sectionColumnWidths = partsSectionsColumnWidths[partName][sectionName]

      for rowIdx, rowData in ipairs(section) do
        if alignHeaderEnabledPtr[0] or rowIdx > 1 then
          local rowStartASTNodeIdx = rowData.__astNodeIdx
          local rowStartASTNode = astNodes[rowStartASTNodeIdx + offset]
          if rowStartASTNode[1] == 'list_begin' then
            local rowASTNodeIdxs = hierarchy[rowStartASTNodeIdx]
            local rowASTNodeIdxStart, rowASTNodeIdxEnd = rowASTNodeIdxs[1], rowASTNodeIdxs[#rowASTNodeIdxs]

            if not doesArraySpanMultipleLines(astNodes, rowASTNodeIdxStart + offset, rowASTNodeIdxEnd + offset) then
              local inited = false
              local i = 1
              local j = rowASTNodeIdxStart
              local endIdx = rowASTNodeIdxEnd
              local astIdx = j
              local inArrDictLevel = 0
              local enteredArrDictFlag = false

              while astIdx < endIdx do
                if inited then
                  local astNode = astNodes[astIdx]
                  local astNodeType = astNode[1]

                  if astNodeType == 'list_begin' or astNodeType == 'object_begin' then
                    if inArrDictLevel == 0 then
                      enteredArrDictFlag = true
                    end
                    inArrDictLevel = inArrDictLevel + 1
                  elseif astNodeType == 'list_end' or astNodeType == 'object_end' then
                    inArrDictLevel = inArrDictLevel - 1
                  end

                  if (enteredArrDictFlag or inArrDictLevel == 0) and astNodeType ~= 'list_end' and astNodeType ~= 'object_end' then
                    if astNodeType ~= 'array_delimiter' and astNodeType ~= 'space' and astNodeType ~= 'tab' then
                      table.insert(astNodes, astIdx, {'array_delimiter'})
                      offset = offset + 1
                      astIdx = j + offset

                      local spacing = sectionMaxColumnWidths[i] - sectionColumnWidths[rowIdx][i] + 1
                      if astNodeType == 'number' then
                        if sign2(astNode[2]) == -1 then
                          spacing = spacing - 1
                        end
                      end

                      table.insert(astNodes, astIdx, {'space', spacing})
                      offset = offset + 1
                      astIdx = j + offset
                      i = i + 1
                    else
                      table.remove(astNodes, astIdx)
                      offset = offset - 1
                      astIdx = j + offset
                    end
                  end
                else
                  inited = true
                end
                j = j + 1
                astIdx = j + offset
                endIdx = rowASTNodeIdxEnd + offset
                enteredArrDictFlag = false
              end
            end
          end
        end
      end
    end
  end
end

-- From: https://web.archive.org/web/20131225070434/http://snippets.luacode.org/snippets/Deep_Comparison_of_Two_Values_3
-- available under MIT/X11
local function deepcompare(t1,t2)
  local ty1 = type(t1)
  local ty2 = type(t2)
  if ty1 ~= ty2 then return false end
  -- non-table types can be directly compared
  if ty1 ~= 'table' then
    if ty1 == 'number' then
      local t1New = roundingEnabledPtr[0] and roundNear(t1, 10^-decimalPlacesPtr[0]) or t1
      return math.abs(t1New - t2) < 0.0001
    else
      return t1 == t2
    end
  end

  local testedKeys = {}
  for k1, v1 in pairs(t1) do
    local v2 = t2[k1]
    if v2 == nil or not deepcompare(v1, v2) then return false end
    testedKeys[k1] = true
  end
  for k2, v2 in pairs(t2) do
    if not testedKeys[k2] then
      local v1 = t1[k2]
      if v1 == nil or not deepcompare(v1, v2) then return false end
    end
  end
  return true
end

local function test(origJBeamStr, newJBeamStr)
  local origData = jsonDecode(origJBeamStr)
  local newData = jsonDecode(newJBeamStr)

  if origData == nil or newData == nil then
    return false
  end

  return deepcompare(origData, newData)
end

local function beautifyJBeamFile(filePath)
  local origJBeamStr = readFile(filePath)
  if not origJBeamStr then
    log('E', '', 'Failed to read JBeam file, aborting beautifying')
    return false
  end
  local data, warnings = jsonDebugDecode(origJBeamStr)
  if not data then
    log('E', '', 'Failed to JSON parse JBeam file, aborting beautifying')
    return false
  end
  if #warnings > 0 then
    log('E', '', 'JBeam file has warnings, aborting beautifying')
    return false
  end
  local astData = jsonAST.parse(origJBeamStr, true)
  if not astData then
    log('E', '', 'Failed to JSON AST parse JBeam file, aborting beautifying')
    return false
  end

  local partsSectionsColumnWidths, partsSectionsMaxColumnWidths = getSectionsWidthPerColumn(astData)
  beautifySections(astData, partsSectionsColumnWidths, partsSectionsMaxColumnWidths)

  local newJBeamStr = jsonAST.stringify(astData.ast)

  -- local res = test(origJBeamStr, newJBeamStr)
  -- if not res then
  --   log('E', '', 'Original and new JBeam data don\'t match')
  --   return false
  -- end

  writeFile(filePath, newJBeamStr)
  return true
end

local function beautifyJBeamFiles(pathToBeautify)
  local jbeamFilePaths = path.is_file(pathToBeautify) and {pathToBeautify} or FS:findFiles(pathToBeautify, "*.jbeam", -1, false, false)
  --local jbeamFilePaths = FS:findFiles('/', "*.jbeam", -1, false, false)
  --local jbeamFilePaths = {'vehicles/bx/bx_doors.jbeam'}

  for _, jbeamFilePath in ipairs(jbeamFilePaths) do
    local res = beautifyJBeamFile(jbeamFilePath)
    if res then
      log('I', '', 'Sucessfully beautified: ' .. jbeamFilePath)
    else
      log('E', '', 'Failed to beautify: ' .. jbeamFilePath)
    end
  end
  print('Done!')
end

local function onUpdate()
  if windowOpen[0] ~= true then return end

  if im.Begin(wndName, windowOpen) then
    im.PushFont3("cairo_semibold_large")
    im.Text("Sections to Beautify:")
    im.PopFont()
    for k,v in ipairs(sectionsToBeautifyTblPtr) do
      local sectionName, boolPtr = v[1], v[2]
      im.Checkbox(sectionName, boolPtr)
    end

    im.Spacing()
    im.Spacing()
    im.Spacing()

    im.PushFont3("cairo_semibold_large")
    im.Text("Settings:")
    im.PopFont()
    if im.Checkbox("Align JBeam Header Row", alignHeaderEnabledPtr) then end
    if im.Checkbox("Round Numbers", roundingEnabledPtr) then end
    im.SliderInt("Decimal Places", decimalPlacesPtr, 0, 5)
    im.InputText("##directoryToBeautify", directoryToBeautifyPtr)
    if im.Button("Select Folder to Beautify") then
      -- Opens a file dialog to choose JBeam file to load
      editor_fileDialog.openFile(function(data)
        ffi.copy(directoryToBeautifyPtr, data.path)
      end, {{"JBeam files", ".jbeam"}}, true, "/vehicles/")
    end
    im.SameLine()
    if im.Button("Select File to Beautify") then
      -- Opens a file dialog to choose JBeam file to load
      editor_fileDialog.openFile(function(data)
        ffi.copy(directoryToBeautifyPtr, data.filepath)
      end, {{"JBeam files", ".jbeam"}}, false, "/vehicles/")
    end

    im.Spacing()
    im.Spacing()
    im.Spacing()

    local directoryToBeautify = ffi.string(directoryToBeautifyPtr)
    local beautiftyEnabled = directoryToBeautify ~= ''

    if not beautiftyEnabled then
      im.BeginDisabled()
    end
    im.PushFont3("cairo_semibold_large")
    if im.Button("Beautify JBeam Files") then
      beautifyJBeamFiles(directoryToBeautify)
    end
    im.PopFont()
    if not beautiftyEnabled then
      im.EndDisabled()
    end
  end

  ::continue::
  im.End()
end

local function open()
  windowOpen[0] = true
end

local function onSerialize()
  return {
    windowOpen = windowOpen[0],
  }
end

local function onDeserialized(data)
  windowOpen[0] = data.windowOpen
end

M.open = open

M.onUpdate = onUpdate

M.onSerialize = onSerialize
M.onDeserialized = onDeserialized

return M