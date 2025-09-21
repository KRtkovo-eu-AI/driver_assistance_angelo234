--[[
  XLSX Parsing Module for Lua
  Copyright 2024 BeamNG GmbH, Thomas Fischer <tfischer@beamng.gmbh>
]]
local M = {}
local slaxdom = require('libs/slaxml/slaxdom')

-- Helper function to get the local name of an XML element
local function getLocalName(element)
  return element.name:match("([^:]+)$")
end

-- Parse XML content from a ZIP file directly in memory
local function parseXMLContent(zip, fileName)
  local content = zip:readFileToMemory(fileName)
  if not content then
    error("Failed to read file content: " .. fileName)
  end
  return slaxdom:dom(content)
end

-- Parse relationships from a .rels file
local function parseRelationships(zip, relsFile)
  local relationships = {}
  local dom = parseXMLContent(zip, relsFile)
  for _, child in ipairs(dom.root.kids) do
    if getLocalName(child) == "Relationship" then
      local id = child.attr["Id"]
      local target = child.attr["Target"]
      local type = child.attr["Type"]
      relationships[id] = { target = target, type = type }
    end
  end
  return relationships
end

-- Parse shared strings from the sharedStrings.xml file
local function parseSharedStrings(zip, sharedStringsFile)
  local sharedStrings = {}
  local dom = parseXMLContent(zip, sharedStringsFile)
  for _, child in ipairs(dom.root.kids) do
    if getLocalName(child) == "si" then
      local textValue = ""
      for _, subChild in ipairs(child.kids) do
        if getLocalName(subChild) == "t" then
          -- Extract text from the text node
          if subChild.kids and subChild.kids[1] and subChild.kids[1].type == "text" then
            textValue = textValue .. subChild.kids[1].value
          else
            textValue = textValue .. ""
          end
        elseif getLocalName(subChild) == "r" then
          -- Rich text run
          for _, rChild in ipairs(subChild.kids) do
            if getLocalName(rChild) == "t" then
              if rChild.kids and rChild.kids[1] and rChild.kids[1].type == "text" then
                textValue = textValue .. rChild.kids[1].value
              else
                textValue = textValue .. ""
              end
            end
          end
        end
      end
      table.insert(sharedStrings, textValue)
    end
  end
  return sharedStrings
end

-- Parse sheet names and resolve their paths using relationships
local function parseWorkbookSheets(zip, workbookFile, workbookRels)
  local sheets = {}
  local dom = parseXMLContent(zip, workbookFile)

  for _, child in ipairs(dom.root.kids) do
    if getLocalName(child) == "sheets" then
      for _, sheet in ipairs(child.kids) do
        if getLocalName(sheet) == "sheet" then
          local name = sheet.attr["name"]
          local sheetId = sheet.attr["sheetId"]
          local rId = sheet.attr["r:id"] or sheet.attr["id"]
          if not rId then
            error("Sheet missing relationship ID: " .. name)
          end
          local rel = workbookRels[rId]
          if not rel then
            error("Relationship not found for sheet: " .. name)
          end
          -- Map sheet name to actual file path
          local target = rel.target
          if not target:match("^/") then
            target = "xl/" .. target
          end
          sheets[name] = target
        end
      end
    end
  end

  return sheets
end

-- Parse cell reference like A1, B2, $C$6
local function parseCellReference(ref)
  local col, row = ref:match("(%$?[A-Z]+)(%$?%d+)")
  if not col or not row then
    error("Invalid cell reference: " .. tostring(ref))
  end

  -- Remove '$' for absolute references
  col = col:gsub("%$", "")
  row = row:gsub("%$", "")

  -- Convert column part to a number (A=1, B=2, ..., Z=26, AA=27, etc.)
  local colIndex = 0
  for i = 1, #col do
    local char = col:sub(i, i):byte()
    if char < 65 or char > 90 then
      error("Invalid column character in cell reference: " .. tostring(ref))
    end
    colIndex = colIndex * 26 + (char - 64)
  end

  -- Convert row part to a number
  local rowIndex = tonumber(row)
  if not rowIndex then
    error("Invalid row number in cell reference: " .. tostring(ref))
  end

  return rowIndex, colIndex
end

-- Parse ranges like A1:C3, A:C, 1:3, B2, or B
local function parseRange(range)
  if range:match("^[A-Z]+$") then
    -- Entire column (e.g., "B")
    local _, col = parseCellReference(range .. "1")
    return nil, col, nil, col
  elseif range:match("^[0-9]+$") then
    -- Entire row (e.g., "2")
    local row = tonumber(range)
    return row, nil, row, nil
  elseif range:match("^[A-Z]+:[A-Z]+$") then
    -- Column range (e.g., "C:E")
    local startCol, endCol = range:match("^([A-Z]+):([A-Z]+)$")
    local _, startColIndex = parseCellReference(startCol .. "1")
    local _, endColIndex = parseCellReference(endCol .. "1")
    if endColIndex < startColIndex then
      error("Invalid column range boundaries: " .. tostring(range))
    end
    return nil, startColIndex, nil, endColIndex
  elseif range:match("^%d+:%d+$") then
    -- Row range (e.g., "2:5")
    local startRow, endRow = range:match("^(%d+):(%d+)$")
    startRow = tonumber(startRow)
    endRow = tonumber(endRow)
    if endRow < startRow then
      error("Invalid range boundaries: " .. tostring(range))
    end
    return startRow, nil, endRow, nil
  elseif range:match("^[A-Z]+[0-9]+$") then
    -- Single cell (e.g., "B2")
    local row, col = parseCellReference(range)
    return row, col, row, col
  elseif range:find(":") then
    -- Range with start and end (e.g., "A1:C3")
    local startRef, endRef = range:match("([^:]+):([^:]+)")
    if not startRef or not endRef then
      error("Invalid range format: " .. tostring(range))
    end
    local startRow, startCol = parseCellReference(startRef)
    local endRow, endCol = parseCellReference(endRef)
    if endRow < startRow or endCol < startCol then
      error("Invalid range boundaries: " .. tostring(range))
    end
    return startRow, startCol, endRow, endCol
  else
    error("Invalid range: " .. tostring(range))
  end
end

-- Helper function to determine if a row is entirely empty
local function isRowEmpty(sheetData, rowIndex, startCol, endCol)
  local row = sheetData[rowIndex]
  if not row then return true end
  for c = startCol, endCol do
    if row[c] ~= nil then
      return false
    end
  end
  return true
end

-- Helper function to determine if a column is entirely empty
local function isColumnEmpty(sheetData, colIndex, startRow, endRow)
  for r = startRow, endRow do
    local row = sheetData[r]
    if row and row[colIndex] ~= nil then
      return false
    end
  end
  return true
end

-- Extract data from a specified range
local function extractDataInRange(sheetData, startRow, startCol, endRow, endCol)
  local result = {}

  -- Determine maximum row and column indices
  local maxRow, maxCol = 0, 0
  for r, row in pairs(sheetData) do
    if r > maxRow then maxRow = r end
    for c in pairs(row) do
      if c > maxCol then maxCol = c end
    end
  end

  -- Handle Entire Row Selection
  if startRow and not startCol and endRow and not endCol then
    -- If selecting a single row, return its contents directly
    if startRow == endRow then
      local row = sheetData[startRow] or {}
      local rowData = {}
      for c = 1, maxCol do
        table.insert(rowData, row[c] or nil)
      end
      return rowData -- Return as a single-row array
    else
      for r = startRow, endRow do
        local row = sheetData[r] or {}
        local rowData = {}
        for c = 1, maxCol do
          table.insert(rowData, row[c] or nil)
        end
        table.insert(result, rowData)
      end
      return result
    end
  end

  -- Handle Entire Column Selection
  if startRow == nil and startCol and endCol and (startCol == endCol) then
    for r = 1, maxRow do
      local value = sheetData[r] and sheetData[r][startCol] or nil
      table.insert(result, value)
    end
    return result
  end

  -- Assign Default Values for Other Ranges
  startRow = startRow or 1
  endRow = endRow or maxRow
  startCol = startCol or 1
  endCol = endCol or maxCol

  -- Trim leading and trailing empty rows and columns
  while startRow <= endRow and isRowEmpty(sheetData, startRow, startCol, endCol) do
    startRow = startRow + 1
  end

  while endRow >= startRow and isRowEmpty(sheetData, endRow, startCol, endCol) do
    endRow = endRow - 1
  end

  while startCol <= endCol and isColumnEmpty(sheetData, startCol, startRow, endRow) do
    startCol = startCol + 1
  end

  while endCol >= startCol and isColumnEmpty(sheetData, endCol, startRow, endRow) do
    endCol = endCol - 1
  end

  -- Return early if no valid range remains
  if startRow > endRow or startCol > endCol then
    return result
  end

  -- Handle Cell Ranges
  for r = startRow, endRow do
    local rowData = {}
    local row = sheetData[r] or {}
    for c = startCol, endCol do
      -- Assign values sequentially to preserve positions, inserting nil for empty cells
      rowData[c - startCol + 1] = row[c] or nil
    end
    table.insert(result, rowData)
  end

  -- For single cell ranges, return the value directly
  if startRow == endRow and startCol == endCol then
    return sheetData[startRow] and sheetData[startRow][startCol] or nil
  end

  return result
end

-- Parse rows and cells from a sheet file
local function parseSheetData(zip, sheetFile, sharedStrings)
  local sheetData = {}
  local dom = parseXMLContent(zip, sheetFile)

  -- Iterate through children of <sheetData> within the worksheet
  for _, child in ipairs(dom.root.kids) do
    if getLocalName(child) == "sheetData" then
      for _, rowNode in ipairs(child.kids) do
        if getLocalName(rowNode) == "row" then
          local rowIndex = tonumber(rowNode.attr.r) -- Row index
          sheetData[rowIndex] = sheetData[rowIndex] or {}

          for _, cellNode in ipairs(rowNode.kids) do
            if getLocalName(cellNode) == "c" then
              local cellRef = cellNode.attr.r -- Cell reference (e.g., "B2")
              local rowIdx, colIdx = parseCellReference(cellRef)

              local cellType = cellNode.attr.t -- Cell type, e.g., "s" for shared strings
              local cellValue = nil

              -- Handle different cell types
              for _, cChild in ipairs(cellNode.kids) do
                if getLocalName(cChild) == "v" then
                  -- Extract the text value from the first child node
                  if cChild.kids and cChild.kids[1] and cChild.kids[1].type == "text" then
                    cellValue = cChild.kids[1].value
                  else
                    cellValue = ""
                  end
                elseif getLocalName(cChild) == "f" then
                  -- Formula (not evaluated)
                  cellValue = nil -- or store the formula if needed
                elseif getLocalName(cChild) == "is" then
                  -- Inline string
                  cellValue = ""
                  for _, isChild in ipairs(cChild.kids) do
                    if getLocalName(isChild) == "t" then
                      cellValue = cellValue .. isChild.value
                    elseif getLocalName(isChild) == "r" then
                      for _, rChild in ipairs(isChild.kids) do
                        if getLocalName(rChild) == "t" then
                          cellValue = cellValue .. rChild.value
                        end
                      end
                    end
                  end
                end
              end

              -- If it's a shared string, resolve the value
              if cellType == "s" then
                local sstIndex = tonumber(cellValue)
                if sstIndex == nil then
                  error("Invalid shared string index: " .. tostring(cellValue) .. " at cell " .. tostring(cellRef))
                end
                cellValue = sharedStrings[sstIndex + 1]
                if cellValue == nil then
                  error("Shared string not found for index: " .. tostring(sstIndex) .. " at cell " .. tostring(cellRef))
                end
              elseif cellType == "b" then
                -- Boolean
                cellValue = cellValue == "1"
              elseif cellType == "str" or cellType == "inlineStr" then
                -- String or inline string, already handled
              else
                -- Assume number
                cellValue = tonumber(cellValue) or cellValue
              end

              -- Assign the cell value if it's not empty
              if cellValue ~= nil and cellValue ~= "" then
                sheetData[rowIdx][colIdx] = cellValue
              end
            end
          end
        end
      end
    end
  end

  return sheetData
end

-- Helper function to check if a table contains a value
local function tableContains(t, value)
  for _, v in ipairs(t) do
    if v == value then
      return true
    end
  end
  return false
end

-- Helper function to get table keys
local function tableKeys(t)
  local keys = {}
  for k in pairs(t) do
    table.insert(keys, k)
  end
  return keys
end

-- Load an .xlsx file and extract shared strings and sheets
local function loadFileXLSX(filepath)
  local zip = ZipArchive()
  if not zip:openArchiveName(filepath, "r") then
    error("Failed to open .xlsx file: " .. filepath)
  end

  local fileList = zip:getFileList()
  if not fileList or #fileList == 0 then
    zip:close()
    error("Invalid .xlsx file: No files found in archive")
  end

  -- Parse relationships at package level
  local packageRels = parseRelationships(zip, "_rels/.rels")

  -- Find the workbook file path
  local workbookRel = nil
  for _, rel in pairs(packageRels) do
    if rel.type == "http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" then
      workbookRel = rel
      break
    end
  end
  if not workbookRel then
    zip:close()
    error("Workbook relationship not found")
  end

  local workbookFile = workbookRel.target
  if not workbookFile:match("^/") then
    workbookFile = workbookFile -- Relative path, no adjustment needed
  end

  -- Parse relationships for workbook
  local workbookRels = parseRelationships(zip, "/xl/_rels/workbook.xml.rels")

  -- Parse sheets from workbook.xml
  local sheets = parseWorkbookSheets(zip, workbookFile, workbookRels)

  -- Find shared strings file path
  local sharedStringsFile = nil
  for _, rel in pairs(workbookRels) do
    if rel.type == "http://schemas.openxmlformats.org/officeDocument/2006/relationships/sharedStrings" then
      sharedStringsFile = rel.target
      if not sharedStringsFile:match("^/") then
        sharedStringsFile = "/xl/" .. sharedStringsFile
      end
      break
    end
  end

  local sharedStrings = {}
  if sharedStringsFile and tableContains(fileList, sharedStringsFile) then
    sharedStrings = parseSharedStrings(zip, sharedStringsFile)
  end

  zip:close()
  return {
    filepath = filepath,
    sharedStrings = sharedStrings,
    sheets = sheets
  }
end

-- Get data from a specific sheet
local function getSheetData(xlsx, sheetName, range)
  -- Find the sheet file (case-insensitive search)
  if not sheetName then
    sheetName = next(xlsx.sheets)
  end
  if not sheetName then
    error("No sheet name found or provided")
  end
  local sheetFile = xlsx.sheets[sheetName] or xlsx.sheets[sheetName:lower()]
  if not sheetFile then
    -- Try to find the sheet ignoring case
    for name, path in pairs(xlsx.sheets) do
      if name:lower() == sheetName:lower() then
        sheetFile = path
        break
      end
    end
  end
  if not sheetFile then
    error("Sheet not found: " .. sheetName)
  end

  local zip = ZipArchive()
  if not zip:openArchiveName(xlsx.filepath, "r") then
    error("Failed to reopen .xlsx file: " .. xlsx.filepath)
  end

  local sheetData = parseSheetData(zip, sheetFile, xlsx.sharedStrings)
  zip:close()

  -- If range is provided, parse it into startRow, startCol, endRow, endCol
  if range then
    local startRow, startCol, endRow, endCol = parseRange(range)
    return extractDataInRange(sheetData, startRow, startCol, endRow, endCol)
  end

  -- Handle the entire sheet selection
  local startRow, startCol, endRow, endCol = 1, 1, nil, nil -- Entire sheet
  return extractDataInRange(sheetData, startRow, startCol, endRow, endCol)
end


-- Expose functions
M.loadFileXLSX = loadFileXLSX
M.getSheetData = getSheetData

return M
