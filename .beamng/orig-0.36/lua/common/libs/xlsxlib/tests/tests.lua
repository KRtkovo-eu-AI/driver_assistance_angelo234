-- Unit tests for the xlsx library
-- Copyright 2024 BeamNG GmbH, Thomas Fischer <tfischer@beamng.gmbh>

-- how to use:
--
-- rerequire('libs/xlsxlib/tests/tests').runTests()
--

local xlsxLib = require('libs/xlsxlib/xlsxlib')
local testManager = require('libs/xlsxlib/tests/TestManager')
local dataPath = 'lua/common/libs/xlsxlib/tests/'

local M = {}

local function normalizeTableKeys(t)
  local normalized = {}
  for k, v in pairs(t) do
    if type(k) == "string" and tonumber(k) then
      normalized[tonumber(k)] = v
    else
      normalized[k] = v
    end
  end
  return normalized
end

-- Function to compare two tables for equality
local function tablesEqual(t1, t2)
  if t1 == t2 then return true end
  if type(t1) ~= "table" or type(t2) ~= "table" then return false end

  t1 = normalizeTableKeys(t1)
  t2 = normalizeTableKeys(t2)

  for k, v in pairs(t1) do
    if type(v) == "table" and type(t2[k]) == "table" then
      if not tablesEqual(v, t2[k]) then return false end
    else
      if v ~= t2[k] then return false end
    end
  end

  for k in pairs(t2) do
    if t1[k] == nil then return false end
  end

  return true
end

-- Helper function for a simple table comparison test
local function compareTest(context, inputFile, expectedFile, queryArgs)
  local xlsxData = xlsxLib.loadFileXLSX(dataPath .. inputFile)
  local data = xlsxLib.getSheetData(xlsxData, unpack(queryArgs))
  local fn = dataPath .. expectedFile
  if not FS:fileExists(fn) then

    local content = string.format('// Test args: %s\r\n', serialize(queryArgs))
    content = content .. jsonEncodePretty({data}, 1)
    writeFile(fn, content)
    log('W', '', 'Wrote test output for the first time, please check the results: ' .. tostring(fn))
  end
  local expectedData = jsonReadFile(fn)[1]
  local equal = tablesEqual(data, expectedData)
  if not equal then
    local txt = 'tables are not the same:\n'
    txt = txt .. ' === Provided data\n'
    txt = txt .. dumps{ data}
    txt = txt .. '\n\n === Expected data\n'
    txt = txt .. dumps{expectedData}
    context:fail(txt)
  end
end

-- Define your Tests table with test methods
local Tests = {}

function Tests:test_nonExistentSheet_shouldFail(context)
  compareTest(context, 'test_input_1.xlsx', '', {'NonExistentSheet'})
end

function Tests:test_firstSheet(context)
  compareTest(context, 'test_input_1.xlsx', 'test_firstSheet_expected.json', {})
end

function Tests:test_selectSingleCell(context)
  compareTest(context, 'test_input_1.xlsx', 'test_selectSingleCell_expected.json', {'Sheet1', 'B2'})
end

function Tests:test_selectEntireColumn(context)
  compareTest(context, 'test_input_1.xlsx', 'test_selectEntireColumn_expected.json', {'Sheet1', 'B'})
end

function Tests:test_selectEntireRow(context)
  compareTest(context, 'test_input_1.xlsx', 'test_selectEntireRow_expected.json', {'Sheet1', '2'})
end

function Tests:test_selectColumnRange(context)
  compareTest(context, 'test_input_1.xlsx', 'test_selectColumnRange_expected.json', {'Sheet1', 'C:E'})
end

function Tests:test_selectRowRange(context)
  compareTest(context, 'test_input_1.xlsx', 'test_selectRowRange_expected.json', {'Sheet1', '2:5'})
end

function Tests:test_selectCellRange(context)
  compareTest(context, 'test_input_1.xlsx', 'test_selectCellRange_expected.json', {'Sheet1', 'A1:C3'})
end

function Tests:test_selectMixedAbsoluteRelative(context)
  compareTest(context, 'test_input_1.xlsx', 'test_selectMixedAbsoluteRelative_expected.json', {'Sheet1', '$A$1:C$3'})
end

function Tests:test_selectInvalidRange_shouldFail(context)
  compareTest(context, 'test_input_1.xlsx', '', {'Sheet1', 'InvalidRange'})
end

function Tests:test_selectEntireSheet(context)
  compareTest(context, 'test_input_1.xlsx', 'test_selectEntireSheet_expected.json', {'Sheet1'})
end

-- Additional tests can be added here following the same pattern

local function runTests()
  FS:directoryCreate('/junit-results/')
  testManager:run(Tests, '/junit-results/xlsxlib-results.xml')
end

M.runTests = runTests

return M
