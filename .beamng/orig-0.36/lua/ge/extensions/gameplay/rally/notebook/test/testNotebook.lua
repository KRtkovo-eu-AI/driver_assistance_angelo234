-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- USAGE
-- run this in the game lua console:
-- extensions.unload("gameplay_notebook_test_testNotebook") extensions.load("gameplay_notebook_test_testNotebook") gameplay_notebook_test_testNotebook.testAll()

-- local SettingsManager = require('/lua/ge/extensions/gameplay/rally/settingsManager')

local logTag = ''
local M = {}

local assertCounts = {
  total = 0,
  passed = 0,
  failed = 0,
}

local fname_notebook_test_v3_1     = '/gameplay/missions/driver_training/rallyStage/aip-test3/rally/notebooks/test_v3_1.notebook.json'
local fname_notebook_test_v3_1_out = '/gameplay/missions/driver_training/rallyStage/aip-test3/rally/notebooks/test_v3_1_out.notebook.json'

local function assertEqual(message, actual, expected)
  assertCounts.total = assertCounts.total + 1

  if actual ~= expected then
    assertCounts.failed = assertCounts.failed + 1
    message = message or "assertion failed"
    log('E', logTag, string.format("%s: expected '%s' but got '%s'", message, tostring(expected), tostring(actual)))
    error("test failed: "..message)
  else
    assertCounts.passed = assertCounts.passed + 1
    log('I', logTag, "test passed: "..message)
  end
end

local function loadNotebook(notebookFname)
  log('D', logTag, 'reading notebook file: ' .. notebookFname)
  local json = jsonReadFile(notebookFname)
  if not json then
    return nil, 'unable to read notebook file at: ' .. notebookFname
  end

  local notebook = require('/lua/ge/extensions/gameplay/rally/notebook/path')()
  notebook:setFname(notebookFname)
  notebook:onDeserialized(json)

  -- dump(notebook.pacenotes.sorted[1], 2)

  -- SettingsManager.load(notebook)

  return notebook
end

local function testStructured_deserialize()
  local notebook = loadNotebook(fname_notebook_test_v3_1)
  assertEqual('corner_degrees present', notebook.pacenotes.sorted[1].structured.fields.corner_degrees, -42)
end

local function testStructured_serialize()
  local notebook = loadNotebook(fname_notebook_test_v3_1)
  notebook:setFname(fname_notebook_test_v3_1_out)

  local newVal = 55
  notebook.pacenotes.sorted[1].structured.fields.corner_degrees = newVal
  notebook:save()

  local notebook = loadNotebook(fname_notebook_test_v3_1)
  assertEqual('corner_degrees same', notebook.pacenotes.sorted[1].structured.fields.corner_degrees, -42)

  local notebook_out = loadNotebook(fname_notebook_test_v3_1_out)
  assertEqual('corner_degrees changed', notebook_out.pacenotes.sorted[1].structured.fields.corner_degrees, newVal)
end

local function testAll()
  testStructured_deserialize()
  testStructured_serialize()
  dump(assertCounts)
end

M.testAll = testAll

return M
