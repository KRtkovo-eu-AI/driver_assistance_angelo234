-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

--[[
  JUnit XML Writer for test results
  This module provides functionality to write test results to a JUnit-compatible XML file using slaxdom.

  Usage:
    - Call JUnitXMLWriter.write(results, outputFile) to write the results to a file.
]]

local SLAXML = require('libs/slaxml/slaxml')

local JUnitXMLWriter = {}

--[[
  Writes the test results to a JUnit XML file using slaxdom.

  @param results A table containing the test results.
  @param outputFile The file path to output the test results.
]]
function JUnitXMLWriter.write(results, outputFile)
  local testResults = results.testResults or {}
  local testFailCount = results.testFailCount or 0
  local totalDuration = results.totalDuration or 0

  -- Create the root element
  local doc = { type = "document", name = "#doc", kids = {} }

  -- <testsuites> element
  local testsuites = {
    type = "element",
    name = "testsuites",
    attr = {
      { type = "attribute", name = "tests", value = tostring(#testResults) },
      { type = "attribute", name = "failures", value = tostring(testFailCount) },
      { type = "attribute", name = "time", value = tostring(totalDuration) },
    },
    kids = {},
  }

  -- <testsuite> element
  local testsuite = {
    type = "element",
    name = "testsuite",
    attr = {
      { type = "attribute", name = "name", value = "LuaUnitTests" },
      { type = "attribute", name = "tests", value = tostring(#testResults) },
      { type = "attribute", name = "failures", value = tostring(testFailCount) },
      { type = "attribute", name = "time", value = tostring(totalDuration) },
    },
    kids = {},
  }

  -- Add test cases
  for _, result in ipairs(testResults) do
    local testcase = {
      type = "element",
      name = "testcase",
      attr = {
        { type = "attribute", name = "name", value = result.name },
        { type = "attribute", name = "time", value = tostring(result.duration) },
      },
      kids = {},
    }

    if not result.passed then
      local failure = {
        type = "element",
        name = "failure",
        attr = {
          { type = "attribute", name = "type", value = result.expectedToFail and "ExpectedFailure" or "Failure" },
        },
        kids = {
          {
            type = "text",
            name = "#text",
            value = result.errorMessage or "",
          },
        },
      }
      table.insert(testcase.kids, failure)
    end

    table.insert(testsuite.kids, testcase)
  end

  table.insert(testsuites.kids, testsuite)
  table.insert(doc.kids, testsuites)

  -- Serialize the XML document
  local xmlString = SLAXML:xml(doc, { indent = 2 })

  writeFile(outputFile, xmlString)
  log('I', '', 'Test results written to ' .. outputFile)
end

return JUnitXMLWriter
