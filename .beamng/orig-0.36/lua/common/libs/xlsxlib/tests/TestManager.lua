--[[
  Unit Test Manager for the xlsx library
  Copyright 2024 BeamNG GmbH, Thomas Fischer <tfischer@beamng.gmbh>

  This module provides a simple test manager for running unit tests on the xlsx library.
  It collects test functions, executes them, logs the results, and outputs the results
  in JUnit XML format for integration with other tools.

  Usage:
    - Define your test functions in a separate table.
    - Each test function should be named starting with "test".
    - If a test is expected to fail, append "_shouldFail" to its name.
    - Use the TestManager to run your tests.

  Example:
    -- See the example usage at the end of this file.
]]

-- Import required modules
local xlsxLib = require('libs.xlsxlib.xlsxlib')
local JUnitXMLWriter = require('libs/xlsxlib/tests/JUnitXMLWriter')
local json = require('json') -- Ensure you have a JSON library compatible with Lua
local socket = require('socket')

local M = {} -- Contains the public interface

local dataPath = 'lua/common/libs/xlsxlib/tests/'

-- TestManager class
local TestManager = {
  -- Log levels
  LogAlways = 1,
  LogErrors = 2,
  LogFails = 3,

  __internal = {
    LogDefault = 1, -- Default to LogAlways
    testPassCount = 0,
    testFailCount = 0,
    testResults = {}, -- Store individual test results
  },

  --[[
    Sets the default log level.

    @param self The TestManager instance.
    @param logLevel The desired log level.
  ]]
  setLogDefault = function(self, logLevel)
    self.__internal.LogDefault = logLevel
  end,

  --[[
    Runs the provided tests.

    @param self The TestManager instance.
    @param tests A table containing test functions.
    @param outputFile Optional. The file path to output the test results in JUnit XML format.
  ]]
  run = function(self, tests, outputFile)

    -- Collect and sort test names
    local testNames = {}
    for name, func in pairs(tests) do
      if type(func) == "function" and name:match("^test") then
        table.insert(testNames, name)
      else
        log('E', '', 'Unknown test function: ' .. tostring(name))
      end
    end
    table.sort(testNames)
    log('I', '', 'Running ' .. #testNames .. ' tests ... ')

    -- Run all test methods in the tests table
    local startTime = socket.gettime()
    for _, name in ipairs(testNames) do
      local func = tests[name]
      local displayName = string.format("%-60s", name:gsub('_shouldFail$', ''))

      local textFuncContext = {
        fail = function(ctx, message)
          error(message or "Test failed")
        end,
        success = function(ctx, message)
          ctx.successMessage = message
        end,
        successMessage = nil,
        -- You can add more fields to the context if needed
      }

      local testStartTime = socket.gettime()
      local status, err = xpcall(function() func(tests, textFuncContext) end, function(e) return e end)
      local testEndTime = socket.gettime()
      local duration = testEndTime - testStartTime

      local expectedToFail = name:match("_shouldFail$") ~= nil
      local testPassed = (status ~= expectedToFail)

      -- Store test result
      local testResult = {
        name = name,
        passed = testPassed,
        expectedToFail = expectedToFail,
        duration = duration,
      }

      if not testPassed then
        testResult.errorMessage = err
      else
        testResult.successMessage = textFuncContext.successMessage
      end

      table.insert(self.__internal.testResults, testResult)

      if testPassed then
        self.__internal.testPassCount = self.__internal.testPassCount + 1
        if expectedToFail then
          log('I', '', ' - ' .. displayName .. " >>> PASSED (expected failure)")
        else
          if textFuncContext.successMessage then
            log('I', '', ' - ' .. displayName .. " >>> PASSED: " .. textFuncContext.successMessage)
          else
            log('I', '', ' - ' .. displayName .. " >>> PASSED")
          end
        end
      else
        self.__internal.testFailCount = self.__internal.testFailCount + 1
        if expectedToFail then
          log('E', '', ' - ' .. displayName .. " >>> FAILED (expected failure but test passed)")
        else
          log('E', '', ' - ' .. displayName .. " >>> FAILED: " .. tostring(err))
        end
      end
    end
    local endTime = socket.gettime()
    local totalDuration = endTime - startTime

    log('I', '', "Test PASS count: " .. tostring(self.__internal.testPassCount))
    if self.__internal.testFailCount > 0 then
      log('I', '', "Test FAIL count: " .. tostring(self.__internal.testFailCount))
    end

    -- Output results to JUnit XML if outputFile is provided
    if outputFile then
      -- Prepare the results table
      local results = {
        testPassCount = self.__internal.testPassCount,
        testFailCount = self.__internal.testFailCount,
        testResults = self.__internal.testResults,
        totalDuration = totalDuration,
      }

      -- Write the results to JUnit XML
      JUnitXMLWriter.write(results, outputFile)
    end

    -- Reset counts after running tests
    self.__internal.testPassCount = 0
    self.__internal.testFailCount = 0
    self.__internal.testResults = {}
  end,
}

return TestManager

--[[ Example Usage:

-- Import the TestManager
local testManager = require('libs/xlsxlib/tests/TestManager')

-- Define your tests in a table
local tests = {}

-- Test functions should start with "test"
function tests.testExampleSuccess(context)
  -- Your test code here
  local result = 42  -- Replace with actual test logic
  local expectedValue = 42
  if result == expectedValue then
    context:success("Result matches expected value")
  else
    context:fail("Result does not match expected value")
  end
end

function tests.testExampleFailure_shouldFail(context)
  -- This test is expected to fail
  context:fail("Intentional failure")
end

-- Run the tests and output results to 'test-results.xml'
testManager:run(tests, 'test-results.xml')

]]
