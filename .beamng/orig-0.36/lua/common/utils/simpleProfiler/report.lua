-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- extensions.utils_simpleProfiler_report.createReport('vehicle_spawn.html')

local M = {}

local lustache = require('common/libs/lustach/src/lustache')

local templateFile = "lua/common/utils/simpleProfiler/interactiveBarChart.mustache.html"

--------------------------------------------------------------------------------
-- The main function for generating a flame chart / HTML report
--------------------------------------------------------------------------------
local function createReport(filebasename, reportTitle)
  if not simpleProfilerGetJournal then
    log('E','simpleProfilerReport',"simpleProfiler is not available.")
    return
  end

  filebasename = filebasename or 'profilerReport'
  log('I','simpleProfilerReport',"Creating report for " .. filebasename .. " ...")
  local root = simpleProfilerGetJournal(true, 0.001) or { children = {}, stats = {} }

  local dataForTemplate = {
    reportTitle = reportTitle or 'Profiler Summary',
    date        = os.date("%Y-%m-%d %H:%M:%S"),
    totalTimeString = string.format("%.3f", root.stats.duration or 0),
    profilerJSON = jsonEncode(root),
  }

  local templateStr = readFile(templateFile)
  if not templateStr then
    log('E','simpleProfilerReport',"Failed to read template file: "..tostring(templateFile))
    return
  end

  local html = lustache:render(templateStr, dataForTemplate)
  writeFile(filebasename .. '.html', html)
  log('I', 'simpleProfilerReport', 'Report created: ' .. filebasename .. '.html')
end


M.createReport = createReport

return M