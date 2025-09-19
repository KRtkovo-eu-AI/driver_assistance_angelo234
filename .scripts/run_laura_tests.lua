#!/usr/bin/env lua

local laura = require('laura')
local Context = laura.Context
local Labels = laura.Labels
local Reporter = laura.Reporter
local Runner = laura.Runner
local Runnable = laura.Runnable
local Terminal = laura.Terminal

local errorx = require('laura.ext.errorx')
local fs = require('laura.util.fs')
local helpers = require('laura.util.helpers')

local args = {...}
local dir = args[1] or 'spec'
local junit_path = args[2] or 'reports/laura.xml'
local summary_path = args[3] or 'reports/laura_summary.json'

local ctx = Context.global()
ctx.config.Dir = dir
ctx.config.TestPattern = '*_test.lua'

local function ensure_parent(path)
  local sep = fs.PathSep
  local esc = sep:gsub('(%W)', '%%%1')
  local parent = path:match('^(.*)' .. esc .. '[^' .. esc .. ']+$')
  if not parent or parent == '' then
    return
  end

  local function dir_exists(dir)
    if dir == '' then
      return true
    end
    if select(1, fs.exists(dir)) then
      return true
    end
    if select(1, fs.exists(dir .. sep)) then
      return true
    end
    return false
  end

  if dir_exists(parent) then
    return
  end

  local command
  if sep == '\\' then
    command = string.format('if not exist "%s" mkdir "%s"', parent, parent)
  else
    local escaped = parent:gsub("'", "'\\''")
    command = string.format("mkdir -p '%s'", escaped)
  end

  local ok = os.execute(command)
  if ok ~= true and ok ~= 0 then
    io.stderr:write(('Unable to create directory %s\n'):format(parent))
    os.exit(ctx.config._Exit.SysErr)
  end

  if not dir_exists(parent) then
    io.stderr:write(('Directory %s not found after creation attempt\n'):format(parent))
    os.exit(ctx.config._Exit.SysErr)
  end
end

local function write_file(path, content)
  ensure_parent(path)
  local fh, err = io.open(path, 'w')
  if not fh then
    io.stderr:write(('Unable to write %s: %s\n'):format(path, err or 'unknown'))
    os.exit(ctx.config._Exit.SysErr)
  end
  fh:write(content)
  fh:close()
end

local function xml_escape(text)
  text = tostring(text or '')
  text = text:gsub('&', '&amp;')
             :gsub('<', '&lt;')
             :gsub('>', '&gt;')
             :gsub('"', '&quot;')
             :gsub("'", '&apos;')
  return text
end

local function xml_attr(text)
  return '"' .. xml_escape(text) .. '"'
end

local search_dir = dir
local dir_exists = select(1, fs.exists(dir))
if fs.isdir(dir) then
  search_dir = dir
elseif dir_exists then
  search_dir = dir .. fs.PathSep .. '.'
elseif dir ~= '.' and dir ~= './' and dir ~= '.\\' then
  io.stderr:write(dir .. ' ' .. Labels.ErrorNotADir .. '\n')
  os.exit(ctx.config._Exit.SysErr)
end

local files, file_count = fs.getFiles(search_dir)
if file_count == 0 then
  local xml = {
    '<?xml version="1.0" encoding="UTF-8"?>',
    string.format('<testsuite name=%s tests="0" failures="0" errors="0" skipped="0" time="0.000000"></testsuite>', xml_attr('Laura tests'))
  }
  write_file(junit_path, table.concat(xml, '\n'))
  write_file(summary_path, '{"tests":0,"passed":0,"failures":0,"errors":0,"skipped":0,"time":0.0}')
  print(Labels.NoTests)
  os.exit(ctx.config._Exit.OK)
end

local load_errors = {}
for fname in helpers.spairs(files) do
  local chunk, err = loadfile(fname, 't', _G)
  if not chunk then
    load_errors[#load_errors + 1] = { name = fname, message = err or Labels.ErrorSyntax }
  else
    local ok, exec_err = xpcall(chunk, debug.traceback)
    if not ok then
      load_errors[#load_errors + 1] = { name = fname, message = exec_err }
    end
  end
end

if #load_errors > 0 then
  local xml = { '<?xml version="1.0" encoding="UTF-8"?>' }
  xml[#xml + 1] = string.format('<testsuite name=%s tests="%d" failures="0" errors="%d" skipped="0" time="0.000000">', xml_attr('Laura tests'), #load_errors, #load_errors)
  for _, item in ipairs(load_errors) do
    xml[#xml + 1] = string.format('  <testcase classname=%s name=%s>', xml_attr('loader'), xml_attr(item.name))
    xml[#xml + 1] = string.format('    <error message=%s>%s</error>', xml_attr(item.message), xml_escape(item.message))
    xml[#xml + 1] = '  </testcase>'
  end
  xml[#xml + 1] = '</testsuite>'
  write_file(junit_path, table.concat(xml, '\n'))
  write_file(summary_path, string.format('{"tests":%d,"passed":0,"failures":0,"errors":%d,"skipped":0,"time":0.0}', #load_errors, #load_errors))
  for _, item in ipairs(load_errors) do
    io.stderr:write(item.name .. ': ' .. item.message .. '\n')
  end
  Terminal.restore()
  print(Labels.ResultFailed)
  os.exit(ctx.config._Exit.SysErr)
end

local runner = Runner:new()
local results = runner:runTests()
local reporter = Reporter:new(results)
reporter:reportErrors()
reporter:finalSummary(0)

local root = ctx.root
local cases = {}
Runnable.traverse(root, function(node)
  if not node:isSuite() then
    cases[#cases + 1] = node
  end
end)

local total = results.total or #cases
local failures = #results.failing
local skipped = #results.skipping
local errors = 0
local passed = total - failures - skipped - errors
if passed < 0 then passed = 0 end
local duration = results.duration or 0

local function build_classname(node)
  local names = {}
  local parent = node.parent
  while parent do
    if parent.description and parent.description ~= ctx.config._rootKey then
      table.insert(names, 1, parent.description)
    end
    parent = parent.parent
  end
  if #names == 0 then
    return 'Laura tests'
  end
  return table.concat(names, ' :: ')
end

local xml = { '<?xml version="1.0" encoding="UTF-8"?>' }
xml[#xml + 1] = string.format('<testsuite name=%s tests="%d" failures="%d" errors="%d" skipped="%d" time="%.6f">', xml_attr('Laura tests'), total, failures, errors, skipped, duration)
for _, node in ipairs(cases) do
  local classname = build_classname(node)
  local name = node.description or 'unnamed test'
  local time = node.execTime or 0
  xml[#xml + 1] = string.format('  <testcase classname=%s name=%s time="%.6f">', xml_attr(classname), xml_attr(name), time)
  if node:isSkipped() then
    xml[#xml + 1] = '    <skipped />'
  elseif node:isFailed() then
    local err = node.error
    local message = err and err.title or Labels.ErrorUnknown
    local details = err and errorx.toString(err, false) or ''
    xml[#xml + 1] = string.format('    <failure message=%s>%s</failure>', xml_attr(message), xml_escape(details))
  end
  xml[#xml + 1] = '  </testcase>'
end
xml[#xml + 1] = '</testsuite>'
write_file(junit_path, table.concat(xml, '\n'))

local summary = string.format('{"tests":%d,"passed":%d,"failures":%d,"errors":%d,"skipped":%d,"time":%.6f}', total, passed, failures, errors, skipped, duration)
write_file(summary_path, summary)

Terminal.restore()
local exit_code
if failures > 0 then
  print(Labels.ResultFailed)
  exit_code = ctx.config._Exit.Failed
else
  print(Labels.ResultPass)
  exit_code = ctx.config._Exit.OK
end

os.exit(exit_code)
