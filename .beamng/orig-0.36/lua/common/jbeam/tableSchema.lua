--[[
This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
If a copy of the bCDDL was not distributed with this
file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
This module contains a set of functions which manipulate behaviours of vehicles.
]]

local M = {}

local min, max = math.min, math.max
local str_byte, str_sub, str_len, str_find = string.byte, string.sub, string.len, string.find

local jbeamUtils = require("jbeam/utils")
local particles = require("particles")
local csvlib = require('csvlib')


local materials, materialsMap = particles.getMaterialsParticlesTable()

-- these are defined in C, do not change the values
local NORMALTYPE = 0
local NODE_FIXED = 1
local NONCOLLIDABLE = 2
local BEAM_ANISOTROPIC = 1
local BEAM_BOUNDED = 2
local BEAM_PRESSURED = 3
local BEAM_LBEAM = 4
local BEAM_HYDRO = 6
local BEAM_SUPPORT = 7


local specialVals = {FLT_MAX = math.huge, MINUS_FLT_MAX = -math.huge}
local typeIds = {
  NORMAL = NORMALTYPE,
  HYDRO = BEAM_HYDRO,
  ANISOTROPIC = BEAM_ANISOTROPIC,
  TIRESIDE = BEAM_ANISOTROPIC,
  BOUNDED = BEAM_BOUNDED,
  PRESSURED = BEAM_PRESSURED,
  SUPPORT = BEAM_SUPPORT,
  LBEAM = BEAM_LBEAM,
  FIXED = NODE_FIXED,
  NONCOLLIDABLE = NONCOLLIDABLE,
  SIGNAL_LEFT = 1,   -- GFX_SIGNAL_LEFT
  SIGNAL_RIGHT = 2,  -- GFX_SIGNAL_RIGHT
  HEADLIGHT = 4,     -- GFX_HEADLIGHT
  BRAKELIGHT = 8,    -- GFX_BRAKELIGHT
  RUNNINGLIGHT = 16, -- GFX_RUNNINGLIGHT
  REVERSELIGHT = 32, -- GFX_REVERSELIGHT
}

local function replaceSpecialValues(val)
  local typeval = type(val)
  if typeval == "table" then
    -- recursive replace
    for k, v in pairs(val) do
      val[k] = replaceSpecialValues(v)
    end
    return val
  end
  if typeval ~= "string" then
    -- only replace strings
    return val
  end

  if specialVals[val] then return specialVals[val] end

  if string.byte(val, 1) == 124 then -- |
    local parts = split(val, "|", 999)
    local ival = 0
    for i = 2, #parts do
      local valuePart = parts[i]
      -- is it a node material?
      if valuePart:sub(1,3) == "NM_" then
        ival = particles.getMaterialIDByName(materials, valuePart:sub(4))
        --log('D', "jbeam.replaceSpecialValues", "replaced "..valuePart.." with "..ival)
      end
      ival = bit.bor(ival, typeIds[valuePart] or 0)
    end
    return ival
  end
  return val
end


local function processJbeamTableRow(ctx, rowValue)
  if type(rowValue) ~= "table" then
    log('W', "", "*** Invalid table row: "..dumps(rowValue))
    return -1
  end
  if tableIsDict(rowValue) then
    if rowValue.include and csvlib then
      local data = csvlib.readFileCSV(rowValue.include, rowValue.delimiter)
      if not data then
        log('E', '', 'unable to read CSV file: ' .. tostring(rowValue.include))
        return
      end
      if #data[1] ~= ctx.headerSize then
        log('E', '', 'CSV file has mismatching header. Required: ' .. dumps(ctx.header) .. ' - present in file: ' .. dumps(headerCsv))
        return
      end
      for i, _ in ipairs(ctx.header) do
        if ctx.header[i] ~= data[1][i] then
          log('E', '', 'CSV file has mismatching header column ' .. tostring(i) .. '. Required: ' .. dumps(ctx.header) .. ' - present in file: ' .. dumps(data[1]))
          return
        end
      end

      local startRowCounter = ctx.rowCounter
      for i = 2, #data do
        local rowValueCsv = data[i]
        processJbeamTableRow(ctx, rowValueCsv, ctx.newList, ctx.omitWarnings)
        ctx.rowCounter = ctx.rowCounter + 1
      end
      --log('I', '', 'Successfully read ' .. tostring(ctx.rowCounter - startRowCounter) .. ' rows from file ' .. tostring(rowValue.include))
      rowValue.include = nil
    end
    -- case where options is a dict on its own, filling a whole line
    tableMerge(ctx.localOptions, replaceSpecialValues(rowValue))
    ctx.localOptions.__astNodeIdx = nil
  else
    local newID = ctx.rowCounter
    local rowValueSize = #rowValue
    --log('D', "" *** "..tostring(ctx.rowCounter).." = "..tostring(rowValue).." ["..type(rowValue).."]")

    -- allow last type to be the options always
    if rowValueSize > ctx.headerSize1 then -- and type(rowValue[#rowValue]) ~= "table" then
      if not ctx.omitWarnings then
        log('W', "", "*** Invalid table header, must be as long as all table cells (plus one additional options column):")
        log('W', "", "*** Table header: "..dumps(ctx.header))
        log('W', "", "*** Mismatched row: "..dumps(rowValue))
      end
      return -1
    end

    -- walk the table row
    -- replace row: reassociate the header colums as keys to the row cells
    local newRow = deepcopy(ctx.localOptions)

    -- check if inline options are provided, merge them then
    for rk = ctx.headerSize1, rowValueSize do
      local rv = rowValue[rk]
      if tableIsDict(rv) then
        tableMerge(newRow, replaceSpecialValues(rv))
        -- remove the options
        rowValue[rk] = nil -- remove them for now
        ctx.header[rk] = "options" -- for fixing some code below - let it know those are the options
        break
      end
    end

    local disableRow = (newRow.disable == true) or (type(newRow.variables) == 'table' and newRow.variables.disable == true)

    if not disableRow then
      newRow.__astNodeIdx = rowValue.__astNodeIdx

      -- now care about the rest
      for rk,rv in ipairs(rowValue) do
        --log('D', "jbeam.", "### "..header[rk].."//"..tostring(newRow[header[rk]]))
        -- if there is a local option named like a row key, use the option instead
        -- copy things
        if ctx.header[rk] == nil then
          log('E', "", "*** unable to parse row, header for entry is missing: ")
          log('E', "", "*** header: "..dumps(ctx.header) .. ' missing key: ' .. tostring(rk) .. ' -- is the section header too short?')
          log('E', "", "*** row: "..dumps(rowValue))
        else
          newRow[ctx.header[rk]] = replaceSpecialValues(rv)
        end
      end

      if newRow.id ~= nil then
        newID = newRow.id
        newRow.name = newRow.id -- this keeps the name for debugging or alike
        newRow.id = nil
      end

      -- done with that row
      ctx.newList[newID] = newRow
      ctx.newListSize = ctx.newListSize + 1
    end
  end
end

local function processTableWithSchemaDestructive(jbeamTable, newList, inputOptions, omitWarnings)
  -- its a list, so a table for us. Verify that the first row is the header
  local header = jbeamTable[1]
  if type(header) ~= "table" then
    if not omitWarnings then
      log('W', "", "*** Invalid table header: " .. dumpsz(header, 2))
    end
    return -1
  end
  if tableIsDict(header) then
    if not omitWarnings then
      log('W', "", "*** Invalid table header, must be a list, not a dict: "..dumps(header))
    end
    return -1
  end

  local ctx = {}
  ctx.rowCounter = 1
  ctx.header = header
  ctx.headerSize = #header
  ctx.headerSize1 = ctx.headerSize + 1
  ctx.newListSize = 0
  ctx.localOptions = replaceSpecialValues(deepcopy(inputOptions)) or {}
  ctx.newList = newList
  ctx.omitWarnings = omitWarnings

  -- remove the header from the data, as we dont need it anymore
  table.remove(jbeamTable, 1)
  --log('D', ""header size: "..ctx.headerSize)

  -- walk the list entries
  for _, rowValue in ipairs(jbeamTable) do
    processJbeamTableRow(ctx, rowValue)
    ctx.rowCounter = ctx.rowCounter + 1
  end

  newList.__astNodeIdx = jbeamTable.__astNodeIdx

  return ctx.newListSize
end

local function process(vehicle, processSlotsTable, omitWarnings)
  profilerPushEvent('jbeam/tableSchema.process')

  --log('D', "","- Preparing jbeam")
  -- check for nodes key
  vehicle.maxIDs = {}
  vehicle.validTables = {}
  vehicle.beams = vehicle.beams or {}

  -- create empty options
  vehicle.options = vehicle.options or {}
  -- walk everything and look for options
  for keyEntry, entry in pairs(vehicle) do
    if type(entry) ~= "table" then
      -- seems to be a option, add it to the vehicle options
      vehicle.options[keyEntry] = entry
      vehicle[keyEntry] = nil
    end
  end

  -- then walk all (keys) / entries of that vehicle
  for keyEntry, entry in pairs(vehicle) do
    -- verify key names to be proper formatted
    --[[
    if type(entry) == "table" and tableIsDict(entry) then
      log('D', ""," ** "..tostring(keyEntry).." = [DICT] #" ..tableSize(entry))
    elseif type(entry) == "table" and not tableIsDict(entry) then
      log('D', ""," ** "..tostring(keyEntry).." = [LIST] #"..tableSize(entry))
    else
      log('D', ""," ** "..tostring(keyEntry).." = "..tostring(entry).." ["..type(entry).."]")
    end
    ]]--

    -- verify element name
    if string.match(keyEntry, "^([a-zA-Z_]+[a-zA-Z0-9_]*)$") == nil then
      log('E', "","*** Invalid attribute name '"..keyEntry.."'")
      profilerPopEvent('jbeam/tableSchema.process')
      return false
    end

    -- init max
    vehicle.maxIDs[keyEntry] = 0
    --log('D', ""," ** creating max val "..tostring(keyEntry).." = "..tostring(vehicle.maxIDs[keyEntry]))
    -- then walk the tables
    if type(entry) == "table" and not tableIsDict(entry) and jbeamUtils.ignoreSections[keyEntry] == nil and not tableIsEmpty(entry) then
      if tableIsDict(entry) then
        -- ENTRY DICTS TO BE WRITTEN
      else
        if (keyEntry == 'slots' or keyEntry == 'slots2') and not processSlotsTable then
          -- slots are preprocessed in the io module
          vehicle.validTables[keyEntry] = true
        else
          if not vehicle.validTables[keyEntry] then
            local newList = {}
            local newListSize = processTableWithSchemaDestructive(entry, newList, vehicle.options, omitWarnings)
              -- this was a correct table, record that so we do not process twice
            if newListSize > 0 then
              vehicle.validTables[keyEntry] = true
            end
            vehicle[keyEntry] = newList
            --log('D', ""," - "..tostring(newListSize).." "..tostring(keyEntry))
          end
        end
      end
    end
  end
  profilerPopEvent('jbeam/tableSchema.process')
  return true
end

M.process = process
M.processTableWithSchemaDestructive = processTableWithSchemaDestructive

return M