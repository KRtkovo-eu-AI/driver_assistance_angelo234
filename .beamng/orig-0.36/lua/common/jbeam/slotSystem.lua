--[[
This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
If a copy of the bCDDL was not distributed with this
file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
This module contains a set of functions which manipulate behaviours of vehicles.
]]

local M = {}

local min, max = math.min, math.max
local str_byte, str_sub, str_len, str_find = string.byte, string.sub, string.len, string.find

local jbeamIO = require('jbeam/io')

--[[
LUA 5.1 compatible

Ordered Table
keys added will be also be stored in a metatable to recall the insertion order
metakeys can be seen with for i,k in ( <this>:ipairs()  or ipairs( <this>._korder ) ) do
ipairs( ) is a bit faster

variable names inside __index shouldn't be added, if so you must delete these again to access the metavariable
or change the metavariable names, except for the 'del' command. thats the reason why one cannot change its value
]] --
local function newT(t)
  local mt = {}
  -- set methods
  mt.__index = {
    -- set key order table inside __index for faster lookup
    _korder = {},
    -- traversal of hidden values
    hidden = function() return pairs(mt.__index) end,
    -- traversal of table ordered: returning index, key
    ipairs = function(self) return ipairs(self._korder) end,
    -- traversal of table
    pairs = function(self) return pairs(self) end,
    -- traversal of table ordered: returning key,value
    opairs = function(self)
      local i = 0
      local function iter(self)
        i = i + 1
        local k = self._korder[i]
        if k then
          return k, self[k]
        end
      end
      return iter, self
    end,
    -- to be able to delete entries we must write a delete function
    del = function(self, key)
      if self[key] then
        self[key] = nil
        for i, k in ipairs(self._korder) do
          if k == key then
            table.remove(self._korder, i)
            return
          end
        end
      end
    end,
  }
  -- set new index handling
  mt.__newindex = function(self, k, v)
    if k ~= "del" and v then
      rawset(self, k, v)
      table.insert(self._korder, k)
    end
  end
  return setmetatable(t or {}, mt)
end

local function unifyParts(target, source_raw, level, slotOptions, partPath)
  local source = deepcopy(source_raw)
  --log('I', "jbeam.unifyParts",string.rep(" ", level).."* merging part "..tostring(source.partName).."{".. tostring(source) .. "}["..dumps(source.slotType).."] => "..tostring(target.partName).." ["..tostring(target.slotType).."] ... ")
  -- walk and merge all sections
  for sectionKey, section in pairs(source) do
    if sectionKey == 'slots' or sectionKey == 'slots2' or sectionKey == "information" then
      goto continue
    end

    --log('D', "jbeam.unifyParts"," *** "..tostring(sectionKey).." = "..tostring(section).." ["..type(section).."] -> "..tostring(sectionKey).." = "..tostring(target[sectionKey]).." ["..type(target[sectionKey]).."]")
    if target[sectionKey] == nil then
      -- easy merge
      target[sectionKey] = section

      -- care about the slotoptions if we are first
      if type(section) == "table" and not tableIsDict(section) then
        local localSlotOptions = deepcopy(slotOptions) or {}
        localSlotOptions.partOrigin = source.partName
        localSlotOptions.partPath = partPath
        --localSlotOptions.partLevel = level
        table.insert(target[sectionKey], 2, localSlotOptions)
        -- now we need to negate the slotoptions out again
        local slotOptionReset = {}
        for k4, v4 in pairs(localSlotOptions) do
          slotOptionReset[k4] = ""
        end
        table.insert(target[sectionKey], slotOptionReset)
      end
    elseif type(target[sectionKey]) == "table" and type(section) == "table" then
      -- append to existing tables
      -- add info where this came from
      local counter = 0
      local localSlotOptions = nil
      for k3, v3 in pairs(section) do
        if tonumber(k3) ~= nil then
          -- if its an index, append if index > 1
          if counter > 0 then
            table.insert(target[sectionKey], v3)
          else
            localSlotOptions = deepcopy(slotOptions) or {}
            localSlotOptions.partOrigin = source.partName
            localSlotOptions.partPath = partPath
            --localSlotOptions.partLevel = level
            --localSlotOptions.partOrigin = sectionKey .. '/' .. source.partName
            table.insert(target[sectionKey], localSlotOptions)
          end
        else
          --it's a key value pair, check how to proceed with merging potentially existing values
          -- check if magic $ appears in the KEY, if new value is a number (for example "$+MyFoo": 42)
          if type(v3) == "number" and str_byte(k3, 1) == 36 then
            local actualK3 = k3:sub(3) --remove the magic chars at the beginning to get the actual KEY, this can potentially lead to issues if k3 omits the second magic char
            local existingValue = target[sectionKey][actualK3]

            local existingModifierValue = target[sectionKey][k3] --in case we are trying to merge a modifier with another modifier, we need to check if this is the case
            if type(existingModifierValue) == "number" then
              --we need to merge a new modifier with an existing modifier, to do that, set our existing value of actualK3 to the existing value of the raw k3 (including the modifier syntax)
              existingValue = existingModifierValue
              --also overwrite the key to be a modifier again (foo -> $+foo), this way the merged value will be written as a modifier value
              actualK3 = k3
            end

            if type(existingValue) == "number" then --check if old value is also a number (and not null)
              local secondChar = str_byte(k3, 2)

              if secondChar == 43 then -- +/sum
                target[sectionKey][actualK3] = existingValue + v3 --do a sum
              elseif secondChar == 42 then -- * / multiplication
                target[sectionKey][actualK3] = existingValue * v3 -- do a multiplication
              elseif secondChar == 60 then -- < / min
                target[sectionKey][actualK3] = min(existingValue, v3) -- use the min
              elseif secondChar == 62 then -- > / max
                target[sectionKey][actualK3] = max(existingValue, v3) -- use the max
              else
                target[sectionKey][k3] = v3
              end
            else
              --we have special merging, but the initial value is no number (or nil), so just pass the modifier value onto the merged data.
              --This specifically does NOT strip the modifier syntax from k3 so that parent parts still know that this is a modifier
              target[sectionKey][k3] = v3
            end
          else
            --we have a regular value, no special merging, just overwrite it
            target[sectionKey][k3] = v3
          end
        end
        counter = counter + 1
      end
      if localSlotOptions then
        -- now we need to negate the slotoptions out again
        local slotOptionReset = {}
        for k4, v4 in pairs(localSlotOptions) do
          slotOptionReset[k4] = ""
        end
        table.insert(target[sectionKey], slotOptionReset)
      end

    else
      -- just overwrite any basic data
      if sectionKey ~= "slotType" and sectionKey ~= "partName" then
        target[sectionKey] = section
      end
    end
    ::continue::
  end
end

-- figures out if a certain part can go into a slot or if we should refuse loading
local function partFitsSlot(part, slot)
  -- slot version 1 support
  if slot.type then
    if type(part.slotType) == 'string' and part.slotType == slot.type then
      return true, nil
    elseif type(part.slotType) == 'table' and not tableContains(part.slotType, slot.type) then
      return true, nil
    end
    return false, "Part type does not match slot type"

  -- slot version 2
  elseif slot.allowTypes then
    -- case 1: the slotType on the part side is a string only
    if type(part.slotType) == 'string' then
      local fits = tableContains(slot.allowTypes, part.slotType)
      return fits, fits and nil or "Part type not in allowed types"

    -- case 2: the slottype on the part is a table
    elseif type(part.slotType) == 'table' then
      local allowListed = false
      for _, slottype in ipairs(part.slotType) do
        if tableContains(slot.allowTypes, slottype) then
          allowListed = true
          break
        end
      end

      local denyListed = false
      for _, slottype in ipairs(part.slotType) do
        if tableContains(slot.denyTypes, slottype) then
          denyListed = true
          break
        end
      end

      if not allowListed then
        return false, "No matching allowed types found"
      end
      if denyListed then
        return false, "Part type is in deny list"
      end
      return true, nil
    end
  end
  return false, "Invalid slot configuration"
end

local function fillSlots_rec(
  ioCtx,
  slotMap,
  vehicleConfig,
  userPartNode,
  currentPart,
  level,
  _slotOptions,
  chosenPartsTree,
  slotPartMap,
  activePartsData,
  activeParts,
  path,
  unifyJournal,
  unifyJournalC
)
  local originalPath = path
  profilerPushEvent(originalPath)
  if level > 50 then
    log("E", "jbeam.fillSlots", "ERROR: more than 50 recursion levels; check for cycles")
    return
  end

  local function getChildNodeForSlotId(parentNode, slotId)
    if parentNode and parentNode.children then
      return parentNode.children[slotId]
    end
    return nil
  end

  local slots = currentPart.slots2 or currentPart.slots
  for _, slotDef in ipairs(slots) do
    local slotOptions = deepcopy(_slotOptions) or {}
    slotOptions = tableMerge(slotOptions, deepcopy(slotDef))

    local slotId = slotDef.name or slotDef.type
    local newPath = path .. slotId .. '/'

    -- Remove unneeded properties
    slotOptions.name = nil
    slotOptions.type = nil
    slotOptions.allowTypes = nil
    slotOptions.denyTypes = nil
    slotOptions.default = nil
    slotOptions.description = nil

    -- Prepare a node for chosenPartsTree
    local slotTreeEntry = {
      id                    = slotId,
      path                  = newPath,
      --suitablePartNames   = {},
      --unsuitablePartNames = {},
      chosenPartName        = nil,
      partPath              = nil,
      children              = nil,
    }

    -- We'll figure out userPartName below
    local userPartName

    -- If we have vehicleConfig.parts, it's an old flat map
    if vehicleConfig.parts then
      local desiredPartName = vehicleConfig.parts[slotTreeEntry.id]
      if desiredPartName == "none" then
        userPartName = ""
      else
        userPartName = desiredPartName
      end
      -- it might be stored with the path as well, so lets try that
      if not userPartName then
        userPartName = vehicleConfig.parts[slotTreeEntry.path]
      end

    -- Otherwise, if we have vehicleConfig.partsTree, it's the new dictionary
    elseif vehicleConfig.partsTree then
      local parentNode = userPartNode or vehicleConfig.partsTree
      local childNode = getChildNodeForSlotId(parentNode, slotId)
      if childNode then
        if childNode.chosenPartName == "none" then
          userPartName = ""
        else
          userPartName = childNode.chosenPartName
        end
      end
    end

    -- 1) Gather available parts for the slot
    profilerPushEvent('getCompatiblePartNamesForSlot')
    slotTreeEntry.suitablePartNames, slotTreeEntry.unsuitablePartNames = jbeamIO.getCompatiblePartNamesForSlot(ioCtx, slotDef, slotMap)
    profilerPopEvent('getCompatiblePartNamesForSlot')

    -- user explicitly wants this slot empty; do not try to fill with defaults
    if userPartName == '' then
      slotTreeEntry.chosenPartName = ''
      slotTreeEntry.decisionMethod = 'user-empty'
      chosenPartsTree[slotId] = slotTreeEntry
      slotPartMap[newPath] = ''
      goto continue
    end

    -- 2) Attempt to load userPartName
    local chosenPart, chosenPartName
    if userPartName then
      chosenPart = jbeamIO.getPart(ioCtx, userPartName)
      if chosenPart then
        chosenPartName = chosenPart.partName
        if not partFitsSlot(chosenPart, slotDef) then
          log('E', 'slotSystem', 'Chosen part has wrong slot type. Required is ' .. tostring(slotDef.type) .. ' provided by part ' .. tostring(userPartName) .. ' is ' .. dumps(chosenPart.slotType) .. '. Resetting to default')
          chosenPart, chosenPartName = nil, nil
        end
        slotTreeEntry.decisionMethod = 'user'
      else
        log('E', "jbeam.fillSlots", 'slot "' .. tostring(slotId) .. '" reset to default part "' .. tostring(slotDef.default) .. '" as the wished part "' .. tostring(userPartName) .. '" was not found')
      end
    end

    -- 3) Fallback to slotDef.default if user didn't choose anything
    if slotDef.default and not chosenPart then
      if slotDef.default == '' then
        slotTreeEntry.chosenPartName = ''
        slotTreeEntry.decisionMethod = 'default-empty'
        chosenPartsTree[slotId] = slotTreeEntry
        slotPartMap[newPath] = ''
        goto continue
      else
        chosenPartName = slotDef.default
        slotTreeEntry.decisionMethod = 'default'
        chosenPart = jbeamIO.getPart(ioCtx, slotDef.default)
      end
    end

    chosenPartsTree[slotId] = slotTreeEntry

    if chosenPart then
      -- if userPartName ~= chosenPartName then
      --   dump{"wished for part:" .. tostring(userPartName) .. " found part:" .. tostring(chosenPartName) .. " reason:" .. tostring(slotTreeEntry.decisionMethod)}
      -- end
      local partPath = newPath .. chosenPartName

      if slotOptions.coreSlot == true then
        slotOptions.coreSlot = nil
      end
      slotOptions.variables = nil

      slotTreeEntry.partPath = partPath
      slotTreeEntry.chosenPartName = chosenPartName
      slotPartMap[newPath] = chosenPartName
      activePartsData[chosenPartName] = deepcopy(chosenPart)
      activeParts[partPath] = chosenPartName

      chosenPart = deepcopy(chosenPart)

      -- If using the new dictionary format, find the child's node for recursion
      local nextUserPartNode
      if vehicleConfig.partsTree then
        local parentNode = userPartNode or vehicleConfig.partsTree
        nextUserPartNode = getChildNodeForSlotId(parentNode, slotId)
      end

      -- Recurse into the chosenPart's subslots
      -- Add to unifyJournal for later merging
      local uj = {currentPart, chosenPart, level, slotOptions, partPath, slotDef}
      table.insert(unifyJournalC, uj)

      local chosenPartSlots = chosenPart.slots2 or chosenPart.slots
      if chosenPartSlots then
        slotTreeEntry.children = {}

        -- recurse
        fillSlots_rec(
          ioCtx,
          slotMap,
          vehicleConfig,
          nextUserPartNode,
          chosenPart,
          level + 1,
          slotOptions,
          slotTreeEntry.children,
          slotPartMap,
          activePartsData,
          activeParts,
          newPath,
          unifyJournal,
          unifyJournalC
        )
      end

      table.insert(unifyJournal, uj)
    else
      slotTreeEntry.chosenPartName = ''
      slotPartMap[newPath] = ''
      if userPartName and userPartName ~= '' then
        log('E', "jbeam.fillSlots", 'slot "' .. tostring(slotId) .. '" left empty as part "' .. tostring(userPartName) .. '" was not found')
      else
        --log('D', "jbeam.fillSlots", "no suitable part found for type: " .. tostring(slot.type))
      end
    end
    ::continue::
  end
  profilerPopEvent(originalPath)
end

local function findParts(ioCtx, vehicleConfig)
  profilerPushEvent('jbeam/slotsystem.findParts')

  local chosenPartsTree = {}      -- new hierarchical structure
  local slotPartMap     = {}      -- key = slot path, value = part name
  local activePartsData = {}      -- key = part name, value = deep copy of fitted part
  local activeParts     = {}      -- key = part path, value = part name

  local rootPart = jbeamIO.getPart(ioCtx, vehicleConfig.mainPartName)
  if not rootPart then
    log('E', "jbeam.loadVehicle", "main slot not found, unable to spawn")
    profilerPopEvent('jbeam/slotsystem.findParts')
    return
  end

  local slotMap = jbeamIO.getAvailableSlotNameMap(ioCtx)
  if not slotMap then
    log('E', "jbeam.loadVehicle", "unable to get slot map, unable to spawn")
    profilerPopEvent('jbeam/slotsystem.findParts')
    return
  end

  local mainPartName = vehicleConfig.mainPartName
  local mainPartPath = vehicleConfig.mainPartPath

  -- For the hierarchy, we represent the main part as the top-level node
  local mainPartNode = {
    id                  = rootPart.slotType,
    path                = '/',
    suitablePartNames   = {mainPartName},
    unsuitablePartNames = {},
    chosenPartName      = mainPartName,
    partPath            = mainPartPath,
    children            = nil,
  }
  chosenPartsTree = mainPartNode

  activePartsData[mainPartName] = deepcopy(rootPart)
  activeParts[mainPartPath] = mainPartName

  local unifyJournal = {}
  local unifyJournalC = {}

  local slots = rootPart.slots2 or rootPart.slots
  if slots then
    mainPartNode.children = {}
    -- Recursively fill both structures
    fillSlots_rec(
      ioCtx,
      slotMap,
      vehicleConfig or {},
      nil,
      rootPart,
      1,
      nil,
      mainPartNode.children,
      slotPartMap,
      activePartsData,
      activeParts,
      '/',
      unifyJournal,
      unifyJournalC
    )
  end

  profilerPopEvent('jbeam/slotsystem.findParts')

  -- Return both flat and tree forms
  return rootPart, unifyJournal, unifyJournalC, chosenPartsTree, slotPartMap, activePartsData, activeParts
end

local function unifyPartJournal(ioCtx, unifyJournal)
  profilerPushEvent('jbeam/slotsystem.unifyParts')
  for i, j in ipairs(unifyJournal) do
    unifyParts(unpack(j))
  end
  profilerPopEvent('jbeam/slotsystem.unifyParts')
  return true
end

M.partFitsSlot = partFitsSlot
M.findParts = findParts
M.unifyPartJournal = unifyPartJournal

return M
