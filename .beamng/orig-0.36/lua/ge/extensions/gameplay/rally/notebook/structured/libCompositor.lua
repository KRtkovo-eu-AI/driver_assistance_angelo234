-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local rallyUtil = require('/lua/ge/extensions/gameplay/rally/util')
local Schema = require('/lua/ge/extensions/gameplay/rally/notebook/structured/schema')

local M = {}

local function getNearestValue(valuesList, targetValue)
  local closest = nil
  local minDiff = math.huge

  for _,entry in ipairs(valuesList) do
    local value = tonumber(entry.value)
    local diff = math.abs(value - targetValue)
    if diff < minDiff then
      minDiff = diff
      closest = entry
    end
  end

  return closest
end

local function getCornerCall(config, cornerSeverity, cornerDirection, cornerSquare)
  -- log('D', '', 'cornerSeverity=' .. tostring(cornerSeverity) .. ' cornerDirection=' .. tostring(cornerDirection) .. ' cornerSquare=' .. tostring(cornerSquare))
  if cornerDirection == 0 then
    return nil, nil
  end

  local dirStr = config.cornerDirection[cornerDirection]

  if cornerSquare then
    return config.cornerSquare, dirStr
  end

  if cornerSeverity == -1 then
    return nil, nil
  end

  local targetNum = tonumber(cornerSeverity)
  if not targetNum or targetNum < 0 then
    log('E', 'Invalid corner severity: ' .. tostring(cornerSeverity))
    return nil, nil
  end

  local closest = getNearestValue(config.cornerSeverity, targetNum)

  return closest.name, dirStr
end

local function getCornerLength(config, length)
  if not length then return nil end
  local closest = getNearestValue(config.cornerLength, length)
  return closest.name
end

local function getCornerRadiusChange(config, change)
  if not change then return nil end
  local closest = getNearestValue(config.cornerRadiusChange, change)
  return closest.name
end

local function getCaution(config, caution)
  if not caution then return nil end
  return config.caution[caution]
end

local function getModifiers(config, fields, limit)
  limit = limit or 3
  local modifiersOut = {}

  local prioritizedModifiers = {}
  for mod,modData in pairs(config.modifiers) do
    table.insert(prioritizedModifiers, { modName = mod, modData = modData })
  end
  table.sort(prioritizedModifiers, function(a, b)
    return a.modData.priority < b.modData.priority
  end)

  local count = 0
  for i, mod in ipairs(prioritizedModifiers) do
    local modValue = fields[mod.modName]
    if modValue and count < limit then
      table.insert(modifiersOut, mod.modData)
      count = count + 1
    end
  end

  return modifiersOut
end

local function getFinishLine(config, finishLine)
  if not finishLine then return nil end
  return config.finishLine.text
end

local function collectVariables(config, compositorState, structured, distanceBefore, distanceAfter)
  local vars = {}

  local fields = structured.fields

  local cornerSeverity, cornerDirection = getCornerCall(config, fields.cornerSeverity, fields.cornerDirection, fields.cornerSquare)
  vars.cornerSeverity = cornerSeverity
  vars.cornerDirection = cornerDirection

  local cornerLength = getCornerLength(config, fields.cornerLength)
  vars.cornerLength = cornerLength

  local cornerRadiusChange = getCornerRadiusChange(config, fields.cornerRadiusChange)
  vars.cornerRadiusChange = cornerRadiusChange

  local caution = getCaution(config, fields.caution)
  if caution ~= '' then
    vars.caution = caution
  end

  local modifiers = getModifiers(config, fields)
  vars.modifier1 = modifiers[1]
  vars.modifier2 = modifiers[2]
  vars.modifier3 = modifiers[3]

  local finishLine = getFinishLine(config, fields.finishLine)
  vars.finishLine = finishLine

  if distanceBefore and rallyUtil.useNote(distanceBefore) then
    if compositorState.escapeVars then
      vars.distanceBefore = rallyUtil.var_db
    else
      vars.distanceBefore = distanceBefore
    end
  end

  if distanceAfter and rallyUtil.useNote(distanceAfter) then
    if compositorState.escapeVars then
      vars.distanceAfter = rallyUtil.var_da
    else
      vars.distanceAfter = distanceAfter
    end
  end

  return vars
end

local function readVar(vars, name)
  if vars[name] then
    return vars[name]
  end
  return nil
end

local function join(a, b, separator)
  separator = separator or " "

  if a and b then
    return a..separator..b
  end

  if a then
    return a
  end

  if b then
    return b
  end

  return ""
end

local function makeCautionPhrase(structured, vars, compositorState)
  local distanceBefore = readVar(vars, 'distanceBefore')
  local caution = readVar(vars, 'caution')

  if distanceBefore and caution then
    compositorState.distanceBeforeUsed = true
    return distanceBefore..' '..caution, compositorState
  end

  if caution then
    return caution, compositorState
  end

  return nil, compositorState
end

local function makeCornerPhrase(config, structured, vars, compositorState)
  local distanceBefore = readVar(vars, 'distanceBefore')
  local cornerSeverity = readVar(vars, 'cornerSeverity')
  local cornerDirection = readVar(vars, 'cornerDirection')
  local cornerLength = readVar(vars, 'cornerLength')
  local cornerRadiusChange = readVar(vars, 'cornerRadiusChange')

  if not cornerSeverity or not cornerDirection then
    return nil, compositorState
  end

  local phrase = ""

  if distanceBefore and not compositorState.distanceBeforeUsed then
    compositorState.distanceBeforeUsed = true
    phrase = join(phrase, distanceBefore, '')
  end

  phrase = join(phrase, cornerSeverity)
  phrase = join(phrase, cornerDirection)

  if cornerLength then
    phrase = join(phrase, cornerLength)
  end

  if cornerRadiusChange then
    phrase = join(phrase, cornerRadiusChange, config.punctuation.intraSubphrase..' ')
  end

  return phrase, compositorState
end

local function makeModifierPhrase(varName, vars, compositorState)
  local distanceBefore = readVar(vars, 'distanceBefore')
  local modifier = readVar(vars, varName)

  if not modifier then return nil, compositorState end

  local phrase = ""
  if distanceBefore and not compositorState.distanceBeforeUsed then
    compositorState.distanceBeforeUsed = true
    phrase = join(phrase, distanceBefore)
    phrase = join(phrase, modifier.textWhenFirst or modifier.text)
  else
    phrase = join(phrase, modifier.text)
  end

  return phrase, compositorState
end

local function makeModifierPhrases(structured, vars, compositorState)
  local distanceBefore = readVar(vars, 'distanceBefore')
  local distanceAfter = readVar(vars, 'distanceAfter')
  local phrases = {}

  local phrase, compositorState = makeModifierPhrase('modifier1', vars, compositorState)
  if phrase then
    table.insert(phrases, phrase)
  end

  local phrase, compositorState = makeModifierPhrase('modifier2', vars, compositorState)
  if phrase then
    table.insert(phrases, phrase)
  end

  local phrase, compositorState = makeModifierPhrase('modifier3', vars, compositorState)
  if phrase then
    table.insert(phrases, phrase)
  end

  return phrases, compositorState
end

local function makeFinishLinePhrase(structured, vars, compositorState)
  local finishLine = readVar(vars, 'finishLine')
  if not finishLine then return nil, compositorState end
  return finishLine, compositorState
end

local function addPunctuation(phrase, punctuation)
  if not punctuation then return phrase end
  return phrase..punctuation
end

local function postProcessPhrase(phrase, punctuation)
  phrase = rallyUtil.trimString(phrase)
  phrase = addPunctuation(phrase, punctuation)
  phrase = rallyUtil.trimString(phrase)
  return phrase
end
M.postProcessPhrase = postProcessPhrase

M.composite = function(config, structured, rawDistanceBefore, rawDistanceAfter, escapeVars)
  escapeVars = escapeVars or false
  local compositorState = { escapeVars = escapeVars }

  local vars = collectVariables(config, compositorState, structured, rawDistanceBefore, rawDistanceAfter)

  local distanceBefore = vars.distanceBefore
  local distanceAfter = vars.distanceAfter

  local cautionPhrase, compositorState = makeCautionPhrase(structured, vars, compositorState)
  local cornerPhrase, compositorState = makeCornerPhrase(config, structured, vars, compositorState)
  local modifierPhrases, compositorState = makeModifierPhrases(structured, vars, compositorState)
  local finishLinePhrase, compositorState = makeFinishLinePhrase(structured, vars, compositorState)

  local result = {}
  local phrase = nil

  if cautionPhrase then
    phrase = postProcessPhrase(cautionPhrase, config.punctuation.phraseEnd)
    table.insert(result, phrase)
  end

  if cornerPhrase then
    phrase = postProcessPhrase(cornerPhrase, config.punctuation.phraseEnd)
    table.insert(result, phrase)
  end

  for _,modPhrase in ipairs(modifierPhrases) do
    phrase = postProcessPhrase(modPhrase, config.punctuation.phraseEnd)
    table.insert(result, phrase)
  end

  if finishLinePhrase then
    phrase = postProcessPhrase(finishLinePhrase, '')
    table.insert(result, phrase)
  end

  if distanceAfter then
    phrase = postProcessPhrase(distanceAfter, config.punctuation.distanceCall)
    table.insert(result, phrase)
  end

  return result
end

M.enumerate = function(textCompositor, config)
  local enumeratedPhrases = {}
  local fields = {}
  Schema.initDefaultFields(fields)

  local structured = {fields = fields}

  -- ensure we are under the threshold for each level by subtracting 1.
  local rawDistanceBeforeValues = {
    textCompositor:getDistanceCallShorthand(config.transitions.level1.threshold - 1),
    textCompositor:getDistanceCallShorthand(config.transitions.level2.threshold - 1),
    textCompositor:getDistanceCallShorthand(config.transitions.level3.threshold - 1),
    -- then ensure we are past the final threshold by adding some meters.
    textCompositor:getDistanceCallShorthand(config.transitions.level3.threshold + 10),
  }

  -- ensure we are past the final threshold by adding some meters.
  local rawDistanceAfter = textCompositor:getDistanceCallShorthand(config.transitions.level3.threshold + 10)

  -- Caution phrases
  for _,rawDistanceBefore in ipairs(rawDistanceBeforeValues) do
    for caution,cautionText in pairs(config.caution) do
      structured.fields.caution = caution
      local compositorState = {}
      local vars = collectVariables(config, compositorState, structured, rawDistanceBefore, rawDistanceAfter)

      local cautionPhrase, compositorState = makeCautionPhrase(structured, vars, compositorState)
      if cautionPhrase then
        cautionPhrase = postProcessPhrase(cautionPhrase, config.punctuation.phraseEnd)
        table.insert(enumeratedPhrases, cautionPhrase)
      elseif caution ~= 0 then
        log('E', '', 'No caution phrase found for ' .. caution)
      end
    end
  end

  -- Corner phrases
  for _,rawDistanceBefore in ipairs(rawDistanceBeforeValues) do
    for _,cornerSeverity in ipairs(config.cornerSeverity) do
      for cornerDirection,_ in pairs(config.cornerDirection) do
        for _,cornerLength in pairs(config.cornerLength) do
          for _,cornerRadiusChange in pairs(config.cornerRadiusChange) do
            structured.fields.cornerSeverity = cornerSeverity.value
            structured.fields.cornerDirection = cornerDirection
            structured.fields.cornerLength = cornerLength.value
            structured.fields.cornerRadiusChange = cornerRadiusChange.value

            local compositorState = {}
            local vars = collectVariables(config, compositorState, structured, rawDistanceBefore, rawDistanceAfter)

            local cornerPhrase, compositorState = makeCornerPhrase(config, structured, vars, compositorState)
            if cornerPhrase then
              cornerPhrase = postProcessPhrase(cornerPhrase, config.punctuation.phraseEnd)
              table.insert(enumeratedPhrases, cornerPhrase)
            elseif cornerDirection ~= 0 then
              log('E', '', 'No corner phrase found for severity=' .. cornerSeverity.value .. ' direction=' .. cornerDirection .. ' length=' .. cornerLength.value .. ' radiusChange=' .. cornerRadiusChange.value)
            end
          end
        end
      end
    end
  end

  -- corner phrases with square
  for _,rawDistanceBefore in ipairs(rawDistanceBeforeValues) do
    -- for _,cornerSquare in ipairs({true, false}) do
      for cornerDirection,_ in pairs(config.cornerDirection) do
        for _,cornerLength in pairs(config.cornerLength) do
          for _,cornerRadiusChange in pairs(config.cornerRadiusChange) do
            structured.fields.cornerSeverity = -1
            structured.fields.cornerDirection = cornerDirection
            structured.fields.cornerLength = cornerLength.value
            structured.fields.cornerRadiusChange = cornerRadiusChange.value
            -- structured.fields.cornerSquare = cornerSquare
            structured.fields.cornerSquare = true

            local compositorState = {}
            local vars = collectVariables(config, compositorState, structured, rawDistanceBefore, rawDistanceAfter)

            local cornerPhrase, compositorState = makeCornerPhrase(config, structured, vars, compositorState)
            if cornerPhrase then
              cornerPhrase = postProcessPhrase(cornerPhrase, config.punctuation.phraseEnd)
              table.insert(enumeratedPhrases, cornerPhrase)
            elseif cornerDirection ~= 0 then
              log('E', '', 'No corner phrase found for square=' .. tostring(cornerSquare) .. ' ' .. cornerDirection .. ' ' .. cornerLength.value .. ' ' .. cornerRadiusChange.value)
            end
          end
        end
      end
    -- end
  end

  -- modifier phrases
  for _,rawDistanceBefore in ipairs(rawDistanceBeforeValues) do
    for modName,modData in pairs(config.modifiers) do

      -- clear modifiers to work with a clean and consistent state.
      for modName,_ in pairs(config.modifiers) do
        structured.fields[modName] = false
      end

      structured.fields[modName] = true

      local compositorState = {}
      local vars = collectVariables(config, compositorState, structured, rawDistanceBefore, rawDistanceAfter)

      local modifierPhrases, compositorState = makeModifierPhrases(structured, vars, compositorState)
      local modifierPhrase = modifierPhrases[1]

      if modifierPhrase then
        modifierPhrase = postProcessPhrase(modifierPhrase, config.punctuation.phraseEnd)
        table.insert(enumeratedPhrases, modifierPhrase)
      else
        log('E', '', 'No modifier phrase found for ' .. modName)
      end
    end
  end

  -- finish line
  -- for _,phrase in ipairs(config.finishLine.variants) do
    -- table.insert(enumeratedPhrases, phrase.text)
  -- end
  table.insert(enumeratedPhrases, config.finishLine.text)

  return enumeratedPhrases
end

-- M.enumerateSystemPacenotes = function(textCompositor, config)
--   local enumeratedPhrases = {}

--   -- local sp = SystemPacenotes.getCopy()

--   for _,sp in ipairs(sp) do
--     table.insert(enumeratedPhrases, sp.text)
--   end

--   return enumeratedPhrases
-- end

M.getRandomWeightedItem = function(items)
  -- First pass: sum all weights
  local totalWeight = 0
  for _, item in ipairs(items) do
    totalWeight = totalWeight + (item.weight or 1)
  end

  -- Generate a random value between 0 and 1
  local randomValue = math.random()

  -- Second pass: find the item
  local cumulativeWeight = 0
  for _, item in ipairs(items) do
    -- Calculate normalized weight
    local normalizedWeight = (item.weight or 1) / totalWeight
    cumulativeWeight = cumulativeWeight + normalizedWeight

    if randomValue <= cumulativeWeight then
      return item
    end
  end

  -- Fallback for floating point edge cases
  return items[#items]
end



return M