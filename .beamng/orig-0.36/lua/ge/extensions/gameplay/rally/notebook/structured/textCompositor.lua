-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local rallyUtil = require('/lua/ge/extensions/gameplay/rally/util')
local LibCompositor = require('/lua/ge/extensions/gameplay/rally/notebook/structured/libCompositor')

local logTag = ''

local C = {}

function C:init(compositorName)
  if not compositorName then
    log('E', logTag, 'compositorName is nil')
  end
  self.compositorName = compositorName
  self.compositor = nil
  self.cachedSystemPacenotes = {}
end

function C:load()
  if FS:fileExists(self:compositorPath()..'.lua') then
    self.compositor = self:_requireCompositor()
    return true
  else
    log('E', logTag, 'compositor not found: '..self.compositorName)
    return false
  end
end

function C:compositorPath()
  return '/lua/ge/extensions/gameplay/rally/compositors/'..self.compositorName..'/compositor'
end

function C:_requireCompositor()
  if not self.compositorName then return nil end
  local fname = self:compositorPath()
  return require(fname)
end

function C:compositeText(structured, distBefore, distAfter)
  if not self.compositor then return nil end
  local result = LibCompositor.composite(self:getConfig(), structured, distBefore, distAfter)
  return result
end

function C:compositeTextEscaped(structured, distBefore, distAfter)
  if not self.compositor then return nil end
  local result = LibCompositor.composite(self:getConfig(), structured, distBefore, distAfter, true)
  return result
end

function C:getConfig()
  if not self.compositor then return nil end
  return self.compositor.config
end

function C:getBreathConfig()
  if not self.compositor then return nil end
  return self.compositor.breathConfig
end

function C:getSystemPacenotes(missionPacenotesDirname)
  if not self.compositor then return nil end

  -- set it to a <none> value to use as a cache key
  missionPacenotesDirname = missionPacenotesDirname or '<none>'

  if self.cachedSystemPacenotes[missionPacenotesDirname] then
    return self.cachedSystemPacenotes[missionPacenotesDirname]
  end

  local systemPacenotes = deepcopy(self.compositor.config.system)
  local compositorVoice = settings.getValue("rallyTextCompositorVoice")

  local sysNotes = {}

  for name,variants in pairs(systemPacenotes) do
    sysNotes[name] = {}
    for _,variant in ipairs(variants) do
      local pacenoteHash = rallyUtil.pacenoteHashSha1(variant.text)
      local pacenoteFname = nil
      if missionPacenotesDirname ~= '<none>' then
        pacenoteFname = missionPacenotesDirname..'/'..rallyUtil.makePacenoteAudioFilename(pacenoteHash)
      else
        pacenoteFname = rallyUtil.getCompositorPacenoteFile(compositorVoice, pacenoteHash)
      end
      variant.audioFname = pacenoteFname

      local file_exists = FS:fileExists(pacenoteFname)
      if not file_exists then
        log('D', logTag, "getSystemPacenotes: couldnt find file for static pacenote with name '"..name.."'")
      end
      table.insert(sysNotes[name], variant)
    end
  end

  self.cachedSystemPacenotes[missionPacenotesDirname] = sysNotes
  return self.cachedSystemPacenotes[missionPacenotesDirname]
end

function C:getSystemPacenote(name, i)
  if not self.compositor then return nil end
  i = i or 1 -- default to the first variant
  local sys = self:getSystemPacenotes()
  return sys[name][i]
end

function C:enumerateDistances()
  local min = self:getDistanceCallLevel3Threshold()
  local max = 5000
  local step = self:getRoundingSmall()

  -- print(string.format("Enumerating from %d to %d with step %d", self.min, self.max, self.step))
  local out = {}
  for n = min, max, step do
    local distStr = self:distanceToString(n)
    distStr = self:postProcessPhrase(distStr, self:getPunctuationDistanceCalls())
    out[distStr] = n
  end

  local compacted = {}
  for distStr,n in pairs(out) do
    compacted[n] = distStr
  end

  local keys = {}
  for k in pairs(compacted) do table.insert(keys, k) end
  table.sort(keys)

  local sorted = {}
  for _, k in ipairs(keys) do
    table.insert(sorted, compacted[k])
  end

  return sorted
end

function C:enumeratePacenotes()
  return LibCompositor.enumerate(self, self:getConfig())
end

function C:enumerateAll()
  local distances = self:enumerateDistances()
  local pacenotes = self:enumeratePacenotes()

  local combined = {}
  local totalChars = 0

  for _, dist in ipairs(distances) do
    table.insert(combined, { phrase = dist, hash = rallyUtil.pacenoteHashSha1(dist) })
    totalChars = totalChars + #dist
  end

  for _, note in ipairs(pacenotes) do
    table.insert(combined, { phrase = note, hash = rallyUtil.pacenoteHashSha1(note) })
    totalChars = totalChars + #note
  end

  local systemCount = 0
  for name,variants in pairs(self:getSystemPacenotes()) do
    systemCount = systemCount + #variants
    for _,variant in ipairs(variants) do
      table.insert(combined, { phrase = variant.text, hash = rallyUtil.pacenoteHashSha1(variant.text) })
      totalChars = totalChars + #variant.text
    end
  end

  -- Check for duplicate phrases and remove them
  local seen = {}
  for _, item in ipairs(combined) do
    if not seen[item.phrase] then
      seen[item.phrase] = true
    else
      log('E', logTag, 'Duplicate phrase found: "'..item.phrase..'"')
      error('Duplicate phrase found: "'..item.phrase..'"')
    end
  end

  local out = {
    stats = {
      totalChars = totalChars,
      totalPhrases = #combined,
      distancePhrases = #distances,
      notePhrases = #pacenotes,
      systemPhrases = systemCount,
    },
    phrases = combined,
  }

  log('I', logTag, string.format('Enumerated %d phrases (%d chars) for compositor "%s"',
    out.stats.totalPhrases, out.stats.totalChars, self.compositorName))
  log('I', logTag, string.format('Details: %d distance calls, %d pacenotes, %d system pacenotes',
    out.stats.distancePhrases, out.stats.notePhrases, out.stats.systemPhrases))

  return out
end

function C:writeEnumerated(fname, enumerated)
  jsonWriteFile(fname, enumerated, true)
end

function C:roundDistance(dist)
  if dist >= self:getRoundingLargeThreshold() then
    local roundedVal = rallyUtil.customRound(dist, self:getRoundingLarge()) / self:getRoundingLargeThreshold()
    return roundedVal, self:getLargeUnit()
  elseif dist >= self:getRoundingMediumThreshold() then
    local val = rallyUtil.customRound(dist, self:getRoundingMedium())
    if val == self:getRoundingLargeThreshold() then
      -- if the rounded value is the same as the large threshold, we need to round up to the next large unit
      val = rallyUtil.customRound(dist, self:getRoundingLarge()) / self:getRoundingLargeThreshold()
      return val, self:getLargeUnit()
    end
    return val, self:getBaseUnit()
  else
    return rallyUtil.customRound(dist, self:getRoundingSmall()), self:getBaseUnit()
  end
end

local function replacePeriodWithPoint(inputString, pointTranslation)
  -- Check if the input string contains a "."
  if string.find(inputString, "%.") then
    -- Split on period and handle each part
    local firstPart, secondPart = inputString:match("(%d+)%.(%d+)")
    if firstPart and secondPart then
      -- Split each digit in second part individually
      local digits = {}
      for digit in secondPart:gmatch("%d") do
        table.insert(digits, digit)
      end
      -- Reassemble with point translation and spaces between all digits
      inputString = firstPart .. " " .. pointTranslation .. " " .. table.concat(digits, " ")
    end
  end
  return inputString
end

function C:distanceToString(dist, forceSkipSeparateDigits)
  local separateDigits = self:getSeparateDigits()
  if forceSkipSeparateDigits then
    separateDigits = false
  end

  dist = math.floor(dist)
  local roundedDist, unit = self:roundDistance(dist)
  local distStr = tostring(roundedDist)

  if unit == self:getLargeUnit() then
    distStr = replacePeriodWithPoint(distStr, self:getPointTranslation())
    distStr = distStr .. " " .. unit
  elseif separateDigits and roundedDist >= self:getRoundingMediumThreshold() and roundedDist % self:getRoundingMediumThreshold() ~= 0 then
    -- separate digits if not a multiple of rounding threshold
    distStr = distStr:sub(1, 1) .. " " .. distStr:sub(2)
  end

  return distStr
end

function C:postProcessPhrase(phrase, punctuation)
  return LibCompositor.postProcessPhrase(phrase, punctuation)
end

function C:getDistanceCallShorthand(dist)
  if dist <= self:getDistanceCallLevel1Threshold() then
    return self:getDistanceCallLevel1Text()
  elseif dist <= self:getDistanceCallLevel2Threshold() then
    return self:getDistanceCallLevel2Text()
  elseif dist <= self:getDistanceCallLevel3Threshold() then
    return self:getDistanceCallLevel3Text()
  else
    return nil
  end
end

function C:getSeparateDigits()
  return self.compositor.config.transitions.separateDigits
end

function C:getPunctuationLastNote()
  return self.compositor.config.punctuation.lastNote
end

function C:getPunctuationDefault()
  return self.compositor.config.punctuation.phraseEnd
end

function C:getPunctuationDistanceCalls()
  return self.compositor.config.punctuation.distanceCall
end

function C:getDistanceCallLevel1Threshold()
  return self.compositor.config.transitions.level1.threshold
end
function C:getDistanceCallLevel1Text()
  return self.compositor.config.transitions.level1.text
end

function C:getDistanceCallLevel2Threshold()
  return self.compositor.config.transitions.level2.threshold
end
function C:getDistanceCallLevel2Text()
  return self.compositor.config.transitions.level2.text
end

function C:getDistanceCallLevel3Threshold()
  return self.compositor.config.transitions.level3.threshold
end
function C:getDistanceCallLevel3Text()
  return self.compositor.config.transitions.level3.text
end

function C:getBaseUnit()
  return self.compositor.config.units.baseUnitTranslation
end
function C:getLargeUnit()
  return self.compositor.config.units.largeUnitTranslation
end
function C:getPointTranslation()
  return self.compositor.config.units.pointTranslation
end

function C:getRoundingSmall()
  return self.compositor.config.distanceRounding.small
end
function C:getRoundingMedium()
  return self.compositor.config.distanceRounding.medium
end
function C:getRoundingMediumThreshold()
  return self.compositor.config.distanceRounding.mediumThreshold
end

function C:getRoundingLarge()
  return self.compositor.config.distanceRounding.large
end
function C:getRoundingLargeThreshold()
  return self.compositor.config.distanceRounding.largeThreshold
end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end