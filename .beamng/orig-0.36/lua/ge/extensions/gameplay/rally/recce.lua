-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- recce is used in the worldEditor to import a recce recording (drivelinev1, cuts and transcripts)
-- and create pacenotes from it.

local C = {}
local logTag = ''

local cc = require('/lua/ge/extensions/gameplay/rally/util/colors')
local rallyUtil = require('/lua/ge/extensions/gameplay/rally/util')
local normalizer = require('/lua/ge/extensions/gameplay/rally/util/normalizer')
local waypointTypes = require('/lua/ge/extensions/gameplay/rally/notebook/waypointTypes')
-- local SettingsManager = require('/lua/ge/extensions/gameplay/rally/settingsManager')
local RecceSettings = require('/lua/ge/extensions/gameplay/rally/recceSettings')
local Snaproad = require('/lua/ge/extensions/gameplay/rally/snaproad')
local Driveline = require('/lua/ge/extensions/gameplay/rally/driveline')

function C:init(missionDir)
  self.missionDir = missionDir
  self._loadedDrivelineAndCuts = false
  self.driveline = nil
  self.cuts = nil
  self:_resetState()
  self.settings = RecceSettings()
  self.settings:load()
end

function C:drivelineAndCutsLoaded()
  return self._loadedDrivelineAndCuts
end

function C:_resetState()
  self.driveline = nil
  self.cuts = nil
end

function C:loadDrivelineAndCuts()
  -- self:_resetState()
  -- self.settings:load()
  if not self:loadDriveline() then
    return false
  end
  if not self:loadCuts() then
    log('E', logTag, 'failed to load recce cuts')
    return false
  end
  self._loadedDrivelineAndCuts = true
  return true
end

function C:loadCuts()
  -- load the transcripts
  local fname = rallyUtil.transcriptsFile(self.missionDir)

  -- local import_language = rallyUtil.default_codriver_language
  -- local word_map = SettingsManager.loadMainSettingsWithLanguage(import_language):getFreeformSubstitutions()
  local word_map = {} -- TODO add back word_map once freeform work is going again.

  local transcripts = {}
  local tscCount = 0

  if FS:fileExists(fname) then
    for line in io.lines(fname) do
      local obj = jsonDecode(line)
      if obj.cutId > 0 then
        transcripts[obj.cutId] = obj
        tscCount = tscCount + 1
      -- else
        -- log('W', logTag, 'loadCuts: skipping transcript with cutId <= 0')
      end
    end
  end

  log('I', logTag, 'loaded '..tostring(tscCount)..' transcripts')

  -- load the cuts
  fname = rallyUtil.cutsFile(self.missionDir)
  if not FS:fileExists(fname) then
    self.cuts = {}
    -- its not a failure if there isnt a cuts file since it is optional.
    log('I', logTag, 'no cuts file found: '..fname)
    return true
  end

  self.cuts = {}

  for line in io.lines(fname) do
    local obj = jsonDecode(line)
    obj.pos = vec3(obj.pos)
    obj.quat = quat(obj.quat)
    local tsc = transcripts[obj.id]
    if tsc then
      local txt = tsc.resp.text
      if txt then
        -- print(txt)
        txt = normalizer.replaceWords(word_map, txt)
        -- print(txt)
      end

      obj.transcript = {
        error = tsc.resp.error,
        text = txt,
      }
    else
      obj.transcript = {}
    end

    table.insert(self.cuts, obj)
  end

  log('I', logTag, 'loaded '..tostring(#self.cuts)..' cuts')
  return true
end

function C:loadDriveline()
  self.driveline = Driveline(self.missionDir)
  if not self.driveline:load() then
    log('E', logTag, 'failed to load recce driveline')
    self.driveline = nil
    return false
  end
  return true
end

function C:drawDebugRecce(drawLabels, mouseInfo)
  if not self.driveline then return end
  self.driveline:drawDebugDriveline(drawLabels, mouseInfo)
  self:drawDebugCuts()
end

function C:drawDebugCuts()
  if not self.cuts then return end

  for _,point in ipairs(self.cuts) do
    local pos = point.pos
    local quat = point.quat
    local txt = point.transcript.text
    self:drawLittleCar(pos, quat, txt)
  end
end

function C:drawLittleCar(pos, quat, txt)
  local h = 1.6
  local w = 1.8
  local l = 4.4

  local forwardVector = vec3(0,1,0)
  local rotatedForwardVector = quat * forwardVector * (l/2) -- assume pos is the center of car so divide length by 2
  local frontOfCar = pos + rotatedForwardVector
  local backOfCar = pos - rotatedForwardVector

  local raise = vec3(0,0,h/2)
  frontOfCar = frontOfCar + raise
  backOfCar = backOfCar + raise

  local wheelPositions = {
    {0.5, vec3(-(w/2*0.9),   l/2 * 0.6,  0.4)}, -- Front left
    {0.5, vec3( (w/2*0.9),   l/2 * 0.6,  0.4)}, -- Front right
    {0.6, vec3(-(w/2*1.1), -(l/2 * 0.6), 0.4)}, -- Rear left
    {0.6, vec3( (w/2*1.1), -(l/2 * 0.6), 0.4)}, -- Rear right
  }

  -- Function to rotate and translate a local position to a world position
  local function toWorldPosition(localPos)
    local rotatedPos = quat * localPos  -- Rotate by car's orientation
    return pos + rotatedPos            -- Translate to car's world position
  end

  -- Draw the wheels
  for _, wheelPos in ipairs(wheelPositions) do
    local worldWheelPos = toWorldPosition(wheelPos[2])
    debugDrawer:drawSphere(worldWheelPos, wheelPos[1], ColorF(0,0,0,1))
  end

  local clr_base = cc.clr_teal
  local clr = clr_base
  local textAlpha = 1.0
  local clr_text_fg = cc.clr_black
  local clr_text_bg = cc.clr_teal

  debugDrawer:drawSquarePrism(
    frontOfCar,
    backOfCar,
    Point2F(h*0.7, w*0.7), -- make the car look more aero
    Point2F(h, w),
    ColorF(clr[1], clr[2], clr[3], 1)
  )

  if txt then
    debugDrawer:drawTextAdvanced(
      backOfCar + vec3(0,0,h/2),
      String(txt),
      ColorF(clr_text_fg[1], clr_text_fg[2], clr_text_fg[3], textAlpha),
      true,
      false,
      ColorI(clr_text_bg[1]*255, clr_text_bg[2]*255, clr_text_bg[3]*255, textAlpha*255)
    )
  end
end

function C:createPacenotesData(notebook)
  if not self:drivelineAndCutsLoaded() then return end
  -- if not self.cuts then return end

  log('I', logTag, 'import pacenotes to notebook')

  local importIdent = notebook:nextImportIdent()
  local import_language = rallyUtil.default_codriver_language

  local snaproad = Snaproad(self)

  local pacenotes = {}
  local prevPacenote = nil
  -- local prevAtPos = nil
  -- local prevCePos = nil
  local prevAtPoint = nil
  local prevCePoint = nil
  local foundDup = false

  -- local cutCount = #self.cuts

  for i,cut in ipairs(self.cuts) do
    local note = cut.transcript.text
    local pos = cut.pos
    local radius = editor_rallyEditor.getPrefDefaultRadius()

    -- set the pacenote name
    local pacenoteNewId = notebook:getNextUniqueIdentifier()
    local name = "Pacenote "..pacenoteNewId
    if importIdent then
      name = "Import_"..importIdent.." " .. pacenoteNewId
    end

    -- set some metadata
    local metadata = {}
    -- if transcript.beamng_file then
    -- metadata['success'] = transcript.success
    -- metadata['beamng_file'] = transcript.beamng_file
    -- end

    local firstSnapPoint = snaproad:firstSnapPoint()
    local pointCe = nil
    local pointCs = nil
    local pointAt = nil

    pointCe = snaproad:closestSnapPoint(pos)

    if pointCe.id == firstSnapPoint.id then
      pointCe = snaproad:pointsForwards(pointCe, 3)
      pointCs = snaproad:pointsBackwards(pointCe, 1)
      pointAt = snaproad:pointsBackwards(pointCs, 1)
    else
      local atLimits = {firstSnapPoint}
      if prevAtPoint then
        table.insert(atLimits, prevAtPoint)
      end
      pointAt = snaproad:distanceBackwards(pointCe, 4*radius, atLimits)

      local csLimits = {firstSnapPoint, pointAt}
      if prevCePoint then
        table.insert(csLimits, prevCePoint)
      end
      pointCs = snaproad:distanceBackwards(pointCe, 2*radius, csLimits)
    end

    -- after we've determined the points, see if there is a dup.
    if prevCePoint and pointCe.id == prevCePoint.id then
      foundDup = true
      if note and prevPacenote then
        local prevTxt = prevPacenote.notes[import_language].note

        local mergedTxt = nil

        if prevTxt then
          mergedTxt = prevTxt..' '..note
        else
          mergedTxt = note
        end

        prevPacenote.notes[import_language].note = mergedTxt
      end
    end

    local posCe = pointCe.pos
    local posCs = pointCs.pos
    local posAt = pointAt.pos

    local normalAt = snaproad:forwardNormalVec(pointAt)

    local pn = {
      name = name,
      notes = { [import_language] = {note = note}},
      metadata = metadata,
      oldId = pacenoteNewId,
      pacenoteWaypoints = {
        {
          name = "audio trigger",
          normal = normalAt,
          oldId = notebook:getNextUniqueIdentifier(),
          pos = posAt,
          radius = radius,
          waypointType = waypointTypes.wpTypeFwdAudioTrigger,
        },
        {
          name = "corner start",
          normal = {0.0, 1.0, 0.0},
          oldId = notebook:getNextUniqueIdentifier(),
          pos = posCs,
          radius = radius,
          waypointType = waypointTypes.wpTypeCornerStart,
        },
        {
          name = "corner end",
          normal = {0.0, 1.0, 0.0},
          oldId = notebook:getNextUniqueIdentifier(),
          pos = posCe,
          radius = radius,
          waypointType = waypointTypes.wpTypeCornerEnd,
        }
      }
    }

    -- prevAtPos = posAt
    -- prevCePos = posCe

    if not foundDup then
      prevPacenote = pn
      prevAtPoint = pointAt
      prevCePoint = pointCe
      table.insert(pacenotes, pn)
    end

    foundDup = false
  end

  return pacenotes
end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end

