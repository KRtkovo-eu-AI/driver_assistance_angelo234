-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local rallyUtil = require('ge/extensions/gameplay/rally/util')

local C = {}

local logTag = ''

function C:init(textCompositor)
  self.textCompositor = textCompositor
end

function C:_getMaxTransitionDistance()
  local config = self.textCompositor:getConfig()
  return config.transitions.level3.threshold
end


-- isLeft: false, done
--
-- type: 'turn1', done
-- type: 'turn2', done
-- type: 'turn3', done
-- type: 'turn4', done
-- type: 'turn5', done
-- type: 'turn6', done
-- type: 'turnHp', done
-- type: 'turnSq', done
--
-- type: 'rocks',
--
-- modifiers:
-- type: 'scissorsSlashed',
-- type: 'circleSlashed',
-- type: 'mathLessThan',
-- type: 'mathGreaterThan',
--
-- type: 'caution', done
-- type: 'doubleCaution', done
--
-- type: 'bridge',
-- type: 'bump',
-- type: 'bumps',
-- type: 'crest',
-- type: 'finish',
-- type: 'jumpOverBump',
-- type: 'narrows',
-- type: 'pothole',
-- type: 'water',


-- needed:
-- tripleCaution


-- {
--   type: "empty",  //  Note icon from the iconfont, string, by ID from icons.js / import { BngIcon, icons } from "@/common/components/base"
--   typeExt: null, // Note icon for custom SVGs, should be url()
--   turnModifier: null, // "Informational" modifiers: opens, narrows, over crest, water splash. Color is not changeable, placed on vertical center. Accepts icon ID from the iconfont
--   background: {
--     color: 'var(--bng-cool-gray-600)', // color override for the background, should be RGB or CSS var with color, no alpha!
--     strokeColor: 'var(--bng-cool-gray-500)', // color override for stroke, same as above, should be RGB, no alpha!
--     opacity: 0.6 // opacity for the BG SVG, that's why you should not use alpha above
--   },
--   isInto: false, // "into" background style. Boolean, apply to the note that is chained to previous one
--   isLeft: false, // Boolean, scales the icon on X axis to -1
--   size: 5, // note container size in REMs, with some additional magic to break the Angular's defaults.
--   turnTypeValue: null, // String, usually 1-6, but you can drop SQ, FL, or whatever else there.
--   distance: null, // string, in meters, drawn below the note
--   additionalNote: { // That's for the important notes, top right corner, colorable. Intended for "Cut" / "Don't cut", "attention", "danger"
--     color: '#fff', // color override, could be css var or color as rgb
--     icon: null, // icon from the iconfont, by ID
--     text: null // that's an alternative, if you need to show text for some reason (NC / C)
--   }
-- }
function C:_getCornerData(noteAttrs)
  local direction = noteAttrs.cornerDirection
  local severity = noteAttrs.cornerSeverity
  local radiusChange = noteAttrs.cornerRadiusChange
  local cornerLength = noteAttrs.cornerLength
  local isSquare = noteAttrs.cornerSquare
  local dontCut = noteAttrs.modDontCut

  local configSeverities = self.textCompositor:getConfig().cornerSeverity
  local configRadiusChanges = self.textCompositor:getConfig().cornerRadiusChange
  local configModifiers = self.textCompositor:getConfig().modifiers

  --
  -- Severity
  --
  local turnType = nil
  local isLeft = false
  if direction == nil or severity == nil then return nil end

  -- Convert direction to left/right string
  local dirStr = nil
  if direction == 0 then
    return nil
  elseif direction == 1 then
    isLeft = false
  elseif direction == -1 then
    isLeft = true
  end

  if isSquare then
    turnType = self.textCompositor:getConfig().cornerSquareVisual
  else
    local targetNum = tonumber(severity)
    if not targetNum or targetNum < 0 then return nil end

    -- find the closest severity in the config
    local minDiff = math.huge
    for _, sevData in ipairs(configSeverities) do
      local sevValue = tonumber(sevData.value)
      local diff = math.abs(sevValue - targetNum)
      if diff < minDiff then
        minDiff = diff
        if sevData.visual then
          turnType = sevData.visual
        else
          turnType = nil
        end
      end
    end
  end

  --
  -- Turn modifier - only radius change at the moment
  --
  local turnModifier = nil
  if radiusChange then
    local targetNum = tonumber(radiusChange)
    -- if not targetNum or targetNum < 0 then return nil end

    -- find the closest value in the config
    local minDiff = math.huge
    for _, radData in ipairs(configRadiusChanges) do
      local radVal = tonumber(radData.value)
      local diff = math.abs(radVal - targetNum)
      if diff < minDiff then
        minDiff = diff
        if radData.visual and radData.visual.icon then
          turnModifier = radData.visual.icon
        else
          turnModifier = nil
        end
      end
    end
  end

  -- TODO: add cornerLength once there are icons

  local additionalNote = nil
  if dontCut then
    additionalNote = {
      color = configModifiers.modDontCut.visual.colorIcon,
      icon = configModifiers.modDontCut.visual.icon,
      text = nil
    }
  end

  return {
    turnType = turnType,
    isLeft = isLeft,
    turnModifier = turnModifier,
    additionalNote = additionalNote
  }
end

local typicalSize = 2
local typicalOpacity = 0.6
local maxNotes = 4

function C:compositeVisual(pacenote, noteAttrs, distBeforeMeters, distAfterMeters)
  local visualPacenotes = {}

  local isInto = distBeforeMeters > 0 and distBeforeMeters < self:_getMaxTransitionDistance()
  local intoColor = nil
  if isInto then
    intoColor = self.textCompositor:getConfig().visualGeneral.intoColor
  end

  local dist = nil
  local distColor = nil
  if distAfterMeters > 0 then
    dist = self.textCompositor:distanceToString(distAfterMeters, true)
    distColor = self.textCompositor:getConfig().visualGeneral.distanceColor
  end

  local intoAccountedFor = false

  if noteAttrs.caution and noteAttrs.caution > 0 then
    local cautionVisual = self.textCompositor:getConfig().cautionVisual
    local icon = "caution"
    if noteAttrs.caution >= 2 then
      icon = "doubleCaution"
    end
    local cautionPacenote = {
      type = icon,
      colorNoteIcon = cautionVisual.colorNoteIcon,
      background = {
        color = cautionVisual.colorBg,
        strokeColor = cautionVisual.colorStroke,
        opacity = typicalOpacity
      },
      size = typicalSize,
    }
    table.insert(visualPacenotes, cautionPacenote)
  end

  if noteAttrs.cornerDirection ~= 0 then
    local cornerData = self:_getCornerData(noteAttrs)
    if cornerData then
      local cornerPacenote = {
        type = cornerData.turnType.icon,     -- Note icon from the iconfont, string, by ID from icons.js / import { BngIcon, icons } from "@/common/components/base"
        typeExt = nil,      -- Note icon for custom SVGs, should be url()
        turnModifier = cornerData.turnModifier, -- "Informational" modifiers: opens, narrows, over crest, water splash. Color is not changeable, placed on vertical center. Accepts icon ID from the iconfont
        colorNoteIcon = cornerData.turnType.colorNoteIcon,
        colorNoteText = cornerData.turnType.colorNoteText,
        background = {
          color = cornerData.turnType.colorBg,       -- color override for the background, should be RGB or CSS var with color, no alpha!
          strokeColor = cornerData.turnType.colorStroke, -- color override for stroke, same as above, should be RGB, no alpha!
          opacity = typicalOpacity                             -- opacity for the BG SVG, that's why you should not use alpha above
        },
        isLeft = cornerData.isLeft,  -- Boolean, scales the icon on X axis to -1
        size = typicalSize,   -- note container size in REMs, with some additional magic to break the Angular's defaults.
        turnTypeValue = cornerData.turnType.text, -- String, usually 1-6, but you can drop SQ, FL, or whatever else there.
        additionalNote = cornerData.additionalNote,
      }
      table.insert(visualPacenotes, cornerPacenote)
    end
  end

  local configModifiers = self.textCompositor:getConfig().modifiers

  local modFields = {
    "modNarrows",
    "modWater",
    "modJump",
    "modCrest",
    "modBumpy",
    "modBump",
  }

  for _, modField in ipairs(modFields) do
    local mod = noteAttrs[modField]
    if mod then
      local configField = configModifiers[modField]
      local modPacenote = {
        type = configField.visual.icon,
        colorNoteIcon = configField.visual.colorIcon,
        background = {
          color = configField.visual.colorBg,
          strokeColor = configField.visual.colorStroke,
          opacity = typicalOpacity
        },
        size = typicalSize
      }
      if #visualPacenotes < maxNotes then
        table.insert(visualPacenotes, modPacenote)
      end
    end
  end


  -- local water = noteAttrs.modWater
  -- local jump = noteAttrs.modJump
  -- local crest = noteAttrs.modCrest
  -- local bumpy = noteAttrs.modBumpy
  -- local bump = noteAttrs.modBump

  local configFinish = self.textCompositor:getConfig().finishLine
  local finish = noteAttrs.finishLine
  if finish then
    local finishPacenote = {
      type = configFinish.visual.icon,
      colorNoteIcon = configFinish.visual.colorIcon,
      background = {
        color = configFinish.visual.colorBg,
        strokeColor = configFinish.visual.colorStroke,
        opacity = typicalOpacity
      },
      size = typicalSize
    }
      -- always add finish
      table.insert(visualPacenotes, finishPacenote)
    -- end
  end


  for i,vp in ipairs(visualPacenotes) do
    vp.id = string.format("%s_%d", string.gsub(pacenote.name, ' ', '_'), i)
    vp.pnId = pacenote.id
    if i == 1 then
      vp.isInto = isInto
      vp.intoColor = intoColor
    end
    if i == #visualPacenotes then
      vp.distance = dist
      vp.colorDistance = distColor
    end
  end

  -- dump(visualPacenotes)

  return visualPacenotes
end

return function(...)
  local o = {}
  setmetatable(o, C)
  C.__index = C
  o:init(...)
  return o
end