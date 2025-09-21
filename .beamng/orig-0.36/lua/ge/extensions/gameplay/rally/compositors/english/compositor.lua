-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

--------------------------------WARNING----------------------------------
--------------------------------WARNING----------------------------------
--------------------------------WARNING----------------------------------
--                                                                     --
-- If you change any strings in this file that are used for pacenotes, --
-- you must regenerate the audio files.                                --
--                                                                     --
--------------------------------WARNING----------------------------------
--------------------------------WARNING----------------------------------
--------------------------------WARNING----------------------------------

local M = {}

M.breathConfig = {
  default = {0.05, 0.075},
  lastSubphrase = {0.10, 0.20}
}


--
-- Colors
--
-- These are passed to the visual pacenotes app.
--
-- Examples:
-- "rbg(100, 100, 100)"
-- "#ff33dd"
-- "var(--bng-add-red-650)"
--

-- general
local clrBngVarOffBlack = "var(--bng-off-black)"
local clrBngVarOffWhite = "var(--bng-off-white)"
local clrIconAndTextDefault = clrBngVarOffWhite

-- corners
local clrBngAddRed550 = "var(--bng-add-red-550)"
local clrBngAddRed650 = "var(--bng-add-red-650)"
local clrBngAddRed750 = "var(--bng-add-red-750)"

local clrBngOrange = "var(--bng-orange)"
local clrBngOrange500 = "var(--bng-orange-500)"

local clrBngTerYellow400 = "var(--bng-ter-yellow-400)"
local clrBngTerYellow500 = "var(--bng-ter-yellow-500)"

local clrBngTerPeach300 = "var(--bng-ter-peach-300)"
local clrBngTerPeach400 = "var(--bng-ter-peach-400)"

local clrBngAddBlue500 = "var(--bng-add-blue-500)"
local clrBngAddBlue600 = "var(--bng-add-blue-600)"

local clrBngAddIndigoblue650 = "var(--bng-add-indigoblue-650)"
local clrBngAddIndigoblue750 = "var(--bng-add-indigoblue-750)"

local clrBngAddGreen500 = "var(--bng-add-green-500)"
local clrBngAddGreen600 = "var(--bng-add-green-600)"

-- modifiers
local clrBngAddMagenta550 = "var(--bng-add-magenta-550)"
local clrBngAddMagenta650 = "var(--bng-add-magenta-650)"

-- modfiers
local clrModifierBg = clrBngAddMagenta550
local clrModifierStroke = clrBngAddMagenta650


M.config = {
  punctuation = {
    phraseEnd = "?",
    lastNote = ".",
    distanceCall = ".",
    intraSubphrase = ","
  },
  transitions = {
    separateDigits = true,
    level1 = {
      threshold = 5,
      text = "<none>"
    },
    level2 = {
      threshold = 15,
      text = "into"
    },
    level3 = {
      threshold = 25,
      text = "and"
    }
  },
  units = {
    system = "metric",
    baseUnitTranslation = "m",
    largeUnitTranslation = "km",
    pointTranslation = "point"
  },
  distanceRounding = {
    small = 10,
    medium = 50,
    mediumThreshold = 100,
    large = 250,
    largeThreshold = 1000
  },
  visualGeneral = {
    intoColor = clrIconAndTextDefault,
    distanceColor = clrBngVarOffWhite
  },

  -- these numbers were made by evenly dividing the range 0-100 into equal parts.
  cornerSeverity = {
    { name = "tight hairpin", value = 100, visual = { icon = "turnHp", text = "HP", colorBg = clrBngAddRed550,    colorStroke = clrBngAddRed650,    colorNoteIcon = clrIconAndTextDefault, colorNoteText = clrIconAndTextDefault }},
    { name = "hairpin",       value = 89,  visual = { icon = "turnHp", text = "HP", colorBg = clrBngAddRed550,    colorStroke = clrBngAddRed650,    colorNoteIcon = clrIconAndTextDefault, colorNoteText = clrIconAndTextDefault }},
    { name = "open hairpin",  value = 78,  visual = { icon = "turnHp", text = "HP", colorBg = clrBngAddRed550,    colorStroke = clrBngAddRed650,    colorNoteIcon = clrIconAndTextDefault, colorNoteText = clrIconAndTextDefault }},
    { name = "one",           value = 68,  visual = { icon = "turn1",  text = "1",  colorBg = clrBngAddRed550,    colorStroke = clrBngAddRed650,    colorNoteIcon = clrIconAndTextDefault, colorNoteText = clrIconAndTextDefault }},
    { name = "two",           value = 57,  visual = { icon = "turn2",  text = "2",  colorBg = clrBngOrange,       colorStroke = clrBngOrange500,    colorNoteIcon = clrIconAndTextDefault, colorNoteText = clrIconAndTextDefault }},
    { name = "three",         value = 47,  visual = { icon = "turn3",  text = "3",  colorBg = clrBngTerYellow400, colorStroke = clrBngTerYellow500, colorNoteIcon = clrIconAndTextDefault, colorNoteText = clrIconAndTextDefault }},
    { name = "four",          value = 36,  visual = { icon = "turn4",  text = "4",  colorBg = clrBngTerPeach300,  colorStroke = clrBngTerPeach400,  colorNoteIcon = clrIconAndTextDefault, colorNoteText = clrIconAndTextDefault }},
    { name = "five",          value = 26,  visual = { icon = "turn5",  text = "5",  colorBg = clrBngAddBlue500,   colorStroke = clrBngAddBlue600,   colorNoteIcon = clrIconAndTextDefault, colorNoteText = clrIconAndTextDefault }},
    { name = "six",           value = 15,  visual = { icon = "turn6",  text = "6",  colorBg = clrBngAddGreen500,  colorStroke = clrBngAddGreen600,  colorNoteIcon = clrIconAndTextDefault, colorNoteText = clrIconAndTextDefault }},
    { name = "flat",          value = 5,   visual = { icon = "turn6",  text = "FL", colorBg = clrBngAddGreen500,  colorStroke = clrBngAddGreen600,  colorNoteIcon = clrIconAndTextDefault, colorNoteText = clrIconAndTextDefault }}
  },
  cornerDirection = {
    [-1] = "left",
    [0] = "straight",
    [1] = "right"
  },
  cornerSquare = "square",
  cornerSquareVisual = { icon = "turnSq", text = "SQ", colorBg = clrBngOrange, colorStroke = clrBngOrange500, colorNoteIcon = clrIconAndTextDefault, colorNoteText = clrIconAndTextDefault },
  cornerLength = {
    { name = "short", value = 40 },
    { name = nil, value = 50 },
    { name = "long", value = 60 },
    { name = "extra long", value = 70 }
  },
  cornerRadiusChange = {
    { name = "opens", value = 40, visual = { icon = "mathLessThan", colorIcon = clrIconAndTextDefault } },
    { name = nil, value = 50 },
    { name = "tightens", value = 60, visual = { icon = "mathGreaterThan", colorIcon = clrIconAndTextDefault } },
  },

  caution = {
    [0] = '',
    [1] = "caution",
    [2] = "double caution",
    [3] = "triple caution"
  },
  cautionVisual = { colorBg = clrBngAddRed550, colorStroke = clrBngAddRed650, colorNoteIcon = clrIconAndTextDefault },

  -- priority defines the order of modifiers in the output slots of modifier1, modifier2, modifier3.
  modifiers = {
    modDontCut = {
      priority = 1,
      text = "don't cut",
      visual = { icon = "scissorsSlashed", colorIcon = clrIconAndTextDefault }
    },
    modNarrows = {
      priority = 2,
      text = "narrows",
      visual = { icon = "narrows", colorIcon = clrIconAndTextDefault, colorBg = clrModifierBg, colorStroke = clrModifierStroke }
    },
    modWater = {
      priority = 3,
      text = "watersplash",
      visual = { icon = "water", colorIcon = clrIconAndTextDefault, colorBg = clrModifierBg, colorStroke = clrModifierStroke }
    },
    modJump = {
      priority = 4,
      text = "over jump",
      textWhenFirst = "jump",
      visual = { icon = "jumpOverBump", colorIcon = clrIconAndTextDefault, colorBg = clrModifierBg, colorStroke = clrModifierStroke }
    },
    modCrest = {
      priority = 5,
      text = "over crest",
      textWhenFirst = "crest",
      visual = { icon = "crest", colorIcon = clrIconAndTextDefault, colorBg = clrModifierBg, colorStroke = clrModifierStroke }
    },
    modBumpy = {
      priority = 6,
      text = "bumpy",
      visual = { icon = "bumps", colorIcon = clrIconAndTextDefault, colorBg = clrModifierBg, colorStroke = clrModifierStroke }
    },
    modBump = {
      priority = 7,
      text = "over bump",
      textWhenFirst = "bump",
      visual = { icon = "bump", colorIcon = clrIconAndTextDefault, colorBg = clrModifierBg, colorStroke = clrModifierStroke }
    }
  },
  finishLine = {
    text = "over finish",
    visual = { icon = "finish", colorIcon = clrBngVarOffWhite, colorBg = clrBngVarOffBlack, colorStroke = clrBngVarOffWhite }
    -- variants = {
      -- { text = "over finish", weight = 0.33 },
      -- { text = "flying finish", weight = 0.33 },
      -- { text = "finish", weight = 0.33 },
    -- }
  },
  -- order variants so that the first one is the default.
  system = {
    damage = {
      { text = "We just took some damage!", chill = false },
    },
    countdowngo = {
      { text = "go.", chill = true },
    },
    countdown1 = {
      { text = "one", chill = true },
    },
    countdown2 = {
      { text = "two", chill = true },
    },
    countdown3 = {
      { text = "three", chill = true },
    },
    countdown4 = {
      { text = "four", chill = true },
    },
    countdown5 = {
      { text = "five", chill = true },
    },
    firstnoteintro = {
      { text = "The first note is:", chill = true, weight = 0.6 },
      { text = "Here's the first note:", chill = true, weight = 0.3 },
    },
    firstnoteoutro = {
      { text = "Good luck!", chill = true, weight = 0.5 },
      { text = "Best of luck!", chill = true, weight = 0.1, fun = true },
      -- { text = "Let's GOOOO!!!", chill = true, weight = 0.1, fun = true },
      -- { text = "Hold on a second, your helmet isn't buckled...", chill = true, weight = 0.1, fun = true },
      -- { text = "The fire extinguisher is empty!", chill = true, weight = 0.1, fun = true },
      -- { text = "Did you check the tires?", chill = true, weight = 0.1, fun = true },
      -- { text = "I forgot to use the toilet...", chill = true, weight = 0.01, fun = true },
    },
    finish = {
      { text = "Good job! Stop at Stop Control...", chill = true, weight = 0.7 },
      { text = "Very nice! Now to Stop Control...", chill = true, weight = 0.3, fun = true },
      -- { text = "Did I leave the stove on?", chill = true, weight = 0.1, fun = true },
      -- { text = "I knew we would finish this time.", chill = true, weight = 0.1, fun = true },
      -- { text = "Can I drive next time?", chill = true, weight = 0.1, fun = true },
      -- { text = "I can't stop yelling! This is how I talk!", chill = true, weight = 0.09, fun = true },
    }
  },
  -- substitutions = {}
}

return M