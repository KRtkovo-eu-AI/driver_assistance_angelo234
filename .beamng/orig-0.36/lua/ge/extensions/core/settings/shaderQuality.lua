-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

M.qualityLevels = {
  Low = {
    caseSensitive = 1,
  },
  High = {
    caseSensitive = 1,
  },
}

M.qualityLevels.Low["$pref::Video::disablePixSpecular"] = 0
M.qualityLevels.Low["$pref::Video::disableNormalmapping"] = 0
M.qualityLevels.Low["$pref::Video::disableParallaxMapping"] = 1
M.qualityLevels.Low["$pref::Water::disableTrueReflections"] = 1
M.qualityLevels.Low["$pref::Video::ShaderQualityGroup"] = "Low"

M.qualityLevels.High["$pref::Video::disablePixSpecular"] = 0
M.qualityLevels.High["$pref::Video::disableNormalmapping"] = 0
M.qualityLevels.High["$pref::Video::disableParallaxMapping"] = 0
M.qualityLevels.High["$pref::Water::disableTrueReflections"] = 0
M.qualityLevels.High["$pref::Video::ShaderQualityGroup"] = "High"

return M