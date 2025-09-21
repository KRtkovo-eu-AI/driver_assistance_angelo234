-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt
local M = {}

M.qualityLevels = {
  Lowest = {
    caseSensitive = 1,
  },
  Low = {
    caseSensitive = 1,
  },
  High = {
    caseSensitive = 1,
  },
  Ultra = {
    caseSensitive = 1,
  }
}

M.qualityLevels.Lowest["$pref::lightManager"] = "Basic Lighting"
M.qualityLevels.Lowest["$pref::Shadows::filterMode"] = "None"

M.qualityLevels.Low["$pref::lightManager"] = "Advanced Lighting"
M.qualityLevels.Low["$pref::Shadows::filterMode"] = "SoftShadow"

M.qualityLevels.High["$pref::lightManager"] = "Advanced Lighting"
M.qualityLevels.High["$pref::Shadows::filterMode"] = "SoftShadowHighQuality"

M.qualityLevels.Ultra["$pref::lightManager"] = "Advanced Lighting 1.5"
M.qualityLevels.Ultra["$pref::Shadows::filterMode"] = "SoftShadowHighQuality"


M.onApply = function()
  setLightManager(TorqueScriptLua.getVar("$pref::lightManager"))
end

return M