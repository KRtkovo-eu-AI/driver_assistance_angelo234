-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
M.type = "auxiliary"

local eyeBeamData
local eyeLimiterData
local eyeAttachmentData
local defaultGravity
local gravityMaxSizeCoef
local sizeCoefSmoother
local maxSizeCoef
local minSizeCoef
local minSizeBoundCoef
local maxSizeBoundCoef
local attachmentMinSizeCoef
local attachmentMaxSizeCoef

local function updateGFX(dt)
  local sizeCoef = linearScale(sensors.gz, 0, defaultGravity * gravityMaxSizeCoef, maxSizeCoef, minSizeCoef)
  --local sizeCoef = linearScale(electrics.values.throttle - electrics.values.brake, -1, 1, maxSizeCoef, minSizeCoef)

  if electrics.values.ignitionLevel > 2 then
    sizeCoef = maxSizeCoef
  end
  sizeCoef = sizeCoefSmoother:get(sizeCoef, dt)
  --print(sizeCoef)

  for cid, data in pairs(eyeBeamData) do
    obj:setBeamLength(cid, data.defaultLength * sizeCoef)
  end

  local boundCoef = linearScale(sizeCoef, maxSizeCoef, minSizeCoef, maxSizeBoundCoef, minSizeBoundCoef)
  for cid, data in pairs(eyeLimiterData) do
    obj:setBoundedBeamLongBound(cid, data.defaultLongBound * boundCoef)
  end

  local attachmentCoef = linearScale(sizeCoef, minSizeCoef, maxSizeCoef, attachmentMinSizeCoef, attachmentMaxSizeCoef)
  for cid, data in pairs(eyeAttachmentData) do
    obj:setBeamLength(cid, data.defaultLength * attachmentCoef)
  end
end

local function reset(jbeamData)
  sizeCoefSmoother:set(1)
end

local function init(jbeamData)
  local eyeBeamTag = jbeamData.eyeBeamTag or "eyeBeam"
  local eyeBeamIds = beamstate.tagBeamMap[eyeBeamTag] or {}
  eyeBeamData = {}
  for _, cid in ipairs(eyeBeamIds) do
    eyeBeamData[cid] = {defaultLength = obj:getBeamRestLength(cid)}
  end

  local eyeLimiterBeamTag = jbeamData.eyeLimiterBeamTag or "eyeLimiterBeam"
  local eyeLimiterBeamIds = beamstate.tagBeamMap[eyeLimiterBeamTag] or {}
  eyeLimiterData = {}
  for _, cid in ipairs(eyeLimiterBeamIds) do
    eyeLimiterData[cid] = {defaultLongBound = v.data.beams[cid].beamLongBound}
  end

  local eyeAttachmentBeamTag = jbeamData.eyeAttachmentBeamTag or "eyeAttachmentBeam"
  local eyeAttachmentBeamIds = beamstate.tagBeamMap[eyeAttachmentBeamTag] or {}
  eyeAttachmentData = {}
  for _, cid in ipairs(eyeAttachmentBeamIds) do
    eyeAttachmentData[cid] = {defaultLength = obj:getBeamRestLength(cid)}
  end

  maxSizeCoef = jbeamData.maxSizeCoef or 1
  minSizeCoef = jbeamData.minSizeCoef or 1
  minSizeBoundCoef = jbeamData.minSizeBoundCoef or 1
  maxSizeBoundCoef = jbeamData.maxSizeBoundCoef or 1
  attachmentMinSizeCoef = jbeamData.attachmentMinSizeCoef or 1
  attachmentMaxSizeCoef = jbeamData.attachmentMaxSizeCoef or 1

  local sizeSmoothingIn = jbeamData.sizeSmoothingShrink or 5
  local sizeSmoothingOut = jbeamData.sizeSmoothingGrow or 5

  sizeCoefSmoother = newTemporalSmoothingNonLinear(sizeSmoothingIn, sizeSmoothingOut)

  gravityMaxSizeCoef = jbeamData.gravityMaxSizeCoef or 1
  defaultGravity = powertrain.currentGravity
  sizeCoefSmoother:set(1)
end

M.init = init
M.reset = reset
M.updateGFX = updateGFX

return M
