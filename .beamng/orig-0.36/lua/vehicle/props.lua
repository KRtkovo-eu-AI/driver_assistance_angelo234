-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}
local props = {}
local propEnd = -1
local breakGroupMap = {}
local deformGroupMap = {}
local min, max = math.min, math.max

local function updateProp(val, prop)
  if not prop.pid then
    return
  end
  --convert any possible bools to 0/1
  val = type(val) ~= "boolean" and val or (val and 1 or 0)
  local pt = prop.translation
  local pr = prop.rotation
  obj:propUpdate(prop.pid, pt.x, pt.y, pt.z, pr.x, pr.y, pr.z, not prop.hidden, val, min(max(val * prop.multiplier, prop.min), prop.max) + prop.offset)
  if prop.scaleLight and prop.lightBrightness and prop.flareScale and prop.lightColor then
    local lightBrightness = linearScale(val, prop.scaleLightBrightnessMinInput, prop.scaleLightBrightnessMaxInput, 0, prop.lightBrightness)
    local flareScale = linearScale(val, prop.scaleLightFlareScaleMinInput, prop.scaleLightFlareScaleMaxInput, 0, prop.flareScale)
    local lightColor = color(linearScale(val, 1, 0, prop.lightColor.r, prop.lightColor.r - prop.scaleLightColorOffsetRed), linearScale(val, 1, 0, prop.lightColor.g, prop.lightColor.g - prop.scaleLightColorOffsetGreen), linearScale(val, 1, 0, prop.lightColor.b, prop.lightColor.b - prop.scaleLightColorOffsetBlue), prop.lightColor.a)
    obj:setPropLight(prop.pid, lightBrightness, flareScale, lightColor)
  end
end

local function updateGFX()
  local evals = electrics.values
  for i = 0, propEnd do
    local prop = props[i]
    if not prop.disabled then
      updateProp(evals[prop.func] or 0, prop)
    end
  end
end

local function disablePropsInDeformGroup(deformGroup)
  if deformGroupMap[deformGroup] then
    for _, prop in ipairs(deformGroupMap[deformGroup]) do
      if not prop.disabled then
        prop.disabled = true
        prop.hidden = true
        updateProp(0, prop)
      end
    end
    deformGroupMap[deformGroup] = nil
  end
end

local function hidePropsInBreakGroup(breakGroup)
  if breakGroupMap[breakGroup] then
    for _, prop in ipairs(breakGroupMap[breakGroup]) do
      if not (prop.hidden and prop.disabled) then
        -- log('D', "props.hidePropsInBreakGroup", "prop hidden: ".. tostring(breakGroup))
        prop.disabled = true
        prop.hidden = true
        updateProp(0, prop)
      end
    end
    breakGroupMap[breakGroup] = nil
  end
end

local function reset()
  props = v.data.props
  if not props or props[0] == nil then
    propEnd = -1
    return
  end

  breakGroupMap = {}
  deformGroupMap = {}

  propEnd = tableSizeC(props) - 1
  if propEnd < 0 then
    return
  end
  M.updateGFX = updateGFX

  for i = 0, propEnd do
    local prop = props[i]

    prop.disabled = false
    prop.hidden = false

    if prop.breakGroup ~= nil then
      local breakGroups = type(prop.breakGroup) == "table" and prop.breakGroup or {prop.breakGroup}
      for _, g in pairs(breakGroups) do
        if type(g) == "string" and g ~= "" then
          if breakGroupMap[g] == nil then
            breakGroupMap[g] = {}
          end
          table.insert(breakGroupMap[g], prop)
        end
      end
    end

    if prop.deformGroup ~= nil then
      local deformGroups = type(prop.deformGroup) == "table" and prop.deformGroup or {prop.deformGroup}
      for _, g in pairs(deformGroups) do
        if type(g) == "string" and g ~= "" then
          if deformGroupMap[g] == nil then
            deformGroupMap[g] = {}
          end
          table.insert(deformGroupMap[g], prop)
        end
      end
    end

    if prop.lightScaling then
      if type(prop.lightScaling) ~= "table" then
        prop.lightScaling = {
          brightnessMinInput = 0,
          brightnessMaxInput = 1,
          flareScaleMinInput = 0.6,
          flareScaleMaxInput = 1,
          lightColorOffsetRed = 0,
          lightColorOffsetGreen = 60,
          lightColorOffsetBlue = 80
        }
      end
      prop.scaleLight = true
      prop.scaleLightBrightnessMinInput = prop.lightScaling.brightnessMinInput or 0
      prop.scaleLightBrightnessMaxInput = prop.lightScaling.brightnessMaxInput or 1
      prop.scaleLightFlareScaleMinInput = prop.lightScaling.flareScaleMinInput or 0.6
      prop.scaleLightFlareScaleMaxInput = prop.lightScaling.flareScaleMaxInput or 1
      prop.scaleLightColorOffsetRed = prop.lightScaling.lightColorOffsetRed or 0
      prop.scaleLightColorOffsetGreen = prop.lightScaling.lightColorOffsetGreen or 60
      prop.scaleLightColorOffsetBlue = prop.lightScaling.lightColorOffsetBlue or 80

      prop.lightScaling = nil
    end
  end
end

-- public interface
M.init = reset
M.reset = reset

M.updateGFX = nop

M.disablePropsInDeformGroup = disablePropsInDeformGroup
M.hidePropsInBreakGroup = hidePropsInBreakGroup

return M
