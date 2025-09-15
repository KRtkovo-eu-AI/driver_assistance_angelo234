-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

local triggers = {}
local triggerList = {}

local deformMeshes = {}
local brokenSwitches = {}
local lastValues = {}
local changedMats = {}
local matState = {}

-- custom materials lua environment
local mv = {}

local function switchMaterial(msc, matname)
  if matname == nil then
    if matState[msc] ~= false then
      matState[msc] = false
      obj:resetMaterials(msc)
    end
  else
    if matState[msc] ~= matname then
      matState[msc] = matname
      obj:switchMaterial(msc, matname)
    end
  end
end

local function switchBrokenMaterial(beam)
  for msc, g in pairs(beam.deformSwitches) do
    --log('D', "material.switchBrokenMaterial", "mesh broke: "..g.mesh.. " with deformGroup " .. g.deformGroup)
    props.disablePropsInDeformGroup(g.deformGroup)
    local dm = deformMeshes[g.deformGroup]
    if dm then --if there is a mesh assigned to this deformGroup
      if dm.deformSound and dm.deformSound ~= "" and not brokenSwitches[msc] then --check if the mesh has a deform sound
        --sounds.playSoundOnceAtNode(dm.deformSound, beam.id1, dm.deformVolume or 1)   --play the deform sound
        sounds.playSoundOnceFollowNode(dm.deformSound, beam.id1, (dm.deformVolume or 1) * 0.5)
        --print ((dm.deformVolume or 1) * 0.5)
        beamstate.addDamage(500)
      end
    end
    switchMaterial(msc, g.dmgMat)
    brokenSwitches[msc] = true
  end
end

local function updateGFX()
  -- check for changes
  local eVals = electrics.values
  local varChanged = false
  for _, f in ipairs(triggerList) do
    local v = eVals[f]
    if v ~= nil and v ~= lastValues[f] then
      lastValues[f] = v
      if type(v) == "boolean" then
        v = v and 1 or 0
      end
      mv[f] = v
      varChanged = true
    end
  end

  if not varChanged then
    return
  end

  -- change materials
  -- log('E', "material.funcChanged", "funcChanged("..f..","..val)
  table.clear(changedMats)
  for _, va in ipairs(triggers) do
    if brokenSwitches[va.msc] == nil then
      local localVal = va.evalFunction()
      if localVal == nil then
        brokenSwitches[va.msc] = true
        return
      end
      local newMat = nil
      --we need to know what the expected max value for the glowmap is as we need to use that to normalize the mesh emissivity
      local emissiveMax
      if localVal > 0.0001 then
        newMat = va.on
        emissiveMax = va.scaleEmissiveMaterialOnMax --use the jbeam-defined max for "on"
        if va.on_intense ~= nil then -- we have sth with 2 glow layers
          if localVal > 0.5 then
            newMat = va.on_intense
            emissiveMax = va.scaleEmissiveMaterialOnIntenseMax --use the jbeam-defined max for "on_intense"
          end
        end
      end
      --only scale the emissivity if we know what the max is supposed to be
      if va.scaleEmissiveMaterial and emissiveMax then
        --scale our input value with the expected max
        local emissiveScaleValue = linearScale(localVal, 0, emissiveMax, 0, 255)
        obj:setMaterialEmissiveFactor(va.msc, color(emissiveScaleValue, emissiveScaleValue, emissiveScaleValue))
      end
      -- log('W', "material.funcChanged", "switchMaterial(" .. tostring(va.msc) .. ", '" .. tostring(newMat).."')")
      if newMat == nil then
        if matState[va.msc] ~= false and changedMats[va.msc] == nil then
          changedMats[va.msc] = false
        end
      else
        changedMats[va.msc] = newMat
      end
    end
  end

  for msc, newMat in pairs(changedMats) do
    if newMat ~= matState[msc] then
      matState[msc] = newMat
      if newMat then
        obj:switchMaterial(msc, newMat)
      else
        obj:resetMaterials(msc)
      end
    end
  end
end

local function forceReset()
  table.clear(changedMats)
  table.clear(lastValues)
  brokenSwitches = {}
  for mid, _ in pairs(matState) do --do not change
    matState[mid] = false
    obj:resetMaterials(mid)
  end
end

local function reset()
  for mid, _ in pairs(matState) do --do not change
    switchMaterial(mid)
  end
  brokenSwitches = {}
  table.clear(lastValues)
end

local function init()
  if not v.data._materials then
    return
  end
  brokenSwitches = {}

  mv = v.data._materials.mv
  triggerList = v.data._materials.triggerList
  triggers = v.data._materials.triggers
  deformMeshes = v.data._materials.deformMeshes or {}

  local funTab = {"return function () ", nil, " end"}
  for _, t in pairs(triggers) do
    local str = t.evalFunctionString or ""
    funTab[2] = str
    local f, err = load(table.concat(funTab), str, "t", mv)
    if f then
      t.evalFunction = f()
    else
      log("E", "material.init", tostring(err))
      t.evalFunction = nop
    end
    if t.materialEmissiveScaling then
      t.scaleEmissiveMaterial = true
      t.scaleEmissiveMaterialOnMax = t.materialEmissiveScaling.on_max
      t.scaleEmissiveMaterialOnIntenseMax = t.materialEmissiveScaling.on_intense_max
      t.materialEmissiveScaling = nil
    end
  end
end

-- public interface
M.init = init
M.reset = reset
M.switchBrokenMaterial = switchBrokenMaterial
M.updateGFX = updateGFX

M.forceReset = forceReset
return M
