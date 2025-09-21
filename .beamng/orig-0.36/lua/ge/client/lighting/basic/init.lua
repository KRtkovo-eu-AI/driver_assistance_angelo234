-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

require("lua/ge/client/lighting/basic/shadowFilter")

local bl_ProjectedShadowSBData = scenetree.findObject("BL_ProjectedShadowSBData")
if not bl_ProjectedShadowSBData then
  bl_ProjectedShadowSBData = createObject("GFXStateBlockData")
  bl_ProjectedShadowSBData.blendDefined = true
  bl_ProjectedShadowSBData.blendEnable = true
  bl_ProjectedShadowSBData.blendEnable = true
  bl_ProjectedShadowSBData:setField("blendSrc", 0, "GFXBlendDestColor")
  bl_ProjectedShadowSBData:setField("blendDest", 0, "GFXBlendZero")
  bl_ProjectedShadowSBData.zDefined = true
  bl_ProjectedShadowSBData.zEnable = true
  bl_ProjectedShadowSBData.zWriteEnable = false
  if TorqueScriptLua.getBoolVar("$Scene::useReversedDepthBuffer") then
      bl_ProjectedShadowSBData:setField("zBias", 0, 1)
      bl_ProjectedShadowSBData:setField("zSlopeBias", 0, 1)
  else
      bl_ProjectedShadowSBData:setField("zBias", 0, -5)
      bl_ProjectedShadowSBData:setField("zSlopeBias", 0, -5)
  end
  bl_ProjectedShadowSBData.samplersDefined = true
  bl_ProjectedShadowSBData:setField("samplerStates", 0, "SamplerClampLinear")
  bl_ProjectedShadowSBData.vertexColorEnable = true
  bl_ProjectedShadowSBData:registerObject("BL_ProjectedShadowSBData")
end

local bl_ProjectedShadowShaderData = scenetree.findObject("BL_ProjectedShadowShaderData")
if not bl_ProjectedShadowShaderData then
  bl_ProjectedShadowShaderData = createObject("ShaderData")
  bl_ProjectedShadowShaderData.DXVertexShaderFile = "shaders/common/projectedShadowV.hlsl"
  bl_ProjectedShadowShaderData.DXPixelShaderFile  = "shaders/common/projectedShadowP.hlsl"
  bl_ProjectedShadowShaderData.pixVersion = 5.0;
  bl_ProjectedShadowShaderData:registerObject("BL_ProjectedShadowShaderData")
end

local bl_ProjectedShadowMaterial = scenetree.findObject("BL_ProjectedShadowMaterial")
if not bl_ProjectedShadowMaterial then
  bl_ProjectedShadowMaterial = createObject("CustomMaterial")
  bl_ProjectedShadowMaterial:setField("sampler", "inputTex", "$miscbuff")
  bl_ProjectedShadowMaterial:setField("shader", 0, "BL_ProjectedShadowShaderData")
  bl_ProjectedShadowMaterial:setField("stateBlock", 0, "BL_ProjectedShadowSBData")
  bl_ProjectedShadowMaterial.version = 5.0
  bl_ProjectedShadowMaterial.forwardLit = true
  bl_ProjectedShadowMaterial:registerObject("BL_ProjectedShadowMaterial")
end

local basicLighting = LightManager.findByName("Basic Lighting")
if basicLighting then
  local basicLightingCallbacks = {}
  basicLightingCallbacks.onActivate = function()
    -- log('I','BLM','Basic Lighting onActivate called...')
    local al_formatToken = scenetree.findObject("AL_FormatToken")
    if al_formatToken then
      al_formatToken:enable()
    end

    -- Create render pass for projected shadow.
    local renderPassManager = createObject("RenderPassManager")
    renderPassManager:registerObject("BL_ProjectedShadowRPM")

    -- Create the mesh bin and add it to the manager.
    local meshBin = createObject("RenderFastMgr")
    renderPassManager:addManager(meshBin)

    -- Add both to the root group so that it doesn't
    -- end up in the MissionCleanup instant group.
    local rootGroup = scenetree.findObject("RootGroup")
    rootGroup:add(renderPassManager)
    rootGroup:add(meshBin)
  end

  basicLightingCallbacks.onDeactivate = function()
    -- Delete the pass manager which also deletes the bin.
    -- log('I','BLM','Basic Lighting onDeactivate called...')
    local al_formatToken = scenetree.findObject("AL_FormatToken")
    if al_formatToken then
      al_formatToken:disable()
    end
    local renderPassManager = scenetree.findObject("BL_ProjectedShadowRPM")
    renderPassManager:delete()
  end
  rawset(_G, "BLMCallbacks", basicLightingCallbacks)
end
-- function setBasicLighting()
-- {
--     setLightManager( "Basic Lighting" );
-- }
