-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- log('I','postfx', 'Smaa.lua loaded.....')

local smaaStateBlock = scenetree.findObject("SMAA_StateBlock")
if not smaaStateBlock then
  local pfxDefaultStateBlock = scenetree.findObject("PFX_DefaultStateBlock")
  smaaStateBlock = createObject("GFXStateBlockData")
  smaaStateBlock:inheritParentFields(pfxDefaultStateBlock)
  smaaStateBlock.samplersDefined = true
  smaaStateBlock:setField("samplerStates", 0, "SamplerClampLinear")
  smaaStateBlock:setField("samplerStates", 1, "SamplerClampLinear")
  smaaStateBlock:setField("samplerStates", 2, "SamplerClampLinear")
  smaaStateBlock:registerObject("SMAA_StateBlock")
end

local smaaEdgeDetectionShaderData = scenetree.findObject("SMAA_EdgeDetectionShaderData")
if not smaaEdgeDetectionShaderData then
  smaaEdgeDetectionShaderData = createObject("ShaderData")
  smaaEdgeDetectionShaderData.DXVertexShaderFile    = "shaders/common/postFx/smaa/smaa_edgeDetectionV.hlsl"
  smaaEdgeDetectionShaderData.DXPixelShaderFile     = "shaders/common/postFx/smaa/smaa_edgeDetectionP.hlsl"
  smaaEdgeDetectionShaderData.pixVersion = 5.0
  smaaEdgeDetectionShaderData:setField("samplerNames", 0, "$colorTexGamma")
  smaaEdgeDetectionShaderData:registerObject("SMAA_EdgeDetectionShaderData")
end

local smaaBlendingWeightShaderData = scenetree.findObject("SMAA_BlendingWeightShaderData")
if not smaaBlendingWeightShaderData then
  smaaBlendingWeightShaderData = createObject("ShaderData")
  smaaBlendingWeightShaderData.DXVertexShaderFile    = "shaders/common/postFx/smaa/smaa_blendingWeightV.hlsl"
  smaaBlendingWeightShaderData.DXPixelShaderFile     = "shaders/common/postFx/smaa/smaa_blendingWeightP.hlsl"
  smaaBlendingWeightShaderData.pixVersion = 5.0
  smaaBlendingWeightShaderData:setField("samplerNames", 0, "$edgesTex")
  smaaBlendingWeightShaderData:setField("samplerNames", 1, "$areaTex")
  smaaBlendingWeightShaderData:setField("samplerNames", 2, "$searchTex")
  smaaBlendingWeightShaderData:registerObject("SMAA_BlendingWeightShaderData")
end

local smaaNeighborhoodBlendingShaderData = scenetree.findObject("SMAA_NeighborhoodBlendingShaderData")
if not smaaNeighborhoodBlendingShaderData then
  smaaNeighborhoodBlendingShaderData = createObject("ShaderData")
  smaaNeighborhoodBlendingShaderData.DXVertexShaderFile    = "shaders/common/postFx/smaa/smaa_NeighborhoodBlendingV.hlsl"
  smaaNeighborhoodBlendingShaderData.DXPixelShaderFile     = "shaders/common/postFx/smaa/smaa_NeighborhoodBlendingP.hlsl"
  smaaNeighborhoodBlendingShaderData.pixVersion = 5.0
  smaaNeighborhoodBlendingShaderData:setField("samplerNames", 0, "$colorTex")
  smaaNeighborhoodBlendingShaderData:setField("samplerNames", 1, "$blendTex")
  smaaNeighborhoodBlendingShaderData:registerObject("SMAA_NeighborhoodBlendingShaderData")
end

local smaaPostEffectCallbacks = {}
smaaPostEffectCallbacks.onEnabled = function()
  -- log('I', 'smaaPostEffect', 'onEnabled called for smaaPostEffect')
  local fxaa_postfx = scenetree.findObject("FXAA_PostEffect")
  if fxaa_postfx then
    fxaa_postfx:disable()
  end
  return true
end

smaaPostEffectCallbacks.preProcess = function()
  local smaaPostEffect = scenetree.findObject("SMAA_PostEffect")
  if smaaPostEffect then
    local rtSize = smaaPostEffect:getRenderTargetSize()
    local rtResolution = string.format("float4(1.0 / %d, 1.0 / %d, %d, %d)", rtSize.x, rtSize.y, rtSize.x, rtSize.y)
    local currentRTResolution = smaaPostEffect:getField("rtResolution", 0)
    if rtResolution ~= currentRTResolution then
      smaaPostEffect:setField("rtResolution", 0, rtResolution)
      smaaPostEffect:setShaderMacro("SMAA_RT_METRICS", rtResolution)
      local smaaPostEffect1 = scenetree.findObject("SMAA_PostEffect1")
      if smaaPostEffect1 then
        smaaPostEffect1:setShaderMacro("SMAA_RT_METRICS", rtResolution)
      end
      local smaaPostEffect2 = scenetree.findObject("SMAA_PostEffect2")
      if smaaPostEffect2 then
        smaaPostEffect2:setShaderMacro("SMAA_RT_METRICS", rtResolution)
      end
    end
  end
end

rawset(_G, "SMAA_PostEffectCallbacks", smaaPostEffectCallbacks)

local smaaPostEffect = scenetree.findObject("SMAA_PostEffect")
if not smaaPostEffect then
  smaaPostEffect = createObject("PostEffect")
  smaaPostEffect.isEnabled = false
  smaaPostEffect.allowReflectPass = false
  smaaPostEffect:setField("renderTime", 0, "PFXAfterDiffuse")
  smaaPostEffect:setField("texture", 0, "$backBuffer")
  smaaPostEffect:setField("target", 0, "$outTex")
  smaaPostEffect:setField("targetClear", 0, "PFXTargetClear_OnDraw")
  smaaPostEffect:setField("targetClearColor", 0, "0 0 0 0")
  smaaPostEffect:setField("stateBlock", 0, "SMAA_StateBlock")
  smaaPostEffect:setField("shader", 0, "SMAA_EdgeDetectionShaderData")

  local smaaPostEffect1 = createObject("PostEffect")
  smaaPostEffect1:inheritParentFields(smaaPostEffect)
  smaaPostEffect1:setField("texture", 0, "$inTex")
  smaaPostEffect1:setField("texture", 1, "shaders/common/postFx/smaa/AreaTexDX9.dds")
  smaaPostEffect1:setField("texture", 2, "shaders/common/postFx/smaa/SearchTex.dds")
  smaaPostEffect1:setField("target", 0, "$outTex")
  smaaPostEffect1:setField("targetClear", 0, "$PFXTargetClear_OnDraw")
  smaaPostEffect1:setField("targetClearColor", 0, "0 0 0 0")
  smaaPostEffect1:setField("stateBlock", 0, "SMAA_StateBlock")
  smaaPostEffect1:setField("shader", 0, "SMAA_BlendingWeightShaderData")

  local smaaPostEffect2 = createObject("PostEffect")
  smaaPostEffect2:inheritParentFields(smaaPostEffect1)
  smaaPostEffect2:setField("texture", 0, "$backBuffer")
  smaaPostEffect2:setField("texture", 1, "$inTex")
  smaaPostEffect2:setField("target", 0, "$backBuffer")
  smaaPostEffect2:setField("stateBlock", 0, "SMAA_StateBlock")
  smaaPostEffect2:setField("shader", 0, "SMAA_NeighborhoodBlendingShaderData")
  smaaPostEffect2:registerObject("SMAA_PostEffect2")

  smaaPostEffect1:add(smaaPostEffect2)
  smaaPostEffect1:registerObject("SMAA_PostEffect1")

  smaaPostEffect:add(smaaPostEffect1)
  smaaPostEffect:registerObject("SMAA_PostEffect")
end

-- log('I','postfx', 'Smaa.lua calling preProcess()....')
smaaPostEffectCallbacks.preProcess()

-- log('I','postfx', 'Smaa.lua load completed')
