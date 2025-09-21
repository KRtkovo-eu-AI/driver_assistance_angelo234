-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

 -- An implementation of "NVIDIA FXAA 3.11" by TIMOTHY LOTTES
 --
 -- http://timothylottes.blogspot.com/
 --
 -- The shader is tuned for the defaul quality and good performance.
 -- See shaders\common\postFx\fxaa\fxaaP.hlsl to tweak the internal
 -- quality and performance settings.
local fxaa_PostEffectCallbacks = {}
fxaa_PostEffectCallbacks.onEnabled = function()
  -- log('I', 'fxaaPostEffect', 'onEnabled called for fxaaPostEffect')
  local smaa_postfx = scenetree.findObject("SMAA_PostEffect")
  if smaa_postfx then
     smaa_postfx:disable()
  end
  return true
end
rawset(_G, "FXAA_PostEffectCallbacks", fxaa_PostEffectCallbacks)

local pfxDefaultStateBlock = scenetree.findObject("PFX_DefaultStateBlock")
local fxaaShaderData = scenetree.findObject("FXAA_PostEffect")
if not fxaaShaderData then
  local fxaaStateBlock = createObject("GFXStateBlockData")
  fxaaStateBlock:inheritParentFields(pfxDefaultStateBlock)
  fxaaStateBlock.samplersDefined = true
  fxaaStateBlock:setField("samplerStates", 0, "SamplerClampLinear")
  fxaaStateBlock:registerObject("FXAA_StateBlock")

  local fxaaShaderData = createObject("ShaderData")
  fxaaShaderData.DXVertexShaderFile    = "shaders/common/postFx/fxaa/fxaaV.hlsl"
  fxaaShaderData.DXPixelShaderFile     = "shaders/common/postFx/fxaa/fxaaP.hlsl"
  fxaaShaderData:setField("samplerNames", 0, "$colorTex")
  fxaaShaderData.pixVersion = 5.0
  fxaaShaderData:registerObject("FXAA_ShaderData")

  local fxaaPostEffect = createObject("PostEffect")
  fxaaPostEffect.isEnabled = false
  fxaaPostEffect.allowReflectPass = false
  fxaaPostEffect:setField("renderTime", 0, "PFXAfterDiffuse")
  fxaaPostEffect:setField("shader", 0, "FXAA_ShaderData")
  fxaaPostEffect:setField("stateBlock", 0, "FXAA_StateBlock")
  fxaaPostEffect:setField("texture", 0, "$backBuffer")
  fxaaPostEffect:setField("target", 0, "$backBuffer")
  fxaaPostEffect:registerObject("FXAA_PostEffect")
end