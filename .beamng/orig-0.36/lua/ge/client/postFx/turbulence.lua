-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local pfxTurbulenceStateBlock = scenetree.findObject("PFX_TurbulenceStateBlock")
if not pfxTurbulenceStateBlock then
  local pfxDefaultStateBlock = scenetree.findObject("PFX_DefaultStateBlock")
  pfxTurbulenceStateBlock = createObject("GFXStateBlockData")
  pfxTurbulenceStateBlock:inheritParentFields(pfxDefaultStateBlock)
  pfxTurbulenceStateBlock.zDefined = false
  pfxTurbulenceStateBlock.zEnable = false
  pfxTurbulenceStateBlock.zWriteEnable = false
  pfxTurbulenceStateBlock.samplersDefined = true
  pfxTurbulenceStateBlock:setField("samplerStates", 0, "SamplerClampLinear")
  pfxTurbulenceStateBlock:registerObject("PFX_TurbulenceStateBlock")
end

local pfxTurbulenceShader = scenetree.findObject("PFX_TurbulenceShader")
if not pfxTurbulenceShader then
  pfxTurbulenceShader = createObject("ShaderData")
  pfxTurbulenceShader.DXVertexShaderFile = "shaders/common/postFx/turbulenceP.hlsl"
  pfxTurbulenceShader.DXPixelShaderFile  = "shaders/common/postFx/turbulenceP.hlsl"
  pfxTurbulenceShader.pixVersion = 5.0
  pfxTurbulenceShader:registerObject("PFX_TurbulenceShader")
end

local turbulenceFx = scenetree.findObject("TurbulenceFx")
if not turbulenceFx then
  turbulenceFx = createObject("PostEffect")
  turbulenceFx.isEnabled = false
  turbulenceFx.allowReflectPass = true
  turbulenceFx:setField("renderTime", 0, "PFXAfterBin")
  turbulenceFx:setField("renderBin", 0, "GlowBin")
  turbulenceFx.renderPriority = 0.5; --Render after the glows themselves
  turbulenceFx:setField("shader", 0, "PFX_TurbulenceShader")
  turbulenceFx:setField("stateBlock", 0, "PFX_TurbulenceStateBlock")
  turbulenceFx:setField("texture", 0, "$backBuffer")
  turbulenceFx:registerObject("TurbulenceFx")
end
