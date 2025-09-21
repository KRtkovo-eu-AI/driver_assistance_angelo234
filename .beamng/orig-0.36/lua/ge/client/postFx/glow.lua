-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local pfxGlowBlurVertShader = scenetree.findObject("PFX_GlowBlurVertShader")
if not pfxGlowBlurVertShader then
  pfxGlowBlurVertShader = createObject("ShaderData")
  pfxGlowBlurVertShader.DXVertexShaderFile    = "shaders/common/postFx/glowBlurV.hlsl"
  pfxGlowBlurVertShader.DXPixelShaderFile     = "shaders/common/postFx/glowBlurP.hlsl"
  pfxGlowBlurVertShader:setField("defines", 0, "BLUR_DIR=float2(0.0,1.0)")
  pfxGlowBlurVertShader:setField("samplerNames", 0, "$diffuseMap")
  pfxGlowBlurVertShader.pixVersion = 5.0
  pfxGlowBlurVertShader:registerObject("PFX_GlowBlurVertShader")
end

local pfxGlowBlurHorzShader = scenetree.findObject("PFX_GlowBlurHorzShader")
if not pfxGlowBlurHorzShader then
  pfxGlowBlurHorzShader = createObject("ShaderData")
  pfxGlowBlurHorzShader:inheritParentFields(pfxGlowBlurVertShader)
  pfxGlowBlurHorzShader:setField("defines", 0, "BLUR_DIR=float2(1.0,0.0)")
  pfxGlowBlurHorzShader:registerObject("PFX_GlowBlurHorzShader")
end

local pfxGlowCombineStateBlock = scenetree.findObject("PFX_GlowCombineStateBlock")
if not pfxGlowCombineStateBlock then
  local pfxDefaultStateBlock = scenetree.findObject("PFX_DefaultStateBlock")
  pfxGlowCombineStateBlock = createObject("GFXStateBlockData")
  pfxGlowCombineStateBlock:inheritParentFields(pfxDefaultStateBlock)
  pfxGlowCombineStateBlock.alphaDefined = true
  pfxGlowCombineStateBlock.alphaTestEnable = true
  pfxGlowCombineStateBlock.alphaTestRef = 1
  pfxGlowCombineStateBlock:setField("alphaTestFunc", 0, "GFXCmpGreaterEqual")
  pfxGlowCombineStateBlock.blendDefined = true
  pfxGlowCombineStateBlock.blendEnable = true
  pfxGlowCombineStateBlock:setField("blendSrc", 0, "GFXBlendOne")
  pfxGlowCombineStateBlock:setField("blendDest", 0, "GFXBlendOne")
  pfxGlowCombineStateBlock:registerObject("PFX_GlowCombineStateBlock")
end

local gammaPostFX = scenetree.findObject("GlowPostFx")
if not gammaPostFX then
  local gammaPostFX = createObject("PostEffect")
  -- Do not allow the glow effect to work in reflection
  -- passes by default so we don't do the extra drawing.
  gammaPostFX.isEnabled = true
  gammaPostFX.allowReflectPass = false

  gammaPostFX:setField("renderTime", 0, "PFXAfterBin")
  gammaPostFX:setField("renderBin", 0, "GlowBin")
  gammaPostFX.renderPriority = 1;

  -- First we down sample the glow buffer.
  gammaPostFX:setField("shader", 0, "PFX_PassthruShader")
  gammaPostFX:setField("stateBlock", 0, "PFX_DefaultStateBlock")
  gammaPostFX:setField("texture", 0, "#glowbuffer")
  gammaPostFX:setField("target", 0, "$outTex")
  gammaPostFX:setField("targetScale", 0, "0.5 0.5")
  gammaPostFX:registerObject("GlowPostFx")

  local blurVertically = createObject("PostEffect")
  blurVertically:setField("shader", 0, "PFX_GlowBlurVertShader")
  blurVertically:setField("stateBlock", 0, "PFX_DefaultStateBlock")
  blurVertically:setField("texture", 0, "$inTex")
  blurVertically:setField("target", 0, "$outTex")
  blurVertically:registerObject()
  gammaPostFX:add(blurVertically)

  local blurHorizontally = createObject("PostEffect")
  blurHorizontally:setField("shader", 0, "PFX_GlowBlurHorzShader")
  blurHorizontally:setField("stateBlock", 0, "PFX_DefaultStateBlock")
  blurHorizontally:setField("texture", 0, "$inTex")
  blurHorizontally:setField("target", 0, "$outTex")
  blurHorizontally:registerObject()
  gammaPostFX:add(blurHorizontally)

  local upsampleAndCombine = createObject("PostEffect")
  upsampleAndCombine:setField("shader", 0, "PFX_PassthruShader")
  upsampleAndCombine:setField("stateBlock", 0, "PFX_GlowCombineStateBlock")
  upsampleAndCombine:setField("texture", 0, "$inTex")
  upsampleAndCombine:setField("target", 0, "$backBuffer")
  upsampleAndCombine:registerObject()
  gammaPostFX:add(upsampleAndCombine)
end