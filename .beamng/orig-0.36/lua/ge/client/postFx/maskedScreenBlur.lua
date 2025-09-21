-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-----------------------------------------------
local screenBlurFX_YShader = scenetree.findObject("ScreenBlurFX_YShader")
if not screenBlurFX_YShader then
  screenBlurFX_YShader = createObject("ShaderData")
  screenBlurFX_YShader.DXVertexShaderFile    = "shaders/common/postFx/dof/DOF_Gausian_V.hlsl"
  screenBlurFX_YShader.DXPixelShaderFile     = "shaders/common/postFx/dof/DOF_Gausian_P.hlsl"
  screenBlurFX_YShader.pixVersion = 5.0
  screenBlurFX_YShader.defines = "BLUR_DIR=float2(0.0,1.0)"
  screenBlurFX_YShader:registerObject("ScreenBlurFX_YShader")
end

local screenBlurFX_XShader = scenetree.findObject("ScreenBlurFX_XShader")
if not screenBlurFX_XShader then
  screenBlurFX_XShader = createObject("ShaderData")
  screenBlurFX_XShader:inheritParentFields(screenBlurFX_YShader)
  screenBlurFX_XShader.defines = "BLUR_DIR=float2(1.0,0.0)"
  screenBlurFX_XShader:registerObject("ScreenBlurFX_XShader")
end

local screenBlurFX_stateBlock = scenetree.findObject("ScreenBlurFX_stateBlock")
if not screenBlurFX_stateBlock then
  screenBlurFX_stateBlock = createObject("GFXStateBlockData")
  screenBlurFX_stateBlock.zDefined = true
  screenBlurFX_stateBlock.zEnable = false
  screenBlurFX_stateBlock.zWriteEnable = false
  screenBlurFX_stateBlock.samplersDefined = true
  screenBlurFX_stateBlock:setField("samplerStates", 0, "SamplerClampLinear")
  screenBlurFX_stateBlock:setField("samplerStates", 1, "SamplerClampPoint")
  screenBlurFX_stateBlock:registerObject("ScreenBlurFX_stateBlock")
end

local simpleBlendShader = scenetree.findObject("SimpleBlendShader")
if not simpleBlendShader then
  simpleBlendShader = createObject("ShaderData")
  simpleBlendShader.DXVertexShaderFile    = "shaders/common/postFx/simpleBlendV.hlsl"
  simpleBlendShader.DXPixelShaderFile     = "shaders/common/postFx/simpleBlendP.hlsl"
  simpleBlendShader.pixVersion = 5.0
  simpleBlendShader:registerObject("SimpleBlendShader")
end

local simpleBlendShaderStateBlock = scenetree.findObject("SimpleBlendShaderStateBlock")
if not simpleBlendShaderStateBlock then
  simpleBlendShaderStateBlock = createObject("GFXStateBlockData")
  simpleBlendShaderStateBlock.zDefined = true
  simpleBlendShaderStateBlock.zEnable = false
  simpleBlendShaderStateBlock.zWriteEnable = false
  simpleBlendShaderStateBlock.samplersDefined = true
  simpleBlendShaderStateBlock:setField("samplerStates", 0, "SamplerClampLinear")
  simpleBlendShaderStateBlock:setField("samplerStates", 1, "SamplerClampLinear")
  simpleBlendShaderStateBlock:setField("samplerStates", 2, "SamplerClampLinear")
  simpleBlendShaderStateBlock:registerObject("SimpleBlendShaderStateBlock")
end

local screenBlurFX = scenetree.findObject("ScreenBlurFX")
if not screenBlurFX then
  screenBlurFX = createObject("PostEffectMaskedBlur")
  screenBlurFX.isEnabled = true
  screenBlurFX:setField("renderTime", 0, "PFXAfterBin")
  screenBlurFX:setField("renderBin", 0, "OverlayRender")
  screenBlurFX:setField("shader", 0, "PFX_PassthruShader")
  screenBlurFX:setField("stateBlock", 0, "AL_FormatTokenState")
  screenBlurFX:setField("texture", 0, "$backBuffer")
  screenBlurFX:setField("target", 0, "$outTex")
  screenBlurFX:setField("targetScale", 0, "0.4 0.4") -- 0.4 = 40% of the screen
  screenBlurFX:registerObject("ScreenBlurFX")

  local screenBlurFX_YShader = createObject("PostEffect")
  screenBlurFX_YShader:setField("shader", 0, "ScreenBlurFX_YShader")
  screenBlurFX_YShader:setField("stateBlock", 0, "ScreenBlurFX_stateBlock")
  screenBlurFX_YShader:setField("texture", 0, "$inTex")
  screenBlurFX_YShader:setField("target", 0, "$outTex")
  screenBlurFX_YShader:registerObject()
  screenBlurFX:add(screenBlurFX_YShader)

  local screenBlurFX_XShader = createObject("PostEffect")
  screenBlurFX_XShader:setField("shader", 0, "ScreenBlurFX_XShader")
  screenBlurFX_XShader:setField("stateBlock", 0, "ScreenBlurFX_stateBlock")
  screenBlurFX_XShader:setField("texture", 0, "$inTex")
  screenBlurFX_XShader:setField("target", 0, "$outTex")
  screenBlurFX_XShader:registerObject()
  screenBlurFX:add(screenBlurFX_XShader)

  local simpleBlendShader = createObject("PostEffect")
  simpleBlendShader:setField("shader", 0, "SimpleBlendShader")
  simpleBlendShader:setField("stateBlock", 0, "SimpleBlendShaderStateBlock")
  simpleBlendShader:setField("texture", 0, "$backBuffer")
  simpleBlendShader:setField("texture", 1, "$inTex")
  simpleBlendShader:setField("texture", 2, "#screenBlurMask")
  simpleBlendShader:setField("target", 0, "$backBuffer")
  simpleBlendShader:registerObject()
  screenBlurFX:add(simpleBlendShader)
end