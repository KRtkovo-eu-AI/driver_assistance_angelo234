-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

------------------------------------------------------------------------------
-- Fog
------------------------------------------------------------------------------

local fogPassShader = scenetree.findObject("FogPassShader")
if not fogPassShader then
  fogPassShader = createObject("ShaderData")
  fogPassShader.DXVertexShaderFile    = "shaders/common/postFx/fogP.hlsl"
  fogPassShader.DXPixelShaderFile     = "shaders/common/postFx/fogP.hlsl"
  fogPassShader:setField("samplerNames", 0, "$prepassTex")
  fogPassShader.pixVersion = 5.0
  fogPassShader:registerObject("FogPassShader")
end


local pfxDefaultStateBlock = scenetree.findObject("PFX_DefaultStateBlock")

local fogPassStateBlock = scenetree.findObject("FogPassStateBlock")
if not fogPassStateBlock then
  fogPassStateBlock = createObject("GFXStateBlockData")
  fogPassStateBlock:inheritParentFields(pfxDefaultStateBlock)
  fogPassStateBlock.blendDefined = true
  fogPassStateBlock.blendEnable = true
  fogPassStateBlock:setField("blendSrc", 0, "GFXBlendSrcAlpha")
  fogPassStateBlock:setField("blendDest", 0, "GFXBlendInvSrcAlpha")
  fogPassStateBlock:registerObject("FogPassStateBlock")
end

local fogPostFx = scenetree.findObject("FogPostFx")
if not fogPostFx then
  fogPostFx = createObject("PostEffect")
  fogPostFx.isEnabled = true
  fogPostFx.allowReflectPass = false
  fogPostFx:setField("renderTime", 0, "PFXBeforeBin")
  fogPostFx:setField("renderBin", 0, "ObjTranslucentBin")
  fogPostFx:setField("shader", 0, "FogPassShader")
  fogPostFx:setField("stateBlock", 0, "FogPassStateBlock")
  fogPostFx:setField("texture", 0, "#prepass[RT0]")
  fogPostFx:setField("texture", 1, "#prepass[Depth]")
  fogPostFx.renderPriority = 5
  fogPostFx:registerObject("FogPostFx")
end

------------------------------------------------------------------------------
-- UnderwaterFog
------------------------------------------------------------------------------

local underwaterFogPostFxCallbacks = {}
underwaterFogPostFxCallbacks.onEnabled = function()
  local causticsFX = scenetree.findObject("CausticsPFX")
  if causticsFX then
    causticsFX:enable()
  end
  return true;
end

underwaterFogPostFxCallbacks.onDisabled = function()
  local causticsFX = scenetree.findObject("CausticsPFX")
  if causticsFX then
    causticsFX:disable()
  end
end
rawset(_G, "UnderwaterFogPostFxCallbacks", underwaterFogPostFxCallbacks)

local underwaterFogPassShader = scenetree.findObject("UnderwaterFogPassShader")
if not underwaterFogPassShader then
  underwaterFogPassShader = createObject("ShaderData")
  underwaterFogPassShader.DXVertexShaderFile    = "shaders/common/postFx/underwaterFogP.hlsl"
  underwaterFogPassShader.DXPixelShaderFile     = "shaders/common/postFx/underwaterFogP.hlsl"
  underwaterFogPassShader:setField("samplerNames", 0, "$prepassTex")
  underwaterFogPassShader.pixVersion = 5.0
  underwaterFogPassShader:registerObject("UnderwaterFogPassShader")
end

local underwaterFogPassStateBlock = scenetree.findObject("UnderwaterFogPassStateBlock")
if not underwaterFogPassStateBlock then
  underwaterFogPassStateBlock = createObject("GFXStateBlockData")
  underwaterFogPassStateBlock:inheritParentFields(pfxDefaultStateBlock)
  underwaterFogPassStateBlock.samplersDefined = true
  underwaterFogPassStateBlock:setField("samplerStates", 0, "SamplerClampPoint")
  underwaterFogPassStateBlock:setField("samplerStates", 1, "SamplerClampPoint")
  underwaterFogPassStateBlock:setField("samplerStates", 2, "SamplerClampPoint")
  underwaterFogPassStateBlock:setField("samplerStates", 3, "SamplerClampLinear")
  underwaterFogPassStateBlock:registerObject("UnderwaterFogPassStateBlock")
end

local underwaterFogPostFx = scenetree.findObject("UnderwaterFogPostFx")
if not underwaterFogPostFx then
  underwaterFogPostFx = createObject("PostEffect")
  underwaterFogPostFx.oneFrameOnly = true
  underwaterFogPostFx.onThisFrame = false
  underwaterFogPostFx.isEnabled = true
  underwaterFogPostFx.allowReflectPass = false
  underwaterFogPostFx:setField("renderTime", 0, "PFXBeforeBin")
  underwaterFogPostFx:setField("renderBin", 0, "ObjTranslucentBin")
  underwaterFogPostFx:setField("shader", 0, "UnderwaterFogPassShader")
  underwaterFogPostFx:setField("stateBlock", 0, "UnderwaterFogPassStateBlock")
  underwaterFogPostFx:setField("texture", 0, "#prepass[RT0]")
  underwaterFogPostFx:setField("texture", 1, "#prepass[Depth]")
  underwaterFogPostFx:setField("texture", 2, "$backBuffer")
  underwaterFogPostFx:setField("texture", 3, "#waterDepthGradMap")
  underwaterFogPostFx.renderPriority = 4

  underwaterFogPostFx:registerObject("UnderwaterFogPostFx")
end