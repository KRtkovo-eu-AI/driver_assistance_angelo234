-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local pfxDefaultEdgeAAStateBlock = scenetree.findObject("PFX_DefaultEdgeAAStateBlock")
if not pfxDefaultEdgeAAStateBlock then
  pfxDefaultEdgeAAStateBlock = createObject("GFXStateBlockData")
  pfxDefaultEdgeAAStateBlock.zDefined = true
  pfxDefaultEdgeAAStateBlock.zEnable = false
  pfxDefaultEdgeAAStateBlock.zWriteEnable = false
  pfxDefaultEdgeAAStateBlock.samplersDefined = true
  pfxDefaultEdgeAAStateBlock:setField("samplerStates", 0, "SamplerClampPoint")
  pfxDefaultEdgeAAStateBlock:setField("samplerStates", 1, "SamplerClampPoint")
  pfxDefaultEdgeAAStateBlock:registerObject("PFX_DefaultEdgeAAStateBlock")
end

local pfxEdgeAADetectShader = scenetree.findObject("PFX_EdgeAADetectShader")
if not pfxEdgeAADetectShader then
  pfxEdgeAADetectShader = createObject("ShaderData")
  pfxEdgeAADetectShader.DXVertexShaderFile    = "shaders/common/postFx/edgeaa/edgeDetectP.hlsl"
  pfxEdgeAADetectShader.DXPixelShaderFile     = "shaders/common/postFx/edgeaa/edgeDetectP.hlsl"
  pfxEdgeAADetectShader:setField("samplerNames", 0, "prepassBuffer")
  pfxEdgeAADetectShader.pixVersion = 5.0
  pfxEdgeAADetectShader:registerObject("PFX_EdgeAADetectShader")
end

local pfxEdgeAAShader = scenetree.findObject("PFX_EdgeAAShader")
if not pfxEdgeAAShader then
  pfxEdgeAAShader = createObject("ShaderData")
  pfxEdgeAAShader.DXVertexShaderFile    = "shaders/common/postFx/edgeaa/edgeAAV.hlsl"
  pfxEdgeAAShader.DXPixelShaderFile     = "shaders/common/postFx/edgeaa/edgeAAP.hlsl"
  pfxEdgeAAShader:setField("samplerNames", 0, "edgeBuffer")
  pfxEdgeAAShader:setField("samplerNames", 1, "backBuffer")
  pfxEdgeAAShader.pixVersion = 5.0
  pfxEdgeAAShader:registerObject("PFX_EdgeAAShader")
end

local pfxEdgeAADebugShader = scenetree.findObject("PFX_EdgeAADebugShader")
if not pfxEdgeAADebugShader then
  pfxEdgeAADebugShader = createObject("ShaderData")
  pfxEdgeAADebugShader.DXVertexShaderFile    = "shaders/common/postFx/edgeaa/dbgEdgeDisplayP.hlsl"
  pfxEdgeAADebugShader.DXPixelShaderFile     = "shaders/common/postFx/edgeaa/dbgEdgeDisplayP.hlsl"
  pfxEdgeAADebugShader:setField("samplerNames", 0, "edgeBuffer")
  pfxEdgeAADebugShader.pixVersion = 5.0
  pfxEdgeAADebugShader:registerObject("PFX_EdgeAADebugShader")
end

local edgeDetectPostEffect = scenetree.findObject("EdgeDetectPostEffect")
if not edgeDetectPostEffect then
  edgeDetectPostEffect = createObject("PostEffect")
  edgeDetectPostEffect.isEnabled = true
  edgeDetectPostEffect:setField("renderTime", 0, "PFXBeforeBin")
  edgeDetectPostEffect:setField("renderBin", 0, "ObjTranslucentBin")
  -- edgeDetectPostEffect.renderPriority = 0.1
  edgeDetectPostEffect:setField("targetScale", 0, "0.5 0.5")
  edgeDetectPostEffect:setField("shader", 0, "PFX_EdgeAADetectShader")
  edgeDetectPostEffect:setField("stateBlock", 0, "PFX_DefaultEdgeAAStateBlock")
  edgeDetectPostEffect:setField("texture", 0, "#prepass[RT0]")
  edgeDetectPostEffect:setField("texture", 1, "#prepass[Depth]")
  edgeDetectPostEffect:setField("target", 0, "#edge")
  edgeDetectPostEffect:registerObject("EdgeDetectPostEffect")
end

local edgeDetectPostEffect = scenetree.findObject("EdgeAAPostEffect")
if not edgeDetectPostEffect then
  edgeDetectPostEffect = createObject("PostEffect")
  edgeDetectPostEffect:setField("renderTime", 0, "PFXAfterDiffuse")
  -- edgeDetectPostEffect:setField("renderBin", 0, "ObjTranslucentBin")
  -- edgeDetectPostEffect.renderPriority = 0.1
  edgeDetectPostEffect:setField("shader", 0, "PFX_EdgeAAShader")
  edgeDetectPostEffect:setField("stateBlock", 0, "PFX_DefaultEdgeAAStateBlock")
  edgeDetectPostEffect:setField("texture", 0, "#edge")
  edgeDetectPostEffect:setField("texture", 1, "$backBuffer")
  edgeDetectPostEffect:setField("target", 0, "$backBuffer")
  edgeDetectPostEffect:registerObject("EdgeAAPostEffect")
end

local debug_EdgeAAPostEffect = scenetree.findObject("Debug_EdgeAAPostEffect")
if not debug_EdgeAAPostEffect then
  debug_EdgeAAPostEffect = createObject("PostEffect")
  debug_EdgeAAPostEffect:setField("renderTime", 0, "PFXAfterDiffuse")
  -- debug_EdgeAAPostEffect:setField("renderBin", 0, "ObjTranslucentBin")
  -- debug_EdgeAAPostEffect.renderPriority = 0.1
  debug_EdgeAAPostEffect:setField("shader", 0, "PFX_EdgeAADebugShader")
  debug_EdgeAAPostEffect:setField("stateBlock", 0, "PFX_DefaultEdgeAAStateBlock")
  debug_EdgeAAPostEffect:setField("texture", 0, "#edge")
  debug_EdgeAAPostEffect:setField("target", 0, "$backBuffer")
  debug_EdgeAAPostEffect:registerObject("Debug_EdgeAAPostEffect")
end
