-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

-- log('I','postfx', 'Caustics.lua loaded.....')

local pfxCausticsStateBlock = scenetree.findObject("PFX_CausticsStateBlock")
if not pfxCausticsStateBlock then
  local pfxDefaultStateBlock = scenetree.findObject("PFX_DefaultStateBlock")
  pfxCausticsStateBlock = createObject("GFXStateBlockData")
  pfxCausticsStateBlock:inheritParentFields(pfxDefaultStateBlock)
  pfxCausticsStateBlock.blendDefined = true
  pfxCausticsStateBlock.blendEnable = true
  pfxCausticsStateBlock:setField("blendSrc", 0, "GFXBlendOne")
  pfxCausticsStateBlock:setField("blendDest", 0, "GFXBlendOne")
  pfxCausticsStateBlock.samplersDefined = true
  pfxCausticsStateBlock:setField("samplerStates", 0, "SamplerClampLinear")
  pfxCausticsStateBlock:setField("samplerStates", 1, "SamplerClampLinear")
  pfxCausticsStateBlock:setField("samplerStates", 2, "SamplerWrapLinear")
  pfxCausticsStateBlock:setField("samplerStates", 3, "SamplerWrapLinear")
  pfxCausticsStateBlock:registerObject("PFX_CausticsStateBlock")
end

local pfxCausticsShader = scenetree.findObject("PFX_CausticsShader")
if not pfxCausticsShader then
  pfxCausticsShader = createObject("ShaderData")
  pfxCausticsShader.DXVertexShaderFile    = "shaders/common/postFx/caustics/causticsP.hlsl"
  pfxCausticsShader.DXPixelShaderFile     = "shaders/common/postFx/caustics/causticsP.hlsl"
  pfxCausticsShader.pixVersion = 5.0
  pfxCausticsShader:registerObject("PFX_CausticsShader")
end

local causticsPFX = scenetree.findObject("CausticsPFX")
if not causticsPFX then
  causticsPFX = createObject("PostEffect")
  causticsPFX.isEnabled = false
  causticsPFX:setField("renderTime", 0, "PFXBeforeBin")
  causticsPFX:setField("renderBin", 0, "ObjTranslucentBin")
  -- causticsPFX.renderPriority = 0.1
  causticsPFX:setField("shader", 0, "PFX_CausticsShader")
  causticsPFX:setField("stateBlock", 0, "PFX_CausticsStateBlock")
  causticsPFX:setField("texture", 0, "#prepass[RT0]")
  causticsPFX:setField("texture", 1, "#prepass[Depth]")
  causticsPFX:setField("texture", 2, "lua/ge/client/postFx/textures/caustics_1.png")
  causticsPFX:setField("texture", 3, "lua/ge/client/postFx/textures/caustics_2.png")
  causticsPFX:setField("target", 0, "$backBuffer")
  causticsPFX:registerObject("CausticsPFX")
end