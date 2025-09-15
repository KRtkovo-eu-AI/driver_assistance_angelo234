-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local pfxFlashShader = scenetree.findObject("PFX_FlashShader")
if not pfxFlashShader then
  pfxFlashShader = createObject("ShaderData")
  pfxFlashShader.DXVertexShaderFile    = "shaders/common/postFx/flashP.hlsl"
  pfxFlashShader.DXPixelShaderFile     = "shaders/common/postFx/flashP.hlsl"
  pfxFlashShader:setField("defines", 0, "WHITE_COLOR=float4(1.0,1.0,1.0,0.0);MUL_COLOR=float4(1.0,0.25,0.25,0.0)")
  pfxFlashShader.pixVersion = 5.0
  pfxFlashShader:registerObject("PFX_FlashShader")
end

local flashFx = scenetree.findObject("FlashFx")
if not flashFx then
  flashFx = createObject("PostEffect")
  flashFx.isEnabled = false
  flashFx.allowReflectPass = false
  flashFx:setField("renderTime", 0, "PFXAfterDiffuse")
  flashFx:setField("shader", 0, "PFX_FlashShader")
  flashFx:setField("stateBlock", 0, "PFX_DefaultStateBlock")
  flashFx:setField("texture", 0, "$backBuffer")
  flashFx.renderPriority = 10

  flashFx:registerObject("FlashFx")
end

local flashFxCallbacks = {}
flashFxCallbacks.setShaderConsts = function()
  local flashFx = scenetree.findObject("FlashFx")
  if flashFx then
    local damageFlash = 0 -- this is suppose to be retrieved from function Game.getDamageFlash()
    local whiteOut = 0 -- this is suppose to be retrieved from function Game.getWhiteOut()
    flashFx:setShaderConst("$damageFlash", damageFlash)
    flashFx:setShaderConst("$whiteOut", whiteOut)
  end
end

rawset(_G, "FlashFxCallbacks", flashFxCallbacks)
