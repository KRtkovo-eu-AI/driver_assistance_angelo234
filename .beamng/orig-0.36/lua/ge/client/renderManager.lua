-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

local M = {}

M.initRenderManager = function()
  -- This token, and the associated render managers, ensure that driver MSAA
  -- does not get used for Advanced Lighting renders.  The 'AL_FormatResolve'
  -- PostEffect copies the result to the backbuffer.
  local alFormatToken = createObject("RenderFormatToken")
  alFormatToken.enabled = false
  alFormatToken:setField("format", 0, "GFXFormatR8G8B8A8")
  alFormatToken:setField("depthFormat", 0, getConsoleVariable("$GFXFormatDefaultDepth"))
  alFormatToken.aaLevel = 0 -- -1 = match backbuffer

  -- The contents of the back buffer before this format token is executed
  -- is provided in $inTex
  alFormatToken:setField("copyEffect", 0, "AL_FormatCopy")

  -- The contents of the render target created by this format token is
  -- provided in $inTex
  alFormatToken:setField("resolveEffect", 0, "AL_FormatCopy")
  alFormatToken:registerObject("AL_FormatToken")
end

-- This post effect is used to copy data from the non-MSAA back-buffer to the
-- device back buffer (which could be MSAA). It must be declared here so that
-- it is initialized when 'AL_FormatToken' is initialzed.
local alFormatTokenState = scenetree.findObject("AL_FormatTokenState")
if not alFormatTokenState then
  alFormatTokenState = createObject("GFXStateBlockData")
  local pfx_defaultstateblock = scenetree.findObject("PFX_DefaultStateBlock")
  if pfx_defaultstateblock then
    alFormatTokenState:inheritParentFields(pfx_defaultstateblock)
  end
  alFormatTokenState:setField("samplersDefined", 0, "true")
  local samplerClampPoint = scenetree.findObject("SamplerClampPoint")
  alFormatTokenState.samplerStates = samplerClampPoint
  alFormatTokenState:registerObject("AL_FormatTokenState")
end

-- This PostEffect is used by 'AL_FormatToken' directly. It is never added to
-- the PostEffectManager. Do not call enable() on it.
local alFormatCopy = scenetree.findObject("AL_FormatCopy")
if not alFormatCopy then
  alFormatCopy = createObject("PostEffect")
  alFormatCopy.isEnabled = false
  alFormatCopy.allowReflectPass = true
  alFormatCopy.shader = "PFX_PassthruShader"
  alFormatCopy.stateBlock = scenetree.findObject("AL_FormatTokenState")
  alFormatCopy.texture = "$inTex"
  alFormatCopy.target = "$backbuffer"
  alFormatCopy:registerObject("AL_FormatCopy")
end
return M