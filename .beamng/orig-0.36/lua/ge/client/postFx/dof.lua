-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

--[[

================================================================================
The DOFPostEffect API
================================================================================

DOFPostEffect::setFocalDist( %dist )

@summary
This method is for manually controlling the focus distance. It will have no
effect if auto focus is currently enabled. Makes use of the parameters set by
setFocusParams.

@param dist
float distance in meters

--------------------------------------------------------------------------------

DOFPostEffect::setAutoFocus( %enabled )

@summary
This method sets auto focus enabled or disabled. Makes use of the parameters set
by setFocusParams. When auto focus is enabled it determines the focal depth
by performing a raycast at the screen-center.

@param enabled
bool

--------------------------------------------------------------------------------

DOFPostEffect::setFocusParams( %nearBlurMax, %farBlurMax, %minRange, %maxRange, %nearSlope, %farSlope )

Set the parameters that control how the near and far equations are calculated
from the focal distance. If you are not using auto focus you will need to call
setFocusParams PRIOR to calling setFocalDist.

@param nearBlurMax
float between 0.0 and 1.0
The max allowed value of near blur.

@param farBlurMax
float between 0.0 and 1.0
The max allowed value of far blur.

@param minRange/maxRange
float in meters
The distance range around the focal distance that remains in focus is a lerp
between the min/maxRange using the normalized focal distance as the parameter.
The point is to allow the focal range to expand as you focus farther away since this is
visually appealing.

Note: since min/maxRange are lerped by the "normalized" focal distance it is
dependant on the visible distance set in your level.

@param nearSlope
float less than zero
The slope of the near equation. A small number causes bluriness to increase gradually
at distances closer than the focal distance. A large number causes bluriness to
increase quickly.

@param farSlope
float greater than zero
The slope of the far equation. A small number causes bluriness to increase gradually
at distances farther than the focal distance. A large number causes bluriness to
increase quickly.

Note: To rephrase, the min/maxRange parameters control how much area around the
focal distance is completely in focus where the near/farSlope parameters control
how quickly or slowly bluriness increases at distances outside of that range.

================================================================================
Examples
================================================================================

Example1: Turn on DOF while zoomed in with a weapon.

NOTE: These are not real callbacks! Hook these up to your code where appropriate!

function onSniperZoom()
{
  // Parameterize how you want DOF to look.
  DOFPostEffect.setFocusParams( 0.3, 0.3, 50, 500, -5, 5 );

  // Turn on auto focus
  DOFPostEffect.setAutoFocus( true );

  // Turn on the PostEffect
  DOFPostEffect.enable();
}

function onSniperUnzoom()
{
  // Turn off the PostEffect
  DOFPostEffect.disable();
}

Example2: Manually control DOF with the mouse wheel.

// Somewhere on startup...

// Parameterize how you want DOF to look.
DOFPostEffect.setFocusParams( 0.3, 0.3, 50, 500, -5, 5 );

// Turn off auto focus
DOFPostEffect.setAutoFocus( false );

// Turn on the PostEffect
DOFPostEffect.enable();


NOTE: These are not real callbacks! Hook these up to your code where appropriate!

function onMouseWheelUp()
{
  // Since setFocalDist is really just a wrapper to assign to the focalDist
  // dynamic field we can shortcut and increment it directly.
  DOFPostEffect.focalDist += 8;
}

function onMouseWheelDown()
{
  DOFPostEffect.focalDist -= 8;
}
]]

--[[

More information...

This DOF technique is based on this paper:
http://http.developer.nvidia.com/GPUGems3/gpugems3_ch28.html

================================================================================
1. Overview of how we represent "Depth of Field"
================================================================================

DOF is expressed as an amount of bluriness per pixel according to its depth.
We represented this by a piecewise linear curve depicted below.

Note: we also refer to "bluriness" as CoC ( circle of confusion ) which is the term
used in the basis paper and in photography.


X-axis (depth)
x = 0.0----------------------------------------------x = 1.0

Y-axis (bluriness)
y = 1.0
|
|   ____(x1,y1)                         (x4,y4)____
|       (ns,nb)\  <--Line1  line2--->  /(fe,fb)
|               \                     /
|                \(x2,y2)     (x3,y3)/
|                 (ne,0)------(fs,0)
y = 0.0


I have labeled the "corners" of this graph with (Xn,Yn) to illustrate that
this is in fact a collection of line segments where the x/y of each point
corresponds to the key below.

key:
ns - (n)ear blur (s)tart distance
nb - (n)ear (b)lur amount (max value)
ne - (n)ear blur (e)nd distance
fs - (f)ar blur (s)tart distance
fe - (f)ar blur (e)nd distance
fb - (f)ar (b)lur amount (max value)

Of greatest importance in this graph is Line1 and Line2. Where...
L1 { (x1,y1), (x2,y2) }
L2 { (x3,y3), (x4,y4) }

Line one represents the amount of "near" blur given a pixels depth and line two
represents the amount of "far" blur at that depth.

Both these equations are evaluated for each pixel and then the larger of the two
is kept. Also the output blur (for each equation) is clamped between 0 and its
maximum allowable value.

Therefore, to specify a DOF "qualify" you need to specify the near-blur-line,
far-blur-line, and maximum near and far blur value.

================================================================================
2. Abstracting a "focal depth"
================================================================================

Although the shader(s) work in terms of a near and far equation it is more
useful to express DOF as an adjustable focal depth and derive the other parameters
"under the hood".

Given a maximum near/far blur amount and a near/far slope we can calculate the
near/far equations for any focal depth. We extend this to also support a range
of depth around the focal depth that is also in focus and for that range to
shrink or grow as the focal depth moves closer or farther.

Keep in mind this is only one implementation and depending on the effect you
desire you may which to express the relationship between focal depth and
the shader paramaters different.

]]

---------------------------------------------------------------------------
-- GFXStateBlockData / ShaderData
---------------------------------------------------------------------------
local pfxDefaultDOFStateBlock = scenetree.findObject("PFX_DefaultDOFStateBlock")
if not pfxDefaultDOFStateBlock then
  pfxDefaultDOFStateBlock = createObject("GFXStateBlockData")
  pfxDefaultDOFStateBlock.zDefined = true
  pfxDefaultDOFStateBlock.zEnable = false
  pfxDefaultDOFStateBlock.zWriteEnable = false
  pfxDefaultDOFStateBlock.samplersDefined = true
  pfxDefaultDOFStateBlock:setField("samplerStates", 0, "SamplerClampPoint")
  pfxDefaultDOFStateBlock:setField("samplerStates", 1, "SamplerClampPoint")
  pfxDefaultDOFStateBlock:registerObject("PFX_DefaultDOFStateBlock")
end

local pfxDOFCalcCoCStateBlock = scenetree.findObject("PFX_DOFCalcCoCStateBlock")
if not pfxDOFCalcCoCStateBlock then
  pfxDOFCalcCoCStateBlock = createObject("GFXStateBlockData")
  pfxDOFCalcCoCStateBlock.zDefined = true
  pfxDOFCalcCoCStateBlock.zEnable = false
  pfxDOFCalcCoCStateBlock.zWriteEnable = false
  pfxDOFCalcCoCStateBlock.samplersDefined = true
  pfxDOFCalcCoCStateBlock:setField("samplerStates", 0, "SamplerClampLinear")
  pfxDOFCalcCoCStateBlock:setField("samplerStates", 1, "SamplerClampLinear")
  pfxDOFCalcCoCStateBlock:registerObject("PFX_DOFCalcCoCStateBlock")
end

local pfxDOFDownSampleStateBlock = scenetree.findObject("PFX_DOFDownSampleStateBlock")
if not pfxDOFDownSampleStateBlock then
  pfxDOFDownSampleStateBlock = createObject("GFXStateBlockData")
  pfxDOFDownSampleStateBlock.zDefined = true
  pfxDOFDownSampleStateBlock.zEnable = false
  pfxDOFDownSampleStateBlock.zWriteEnable = false
  pfxDOFDownSampleStateBlock.samplersDefined = true
  pfxDOFDownSampleStateBlock:setField("samplerStates", 0, "SamplerClampLinear")
  pfxDOFDownSampleStateBlock:setField("samplerStates", 1, "SamplerClampPoint")
  pfxDOFDownSampleStateBlock:setField("samplerStates", 2, "SamplerClampPoint")
  pfxDOFDownSampleStateBlock:registerObject("PFX_DOFDownSampleStateBlock")
end

local pfxDOFBlurStateBlock = scenetree.findObject("PFX_DOFBlurStateBlock")
if not pfxDOFBlurStateBlock then
  pfxDOFBlurStateBlock = createObject("GFXStateBlockData")
  pfxDOFBlurStateBlock.zDefined = true
  pfxDOFBlurStateBlock.zEnable = false
  pfxDOFBlurStateBlock.zWriteEnable = false
  pfxDOFBlurStateBlock.samplersDefined = true
  pfxDOFBlurStateBlock:setField("samplerStates", 0, "SamplerClampLinear")
  pfxDOFBlurStateBlock:registerObject("PFX_DOFBlurStateBlock")
end

local pfxDOFFinalStateBlock = scenetree.findObject("PFX_DOFFinalStateBlock")
if not pfxDOFFinalStateBlock then
  pfxDOFFinalStateBlock = createObject("GFXStateBlockData")
  pfxDOFFinalStateBlock.zDefined = true
  pfxDOFFinalStateBlock.zEnable = false
  pfxDOFFinalStateBlock.zWriteEnable = false
  pfxDOFFinalStateBlock.samplersDefined = true
  pfxDOFFinalStateBlock:setField("samplerStates", 0, "SamplerClampLinear")
  pfxDOFFinalStateBlock:setField("samplerStates", 1, "SamplerClampLinear")
  pfxDOFFinalStateBlock:setField("samplerStates", 2, "SamplerClampLinear")
  pfxDOFFinalStateBlock:setField("samplerStates", 3, "SamplerClampPoint")
  pfxDOFFinalStateBlock:setField("samplerStates", 4, "SamplerClampPoint")
  pfxDOFFinalStateBlock.blendDefined = true
  pfxDOFFinalStateBlock.blendEnable = true
  pfxDOFFinalStateBlock:setField("blendDest", 0, "GFXBlendInvSrcAlpha")
  pfxDOFFinalStateBlock:setField("blendSrc", 0, "GFXBlendOne")
  pfxDOFFinalStateBlock:registerObject("PFX_DOFFinalStateBlock")
end

local pfxDOFDownSampleShader = scenetree.findObject("PFX_DOFDownSampleShader")
if not pfxDOFDownSampleShader then
  pfxDOFDownSampleShader = createObject("ShaderData")
  pfxDOFDownSampleShader.DXVertexShaderFile    = "shaders/common/postFx/dof/DOF_DownSample_V.hlsl"
  pfxDOFDownSampleShader.DXPixelShaderFile     = "shaders/common/postFx/dof/DOF_DownSample_P.hlsl"
  pfxDOFDownSampleShader.pixVersion = 5.0
  pfxDOFDownSampleShader:registerObject("PFX_DOFDownSampleShader")
end

local pfxDOFBlurYShader = scenetree.findObject("PFX_DOFBlurYShader")
if not pfxDOFBlurYShader then
  pfxDOFBlurYShader = createObject("ShaderData")
  pfxDOFBlurYShader.DXVertexShaderFile    = "shaders/common/postFx/dof/DOF_Gausian_V.hlsl"
  pfxDOFBlurYShader.DXPixelShaderFile     = "shaders/common/postFx/dof/DOF_Gausian_P.hlsl"
  pfxDOFBlurYShader.pixVersion = 5.0
  pfxDOFBlurYShader.defines = "BLUR_DIR=float2(0.0,1.0)"
  pfxDOFBlurYShader:registerObject("PFX_DOFBlurYShader")
end

local pfxDOFBlurXShader = scenetree.findObject("PFX_DOFBlurXShader")
if not pfxDOFBlurXShader then
  pfxDOFBlurXShader = createObject("ShaderData")
  pfxDOFBlurXShader:inheritParentFields(pfxDOFBlurYShader)
  pfxDOFBlurXShader.defines = "BLUR_DIR=float2(1.0,0.0)"
  pfxDOFBlurXShader:registerObject("PFX_DOFBlurXShader")
end

local pfxDOFCalcCoCShader = scenetree.findObject("PFX_DOFCalcCoCShader")
if not pfxDOFCalcCoCShader then
  pfxDOFCalcCoCShader = createObject("ShaderData")
  pfxDOFCalcCoCShader.DXVertexShaderFile    = "shaders/common/postFx/dof/DOF_CalcCoC_V.hlsl"
  pfxDOFCalcCoCShader.DXPixelShaderFile     = "shaders/common/postFx/dof/DOF_CalcCoC_P.hlsl"
  pfxDOFCalcCoCShader.pixVersion = 5.0
  pfxDOFCalcCoCShader:registerObject("PFX_DOFCalcCoCShader")
end

local pfxDOFSmallBlurShader = scenetree.findObject("PFX_DOFSmallBlurShader")
if not pfxDOFSmallBlurShader then
  pfxDOFSmallBlurShader = createObject("ShaderData")
  pfxDOFSmallBlurShader.DXVertexShaderFile    = "shaders/common/postFx/dof/DOF_SmallBlur_V.hlsl"
  pfxDOFSmallBlurShader.DXPixelShaderFile     = "shaders/common/postFx/dof/DOF_SmallBlur_P.hlsl"
  pfxDOFSmallBlurShader.pixVersion = 5.0
  pfxDOFSmallBlurShader:registerObject("PFX_DOFSmallBlurShader")
end

local pfxDOFFinalShader = scenetree.findObject("PFX_DOFFinalShader")
if not pfxDOFFinalShader then
  pfxDOFFinalShader = createObject("ShaderData")
  pfxDOFFinalShader.DXVertexShaderFile    = "shaders/common/postFx/dof/DOF_Final_V.hlsl"
  pfxDOFFinalShader.DXPixelShaderFile     = "shaders/common/postFx/dof/DOF_Final_P.hlsl"
  pfxDOFFinalShader.pixVersion = 5.0
  pfxDOFFinalShader:registerObject("PFX_DOFFinalShader")
end

---------------------------------------------------------------------------
-- PostEffects
---------------------------------------------------------------------------
local function setLerpDist(dof, d0, d1, d2)
  dof.lerpScale = string.format("%f %f %f %f", -1.0/d0, -1.0/d1, -1.0/d2, 1.0/d2)
  dof.lerpBias = string.format("1.0 %f %f %f", (1.0 - d2)/d1, 1.0 / d2, (d2 - 1.0) / d2)
end

-- This method is for manually controlling the focal distance. It will have no
-- effect if auto focus is currently enabled. Makes use of the parameters set by
-- setFocusParams.
local function setFocalDist(dof, dist)
  -- Only set manual focal distance if autofocus is disabled
  if not dof.autoFocusEnabled then
    dof.focalDist = dist
  end
end

-- This method sets auto focus enabled or disabled. Makes use of the parameters set
-- by setFocusParams. When auto focus is enabled it determine the focal depth
-- by performing a raycast at the screen-center.
local function setAutoFocus(dof, enabled)
  dof.autoFocusEnabled = enabled
end

local function setDebugMode(dof, enabled)
  dof.debugModeEnabled = enabled
end

-- Set the parameters that control how the near and far equations are calculated
-- from the focal distance. If you are not using auto focus you will need to call
-- setFocusParams PRIOR to calling setFocalDist.
local function setFocusParams(dof, nearBlurMax, farBlurMax, minRange, maxRange, nearSlope, farSlope )
  dof.nearBlurMax = clamp(tonumber(nearBlurMax), 0.0, 1.0)
  dof.farBlurMax  = clamp(tonumber(farBlurMax) , 0.0, 1.0)
  dof.minRange = tonumber(minRange)
  dof.maxRange = tonumber(maxRange)
  dof.nearSlope = tonumber(nearSlope)
  dof.farSlope = tonumber(farSlope)
end

local function calculateFocusDistance()
  local camera = getCamera()
  if not camera then return nil end

  local camPos = vec3(core_camera.getPosition())

  -- First try to focus on player vehicle if it's visible
  local veh = be:getPlayerVehicle(0)
  if veh then
    local sceneObject = scenetree.findObjectById(be:getPlayerVehicleID(0))
    if sceneObject and not Engine.sceneGetCameraFrustum():isBoxOutside(sceneObject:getWorldBox()) then
      local vehPos = vec3(veh:getPosition())
      -- Calculate distance between camera and vehicle
      return (vehPos - camPos):length()
    end
  end

  -- Fallback to center raycast if vehicle is not in view
  local direction = vec3(core_camera.getForward())
  local farDist = tonumber(TorqueScriptLua.getVar("$Param::FarDist"))
  local endPoint = camPos + farDist * direction

  local m1 = tonumber(TorqueScriptLua.getVar("$TypeMasks::StaticObjectType"))
  local m2 = tonumber(TorqueScriptLua.getVar("$TypeMasks::TerrainObjectType"))
  local m3 = tonumber(TorqueScriptLua.getVar("$TypeMasks::VehicleObjectType"))
  local m4 = tonumber(TorqueScriptLua.getVar("$TypeMasks::DynamicShapeObjectType"))
  local mask = m1 + m2 + m3 + m4

  local result = containerRayCast(camPos:toPoint3F(), endPoint:toPoint3F(), mask, camera, true)

  local resultArgs = split(result, ' ')
  if result == "" or tableSize(resultArgs) < 4 or not resultArgs[1] or not resultArgs[2] or not resultArgs[3] then
    return farDist
  end

  local hitPos = vec3(tonumber(resultArgs[2]), tonumber(resultArgs[3]), tonumber(resultArgs[4]))
  return (hitPos - camPos):length()
end

-- Add temporal smoothing helper
local viewSmoother = newTemporalSmoothing()
local afSpeed = 150 -- Adjust this value to control smoothing speed (higher = faster)

local function autoFocus(dof)
  local targetDistance = calculateFocusDistance()
  if targetDistance then
    -- Apply temporal smoothing to the focus distance
    local smoothedDistance = viewSmoother:getWithRateUncapped(targetDistance, dtReal or 0.016, afSpeed)
    dof.focalDist = smoothedDistance
  end
end

local dOFPostEffectCallbacks = {}

dOFPostEffectCallbacks.onAdd = function()
  -- The weighted distribution of CoC value to the three blur textures
  -- in the order small, medium, large. Most likely you will not need to
  -- change this value.
  local dofPostEffect = scenetree.findObject("DOFPostEffect")
  if not dofPostEffect then
    return
  end
  -- log('I','dof','DOF onadd called....')
  -- log('I','dof','dof = '..dumps(dof))
  -- log('I','dof','dofPostEffect = '..dumps(dofPostEffect))
  setLerpDist(dofPostEffect, 0.2, 0.3, 0.5)

  -- Fill out some default values but DOF really should not be turned on
  -- without actually specifying your own parameters!
  dofPostEffect.autoFocusEnabled = false
  dofPostEffect.debugModeEnabled = false
  dofPostEffect.focalDist = 0.0
  dofPostEffect.nearBlurMax = 0.5
  dofPostEffect.farBlurMax = 0.5
  dofPostEffect.minRange = 50
  dofPostEffect.maxRange = 500
  dofPostEffect.nearSlope = -5.0
  dofPostEffect.farSlope = 5.0
end

dOFPostEffectCallbacks.setShaderConsts = function()
  local dofPostEffect = scenetree.findObject("DOFPostEffect")
  if not dofPostEffect or not dofPostEffect.focalDist then
    return
  end

  if dofPostEffect.autoFocusEnabled == true then
    -- log('I','dof', ' setShaderConsts dofPostEffect = '..dumps(dofPostEffect))
    -- log('I','dof', '     dof = '..dumps(dof))
    autoFocus(dofPostEffect)
  end

  local farDist = tonumber(TorqueScriptLua.getVar("$Param::FarDist"))
  local fd = dofPostEffect.focalDist / farDist

  -- rangeNear is done in two phases, so that it can be clamped with rangeFar
  local rangeNearTemp = lerp(dofPostEffect.minRange, dofPostEffect.maxRange, fd) / farDist * 0.5
  local rangeFar = lerp(dofPostEffect.maxRange, dofPostEffect.minRange, fd) / farDist * 0.5
  local rangeNear = clamp(rangeNearTemp, 0, rangeFar)

  -- We work in "depth" space rather than real-world units for the
  -- rest of this method...

  -- Given the focal distance and the range around it we want in focus
  -- we can determine the near-end-distance and far-start-distance

  -- Original code was broken, so instead we re-use fsd
  -- %ned = getMax( %fd - %range, 0.0 );

  local ned = math.min(fd + rangeNear, 1.0)
  local fsd = math.min(fd + rangeFar, 1.0)

  -- nearSlope
  local nsl = dofPostEffect.nearSlope

  -- Given slope of near blur equation and the near end dist and amount (x2,y2)
  -- solve for the y-intercept
  -- y = mx + b
  -- so...
  -- y - mx = b

  local b = 0.0 - nsl * ned
  local eqNear = string.format("%f %f 0.0", nsl, b)

  -- Do the same for the far blur equation...
  local fsl = dofPostEffect.farSlope
  b = 0.0 - fsl * fsd
  local eqFar = string.format("%f %f 1.0", fsl, b)

  -- Exposed the variables to the shader
  local dofFinalPFX = scenetree.findObject("DOFFinalPFX")
  if dofFinalPFX then
    dofFinalPFX:setShaderConst("$dofEqWorld", eqNear)
    dofFinalPFX:setShaderConst("$dofEqFar", eqFar)
    dofFinalPFX:setShaderConst("$maxWorldCoC", dofPostEffect.nearBlurMax)
    dofFinalPFX:setShaderConst("$maxFarCoC", dofPostEffect.farBlurMax)
    dofFinalPFX:setShaderConst("$dofLerpScale", dofPostEffect.lerpScale)
    dofFinalPFX:setShaderConst("$dofLerpBias", dofPostEffect.lerpBias)
    dofFinalPFX:setShaderConst("$isDebugMode", dofPostEffect.debugModeEnabled and "1" or "0") -- send "1" for true and "0" for false
  end
end
rawset(_G, "DOFPostEffectCallbacks", dOFPostEffectCallbacks)

local dofPostEffect = scenetree.findObject("DOFPostEffect")
if not dofPostEffect then
  dofPostEffect = createObject("PostEffect")
  dofPostEffect.isEnabled = false
  dofPostEffect:setField("renderTime", 0, "PFXAfterBin")
  dofPostEffect:setField("renderBin", 0, "GlowBin")
  dofPostEffect.renderPriority = 0.1
  dofPostEffect:setField("shader", 0, "PFX_DOFDownSampleShader")
  dofPostEffect:setField("stateBlock", 0, "PFX_DOFDownSampleStateBlock")

  dofPostEffect:setField("texture", 0, "$backBuffer")
  dofPostEffect:setField("texture", 1, "#prepass[RT0]")
  dofPostEffect:setField("texture", 2, "#prepass[Depth]")

  dofPostEffect:setField("target", 0, "#shrunk")
  dofPostEffect:setField("targetFormat", 0, "GFXFormatR16G16B16A16F")
  dofPostEffect:setField("targetScale", 0, "0.4 0.4")

  dofPostEffect:registerObject("DOFPostEffect")

  local dofBlurY = createObject("PostEffect")
  dofBlurY:setField("shader", 0, "PFX_DOFBlurYShader")
  dofBlurY:setField("stateBlock", 0, "PFX_DOFBlurStateBlock")
  dofBlurY:setField("texture", 0, "#shrunk")
  dofBlurY:setField("target", 0, "$outTex")
  dofBlurY:setField("targetFormat", 0, "GFXFormatR16G16B16A16F")
  dofBlurY:registerObject("DOFBlurY")
  dofPostEffect:add(dofBlurY)

  local dofBlurX = createObject("PostEffect")
  dofBlurX:setField("shader", 0, "PFX_DOFBlurXShader")
  dofBlurX:setField("stateBlock", 0, "PFX_DOFBlurStateBlock")
  dofBlurX:setField("texture", 0, "$inTex")
  dofBlurX:setField("target", 0, "#largeBlur")
  dofBlurX:setField("targetFormat", 0, "GFXFormatR16G16B16A16F")
  dofBlurX:registerObject("DOFBlurX")
  dofPostEffect:add(dofBlurX)

  local dofCalcCoC = createObject("PostEffect")
  dofCalcCoC:setField("shader", 0, "PFX_DOFCalcCoCShader")
  dofCalcCoC:setField("stateBlock", 0, "PFX_DOFCalcCoCStateBlock")
  dofCalcCoC:setField("texture", 0, "#shrunk")
  dofCalcCoC:setField("texture", 1, "#largeBlur")
  dofCalcCoC:setField("target", 0, "$outTex")
  dofCalcCoC:setField("targetFormat", 0, "GFXFormatR16G16B16A16F")
  dofCalcCoC:registerObject("DOFCalcCoC")
  dofPostEffect:add(dofCalcCoC)

  local dofSmallBlur = createObject("PostEffect")
  dofSmallBlur:setField("shader", 0, "PFX_DOFSmallBlurShader")
  dofSmallBlur:setField("stateBlock", 0, "PFX_DefaultDOFStateBlock")
  dofSmallBlur:setField("texture", 0, "$inTex")
  dofSmallBlur:setField("target", 0, "$outTex")
  dofSmallBlur:setField("targetFormat", 0, "GFXFormatR16G16B16A16F")
  dofSmallBlur:registerObject("DOFSmallBlur")
  dofPostEffect:add(dofSmallBlur)

  local dofFinalPFX = createObject("PostEffect")
  dofFinalPFX:setField("shader", 0, "PFX_DOFFinalShader")
  dofFinalPFX:setField("stateBlock", 0, "PFX_DOFFinalStateBlock")
  dofFinalPFX:setField("texture", 0, "$backBuffer")
  dofFinalPFX:setField("texture", 1, "$inTex")
  dofFinalPFX:setField("texture", 2, "#largeBlur")
  dofFinalPFX:setField("texture", 3, "#prepass[RT0]")
  dofFinalPFX:setField("texture", 4, "#prepass[Depth]")
  dofFinalPFX:setField("target", 0, "$backBuffer")
  dofFinalPFX:registerObject("DOFFinalPFX")
  dofPostEffect:add(dofFinalPFX)
end

local M = {}

M.updateDOFSettings = function()
  local dofPostEffect = scenetree.findObject("DOFPostEffect")
  if not dofPostEffect then
    return
  end
  local blurMin = TorqueScriptLua.getVar('$DOFPostFx::BlurMin')
  local blurMax = TorqueScriptLua.getVar('$DOFPostFx::BlurMax')
  local focusRangeMin = TorqueScriptLua.getVar('$DOFPostFx::FocusRangeMin')
  local focusRangeMax = TorqueScriptLua.getVar('$DOFPostFx::FocusRangeMax')
  local blurCurveNear = TorqueScriptLua.getVar('$DOFPostFx::BlurCurveNear')
  local blurCurveFar = TorqueScriptLua.getVar('$DOFPostFx::BlurCurveFar')

  setFocusParams(dofPostEffect, blurMin, blurMax, focusRangeMin, focusRangeMax, -blurCurveNear, blurCurveFar)
  setAutoFocus(dofPostEffect, TorqueScriptLua.getBoolVar("$DOFPostFx::EnableAutoFocus"))
  setDebugMode(dofPostEffect, TorqueScriptLua.getBoolVar("$DOFPostFx::EnableDebugMode"))
  setFocalDist(dofPostEffect, 0)

  if TorqueScriptLua.getBoolVar("$DOFPostFx::Enable") then
    dofPostEffect:enable()
  else
    dofPostEffect:disable()
  end
end

return M

--[[ For debugging, use with care
function reloadDOF()
{
   exec( "./dof.cs" );
   DOFPostEffect.reload();
   DOFPostEffect.disable();
   DOFPostEffect.enable();
}

function dofMetricsCallback()
{
   return "  | DOF |" @
         "  Dist: " @ $DOF::debug_dist @
         "  Depth: " @ $DOF::debug_depth;
}
]]