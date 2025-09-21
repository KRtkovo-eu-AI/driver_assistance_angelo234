-- Copyright (c) 2024, Enrico Bertolazzi, Marco Frego and BeamNG
-- All rights reserved.
-- Redistribution and use in source and binary forms, with or without
-- modification, are permitted provided that the following conditions are
-- met:

--     * Redistributions of source code must retain the above copyright
--       notice, this list of conditions and the following disclaimer.
--     * Redistributions in binary form must reproduce the above copyright
--       notice, this list of conditions and the following disclaimer in
--       the documentation and/or other materials provided with the distribution

-- THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
-- AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
-- IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
-- ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
-- LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
-- CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
-- SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
-- INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
-- CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
-- ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
-- POSSIBILITY OF SUCH DAMAGE.

-- The code in this module is heavily based on the code described here: https://ebertolazzi.github.io/Clothoids/
-- and also described in the following papers:
-- https://doi.org/10.1002/mma.3114
-- https://doi.org/10.1016/j.cam.2018.03.029

local M = {}

local logTag = 'roadNetworkEditor'

-- Private constants.
local abs, pi, sqrt, sin, cos = math.abs, math.pi, math.sqrt, math.sin, math.cos
local min, max, floor = math.min, math.max, math.floor
local fn, fd, gn, gd = {}, {}, {}, {}
fn[0] = 0.49999988085884732562
fn[1] = 1.3511177791210715095
fn[2] = 1.3175407836168659241
fn[3] = 1.1861149300293854992
fn[4] = 0.7709627298888346769
fn[5] = 0.4173874338787963957
fn[6] = 0.19044202705272903923
fn[7] = 0.06655998896627697537
fn[8] = 0.022789258616785717418
fn[9] = 0.0040116689358507943804
fn[10] = 0.0012192036851249883877
fd[0] = 1.0
fd[1] = 2.7022305772400260215
fd[2] = 4.2059268151438492767
fd[3] = 4.5221882840107715516
fd[4] = 3.7240352281630359588
fd[5] = 2.4589286254678152943
fd[6] = 1.3125491629443702962
fd[7] = 0.5997685720120932908
fd[8] = 0.20907680750378849485
fd[9] = 0.07159621634657901433
fd[10] = 0.012602969513793714191
fd[11] = 0.0038302423512931250065

gn[0] = 0.50000014392706344801
gn[1] = 0.032346434925349128728
gn[2] = 0.17619325157863254363
gn[3] = 0.038606273170706486252
gn[4] = 0.023693692309257725361
gn[5] = 0.007092018516845033662
gn[6] = 0.0012492123212412087428
gn[7] = 0.00044023040894778468486
gn[8] = -8.80266827476172521e-6
gn[9] = -1.4033554916580018648e-8
gn[10] = 2.3509221782155474353e-10

gd[0] = 1.0
gd[1] = 2.0646987497019598937
gd[2] = 2.9109311766948031235
gd[3] = 2.6561936751333032911
gd[4] = 2.0195563983177268073
gd[5] = 1.1167891129189363902
gd[6] = 0.57267874755973172715
gd[7] = 0.19408481169593070798
gd[8] = 0.07634808341431248904
gd[9] = 0.011573247407207865977
gd[10] = 0.0044099273693067311209
gd[11] = -0.00009070958410429993314

local halfPi = pi * 0.5
local eps = 1e-15
local eps10 = 0.1 * eps
local m_pi = 3.14159265358979323846264338328
local m_pi_2 = 1.57079632679489661923132169164
local m_1_sqrt_pi = 0.564189583547756286948079451561


-- Computes the Fresnel integrals C(x), S(x) using FCS.
local function fresnelCS(y)

  local FresnelC, FresnelS = nil, nil

  local x = abs(y)
  local xSq = x * x
  if x < 1.0 then
    local f1 = halfPi * xSq
    local t = -f1 * f1

    -- Cosine integral series.
    local twofn, fact, denterm, numterm, sum, term = 0.0, 1.0, 1.0, 1.0, 1.0, 1e99
    while abs(term) > eps * abs(sum) do
      twofn = twofn + 2.0
      fact = fact * twofn * (twofn - 1.0)
      denterm = denterm + 4.0
      numterm = numterm * t
      term = numterm / (fact * denterm)
      sum = sum + term
    end

    FresnelC = x * sum

    -- Sine integral series.
    twofn, fact, denterm, numterm, sum, term = 1.0, 1.0, 3.0, 1.0, 0.333333333333333, 1e99
    while abs(term) > eps * abs(sum) do
      twofn = twofn + 2.0
      fact = fact * twofn * (twofn - 1.0)
      denterm = denterm + 4.0
      numterm = numterm * t
      term = numterm / (fact * denterm)
      sum = sum + term
    end
    FresnelS = m_pi_2 * sum * (xSq * x)

  elseif x < 6.0 then
    -- Rational approximation for f.
    local sumn, sumd = 0.0, fd[11]
    for k = 10, 0, -1 do
      sumn = fn[k] + x * sumn
      sumd = fd[k] + x * sumd
    end
    local f = sumn / sumd

    -- Rational approximation for g.
    sumn, sumd = 0.0, gd[11]
    for k = 10, 0, -1 do
      sumn = gn[k] + x * sumn
      sumd = gd[k] + x * sumd
    end
    local g = sumn/sumd

    local U = m_pi_2 * xSq
    local SinU, CosU = sin(U), cos(U)
    FresnelC = 0.5 + f * SinU - g * CosU
    FresnelS = 0.5 - f * CosU - g * SinU
  else
    -- x >= 6; asymptotic expansions for f and g.
    local s = m_pi * xSq
    local t = -1 / (s * s)

    -- Expansion for f.
    local numterm, term, sum, oldterm, absterm = -1.0, 1.0, 1.0, 1.0, 1e99
    while absterm > eps10 * abs(sum) do
      numterm = numterm + 4.0
      term = term * numterm * (numterm - 2.0) * t
      sum = sum + term
      absterm = abs(term)
      if oldterm < absterm then
        log('W', logTag, 'Clothoid fitting - f did not converge.')
      end
      oldterm = absterm
    end
    local f = sum / (m_pi * x)

    -- Expansion for g.
    numterm, term, sum, oldterm, absterm = -1.0, 1.0, 1.0, 1.0, 1e99
    while absterm > eps10 * abs(sum) do
      numterm = numterm + 4.0
      term = term * numterm * (numterm + 2.0) * t
      sum = sum + term
      absterm = abs(term)
      if oldterm < absterm then
        log('W', logTag, 'Clothoid fitting - g did not converge.')
      end
      oldterm = absterm
    end
    local gg = m_pi * x
    local g = sum / ( gg * gg * x)

    local U = m_pi_2 * xSq
    local SinU, CosU = sin(U), cos(U)
    FresnelC = 0.5 + f * SinU - g * CosU
    FresnelS = 0.5 - f * CosU - g * SinU
  end

  if y < 0 then
    FresnelC, FresnelS = -FresnelC, -FresnelS
  end

  return FresnelC, FresnelS
end

local function rLommel(mu, nu, b)
  local tmp = 1.0 / ((mu + nu + 1.0) * (mu - nu + 1.0))
  local res = tmp
  for n = 1, 100 do
    tmp = tmp * (-b / (2 * n + mu - nu + 1)) * (b / (2 * n + mu + nu + 1))
    res = res + tmp
    if abs(tmp) < abs(res) * 1e-50 then
      break
    end
  end
  return res
end

local function evalXYazero(b)
  local X, Y = {}, {}
  for i = 0, 15 do
    X[i], Y[i] = 0.0, 0.0
  end

  local sb, cb, b2, bInv = sin(b), cos(b), b * b, 1.0 / b
  if abs(b) < 1e-3 then
    X[0] = 1 - (b2 * 0.16666666666) * (1 - (b2 * 0.05) * (1 - (b2 * 0.0238095238)))
    Y[0] = (b * 0.5) * (1 - (b2 * 0.08333333333) * (1 - (b2 * 0.03333333333)))
  else
    X[0], Y[0] = sb * bInv, (1 - cb) * bInv
  end

  -- Use recurrence in the stable part.
  local m = min(max(1, floor(2 * b)), 15)
  local mMinus1 = m - 1
  for k = 1, mMinus1 do
    local kMinus1 = k - 1
    X[k], Y[k] = (sb - k * Y[kMinus1]) * bInv, (k * X[kMinus1] - cb) * bInv
  end

  -- Use Lommel in the unstable part.
  if m < 15 then
    local AA = b * sb
    local DD = sb - b * cb
    local BB = b * DD
    local CC = -b2 * sb
    local mPlusHalf = m + 0.5
    local rLa, rLd = rLommel(mPlusHalf, 1.5, b), rLommel(mPlusHalf, 0.5, b)
    for k = m, 14 do
      local kPlus1p5 = k + 1.5
      local rLb, rLc = rLommel(kPlus1p5, 0.5, b), rLommel(kPlus1p5, 1.5, b)
      X[k], Y[k] = (k * AA * rLa + BB * rLb + cb) / (k + 1), (CC * rLc + sb) / (k + 2) + DD * rLd
      rLa, rLd = rLc, rLb
    end
  end
  return X, Y
end

local function evalXYaSmall(a, b)
  local X0, Y0 = evalXYazero(b)
  local halfA = a * 0.5
  local tmpX = X0[0] - halfA * Y0[2]
  local tmpY = Y0[0] + halfA * X0[2]
  local aa, t = -a * a * 0.25, 1
  for n = 1, 3 do
    local n2 = 2 * n
    t = t * aa / (n2 * (n2 - 1))
    local bf = a / (4 * n + 2)
    local jj = 4 * n
    local jjPlus2 = jj + 2
    tmpX, tmpY = tmpX + t * (X0[jj] - bf * Y0[jjPlus2]), t * (Y0[jj] + bf * X0[jjPlus2] )
  end
  return tmpX, tmpY
end

local function evalXYaLarge(a, b)
  local s = sign2(a)
  local absA = abs(a)
  local z = m_1_sqrt_pi * sqrt(absA)
  local ell = s * b * m_1_sqrt_pi / sqrt(absA)
  local g = -0.5 * s * (b * b) / absA
  local zInv = 1.0 / z
  local cg, sg = cos(g) * zInv, sin(g) * zInv
  local Cl, Sl = fresnelCS(ell)
  local Cz, Sz = fresnelCS(ell + z)
  local dC, dS = Cz - Cl, Sz - Sl
  return cg * dC - s * sg * dS, sg * dC + s * cg * dS
end

local function generalizedFresnelCS(a, b, c)
  local X, Y = nil, nil
  if abs(a) < 1e-2 then                       -- case: 'a' small.
    X, Y = evalXYaSmall(a, b)
  else
    X, Y = evalXYaLarge(a, b)
  end
  local cc, ss = cos(c), sin(c)
  return X * cc - Y * ss, X * ss + Y * cc
end

-- Evaluates a Clothoid (Euler) spiral, at some parameter p in [0, geodesic length].
-- [INPUTS: Start point = start, Heading (rad) = theta0, Curvature = kappa, Rate of change of curvature of length = dKappa].
local function evaluate(start, theta0, kappa, dKappa, p)
  local C, S = generalizedFresnelCS(dKappa * p * p, kappa * p, theta0)
  return vec3(start.x + p * C, start.y + p * S)
end


-- Public interface.
M.evaluate =                                              evaluate

return M