angular.module('beamng.apps')
.directive('laneCenteringAssistApp', [function () {
  return {
    templateUrl: '/ui/modules/apps/laneCenteringAssistApp/app.html',
    replace: true,
    restrict: 'EA',
    scope: true,
    controller: ['$scope', '$element', function ($scope, $element) {
      var canvas = $element.find('canvas')[0]
      var ctx = canvas.getContext('2d')
      var carColor = [255, 255, 255]
      var containerEl = $element[0]
      var resizeObserver = null
      var resizeScheduled = false
      var windowResizeHandler = null

      function scheduleFrame(callback) {
        var raf = (typeof window !== 'undefined' && window.requestAnimationFrame) || null
        if (raf) {
          raf(callback)
        } else {
          setTimeout(callback, 16)
        }
      }

      function computeScale(rect) {
        if (!rect || rect.width <= 0) return 1
        var widthScale = rect.width / 280
        var heightScale = rect.height > 0 ? rect.height / 240 : widthScale
        var scale = Math.min(widthScale, heightScale)
        if (!isFinite(scale) || scale <= 0) scale = 1
        if (scale < 0.85) scale = 0.85
        if (scale > 1.6) scale = 1.6
        return scale
      }

      function applyLayoutScale() {
        if (!containerEl || !containerEl.style) return
        var rect = containerEl.getBoundingClientRect()
        if (!rect || !rect.width) return
        var scale = computeScale(rect)
        containerEl.style.setProperty('--lca-scale', scale.toFixed(3))
      }

      function syncCanvasSize() {
        if (!canvas) return { width: 0, height: 0 }
        var width = canvas.clientWidth
        var height = canvas.clientHeight
        if (!width || !height) {
          ctx.setTransform(1, 0, 0, 1, 0, 0)
          return { width: width || 0, height: height || 0 }
        }

        var dpr = (typeof window !== 'undefined' && window.devicePixelRatio) ? window.devicePixelRatio : 1
        var pixelWidth = Math.round(width * dpr)
        var pixelHeight = Math.round(height * dpr)
        if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
          canvas.width = pixelWidth
          canvas.height = pixelHeight
        }
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
        return { width: width, height: height }
      }

      function handleResize() {
        applyLayoutScale()
        syncCanvasSize()
      }

      function scheduleResize() {
        if (resizeScheduled) return
        resizeScheduled = true
        scheduleFrame(function () {
          resizeScheduled = false
          handleResize()
        })
      }

      var vm = $scope.vm = {
        statusText: 'Not installed',
        statusClass: 'lca-status-missing',
        statusDetail: '',
        laneWidth: '—',
        offsetLane: '—',
        offsetTarget: '—',
        offsetAbsolute: '—',
        offsetNormalized: '—',
        offsetLegal: '—',
        offsetLegalError: '—',
        offsetLegalNormalized: '—',
        headingError: '—',
        curvature: '—',
        lookahead: '—',
        lookaheadTarget: '—',
        pathCoverage: '—',
        pathSegments: '—',
        pathTruncated: '—',
        speed: '—',
        assistForce: '—',
        assistWeight: '—',
        driverInput: '—',
        finalSteer: '—',
        pathPoints: '—',
        curvatureRadius: '—',
        branchCount: '—',
        branchInfo: '—',
        routeLength: '—',
        routeNodes: '—',
        routeSpan: '—',
        routeUpdated: '—',
        trafficSide: '—',
        warning: false
      }

      function formatNumber(value, digits) {
        if (typeof value !== 'number' || !isFinite(value)) {
          return '—'
        }
        return value.toFixed(digits)
      }

      function updateState(data) {
        var status = (data && data.status) || {}
        var lane = data && data.lane
        var assist = data && data.assist

        var statusText = 'Not installed'
        var statusClass = 'lca-status-missing'
        var statusDetail = ''

        if (!status.installed) {
          statusText = 'Not installed'
          statusClass = 'lca-status-missing'
        } else if (status.active) {
          statusText = 'Active'
          statusClass = 'lca-status-active'
        } else if (!status.enabled) {
          statusText = 'Standby'
          if (status.reason === 'ai_route_conflict') {
            statusClass = 'lca-status-warning'
            statusDetail = 'AI route conflict'
          } else if (status.reason === 'off_road') {
            statusClass = 'lca-status-warning'
            statusDetail = 'Off road'
          } else {
            statusClass = 'lca-status-idle'
            statusDetail = 'Disabled'
          }
        } else if (status.driverOverride || status.reason === 'driver_override') {
          statusText = 'Standby'
          statusClass = 'lca-status-warning'
          statusDetail = 'Driver override'
        } else if (status.reason === 'low_speed') {
          statusText = 'Standby'
          statusClass = 'lca-status-idle'
          statusDetail = 'Waiting for speed'
        } else if (!status.available) {
          statusText = 'Standby'
          statusClass = 'lca-status-idle'
          statusDetail = 'Searching lane'
        } else {
          statusText = 'Ready'
          statusClass = 'lca-status-ready'
        }

        vm.statusText = statusText
        vm.statusClass = statusClass
        vm.statusDetail = statusDetail
        vm.warning = !!(assist && assist.warning)

        if (lane) {
          vm.laneWidth = formatNumber(lane.width, 2)

          var laneOffset = (lane && lane.offset) || {}
          var targetOffset = (typeof laneOffset.target === 'number' && isFinite(laneOffset.target)) ? laneOffset.target : null
          var absoluteOffset = (typeof laneOffset.current === 'number' && isFinite(laneOffset.current)) ? laneOffset.current : null
          var relativeOffset = (typeof laneOffset.error === 'number' && isFinite(laneOffset.error)) ? laneOffset.error : null
          var legalOffset = (typeof laneOffset.legal === 'number' && isFinite(laneOffset.legal)) ? laneOffset.legal : null
          var legalError = (typeof laneOffset.legalError === 'number' && isFinite(laneOffset.legalError)) ? laneOffset.legalError : null
          var legalNormalized = (typeof laneOffset.legalNormalized === 'number' && isFinite(laneOffset.legalNormalized)) ? laneOffset.legalNormalized : null

          if (relativeOffset === null && absoluteOffset !== null && targetOffset !== null) {
            relativeOffset = absoluteOffset - targetOffset
          }

          if (absoluteOffset === null && relativeOffset !== null && targetOffset !== null) {
            absoluteOffset = relativeOffset + targetOffset
          }

          if (legalError === null && legalOffset !== null && absoluteOffset !== null) {
            legalError = absoluteOffset - legalOffset
          }

          vm.offsetLane = formatNumber(relativeOffset, 2)
          vm.offsetTarget = formatNumber(targetOffset, 2)
          vm.offsetAbsolute = formatNumber(absoluteOffset, 2)
          vm.offsetNormalized = formatNumber(laneOffset.normalized, 2)
          vm.offsetLegal = formatNumber(legalOffset, 2)
          vm.offsetLegalError = formatNumber(legalError, 2)
          vm.offsetLegalNormalized = formatNumber(legalNormalized, 2)
          vm.headingError = formatNumber((lane.heading.error || 0) * 180 / Math.PI, 1)
          vm.curvature = formatNumber(lane.curvature, 4)
          var path = lane.path
          if (path) {
            var pathLength = path.length
            var pathTarget = path.targetLength
            vm.lookahead = formatNumber(pathLength, 1)
            vm.lookaheadTarget = formatNumber(pathTarget, 1)
            var coveragePercent = null
            if (typeof pathLength === 'number' && typeof pathTarget === 'number' && isFinite(pathLength) && isFinite(pathTarget) && pathTarget > 0) {
              coveragePercent = Math.min(999, (pathLength / pathTarget) * 100)
            }
            vm.pathCoverage = coveragePercent !== null ? coveragePercent.toFixed(0) + '%' : '—'
            var segmentCount = path.segments
            var segmentLimit = path.segmentLimit
            var segmentGoal = path.segmentGoal
            var segmentCap = path.segmentCap
            var segmentStep = path.segmentStep
            if (typeof segmentCount === 'number' && isFinite(segmentCount)) {
              var segmentLabel = segmentCount.toString()
              if (typeof segmentLimit === 'number' && isFinite(segmentLimit) && segmentLimit > 0) {
                segmentLabel += ' / ' + segmentLimit
              }
              if (typeof segmentGoal === 'number' && isFinite(segmentGoal) && segmentGoal > 0 && (!segmentLimit || segmentGoal !== segmentLimit)) {
                segmentLabel += ' (goal ' + segmentGoal + ')'
              }
              if (typeof segmentCap === 'number' && isFinite(segmentCap) && (!segmentLimit || segmentCap > segmentLimit)) {
                segmentLabel += ' [cap ' + segmentCap + ']'
              }
              if (typeof segmentStep === 'number' && isFinite(segmentStep) && segmentStep > 0 && (!segmentCap || (typeof segmentLimit === 'number' && segmentLimit < segmentCap))) {
                segmentLabel += ' {step ' + segmentStep + '}'
              }
              vm.pathSegments = segmentLabel
            } else {
              vm.pathSegments = '—'
            }
            var truncated = !!path.truncated
            var coverageLimited = coveragePercent !== null && coveragePercent < 100
            vm.pathTruncated = (truncated || coverageLimited) ? 'Yes' : 'No'
            var pointCount = path.center && path.center.length
            vm.pathPoints = pointCount ? pointCount : '—'
            var branches = path.branches || []
            if (branches.length) {
              vm.branchCount = branches.length.toString()
              var left = 0
              var right = 0
              var forward = 0
              for (var b = 0; b < branches.length; b++) {
                var branch = branches[b]
                if (branch && typeof branch.turn === 'number') {
                  if (branch.turn > 0.05) {
                    left++
                  } else if (branch.turn < -0.05) {
                    right++
                  } else {
                    forward++
                  }
                }
              }
              var parts = []
              if (forward) parts.push(forward + ' straight')
              if (left) parts.push(left + ' left')
              if (right) parts.push(right + ' right')
              vm.branchInfo = parts.length ? parts.join(' • ') : branches.length + ' options'
            } else {
              vm.branchCount = '0'
              vm.branchInfo = '—'
            }
            var route = lane.route || path.routePreview
            if (route && route.center && route.center.length) {
              var previewNodes = route.previewNodes || route.center.length
              var totalNodes = route.totalNodes || previewNodes
              var lengthValue = typeof route.length === 'number' && isFinite(route.length) ? route.length : (path.length || 0)
              vm.routeLength = formatNumber(lengthValue, 1)
              if (previewNodes && totalNodes && totalNodes !== previewNodes) {
                vm.routeNodes = previewNodes + ' / ' + totalNodes
              } else if (previewNodes) {
                vm.routeNodes = previewNodes.toString()
              } else {
                vm.routeNodes = '—'
              }
              if (typeof route.startIndex === 'number' && isFinite(route.startIndex) && previewNodes) {
                var routeEnd = route.startIndex + previewNodes - 1
                vm.routeSpan = route.startIndex + ' → ' + routeEnd
              } else {
                vm.routeSpan = '—'
              }
              if (typeof route.seq === 'number' && isFinite(route.seq)) {
                vm.routeUpdated = '#' + route.seq
              } else if (route.updatedAt) {
                vm.routeUpdated = route.updatedAt.toString()
              } else {
                vm.routeUpdated = '—'
              }
            } else {
              vm.routeLength = vm.routeNodes = vm.routeSpan = vm.routeUpdated = '—'
            }
          } else {
            vm.lookahead = vm.lookaheadTarget = '—'
            vm.pathCoverage = vm.pathSegments = vm.pathTruncated = '—'
            vm.pathPoints = '—'
            vm.branchCount = '—'
            vm.branchInfo = '—'
          }
          vm.speed = formatNumber(lane.speed, 1)
          if (typeof lane.curvature === 'number' && isFinite(lane.curvature)) {
            if (Math.abs(lane.curvature) > 1e-4) {
              vm.curvatureRadius = formatNumber(1 / Math.abs(lane.curvature), 1)
            } else {
              vm.curvatureRadius = '∞'
            }
          } else {
            vm.curvatureRadius = '—'
          }

          var trafficSide = null
          if (lane.roadRules) {
            if (typeof lane.roadRules.trafficSide === 'string' && lane.roadRules.trafficSide.length) {
              trafficSide = lane.roadRules.trafficSide
            } else if (typeof lane.roadRules.rightHandDrive === 'boolean') {
              trafficSide = lane.roadRules.rightHandDrive ? 'left' : 'right'
            }
          }
          if (trafficSide) {
            var label = trafficSide.charAt(0).toUpperCase() + trafficSide.slice(1) + '-hand traffic'
            vm.trafficSide = label
          } else {
            vm.trafficSide = '—'
          }
        } else {
          vm.laneWidth = vm.offsetLane = vm.offsetTarget = vm.offsetAbsolute = '—'
          vm.offsetNormalized = vm.offsetLegal = vm.offsetLegalError = vm.offsetLegalNormalized = '—'
          vm.headingError = vm.curvature = vm.lookahead = vm.lookaheadTarget = '—'
          vm.pathCoverage = vm.pathSegments = vm.pathTruncated = vm.speed = '—'
          vm.pathPoints = '—'
          vm.curvatureRadius = '—'
          vm.branchCount = '—'
          vm.branchInfo = '—'
          vm.routeLength = vm.routeNodes = vm.routeSpan = vm.routeUpdated = '—'
          vm.trafficSide = '—'
        }

        if (assist && assist.steering) {
          vm.assistForce = formatNumber(assist.steering.target, 3)
          vm.assistWeight = formatNumber(assist.steering.weight, 2)
          vm.driverInput = formatNumber(assist.steering.driver, 3)
          vm.finalSteer = formatNumber(assist.steering.final, 3)
        } else {
          vm.assistForce = vm.assistWeight = vm.driverInput = vm.finalSteer = '—'
        }
      }

      function drawVehicle(x, y, sizeScale) {
        var scale = sizeScale || 1
        ctx.save()
        ctx.fillStyle = 'rgba(' +
          Math.round(carColor[0]) + ',' +
          Math.round(carColor[1]) + ',' +
          Math.round(carColor[2]) + ',0.6)'
        var carWidth = 14 * scale
        var carLength = 28 * scale
        ctx.translate(x, y)
        ctx.fillRect(-carWidth / 2, -carLength, carWidth, carLength)
        ctx.restore()
      }

      function drawArrow(baseX, baseY, dirX, dirY, length, color, thickness) {
        var magnitude = Math.sqrt(dirX * dirX + dirY * dirY)
        if (!isFinite(magnitude) || magnitude < 1e-5) return
        var normX = dirX / magnitude
        var normY = dirY / magnitude
        var canvasDirX = normX
        var canvasDirY = -normY
        var tipX = baseX + canvasDirX * length
        var tipY = baseY + canvasDirY * length

        ctx.strokeStyle = color
        ctx.lineWidth = thickness || 2
        ctx.beginPath()
        ctx.moveTo(baseX, baseY)
        ctx.lineTo(tipX, tipY)
        ctx.stroke()

        var headSize = Math.min(6, length * 0.35)
        var leftDirX = -canvasDirY
        var leftDirY = canvasDirX
        var leftX = tipX - canvasDirX * headSize + leftDirX * headSize * 0.5
        var leftY = tipY - canvasDirY * headSize + leftDirY * headSize * 0.5
        var rightX = tipX - canvasDirX * headSize - leftDirX * headSize * 0.5
        var rightY = tipY - canvasDirY * headSize - leftDirY * headSize * 0.5

        ctx.fillStyle = color
        ctx.beginPath()
        ctx.moveTo(tipX, tipY)
        ctx.lineTo(leftX, leftY)
        ctx.lineTo(rightX, rightY)
        ctx.closePath()
        ctx.fill()
      }

      function plotPath(points, color, width, carX, carY, scale, startIndex, endIndex, lateralShift) {
        if (!points || points.length < 2) return
        var start = startIndex == null ? 0 : Math.max(0, startIndex)
        var end = endIndex == null ? points.length : Math.min(points.length, endIndex)
        if (end - start < 2) return
        var shift = (typeof lateralShift === 'number' && isFinite(lateralShift)) ? lateralShift : 0
        ctx.strokeStyle = color
        ctx.lineWidth = width
        ctx.beginPath()
        var p = points[start]
        ctx.moveTo(carX + (p.x + shift) * scale, carY - p.y * scale)
        for (var i = start + 1; i < end; i++) {
          p = points[i]
          ctx.lineTo(carX + (p.x + shift) * scale, carY - p.y * scale)
        }
        ctx.stroke()
      }

      function draw(data) {
        var view = syncCanvasSize()
        var width = view.width
        var height = view.height
        if (!width || !height) {
          return
        }

        ctx.clearRect(0, 0, width, height)
        ctx.fillStyle = 'rgba(0, 0, 0, 0.45)'
        ctx.fillRect(0, 0, width, height)

        var carX = width / 2
        var carY = height * 0.78
        var pixelScale = Math.max(0.75, Math.min(width / 320, 1.6))

        var lane = data && data.lane
        var lanePath = lane && lane.path
        var route = lane && (lane.route || (lanePath && lanePath.routePreview))
        var path = lanePath
        if (!path || !path.center || path.center.length < 2) {
          if (route && route.center && route.center.length >= 2) {
            path = {
              center: route.center,
              left: route.left || [],
              right: route.right || [],
              branches: []
            }
          } else {
            drawVehicle(carX, carY, pixelScale)
            return
          }
        }

        var laneInfo = lane && lane.lanes
        var laneOffsets = null
        if (laneInfo && Array.isArray(laneInfo.offsets) && laneInfo.offsets.length) {
          laneOffsets = laneInfo.offsets
        }

        var laneWidthValue = null
        if (laneInfo && typeof laneInfo.width === 'number' && isFinite(laneInfo.width)) {
          laneWidthValue = laneInfo.width
        } else if (lane && typeof lane.width === 'number' && isFinite(lane.width)) {
          laneWidthValue = lane.width
        }
        if (laneInfo && typeof laneInfo.roadWidth === 'number' && isFinite(laneInfo.roadWidth) && laneOffsets && laneOffsets.length) {
          var derivedLaneWidth = laneInfo.roadWidth / laneOffsets.length
          if (derivedLaneWidth > 0 && (!laneWidthValue || !isFinite(laneWidthValue) || laneWidthValue <= 0)) {
            laneWidthValue = derivedLaneWidth
          }
        }

        var laneHalfWidth = (typeof laneWidthValue === 'number' && isFinite(laneWidthValue)) ? laneWidthValue * 0.5 : null
        var laneOffset = (lane && lane.offset) || {}
        var targetOffset = (typeof laneOffset.target === 'number' && isFinite(laneOffset.target)) ? laneOffset.target : null
        var absoluteOffset = (typeof laneOffset.current === 'number' && isFinite(laneOffset.current)) ? laneOffset.current : null
        var laneTargetError = (typeof laneOffset.error === 'number' && isFinite(laneOffset.error)) ? laneOffset.error : null
        var legalOffset = (typeof laneOffset.legal === 'number' && isFinite(laneOffset.legal)) ? laneOffset.legal : null
        var legalError = (typeof laneOffset.legalError === 'number' && isFinite(laneOffset.legalError)) ? laneOffset.legalError : null

        if (laneTargetError === null && absoluteOffset !== null && targetOffset !== null) {
          laneTargetError = absoluteOffset - targetOffset
        }

        if (legalError === null && legalOffset !== null && absoluteOffset !== null) {
          legalError = absoluteOffset - legalOffset
        }

        if (absoluteOffset === null && laneTargetError !== null && targetOffset !== null) {
          absoluteOffset = laneTargetError + targetOffset
        }

        var pathOffset = (lanePath && typeof lanePath.offset === 'number' && isFinite(lanePath.offset)) ? lanePath.offset : null
        var baseLaneOffset = pathOffset
        if (baseLaneOffset === null && laneInfo && typeof laneInfo.selectedOffset === 'number' && isFinite(laneInfo.selectedOffset)) {
          baseLaneOffset = laneInfo.selectedOffset
        }
        if (baseLaneOffset === null && targetOffset !== null) {
          baseLaneOffset = targetOffset
        }
        if (baseLaneOffset === null && legalOffset !== null) {
          baseLaneOffset = legalOffset
        }
        if (baseLaneOffset === null) {
          baseLaneOffset = 0
        }
        var baselineOffset = pathOffset !== null ? pathOffset : baseLaneOffset

        var displayLaneError = laneTargetError
        if (legalError !== null) {
          displayLaneError = legalError
        }
        if (displayLaneError === null) {
          displayLaneError = 0
        }

        var laneGeometry = lanePath && lanePath.center && lanePath.center.length > 1 ? lanePath : null
        var laneCenters = []
        var laneBoundaries = []
        var legalLaneShift = null
        if (typeof legalOffset === 'number' && isFinite(legalOffset)) {
          legalLaneShift = legalOffset - baselineOffset
        } else if (laneInfo && typeof laneInfo.legalOffset === 'number' && isFinite(laneInfo.legalOffset)) {
          legalLaneShift = laneInfo.legalOffset - baselineOffset
        }
        if (laneGeometry && laneOffsets && laneOffsets.length) {
          var boundaryMap = Object.create(null)
          var laneHalfSpacing = laneHalfWidth
          if (!laneHalfSpacing && typeof laneWidthValue === 'number' && isFinite(laneWidthValue)) {
            laneHalfSpacing = laneWidthValue * 0.5
          }
          if ((!laneHalfSpacing || laneHalfSpacing <= 0) && laneInfo && typeof laneInfo.roadWidth === 'number' && isFinite(laneInfo.roadWidth) && laneOffsets.length) {
            laneHalfSpacing = (laneInfo.roadWidth / laneOffsets.length) * 0.5
          }
          for (var li = 0; li < laneOffsets.length; li++) {
            var offsetVal = laneOffsets[li]
            if (typeof offsetVal !== 'number' || !isFinite(offsetVal)) continue
            var centerShift = offsetVal - baselineOffset
            laneCenters.push({ shift: centerShift, offset: offsetVal })
            if (laneHalfSpacing && isFinite(laneHalfSpacing) && laneHalfSpacing > 0) {
              var leftVal = offsetVal - laneHalfSpacing
              var rightVal = offsetVal + laneHalfSpacing
              var leftKey = leftVal.toFixed(4)
              var rightKey = rightVal.toFixed(4)
              var leftEntry = boundaryMap[leftKey]
              if (!leftEntry) {
                leftEntry = { value: leftVal, count: 0 }
                boundaryMap[leftKey] = leftEntry
              }
              leftEntry.count += 1
              var rightEntry = boundaryMap[rightKey]
              if (!rightEntry) {
                rightEntry = { value: rightVal, count: 0 }
                boundaryMap[rightKey] = rightEntry
              }
              rightEntry.count += 1
            }
          }
          if (laneInfo) {
            if (typeof laneInfo.roadLeft === 'number' && isFinite(laneInfo.roadLeft)) {
              var leftEdge = laneInfo.roadLeft - baselineOffset
              var leftEdgeKey = leftEdge.toFixed(4)
              var existingLeft = boundaryMap[leftEdgeKey]
              if (!existingLeft) {
                boundaryMap[leftEdgeKey] = { value: leftEdge, count: laneOffsets.length }
              } else {
                existingLeft.value = leftEdge
                existingLeft.count = Math.max(existingLeft.count, laneOffsets.length)
              }
            }
            if (typeof laneInfo.roadRight === 'number' && isFinite(laneInfo.roadRight)) {
              var rightEdge = laneInfo.roadRight - baselineOffset
              var rightEdgeKey = rightEdge.toFixed(4)
              var existingRight = boundaryMap[rightEdgeKey]
              if (!existingRight) {
                boundaryMap[rightEdgeKey] = { value: rightEdge, count: laneOffsets.length }
              } else {
                existingRight.value = rightEdge
                existingRight.count = Math.max(existingRight.count, laneOffsets.length)
              }
            }
          }
          var boundaryKeys = Object.keys(boundaryMap)
          for (var bk = 0; bk < boundaryKeys.length; bk++) {
            laneBoundaries.push(boundaryMap[boundaryKeys[bk]])
          }
          laneBoundaries.sort(function (a, b) { return a.value - b.value })
        }

        var routeCenter = route && route.center
        var routeLeft = route && route.left
        var routeRight = route && route.right

        var maxForward = 5
        var maxLateral = laneHalfWidth ? laneHalfWidth : (laneWidthValue && isFinite(laneWidthValue) ? laneWidthValue : 5)
        for (var i = 0; i < path.center.length; i++) {
          var pt = path.center[i]
          if (pt.y > maxForward) maxForward = pt.y
          var absX = Math.abs(pt.x)
          if (absX > maxLateral) maxLateral = absX
        }
        if (path.left) {
          for (var li = 0; li < path.left.length; li++) {
            var lp = path.left[li]
            if (lp.y > maxForward) maxForward = lp.y
            var la = Math.abs(lp.x)
            if (la > maxLateral) maxLateral = la
          }
        }
        if (path.right) {
          for (var ri = 0; ri < path.right.length; ri++) {
            var rp = path.right[ri]
            if (rp.y > maxForward) maxForward = rp.y
            var ra = Math.abs(rp.x)
            if (ra > maxLateral) maxLateral = ra
          }
        }
        var branchList = path.branches || []
        for (var bi = 0; bi < branchList.length; bi++) {
          var branchPath = branchList[bi] && branchList[bi].center
          if (!branchPath) continue
          for (var bj = 0; bj < branchPath.length; bj++) {
            var bp = branchPath[bj]
            if (bp.y > maxForward) maxForward = bp.y
            var bAbsX = Math.abs(bp.x)
            if (bAbsX > maxLateral) maxLateral = bAbsX
          }
        }
        if (routeCenter) {
          for (var rc = 0; rc < routeCenter.length; rc++) {
            var rcp = routeCenter[rc]
            if (rcp.y > maxForward) maxForward = rcp.y
            var rcAbs = Math.abs(rcp.x)
            if (rcAbs > maxLateral) maxLateral = rcAbs
          }
        }
        if (routeLeft) {
          for (var rl = 0; rl < routeLeft.length; rl++) {
            var rlp = routeLeft[rl]
            if (rlp.y > maxForward) maxForward = rlp.y
            var rlAbs = Math.abs(rlp.x)
            if (rlAbs > maxLateral) maxLateral = rlAbs
          }
        }
        if (routeRight) {
          for (var rr = 0; rr < routeRight.length; rr++) {
            var rrp = routeRight[rr]
            if (rrp.y > maxForward) maxForward = rrp.y
            var rrAbs = Math.abs(rrp.x)
            if (rrAbs > maxLateral) maxLateral = rrAbs
          }
        }
        if (laneCenters.length) {
          var lateralMargin = laneHalfWidth || (laneWidthValue && isFinite(laneWidthValue) ? laneWidthValue * 0.5 : 0)
          for (var lc = 0; lc < laneCenters.length; lc++) {
            var centerEntry = laneCenters[lc]
            if (!centerEntry) continue
            var reach = Math.abs(centerEntry.shift)
            if (lateralMargin) {
              reach += lateralMargin
            }
            if (reach > maxLateral) {
              maxLateral = reach
            }
          }
        }
        if (laneBoundaries.length) {
          for (var lb = 0; lb < laneBoundaries.length; lb++) {
            var boundaryEntry = laneBoundaries[lb]
            if (!boundaryEntry) continue
            var boundaryReach = Math.abs(boundaryEntry.value - baselineOffset)
            if (boundaryReach > maxLateral) {
              maxLateral = boundaryReach
            }
          }
        }

        var offsetReach = (absoluteOffset !== null ? Math.abs(absoluteOffset - baseLaneOffset) : Math.abs(displayLaneError)) + (laneHalfWidth || 0)
        if (offsetReach > maxLateral) maxLateral = offsetReach
        if (laneHalfWidth && laneHalfWidth * 1.2 > maxLateral) {
          maxLateral = laneHalfWidth * 1.2
        }

        var scaleY = (height * 0.65) / (maxForward + 5)
        var scaleX = (width * 0.4) / (maxLateral + (laneWidthValue || 3))
        var scale = Math.min(scaleX, scaleY)
        if (!isFinite(scale) || scale <= 0) scale = 6

        var laneCenterX = carX
        if (absoluteOffset !== null && baseLaneOffset !== null) {
          laneCenterX = carX - (absoluteOffset - baseLaneOffset) * scale
        } else {
          laneCenterX = carX - displayLaneError * scale
        }

        if (laneHalfWidth && isFinite(laneHalfWidth)) {
          var laneDepth = height * 0.68
          var laneTaper = 0.35
          var laneLeftBottom = laneCenterX - laneHalfWidth * scale
          var laneRightBottom = laneCenterX + laneHalfWidth * scale
          var laneLeftTop = laneCenterX - laneHalfWidth * scale * laneTaper
          var laneRightTop = laneCenterX + laneHalfWidth * scale * laneTaper
          ctx.fillStyle = 'rgba(90, 140, 200, 0.14)'
          ctx.beginPath()
          ctx.moveTo(laneLeftBottom, carY)
          ctx.lineTo(laneRightBottom, carY)
          ctx.lineTo(laneRightTop, carY - laneDepth)
          ctx.lineTo(laneLeftTop, carY - laneDepth)
          ctx.closePath()
          ctx.fill()
          ctx.strokeStyle = 'rgba(160, 210, 255, 0.25)'
          ctx.lineWidth = Math.max(1, 1.1 * pixelScale)
          ctx.beginPath()
          ctx.moveTo(laneLeftBottom, carY)
          ctx.lineTo(laneLeftTop, carY - laneDepth)
          ctx.moveTo(laneRightBottom, carY)
          ctx.lineTo(laneRightTop, carY - laneDepth)
          ctx.stroke()
        }

        if (laneGeometry && laneCenters.length) {
          var neighborWidth = Math.max(1.1, 1.5 * pixelScale)
          for (var lc2 = 0; lc2 < laneCenters.length; lc2++) {
            var laneEntry = laneCenters[lc2]
            if (!laneEntry) continue
            if (Math.abs(laneEntry.shift) < 1e-3) continue
            var laneColor = 'rgba(150, 190, 255, 0.45)'
            if (legalLaneShift !== null && Math.abs(laneEntry.shift - legalLaneShift) < 1e-3) {
              laneColor = 'rgba(120, 220, 255, 0.6)'
            }
            plotPath(laneGeometry.center, laneColor, neighborWidth, carX, carY, scale, null, null, laneEntry.shift)
          }
        }

        if (laneGeometry && laneBoundaries.length) {
          var boundaryLineWidth = Math.max(1, 1.2 * pixelScale)
          var dash = Math.max(3, 4 * pixelScale)
          for (var lb2 = 0; lb2 < laneBoundaries.length; lb2++) {
            var boundary = laneBoundaries[lb2]
            if (!boundary) continue
            var shiftValue = boundary.value - baselineOffset
            ctx.save()
            if (boundary.count > 1) {
              ctx.setLineDash([dash, dash])
            }
            var boundaryColor = boundary.count > 1 ? 'rgba(210, 220, 255, 0.38)' : 'rgba(230, 240, 255, 0.55)'
            plotPath((laneGeometry && laneGeometry.center) || path.center, boundaryColor, boundaryLineWidth, carX, carY, scale, null, null, shiftValue)
            if (boundary.count > 1) {
              ctx.setLineDash([])
            }
            ctx.restore()
          }
        }

        var routeCount = routeCenter && routeCenter.length > 1 ? routeCenter.length : 0
        var sharedCount = 0
        if (routeCount && path.center) {
          sharedCount = Math.min(path.center.length, routeCount)
        }
        if (routeCount) {
          var routeBaseWidth = Math.max(1.2, 1.7 * pixelScale)
          var routeHighlightWidth = Math.max(1.6, 2.1 * pixelScale)
          plotPath(routeCenter, 'rgba(100, 210, 255, 0.55)', routeBaseWidth, carX, carY, scale, 0, routeCount)
          if (sharedCount > 1) {
            plotPath(routeCenter, 'rgba(70, 240, 180, 0.9)', routeHighlightWidth, carX, carY, scale, 0, sharedCount)
          }
          if (routeCount > Math.max(1, sharedCount)) {
            ctx.save()
            var dash = Math.max(3, 4 * pixelScale)
            ctx.setLineDash([dash, dash])
            plotPath(routeCenter, 'rgba(120, 200, 255, 0.65)', Math.max(1.1, 1.6 * pixelScale), carX, carY, scale, Math.max(0, sharedCount - 1), routeCount)
            ctx.setLineDash([])
            ctx.restore()
            var step = Math.max(1, Math.round((routeCount - sharedCount) / 24))
            ctx.fillStyle = 'rgba(200, 220, 255, 0.28)'
            for (var ri2 = sharedCount; ri2 < routeCount; ri2 += step) {
              var markerRoute = routeCenter[ri2]
              ctx.beginPath()
              ctx.arc(carX + markerRoute.x * scale, carY - markerRoute.y * scale, Math.max(1.2, 1.5 * pixelScale), 0, Math.PI * 2)
              ctx.fill()
            }
          }
          var routeTail = routeCenter[routeCount - 1]
          var routePrev = routeCenter[Math.max(0, routeCount - 2)]
          drawArrow(
            carX + routeTail.x * scale,
            carY - routeTail.y * scale,
            routeTail.x - routePrev.x,
            routeTail.y - routePrev.y,
            12 * pixelScale,
            'rgba(120, 200, 255, 0.85)',
            Math.max(1.2, 1.6 * pixelScale)
          )
        }

        var boundaryWidth = Math.max(1.6, 2 * pixelScale)
        if (path.left && path.left.length > 1) {
          plotPath(path.left, 'rgba(255, 255, 0, 0.75)', boundaryWidth, carX, carY, scale)
        }
        if (path.right && path.right.length > 1) {
          plotPath(path.right, 'rgba(255, 255, 0, 0.75)', boundaryWidth, carX, carY, scale)
        }
        var centerColor = vm.warning ? 'rgba(255, 120, 120, 0.9)' : 'rgba(60, 220, 120, 0.92)'
        plotPath(path.center, centerColor, Math.max(1.8, 2.2 * pixelScale), carX, carY, scale)

        if (branchList.length) {
          ctx.save()
          var dash = Math.max(3, 4 * pixelScale)
          ctx.setLineDash([dash, dash])
          for (var bi2 = 0; bi2 < branchList.length; bi2++) {
            var branch = branchList[bi2]
            if (!branch || !branch.center || branch.center.length < 2) continue
            var hueColor = 'rgba(160, 180, 255, 0.6)'
            if (typeof branch.turn === 'number') {
              if (branch.turn > 0.05) {
                hueColor = 'rgba(255, 200, 120, 0.7)'
              } else if (branch.turn < -0.05) {
                hueColor = 'rgba(120, 200, 255, 0.7)'
              }
            }
            plotPath(branch.center, hueColor, Math.max(1.1, 1.5 * pixelScale), carX, carY, scale)
          }
          ctx.setLineDash([])
          ctx.restore()
        }

        if (path.center && path.center.length > 1) {
          ctx.fillStyle = 'rgba(180, 220, 255, 0.28)'
          var markerRadius = Math.max(1.2, 1.5 * pixelScale)
          for (var j = 0; j < path.center.length; j += 2) {
            var marker = path.center[j]
            ctx.beginPath()
            ctx.arc(carX + marker.x * scale, carY - marker.y * scale, markerRadius, 0, Math.PI * 2)
            ctx.fill()
          }

          var tail = path.center[path.center.length - 1]
          var tailPrev = path.center[path.center.length - 2]
          drawArrow(
            carX + tail.x * scale,
            carY - tail.y * scale,
            tail.x - tailPrev.x,
            tail.y - tailPrev.y,
            14 * pixelScale,
            'rgba(140, 200, 255, 0.85)',
            Math.max(1.5, 2 * pixelScale)
          )
        }

        var assist = data && data.assist && data.assist.steering
        var hasLaneOffset = false
        if ((laneOffset && typeof laneOffset.error === 'number' && isFinite(laneOffset.error)) || legalError !== null) {
          hasLaneOffset = true
        } else if (absoluteOffset !== null && targetOffset !== null) {
          hasLaneOffset = true
        }

        if (hasLaneOffset) {
          var indicatorError = laneTargetError
          if (indicatorError === null && absoluteOffset !== null && targetOffset !== null) {
            indicatorError = absoluteOffset - targetOffset
          }
          if (indicatorError === null) {
            indicatorError = displayLaneError
          }
          var targetX = -indicatorError * scale
          ctx.strokeStyle = 'rgba(255, 90, 90, 0.9)'
          ctx.lineWidth = Math.max(1.6, 2 * pixelScale)
          ctx.beginPath()
          ctx.moveTo(carX, carY)
          ctx.lineTo(carX + targetX, carY)
          ctx.stroke()

          ctx.fillStyle = 'rgba(120, 160, 255, 0.9)'
          ctx.beginPath()
          ctx.arc(carX - indicatorError * scale, carY, Math.max(3, 4 * pixelScale), 0, Math.PI * 2)
          ctx.fill()

          if (targetOffset !== null) {
            ctx.strokeStyle = 'rgba(200, 200, 255, 0.6)'
            ctx.lineWidth = Math.max(1, 1.2 * pixelScale)
            ctx.beginPath()
            ctx.moveTo(carX - targetOffset * scale, carY)
            ctx.lineTo(carX - targetOffset * scale, carY - 12 * pixelScale)
            ctx.stroke()
          }
        }

        if (assist && typeof assist.target === 'number' && typeof assist.weight === 'number') {
          var vector = assist.target * assist.weight
          ctx.strokeStyle = 'rgba(0, 180, 255, 0.9)'
          ctx.lineWidth = Math.max(2, 3 * pixelScale)
          ctx.beginPath()
          ctx.moveTo(carX, carY - 6 * pixelScale)
          ctx.lineTo(carX + vector * scale * 10, carY - 26 * pixelScale)
          ctx.stroke()
        }

        drawVehicle(carX, carY, pixelScale)
      }

      if (typeof window !== 'undefined') {
        if (window.ResizeObserver) {
          resizeObserver = new ResizeObserver(function () {
            scheduleResize()
          })
          resizeObserver.observe(containerEl)
        } else {
          windowResizeHandler = function () {
            scheduleResize()
          }
          window.addEventListener('resize', windowResizeHandler)
        }
      }
      scheduleResize()

      function update() {
        bngApi.engineLua('extensions.driver_assistance_angelo234.getLaneCenteringData()', function (data) {
          $scope.$evalAsync(function () {
            if (data && data.color) {
              carColor = [
                typeof data.color.r === 'number' ? data.color.r : 255,
                typeof data.color.g === 'number' ? data.color.g : 255,
                typeof data.color.b === 'number' ? data.color.b : 255
              ]
            }
            updateState(data)
            draw(data)
          })
        })
      }

      var interval = setInterval(update, 100)
      $scope.$on('$destroy', function () {
        clearInterval(interval)
        if (resizeObserver) {
          resizeObserver.disconnect()
          resizeObserver = null
        }
        if (typeof window !== 'undefined' && windowResizeHandler) {
          window.removeEventListener('resize', windowResizeHandler)
          windowResizeHandler = null
        }
      })
    }]
  }
}])
