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
        offsetCurrent: '—',
        offsetTarget: '—',
        offsetError: '—',
        offsetNormalized: '—',
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
          statusClass = 'lca-status-idle'
          statusDetail = 'Disabled'
        } else if (status.driverOverride || status.reason === 'driver_override') {
          statusText = 'Standby'
          statusClass = 'lca-status-warning'
          statusDetail = 'Driver override'
        } else if (status.reason === 'cooldown') {
          statusText = 'Standby'
          statusClass = 'lca-status-warning'
          statusDetail = 'Cooldown'
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
          vm.offsetCurrent = formatNumber(lane.offset.current, 2)
          vm.offsetTarget = formatNumber(lane.offset.target, 2)
          vm.offsetError = formatNumber(lane.offset.error, 2)
          vm.offsetNormalized = formatNumber(lane.offset.normalized, 2)
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
        } else {
          vm.laneWidth = vm.offsetCurrent = vm.offsetTarget = vm.offsetError = '—'
          vm.offsetNormalized = vm.headingError = vm.curvature = vm.lookahead = vm.lookaheadTarget = '—'
          vm.pathCoverage = vm.pathSegments = vm.pathTruncated = vm.speed = '—'
          vm.pathPoints = '—'
          vm.curvatureRadius = '—'
          vm.branchCount = '—'
          vm.branchInfo = '—'
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

      function plotPath(points, color, width, carX, carY, scale) {
        if (!points || points.length < 2) return
        ctx.strokeStyle = color
        ctx.lineWidth = width
        ctx.beginPath()
        var p = points[0]
        ctx.moveTo(carX + p.x * scale, carY - p.y * scale)
        for (var i = 1; i < points.length; i++) {
          p = points[i]
          ctx.lineTo(carX + p.x * scale, carY - p.y * scale)
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
        var path = lane && lane.path
        if (!path || !path.center || path.center.length < 2) {
          drawVehicle(carX, carY, pixelScale)
          return
        }

        var maxForward = 5
        var maxLateral = lane.width || 5
        for (var i = 0; i < path.center.length; i++) {
          var pt = path.center[i]
          if (pt.y > maxForward) maxForward = pt.y
          var absX = Math.abs(pt.x)
          if (absX > maxLateral) maxLateral = absX
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

        var scaleY = (height * 0.65) / (maxForward + 5)
        var scaleX = (width * 0.4) / (maxLateral + (lane.width || 3))
        var scale = Math.min(scaleX, scaleY)
        if (!isFinite(scale) || scale <= 0) scale = 6

        var boundaryWidth = Math.max(1.6, 2 * pixelScale)
        plotPath(path.left, 'rgba(255, 255, 0, 0.8)', boundaryWidth, carX, carY, scale)
        plotPath(path.right, 'rgba(255, 255, 0, 0.8)', boundaryWidth, carX, carY, scale)
        var centerColor = vm.warning ? 'rgba(255, 120, 120, 0.9)' : 'rgba(60, 220, 120, 0.9)'
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

        var laneOffset = lane.offset || {}
        var assist = data && data.assist && data.assist.steering

        if (laneOffset && typeof laneOffset.error === 'number') {
          var targetX = -laneOffset.error * scale
          ctx.strokeStyle = 'rgba(255, 90, 90, 0.9)'
          ctx.lineWidth = Math.max(1.6, 2 * pixelScale)
          ctx.beginPath()
          ctx.moveTo(carX, carY)
          ctx.lineTo(carX + targetX, carY)
          ctx.stroke()

          ctx.fillStyle = 'rgba(120, 160, 255, 0.9)'
          ctx.beginPath()
          ctx.arc(carX - laneOffset.error * scale, carY, Math.max(3, 4 * pixelScale), 0, Math.PI * 2)
          ctx.fill()

          if (typeof laneOffset.target === 'number') {
            ctx.strokeStyle = 'rgba(200, 200, 255, 0.6)'
            ctx.lineWidth = Math.max(1, 1.2 * pixelScale)
            ctx.beginPath()
            ctx.moveTo(carX - laneOffset.target * scale, carY)
            ctx.lineTo(carX - laneOffset.target * scale, carY - 12 * pixelScale)
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
