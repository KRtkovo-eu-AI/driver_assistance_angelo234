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
        speed: '—',
        assistForce: '—',
        assistWeight: '—',
        driverInput: '—',
        finalSteer: '—',
        pathPoints: '—',
        curvatureRadius: '—',
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
          vm.lookahead = formatNumber((lane.path && lane.path.length) || 0, 1)
          vm.speed = formatNumber(lane.speed, 1)
          var pointCount = lane.path && lane.path.center && lane.path.center.length
          vm.pathPoints = pointCount ? pointCount : '—'
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
          vm.offsetNormalized = vm.headingError = vm.curvature = vm.lookahead = vm.speed = '—'
          vm.pathPoints = '—'
          vm.curvatureRadius = '—'
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

      function drawVehicle(x, y) {
        ctx.save()
        ctx.fillStyle = 'rgba(' +
          Math.round(carColor[0]) + ',' +
          Math.round(carColor[1]) + ',' +
          Math.round(carColor[2]) + ',0.6)'
        var carWidth = 14
        var carLength = 28
        ctx.translate(x, y)
        ctx.fillRect(-carWidth / 2, -carLength, carWidth, carLength)
        ctx.restore()
      }

      function drawArrow(baseX, baseY, dirX, dirY, length, color) {
        var magnitude = Math.sqrt(dirX * dirX + dirY * dirY)
        if (!isFinite(magnitude) || magnitude < 1e-5) return
        var normX = dirX / magnitude
        var normY = dirY / magnitude
        var canvasDirX = normX
        var canvasDirY = -normY
        var tipX = baseX + canvasDirX * length
        var tipY = baseY + canvasDirY * length

        ctx.strokeStyle = color
        ctx.lineWidth = 2
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
        ctx.clearRect(0, 0, canvas.width, canvas.height)
        ctx.fillStyle = 'rgba(0, 0, 0, 0.45)'
        ctx.fillRect(0, 0, canvas.width, canvas.height)

        var carX = canvas.width / 2
        var carY = canvas.height * 0.78

        var lane = data && data.lane
        var path = lane && lane.path
        if (!path || !path.center || path.center.length < 2) {
          drawVehicle(carX, carY)
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

        var scaleY = (canvas.height * 0.65) / (maxForward + 5)
        var scaleX = (canvas.width * 0.4) / (maxLateral + (lane.width || 3))
        var scale = Math.min(scaleX, scaleY)
        if (!isFinite(scale) || scale <= 0) scale = 6

        plotPath(path.left, 'rgba(255, 255, 0, 0.8)', 2, carX, carY, scale)
        plotPath(path.right, 'rgba(255, 255, 0, 0.8)', 2, carX, carY, scale)
        var centerColor = vm.warning ? 'rgba(255, 120, 120, 0.9)' : 'rgba(60, 220, 120, 0.9)'
        plotPath(path.center, centerColor, 2, carX, carY, scale)

        if (path.center && path.center.length > 1) {
          ctx.fillStyle = 'rgba(180, 220, 255, 0.28)'
          for (var i = 0; i < path.center.length; i += 2) {
            var pt = path.center[i]
            ctx.beginPath()
            ctx.arc(carX + pt.x * scale, carY - pt.y * scale, 1.5, 0, Math.PI * 2)
            ctx.fill()
          }

          var tail = path.center[path.center.length - 1]
          var tailPrev = path.center[path.center.length - 2]
          drawArrow(
            carX + tail.x * scale,
            carY - tail.y * scale,
            tail.x - tailPrev.x,
            tail.y - tailPrev.y,
            14,
            'rgba(140, 200, 255, 0.85)'
          )
        }

        var laneOffset = lane.offset || {}
        var assist = data && data.assist && data.assist.steering

        if (laneOffset && typeof laneOffset.error === 'number') {
          var targetX = -laneOffset.error * scale
          ctx.strokeStyle = 'rgba(255, 90, 90, 0.9)'
          ctx.lineWidth = 2
          ctx.beginPath()
          ctx.moveTo(carX, carY)
          ctx.lineTo(carX + targetX, carY)
          ctx.stroke()

          ctx.fillStyle = 'rgba(120, 160, 255, 0.9)'
          ctx.beginPath()
          ctx.arc(carX - laneOffset.error * scale, carY, 4, 0, Math.PI * 2)
          ctx.fill()

          if (typeof laneOffset.target === 'number') {
            ctx.strokeStyle = 'rgba(200, 200, 255, 0.6)'
            ctx.lineWidth = 1.5
            ctx.beginPath()
            ctx.moveTo(carX - laneOffset.target * scale, carY)
            ctx.lineTo(carX - laneOffset.target * scale, carY - 12)
            ctx.stroke()
          }
        }

        if (assist && typeof assist.target === 'number' && typeof assist.weight === 'number') {
          var vector = assist.target * assist.weight
          ctx.strokeStyle = 'rgba(0, 180, 255, 0.9)'
          ctx.lineWidth = 3
          ctx.beginPath()
          ctx.moveTo(carX, carY - 6)
          ctx.lineTo(carX + vector * scale * 10, carY - 26)
          ctx.stroke()
        }

        drawVehicle(carX, carY)
      }

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
      })
    }]
  }
}])
