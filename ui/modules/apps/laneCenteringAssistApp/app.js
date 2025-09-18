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
      var statusEl = $element[0].querySelector('.status-text')
      var offsetEl = $element[0].querySelector('.metric-offset')
      var headingEl = $element[0].querySelector('.metric-heading')

      var carColor = [255, 255, 255]

      function rgba(r, g, b, a) {
        return 'rgba(' + Math.round(r) + ',' + Math.round(g) + ',' + Math.round(b) + ',' + (a || 1) + ')'
      }

      function drawBackground() {
        var gradient = ctx.createLinearGradient(0, 0, 0, canvas.height)
        gradient.addColorStop(0, 'rgba(10, 26, 44, 0.85)')
        gradient.addColorStop(0.5, 'rgba(5, 14, 24, 0.95)')
        gradient.addColorStop(1, 'rgba(2, 6, 12, 0.98)')
        ctx.fillStyle = gradient
        ctx.fillRect(0, 0, canvas.width, canvas.height)

        ctx.save()
        ctx.strokeStyle = 'rgba(52, 102, 160, 0.12)'
        ctx.lineWidth = 1
        var spacingY = canvas.height / 6
        for (var y = canvas.height; y >= 0; y -= spacingY) {
          ctx.beginPath()
          ctx.moveTo(0, y)
          ctx.lineTo(canvas.width, y)
          ctx.stroke()
        }
        ctx.beginPath()
        ctx.moveTo(canvas.width / 2, 0)
        ctx.lineTo(canvas.width / 2, canvas.height)
        ctx.stroke()
        ctx.restore()
      }

      function toCanvas(pt, scaleX, scaleY) {
        return [
          canvas.width / 2 + pt.x * scaleX,
          canvas.height - pt.y * scaleY
        ]
      }

      function drawPolyline(points, color, width, scaleX, scaleY) {
        if (!points || points.length < 2) return
        ctx.save()
        ctx.beginPath()
        var first = toCanvas(points[0], scaleX, scaleY)
        ctx.moveTo(first[0], first[1])
        for (var i = 1; i < points.length; i++) {
          var c = toCanvas(points[i], scaleX, scaleY)
          ctx.lineTo(c[0], c[1])
        }
        ctx.lineWidth = width
        ctx.lineCap = 'round'
        ctx.lineJoin = 'round'
        ctx.strokeStyle = color
        ctx.stroke()
        ctx.restore()
      }

      function drawProjection(projection, scaleX, scaleY) {
        if (!projection) return
        var carPos = toCanvas({ x: 0, y: 0 }, scaleX, scaleY)
        var proj = toCanvas(projection, scaleX, scaleY)
        ctx.save()
        ctx.strokeStyle = 'rgba(255, 125, 95, 0.9)'
        ctx.setLineDash([6, 6])
        ctx.lineWidth = 2
        ctx.beginPath()
        ctx.moveTo(carPos[0], carPos[1])
        ctx.lineTo(proj[0], proj[1])
        ctx.stroke()
        ctx.restore()
      }

      function drawTarget(target, scaleX, scaleY) {
        if (!target) return
        var c = toCanvas(target, scaleX, scaleY)
        ctx.save()
        ctx.fillStyle = 'rgba(255, 214, 102, 0.95)'
        ctx.strokeStyle = 'rgba(0, 0, 0, 0.2)'
        ctx.beginPath()
        ctx.arc(c[0], c[1], 5, 0, Math.PI * 2)
        ctx.fill()
        ctx.lineWidth = 1.5
        ctx.stroke()
        ctx.restore()
      }

      function drawVehicle() {
        ctx.save()
        var carWidth = canvas.width * 0.2
        var carLength = canvas.height * 0.28
        ctx.translate(canvas.width / 2, canvas.height - carLength * 0.35)
        var body = ctx.createLinearGradient(0, -carLength, 0, carLength * 0.2)
        body.addColorStop(0, rgba(carColor[0], carColor[1], carColor[2], 0.85))
        body.addColorStop(1, rgba(carColor[0], carColor[1], carColor[2], 0.35))
        ctx.fillStyle = body
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.25)'
        ctx.lineWidth = 2
        ctx.beginPath()
        ctx.moveTo(-carWidth * 0.55, 0)
        ctx.lineTo(-carWidth * 0.38, -carLength)
        ctx.lineTo(carWidth * 0.38, -carLength)
        ctx.lineTo(carWidth * 0.55, 0)
        ctx.closePath()
        ctx.fill()
        ctx.stroke()
        ctx.restore()
      }

      function getStatusInfo(data) {
        var status = data && data.status ? String(data.status).toUpperCase() : 'OFF'
        var info = { text: status, className: 'inactive' }
        switch (status) {
          case 'ACTIVE':
            info.className = 'active'
            info.text = 'ACTIVE'
            break
          case 'SEARCHING':
            info.className = 'searching'
            info.text = 'SEARCHING'
            break
          case 'ARMING':
            info.className = 'searching'
            info.text = 'STANDBY'
            break
          case 'OVERRIDDEN':
            info.className = 'override'
            info.text = 'MANUAL'
            break
          case 'OFF':
          default:
            info.className = 'inactive'
            info.text = 'OFF'
            break
        }
        return info
      }

      function updateOverlay(data) {
        var info = getStatusInfo(data)
        if (statusEl) {
          statusEl.textContent = info.text
          statusEl.className = 'status-text ' + info.className
        }

        if (offsetEl) {
          if (data && typeof data.lateral_offset === 'number' && info.className !== 'inactive') {
            offsetEl.textContent = 'Offset: ' + data.lateral_offset.toFixed(2) + ' m'
          } else {
            offsetEl.textContent = 'Offset: --'
          }
        }

        if (headingEl) {
          if (data && typeof data.heading_error === 'number' && info.className !== 'inactive') {
            var heading = data.heading_error * 180 / Math.PI
            headingEl.textContent = 'Heading: ' + heading.toFixed(1) + '°'
          } else {
            headingEl.textContent = 'Heading: --'
          }
        }
      }

      function draw(data) {
        drawBackground()
        var view = (data && data.view) || {}
        var forwardRange = Math.max(12, view.forward || 40)
        var lateralRange = Math.max(3, view.lateral || 6)
        var scaleY = canvas.height / forwardRange
        var scaleX = (canvas.width / 2) / lateralRange

        if (data && data.path && data.path.length >= 2) {
          drawPolyline(data.path_left, 'rgba(43, 197, 244, 0.45)', 3, scaleX, scaleY)
          drawPolyline(data.path_right, 'rgba(43, 197, 244, 0.45)', 3, scaleX, scaleY)
          drawPolyline(data.path, 'rgba(247, 201, 72, 0.9)', 4, scaleX, scaleY)
          drawProjection(data.projection, scaleX, scaleY)
          drawTarget(data.target_point, scaleX, scaleY)
        }

        drawVehicle()
      }

      function refresh(data) {
        if (data && data.color) {
          carColor = [
            typeof data.color.r === 'number' ? data.color.r : 255,
            typeof data.color.g === 'number' ? data.color.g : 255,
            typeof data.color.b === 'number' ? data.color.b : 255
          ]
        }
        updateOverlay(data || {})
        draw(data || {})
      }

      function update() {
        bngApi.engineLua('extensions.driver_assistance_angelo234.getLaneCenteringData()', function (data) {
          $scope.$evalAsync(function () {
            refresh(data)
          })
        })
      }

      var interval = setInterval(update, 120)
      $scope.$on('$destroy', function () {
        clearInterval(interval)
      })
    }]
  }
}])
