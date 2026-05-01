import { useRef, useEffect, useCallback } from 'react'
import { cellColor, robotColor, COLOR_OBSTACLE } from '../utils/colors'

const CANVAS_SIZE = 560   // px — the rendered square

/**
 * Renders the live occupancy + coverage map and robot positions.
 *
 * Props
 * -----
 * baseMap        : OccupancyGrid message (the static /map)
 * coverageMap    : OccupancyGrid message (live /coverage_map)
 * robotPoses     : { [id]: PoseStamped }
 * robotStatuses  : { [id]: string }
 * numRobots      : number
 */
export default function MapCanvas({ baseMap, coverageMap, robotPoses, robotStatuses, numRobots }) {
  const canvasRef = useRef(null)

  const draw = useCallback(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const ctx = canvas.getContext('2d')
    const map  = coverageMap || baseMap
    if (!map) {
      // Draw placeholder
      ctx.fillStyle = '#111827'
      ctx.fillRect(0, 0, CANVAS_SIZE, CANVAS_SIZE)
      ctx.fillStyle = '#374151'
      ctx.font = '16px Inter'
      ctx.textAlign = 'center'
      ctx.fillText('Waiting for map…', CANVAS_SIZE / 2, CANVAS_SIZE / 2)
      return
    }

    const { width, height, data } = map
    const scale = CANVAS_SIZE / Math.max(width, height)
    const canvasW = Math.round(width  * scale)
    const canvasH = Math.round(height * scale)

    // ── Draw grid via ImageData for performance ──────────────────────────────
    const imageData = ctx.createImageData(canvasW, canvasH)
    const buf       = imageData.data

    for (let row = 0; row < height; row++) {
      for (let col = 0; col < width; col++) {
        const cellVal = data[row * width + col]
        const [r, g, b] = cellColor(cellVal)

        // Flip Y — ROS row 0 is at bottom, canvas row 0 is at top
        const canvasRow = height - 1 - row
        const px0 = Math.round(col  * scale)
        const py0 = Math.round(canvasRow * scale)
        const px1 = Math.round((col  + 1) * scale)
        const py1 = Math.round((canvasRow + 1) * scale)

        for (let py = py0; py < py1; py++) {
          for (let px = px0; px < px1; px++) {
            const i = (py * canvasW + px) * 4
            buf[i]     = r
            buf[i + 1] = g
            buf[i + 2] = b
            buf[i + 3] = 255
          }
        }
      }
    }
    ctx.putImageData(imageData, 0, 0)

    // ── Draw grid lines (subtle) ─────────────────────────────────────────────
    if (scale >= 3) {
      ctx.strokeStyle = 'rgba(255,255,255,0.03)'
      ctx.lineWidth   = 0.5
      for (let col = 0; col <= width; col++) {
        const x = Math.round(col * scale)
        ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, canvasH); ctx.stroke()
      }
      for (let row = 0; row <= height; row++) {
        const y = Math.round(row * scale)
        ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(canvasW, y); ctx.stroke()
      }
    }

    // ── Draw robots ──────────────────────────────────────────────────────────
    const info = baseMap?.info || map.info
    if (!info) return

    for (let id = 0; id < numRobots; id++) {
      const pose   = robotPoses[id]
      const status = robotStatuses[id] || 'idle'
      if (!pose) continue

      const wx = pose.pose.position.x
      const wy = pose.pose.position.y
      const ox = info.origin.position.x
      const oy = info.origin.position.y
      const res = info.resolution

      const col    = (wx - ox) / res
      const row    = (wy - oy) / res
      const cx     = col * scale
      const cy     = (height - row) * scale   // flip Y
      const radius = Math.max(5, scale * 1.5)

      const color  = robotColor(id)
      const failed = status === 'failed'

      // Glow ring
      const grd = ctx.createRadialGradient(cx, cy, radius * 0.3, cx, cy, radius * 2)
      grd.addColorStop(0, `${failed ? '#ef4444' : color.hex}55`)
      grd.addColorStop(1, 'transparent')
      ctx.beginPath()
      ctx.arc(cx, cy, radius * 2, 0, Math.PI * 2)
      ctx.fillStyle = grd
      ctx.fill()

      // Robot body
      ctx.beginPath()
      ctx.arc(cx, cy, radius, 0, Math.PI * 2)
      ctx.fillStyle   = failed ? '#ef4444' : color.hex
      ctx.strokeStyle = '#fff'
      ctx.lineWidth   = 1.5
      ctx.fill()
      ctx.stroke()

      // Direction arrow using quaternion yaw
      const q  = pose.pose.orientation
      const yaw = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
      const ax = cx + Math.cos(-yaw) * radius * 1.6  // canvas Y is flipped
      const ay = cy + Math.sin(-yaw) * radius * 1.6
      ctx.beginPath()
      ctx.moveTo(cx, cy)
      ctx.lineTo(ax, ay)
      ctx.strokeStyle = '#fff'
      ctx.lineWidth   = 2
      ctx.stroke()

      // Robot ID label
      ctx.fillStyle  = '#fff'
      ctx.font       = `bold ${Math.max(10, radius)}px Inter`
      ctx.textAlign  = 'center'
      ctx.textBaseline = 'middle'
      ctx.fillText(`R${id}`, cx, cy - radius * 2)
    }
  }, [baseMap, coverageMap, robotPoses, robotStatuses, numRobots])

  useEffect(() => { draw() }, [draw])

  return (
    <canvas
      ref={canvasRef}
      width={CANVAS_SIZE}
      height={CANVAS_SIZE}
      className="rounded-xl border border-surface-600"
      style={{ width: CANVAS_SIZE, height: CANVAS_SIZE }}
    />
  )
}
