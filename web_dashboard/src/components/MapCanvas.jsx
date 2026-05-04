import { useRef, useEffect } from 'react'
import { cellColor, robotColor } from '../utils/colors'

const CANVAS_SIZE  = 560
const TRAIL_LENGTH = 300
const LERP_FACTOR  = 0.18

/**
 * Live coverage map renderer.
 *
 * Performance design:
 *   - The map itself only changes a few times per second (rosbridge publishes
 *     /coverage_map at ~2 Hz).  We render it ONCE per update to an offscreen
 *     canvas, then blit that cached image every animation frame.
 *   - The robot/path/trail layer is redrawn every frame at 60 fps, but only
 *     ~3 robots and a few hundred trail points — cheap.
 *   - Pose interpolation (via lerp toward latest target) is internal to this
 *     component, so React never re-renders on each pose tick.
 */
export default function MapCanvas({
  baseMap, coverageMap, redundancyMap,
  robotPoses, robotStatuses,
  robotPaths, robotTrails,
  numRobots,
  overlays,
  sensorRadius = 0.5,
}) {
  const canvasRef     = useRef(null)
  const offscreenRef  = useRef(null)            // cached map image
  const targetsRef    = useRef({})              // pose targets (lerp dest)
  const currentRef    = useRef({})              // interpolated poses
  const propsRef      = useRef({})              // latest props for draw loop
  const lastMapKeyRef = useRef('')              // detect map data changes

  // Mirror props into a ref so the draw loop always sees the latest values
  propsRef.current = {
    baseMap, coverageMap, redundancyMap,
    robotStatuses, robotPaths, robotTrails,
    numRobots, overlays, sensorRadius,
  }

  // Update interpolation targets when new poses arrive
  useEffect(() => {
    for (const id in robotPoses) {
      const p = robotPoses[id]?.pose
      if (!p) continue
      const yaw = yawFromQuat(p.orientation)
      targetsRef.current[id] = { x: p.position.x, y: p.position.y, yaw }
      if (!currentRef.current[id]) {
        currentRef.current[id] = { ...targetsRef.current[id] }
      }
    }
  }, [robotPoses])

  // Re-render the offscreen map cache whenever the map data changes
  useEffect(() => {
    const map = coverageMap || baseMap
    if (!map?.info || !map?.data) return

    const key = mapCacheKey(map, redundancyMap, overlays?.heatmap, overlays?.grid)
    if (key === lastMapKeyRef.current) return
    lastMapKeyRef.current = key

    if (!offscreenRef.current) {
      offscreenRef.current = document.createElement('canvas')
      offscreenRef.current.width  = CANVAS_SIZE
      offscreenRef.current.height = CANVAS_SIZE
    }
    renderMapToCache(offscreenRef.current, map, redundancyMap, overlays)
  }, [coverageMap, baseMap, redundancyMap, overlays?.heatmap, overlays?.grid])

  // Single 60 fps draw loop
  useEffect(() => {
    let raf
    const tick = () => {
      // Lerp current → target
      for (const id in targetsRef.current) {
        const t = targetsRef.current[id]
        const c = currentRef.current[id]
        if (!c) continue
        c.x += (t.x - c.x) * LERP_FACTOR
        c.y += (t.y - c.y) * LERP_FACTOR
        let dy = t.yaw - c.yaw
        while (dy >  Math.PI) dy -= 2 * Math.PI
        while (dy < -Math.PI) dy += 2 * Math.PI
        c.yaw += dy * LERP_FACTOR
      }
      drawFrame(canvasRef.current, offscreenRef.current, propsRef.current, currentRef.current)
      raf = requestAnimationFrame(tick)
    }
    raf = requestAnimationFrame(tick)
    return () => cancelAnimationFrame(raf)
  }, [])

  return (
    <canvas
      ref={canvasRef}
      width={CANVAS_SIZE}
      height={CANVAS_SIZE}
      className="rounded-xl border border-surface-600 shadow-2xl"
      style={{
        width: CANVAS_SIZE,
        height: CANVAS_SIZE,
        background: 'linear-gradient(180deg,#0b0f1a 0%,#070a13 100%)',
      }}
    />
  )
}

// ──────────────────────────────────────────────────────────────────────────
// Cache key — changes only when the map content actually changes
// ──────────────────────────────────────────────────────────────────────────
function mapCacheKey(map, red, heatmap, grid) {
  const stamp = map?.header?.stamp
  const t = stamp ? `${stamp.sec}.${stamp.nanosec}` : 'no-t'
  const w = map?.info?.width ?? 0
  const h = map?.info?.height ?? 0
  const rt = red?.header?.stamp
  const rstamp = rt ? `${rt.sec}.${rt.nanosec}` : 'no-r'
  return `${t}|${w}x${h}|hm=${heatmap ? '1' : '0'}|gr=${grid ? '1' : '0'}|rs=${heatmap ? rstamp : '-'}`
}

// ──────────────────────────────────────────────────────────────────────────
// Render the static map into an offscreen canvas (called only on changes)
// ──────────────────────────────────────────────────────────────────────────
function renderMapToCache(canvas, map, redundancyMap, overlays) {
  const ctx = canvas.getContext('2d')
  const width  = map.info.width
  const height = map.info.height
  if (!width || !height) return

  const data = map.data
  const dataArr = Array.isArray(data) ? data : Array.from(data)
  const scale   = CANVAS_SIZE / Math.max(width, height)
  const canvasW = Math.round(width  * scale)
  const canvasH = Math.round(height * scale)

  ctx.fillStyle = '#0b0f1a'
  ctx.fillRect(0, 0, CANVAS_SIZE, CANVAS_SIZE)

  const showHeatmap = overlays?.heatmap && redundancyMap?.data && redundancyMap?.info?.width === width
  const redArr = showHeatmap
    ? (Array.isArray(redundancyMap.data) ? redundancyMap.data : Array.from(redundancyMap.data))
    : null

  const imageData = ctx.createImageData(canvasW, canvasH)
  const buf       = imageData.data

  for (let row = 0; row < height; row++) {
    for (let col = 0; col < width; col++) {
      const idx = row * width + col
      const cellVal = dataArr[idx]
      let r, g, b
      if (showHeatmap) {
        ;[r, g, b] = heatmapColor(redArr[idx], cellVal)
      } else {
        ;[r, g, b] = cellColor(cellVal)
      }
      const canvasRow = height - 1 - row
      const px0 = Math.round(col       * scale)
      const py0 = Math.round(canvasRow * scale)
      const px1 = Math.round((col + 1) * scale)
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

  // Vignette
  const vignette = ctx.createRadialGradient(
    canvasW / 2, canvasH / 2, canvasW * 0.4,
    canvasW / 2, canvasH / 2, canvasW * 0.7
  )
  vignette.addColorStop(0, 'rgba(0,0,0,0)')
  vignette.addColorStop(1, 'rgba(0,0,0,0.35)')
  ctx.fillStyle = vignette
  ctx.fillRect(0, 0, canvasW, canvasH)

  // Optional grid lines
  if (overlays?.grid && scale >= 2) {
    ctx.strokeStyle = 'rgba(148,163,184,0.06)'
    ctx.lineWidth   = 0.5
    for (let col = 0; col <= width; col += 2) {
      const x = Math.round(col * scale)
      ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, canvasH); ctx.stroke()
    }
    for (let row = 0; row <= height; row += 2) {
      const y = Math.round(row * scale)
      ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(canvasW, y); ctx.stroke()
    }
  }
}

// ──────────────────────────────────────────────────────────────────────────
// Per-frame: blit cached map, then draw robots/paths/trails
// ──────────────────────────────────────────────────────────────────────────
function drawFrame(canvas, offscreen, props, robotsLerp) {
  if (!canvas) return
  const ctx = canvas.getContext('2d')
  const {
    baseMap, coverageMap,
    robotStatuses, robotPaths, robotTrails,
    numRobots, overlays, sensorRadius,
  } = props

  // Background blit (cached) — replaces 40k pixel ImageData write per frame
  if (offscreen) {
    ctx.drawImage(offscreen, 0, 0)
  } else {
    drawWaitingPlaceholder(ctx)
    return
  }

  const map = coverageMap || baseMap
  if (!map?.info) return
  const width  = map.info.width
  const height = map.info.height
  const scale  = CANVAS_SIZE / Math.max(width, height)
  const info   = baseMap?.info || map.info
  if (!info) return
  const ox  = info.origin.position.x
  const oy  = info.origin.position.y
  const res = info.resolution
  const w2c = (wx, wy) => ({
    cx: ((wx - ox) / res) * scale,
    cy: (height - (wy - oy) / res) * scale,
  })

  for (let id = 0; id < numRobots; id++) {
    const color  = robotColor(id)
    const status = robotStatuses?.[id] || 'idle'
    const failed = status === 'failed'
    const active = status === 'active'
    const hex    = failed ? '#ef4444' : color.hex
    const rgb    = failed ? [239, 68, 68] : color.rgb

    // Trail
    if (overlays?.trail) {
      const trail = robotTrails?.[id] || []
      if (trail.length > 1) {
        ctx.lineCap = 'round'
        ctx.lineJoin = 'round'
        for (let i = 1; i < trail.length; i++) {
          const alpha = (i / trail.length) * 0.6
          const { cx: x0, cy: y0 } = w2c(...trail[i - 1])
          const { cx: x1, cy: y1 } = w2c(...trail[i])
          ctx.beginPath()
          ctx.moveTo(x0, y0)
          ctx.lineTo(x1, y1)
          ctx.strokeStyle = `rgba(${rgb[0]},${rgb[1]},${rgb[2]},${alpha})`
          ctx.lineWidth   = 1.6
          ctx.stroke()
        }
      }
    }

    // Planned path
    if (overlays?.path) {
      const path = robotPaths?.[id]
      if (path?.poses?.length > 1) {
        ctx.beginPath()
        const p0 = w2c(path.poses[0].pose.position.x, path.poses[0].pose.position.y)
        ctx.moveTo(p0.cx, p0.cy)
        const stride = Math.max(1, Math.floor(path.poses.length / 200))
        for (let i = stride; i < path.poses.length; i += stride) {
          const p = w2c(path.poses[i].pose.position.x, path.poses[i].pose.position.y)
          ctx.lineTo(p.cx, p.cy)
        }
        ctx.strokeStyle = `rgba(${rgb[0]},${rgb[1]},${rgb[2]},0.32)`
        ctx.lineWidth   = 1
        ctx.setLineDash([3, 4])
        ctx.stroke()
        ctx.setLineDash([])
      }
    }

    // Robot body — interpolated pose
    const lerp = robotsLerp[id]
    if (!lerp) continue
    const { cx, cy } = w2c(lerp.x, lerp.y)
    const drawYaw  = -lerp.yaw
    const bodySize = Math.max(8, scale * 2.4)

    // FOV ring
    if (overlays?.fov) {
      const fovR = (sensorRadius / res) * scale
      ctx.beginPath()
      ctx.arc(cx, cy, fovR, 0, Math.PI * 2)
      ctx.fillStyle = `rgba(${rgb[0]},${rgb[1]},${rgb[2]},0.06)`
      ctx.fill()
      ctx.beginPath()
      ctx.arc(cx, cy, fovR, 0, Math.PI * 2)
      ctx.strokeStyle = `rgba(${rgb[0]},${rgb[1]},${rgb[2]},0.28)`
      ctx.lineWidth   = 1
      ctx.setLineDash([3, 3])
      ctx.stroke()
      ctx.setLineDash([])
    }

    // Glow
    const glow = ctx.createRadialGradient(cx, cy, bodySize * 0.4, cx, cy, bodySize * 2.4)
    glow.addColorStop(0, `rgba(${rgb[0]},${rgb[1]},${rgb[2]},${active ? 0.55 : 0.3})`)
    glow.addColorStop(1, 'transparent')
    ctx.fillStyle = glow
    ctx.beginPath()
    ctx.arc(cx, cy, bodySize * 2.4, 0, Math.PI * 2)
    ctx.fill()

    // Chassis
    ctx.save()
    ctx.translate(cx, cy)
    ctx.rotate(drawYaw)

    const halfW = bodySize
    const halfH = bodySize * 0.78
    const radius = bodySize * 0.3

    drawRoundedRect(ctx, -halfW, -halfH, halfW * 2, halfH * 2, radius)
    ctx.fillStyle = hex
    ctx.fill()
    ctx.strokeStyle = 'rgba(255,255,255,0.85)'
    ctx.lineWidth = 1.4
    ctx.stroke()

    drawRoundedRect(ctx, -halfW * 0.55, -halfH * 0.6, halfW * 1.1, halfH * 1.2, radius * 0.5)
    ctx.fillStyle = `rgba(255,255,255,0.18)`
    ctx.fill()

    ctx.beginPath()
    ctx.moveTo(halfW * 0.85, 0)
    ctx.lineTo(halfW * 0.4, -halfH * 0.5)
    ctx.lineTo(halfW * 0.4,  halfH * 0.5)
    ctx.closePath()
    ctx.fillStyle = '#ffffff'
    ctx.fill()

    ctx.beginPath()
    ctx.arc(0, 0, halfW * 0.28, 0, Math.PI * 2)
    ctx.fillStyle = active ? '#ffffff' : 'rgba(255,255,255,0.7)'
    ctx.fill()

    ctx.restore()

    // ID badge
    ctx.fillStyle    = '#0b0f1a'
    ctx.strokeStyle  = `rgba(${rgb[0]},${rgb[1]},${rgb[2]},0.9)`
    ctx.lineWidth    = 1.5
    const badgeR = bodySize * 0.7
    const badgeX = cx
    const badgeY = cy - bodySize * 1.7
    drawRoundedRect(ctx, badgeX - badgeR, badgeY - bodySize * 0.45, badgeR * 2, bodySize * 0.9, bodySize * 0.45)
    ctx.fill()
    ctx.stroke()
    ctx.fillStyle = '#ffffff'
    ctx.font      = `600 ${Math.max(9, bodySize * 0.7)}px Inter, sans-serif`
    ctx.textAlign = 'center'
    ctx.textBaseline = 'middle'
    ctx.fillText(`R${id}`, badgeX, badgeY)

    if (failed) {
      ctx.save()
      ctx.translate(cx, cy)
      ctx.strokeStyle = '#ef4444'
      ctx.lineWidth   = 2.5
      ctx.lineCap = 'round'
      const cross = bodySize * 0.7
      ctx.beginPath()
      ctx.moveTo(-cross, -cross); ctx.lineTo(cross, cross)
      ctx.moveTo(cross, -cross);  ctx.lineTo(-cross, cross)
      ctx.stroke()
      ctx.restore()
    }
  }
}

// ── Heatmap colour map ─────────────────────────────────────────────────────
function heatmapColor(visits, cellVal) {
  if (cellVal === 100 || visits === -1) return [11, 17, 32]
  if (visits <= 0) return [30, 41, 59]
  if (visits === 1) return [ 37,  99, 239]
  if (visits === 2) return [ 34, 197,  94]
  if (visits === 3) return [245, 158,  11]
  return [239, 68, 68]
}

// ── Helpers ────────────────────────────────────────────────────────────────
function yawFromQuat(q) {
  return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
}

function drawRoundedRect(ctx, x, y, w, h, r) {
  ctx.beginPath()
  ctx.moveTo(x + r, y)
  ctx.lineTo(x + w - r, y)
  ctx.arcTo(x + w, y, x + w, y + r, r)
  ctx.lineTo(x + w, y + h - r)
  ctx.arcTo(x + w, y + h, x + w - r, y + h, r)
  ctx.lineTo(x + r, y + h)
  ctx.arcTo(x, y + h, x, y + h - r, r)
  ctx.lineTo(x, y + r)
  ctx.arcTo(x, y, x + r, y, r)
  ctx.closePath()
}

function drawWaitingPlaceholder(ctx) {
  ctx.fillStyle = '#0b0f1a'
  ctx.fillRect(0, 0, CANVAS_SIZE, CANVAS_SIZE)
  ctx.fillStyle = '#475569'
  ctx.font = '13px Inter, sans-serif'
  ctx.textAlign = 'center'
  ctx.fillText('Waiting for /map …', CANVAS_SIZE / 2, CANVAS_SIZE / 2)
}

export { TRAIL_LENGTH }
