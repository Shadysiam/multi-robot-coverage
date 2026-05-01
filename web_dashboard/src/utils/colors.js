/** Robot colour palette — one per robot ID (max 6 distinct, then repeats). */
export const ROBOT_COLORS = [
  { hex: '#3b82f6', rgb: [ 59, 130, 246], name: 'Blue'   },
  { hex: '#22c55e', rgb: [ 34, 197,  94], name: 'Green'  },
  { hex: '#f59e0b', rgb: [245, 158,  11], name: 'Amber'  },
  { hex: '#a855f7', rgb: [168,  85, 247], name: 'Purple' },
  { hex: '#ec4899', rgb: [236,  72, 153], name: 'Pink'   },
  { hex: '#06b6d4', rgb: [  6, 182, 212], name: 'Cyan'   },
]

export const COLOR_OBSTACLE = [ 11,  17,  32]   // darker than free cells
export const COLOR_FREE     = [ 30,  41,  59]   // slate-800
export const COLOR_UNKNOWN  = [ 11,  17,  32]   // same as obstacle

/**
 * Map a coverage grid cell value to an RGB triplet.
 *
 * Encoding (from coverage_coordinator.py):
 *   0        → uncovered free space
 *   10*N     → covered by robot N-1  (10, 20, 30 …)
 *   100      → obstacle (or inflated safety zone)
 *  -1 / 255  → unknown
 */
export function cellColor(value) {
  if (value === 100)               return COLOR_OBSTACLE
  if (value <= 0 || value === 255) return COLOR_FREE
  const robotIdx = Math.floor(value / 10) - 1
  const c = ROBOT_COLORS[robotIdx % ROBOT_COLORS.length]
  // Blend with free-space colour so coverage feels translucent, not garish
  const t = 0.42   // 0 = pure free, 1 = pure robot colour
  return [
    Math.round(COLOR_FREE[0] * (1 - t) + c.rgb[0] * t),
    Math.round(COLOR_FREE[1] * (1 - t) + c.rgb[1] * t),
    Math.round(COLOR_FREE[2] * (1 - t) + c.rgb[2] * t),
  ]
}

export function robotColor(id) {
  return ROBOT_COLORS[id % ROBOT_COLORS.length]
}
