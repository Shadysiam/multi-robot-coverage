/** Robot colour palette — one colour per robot ID. */
export const ROBOT_COLORS = [
  { hex: '#3b82f6', rgb: [59,  130, 246], name: 'Blue'   },
  { hex: '#22c55e', rgb: [34,  197,  94], name: 'Green'  },
  { hex: '#f59e0b', rgb: [245, 158,  11], name: 'Amber'  },
  { hex: '#a855f7', rgb: [168,  85, 247], name: 'Purple' },
  { hex: '#ef4444', rgb: [239,  68,  68], name: 'Red'    },
  { hex: '#06b6d4', rgb: [  6, 182, 212], name: 'Cyan'   },
]

export const COLOR_OBSTACLE  = [15,  23,  42]   // dark navy
export const COLOR_FREE      = [30,  41,  59]   // slate-800
export const COLOR_UNKNOWN   = [15,  23,  42]   // same as obstacle

/**
 * Map a coverage grid cell value to an RGBA triplet.
 *
 * Encoding (from coverage_coordinator.py):
 *   0        → uncovered free space
 *   10*N     → covered by robot N-1  (10, 20, 30 …)
 *   100      → obstacle
 *  -1 / 255  → unknown
 */
export function cellColor(value) {
  if (value === 100)               return COLOR_OBSTACLE
  if (value <= 0 || value === 255) return COLOR_FREE
  const robotIdx = Math.floor(value / 10) - 1
  const c = ROBOT_COLORS[robotIdx % ROBOT_COLORS.length]
  // Slightly muted coverage colour so robots stand out on top
  return c.rgb.map(ch => Math.round(ch * 0.55 + 30))
}

export function robotColor(id) {
  return ROBOT_COLORS[id % ROBOT_COLORS.length]
}
