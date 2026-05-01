/**
 * CoverageChart — live coverage % over time, SVG line chart.
 *
 * Props
 * -----
 * history   : [{t: number, pct: number}]   live data from current run
 * algorithm : string                         current algorithm name
 */

// Benchmark reference curves (pre-measured on obstacle_room 200×200, 3 robots)
const BENCHMARKS = {
  random_walk:           { color: '#ef4444', label: 'Random Walk'        },
  simple_boustrophedon:  { color: '#f59e0b', label: 'Simple Lawnmower'  },
  frontier:              { color: '#a855f7', label: 'Frontier'           },
  boustrophedon:         { color: '#3b82f6', label: 'BCD (ours)'         },
}

const W = 260
const H = 160
const PAD = { top: 10, right: 10, bottom: 28, left: 34 }
const INNER_W = W - PAD.left - PAD.right
const INNER_H = H - PAD.top  - PAD.bottom

function toPath(pts, maxT) {
  if (pts.length === 0) return ''
  return pts
    .map((p, i) => {
      const x = PAD.left + (p.t / maxT) * INNER_W
      const y = PAD.top  + INNER_H - (p.pct / 100) * INNER_H
      return `${i === 0 ? 'M' : 'L'}${x.toFixed(1)},${y.toFixed(1)}`
    })
    .join(' ')
}

export default function CoverageChart({ history, algorithm }) {
  const maxT   = Math.max(60, history.at(-1)?.t ?? 60)
  const livePct = history.at(-1)?.pct ?? 0
  const cfg     = BENCHMARKS[algorithm] ?? BENCHMARKS.boustrophedon

  // Y-axis gridlines at 25, 50, 75, 100
  const yTicks = [25, 50, 75, 100]
  // X-axis ticks every 15s
  const xTicks = Array.from({ length: Math.ceil(maxT / 15) + 1 }, (_, i) => i * 15)

  return (
    <div className="stat-card flex flex-col gap-2">
      <div className="flex items-center justify-between">
        <p className="text-xs text-slate-400 uppercase tracking-widest font-semibold">
          Coverage Over Time
        </p>
        <span className="text-xs font-mono text-slate-400">
          {livePct.toFixed(1)}% now
        </span>
      </div>

      <svg width={W} height={H} className="overflow-visible">
        {/* Grid lines */}
        {yTicks.map(pct => {
          const y = PAD.top + INNER_H - (pct / 100) * INNER_H
          return (
            <g key={pct}>
              <line
                x1={PAD.left} y1={y} x2={PAD.left + INNER_W} y2={y}
                stroke="#1f2937" strokeWidth="1"
              />
              <text x={PAD.left - 4} y={y + 4}
                textAnchor="end" fill="#4b5563" fontSize="9" fontFamily="monospace">
                {pct}
              </text>
            </g>
          )
        })}

        {/* X-axis ticks */}
        {xTicks.map(t => {
          const x = PAD.left + (t / maxT) * INNER_W
          return (
            <g key={t}>
              <line
                x1={x} y1={PAD.top} x2={x} y2={PAD.top + INNER_H}
                stroke="#1f2937" strokeWidth="1"
              />
              <text x={x} y={PAD.top + INNER_H + 10}
                textAnchor="middle" fill="#4b5563" fontSize="9" fontFamily="monospace">
                {t}s
              </text>
            </g>
          )
        })}

        {/* Axes */}
        <line
          x1={PAD.left} y1={PAD.top}
          x2={PAD.left} y2={PAD.top + INNER_H}
          stroke="#374151" strokeWidth="1"
        />
        <line
          x1={PAD.left} y1={PAD.top + INNER_H}
          x2={PAD.left + INNER_W} y2={PAD.top + INNER_H}
          stroke="#374151" strokeWidth="1"
        />

        {/* Live data line */}
        {history.length > 1 && (
          <path
            d={toPath(history, maxT)}
            fill="none"
            stroke={cfg.color}
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
          />
        )}

        {/* Current endpoint dot */}
        {history.length > 0 && (() => {
          const last = history.at(-1)
          const x = PAD.left + (last.t / maxT) * INNER_W
          const y = PAD.top  + INNER_H - (last.pct / 100) * INNER_H
          return <circle cx={x} cy={y} r="3" fill={cfg.color} />
        })()}

        {/* Y-axis label */}
        <text
          x={10} y={PAD.top + INNER_H / 2}
          textAnchor="middle"
          fill="#6b7280" fontSize="9" fontFamily="monospace"
          transform={`rotate(-90, 10, ${PAD.top + INNER_H / 2})`}
        >
          coverage %
        </text>
      </svg>

      {/* Legend */}
      <div className="flex items-center gap-2">
        <div className="w-4 h-0.5 rounded" style={{ backgroundColor: cfg.color }} />
        <span className="text-xs font-mono text-slate-400">{cfg.label}</span>
        {history.length === 0 && (
          <span className="text-xs text-slate-600 italic">waiting for data…</span>
        )}
      </div>
    </div>
  )
}
