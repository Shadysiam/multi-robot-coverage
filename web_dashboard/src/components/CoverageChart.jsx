import { useRef, useEffect, useState } from 'react'

/**
 * CoverageChart — live coverage % over time, SVG line chart.
 *
 * Auto-resizes to fill the parent container's width using a ResizeObserver.
 * The chart adapts its scale and tick density based on width.
 */

const ALGO_STYLE = {
  random_walk:           { color: '#ef4444', label: 'Random Walk'      },
  simple_boustrophedon:  { color: '#f59e0b', label: 'Simple Lawnmower' },
  frontier:              { color: '#a855f7', label: 'Frontier'         },
  boustrophedon:         { color: '#3b82f6', label: 'BCD (ours)'       },
}

export default function CoverageChart({ history, algorithm }) {
  const wrapRef = useRef(null)
  const [w, setW] = useState(320)
  const H = 220

  useEffect(() => {
    if (!wrapRef.current) return
    const ro = new ResizeObserver(entries => {
      for (const e of entries) {
        setW(Math.max(240, Math.floor(e.contentRect.width)))
      }
    })
    ro.observe(wrapRef.current)
    return () => ro.disconnect()
  }, [])

  const PAD = { top: 14, right: 12, bottom: 30, left: 38 }
  const innerW = w - PAD.left - PAD.right
  const innerH = H - PAD.top  - PAD.bottom

  const maxT    = Math.max(60, history.at(-1)?.t ?? 60)
  const livePct = history.at(-1)?.pct ?? 0
  const cfg     = ALGO_STYLE[algorithm] ?? ALGO_STYLE.boustrophedon

  const yTicks = [25, 50, 75, 100]
  const xTickStep = maxT > 180 ? 60 : maxT > 90 ? 30 : 15
  const xTicks = []
  for (let t = 0; t <= maxT + 0.01; t += xTickStep) xTicks.push(t)

  const path = history.length === 0 ? '' : history
    .map((p, i) => {
      const x = PAD.left + (p.t / maxT) * innerW
      const y = PAD.top  + innerH - (p.pct / 100) * innerH
      return `${i === 0 ? 'M' : 'L'}${x.toFixed(1)},${y.toFixed(1)}`
    })
    .join(' ')

  const area = history.length === 0 ? '' : (() => {
    const baseY = PAD.top + innerH
    const startX = PAD.left + (history[0].t / maxT) * innerW
    const endX   = PAD.left + (history.at(-1).t / maxT) * innerW
    return `M${startX},${baseY} ${path.replace(/^M/, 'L')} L${endX},${baseY} Z`
  })()

  return (
    <div className="stat-card flex flex-col gap-2" ref={wrapRef}>
      <div className="flex items-center justify-between">
        <p className="text-[10px] text-slate-400 uppercase tracking-[0.15em] font-semibold">
          Coverage Over Time
        </p>
        <span className="text-xs font-mono text-slate-300">
          <span style={{ color: cfg.color }}>●</span> {livePct.toFixed(1)}%
        </span>
      </div>

      <svg width={w} height={H} className="overflow-visible">
        <defs>
          <linearGradient id={`area-${algorithm}`} x1="0" y1="0" x2="0" y2="1">
            <stop offset="0%"   stopColor={cfg.color} stopOpacity="0.35" />
            <stop offset="100%" stopColor={cfg.color} stopOpacity="0" />
          </linearGradient>
        </defs>

        {/* Y gridlines + labels */}
        {yTicks.map(pct => {
          const y = PAD.top + innerH - (pct / 100) * innerH
          return (
            <g key={pct}>
              <line
                x1={PAD.left} y1={y} x2={PAD.left + innerW} y2={y}
                stroke="#1f2937" strokeWidth="1" strokeDasharray="3,3"
              />
              <text x={PAD.left - 6} y={y + 3}
                textAnchor="end" fill="#64748b" fontSize="10" fontFamily="monospace">
                {pct}
              </text>
            </g>
          )
        })}

        {/* X tick labels */}
        {xTicks.map(t => {
          const x = PAD.left + (t / maxT) * innerW
          return (
            <g key={t}>
              <line
                x1={x} y1={PAD.top + innerH} x2={x} y2={PAD.top + innerH + 4}
                stroke="#374151" strokeWidth="1"
              />
              <text x={x} y={PAD.top + innerH + 14}
                textAnchor="middle" fill="#64748b" fontSize="10" fontFamily="monospace">
                {t}s
              </text>
            </g>
          )
        })}

        {/* Axes */}
        <line
          x1={PAD.left} y1={PAD.top} x2={PAD.left} y2={PAD.top + innerH}
          stroke="#374151" strokeWidth="1"
        />
        <line
          x1={PAD.left} y1={PAD.top + innerH} x2={PAD.left + innerW} y2={PAD.top + innerH}
          stroke="#374151" strokeWidth="1"
        />

        {/* Area fill */}
        {history.length > 1 && (
          <path d={area} fill={`url(#area-${algorithm})`} />
        )}

        {/* Line */}
        {history.length > 1 && (
          <path
            d={path}
            fill="none"
            stroke={cfg.color}
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
          />
        )}

        {/* Endpoint dot */}
        {history.length > 0 && (() => {
          const last = history.at(-1)
          const x = PAD.left + (last.t / maxT) * innerW
          const y = PAD.top  + innerH - (last.pct / 100) * innerH
          return (
            <>
              <circle cx={x} cy={y} r="6" fill={cfg.color} fillOpacity="0.2" />
              <circle cx={x} cy={y} r="3.5" fill={cfg.color} />
            </>
          )
        })()}

        {/* Y-axis label */}
        <text
          x={11} y={PAD.top + innerH / 2}
          textAnchor="middle"
          fill="#64748b" fontSize="9" fontFamily="monospace" letterSpacing="1.5"
          transform={`rotate(-90, 11, ${PAD.top + innerH / 2})`}
        >
          COVERAGE %
        </text>
      </svg>

      {/* Legend */}
      <div className="flex items-center gap-2 mt-1">
        <div className="w-3 h-0.5 rounded" style={{ backgroundColor: cfg.color }} />
        <span className="text-xs font-mono text-slate-400">{cfg.label}</span>
        {history.length === 0 && (
          <span className="text-xs text-slate-600 italic ml-auto">waiting for data…</span>
        )}
      </div>
    </div>
  )
}
