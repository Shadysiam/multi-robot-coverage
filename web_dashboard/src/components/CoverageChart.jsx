import { useRef, useEffect, useState } from 'react'

/**
 * CoverageChart — live coverage % over time with benchmark overlay.
 *
 * Shows the live coverage curve as a bold line, with the other 3 algorithms'
 * benchmark curves (from /benchmarks/<map>_<algorithm>.json) as faded
 * reference lines for direct comparison. Benchmarks are fetched on mount
 * and whenever the map changes.
 *
 * Auto-resizes to fill the parent container's width using a ResizeObserver.
 */

const ALGO_STYLE = {
  random_walk:           { color: '#ef4444', label: 'Random Walk'      },
  simple_boustrophedon:  { color: '#f59e0b', label: 'Simple Lawnmower' },
  frontier:              { color: '#a855f7', label: 'Frontier'         },
  boustrophedon:         { color: '#3b82f6', label: 'BCD (ours)'       },
}

const ALL_ALGOS = ['boustrophedon', 'frontier', 'simple_boustrophedon', 'random_walk']

export default function CoverageChart({ history, algorithm, mapName }) {
  const wrapRef = useRef(null)
  const [w, setW] = useState(320)
  const [benchmarks, setBenchmarks] = useState({})  // { [algo]: [{t, pct}] }
  const [showBenchmarks, setShowBenchmarks] = useState(true)
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

  // Fetch benchmark curves for the current map
  useEffect(() => {
    if (!mapName) return
    let cancelled = false
    const fetchAll = async () => {
      const result = {}
      for (const algo of ALL_ALGOS) {
        try {
          const res = await fetch(`/benchmarks/${mapName}_${algo}.json`)
          if (!res.ok) continue
          const data = await res.json()
          const curve = (data.results?.coverage_curve ?? []).map(p => ({
            t: p.t, pct: p.pct,
          }))
          if (curve.length > 1) result[algo] = curve
        } catch (e) {
          // 404 or missing — fine, just skip
        }
      }
      if (!cancelled) setBenchmarks(result)
    }
    fetchAll()
    return () => { cancelled = true }
  }, [mapName])

  const PAD = { top: 14, right: 12, bottom: 30, left: 38 }
  const innerW = w - PAD.left - PAD.right
  const innerH = H - PAD.top  - PAD.bottom

  // Compute maxT across live + all visible benchmarks so scales align
  const benchMaxT = Math.max(
    0,
    ...Object.values(benchmarks).map(c => c.at(-1)?.t ?? 0),
  )
  const liveMaxT  = history.at(-1)?.t ?? 0
  const maxT      = Math.max(60, liveMaxT, showBenchmarks ? benchMaxT : 0)
  const livePct   = history.at(-1)?.pct ?? 0
  const cfg       = ALGO_STYLE[algorithm] ?? ALGO_STYLE.boustrophedon

  const yTicks = [25, 50, 75, 100]
  const xTickStep = maxT > 180 ? 60 : maxT > 90 ? 30 : 15
  const xTicks = []
  for (let t = 0; t <= maxT + 0.01; t += xTickStep) xTicks.push(t)

  const buildPath = (points) => {
    if (!points || points.length === 0) return ''
    return points
      .map((p, i) => {
        const x = PAD.left + (p.t / maxT) * innerW
        const y = PAD.top  + innerH - (p.pct / 100) * innerH
        return `${i === 0 ? 'M' : 'L'}${x.toFixed(1)},${y.toFixed(1)}`
      })
      .join(' ')
  }

  const livePath = buildPath(history)

  const area = history.length === 0 ? '' : (() => {
    const baseY = PAD.top + innerH
    const startX = PAD.left + (history[0].t / maxT) * innerW
    const endX   = PAD.left + (history.at(-1).t / maxT) * innerW
    return `M${startX},${baseY} ${livePath.replace(/^M/, 'L')} L${endX},${baseY} Z`
  })()

  // Algorithms whose benchmarks we'll render.
  // We INCLUDE the current live algorithm's benchmark curve too — without it,
  // the live curve has no same-algorithm reference and gets compared against
  // other algorithms, which is unfair and confusing.  With it, the viewer
  // sees "live <algo>" vs "benchmark <algo>" side by side (the gap = ROS /
  // message-bus / scheduling overhead) plus the 3 other algorithms.
  const benchAlgos = ALL_ALGOS.filter(a => benchmarks[a])

  return (
    <div className="stat-card flex flex-col gap-2" ref={wrapRef}>
      <div className="flex items-center justify-between">
        <p className="text-[10px] text-slate-400 uppercase tracking-[0.15em] font-semibold">
          Coverage Over Time
          {Object.keys(benchmarks).length > 0 && (
            <span className="ml-2 text-slate-500 normal-case tracking-normal font-normal">
              ({Object.keys(benchmarks).length} benchmark{Object.keys(benchmarks).length === 1 ? '' : 's'} loaded)
            </span>
          )}
        </p>
        <div className="flex items-center gap-3">
          {Object.keys(benchmarks).length > 0 && (
            <button
              onClick={() => setShowBenchmarks(b => !b)}
              className="text-[10px] text-slate-500 hover:text-slate-300 font-mono uppercase tracking-wider"
              title="Toggle benchmark overlay"
            >
              {showBenchmarks ? 'hide bench' : 'show bench'}
            </button>
          )}
          <span className="text-xs font-mono text-slate-300">
            <span style={{ color: cfg.color }}>●</span> {livePct.toFixed(1)}%
          </span>
        </div>
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

        {/* Benchmark reference curves — drawn UNDER the live one, faded */}
        {showBenchmarks && benchAlgos.map(algo => {
          const style = ALGO_STYLE[algo]
          const pathD = buildPath(benchmarks[algo])
          if (!pathD) return null
          return (
            <path
              key={algo}
              d={pathD}
              fill="none"
              stroke={style.color}
              strokeWidth="1.5"
              strokeDasharray="4,3"
              strokeLinecap="round"
              strokeLinejoin="round"
              opacity="0.45"
            />
          )
        })}

        {/* Area fill (live curve) */}
        {history.length > 1 && (
          <path d={area} fill={`url(#area-${algorithm})`} />
        )}

        {/* Live curve — bold on top */}
        {history.length > 1 && (
          <path
            d={livePath}
            fill="none"
            stroke={cfg.color}
            strokeWidth="2.4"
            strokeLinecap="round"
            strokeLinejoin="round"
          />
        )}

        {/* Endpoint dot for live curve */}
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

      {/* Legend — live curve solid, benchmarks dashed */}
      <div className="flex items-center flex-wrap gap-x-3 gap-y-1 mt-1">
        {/* Live algorithm — solid line */}
        <div className="flex items-center gap-1.5">
          <div className="w-4 h-0.5 rounded" style={{ backgroundColor: cfg.color }} />
          <span className="text-xs font-mono text-slate-300">{cfg.label}</span>
          <span className="text-[10px] text-slate-500 uppercase">live</span>
        </div>
        {/* Benchmark curves (current map) — dashed */}
        {showBenchmarks && benchAlgos.map(algo => {
          const style = ALGO_STYLE[algo]
          // If this benchmark is for the same algorithm as the live curve,
          // label it as "benchmark" so the side-by-side comparison is clear.
          const isLiveAlgo = algo === algorithm
          return (
            <div key={algo} className="flex items-center gap-1.5">
              <div
                className="w-4 h-0 rounded"
                style={{
                  borderTop: `1.5px dashed ${style.color}`,
                  opacity: 0.6,
                }}
              />
              <span className="text-xs font-mono text-slate-500">{style.label}</span>
              {isLiveAlgo && (
                <span className="text-[10px] text-slate-600 uppercase">benchmark</span>
              )}
            </div>
          )
        })}
        {history.length === 0 && Object.keys(benchmarks).length === 0 && (
          <span className="text-xs text-slate-600 italic ml-auto">waiting for data…</span>
        )}
      </div>
    </div>
  )
}
