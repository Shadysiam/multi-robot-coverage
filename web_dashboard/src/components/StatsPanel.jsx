import { robotColor } from '../utils/colors'

/** Circular progress ring SVG */
function ProgressRing({ pct, completed }) {
  const r    = 54
  const cx   = 70
  const circ = 2 * Math.PI * r
  const stroke = completed ? '#22c55e' : '#3b82f6'

  return (
    <svg width="140" height="140" className="mx-auto">
      <defs>
        <linearGradient id="ringGrad" x1="0" y1="0" x2="1" y2="1">
          <stop offset="0%"   stopColor={stroke}    stopOpacity="1" />
          <stop offset="100%" stopColor={stroke}    stopOpacity="0.6" />
        </linearGradient>
      </defs>
      <circle cx={cx} cy={cx} r={r} fill="none" stroke="#1f2937" strokeWidth="9" />
      <circle
        cx={cx} cy={cx} r={r}
        fill="none" stroke="url(#ringGrad)" strokeWidth="9" strokeLinecap="round"
        strokeDasharray={circ}
        strokeDashoffset={circ * (1 - pct / 100)}
        transform={`rotate(-90 ${cx} ${cx})`}
        style={{ transition: 'stroke-dashoffset 0.6s ease, stroke 0.4s' }}
      />
      <text x={cx} y={cx + 1} textAnchor="middle" dominantBaseline="middle"
        fill="#f1f5f9" fontSize="24" fontWeight="600" fontFamily="Inter">
        {pct.toFixed(1)}
        <tspan fontSize="14" fontWeight="500" fill="#94a3b8">%</tspan>
      </text>
      <text x={cx} y={cx + 23} textAnchor="middle" dominantBaseline="middle"
        fill="#64748b" fontSize="9" fontFamily="Inter" letterSpacing="2">
        COVERED
      </text>
    </svg>
  )
}

function formatTime(s) {
  const m = String(Math.floor(s / 60)).padStart(2, '0')
  const sec = String(Math.floor(s % 60)).padStart(2, '0')
  return `${m}:${sec}`
}

function MiniBar({ pct, color }) {
  return (
    <div className="w-full h-1.5 bg-surface-600/60 rounded-full overflow-hidden">
      <div
        className="h-full rounded-full"
        style={{
          width: `${Math.min(100, Math.max(0, pct))}%`,
          background: `linear-gradient(90deg, ${color}, ${color}aa)`,
          boxShadow: `0 0 6px ${color}66`,
          transition: 'width 0.5s ease',
        }}
      />
    </div>
  )
}

const ALGO_LABEL = {
  boustrophedon:        'BCD',
  frontier:             'Frontier',
  random_walk:          'Random Walk',
  simple_boustrophedon: 'Simple Lawn',
}

const STATUS_BADGE = {
  active:   { fg: 'text-emerald-400', bg: 'bg-emerald-500/10', dot: 'bg-emerald-400' },
  complete: { fg: 'text-blue-400',    bg: 'bg-blue-500/10',    dot: 'bg-blue-400'    },
  failed:   { fg: 'text-red-400',     bg: 'bg-red-500/10',     dot: 'bg-red-400'     },
  idle:     { fg: 'text-slate-500',   bg: 'bg-slate-700/40',   dot: 'bg-slate-500'   },
}

export default function StatsPanel({ stats, numRobots, robotStatuses, robotDistances, speed }) {
  const pct       = stats?.coverage_percentage ?? 0
  const elapsed   = stats?.elapsed_time        ?? 0
  const algorithm = stats?.algorithm           ?? '—'
  const active    = stats?.robots_active       ?? 0
  const completed = stats?.completed           ?? false

  const algoLabel = ALGO_LABEL[algorithm] || algorithm
  const distValues = Object.values(robotDistances || {})
  const totalDist  = distValues.reduce((a, b) => a + b, 0)
  // Bar fill = this robot's distance / busiest robot's distance.  Simple and
  // honest — the busiest robot's bar is always full, others scale linearly.
  // Previously the math was (dist/totalDist) * pct * numRobots which is
  // unintuitive and capped weirdly when one robot did most of the work.
  const maxDist = distValues.length ? Math.max(...distValues, 0) : 0

  // ETA: linear extrapolation from current rate
  const etaSec = (() => {
    if (!stats || pct <= 1 || pct >= 100 || elapsed <= 0) return null
    const remaining = 100 - pct
    const rate = pct / elapsed
    if (rate <= 0) return null
    return remaining / rate
  })()

  return (
    <div className="flex flex-col gap-3">

      {/* Coverage ring */}
      <div className="stat-card flex flex-col items-center gap-2 !py-4">
        <ProgressRing pct={pct} completed={completed} />
        {completed && (
          <span className="badge bg-emerald-900/30 text-emerald-300 border border-emerald-700/40">
            ✓ MISSION COMPLETE
          </span>
        )}
      </div>

      {/* Metric grid */}
      <div className="stat-card grid grid-cols-2 gap-x-4 gap-y-3">
        <Metric label="Algorithm" value={algoLabel} accent="blue" />
        <Metric label="Speed"     value={speed}     accent="emerald" mono />
        <Metric label="Elapsed"   value={formatTime(elapsed)} mono />
        <Metric label="ETA"       value={etaSec != null ? formatTime(etaSec) : '—'} mono />
        <Metric label="Active"    value={`${active} / ${numRobots}`} mono />
        <Metric label="Distance"  value={`${totalDist.toFixed(1)} m`} mono />
      </div>

      {/* Per-robot panel */}
      <div className="stat-card flex flex-col gap-3">
        <div className="flex items-center justify-between">
          <p className="text-[10px] text-slate-400 uppercase tracking-[0.15em] font-semibold">
            Robot Fleet
          </p>
          <span className="text-[10px] text-slate-500 font-mono">{numRobots} agents</span>
        </div>
        <div className="flex flex-col gap-2.5">
          {Array.from({ length: numRobots }, (_, id) => {
            const status = robotStatuses?.[id] || 'idle'
            const color  = robotColor(id)
            const dist   = robotDistances?.[id] ?? 0
            const badge  = STATUS_BADGE[status] || STATUS_BADGE.idle
            const barPct = maxDist > 0 ? (dist / maxDist) * 100 : 0

            return (
              <div key={id} className="flex flex-col gap-1.5">
                <div className="flex items-center gap-2">
                  <div className="w-3 h-3 rounded-md flex-shrink-0 shadow-md"
                       style={{ backgroundColor: color.hex, boxShadow: `0 0 8px ${color.hex}55` }} />
                  <span className="text-xs text-slate-200 flex-1 font-medium">Robot {id}</span>
                  <span className={`text-[9px] font-mono uppercase tracking-wider px-1.5 py-0.5 rounded ${badge.bg} ${badge.fg}`}>
                    {status}
                  </span>
                </div>
                <div className="pl-5 flex items-center gap-2">
                  <MiniBar pct={barPct} color={color.hex} />
                  <span className="text-[10px] text-slate-500 font-mono w-12 text-right">
                    {dist.toFixed(1)}m
                  </span>
                </div>
              </div>
            )
          })}
        </div>
      </div>
    </div>
  )
}

function Metric({ label, value, mono, accent }) {
  const accentClass = {
    blue:    'text-blue-300',
    emerald: 'text-emerald-300',
  }[accent] || 'text-slate-100'
  return (
    <div className="flex flex-col gap-0.5">
      <span className="text-[10px] text-slate-500 uppercase tracking-[0.12em]">{label}</span>
      <span className={`text-sm ${accentClass} ${mono ? 'font-mono' : 'font-semibold'}`}>
        {value}
      </span>
    </div>
  )
}
