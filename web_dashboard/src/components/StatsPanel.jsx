import { robotColor } from '../utils/colors'

/** Circular progress ring SVG */
function ProgressRing({ pct, completed }) {
  const r    = 50
  const cx   = 64
  const circ = 2 * Math.PI * r
  const stroke = completed ? '#22c55e' : '#3b82f6'

  return (
    <svg width="128" height="128" className="mx-auto">
      <circle cx={cx} cy={cx} r={r} fill="none" stroke="#1f2937" strokeWidth="9" />
      <circle
        cx={cx} cy={cx} r={r}
        fill="none" stroke={stroke} strokeWidth="9" strokeLinecap="round"
        strokeDasharray={circ}
        strokeDashoffset={circ * (1 - pct / 100)}
        transform={`rotate(-90 ${cx} ${cx})`}
        style={{ transition: 'stroke-dashoffset 0.6s ease, stroke 0.4s' }}
      />
      <text x={cx} y={cx + 1} textAnchor="middle" dominantBaseline="middle"
        fill="#f1f5f9" fontSize="22" fontWeight="600" fontFamily="Inter">
        {pct.toFixed(1)}
        <tspan fontSize="13" fontWeight="500" fill="#94a3b8">%</tspan>
      </text>
      <text x={cx} y={cx + 22} textAnchor="middle" dominantBaseline="middle"
        fill="#64748b" fontSize="9" fontFamily="Inter" letterSpacing="1.5">
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
    <div className="w-full h-1 bg-surface-600 rounded-full overflow-hidden">
      <div
        className="h-full rounded-full"
        style={{
          width: `${Math.min(100, Math.max(0, pct))}%`,
          backgroundColor: color,
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

/**
 * Props
 * -----
 * stats          CoverageStats | null
 * numRobots      number
 * robotStatuses  { [id]: string }
 * robotDistances { [id]: number }   metres travelled
 * speed          string             active speed label
 */
export default function StatsPanel({ stats, numRobots, robotStatuses, robotDistances, speed }) {
  const pct       = stats?.coverage_percentage ?? 0
  const elapsed   = stats?.elapsed_time        ?? 0
  const algorithm = stats?.algorithm           ?? '—'
  const active    = stats?.robots_active       ?? 0
  const completed = stats?.completed           ?? false

  const algoLabel = ALGO_LABEL[algorithm] || algorithm
  const totalDist = Object.values(robotDistances || {})
    .reduce((a, b) => a + b, 0)

  // ETA: linear extrapolation from current rate (only meaningful when active)
  const etaSec = (() => {
    if (!stats || pct <= 1 || pct >= 100 || elapsed <= 0) return null
    const remaining = 100 - pct
    const rate = pct / elapsed   // %/s
    if (rate <= 0) return null
    return remaining / rate
  })()

  return (
    <div className="flex flex-col gap-3">

      {/* Coverage ring */}
      <div className="stat-card flex flex-col items-center gap-2 py-3">
        <ProgressRing pct={pct} completed={completed} />
        {completed && (
          <span className="badge bg-green-900/40 text-green-300 border border-green-700/50">
            ✓ Mission complete
          </span>
        )}
      </div>

      {/* Metric grid */}
      <div className="stat-card grid grid-cols-2 gap-3 py-3">
        <Metric label="Algorithm" value={algoLabel} />
        <Metric label="Speed"     value={speed} />
        <Metric label="Elapsed"   value={formatTime(elapsed)} mono />
        <Metric label="ETA"       value={etaSec != null ? formatTime(etaSec) : '—'} mono />
        <Metric label="Active"    value={`${active}/${numRobots}`} mono />
        <Metric label="Distance"  value={`${totalDist.toFixed(1)} m`} mono />
      </div>

      {/* Per-robot panel */}
      <div className="stat-card flex flex-col gap-3 py-3">
        <p className="text-[10px] text-slate-500 uppercase tracking-[0.15em] font-semibold">
          Robots
        </p>
        {Array.from({ length: numRobots }, (_, id) => {
          const status = robotStatuses?.[id] || 'idle'
          const color  = robotColor(id)
          const dist   = robotDistances?.[id] ?? 0
          const statusColor = {
            active:   'text-green-400',
            complete: 'text-blue-400',
            failed:   'text-red-400',
            idle:     'text-slate-500',
          }[status] || 'text-slate-500'

          // proxy per-robot pct: how much of total distance this robot covers
          const proxyPct = totalDist > 0 ? (dist / totalDist) * 100 * (pct / 100) * numRobots : 0

          return (
            <div key={id} className="flex flex-col gap-1.5">
              <div className="flex items-center gap-2">
                <div className="w-2.5 h-2.5 rounded-sm flex-shrink-0"
                     style={{ backgroundColor: color.hex }} />
                <span className="text-xs text-slate-300 flex-1 font-medium">Robot {id}</span>
                <span className={`text-[10px] font-mono uppercase tracking-wide ${statusColor}`}>
                  {status}
                </span>
              </div>
              <div className="pl-[18px] flex flex-col gap-1">
                <MiniBar pct={proxyPct} color={color.hex} />
                <span className="text-[10px] text-slate-500 font-mono">
                  {dist.toFixed(1)} m
                </span>
              </div>
            </div>
          )
        })}
      </div>
    </div>
  )
}

function Metric({ label, value, mono }) {
  return (
    <div className="flex flex-col gap-0.5">
      <span className="text-[10px] text-slate-500 uppercase tracking-[0.12em]">{label}</span>
      <span className={`text-sm text-slate-200 ${mono ? 'font-mono' : 'font-semibold'}`}>
        {value}
      </span>
    </div>
  )
}
