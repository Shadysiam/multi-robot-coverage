import { robotColor } from '../utils/colors'

/** Circular progress ring SVG */
function ProgressRing({ pct }) {
  const r  = 52
  const cx = 64
  const circumference = 2 * Math.PI * r

  return (
    <svg width="128" height="128" className="mx-auto">
      {/* track */}
      <circle cx={cx} cy={cx} r={r} fill="none" stroke="#1f2937" strokeWidth="10" />
      {/* progress */}
      <circle
        cx={cx} cy={cx} r={r}
        fill="none"
        stroke="#3b82f6"
        strokeWidth="10"
        strokeLinecap="round"
        strokeDasharray={circumference}
        strokeDashoffset={circumference * (1 - pct / 100)}
        transform={`rotate(-90 ${cx} ${cx})`}
        style={{ transition: 'stroke-dashoffset 0.6s ease' }}
      />
      <text x={cx} y={cx + 2} textAnchor="middle" dominantBaseline="middle"
        fill="#e2e8f0" fontSize="22" fontWeight="700" fontFamily="Inter">
        {pct.toFixed(1)}%
      </text>
      <text x={cx} y={cx + 20} textAnchor="middle" dominantBaseline="middle"
        fill="#94a3b8" fontSize="10" fontFamily="Inter">
        covered
      </text>
    </svg>
  )
}

/** Elapsed time → mm:ss */
function formatTime(seconds) {
  const m = String(Math.floor(seconds / 60)).padStart(2, '0')
  const s = String(Math.floor(seconds % 60)).padStart(2, '0')
  return `${m}:${s}`
}

/**
 * Props
 * -----
 * stats       : CoverageStats message | null
 * numRobots   : number
 * robotStatuses : { [id]: string }
 */
export default function StatsPanel({ stats, numRobots, robotStatuses }) {
  const pct       = stats?.coverage_percentage ?? 0
  const elapsed   = stats?.elapsed_time        ?? 0
  const algorithm = stats?.algorithm           ?? '—'
  const active    = stats?.robots_active       ?? 0
  const completed = stats?.completed           ?? false

  return (
    <div className="flex flex-col gap-4 h-full">

      {/* Coverage ring */}
      <div className="stat-card flex flex-col items-center gap-2">
        <p className="text-xs text-slate-400 uppercase tracking-widest font-semibold">Coverage</p>
        <ProgressRing pct={pct} />
        {completed && (
          <span className="badge bg-green-900 text-green-300">✓ Complete</span>
        )}
      </div>

      {/* Algorithm + time */}
      <div className="stat-card flex flex-col gap-3">
        <div className="flex justify-between items-center">
          <span className="text-xs text-slate-400 uppercase tracking-widest">Algorithm</span>
          <span className="badge bg-blue-900 text-blue-300 font-mono">{algorithm}</span>
        </div>
        <div className="flex justify-between items-center">
          <span className="text-xs text-slate-400 uppercase tracking-widest">Elapsed</span>
          <span className="font-mono text-slate-200 text-sm">{formatTime(elapsed)}</span>
        </div>
        <div className="flex justify-between items-center">
          <span className="text-xs text-slate-400 uppercase tracking-widest">Robots</span>
          <span className="font-mono text-slate-200 text-sm">{active}/{numRobots} active</span>
        </div>
      </div>

      {/* Per-robot status */}
      <div className="stat-card flex flex-col gap-2 flex-1">
        <p className="text-xs text-slate-400 uppercase tracking-widest font-semibold mb-1">Robots</p>
        {Array.from({ length: numRobots }, (_, id) => {
          const status = robotStatuses[id] || 'idle'
          const color  = robotColor(id)
          const statusColor = {
            active:   'text-green-400',
            complete: 'text-blue-400',
            failed:   'text-red-400',
            idle:     'text-slate-500',
          }[status] || 'text-slate-500'

          return (
            <div key={id} className="flex items-center gap-2">
              <div className="w-3 h-3 rounded-full flex-shrink-0"
                   style={{ backgroundColor: color.hex }} />
              <span className="text-sm text-slate-300 flex-1">Robot {id}</span>
              <span className={`text-xs font-mono ${statusColor}`}>{status}</span>
            </div>
          )
        })}
      </div>

      {/* ROS topic hint */}
      <p className="text-xs text-slate-600 font-mono text-center">
        rosbridge :9090
      </p>
    </div>
  )
}
