import { useState, useEffect, useCallback, useRef } from 'react'
import { useRos } from './hooks/useRos'
import MapCanvas from './components/MapCanvas'
import StatsPanel from './components/StatsPanel'
import { robotColor } from './utils/colors'

const ROSBRIDGE_URL = import.meta.env.VITE_ROSBRIDGE_URL || 'ws://localhost:9090'
const NUM_ROBOTS    = 3   // updated live from stats

/** Connection status pill */
function StatusPill({ status }) {
  const cfg = {
    connected:    { dot: 'bg-green-400', text: 'text-green-300',  label: 'Connected'    },
    connecting:   { dot: 'bg-yellow-400 animate-pulse', text: 'text-yellow-300', label: 'Connecting…' },
    disconnected: { dot: 'bg-red-400',   text: 'text-red-300',    label: 'Disconnected' },
  }[status]

  return (
    <div className="flex items-center gap-2 bg-surface-700 rounded-full px-3 py-1">
      <div className={`w-2 h-2 rounded-full ${cfg.dot}`} />
      <span className={`text-xs font-mono font-semibold ${cfg.text}`}>{cfg.label}</span>
    </div>
  )
}

export default function App() {
  const { status, subscribe } = useRos(ROSBRIDGE_URL)

  const [baseMap,       setBaseMap]       = useState(null)
  const [coverageMap,   setCoverageMap]   = useState(null)
  const [robotPoses,    setRobotPoses]    = useState({})
  const [robotStatuses, setRobotStatuses] = useState({})
  const [stats,         setStats]         = useState(null)
  const [numRobots,     setNumRobots]     = useState(NUM_ROBOTS)

  // Track subscriptions so we can clean up per-robot ones when numRobots changes
  const robotUnsubs = useRef([])

  // ── Static map ─────────────────────────────────────────────────────────────
  useEffect(() => subscribe('/map', 'nav_msgs/OccupancyGrid', setBaseMap),
    [subscribe])

  // ── Coverage map ───────────────────────────────────────────────────────────
  useEffect(() => subscribe('/coverage_map', 'nav_msgs/OccupancyGrid', setCoverageMap),
    [subscribe])

  // ── Coverage stats ─────────────────────────────────────────────────────────
  useEffect(() => subscribe(
    '/coverage_stats',
    'multi_robot_coverage_msgs/CoverageStats',
    (msg) => {
      setStats(msg)
      if (msg.total_robots > 0) setNumRobots(msg.total_robots)
    }
  ), [subscribe])

  // ── Per-robot pose + status (re-subscribes when numRobots changes) ─────────
  useEffect(() => {
    // Clean up old subscriptions
    robotUnsubs.current.forEach(u => u())
    robotUnsubs.current = []

    for (let id = 0; id < numRobots; id++) {
      const poseUnsub = subscribe(
        `/robot_${id}/pose`,
        'geometry_msgs/PoseStamped',
        (msg) => setRobotPoses(prev => ({ ...prev, [id]: msg }))
      )
      const statusUnsub = subscribe(
        `/robot_${id}/status`,
        'std_msgs/String',
        (msg) => setRobotStatuses(prev => ({ ...prev, [id]: msg.data }))
      )
      robotUnsubs.current.push(poseUnsub, statusUnsub)
    }

    return () => robotUnsubs.current.forEach(u => u())
  }, [subscribe, numRobots])

  return (
    <div className="h-screen w-screen flex flex-col bg-surface-900 overflow-hidden select-none">

      {/* ── Header ─────────────────────────────────────────────────────────── */}
      <header className="flex items-center justify-between px-6 py-3 border-b border-surface-700 flex-shrink-0">
        <div className="flex items-center gap-3">
          <span className="text-2xl">🤖</span>
          <div>
            <h1 className="text-base font-bold text-slate-100 leading-tight">
              Multi-Robot Coverage
            </h1>
            <p className="text-xs text-slate-500 font-mono">ROS2 Humble · Boustrophedon BCD</p>
          </div>
        </div>
        <div className="flex items-center gap-3">
          {/* Robot colour badges */}
          <div className="flex gap-1">
            {Array.from({ length: numRobots }, (_, id) => (
              <div key={id} className="flex items-center gap-1 bg-surface-700 rounded-full px-2 py-0.5">
                <div className="w-2 h-2 rounded-full"
                     style={{ backgroundColor: robotColor(id).hex }} />
                <span className="text-xs font-mono text-slate-400">R{id}</span>
              </div>
            ))}
          </div>
          <StatusPill status={status} />
        </div>
      </header>

      {/* ── Main content ───────────────────────────────────────────────────── */}
      <div className="flex flex-1 gap-4 p-4 overflow-hidden">

        {/* Map panel */}
        <div className="flex flex-col gap-3 flex-shrink-0">
          <div className="flex items-center justify-between">
            <h2 className="text-xs text-slate-400 uppercase tracking-widest font-semibold">
              Live Coverage Map
            </h2>
            {coverageMap && (
              <span className="badge bg-surface-700 text-slate-400 font-mono">
                {coverageMap.info.width}×{coverageMap.info.height} · {coverageMap.info.resolution}m/px
              </span>
            )}
          </div>
          <MapCanvas
            baseMap={baseMap}
            coverageMap={coverageMap}
            robotPoses={robotPoses}
            robotStatuses={robotStatuses}
            numRobots={numRobots}
          />

          {/* Legend */}
          <div className="flex items-center gap-4 px-1">
            <LegendItem color="#1e293b" label="Free" />
            <LegendItem color="#0f172a" label="Obstacle" border />
            {Array.from({ length: numRobots }, (_, id) => (
              <LegendItem key={id} color={robotColor(id).hex} label={`R${id} coverage`} />
            ))}
          </div>
        </div>

        {/* Stats sidebar */}
        <div className="flex-1 min-w-[220px] max-w-[280px]">
          <h2 className="text-xs text-slate-400 uppercase tracking-widest font-semibold mb-3">
            Mission Stats
          </h2>
          <StatsPanel
            stats={stats}
            numRobots={numRobots}
            robotStatuses={robotStatuses}
          />
        </div>
      </div>
    </div>
  )
}

function LegendItem({ color, label, border }) {
  return (
    <div className="flex items-center gap-1.5">
      <div
        className="w-3 h-3 rounded-sm flex-shrink-0"
        style={{
          backgroundColor: color,
          border: border ? '1px solid #374151' : 'none',
        }}
      />
      <span className="text-xs text-slate-500">{label}</span>
    </div>
  )
}
