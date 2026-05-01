import { useState, useEffect, useCallback, useRef } from 'react'
import { useRos } from './hooks/useRos'
import MapCanvas, { TRAIL_LENGTH } from './components/MapCanvas'
import StatsPanel from './components/StatsPanel'
import ControlBar from './components/ControlBar'
import CoverageChart from './components/CoverageChart'
import { robotColor } from './utils/colors'

const ROSBRIDGE_URL  = import.meta.env.VITE_ROSBRIDGE_URL || 'ws://localhost:9090'
const DEFAULT_ROBOTS = 3

const SPEED_VALUES = { '0.5×': 0.5, '1×': 1.0, '2×': 2.0, '5×': 5.0 }

function StatusPill({ status }) {
  const cfg = {
    connected:    { dot: 'bg-green-400',                      text: 'text-green-300',  label: 'Connected'    },
    connecting:   { dot: 'bg-yellow-400 animate-pulse',       text: 'text-yellow-300', label: 'Connecting…'  },
    disconnected: { dot: 'bg-red-400',                        text: 'text-red-300',    label: 'Disconnected' },
  }[status]
  return (
    <div className="flex items-center gap-2 bg-surface-700 rounded-full px-3 py-1">
      <div className={`w-2 h-2 rounded-full ${cfg.dot}`} />
      <span className={`text-xs font-mono font-semibold ${cfg.text}`}>{cfg.label}</span>
    </div>
  )
}

export default function App() {
  const { status, subscribe, publish } = useRos(ROSBRIDGE_URL)

  // ── Map / coverage data ────────────────────────────────────────────────────
  const [baseMap,       setBaseMap]       = useState(null)
  const [coverageMap,   setCoverageMap]   = useState(null)
  const [robotPoses,    setRobotPoses]    = useState({})
  const [robotStatuses, setRobotStatuses] = useState({})
  const [robotPaths,    setRobotPaths]    = useState({})   // planned paths
  const [stats,         setStats]         = useState(null)
  const [numRobots,     setNumRobots]     = useState(DEFAULT_ROBOTS)

  // ── Robot trails (ring buffer per robot) ──────────────────────────────────
  const [robotTrails,    setRobotTrails]    = useState({})
  const [robotDistances, setRobotDistances] = useState({})
  const prevPosRef = useRef({})   // {[id]: [x,y]} for distance calc

  // ── Coverage history for chart ─────────────────────────────────────────────
  const [coverageHistory, setCoverageHistory] = useState([])  // [{t, pct}]
  const chartStartRef = useRef(null)

  // ── UI state ───────────────────────────────────────────────────────────────
  const [overlays,   setOverlays]   = useState({ path: true, fov: true, trail: true, grid: false })
  const [speed,      setSpeed]      = useState('1×')
  const [mapName,    setMapName]    = useState('obstacle_room')
  const [algorithm,  setAlgorithm]  = useState('boustrophedon')

  const robotUnsubs = useRef([])

  // ── Toggle overlay ────────────────────────────────────────────────────────
  const handleToggle = useCallback((key) => {
    setOverlays(prev => ({ ...prev, [key]: !prev[key] }))
  }, [])

  // ── Algorithm change → publishes to /set_algorithm, coordinator replans ──
  const handleAlgorithmChange = useCallback((algo) => {
    setAlgorithm(algo)
    // Reset all per-run state so the new algorithm starts clean
    setCoverageHistory([])
    setRobotTrails({})
    setRobotDistances({})
    setRobotPaths({})
    prevPosRef.current = {}
    chartStartRef.current = null
    publish('/set_algorithm', 'std_msgs/String', { data: algo })
  }, [publish])

  // ── Speed change → publishes to /set_speed, robot agents update live ─────
  const handleSpeed = useCallback((label) => {
    setSpeed(label)
    const val = SPEED_VALUES[label] ?? 1.0
    publish('/set_speed', 'std_msgs/Float64', { data: val })
  }, [publish])

  // ── Failure injection → coordinator picks a random active robot to kill ──
  const handleInjectFailure = useCallback(() => {
    publish('/inject_failure', 'std_msgs/String', { data: 'auto' })
  }, [publish])

  // ── Static map ─────────────────────────────────────────────────────────────
  useEffect(() => subscribe('/map', 'nav_msgs/OccupancyGrid', setBaseMap), [subscribe])

  // ── Coverage map ───────────────────────────────────────────────────────────
  useEffect(() => subscribe('/coverage_map', 'nav_msgs/OccupancyGrid', setCoverageMap), [subscribe])

  // ── Stats ──────────────────────────────────────────────────────────────────
  useEffect(() => subscribe(
    '/coverage_stats',
    'multi_robot_coverage_msgs/CoverageStats',
    (msg) => {
      setStats(msg)
      if (msg.total_robots > 0) setNumRobots(msg.total_robots)
      if (msg.algorithm) setAlgorithm(msg.algorithm)

      // Build coverage-over-time history
      const t   = msg.elapsed_time ?? 0
      const pct = msg.coverage_percentage ?? 0
      if (t > 0) {
        if (chartStartRef.current === null) chartStartRef.current = t
        setCoverageHistory(prev => {
          // Avoid duplicate time entries; cap at 500 points
          const last = prev.at(-1)
          if (last && Math.abs(last.t - t) < 0.4) return prev
          const next = [...prev, { t: parseFloat(t.toFixed(1)), pct: parseFloat(pct.toFixed(2)) }]
          return next.length > 500 ? next.slice(-500) : next
        })
      }
    }
  ), [subscribe])

  // ── Per-robot subscriptions (pose, status, path) ──────────────────────────
  useEffect(() => {
    robotUnsubs.current.forEach(u => u())
    robotUnsubs.current = []

    for (let id = 0; id < numRobots; id++) {
      const poseUnsub = subscribe(
        `/robot_${id}/pose`,
        'geometry_msgs/PoseStamped',
        (msg) => {
          const x = msg.pose.position.x
          const y = msg.pose.position.y

          setRobotPoses(prev => ({ ...prev, [id]: msg }))

          // Trail: append position, keep last TRAIL_LENGTH entries
          setRobotTrails(prev => {
            const trail = prev[id] || []
            const next  = [...trail, [x, y]]
            return { ...prev, [id]: next.length > TRAIL_LENGTH ? next.slice(-TRAIL_LENGTH) : next }
          })

          // Distance accumulation
          const prev2 = prevPosRef.current[id]
          if (prev2) {
            const dx = x - prev2[0]
            const dy = y - prev2[1]
            const d  = Math.hypot(dx, dy)
            setRobotDistances(prev => ({ ...prev, [id]: (prev[id] || 0) + d }))
          }
          prevPosRef.current[id] = [x, y]
        }
      )

      const statusUnsub = subscribe(
        `/robot_${id}/status`,
        'std_msgs/String',
        (msg) => setRobotStatuses(prev => ({ ...prev, [id]: msg.data }))
      )

      const pathUnsub = subscribe(
        `/robot_${id}/path`,
        'nav_msgs/Path',
        (msg) => setRobotPaths(prev => ({ ...prev, [id]: msg }))
      )

      robotUnsubs.current.push(poseUnsub, statusUnsub, pathUnsub)
    }

    return () => robotUnsubs.current.forEach(u => u())
  }, [subscribe, numRobots])

  return (
    <div className="h-screen w-screen flex flex-col bg-surface-900 overflow-hidden select-none">

      {/* ── Header ─────────────────────────────────────────────────────────── */}
      <header className="flex items-center justify-between px-6 py-3 border-b border-surface-700 flex-shrink-0">
        <div className="flex items-center gap-4">
          {/* Logo mark */}
          <div className="flex items-center justify-center w-8 h-8 rounded-lg bg-blue-600">
            <svg width="16" height="16" viewBox="0 0 16 16" fill="none">
              <rect x="1" y="1" width="6" height="6" rx="1" fill="white" fillOpacity="0.9"/>
              <rect x="9" y="1" width="6" height="6" rx="1" fill="white" fillOpacity="0.5"/>
              <rect x="1" y="9" width="6" height="6" rx="1" fill="white" fillOpacity="0.5"/>
              <rect x="9" y="9" width="6" height="6" rx="1" fill="white" fillOpacity="0.9"/>
            </svg>
          </div>
          <div>
            <h1 className="text-sm font-semibold text-slate-100 tracking-tight">
              Multi-Robot Coverage Planner
            </h1>
            <p className="text-xs text-slate-500 font-mono">
              ROS2 Humble &nbsp;·&nbsp; BCD + A* &nbsp;·&nbsp; {numRobots} agents
            </p>
          </div>
        </div>
        <div className="flex items-center gap-3">
          {/* Robot colour badges */}
          <div className="flex gap-1">
            {Array.from({ length: numRobots }, (_, id) => (
              <div key={id} className="flex items-center gap-1.5 bg-surface-700 rounded-md px-2.5 py-1">
                <div className="w-2 h-2 rounded-full" style={{ backgroundColor: robotColor(id).hex }} />
                <span className="text-xs font-mono text-slate-400">R{id}</span>
              </div>
            ))}
          </div>
          <StatusPill status={status} />
        </div>
      </header>

      {/* ── Main content ───────────────────────────────────────────────────── */}
      <div className="flex flex-1 gap-4 p-4 overflow-hidden">

        {/* Left: map + controls */}
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
            robotPaths={robotPaths}
            robotTrails={robotTrails}
            numRobots={numRobots}
            overlays={overlays}
            sensorRadius={0.5}
          />

          {/* Overlay + controls bar */}
          <ControlBar
            overlays={overlays}     onToggle={handleToggle}
            speed={speed}           onSpeed={handleSpeed}
            mapName={mapName}       onMapChange={setMapName}
            algorithm={algorithm}   onAlgorithmChange={handleAlgorithmChange}
            onInjectFailure={handleInjectFailure}
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

        {/* Right: stats sidebar */}
        <div className="flex-1 min-w-[260px] max-w-[300px] flex flex-col gap-3 overflow-y-auto">
          <h2 className="text-xs text-slate-400 uppercase tracking-widest font-semibold">
            Mission Stats
          </h2>
          <StatsPanel
            stats={stats}
            numRobots={numRobots}
            robotStatuses={robotStatuses}
            robotDistances={robotDistances}
            speed={speed}
          />
          <CoverageChart
            history={coverageHistory}
            algorithm={algorithm}
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
        style={{ backgroundColor: color, border: border ? '1px solid #374151' : 'none' }}
      />
      <span className="text-xs text-slate-500">{label}</span>
    </div>
  )
}
