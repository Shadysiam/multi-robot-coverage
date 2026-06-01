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
  const [baseMap,        setBaseMap]        = useState(null)
  const [coverageMap,    setCoverageMap]    = useState(null)
  const [redundancyMap,  setRedundancyMap]  = useState(null)
  const [robotPoses,     setRobotPoses]     = useState({})
  const [robotStatuses,  setRobotStatuses]  = useState({})
  const [robotPaths,     setRobotPaths]     = useState({})   // planned paths
  const [stats,          setStats]          = useState(null)
  const [numRobots,      setNumRobots]      = useState(DEFAULT_ROBOTS)

  // ── Robot trails (ring buffer per robot) ──────────────────────────────────
  const [robotTrails,    setRobotTrails]    = useState({})
  const [robotDistances, setRobotDistances] = useState({})
  const prevPosRef = useRef({})   // {[id]: [x,y]} for distance calc

  // ── Coverage history for chart ─────────────────────────────────────────────
  const [coverageHistory, setCoverageHistory] = useState([])  // [{t, pct, completed}]
  const chartStartRef = useRef(null)
  // Sync the algorithm dropdown from /coverage_stats ONLY on the first
  // message — after that the user's selection is the source of truth.
  // Without this guard the dropdown flickers back to the old algorithm
  // during the ~500 ms replan window, which felt like "switch lag."
  const algoInitRef = useRef(false)

  // ── UI state ───────────────────────────────────────────────────────────────
  const [overlays,   setOverlays]   = useState({
    path: true, fov: true, trail: true, grid: false, heatmap: false,
  })
  const [speed,      setSpeed]      = useState('1×')
  const [mapName,    setMapName]    = useState('obstacle_room')
  const [algorithm,  setAlgorithm]  = useState('boustrophedon')
  const [isPaused,   setIsPaused]   = useState(false)
  // Brief "Planning..." overlay shown for ~1s after the user triggers a
  // new run (algorithm change / map change / reset).  Cleared automatically
  // by the first /coverage_stats message with pct > 0, or after a 1800 ms
  // safety timeout in case stats never start flowing.
  const [isTransitioning, setIsTransitioning] = useState(false)
  const transitionTimerRef = useRef(null)
  useEffect(() => {
    if (!isTransitioning) return
    if (transitionTimerRef.current) clearTimeout(transitionTimerRef.current)
    transitionTimerRef.current = setTimeout(() => {
      setIsTransitioning(false)
      transitionTimerRef.current = null
    }, 1800)
    return () => {
      if (transitionTimerRef.current) {
        clearTimeout(transitionTimerRef.current)
        transitionTimerRef.current = null
      }
    }
  }, [isTransitioning])

  const robotUnsubs = useRef([])
  // Per-robot debounce timers for the "complete" status.  With frontier,
  // robots cycle complete → active every ~200-350 ms, which made the
  // status badge flicker.  We delay propagating "complete" by 400 ms; if
  // a new status arrives before the timeout fires, the transient complete
  // is dropped and the badge never shows it.  Real completions (run truly
  // ended) persist beyond 400 ms and display normally.
  const statusTimeoutsRef = useRef({})

  // ── Toggle overlay ────────────────────────────────────────────────────────
  const handleToggle = useCallback((key) => {
    setOverlays(prev => ({ ...prev, [key]: !prev[key] }))
  }, [])

  // Reset all per-run visual state to a clean baseline.  Shared by the
  // three "fresh run" handlers (algorithm change, map change, reset sim)
  // so each one looks identical to the user — no leftover bars / chart
  // points / coverage paint linger between runs.  We also optimistically
  // set stats to 0% / 0s so the ring + elapsed display update INSTANTLY
  // rather than waiting ~500 ms for the first new /coverage_stats
  // message; at 5x playback the gap let robots paint ~3-4% before the
  // ring showed any value, which read as a jump from 0 to 4 instead of
  // a smooth curve from origin.
  const resetVisualState = useCallback(() => {
    setCoverageHistory([])
    setRobotTrails({})
    setRobotDistances({})
    setRobotPaths({})
    setCoverageMap(null)
    setRedundancyMap(null)
    setStats(prev => prev
      ? { ...prev, coverage_percentage: 0, elapsed_time: 0, completed: false }
      : null
    )
    prevPosRef.current = {}
    chartStartRef.current = null
  }, [])

  // ── Algorithm change → publishes to /set_algorithm, coordinator replans ──
  // Also wipes local visual state immediately — without this, the old
  // painted coverage cells linger on screen for ~500 ms while the coordinator
  // re-plans, which makes the switch feel like it froze the sim.
  const handleAlgorithmChange = useCallback((algo) => {
    setAlgorithm(algo)
    resetVisualState()
    setIsTransitioning(true)
    // If the sim is paused, resume it so the new algorithm actually runs
    if (isPaused) {
      setIsPaused(false)
      publish('/set_paused', 'std_msgs/Bool', { data: false })
    }
    publish('/set_algorithm', 'std_msgs/String', { data: algo })
  }, [publish, isPaused, resetVisualState])

  // ── Speed change → publishes to /set_speed, robot agents update live ─────
  const handleSpeed = useCallback((label) => {
    setSpeed(label)
    const val = SPEED_VALUES[label] ?? 1.0
    publish('/set_speed', 'std_msgs/Float64', { data: val })
  }, [publish])

  // ── Failure injection → coordinator picks a random active robot to kill ──
  // Throttled to once every 2 seconds — rapid clicks were stacking failures
  // before the previous reallocation could finish, producing weird paths.
  const lastFailRef = useRef(0)
  const handleInjectFailure = useCallback(() => {
    const now = Date.now()
    if (now - lastFailRef.current < 2000) return
    lastFailRef.current = now
    publish('/inject_failure', 'std_msgs/String', { data: 'auto' })
  }, [publish])

  // ── Pause toggle → freezes robots + sim-time integration on the
  //    coordinator side, so the chart axis stops advancing.  Useful for
  //    narrated demo recordings where you want to stop on a specific frame.
  const handleTogglePause = useCallback(() => {
    setIsPaused(prev => {
      const next = !prev
      publish('/set_paused', 'std_msgs/Bool', { data: next })
      return next
    })
  }, [publish])

  // ── Map change → publish to /set_map, map_server loads new file ──────────
  const handleMapChange = useCallback((newMap) => {
    setMapName(newMap)
    resetVisualState()
    setBaseMap(null)
    setRobotPoses({})   // old robot positions are nonsense in a different map
    setRobotStatuses({})
    setIsTransitioning(true)
    if (isPaused) {
      setIsPaused(false)
      publish('/set_paused', 'std_msgs/Bool', { data: false })
    }
    publish('/set_map', 'std_msgs/String', { data: newMap })
  }, [publish, isPaused, resetVisualState])

  // ── Reset Sim → revive failed robots + replan from scratch ───────────────
  const handleResetSim = useCallback(() => {
    resetVisualState()
    setIsTransitioning(true)
    if (isPaused) {
      setIsPaused(false)
      publish('/set_paused', 'std_msgs/Bool', { data: false })
    }
    publish('/reset_sim', 'std_msgs/String', { data: 'reset' })
  }, [publish, isPaused, resetVisualState])

  // ── Static map ─────────────────────────────────────────────────────────────
  useEffect(() => subscribe('/map', 'nav_msgs/OccupancyGrid', setBaseMap), [subscribe])

  // ── Coverage map ───────────────────────────────────────────────────────────
  useEffect(() => subscribe('/coverage_map', 'nav_msgs/OccupancyGrid', setCoverageMap), [subscribe])

  // ── Redundancy heatmap ─────────────────────────────────────────────────────
  useEffect(() => subscribe('/coverage_redundancy', 'nav_msgs/OccupancyGrid', setRedundancyMap), [subscribe])

  // ── Stats ──────────────────────────────────────────────────────────────────
  useEffect(() => subscribe(
    '/coverage_stats',
    'multi_robot_coverage_msgs/CoverageStats',
    (msg) => {
      setStats(msg)
      if (msg.total_robots > 0) setNumRobots(msg.total_robots)
      // Clear the "Planning..." overlay as soon as a real stat with motion arrives.
      if (isTransitioning && (msg.coverage_percentage ?? 0) > 0.1) {
        setIsTransitioning(false)
      }
      // Only initialize the dropdown from the coordinator on first message;
      // after that the user's local selection wins.
      if (msg.algorithm && !algoInitRef.current) {
        setAlgorithm(msg.algorithm)
        algoInitRef.current = true
      }

      // Build coverage-over-time history.
      // We stop extending the curve once the coordinator reports complete:
      //   - First completion message → append the final point so the curve
      //     terminates exactly at the completion time.
      //   - Subsequent completion messages → ignore (the coordinator keeps
      //     publishing heartbeats with growing elapsed_time but the mission
      //     is over — the graph used to slide right indefinitely).
      // t == 0 IS allowed now — anchors the chart curve at the origin so
      // the live curve starts at (0, 0) instead of jumping in mid-run after
      // a 1-2 s planning delay.
      const t   = msg.elapsed_time ?? 0
      const pct = msg.coverage_percentage ?? 0
      if (t >= 0) {
        if (chartStartRef.current === null) chartStartRef.current = t
        setCoverageHistory(prev => {
          const last = prev.at(-1)
          // Drop heartbeats sent after completion if we already terminated
          // the curve at a "completed" point.
          if (msg.completed && last && last.completed) return prev
          // Avoid duplicate time entries (within 400 ms)
          if (last && Math.abs(last.t - t) < 0.4) return prev
          const next = [
            ...prev,
            {
              t: parseFloat(t.toFixed(1)),
              pct: parseFloat(pct.toFixed(2)),
              completed: !!msg.completed,
            },
          ]
          return next.length > 500 ? next.slice(-500) : next
        })
      }
    }
  ), [subscribe])

  // ── Per-robot subscriptions (pose, status, path) ──────────────────────────
  // Trail/distance accumulators are batched in refs and flushed to state
  // every 100 ms — this prevents React from re-rendering 60×/sec just to
  // append a single trail point.
  const trailBufRef    = useRef({})
  const distAccumRef   = useRef({})

  useEffect(() => {
    const flushInterval = setInterval(() => {
      // CRITICAL: snapshot the ref into a local before swapping it out, so
      // the functional setState updater (which React invokes *later* during
      // commit) closes over the snapshot, not the now-emptied ref.  Without
      // this snapshot React reads the cleared ref at commit time, silently
      // losing every flush — that's the bug where Robot 1/2 distances stayed
      // at 0.0 m while Robot 0 occasionally caught a stray value.
      if (Object.keys(trailBufRef.current).length > 0) {
        const trailSnap = trailBufRef.current
        trailBufRef.current = {}
        setRobotTrails(prev => {
          const next = { ...prev }
          for (const id in trailSnap) {
            const incoming = trailSnap[id]
            const cur = next[id] || []
            const merged = [...cur, ...incoming]
            next[id] = merged.length > TRAIL_LENGTH ? merged.slice(-TRAIL_LENGTH) : merged
          }
          return next
        })
      }
      if (Object.keys(distAccumRef.current).length > 0) {
        const distSnap = distAccumRef.current
        distAccumRef.current = {}
        setRobotDistances(prev => {
          const next = { ...prev }
          for (const id in distSnap) {
            next[id] = (next[id] || 0) + distSnap[id]
          }
          return next
        })
      }
    }, 100)
    return () => clearInterval(flushInterval)
  }, [])

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

          // Buffer trail/distance — flushed every 100 ms (see flushInterval)
          if (!trailBufRef.current[id]) trailBufRef.current[id] = []
          trailBufRef.current[id].push([x, y])
          if (trailBufRef.current[id].length > 50) {
            trailBufRef.current[id] = trailBufRef.current[id].slice(-50)
          }

          const prev = prevPosRef.current[id]
          if (prev) {
            const dx = x - prev[0]
            const dy = y - prev[1]
            distAccumRef.current[id] = (distAccumRef.current[id] || 0) + Math.hypot(dx, dy)
          }
          prevPosRef.current[id] = [x, y]
        }
      )

      const statusUnsub = subscribe(
        `/robot_${id}/status`,
        'std_msgs/String',
        (msg) => {
          const newStatus = msg.data
          // Cancel any pending delayed "complete" for this robot.
          if (statusTimeoutsRef.current[id]) {
            clearTimeout(statusTimeoutsRef.current[id])
            delete statusTimeoutsRef.current[id]
          }
          if (newStatus === 'complete') {
            // Delay the UI update; if a new active/idle arrives within
            // 400 ms the complete frame is dropped before it ever renders.
            statusTimeoutsRef.current[id] = setTimeout(() => {
              setRobotStatuses(prev => ({ ...prev, [id]: 'complete' }))
              delete statusTimeoutsRef.current[id]
            }, 400)
          } else {
            // Non-complete statuses (active / idle / failed) apply instantly.
            setRobotStatuses(prev => ({ ...prev, [id]: newStatus }))
          }
        }
      )

      const pathUnsub = subscribe(
        `/robot_${id}/path`,
        'nav_msgs/Path',
        (msg) => setRobotPaths(prev => ({ ...prev, [id]: msg }))
      )

      robotUnsubs.current.push(poseUnsub, statusUnsub, pathUnsub)
    }

    return () => {
      robotUnsubs.current.forEach(u => u())
      // Clear any pending debounce timers so we don't leak on remount.
      Object.values(statusTimeoutsRef.current).forEach(clearTimeout)
      statusTimeoutsRef.current = {}
    }
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
          {/* Global sim actions — moved here from the ControlBar so they
              can never be clipped off the bottom of the map column on
              short viewports. Stays visible regardless of scroll state. */}
          <button
            onClick={handleTogglePause}
            className={`px-3 py-1.5 rounded-md text-[11px] font-mono font-semibold whitespace-nowrap border ${
              isPaused
                ? 'bg-amber-500 text-white border-amber-400 shadow-md shadow-amber-500/30'
                : 'bg-amber-500/15 text-amber-400 border-amber-500/30 hover:bg-amber-500 hover:text-white'
            }`}
            title={isPaused ? 'Resume the simulation' : 'Pause the simulation (freezes robots + chart axis)'}
          >
            {isPaused ? '▶ Resume' : '⏸ Pause'}
          </button>
          <button
            onClick={handleInjectFailure}
            className="px-3 py-1.5 rounded-md text-[11px] font-mono font-semibold bg-red-600/15 text-red-400 border border-red-600/30 hover:bg-red-600 hover:text-white whitespace-nowrap"
            title="Kill a random active robot — surviving robots will reallocate the dead robot's cells (Gong et al. 2024 propagation method)"
          >
            ⚠ Inject Failure
          </button>
          <button
            onClick={handleResetSim}
            className="px-3 py-1.5 rounded-md text-[11px] font-mono font-semibold bg-blue-600/15 text-blue-400 border border-blue-600/30 hover:bg-blue-600 hover:text-white whitespace-nowrap"
            title="Reset simulation: revive failed robots, clear coverage, replan from scratch"
          >
            ↺ Reset Sim
          </button>
          <StatusPill status={status} />
        </div>
      </header>

      {/* ── Main content ───────────────────────────────────────────────────── */}
      <div className="flex flex-1 gap-5 p-5 overflow-hidden">

        {/* Left: map + controls. min-h-0 + overflow-y-auto is a defensive
            scroll in case the map column ever exceeds viewport height —
            otherwise content at the bottom (controls, legend) gets
            clipped because the parent uses overflow-hidden. */}
        <div className="flex flex-col gap-3 flex-shrink-0 min-h-0 overflow-y-auto pr-1" style={{ width: 600 }}>
          <div className="flex items-center justify-between">
            <h2 className="text-xs text-slate-400 uppercase tracking-[0.15em] font-semibold">
              Live Coverage Map
            </h2>
            {coverageMap && (
              <span className="badge bg-surface-700 text-slate-400 font-mono">
                {coverageMap.info.width}×{coverageMap.info.height} · {coverageMap.info.resolution}m/px
              </span>
            )}
          </div>

          <div className="relative">
            <MapCanvas
              baseMap={baseMap}
              coverageMap={coverageMap}
              redundancyMap={redundancyMap}
              robotPoses={robotPoses}
              robotStatuses={robotStatuses}
              robotPaths={robotPaths}
              robotTrails={robotTrails}
              numRobots={numRobots}
              overlays={overlays}
              sensorRadius={0.5}
            />
            {/* Centred "PAUSED" badge overlay — visible only when paused.
                Makes the pause state unambiguous in demo recordings. */}
            {isPaused && (
              <div className="absolute inset-0 flex items-start justify-center pointer-events-none pt-4">
                <div className="bg-amber-500/95 text-white px-4 py-1.5 rounded-full text-xs font-mono font-bold tracking-wider shadow-lg shadow-amber-500/40 backdrop-blur-sm flex items-center gap-2">
                  <span className="w-2 h-2 rounded-full bg-white animate-pulse" />
                  PAUSED
                </div>
              </div>
            )}
            {/* "Planning…" overlay for the brief window between user-triggered
                run start (reset / algo change / map change) and first stats
                flowing. Bridges the visual gap so the user sees clear feedback
                instead of a blank canvas with no indication anything's happening. */}
            {isTransitioning && !isPaused && (
              <div className="absolute inset-0 flex items-center justify-center pointer-events-none bg-surface-900/40 backdrop-blur-[2px]">
                <div className="bg-blue-600/95 text-white px-5 py-2 rounded-full text-xs font-mono font-bold tracking-wider shadow-lg shadow-blue-600/40 flex items-center gap-2.5">
                  <span className="w-2 h-2 rounded-full bg-white animate-pulse" />
                  PLANNING…
                </div>
              </div>
            )}
          </div>

          {/* Overlay + controls bar — action buttons live in the header */}
          <ControlBar
            overlays={overlays}     onToggle={handleToggle}
            speed={speed}           onSpeed={handleSpeed}
            mapName={mapName}       onMapChange={handleMapChange}
            algorithm={algorithm}   onAlgorithmChange={handleAlgorithmChange}
          />

          {/* Legend — context-aware: shows heatmap key when heatmap is on */}
          <div className="flex items-center flex-wrap gap-4 px-1">
            {overlays.heatmap ? (
              <>
                <span className="text-[10px] text-slate-500 uppercase tracking-[0.15em] font-semibold mr-1">
                  Visits per cell
                </span>
                <LegendItem color="#1e293b" label="0" />
                <LegendItem color="#2563ef" label="1" />
                <LegendItem color="#22c55e" label="2" />
                <LegendItem color="#f59e0b" label="3" />
                <LegendItem color="#ef4444" label="4+" />
              </>
            ) : (
              <>
                <LegendItem color="#1e293b" label="Free" />
                <LegendItem color="#0b1120" label="Obstacle" border />
                {Array.from({ length: numRobots }, (_, id) => (
                  <LegendItem key={id} color={robotColor(id).hex} label={`R${id}`} />
                ))}
              </>
            )}
          </div>
        </div>

        {/* Right: stats sidebar — flex-grow, wider so nothing is squished */}
        <div className="flex-1 min-w-[340px] flex flex-col gap-3 overflow-y-auto pr-1">
          <h2 className="text-xs text-slate-400 uppercase tracking-[0.15em] font-semibold">
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
            mapName={mapName}
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
