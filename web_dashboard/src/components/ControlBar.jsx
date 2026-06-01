/**
 * ControlBar — overlay toggles + live simulation controls.
 *
 * All actions publish to ROS topics for live, no-restart control:
 *   /set_algorithm    /set_speed    /set_map    /inject_failure    /reset_sim
 */
export default function ControlBar({
  overlays, onToggle,
  speed, onSpeed,
  mapName, onMapChange,
  algorithm, onAlgorithmChange,
  onInjectFailure,
  onResetSim,
}) {
  const OVERLAYS = [
    { key: 'path',    label: 'Path'    },
    { key: 'fov',     label: 'FOV'     },
    { key: 'trail',   label: 'Trail'   },
    { key: 'grid',    label: 'Grid'    },
    { key: 'heatmap', label: 'Heatmap' },
  ]

  const SPEEDS = [
    { label: '0.5×', value: 0.5 },
    { label: '1×',   value: 1.0 },
    { label: '2×',   value: 2.0 },
    { label: '5×',   value: 5.0 },
  ]

  const MAPS = [
    { value: 'simple_room',   label: 'Simple Room'   },
    { value: 'obstacle_room', label: 'Obstacle Room' },
    { value: 'warehouse',     label: 'Warehouse'     },
  ]

  const ALGOS = [
    { value: 'boustrophedon',        label: 'BCD (ours)'        },
    { value: 'frontier',             label: 'Frontier'          },
    { value: 'simple_boustrophedon', label: 'Simple Lawnmower'  },
    { value: 'random_walk',          label: 'Random Walk'       },
  ]

  return (
    <div className="flex flex-wrap items-center gap-x-4 gap-y-2 stat-card !p-3">

      {/* Overlay toggles */}
      <Section label="View">
        <div className="flex gap-1">
          {OVERLAYS.map(({ key, label }) => (
            <button
              key={key}
              onClick={() => onToggle(key)}
              className={`px-2.5 py-1 rounded-md text-[11px] font-mono font-semibold ${
                overlays[key]
                  ? 'bg-blue-600 text-white shadow-md shadow-blue-600/20'
                  : 'bg-surface-700 text-slate-400 hover:bg-surface-600 hover:text-slate-200'
              }`}
            >
              {label}
            </button>
          ))}
        </div>
      </Section>

      <Divider />

      {/* Speed */}
      <Section label="Speed">
        <div className="flex gap-1">
          {SPEEDS.map(({ label }) => (
            <button
              key={label}
              onClick={() => onSpeed(label)}
              className={`w-9 px-1 py-1 rounded-md text-[11px] font-mono font-semibold ${
                speed === label
                  ? 'bg-emerald-600 text-white shadow-md shadow-emerald-600/20'
                  : 'bg-surface-700 text-slate-400 hover:bg-surface-600 hover:text-slate-200'
              }`}
            >
              {label}
            </button>
          ))}
        </div>
      </Section>

      <Divider />

      {/* Algorithm */}
      <Section label="Algorithm">
        <Select value={algorithm} onChange={onAlgorithmChange} options={ALGOS} />
      </Section>

      {/* Map */}
      <Section label="Map">
        <Select value={mapName} onChange={onMapChange} options={MAPS} />
      </Section>

      {/* Action buttons grouped so they never wrap separately and stay
          visible even when the control bar is squeezed. Shorter labels
          ("Fail" / "Reset") keep both fully visible at typical sidebar
          widths — the previous "Inject Failure" + "Reset Sim" combo
          pushed Reset off the end of the row. */}
      <Section label="Actions">
        <div className="flex gap-1.5">
          <button
            onClick={onInjectFailure}
            className="px-2.5 py-1 rounded-md text-[11px] font-mono font-semibold bg-red-600/15 text-red-400 border border-red-600/30 hover:bg-red-600 hover:text-white whitespace-nowrap"
            title="Kill a random active robot — surviving robots will reallocate the dead robot's cells (Gong et al. 2024 propagation method)"
          >
            ⚠ Fail
          </button>
          <button
            onClick={onResetSim}
            className="px-2.5 py-1 rounded-md text-[11px] font-mono font-semibold bg-blue-600/15 text-blue-400 border border-blue-600/30 hover:bg-blue-600 hover:text-white whitespace-nowrap"
            title="Reset simulation: revive failed robots, clear coverage, replan from scratch"
          >
            ↺ Reset
          </button>
        </div>
      </Section>
    </div>
  )
}

function Section({ label, children }) {
  return (
    <div className="flex items-center gap-2">
      <span className="text-[10px] text-slate-500 uppercase tracking-[0.12em] font-semibold">
        {label}
      </span>
      {children}
    </div>
  )
}

function Divider() {
  return <div className="w-px h-5 bg-surface-600 self-center" />
}

function Select({ value, onChange, options }) {
  return (
    <select
      value={value}
      onChange={e => onChange(e.target.value)}
      className="bg-surface-700 text-slate-300 text-[11px] font-mono rounded-md px-2 py-1 border border-surface-600 focus:outline-none focus:border-blue-500 cursor-pointer hover:bg-surface-600"
    >
      {options.map(o => (
        <option key={o.value} value={o.value}>{o.label}</option>
      ))}
    </select>
  )
}
