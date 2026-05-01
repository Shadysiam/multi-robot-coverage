/**
 * ControlBar — overlay toggles + live simulation controls
 *
 * Algorithm and speed changes publish directly to ROS topics:
 *   /set_algorithm  (std_msgs/String)   → coordinator replans immediately
 *   /set_speed      (std_msgs/Float64)  → robots update speed in real time
 */
export default function ControlBar({
  overlays, onToggle,
  speed, onSpeed,
  mapName, onMapChange,
  algorithm, onAlgorithmChange,
  onInjectFailure,
}) {
  const OVERLAYS = [
    { key: 'path',  label: 'Path'  },
    { key: 'fov',   label: 'FOV'   },
    { key: 'trail', label: 'Trail' },
    { key: 'grid',  label: 'Grid'  },
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
    <div className="flex flex-wrap items-center gap-3 px-1">

      {/* Overlay toggles */}
      <div className="flex gap-1">
        {OVERLAYS.map(({ key, label }) => (
          <button
            key={key}
            onClick={() => onToggle(key)}
            className={`px-3 py-1 rounded-lg text-xs font-mono font-semibold transition-colors ${
              overlays[key]
                ? 'bg-blue-600 text-white'
                : 'bg-surface-700 text-slate-400 hover:bg-surface-600'
            }`}
          >
            {label}
          </button>
        ))}
      </div>

      <div className="w-px h-4 bg-surface-600" />

      {/* Speed — publishes /set_speed to robot agents */}
      <div className="flex gap-1 items-center">
        <span className="text-xs text-slate-500 font-mono mr-1">Speed</span>
        {SPEEDS.map(({ label }) => (
          <button
            key={label}
            onClick={() => onSpeed(label)}
            className={`px-2 py-1 rounded-lg text-xs font-mono font-semibold transition-colors ${
              speed === label
                ? 'bg-green-600 text-white'
                : 'bg-surface-700 text-slate-400 hover:bg-surface-600'
            }`}
          >
            {label}
          </button>
        ))}
      </div>

      <div className="w-px h-4 bg-surface-600" />

      {/* Algorithm — publishes /set_algorithm, coordinator replans */}
      <div className="flex items-center gap-2">
        <span className="text-xs text-slate-500 font-mono">Algorithm</span>
        <select
          value={algorithm}
          onChange={e => onAlgorithmChange(e.target.value)}
          className="bg-surface-700 text-slate-300 text-xs font-mono rounded-lg px-2 py-1 border border-surface-600 focus:outline-none cursor-pointer"
        >
          {ALGOS.map(a => (
            <option key={a.value} value={a.value}>{a.label}</option>
          ))}
        </select>
      </div>

      {/* Map — requires full restart */}
      <div className="flex items-center gap-2">
        <span className="text-xs text-slate-500 font-mono">Map</span>
        <select
          value={mapName}
          onChange={e => onMapChange(e.target.value)}
          className="bg-surface-700 text-slate-300 text-xs font-mono rounded-lg px-2 py-1 border border-surface-600 focus:outline-none cursor-pointer"
        >
          {MAPS.map(m => (
            <option key={m.value} value={m.value}>{m.label}</option>
          ))}
        </select>
        <span className="text-xs text-slate-600 font-mono italic">(restart sim)</span>
      </div>

      <div className="w-px h-4 bg-surface-600" />

      {/* Failure injection — demonstrates propagation-based reallocation */}
      <button
        onClick={onInjectFailure}
        className="px-3 py-1 rounded-lg text-xs font-mono font-semibold bg-red-600/20 text-red-400 border border-red-600/40 hover:bg-red-600 hover:text-white transition-colors"
        title="Kill a random active robot — surviving robots will reallocate the dead robot's cells (Gong et al. 2024 propagation method)"
      >
        ⚠ Inject Failure
      </button>
    </div>
  )
}
