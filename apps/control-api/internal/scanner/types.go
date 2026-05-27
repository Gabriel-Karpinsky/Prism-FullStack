package scanner

type ActivityEntry struct {
	Source  string `json:"source"`
	TS      string `json:"ts"`
	Message string `json:"message"`
	Level   string `json:"level"`
}

type ScanSettings struct {
	YawMin                 float64 `json:"yawMin"`
	YawMax                 float64 `json:"yawMax"`
	PitchMin               float64 `json:"pitchMin"`
	PitchMax               float64 `json:"pitchMax"`
	SweepSpeedDegPerSec    float64 `json:"sweepSpeedDegPerSec"`
	Resolution             string  `json:"resolution"`
	SampleStrideMicrosteps int     `json:"sampleStrideMicrosteps"`
	ScanMode               string  `json:"scanMode"`           // "sweep" | "step"
	SweepMaxSpeedDegS      float64 `json:"sweepMaxSpeedDegS"`  // configured sweep ceiling
}

// Metrics is a placeholder for future real sensor telemetry. The fields that
// used to live here (motor temp/current, lidar/radar fps, latency, packets
// dropped) were fabricated values shown as if measured, and have been removed.
// Kept as an empty struct so the wire schema can grow without another break.
type Metrics struct{}

// AxisMotion mirrors the edge-daemon's per-axis motion envelope. Keys are
// snake_case to match the wire format the daemon ships on /api/config/motion.
type AxisMotion struct {
	MinDeg        float64 `json:"min_deg"`
	MaxDeg        float64 `json:"max_deg"`
	MaxSpeedDegS  float64 `json:"max_speed_deg_s"`
	AccelDegS2    float64 `json:"accel_deg_s2"`
}

// MotionConfig is the combined yaw/pitch envelope used by both GET and PUT
// /api/config/motion. PUT accepts a partial payload (either axis may be
// omitted); the edge daemon validates + persists atomically.
type MotionConfig struct {
	Yaw   AxisMotion `json:"yaw"`
	Pitch AxisMotion `json:"pitch"`
}

// GridUpdate is the incremental scan-grid payload. The client holds the grid
// locally and applies deltas: idx/val are parallel arrays (idx = row-major cell
// index y*Width+x, val = normalised 0..1 height). Full means "rebuild from these
// cells" (sent when the client's generation is stale, e.g. after a resolution
// change or scan reset, which bump Generation). See the daemon's GetGridUpdate.
type GridUpdate struct {
	Generation uint64    `json:"generation"`
	Version    uint64    `json:"version"`
	Width      int       `json:"width"`
	Height     int       `json:"height"`
	Full       bool      `json:"full"`
	Idx        []int     `json:"idx"`
	Val        []float64 `json:"val"`
}

type Snapshot struct {
	Connected             bool            `json:"connected"`
	Mode                  string          `json:"mode"`
	ControlOwner          string          `json:"controlOwner"`
	ControlLeaseExpiresAt *string         `json:"controlLeaseExpiresAt"`
	Yaw                   float64         `json:"yaw"`
	Pitch                 float64         `json:"pitch"`
	Coverage              float64         `json:"coverage"`
	ScanProgress          float64         `json:"scanProgress"`
	ScanDurationSeconds   float64         `json:"scanDurationSeconds"`
	LastCompletedScanAt   *string         `json:"lastCompletedScanAt"`
	ScanSettings          ScanSettings    `json:"scanSettings"`
	Metrics               Metrics         `json:"metrics"`
	Faults                []string        `json:"faults"`
	Activity              []ActivityEntry `json:"activity"`
	// GridUpdate is only populated on the polled /api/state path (when the client
	// sends ?since=). Command/acquire/release responses leave it nil so they stay
	// small; the UI refreshes the grid from the poll loop.
	GridUpdate *GridUpdate `json:"gridUpdate,omitempty"`
}
