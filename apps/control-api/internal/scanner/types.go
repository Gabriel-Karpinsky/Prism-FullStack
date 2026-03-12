package scanner

type ActivityEntry struct {
	Source  string `json:"source"`
	TS      string `json:"ts"`
	Message string `json:"message"`
	Level   string `json:"level"`
}

type ScanSettings struct {
	YawMin              float64 `json:"yawMin"`
	YawMax              float64 `json:"yawMax"`
	PitchMin            float64 `json:"pitchMin"`
	PitchMax            float64 `json:"pitchMax"`
	SweepSpeedDegPerSec float64 `json:"sweepSpeedDegPerSec"`
	Resolution          string  `json:"resolution"`
}

type Metrics struct {
	MotorTempC     float64 `json:"motorTempC"`
	MotorCurrentA  float64 `json:"motorCurrentA"`
	LidarFPS       int     `json:"lidarFps"`
	RadarFPS       int     `json:"radarFps"`
	LatencyMS      int     `json:"latencyMs"`
	PacketsDropped int     `json:"packetsDropped"`
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
	Grid                  [][]float64     `json:"grid"`
}
