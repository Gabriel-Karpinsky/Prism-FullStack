package scanner

import (
	"errors"
	"math"
	"strconv"
	"strings"
	"sync"
	"time"
)

const (
	gridWidth    = 48
	gridHeight   = 24
	leaseSeconds = 120
)

type state struct {
	connected           bool
	mode                string
	controlOwner        string
	controlLeaseExpiry  *time.Time
	yaw                 float64
	pitch               float64
	coverage            float64
	scanProgress        float64
	scanDurationSeconds float64
	scanStartedAt       *time.Time
	scanAccumulated     float64
	filledCells         int
	lastCompletedScanAt *time.Time
	grid                [][]float64
	scanSettings        ScanSettings
	metrics             Metrics
	faults              []string
	activity            []ActivityEntry
}

type Service struct {
	mu    sync.Mutex
	state state
}

func NewService() *Service {
	s := &Service{
		state: state{
			connected: true,
			mode:      "idle",
			grid:      newGrid(),
			scanSettings: ScanSettings{
				YawMin:              -60,
				YawMax:              60,
				PitchMin:            -20,
				PitchMax:            35,
				SweepSpeedDegPerSec: 20,
				Resolution:          "medium",
			},
			metrics: Metrics{
				MotorTempC:     31.4,
				MotorCurrentA:  1.3,
				LidarFPS:       18,
				RadarFPS:       11,
				LatencyMS:      42,
				PacketsDropped: 0,
			},
			faults:   []string{},
			activity: []ActivityEntry{},
		},
	}

	s.addLog("system", "info", "Go control API initialized.")
	s.addLog("scanner", "info", "Scanner connected to simulated control bus.")
	return s
}

func (s *Service) Snapshot() Snapshot {
	s.mu.Lock()
	defer s.mu.Unlock()

	s.updateLocked()
	return s.snapshotLocked()
}

func (s *Service) Acquire(user string) (Snapshot, error) {
	s.mu.Lock()
	defer s.mu.Unlock()

	user = strings.TrimSpace(user)
	if user == "" {
		return Snapshot{}, errors.New("user is required to acquire control")
	}

	s.clearExpiredLeaseLocked()
	if s.state.controlOwner != "" && s.state.controlOwner != user {
		return Snapshot{}, errors.New("control is already held by " + s.state.controlOwner)
	}

	expiry := time.Now().UTC().Add(leaseSeconds * time.Second)
	s.state.controlOwner = user
	s.state.controlLeaseExpiry = &expiry
	s.addLog("control", "info", user+" acquired control.")
	s.updateLocked()
	return s.snapshotLocked(), nil
}

func (s *Service) Release(user string) (Snapshot, error) {
	s.mu.Lock()
	defer s.mu.Unlock()

	if err := s.requireControlLocked(strings.TrimSpace(user)); err != nil {
		return Snapshot{}, err
	}

	s.state.controlOwner = ""
	s.state.controlLeaseExpiry = nil
	s.addLog("control", "info", strings.TrimSpace(user)+" released control.")
	s.updateLocked()
	return s.snapshotLocked(), nil
}

func (s *Service) Command(user, command string, payload map[string]any) (Snapshot, error) {
	s.mu.Lock()
	defer s.mu.Unlock()

	user = strings.TrimSpace(user)
	command = strings.ToLower(strings.TrimSpace(command))
	if command == "" {
		return Snapshot{}, errors.New("command is required")
	}

	if command != "connect" {
		if err := s.requireControlLocked(user); err != nil {
			return Snapshot{}, err
		}
	}

	switch command {
	case "connect":
		s.state.connected = true
		s.addLog("scanner", "info", "Scanner transport connected.")
	case "home":
		if !s.state.connected {
			return Snapshot{}, errors.New("scanner is not connected")
		}
		if s.state.mode == "fault" {
			return Snapshot{}, errors.New("clear the fault before homing")
		}
		s.state.mode = "idle"
		s.state.yaw = 0
		s.state.pitch = 0
		s.addLog("motion", "info", "Gantry returned to home position.")
	case "jog":
		if s.state.mode == "scanning" {
			return Snapshot{}, errors.New("pause the scan before jogging")
		}

		axis := strings.ToLower(strings.TrimSpace(stringFromMap(payload, "axis")))
		delta, err := floatFromMap(payload, "delta")
		if err != nil {
			return Snapshot{}, err
		}

		switch axis {
		case "yaw":
			s.state.yaw = round2(clamp(s.state.yaw+delta, s.state.scanSettings.YawMin, s.state.scanSettings.YawMax))
		case "pitch":
			s.state.pitch = round2(clamp(s.state.pitch+delta, s.state.scanSettings.PitchMin, s.state.scanSettings.PitchMax))
		default:
			return Snapshot{}, errors.New("unsupported jog axis '" + axis + "'")
		}

		s.state.mode = "manual"
		s.addLog("motion", "info", "Jogged "+axis+" by "+formatFloat(delta)+" degrees.")
	case "set_resolution":
		resolution := strings.ToLower(strings.TrimSpace(stringFromMap(payload, "resolution")))
		if resolution != "low" && resolution != "medium" && resolution != "high" {
			return Snapshot{}, errors.New("resolution must be low, medium, or high")
		}
		s.state.scanSettings.Resolution = resolution
		s.addLog("scanner", "info", "Scan resolution set to "+resolution+".")
	case "start_scan":
		if !s.state.connected {
			return Snapshot{}, errors.New("scanner is not connected")
		}
		if s.state.mode == "fault" {
			return Snapshot{}, errors.New("clear the fault before starting a scan")
		}
		s.resetScanLocked()
		now := time.Now().UTC()
		s.state.mode = "scanning"
		s.state.scanStartedAt = &now
		s.addLog("scanner", "info", "Scan started.")
	case "pause_scan":
		if s.state.mode != "scanning" || s.state.scanStartedAt == nil {
			return Snapshot{}, errors.New("the scanner is not currently scanning")
		}
		s.state.scanAccumulated += time.Since(*s.state.scanStartedAt).Seconds()
		s.state.scanStartedAt = nil
		s.state.mode = "paused"
		s.addLog("scanner", "warn", "Scan paused.")
	case "resume_scan":
		if s.state.mode != "paused" {
			return Snapshot{}, errors.New("the scanner is not paused")
		}
		now := time.Now().UTC()
		s.state.mode = "scanning"
		s.state.scanStartedAt = &now
		s.addLog("scanner", "info", "Scan resumed.")
	case "stop_scan":
		if s.state.mode != "scanning" && s.state.mode != "paused" {
			return Snapshot{}, errors.New("no scan is active")
		}
		s.state.mode = "idle"
		s.state.scanStartedAt = nil
		s.state.scanAccumulated = 0
		s.addLog("scanner", "warn", "Scan stopped.")
	case "estop":
		s.state.mode = "fault"
		s.state.scanStartedAt = nil
		s.state.scanAccumulated = 0
		s.state.faults = []string{"Emergency stop asserted"}
		s.addLog("safety", "error", "Emergency stop triggered.")
	case "clear_fault":
		s.state.faults = []string{}
		s.state.mode = "idle"
		s.addLog("safety", "info", "Fault state cleared.")
	default:
		return Snapshot{}, errors.New("unsupported command '" + command + "'")
	}

	s.updateLocked()
	return s.snapshotLocked(), nil
}

func (s *Service) updateLocked() {
	s.clearExpiredLeaseLocked()
	s.state.scanDurationSeconds = scanDurationForResolution(s.state.scanSettings.Resolution)

	switch s.state.mode {
	case "scanning":
		if s.state.scanStartedAt != nil {
			elapsed := s.state.scanAccumulated + time.Since(*s.state.scanStartedAt).Seconds()
			progress := clamp(elapsed/s.state.scanDurationSeconds, 0, 1)
			targetFilled := int(math.Floor(progress * float64(gridWidth*gridHeight)))
			s.fillToLocked(targetFilled)
			s.state.scanProgress = round4(progress)
			s.setHeadLocked(s.state.filledCells)

			if progress >= 1 {
				now := time.Now().UTC()
				s.state.mode = "idle"
				s.state.scanStartedAt = nil
				s.state.scanAccumulated = 0
				s.state.lastCompletedScanAt = &now
				s.addLog("scanner", "info", "Scan complete. Surface model updated.")
			}
		}
	case "paused":
		if s.state.scanDurationSeconds > 0 {
			s.state.scanProgress = round4(clamp(s.state.scanAccumulated/s.state.scanDurationSeconds, 0, 1))
		}
	}

	s.updateMetricsLocked()
}

func (s *Service) snapshotLocked() Snapshot {
	return Snapshot{
		Connected:             s.state.connected,
		Mode:                  s.state.mode,
		ControlOwner:          s.state.controlOwner,
		ControlLeaseExpiresAt: timeStringPtr(s.state.controlLeaseExpiry),
		Yaw:                   s.state.yaw,
		Pitch:                 s.state.pitch,
		Coverage:              s.state.coverage,
		ScanProgress:          s.state.scanProgress,
		ScanDurationSeconds:   s.state.scanDurationSeconds,
		LastCompletedScanAt:   timeStringPtr(s.state.lastCompletedScanAt),
		ScanSettings:          s.state.scanSettings,
		Metrics:               s.state.metrics,
		Faults:                cloneStrings(s.state.faults),
		Activity:              cloneActivity(s.state.activity),
		Grid:                  cloneGrid(s.state.grid),
	}
}

func (s *Service) clearExpiredLeaseLocked() {
	if s.state.controlLeaseExpiry == nil {
		return
	}
	if !s.state.controlLeaseExpiry.After(time.Now().UTC()) {
		owner := s.state.controlOwner
		s.state.controlOwner = ""
		s.state.controlLeaseExpiry = nil
		if owner != "" {
			s.addLog("control", "warn", "Control lease expired for "+owner+".")
		}
	}
}

func (s *Service) requireControlLocked(user string) error {
	s.clearExpiredLeaseLocked()
	if user == "" {
		return errors.New("a user name is required")
	}
	if s.state.controlOwner != user {
		return errors.New("control is currently held by " + s.state.controlOwner)
	}

	expiry := time.Now().UTC().Add(leaseSeconds * time.Second)
	s.state.controlLeaseExpiry = &expiry
	return nil
}

func (s *Service) resetScanLocked() {
	s.state.grid = newGrid()
	s.state.coverage = 0
	s.state.scanProgress = 0
	s.state.filledCells = 0
	s.state.scanAccumulated = 0
	s.state.scanStartedAt = nil
	s.state.lastCompletedScanAt = nil
}

func (s *Service) addLog(source, level, message string) {
	entry := ActivityEntry{
		Source:  source,
		TS:      time.Now().UTC().Format(time.RFC3339Nano),
		Message: message,
		Level:   level,
	}
	s.state.activity = append([]ActivityEntry{entry}, s.state.activity...)
	if len(s.state.activity) > 20 {
		s.state.activity = s.state.activity[:20]
	}
}

func (s *Service) fillToLocked(target int) {
	maxCount := gridWidth * gridHeight
	target = int(clamp(float64(target), 0, float64(maxCount)))

	for i := s.state.filledCells; i < target; i++ {
		x, y := coordForIndex(i)
		s.state.grid[y][x] = sampleHeight(x, y)
	}

	s.state.filledCells = target
	s.state.coverage = round4(float64(target) / float64(maxCount))
}

func (s *Service) setHeadLocked(index int) {
	maxIndex := (gridWidth * gridHeight) - 1
	if index > maxIndex {
		index = maxIndex
	}

	x, y := coordForIndex(index)
	yawRange := s.state.scanSettings.YawMax - s.state.scanSettings.YawMin
	pitchRange := s.state.scanSettings.PitchMax - s.state.scanSettings.PitchMin

	s.state.yaw = round2(s.state.scanSettings.YawMin + (float64(x)/float64(max(1, gridWidth-1)))*yawRange)
	s.state.pitch = round2(s.state.scanSettings.PitchMin + (float64(y)/float64(max(1, gridHeight-1)))*pitchRange)
}

func (s *Service) updateMetricsLocked() {
	phase := float64(time.Now().UTC().UnixMilli()) / 1000.0
	scanLoad := 0.0
	if s.state.mode == "scanning" {
		scanLoad = 1.0
	}

	s.state.metrics.MotorTempC = round1(31.0 + (math.Sin(phase*0.35) * 1.8) + (scanLoad * 3.0))
	s.state.metrics.MotorCurrentA = round2(1.2 + (scanLoad * 0.45) + ((1.0 - scanLoad) * 0.08) + (math.Cos(phase*0.52) * 0.08))
	s.state.metrics.LidarFPS = int(math.Round(18 + (scanLoad * 6.0) + (math.Sin(phase) * 1.5)))
	s.state.metrics.RadarFPS = int(math.Round(11 + (scanLoad * 4.0) + (math.Cos(phase*0.7) * 1.2)))
	s.state.metrics.LatencyMS = int(math.Round(36 + (scanLoad * 14.0) + ((1.0 - scanLoad) * 4.0) + (math.Abs(math.Sin(phase*0.45)) * 8)))
}

func newGrid() [][]float64 {
	grid := make([][]float64, gridHeight)
	for y := range grid {
		grid[y] = make([]float64, gridWidth)
		for x := range grid[y] {
			grid[y][x] = -1
		}
	}
	return grid
}

func cloneGrid(in [][]float64) [][]float64 {
	out := make([][]float64, len(in))
	for i := range in {
		out[i] = append([]float64(nil), in[i]...)
	}
	return out
}

func cloneStrings(in []string) []string {
	if len(in) == 0 {
		return []string{}
	}
	return append([]string{}, in...)
}

func cloneActivity(in []ActivityEntry) []ActivityEntry {
	if len(in) == 0 {
		return []ActivityEntry{}
	}
	return append([]ActivityEntry{}, in...)
}

func coordForIndex(index int) (int, int) {
	row := index / gridWidth
	col := index % gridWidth
	if row%2 == 1 {
		col = (gridWidth - 1) - col
	}
	return col, row
}

func sampleHeight(x, y int) float64 {
	xf := (float64(x) / float64(max(1, gridWidth-1))) * 4.6
	yf := (float64(y) / float64(max(1, gridHeight-1))) * 3.2

	value := 0.28 + (float64((gridHeight-1)-y)/float64(max(1, gridHeight-1)))*0.34
	value += math.Exp(-(math.Pow(xf-2.25, 2.0) * 1.55)) * 0.52
	value += (math.Sin(xf*2.1) * 0.11) + (math.Cos(yf*3.5) * 0.07)
	value -= math.Exp(-((math.Pow(xf-3.3, 2.0) + math.Pow(yf-1.25, 2.0)) * 3.8)) * 0.21

	return round4(clamp(value, 0, 1))
}

func scanDurationForResolution(resolution string) float64 {
	switch resolution {
	case "low":
		return 20
	case "high":
		return 56
	default:
		return 36
	}
}

func timeStringPtr(t *time.Time) *string {
	if t == nil {
		return nil
	}
	value := t.UTC().Format(time.RFC3339Nano)
	return &value
}

func stringFromMap(m map[string]any, key string) string {
	if m == nil {
		return ""
	}
	if value, ok := m[key]; ok {
		if str, ok := value.(string); ok {
			return str
		}
	}
	return ""
}

func floatFromMap(m map[string]any, key string) (float64, error) {
	if m == nil {
		return 0, errors.New("missing payload value '" + key + "'")
	}

	value, ok := m[key]
	if !ok {
		return 0, errors.New("missing payload value '" + key + "'")
	}

	switch v := value.(type) {
	case float64:
		return v, nil
	case float32:
		return float64(v), nil
	case int:
		return float64(v), nil
	case int64:
		return float64(v), nil
	default:
		return 0, errors.New("payload value '" + key + "' must be numeric")
	}
}

func formatFloat(v float64) string {
	return strings.TrimRight(strings.TrimRight(strconv.FormatFloat(v, 'f', 2, 64), "0"), ".")
}

func clamp(value, minValue, maxValue float64) float64 {
	if value < minValue {
		return minValue
	}
	if value > maxValue {
		return maxValue
	}
	return value
}

func round1(v float64) float64 { return math.Round(v*10) / 10 }
func round2(v float64) float64 { return math.Round(v*100) / 100 }
func round4(v float64) float64 { return math.Round(v*10000) / 10000 }

func max(a, b int) int {
	if a > b {
		return a
	}
	return b
}
