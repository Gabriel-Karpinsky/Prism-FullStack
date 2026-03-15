package scanner

import (
	"errors"
	"strings"
	"sync"
	"time"
)

type HardwareClient interface {
	Snapshot() (Snapshot, error)
	Command(command string, payload map[string]any) (Snapshot, error)
}

type EdgeService struct {
	mu                 sync.Mutex
	client             HardwareClient
	controlOwner       string
	controlLeaseExpiry *time.Time
	activity           []ActivityEntry
	lastSnapshot       Snapshot
	lastError          string
}

func NewEdgeService(client HardwareClient) *EdgeService {
	s := &EdgeService{
		client: client,
		activity: []ActivityEntry{
			{
				Source:  "system",
				TS:      time.Now().UTC().Format(time.RFC3339Nano),
				Message: "Edge-backed control API initialized.",
				Level:   "info",
			},
		},
	}

	if snapshot, err := client.Snapshot(); err == nil {
		s.lastSnapshot = snapshot
	} else {
		s.lastError = err.Error()
	}

	return s
}

func (s *EdgeService) Snapshot() Snapshot {
	s.mu.Lock()
	defer s.mu.Unlock()

	syncErr := s.syncHardwareLocked()
	return s.decorateSnapshotLocked(s.lastSnapshot, syncErr)
}

func (s *EdgeService) Acquire(user string) (Snapshot, error) {
	s.mu.Lock()
	defer s.mu.Unlock()

	user = strings.TrimSpace(user)
	if user == "" {
		return Snapshot{}, errors.New("user is required to acquire control")
	}

	s.clearExpiredLeaseLocked()
	if s.controlOwner != "" && s.controlOwner != user {
		return Snapshot{}, errors.New("control is already held by " + s.controlOwner)
	}

	expiry := time.Now().UTC().Add(leaseSeconds * time.Second)
	s.controlOwner = user
	s.controlLeaseExpiry = &expiry
	s.addLocalLogLocked("control", "info", user+" acquired control.")

	syncErr := s.syncHardwareLocked()
	return s.decorateSnapshotLocked(s.lastSnapshot, syncErr), nil
}

func (s *EdgeService) Release(user string) (Snapshot, error) {
	s.mu.Lock()
	defer s.mu.Unlock()

	if err := s.requireControlLocked(strings.TrimSpace(user)); err != nil {
		return Snapshot{}, err
	}

	s.controlOwner = ""
	s.controlLeaseExpiry = nil
	s.addLocalLogLocked("control", "info", strings.TrimSpace(user)+" released control.")

	syncErr := s.syncHardwareLocked()
	return s.decorateSnapshotLocked(s.lastSnapshot, syncErr), nil
}

func (s *EdgeService) Command(user, command string, payload map[string]any) (Snapshot, error) {
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

	snapshot, err := s.client.Command(command, payload)
	if err != nil {
		s.lastError = err.Error()
		decorated := s.decorateSnapshotLocked(s.lastSnapshot, err)
		return decorated, err
	}

	s.lastSnapshot = snapshot
	s.lastError = ""
	return s.decorateSnapshotLocked(snapshot, nil), nil
}

func (s *EdgeService) syncHardwareLocked() error {
	s.clearExpiredLeaseLocked()

	snapshot, err := s.client.Snapshot()
	if err != nil {
		s.lastError = err.Error()
		return err
	}

	s.lastSnapshot = snapshot
	s.lastError = ""
	return nil
}

func (s *EdgeService) decorateSnapshotLocked(snapshot Snapshot, syncErr error) Snapshot {
	snapshot.ControlOwner = s.controlOwner
	snapshot.ControlLeaseExpiresAt = timeStringPtr(s.controlLeaseExpiry)
	snapshot.Activity = mergeActivity(cloneActivity(s.activity), snapshot.Activity)

	if syncErr != nil {
		snapshot.Connected = false
		snapshot.Faults = append(cloneStrings(snapshot.Faults), "Edge daemon unavailable: "+syncErr.Error())
	}

	return snapshot
}

func (s *EdgeService) clearExpiredLeaseLocked() {
	if s.controlLeaseExpiry == nil {
		return
	}
	if !s.controlLeaseExpiry.After(time.Now().UTC()) {
		owner := s.controlOwner
		s.controlOwner = ""
		s.controlLeaseExpiry = nil
		if owner != "" {
			s.addLocalLogLocked("control", "warn", "Control lease expired for "+owner+".")
		}
	}
}

func (s *EdgeService) requireControlLocked(user string) error {
	s.clearExpiredLeaseLocked()
	if user == "" {
		return errors.New("a user name is required")
	}
	if s.controlOwner != user {
		return errors.New("control is currently held by " + s.controlOwner)
	}

	expiry := time.Now().UTC().Add(leaseSeconds * time.Second)
	s.controlLeaseExpiry = &expiry
	return nil
}

func (s *EdgeService) addLocalLogLocked(source, level, message string) {
	entry := ActivityEntry{
		Source:  source,
		TS:      time.Now().UTC().Format(time.RFC3339Nano),
		Message: message,
		Level:   level,
	}
	s.activity = append([]ActivityEntry{entry}, s.activity...)
	if len(s.activity) > 8 {
		s.activity = s.activity[:8]
	}
}

func mergeActivity(primary, secondary []ActivityEntry) []ActivityEntry {
	merged := append([]ActivityEntry{}, primary...)
	merged = append(merged, secondary...)
	if len(merged) > 20 {
		merged = merged[:20]
	}
	return merged
}
