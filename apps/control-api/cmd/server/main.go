package main

import (
	"encoding/json"
	"errors"
	"log"
	"net/http"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"time"

	"cliffscanner/control-api/internal/edgeclient"
	"cliffscanner/control-api/internal/scanner"
)

type apiService interface {
	// Snapshot returns current state without a grid payload (used for error
	// responses and command/lease decoration). SnapshotDelta is the polled path:
	// it carries an incremental GridUpdate for the client's since/generation.
	Snapshot() scanner.Snapshot
	SnapshotDelta(sinceVersion, gen uint64) scanner.Snapshot
	Acquire(user string) (scanner.Snapshot, error)
	Release(user string) (scanner.Snapshot, error)
	Command(user, command string, payload map[string]any) (scanner.Snapshot, error)
	MotionConfig() (scanner.MotionConfig, error)
	UpdateMotionConfig(user string, cfg scanner.MotionConfig) (scanner.MotionConfig, error)
}

type acquireRequest struct {
	User string `json:"user"`
}

type commandRequest struct {
	User    string         `json:"user"`
	Command string         `json:"command"`
	Payload map[string]any `json:"payload"`
}

// motionConfigRequest is the PUT payload for /api/config/motion. User is
// required so the sim service can honour the same lease semantics as the
// edge-backed service. Motion carries the envelope itself.
type motionConfigRequest struct {
	User   string               `json:"user"`
	Motion scanner.MotionConfig `json:"motion"`
}

func main() {
	port := envOrDefault("PORT", "8080")
	listenAddr := envOrDefault("HTTP_BIND", ":"+port)
	repoRoot := envOrDefault("REPO_ROOT", guessRepoRoot())
	uiRoot := envOrDefault("UI_ROOT", filepath.Join(repoRoot, "apps", "web-ui"))

	service := buildService()
	mux := http.NewServeMux()

	mux.HandleFunc("/", serveFile(filepath.Join(uiRoot, "index.html"), "text/html; charset=utf-8"))
	mux.HandleFunc("/styles.css", serveFile(filepath.Join(uiRoot, "styles.css"), "text/css; charset=utf-8"))
	mux.HandleFunc("/app.js", serveFile(filepath.Join(uiRoot, "app.js"), "application/javascript; charset=utf-8"))
	mux.HandleFunc("/healthz", func(w http.ResponseWriter, r *http.Request) {
		if r.Method != http.MethodGet {
			writeJSON(w, http.StatusMethodNotAllowed, map[string]any{"error": "Method not allowed."})
			return
		}
		writeJSON(w, http.StatusOK, map[string]any{"ok": true})
	})

	mux.HandleFunc("/api/state", func(w http.ResponseWriter, r *http.Request) {
		if r.Method != http.MethodGet {
			writeJSON(w, http.StatusMethodNotAllowed, map[string]any{"error": "Method not allowed."})
			return
		}
		since, gen := parseGridCursor(r)
		writeJSON(w, http.StatusOK, service.SnapshotDelta(since, gen))
	})

	mux.HandleFunc("/api/control/acquire", func(w http.ResponseWriter, r *http.Request) {
		if r.Method != http.MethodPost {
			writeJSON(w, http.StatusMethodNotAllowed, map[string]any{"error": "Method not allowed."})
			return
		}

		var req acquireRequest
		if err := decodeJSON(w, r, &req); err != nil {
			writeJSON(w, http.StatusBadRequest, map[string]any{"error": err.Error(), "state": service.Snapshot()})
			return
		}

		snapshot, err := service.Acquire(req.User)
		if err != nil {
			writeJSON(w, http.StatusConflict, map[string]any{"error": err.Error(), "state": service.Snapshot()})
			return
		}
		writeJSON(w, http.StatusOK, map[string]any{"ok": true, "state": snapshot})
	})

	mux.HandleFunc("/api/control/release", func(w http.ResponseWriter, r *http.Request) {
		if r.Method != http.MethodPost {
			writeJSON(w, http.StatusMethodNotAllowed, map[string]any{"error": "Method not allowed."})
			return
		}

		var req acquireRequest
		if err := decodeJSON(w, r, &req); err != nil {
			writeJSON(w, http.StatusBadRequest, map[string]any{"error": err.Error(), "state": service.Snapshot()})
			return
		}

		snapshot, err := service.Release(req.User)
		if err != nil {
			writeJSON(w, http.StatusConflict, map[string]any{"error": err.Error(), "state": service.Snapshot()})
			return
		}
		writeJSON(w, http.StatusOK, map[string]any{"ok": true, "state": snapshot})
	})

	mux.HandleFunc("/api/command", func(w http.ResponseWriter, r *http.Request) {
		if r.Method != http.MethodPost {
			writeJSON(w, http.StatusMethodNotAllowed, map[string]any{"error": "Method not allowed."})
			return
		}

		var req commandRequest
		if err := decodeJSON(w, r, &req); err != nil {
			writeJSON(w, http.StatusBadRequest, map[string]any{"error": err.Error(), "state": service.Snapshot()})
			return
		}

		snapshot, err := service.Command(req.User, req.Command, req.Payload)
		if err != nil {
			writeJSON(w, http.StatusConflict, map[string]any{"error": err.Error(), "state": service.Snapshot()})
			return
		}
		writeJSON(w, http.StatusOK, map[string]any{"ok": true, "state": snapshot})
	})

	// Motion envelope: read is public (so the UI can render current limits on
	// load without acquiring the lease); write is lease-gated by the service.
	mux.HandleFunc("/api/config/motion", func(w http.ResponseWriter, r *http.Request) {
		switch r.Method {
		case http.MethodGet:
			cfg, err := service.MotionConfig()
			if err != nil {
				writeJSON(w, http.StatusBadGateway, map[string]any{"error": err.Error()})
				return
			}
			writeJSON(w, http.StatusOK, cfg)
		case http.MethodPut:
			var req motionConfigRequest
			if err := decodeJSON(w, r, &req); err != nil {
				writeJSON(w, http.StatusBadRequest, map[string]any{"error": err.Error()})
				return
			}
			cfg, err := service.UpdateMotionConfig(req.User, req.Motion)
			if err != nil {
				writeJSON(w, http.StatusConflict, map[string]any{"error": err.Error()})
				return
			}
			writeJSON(w, http.StatusOK, map[string]any{"ok": true, "motion": cfg})
		default:
			writeJSON(w, http.StatusMethodNotAllowed, map[string]any{"error": "Method not allowed."})
		}
	})

	// Explicit timeouts: a default http.Server has none, leaving it open to
	// slowloris (a client that dribbles a request out forever ties up a
	// connection). WriteTimeout is generous because some responses proxy to the
	// edge daemon. MaxHeaderBytes bounds the header buffer (B5).
	srv := &http.Server{
		Addr:              listenAddr,
		Handler:           mux,
		ReadHeaderTimeout: 5 * time.Second,
		ReadTimeout:       10 * time.Second,
		WriteTimeout:      15 * time.Second,
		IdleTimeout:       60 * time.Second,
		MaxHeaderBytes:    1 << 16, // 64 KiB
	}

	log.Printf("Go control API listening on http://%s\n", listenAddr)
	if err := srv.ListenAndServe(); err != nil {
		log.Fatal(err)
	}
}

func buildService() apiService {
	mode := strings.ToLower(strings.TrimSpace(envOrDefault("SCANNER_BACKEND", "sim")))
	if mode == "edge" {
		baseURL := envOrDefault("EDGE_DAEMON_BASE_URL", "http://127.0.0.1:9090")
		return scanner.NewEdgeService(edgeclient.New(baseURL))
	}
	return scanner.NewService()
}

func serveFile(path, contentType string) http.HandlerFunc {
	return func(w http.ResponseWriter, r *http.Request) {
		if _, err := os.Stat(path); err != nil {
			writeJSON(w, http.StatusNotFound, map[string]any{"error": "File not found."})
			return
		}
		w.Header().Set("Content-Type", contentType)
		http.ServeFile(w, r, path)
	}
}

// maxRequestBodyBytes caps the JSON body we'll read on any control endpoint.
// Control payloads (lease user, command, motion envelope) are a few hundred
// bytes; this is generous headroom while closing the unbounded-read DoS (B5).
const maxRequestBodyBytes = 1 << 20 // 1 MiB

func decodeJSON(w http.ResponseWriter, r *http.Request, dst any) error {
	defer r.Body.Close()

	if r.ContentLength == 0 {
		return errors.New("request body is required")
	}

	// MaxBytesReader caps the bytes read and, with the ResponseWriter, marks the
	// connection to close once the limit is hit — so a client can't stream an
	// unbounded body (or lie about Content-Length) to exhaust memory.
	r.Body = http.MaxBytesReader(w, r.Body, maxRequestBodyBytes)
	decoder := json.NewDecoder(r.Body)
	if err := decoder.Decode(dst); err != nil {
		return err
	}
	return nil
}

func writeJSON(w http.ResponseWriter, status int, body any) {
	// Compact JSON: every response is machine-read (the UI or the edge client),
	// and the grid payloads can be large — pretty-printing wastes bytes and CPU.
	payload, err := json.Marshal(body)
	if err != nil {
		http.Error(w, err.Error(), http.StatusInternalServerError)
		return
	}
	w.Header().Set("Content-Type", "application/json; charset=utf-8")
	w.Header().Set("Cache-Control", "no-store")
	w.WriteHeader(status)
	_, _ = w.Write(payload)
}

// parseGridCursor reads the incremental-grid cursor the UI sends on /api/state.
// Both default to 0 (a fresh client), which the backend treats as "send a full
// grid". Unparseable values fall back to 0.
func parseGridCursor(r *http.Request) (since uint64, gen uint64) {
	q := r.URL.Query()
	if v, err := strconv.ParseUint(q.Get("since"), 10, 64); err == nil {
		since = v
	}
	if v, err := strconv.ParseUint(q.Get("gen"), 10, 64); err == nil {
		gen = v
	}
	return since, gen
}

func envOrDefault(key, fallback string) string {
	if value := os.Getenv(key); value != "" {
		return value
	}
	return fallback
}

func guessRepoRoot() string {
	wd, err := os.Getwd()
	if err != nil {
		return "."
	}
	return filepath.Clean(filepath.Join(wd, "..", "..", ".."))
}
