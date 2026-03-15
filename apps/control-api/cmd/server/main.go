package main

import (
	"encoding/json"
	"errors"
	"log"
	"net/http"
	"os"
	"path/filepath"
	"strings"

	"cliffscanner/control-api/internal/edgeclient"
	"cliffscanner/control-api/internal/scanner"
)

type apiService interface {
	Snapshot() scanner.Snapshot
	Acquire(user string) (scanner.Snapshot, error)
	Release(user string) (scanner.Snapshot, error)
	Command(user, command string, payload map[string]any) (scanner.Snapshot, error)
}

type acquireRequest struct {
	User string `json:"user"`
}

type commandRequest struct {
	User    string         `json:"user"`
	Command string         `json:"command"`
	Payload map[string]any `json:"payload"`
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
		writeJSON(w, http.StatusOK, service.Snapshot())
	})

	mux.HandleFunc("/api/control/acquire", func(w http.ResponseWriter, r *http.Request) {
		if r.Method != http.MethodPost {
			writeJSON(w, http.StatusMethodNotAllowed, map[string]any{"error": "Method not allowed."})
			return
		}

		var req acquireRequest
		if err := decodeJSON(r, &req); err != nil {
			writeJSON(w, http.StatusConflict, map[string]any{"error": err.Error(), "state": service.Snapshot()})
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
		if err := decodeJSON(r, &req); err != nil {
			writeJSON(w, http.StatusConflict, map[string]any{"error": err.Error(), "state": service.Snapshot()})
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
		if err := decodeJSON(r, &req); err != nil {
			writeJSON(w, http.StatusConflict, map[string]any{"error": err.Error(), "state": service.Snapshot()})
			return
		}

		snapshot, err := service.Command(req.User, req.Command, req.Payload)
		if err != nil {
			writeJSON(w, http.StatusConflict, map[string]any{"error": err.Error(), "state": service.Snapshot()})
			return
		}
		writeJSON(w, http.StatusOK, map[string]any{"ok": true, "state": snapshot})
	})

	log.Printf("Go control API listening on http://%s\n", listenAddr)
	if err := http.ListenAndServe(listenAddr, mux); err != nil {
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

func decodeJSON(r *http.Request, dst any) error {
	defer r.Body.Close()

	if r.ContentLength == 0 {
		return errors.New("request body is required")
	}

	decoder := json.NewDecoder(r.Body)
	if err := decoder.Decode(dst); err != nil {
		return err
	}
	return nil
}

func writeJSON(w http.ResponseWriter, status int, body any) {
	payload, err := json.MarshalIndent(body, "", "  ")
	if err != nil {
		http.Error(w, err.Error(), http.StatusInternalServerError)
		return
	}
	w.Header().Set("Content-Type", "application/json; charset=utf-8")
	w.Header().Set("Cache-Control", "no-store")
	w.WriteHeader(status)
	_, _ = w.Write(payload)
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
