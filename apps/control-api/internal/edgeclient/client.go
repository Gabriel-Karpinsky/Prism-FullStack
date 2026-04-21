package edgeclient

import (
	"bytes"
	"encoding/json"
	"fmt"
	"io"
	"net/http"
	"strings"
	"time"

	"cliffscanner/control-api/internal/scanner"
)

type Client struct {
	baseURL string
	http    *http.Client
}

type commandResponse struct {
	OK    bool             `json:"ok"`
	State scanner.Snapshot `json:"state"`
	Error string           `json:"error"`
}

func New(baseURL string) *Client {
	trimmed := strings.TrimRight(strings.TrimSpace(baseURL), "/")
	if trimmed == "" {
		trimmed = "http://127.0.0.1:9090"
	}

	return &Client{
		baseURL: trimmed,
		http: &http.Client{
			Timeout: 5 * time.Second,
		},
	}
}

func (c *Client) Snapshot() (scanner.Snapshot, error) {
	var snapshot scanner.Snapshot
	if err := c.getJSON("/api/hardware/state", &snapshot); err != nil {
		return scanner.Snapshot{}, err
	}
	return snapshot, nil
}

func (c *Client) Command(command string, payload map[string]any) (scanner.Snapshot, error) {
	body := map[string]any{"command": command}
	for key, value := range payload {
		body[key] = value
	}

	var response commandResponse
	if err := c.postJSON("/api/hardware/command", body, &response); err != nil {
		return scanner.Snapshot{}, err
	}
	if response.Error != "" {
		return scanner.Snapshot{}, fmt.Errorf(response.Error)
	}
	return response.State, nil
}

// MotionConfig fetches the current motion envelope from the edge daemon.
func (c *Client) MotionConfig() (scanner.MotionConfig, error) {
	var cfg scanner.MotionConfig
	if err := c.getJSON("/api/config/motion", &cfg); err != nil {
		return scanner.MotionConfig{}, err
	}
	return cfg, nil
}

// UpdateMotionConfig sends a PUT /api/config/motion. Either axis may be
// omitted (the daemon merges the payload with the current config); we forward
// the caller's struct verbatim. The daemon validates + persists atomically.
func (c *Client) UpdateMotionConfig(cfg scanner.MotionConfig) (scanner.MotionConfig, error) {
	var response struct {
		OK     bool                 `json:"ok"`
		Error  string               `json:"error"`
		Motion scanner.MotionConfig `json:"motion"`
	}
	if err := c.putJSON("/api/config/motion", cfg, &response); err != nil {
		return scanner.MotionConfig{}, err
	}
	if !response.OK && response.Error != "" {
		return scanner.MotionConfig{}, fmt.Errorf("%s", response.Error)
	}
	return response.Motion, nil
}

func (c *Client) getJSON(path string, dst any) error {
	response, err := c.http.Get(c.baseURL + path)
	if err != nil {
		return err
	}
	defer response.Body.Close()

	if response.StatusCode >= 400 {
		return decodeError(response)
	}

	return json.NewDecoder(response.Body).Decode(dst)
}

func (c *Client) putJSON(path string, body any, dst any) error {
	payload, err := json.Marshal(body)
	if err != nil {
		return err
	}

	req, err := http.NewRequest(http.MethodPut, c.baseURL+path, bytes.NewReader(payload))
	if err != nil {
		return err
	}
	req.Header.Set("Content-Type", "application/json")

	response, err := c.http.Do(req)
	if err != nil {
		return err
	}
	defer response.Body.Close()

	if response.StatusCode >= 400 {
		return decodeError(response)
	}

	return json.NewDecoder(response.Body).Decode(dst)
}

func (c *Client) postJSON(path string, body any, dst any) error {
	payload, err := json.Marshal(body)
	if err != nil {
		return err
	}

	response, err := c.http.Post(c.baseURL+path, "application/json", bytes.NewReader(payload))
	if err != nil {
		return err
	}
	defer response.Body.Close()

	if response.StatusCode >= 400 {
		return decodeError(response)
	}

	return json.NewDecoder(response.Body).Decode(dst)
}

func decodeError(response *http.Response) error {
	body, err := io.ReadAll(response.Body)
	if err != nil {
		return fmt.Errorf("edge daemon returned status %d", response.StatusCode)
	}

	var payload map[string]any
	if err := json.Unmarshal(body, &payload); err == nil {
		if message, ok := payload["error"].(string); ok && message != "" {
			return fmt.Errorf(message)
		}
	}

	message := strings.TrimSpace(string(body))
	if message == "" {
		message = http.StatusText(response.StatusCode)
	}
	return fmt.Errorf("edge daemon returned status %d: %s", response.StatusCode, message)
}
