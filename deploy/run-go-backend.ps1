param(
    [int]$Port = 8080,
    [ValidateSet('sim','edge')]
    [string]$Mode = 'sim',
    [string]$EdgeDaemonBaseUrl = 'http://127.0.0.1:9090'
)

$repoRoot = Split-Path -Parent $PSScriptRoot
$apiRoot = Join-Path $repoRoot "apps\control-api"
$goCache = Join-Path $repoRoot ".gocache"
$goModCache = Join-Path $repoRoot ".gomodcache"

if (-not (Test-Path $apiRoot)) {
    throw "Go backend directory not found at $apiRoot"
}

New-Item -ItemType Directory -Force -Path $goCache | Out-Null
New-Item -ItemType Directory -Force -Path $goModCache | Out-Null

Write-Host "Starting Go control API on http://localhost:$Port in $Mode mode"
Write-Host "Press Ctrl+C to stop."

$env:PORT = "$Port"
$env:HTTP_BIND = ":$Port"
$env:REPO_ROOT = $repoRoot
$env:GOCACHE = $goCache
$env:GOMODCACHE = $goModCache
$env:SCANNER_BACKEND = $Mode
$env:EDGE_DAEMON_BASE_URL = $EdgeDaemonBaseUrl

Push-Location $apiRoot
try {
    go run .\cmd\server
}
finally {
    Pop-Location
}
