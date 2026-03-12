param(
    [int]$Port = 8080
)

$repoRoot = Split-Path -Parent $PSScriptRoot
$serverScript = Join-Path $repoRoot "apps\prototype-server\tcp-server.ps1"

if (-not (Test-Path $serverScript)) {
    throw "Prototype server script not found at $serverScript"
}

Write-Host "Starting cliff face scanner prototype on http://localhost:$Port"
Write-Host "Press Ctrl+C to stop."

& powershell -ExecutionPolicy Bypass -File $serverScript -Port $Port -RepoRoot $repoRoot

