
param(
    [int]$Port = 8080,
    [string]$RepoRoot = (Split-Path -Parent (Split-Path -Parent $PSScriptRoot))
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$uiRoot = Join-Path $RepoRoot 'apps\web-ui'
if (-not (Test-Path $uiRoot)) { throw "UI directory not found at $uiRoot" }

$script:gridW = 48
$script:gridH = 24
$script:leaseSeconds = 120

function New-Grid {
    $grid = @()
    for ($y = 0; $y -lt $script:gridH; $y++) {
        $row = @()
        for ($x = 0; $x -lt $script:gridW; $x++) { $row += -1.0 }
        $grid += ,$row
    }
    $grid
}

function Sample-Height([int]$x, [int]$y) {
    $xf = ($x / [double]([Math]::Max(1, $script:gridW - 1))) * 4.6
    $yf = ($y / [double]([Math]::Max(1, $script:gridH - 1))) * 3.2
    $value = 0.28 + ((($script:gridH - 1) - $y) / [double]([Math]::Max(1, $script:gridH - 1))) * 0.34
    $value += [Math]::Exp(-([Math]::Pow($xf - 2.25, 2.0) * 1.55)) * 0.52
    $value += ([Math]::Sin($xf * 2.1) * 0.11) + ([Math]::Cos($yf * 3.5) * 0.07)
    $value -= [Math]::Exp(-((([Math]::Pow($xf - 3.3, 2.0) + [Math]::Pow($yf - 1.25, 2.0)) * 3.8))) * 0.21
    [Math]::Round([Math]::Max(0.0, [Math]::Min(1.0, $value)), 4)
}

function Add-Log([string]$source, [string]$level, [string]$message) {
    $entry = @{ ts = [DateTimeOffset]::UtcNow.ToString('o'); source = $source; level = $level; message = $message }
    $script:state.activity = @($entry) + @($script:state.activity)
    if ($script:state.activity.Count -gt 20) { $script:state.activity = $script:state.activity[0..19] }
}

$script:state = @{
    connected = $true
    mode = 'idle'
    controlOwner = $null
    controlLeaseExpiresAt = $null
    yaw = 0.0
    pitch = 0.0
    coverage = 0.0
    scanProgress = 0.0
    scanDurationSeconds = 36.0
    scanStartedAt = $null
    scanAccumulatedSeconds = 0.0
    filledCells = 0
    lastCompletedScanAt = $null
    grid = New-Grid
    scanSettings = @{ yawMin = -60; yawMax = 60; pitchMin = -20; pitchMax = 35; sweepSpeedDegPerSec = 20; resolution = 'medium' }
    metrics = @{ motorTempC = 31.4; motorCurrentA = 1.3; lidarFps = 18; radarFps = 11; latencyMs = 42; packetsDropped = 0 }
    faults = @()
    activity = @()
}
Add-Log 'system' 'info' 'Prototype scanner server initialized.'
Add-Log 'scanner' 'info' 'Scanner connected to simulated control bus.'

function Clear-ExpiredLease {
    if ($null -eq $script:state.controlLeaseExpiresAt) { return }
    $expiry = [DateTimeOffset]::Parse($script:state.controlLeaseExpiresAt)
    if ($expiry -le [DateTimeOffset]::UtcNow) {
        $owner = $script:state.controlOwner
        $script:state.controlOwner = $null
        $script:state.controlLeaseExpiresAt = $null
        if ($owner) { Add-Log 'control' 'warn' "Control lease expired for $owner." }
    }
}

function Get-ScanDuration {
    switch ($script:state.scanSettings.resolution) {
        'low' { 20.0 }
        'high' { 56.0 }
        default { 36.0 }
    }
}

function Get-Coord([int]$index) {
    $row = [Math]::Floor($index / $script:gridW)
    $col = $index % $script:gridW
    if (($row % 2) -eq 1) { $col = ($script:gridW - 1) - $col }
    @{ x = [int]$col; y = [int]$row }
}

function Set-Head([int]$index) {
    $coord = Get-Coord ([Math]::Min($index, ($script:gridW * $script:gridH) - 1))
    $yawRange = $script:state.scanSettings.yawMax - $script:state.scanSettings.yawMin
    $pitchRange = $script:state.scanSettings.pitchMax - $script:state.scanSettings.pitchMin
    $script:state.yaw = [Math]::Round($script:state.scanSettings.yawMin + (($coord.x / [double]([Math]::Max(1, $script:gridW - 1))) * $yawRange), 2)
    $script:state.pitch = [Math]::Round($script:state.scanSettings.pitchMin + (($coord.y / [double]([Math]::Max(1, $script:gridH - 1))) * $pitchRange), 2)
}
function Fill-To([int]$target) {
    $max = $script:gridW * $script:gridH
    $target = [Math]::Max(0, [Math]::Min($target, $max))
    for ($i = $script:state.filledCells; $i -lt $target; $i++) {
        $coord = Get-Coord $i
        $script:state.grid[$coord.y][$coord.x] = Sample-Height $coord.x $coord.y
    }
    $script:state.filledCells = $target
    $script:state.coverage = [Math]::Round($target / [double]$max, 4)
}

function Update-Metrics {
    $phase = [DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds() / 1000.0
    $scanLoad = if ($script:state.mode -eq 'scanning') { 1.0 } else { 0.0 }
    $script:state.metrics.motorTempC = [Math]::Round(31.0 + ([Math]::Sin($phase * 0.35) * 1.8) + ($scanLoad * 3.0), 1)
    $script:state.metrics.motorCurrentA = [Math]::Round(1.2 + ($scanLoad * 0.45) + ((1.0 - $scanLoad) * 0.08) + ([Math]::Cos($phase * 0.52) * 0.08), 2)
    $script:state.metrics.lidarFps = [int][Math]::Round(18 + ($scanLoad * 6.0) + ([Math]::Sin($phase) * 1.5))
    $script:state.metrics.radarFps = [int][Math]::Round(11 + ($scanLoad * 4.0) + ([Math]::Cos($phase * 0.7) * 1.2))
    $script:state.metrics.latencyMs = [int][Math]::Round(36 + ($scanLoad * 14.0) + ((1.0 - $scanLoad) * 4.0) + ([Math]::Abs([Math]::Sin($phase * 0.45)) * 8))
}

function Update-State {
    Clear-ExpiredLease
    $script:state.scanDurationSeconds = Get-ScanDuration
    if ($script:state.mode -eq 'scanning' -and $null -ne $script:state.scanStartedAt) {
        $started = [DateTimeOffset]::Parse($script:state.scanStartedAt)
        $elapsed = $script:state.scanAccumulatedSeconds + ([DateTimeOffset]::UtcNow - $started).TotalSeconds
        $progress = [Math]::Max(0.0, [Math]::Min(1.0, $elapsed / $script:state.scanDurationSeconds))
        Fill-To ([int][Math]::Floor($progress * ($script:gridW * $script:gridH)))
        $script:state.scanProgress = [Math]::Round($progress, 4)
        Set-Head $script:state.filledCells
        if ($progress -ge 1.0) {
            $script:state.mode = 'idle'
            $script:state.scanStartedAt = $null
            $script:state.scanAccumulatedSeconds = 0.0
            $script:state.lastCompletedScanAt = [DateTimeOffset]::UtcNow.ToString('o')
            Add-Log 'scanner' 'info' 'Scan complete. Surface model updated.'
        }
    } elseif ($script:state.mode -eq 'paused') {
        $script:state.scanProgress = [Math]::Round([Math]::Max(0.0, [Math]::Min(1.0, $script:state.scanAccumulatedSeconds / $script:state.scanDurationSeconds)), 4)
    }
    Update-Metrics
}

function Get-Snapshot {
    Update-State
    @{
        connected = $script:state.connected
        mode = $script:state.mode
        controlOwner = $script:state.controlOwner
        controlLeaseExpiresAt = $script:state.controlLeaseExpiresAt
        yaw = $script:state.yaw
        pitch = $script:state.pitch
        coverage = $script:state.coverage
        scanProgress = $script:state.scanProgress
        scanDurationSeconds = $script:state.scanDurationSeconds
        lastCompletedScanAt = $script:state.lastCompletedScanAt
        scanSettings = $script:state.scanSettings
        metrics = $script:state.metrics
        faults = $script:state.faults
        activity = $script:state.activity
        grid = $script:state.grid
    }
}

function Require-Control([string]$user) {
    Clear-ExpiredLease
    if ([string]::IsNullOrWhiteSpace($user)) { throw 'A user name is required.' }
    if ($script:state.controlOwner -ne $user) { throw "Control is currently held by $($script:state.controlOwner)." }
    $script:state.controlLeaseExpiresAt = [DateTimeOffset]::UtcNow.AddSeconds($script:leaseSeconds).ToString('o')
}

function Reset-Scan {
    $script:state.grid = New-Grid
    $script:state.coverage = 0.0
    $script:state.scanProgress = 0.0
    $script:state.filledCells = 0
    $script:state.scanAccumulatedSeconds = 0.0
    $script:state.scanStartedAt = $null
    $script:state.lastCompletedScanAt = $null
}

function Handle-Acquire($body) {
    $user = "$($body.user)".Trim()
    if ([string]::IsNullOrWhiteSpace($user)) { throw 'User is required to acquire control.' }
    Clear-ExpiredLease
    if ($null -ne $script:state.controlOwner -and $script:state.controlOwner -ne $user) { throw "Control is already held by $($script:state.controlOwner)." }
    $script:state.controlOwner = $user
    $script:state.controlLeaseExpiresAt = [DateTimeOffset]::UtcNow.AddSeconds($script:leaseSeconds).ToString('o')
    Add-Log 'control' 'info' "$user acquired control."
}

function Handle-Release($body) {
    $user = "$($body.user)".Trim()
    Require-Control $user
    $script:state.controlOwner = $null
    $script:state.controlLeaseExpiresAt = $null
    Add-Log 'control' 'info' "$user released control."
}
function Handle-Command($body) {
    $user = "$($body.user)".Trim()
    $command = "$($body.command)".Trim().ToLowerInvariant()
    $payload = if ($body.ContainsKey('payload')) { $body.payload } else { @{} }
    if ([string]::IsNullOrWhiteSpace($command)) { throw 'Command is required.' }
    if ($command -ne 'connect') { Require-Control $user }
    switch ($command) {
        'connect' { $script:state.connected = $true; Add-Log 'scanner' 'info' 'Scanner transport connected.' }
        'home' {
            if (-not $script:state.connected) { throw 'Scanner is not connected.' }
            if ($script:state.mode -eq 'fault') { throw 'Clear the fault before homing.' }
            $script:state.mode = 'idle'; $script:state.yaw = 0.0; $script:state.pitch = 0.0
            Add-Log 'motion' 'info' 'Gantry returned to home position.'
        }
        'jog' {
            if ($script:state.mode -eq 'scanning') { throw 'Pause the scan before jogging.' }
            $axis = "$($payload.axis)".Trim().ToLowerInvariant()
            $delta = [double]$payload.delta
            if ($axis -eq 'yaw') { $script:state.yaw = [Math]::Round([Math]::Max($script:state.scanSettings.yawMin, [Math]::Min($script:state.scanSettings.yawMax, $script:state.yaw + $delta)), 2) }
            elseif ($axis -eq 'pitch') { $script:state.pitch = [Math]::Round([Math]::Max($script:state.scanSettings.pitchMin, [Math]::Min($script:state.scanSettings.pitchMax, $script:state.pitch + $delta)), 2) }
            else { throw "Unsupported jog axis '$axis'." }
            $script:state.mode = 'manual'
            Add-Log 'motion' 'info' "Jogged $axis by $delta degrees."
        }
        'set_resolution' {
            $resolution = "$($payload.resolution)".Trim().ToLowerInvariant()
            if ($resolution -notin @('low','medium','high')) { throw 'Resolution must be low, medium, or high.' }
            $script:state.scanSettings.resolution = $resolution
            Add-Log 'scanner' 'info' "Scan resolution set to $resolution."
        }
        'start_scan' {
            if (-not $script:state.connected) { throw 'Scanner is not connected.' }
            if ($script:state.mode -eq 'fault') { throw 'Clear the fault before starting a scan.' }
            Reset-Scan
            $script:state.mode = 'scanning'
            $script:state.scanStartedAt = [DateTimeOffset]::UtcNow.ToString('o')
            Add-Log 'scanner' 'info' 'Scan started.'
        }
        'pause_scan' {
            if ($script:state.mode -ne 'scanning') { throw 'The scanner is not currently scanning.' }
            $started = [DateTimeOffset]::Parse($script:state.scanStartedAt)
            $script:state.scanAccumulatedSeconds += ([DateTimeOffset]::UtcNow - $started).TotalSeconds
            $script:state.scanStartedAt = $null
            $script:state.mode = 'paused'
            Add-Log 'scanner' 'warn' 'Scan paused.'
        }
        'resume_scan' {
            if ($script:state.mode -ne 'paused') { throw 'The scanner is not paused.' }
            $script:state.mode = 'scanning'
            $script:state.scanStartedAt = [DateTimeOffset]::UtcNow.ToString('o')
            Add-Log 'scanner' 'info' 'Scan resumed.'
        }
        'stop_scan' {
            if ($script:state.mode -notin @('scanning','paused')) { throw 'No scan is active.' }
            $script:state.mode = 'idle'
            $script:state.scanStartedAt = $null
            $script:state.scanAccumulatedSeconds = 0.0
            Add-Log 'scanner' 'warn' 'Scan stopped.'
        }
        'estop' {
            $script:state.mode = 'fault'
            $script:state.scanStartedAt = $null
            $script:state.scanAccumulatedSeconds = 0.0
            $script:state.faults = @('Emergency stop asserted')
            Add-Log 'safety' 'error' 'Emergency stop triggered.'
        }
        'clear_fault' {
            $script:state.faults = @()
            $script:state.mode = 'idle'
            Add-Log 'safety' 'info' 'Fault state cleared.'
        }
        default { throw "Unsupported command '$command'." }
    }
}

function Convert-ObjectToHashtable($value) {
    if ($null -eq $value) { return $null }
    if ($value -is [string] -or $value -is [int] -or $value -is [double] -or $value -is [bool]) { return $value }
    if ($value -is [pscustomobject]) {
        $hash = @{}
        foreach ($prop in $value.PSObject.Properties) {
            $hash[$prop.Name] = Convert-ObjectToHashtable $prop.Value
        }
        return $hash
    }
    return $value
}

function Get-JsonBody($req) {
    if ([string]::IsNullOrWhiteSpace($req.Body)) { return @{} }
    $parsed = ConvertFrom-Json -InputObject $req.Body
    $result = @{}
    foreach ($prop in $parsed.PSObject.Properties) {
        $result[$prop.Name] = Convert-ObjectToHashtable $prop.Value
    }
    return $result
}

function Read-Request([System.Net.Sockets.TcpClient]$client) {
    $stream = $client.GetStream()
    $reader = New-Object System.IO.StreamReader($stream, [System.Text.Encoding]::UTF8, $false, 4096, $true)
    $requestLine = $reader.ReadLine()
    if ([string]::IsNullOrWhiteSpace($requestLine)) { return $null }
    $parts = $requestLine.Split(' ')
    if ($parts.Count -lt 2) { throw 'Malformed HTTP request line.' }
    $headers = @{}
    while ($true) {
        $line = $reader.ReadLine()
        if ($null -eq $line -or $line.Length -eq 0) { break }
        $separator = $line.IndexOf(':')
        if ($separator -gt 0) { $headers[$line.Substring(0, $separator).Trim().ToLowerInvariant()] = $line.Substring($separator + 1).Trim() }
    }
    $body = ''
    if ($headers.ContainsKey('content-length')) {
        $length = [int]$headers['content-length']
        if ($length -gt 0) {
            $buffer = New-Object char[] $length
            $offset = 0
            while ($offset -lt $length) {
                $read = $reader.Read($buffer, $offset, $length - $offset)
                if ($read -le 0) { break }
                $offset += $read
            }
            if ($offset -gt 0) { $body = -join $buffer[0..($offset - 1)] }
        }
    }
    $pathValue = $parts[1]
    if ($pathValue.Contains('?') ) { $pathValue = $pathValue.Split('?')[0] }
    @{ Method = $parts[0].ToUpperInvariant(); Path = [System.Uri]::UnescapeDataString($pathValue); Headers = $headers; Body = $body; Stream = $stream; Reader = $reader }
}

function Write-Response([System.IO.Stream]$stream, [int]$status, [string]$contentType, [byte[]]$bodyBytes) {
    $texts = @{ 200 = 'OK'; 404 = 'Not Found'; 405 = 'Method Not Allowed'; 409 = 'Conflict' }
    $writer = New-Object System.IO.StreamWriter($stream, [System.Text.Encoding]::ASCII, 1024, $true)
    try {
        $writer.NewLine = "`r`n"
        $writer.WriteLine("HTTP/1.1 $status $($texts[$status])")
        $writer.WriteLine("Content-Type: $contentType")
        $writer.WriteLine("Content-Length: $($bodyBytes.Length)")
        $writer.WriteLine('Cache-Control: no-store')
        $writer.WriteLine('Connection: close')
        $writer.WriteLine()
        $writer.Flush()
        if ($bodyBytes.Length -gt 0) { $stream.Write($bodyBytes, 0, $bodyBytes.Length) }
        $stream.Flush()
    } finally {
        $writer.Dispose()
    }
}

function Send-Json($req, [int]$status, $body) {
    $json = ConvertTo-Json -InputObject $body -Depth 8
    $bytes = [System.Text.Encoding]::UTF8.GetBytes($json)
    Write-Response $req.Stream $status 'application/json; charset=utf-8' $bytes
}

function Send-File($req, [string]$pathValue, [string]$contentType) {
    if (-not (Test-Path $pathValue)) { Send-Json $req 404 @{ error = 'File not found.' }; return }
    $bytes = [System.IO.File]::ReadAllBytes($pathValue)
    Write-Response $req.Stream 200 $contentType $bytes
}

$listener = [System.Net.Sockets.TcpListener]::new([System.Net.IPAddress]::Any, $Port)
$listener.Start()
Write-Host "Prototype server listening on http://localhost:$Port"

try {
    while ($true) {
        $client = $listener.AcceptTcpClient()
        $req = $null
        try {
            $req = Read-Request $client
            if ($null -eq $req) { continue }
            switch -Regex ($req.Path) {
                '^/$' { Send-File $req (Join-Path $uiRoot 'index.html') 'text/html; charset=utf-8' }
                '^/styles\.css$' { Send-File $req (Join-Path $uiRoot 'styles.css') 'text/css; charset=utf-8' }
                '^/app\.js$' { Send-File $req (Join-Path $uiRoot 'app.js') 'application/javascript; charset=utf-8' }
                '^/api/state$' {
                    if ($req.Method -ne 'GET') { Send-Json $req 405 @{ error = 'Method not allowed.' } }
                    else { Send-Json $req 200 (Get-Snapshot) }
                }
                '^/api/control/acquire$' {
                    if ($req.Method -ne 'POST') { Send-Json $req 405 @{ error = 'Method not allowed.' } }
                    else { $body = Get-JsonBody $req; Handle-Acquire $body; Send-Json $req 200 @{ ok = $true; state = Get-Snapshot } }
                }
                '^/api/control/release$' {
                    if ($req.Method -ne 'POST') { Send-Json $req 405 @{ error = 'Method not allowed.' } }
                    else { $body = Get-JsonBody $req; Handle-Release $body; Send-Json $req 200 @{ ok = $true; state = Get-Snapshot } }
                }
                '^/api/command$' {
                    if ($req.Method -ne 'POST') { Send-Json $req 405 @{ error = 'Method not allowed.' } }
                    else { $body = Get-JsonBody $req; Handle-Command $body; Send-Json $req 200 @{ ok = $true; state = Get-Snapshot } }
                }
                default { Send-Json $req 404 @{ error = 'Not found.' } }
            }
        } catch {
            if ($null -ne $req) {
                try { Send-Json $req 409 @{ error = $_.Exception.Message; state = Get-Snapshot } } catch {}
            }
        } finally {
            if ($null -ne $req -and $null -ne $req.Reader) { $req.Reader.Dispose() }
            $client.Close()
        }
    }
} finally {
    $listener.Stop()
}

