param(
    [ValidateSet('Runtime', 'Build')]
    [string] $Purpose = 'Runtime',
    [string[]] $Candidate = @(),
    [switch] $OnlyExplicitCandidates
)

$ErrorActionPreference = 'Continue'
$candidates = [System.Collections.Generic.List[string]]::new()
$seen = [System.Collections.Generic.HashSet[string]]::new(
    [System.StringComparer]::OrdinalIgnoreCase)
$validatedHomes = [System.Collections.Generic.HashSet[string]]::new(
    [System.StringComparer]::OrdinalIgnoreCase)
$rejections = [System.Collections.Generic.List[string]]::new()

function Add-JavaCandidate([string] $path) {
    if ([string]::IsNullOrWhiteSpace($path)) { return }
    try {
        $expanded = [Environment]::ExpandEnvironmentVariables($path.Trim('"'))
        $fullPath = [System.IO.Path]::GetFullPath($expanded)
        if (Test-Path -LiteralPath $fullPath -PathType Container) {
            $fullPath = Join-Path $fullPath 'bin\java.exe'
        }
        if ((Test-Path -LiteralPath $fullPath -PathType Leaf) -and $seen.Add($fullPath)) {
            $candidates.Add($fullPath)
        } elseif (-not (Test-Path -LiteralPath $fullPath -PathType Leaf)) {
            $rejections.Add("$fullPath (bin\java.exe not found)")
        }
    } catch {
        $rejections.Add("$path (invalid path: $($_.Exception.Message))")
    }
}

foreach ($path in $Candidate) { Add-JavaCandidate $path }

if (-not $OnlyExplicitCandidates) {
    # Prefer a bundled runtime for users, then their explicit configuration.
    Add-JavaCandidate (Join-Path $PSScriptRoot 'runtime')
    if ($env:JAVA_HOME) { Add-JavaCandidate $env:JAVA_HOME }

    Get-Command java.exe -All -ErrorAction SilentlyContinue | ForEach-Object {
        Add-JavaCandidate $_.Source
    }

    $registryRoots = @(
        'HKLM:\SOFTWARE\JavaSoft\JDK',
        'HKLM:\SOFTWARE\JavaSoft\Java Runtime Environment',
        'HKLM:\SOFTWARE\Eclipse Adoptium\JDK',
        'HKLM:\SOFTWARE\Microsoft\JDK',
        'HKLM:\SOFTWARE\Azul Systems\Zulu'
    )
    foreach ($root in $registryRoots) {
        if (-not (Test-Path -LiteralPath $root)) { continue }
        @((Get-Item -LiteralPath $root),
            (Get-ChildItem -LiteralPath $root -Recurse -ErrorAction SilentlyContinue)) |
            Where-Object { $_ } |
            ForEach-Object {
                $registryJavaHome = (Get-ItemProperty -LiteralPath $_.PSPath -ErrorAction SilentlyContinue).JavaHome
                if ($registryJavaHome) { Add-JavaCandidate $registryJavaHome }
            }
    }

    $vendorRoots = @(
        (Join-Path $env:ProgramFiles 'Java'),
        (Join-Path $env:ProgramFiles 'Eclipse Adoptium'),
        (Join-Path $env:ProgramFiles 'Microsoft'),
        (Join-Path $env:ProgramFiles 'Zulu'),
        (Join-Path $env:ProgramFiles 'Amazon Corretto'),
        (Join-Path $env:ProgramFiles 'BellSoft'),
        (Join-Path $env:ProgramFiles 'Semeru')
    )
    foreach ($root in $vendorRoots) {
        if (-not (Test-Path -LiteralPath $root)) { continue }
        Get-ChildItem -LiteralPath $root -Directory -Recurse -Depth 2 -ErrorAction SilentlyContinue |
            ForEach-Object { Add-JavaCandidate $_.FullName }
    }
}

foreach ($java in $candidates) {
    $details = (& $java -XshowSettings:properties -version 2>&1 | Out-String)
    if ($LASTEXITCODE -ne 0) {
        $rejections.Add("$java (failed to execute)")
        continue
    }
    $versionMatch = [regex]::Match($details,
        '(?m)^\s*java\.version\s*=\s*([^\s]+)')
    if (-not $versionMatch.Success) {
        $versionMatch = [regex]::Match($details, 'version\s+"([^"]+)"')
    }
    if (-not $versionMatch.Success) {
        $rejections.Add("$java (could not determine Java version)")
        continue
    }

    $parts = $versionMatch.Groups[1].Value -split '[._+-]'
    $major = 0
    if ($parts.Count -gt 1 -and $parts[0] -eq '1') {
        [void][int]::TryParse($parts[1], [ref]$major)
    } else {
        [void][int]::TryParse($parts[0], [ref]$major)
    }
    if ($major -lt 17) {
        $rejections.Add("$java (Java $major is older than 17)")
        continue
    }

    $archMatch = [regex]::Match($details, '(?m)^\s*os\.arch\s*=\s*([^\s]+)')
    if (-not $archMatch.Success -or
            $archMatch.Groups[1].Value -notmatch '^(amd64|x86_64)$') {
        $arch = if ($archMatch.Success) { $archMatch.Groups[1].Value } else { 'unknown' }
        $rejections.Add("$java (unsupported architecture: $arch; x64 required)")
        continue
    }

    $homeMatch = [regex]::Match($details, '(?m)^\s*java\.home\s*=\s*(.+?)\s*$')
    if ($homeMatch.Success) {
        $javaHome = $homeMatch.Groups[1].Value.Trim()
    } else {
        $javaHome = [System.IO.Path]::GetFullPath(
            (Join-Path (Split-Path -Parent $java) '..'))
    }
    try { $javaHome = [System.IO.Path]::GetFullPath($javaHome) } catch {
        $rejections.Add("$java (reported invalid java.home: $javaHome)")
        continue
    }
    if (-not $validatedHomes.Add($javaHome)) { continue }
    if (-not (Test-Path -LiteralPath (Join-Path $javaHome 'bin\java.exe') -PathType Leaf)) {
        $rejections.Add("$java (reported java.home lacks bin\java.exe: $javaHome)")
        continue
    }
    if ($Purpose -eq 'Build' -and
            -not (Test-Path -LiteralPath (Join-Path $javaHome 'bin\javac.exe') -PathType Leaf)) {
        $rejections.Add("$javaHome (runtime only; bin\javac.exe is missing)")
        continue
    }

    Write-Output $javaHome
    exit 0
}

[Console]::Error.WriteLine(
    "No compatible 64-bit Java 17+ {0} was found. Candidates checked:" -f
    $(if ($Purpose -eq 'Build') { 'JDK' } else { 'runtime' }))
if ($rejections.Count -eq 0) {
    [Console]::Error.WriteLine('  (no Java candidates were discovered)')
} else {
    foreach ($rejection in $rejections) {
        [Console]::Error.WriteLine("  - $rejection")
    }
}
exit 1
