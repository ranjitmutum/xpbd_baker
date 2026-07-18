param([Parameter(Mandatory = $true)][string] $RealJdkHome)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$findJava = Join-Path $PSScriptRoot 'find-java.ps1'
$temporaryRoot = Join-Path ([System.IO.Path]::GetTempPath()) (
    'xpbd-java-matrix-' + [Guid]::NewGuid().ToString('N'))

function Assert-True([bool] $condition, [string] $message) {
    if (-not $condition) { throw $message }
}

function New-FakeJavaHome([string] $path, [string] $version,
                          [string] $architecture, [bool] $withCompiler) {
    $bin = Join-Path $path 'bin'
    New-Item -ItemType Directory -Path $bin -Force | Out-Null
    $escapedHome = $path.Replace('\', '\\').Replace('"', '\"')
    $className = 'FakeJava' + [Guid]::NewGuid().ToString('N')
    $source = @"
using System;
public static class $className {
    public static void Main() {
        Console.Error.WriteLine("Property settings:");
        Console.Error.WriteLine("    java.version = $version");
        Console.Error.WriteLine("    java.home = $escapedHome");
        Console.Error.WriteLine("    os.arch = $architecture");
    }
}
"@
    Add-Type -TypeDefinition $source -Language CSharp -OutputType ConsoleApplication `
        -OutputAssembly (Join-Path $bin 'java.exe')
    if ($withCompiler) {
        New-Item -ItemType File -Path (Join-Path $bin 'javac.exe') -Force | Out-Null
    }
}

function Invoke-Finder([string] $purpose, [string[]] $candidates,
                       [bool] $onlyExplicit = $true) {
    $arguments = @('-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', $findJava,
        '-Purpose', $purpose)
    if ($onlyExplicit) { $arguments += '-OnlyExplicitCandidates' }
    if ($candidates.Count -gt 0) { $arguments += @('-Candidate') + $candidates }
    $previousErrorPreference = $ErrorActionPreference
    try {
        $ErrorActionPreference = 'Continue'
        $output = & powershell.exe @arguments 2>&1
        $exitCode = $LASTEXITCODE
    } finally {
        $ErrorActionPreference = $previousErrorPreference
    }
    return [pscustomobject]@{ ExitCode = $exitCode; Output = @($output) }
}

try {
    New-Item -ItemType Directory -Path $temporaryRoot -Force | Out-Null
    $jdk = Join-Path $temporaryRoot 'JDK 空格'
    $jre = Join-Path $temporaryRoot 'JRE only'
    $wrongArch = Join-Path $temporaryRoot 'wrong-arch'
    New-FakeJavaHome $jdk '21.0.8' 'amd64' $true
    New-FakeJavaHome $jre '21.0.8' 'amd64' $false
    New-FakeJavaHome $wrongArch '21.0.8' 'x86' $true

    $build = Invoke-Finder 'Build' @($jdk)
    Assert-True ($build.ExitCode -eq 0) 'Build discovery rejected the valid JDK.'
    Assert-True ($build.Output[-1].ToString() -eq $jdk) 'Build discovery selected the wrong home.'

    $wrongArchitectureBuild = Invoke-Finder 'Build' @($wrongArch)
    Assert-True ($wrongArchitectureBuild.ExitCode -ne 0) `
        'Build discovery accepted a 32-bit Java home.'
    Assert-True (($wrongArchitectureBuild.Output -join "`n") -match 'architecture') `
        'Wrong-architecture rejection did not explain the x64 requirement.'

    $runtime = Invoke-Finder 'Runtime' @($jre)
    Assert-True ($runtime.ExitCode -eq 0) 'Runtime discovery rejected a valid JRE-only home.'
    Assert-True ($runtime.Output[-1].ToString() -eq $jre) 'Runtime discovery returned the wrong home.'

    $buildFromJre = Invoke-Finder 'Build' @($jre)
    Assert-True ($buildFromJre.ExitCode -ne 0) 'Build discovery accepted a JRE-only home.'
    Assert-True (($buildFromJre.Output -join "`n") -match 'javac\.exe') `
        'JRE-only rejection did not explain that javac.exe is missing.'

    $oldDiscoveryJavaHome = $env:JAVA_HOME
    try {
        $env:JAVA_HOME = $jdk
        $environmentRuntime = Invoke-Finder 'Runtime' @() $false
    } finally {
        $env:JAVA_HOME = $oldDiscoveryJavaHome
    }
    Assert-True ($environmentRuntime.ExitCode -eq 0) `
        'Normal discovery rejected JAVA_HOME.'
    Assert-True ($environmentRuntime.Output[-1].ToString() -eq $jdk) `
        'Normal discovery did not prefer JAVA_HOME.'
    Assert-True (($environmentRuntime.Output -join "`n") -notmatch 'VariableNotWritable') `
        'Normal registry discovery attempted to overwrite PowerShell HOME.'

    Assert-True (Test-Path -LiteralPath (Join-Path $RealJdkHome 'bin\javac.exe')) `
        'The Maven test runtime is not a complete JDK.'
    $junction = Join-Path $temporaryRoot '真实 JDK with spaces'
    New-Item -ItemType Junction -Path $junction -Target $RealJdkHome | Out-Null
    $oldJavaHome = $env:JAVA_HOME
    $oldNoPause = $env:XPBD_NO_PAUSE
    try {
        $env:JAVA_HOME = $junction
        $env:XPBD_NO_PAUSE = '1'
        $wrapperOutput = & cmd.exe /d /c "call mvnw.cmd -version" 2>&1
        Assert-True ($LASTEXITCODE -eq 0) (
            "Maven Wrapper failed with a spaced/unicode JAVA_HOME:`n" +
            ($wrapperOutput -join "`n"))
    } finally {
        $env:JAVA_HOME = $oldJavaHome
        $env:XPBD_NO_PAUSE = $oldNoPause
    }
} finally {
    if (Test-Path -LiteralPath $temporaryRoot) {
        Remove-Item -LiteralPath $temporaryRoot -Recurse -Force
    }
}
