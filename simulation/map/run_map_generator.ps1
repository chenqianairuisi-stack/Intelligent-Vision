param(
    [int]$Count = 10,
    [int]$Boxes = 3,
    [int]$Bombs = 0,
    [ValidateSet("no-bomb", "with-bomb", "bomb")]
    [string]$Mode = "no-bomb",
    [ValidateSet("normal", "hard")]
    [string]$Difficulty = "normal",
    [string]$OutDir = "map\map_generated",
    [string]$Prefix = "gen",
    [uint32]$Seed = 0,
    [double]$WallDensity = 0.24,
    [int]$MinPairPushes = 4,
    [int]$MinBombRequiredPairs = 1,
    [int]$QualityCandidates = 56,
    [int]$MaxAttempts = 50000,
    [switch]$WriteMeta,
    [switch]$RequirePhase2SpecificBomb,
    [switch]$Build
)

$ErrorActionPreference = "Stop"

$Root = Resolve-Path (Join-Path $PSScriptRoot "..")
$BuildDir = Join-Path $Root ".codex_tmp"
$Generator = Join-Path $BuildDir "map_generator.exe"

if (!(Test-Path $BuildDir)) {
    New-Item -ItemType Directory -Path $BuildDir | Out-Null
}

if ($Build -or !(Test-Path $Generator)) {
    Push-Location $Root
    try {
        g++ -std=c++17 -O2 map\MapGenerator.cpp -o $Generator
    } finally {
        Pop-Location
    }
}

$argsList = @(
    "--count", "$Count",
    "--boxes", "$Boxes",
    "--out-dir", "$OutDir",
    "--prefix", "$Prefix",
    "--wall-density", "$WallDensity",
    "--min-pair-pushes", "$MinPairPushes",
    "--quality-candidates", "$QualityCandidates",
    "--max-attempts", "$MaxAttempts",
    "--mode", "$Mode",
    "--bombs", "$Bombs",
    "--difficulty", "$Difficulty",
    "--min-bomb-required-pairs", "$MinBombRequiredPairs"
)

if ($Seed -ne 0) {
    $argsList += @("--seed", "$Seed")
}

if ($WriteMeta) {
    $argsList += @("--write-meta")
}

if ($RequirePhase2SpecificBomb) {
    $argsList += @("--require-phase2-specific-bomb")
}

Push-Location $Root
try {
    & $Generator @argsList
} finally {
    Pop-Location
}
