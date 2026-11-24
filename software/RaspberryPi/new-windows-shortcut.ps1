param(
    [Parameter(Mandatory = $true)]
    [ValidatePattern('^[A-Za-z0-9_][A-Za-z0-9_.-]*@[A-Za-z0-9][A-Za-z0-9.-]*$')]
    [string]$Pi,
    [string]$Python = 'python'
)

$ErrorActionPreference = 'Stop'
$interpreter = (Get-Command $Python -CommandType Application | Select-Object -First 1).Source
$panel = Join-Path $PSScriptRoot 'desktop.py'
$desktop = [Environment]::GetFolderPath('DesktopDirectory')
$shell = New-Object -ComObject WScript.Shell
$shortcut = $shell.CreateShortcut((Join-Path $desktop 'ARGOS Pi.lnk'))
$shortcut.TargetPath = $interpreter
$shortcut.Arguments = ('"{0}" --ssh "{1}"' -f $panel, $Pi)
$shortcut.WorkingDirectory = $PSScriptRoot
$shortcut.Description = 'ARGOS panel with automatic Raspberry Pi connection'
$shortcut.Save()
