param([string]$Root = (Join-Path $PSScriptRoot 'raw\ecostress_raw'), [string]$OutputDirectory = $PSScriptRoot)
$ErrorActionPreference = 'Stop'
$rows = @(Get-ChildItem -LiteralPath $Root -Recurse -File | ForEach-Object {
    $file = $_
    $headers = @{}
    $reader = [IO.StreamReader]::new($file.FullName)
    try {
        for ($i=0; $i -lt 128 -and !$reader.EndOfStream; ++$i) {
            $line = $reader.ReadLine()
            if ($line -match '^\s*[+\-\d.]') { break }
            if ($line -match '^([^:]+):\s*(.*)$') { $headers[$Matches[1].Trim()] = $Matches[2].Trim() }
        }
    } finally { $reader.Dispose() }
    [pscustomobject]@{
        path=[IO.Path]::GetRelativePath($Root,$file.FullName); bytes=$file.Length
        name=$headers['Name']; type=$headers['Type']; class=$headers['Class']; subclass=$headers['Subclass']
        sampleNumber=$headers['Sample No.']; measurement=$headers['Measurement']; xUnits=$headers['X Units']
        yUnits=$headers['Y Units']; count=$headers['Number of X Values']; firstX=$headers['First X Value']; lastX=$headers['Last X Value']
    }
})
$rows | Export-Csv -LiteralPath (Join-Path $OutputDirectory 'metadata-index.csv') -NoTypeInformation -Encoding utf8
'TYPES'; $rows | Group-Object type | Sort-Object Count -Descending | Select-Object Count,Name | Format-Table -AutoSize
'CLASSES'; $rows | Group-Object class | Sort-Object Count -Descending | Select-Object Count,Name | Format-Table -AutoSize
'MEASUREMENTS'; $rows | Group-Object measurement | Sort-Object Count -Descending | Select-Object Count,Name | Format-Table -AutoSize
'Y UNITS'; $rows | Group-Object yUnits | Sort-Object Count -Descending | Select-Object Count,Name | Format-Table -AutoSize
'X UNITS'; $rows | Group-Object xUnits | Sort-Object Count -Descending | Select-Object Count,Name | Format-Table -AutoSize
'MANMADE'; $rows | Where-Object { $_.type -match 'man' -or $_.class -match 'man' -or $_.path -match 'manmade' } | Select-Object path,name,class,measurement | Format-Table -AutoSize
