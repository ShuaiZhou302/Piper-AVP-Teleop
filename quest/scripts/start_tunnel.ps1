<#
.SYNOPSIS
  Keeps an SSH -L port forward to cobot_magic alive for Quest teleop.

.DESCRIPTION
  cobot_magic is only reachable through the SOCKS5 VPN proxy already
  configured in ~/.ssh/config (ProxyCommand via ncat), so this reuses that
  host entry rather than opening any new network path. quest_client.py then
  talks to 127.0.0.1:<Port>, which SSH forwards to cobot_magic:<Port>, where
  quest_server.py is listening on 127.0.0.1 only.

  Windows has no autossh, so this is a plain retry loop: if ssh exits for
  any reason (VPN blip, sleep/wake, robot reboot), it reconnects after a
  short delay. Run this in its own terminal and leave it open for the whole
  teleop session; quest_client.py does its own reconnect on top of this.

.PARAMETER Port
  Local AND remote port for the forward (must match quest_server.py --port).

.PARAMETER SshHost
  SSH config alias to use. Defaults to 'cobot_magic'.

.EXAMPLE
  powershell -File scripts\start_tunnel.ps1
  powershell -File scripts\start_tunnel.ps1 -Port 8770 -SshHost cobot_magic
#>
param(
    [int]$Port = 8770,
    [string]$SshHost = "cobot_magic"
)

Write-Output "Quest teleop tunnel: 127.0.0.1:$Port <-> ${SshHost}:$Port"
Write-Output "Ctrl-C to stop. Reconnects automatically on drop."

while ($true) {
    $start = Get-Date
    Write-Output "[$start] connecting..."

    # -N: no remote command, just forward. -L: local forward.
    # ExitOnForwardFailure / ServerAliveInterval already set on the host in
    # ~/.ssh/config, so a dead link is detected and this process exits,
    # letting the loop below reconnect.
    ssh -N -L 127.0.0.1:${Port}:127.0.0.1:${Port} $SshHost

    $elapsed = (Get-Date) - $start
    Write-Output "[$(Get-Date)] tunnel dropped after $($elapsed.TotalSeconds.ToString('F0'))s, reconnecting in 2s..."
    Start-Sleep -Seconds 2
}
