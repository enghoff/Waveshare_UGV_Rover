# Drive console, on the rover

A page with the driving tools, hosted here so a phone or another machine opens
it without a desk process. The daemon is unchanged: this is still a client of
TCP 8769, just on loopback.

```
  phone / desk browser
          |
          |  HTTP :8771   (page, event stream, /map.png, /frame.jpg)
          v
  drive_web.py          --idle, from boot
          |
          |  six TCP connections to 127.0.0.1:8769
          v
  rover_daemon.py
```

A Pi 1 could not afford this and was never asked to. The Banana Pi M4 Zero can:
the HTTP and the JSON are cheap, and the expensive work (drawing a map, taking a
frame) is the daemon's. `--idle` is why a process that lives from boot is not
that client overnight -- it opens the six connections when a browser arrives and
drops them when the last tab has been gone a couple of seconds.

**8771, not 8770.** 8770 is [oak_depth](../oak_depth/README.md). There is no
password on this page; anyone on the LAN who can load it can drive the rover.

The pacing and the English live in
[voice_chat/console_model.py](../voice_chat/console_model.py); the wire is
[voice_chat/rover_tools.py](../voice_chat/rover_tools.py). Both are copied next
to this on deploy, because talk.py still imports them from `voice_chat/`.

## Running it

`crontab` for `admin`, beside the daemon's own entry, for the reason
[run_daemon.sh](../rover_daemon/run_daemon.sh) gives -- a system unit would need
a sudo password no script has, and cron needs none:

```
@reboot /home/admin/ugv/run_daemon.sh --vision --lidar
@reboot /home/admin/ugv/oak_depth/run_oak_depth.sh
@reboot /home/admin/ugv/drive_web/run_drive_web.sh
```

```bash
scp drive_web/*.py drive_web/*.html drive_web/*.sh drive_web/README.md bpi-m4zero:~/ugv/drive_web/
scp voice_chat/console_model.py voice_chat/rover_tools.py bpi-m4zero:~/ugv/drive_web/
ssh bpi-m4zero '~/ugv/drive_web/install.sh'     # crontab, once
ssh bpi-m4zero '~/ugv/drive_web/restart.sh'     # prints /health
python drive_web/selftest.py
```

**Use `restart.sh` rather than typing the `pkill` yourself.** The pattern that
matches the server also matches the ssh command carrying it.

```
http://bpi-m4zero.local:8771/
http://192.168.1.47:8771/
```

Windows often cannot resolve `.local`; the number works.

`GET /health` is what `restart.sh` waits on. `watching` is how many event
streams are open; `rover` is empty until one of them has made this a client.

Against an invented room, with no rover:

```
python voice_chat/mock_rover.py --drive
python drive_web/drive_web.py --no-idle --bind 127.0.0.1
```
