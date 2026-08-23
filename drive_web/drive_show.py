"""How a console Session turns rover replies into the page's JSON."""
from __future__ import annotations

import base64
import time
from typing import Any

import _paths  # noqa: F401 — console_model
from console_model import (
    ALARM_WHEN_FALSE, ALARM_WHEN_TRUE, BATTERY_NOTES, BATTERY_STALE_S, LOG_LINES,
    LOUD_PHASES, MAP_LEGEND, Reply, STATUS_FIELDS, TURN_ROWS, WIFI_POLL_S,
    move_sentence, or_dash, rung, wifi_verdict, worth_logging,
)


def _png_width(png: bytes) -> int:
    """The width out of a PNG's IHDR, which is bytes 16 to 20 of any PNG there is.

    Here so that a map still gets a size when the reply does not carry one, and it
    is the same four bytes `rover_daemon` reads to fill in `pixels`: a whole number
    of cells at a whole number of pixels rarely lands on the size that was asked
    for, so the only honest source is the picture."""
    return int.from_bytes(png[16:20], "big") if len(png) >= 20 else 0


def _number(value: Any, fallback):
    """A number out of JSON, or the fallback. The page sends what was typed into a
    box, and what was typed into a box is whatever somebody typed."""
    try:
        if value is None or value == "":
            return fallback
        return float(value)
    except (TypeError, ValueError):
        return fallback


class SessionShow:
    """Renderers mixed into Session."""

    def show_tools(self, body: dict[str, Any]) -> None:
        if not body.get("ok"):
            self.link_text = "connected, but it would not say what it can do"
            self.say(f"list_tools failed: {body.get('error')}\n", "bad")
            return
        self.tools = [t.get("function", {}).get("name", "?") for t in body["tools"]]
        self.can_drive = {"drive", "turn_in_place"} <= set(self.tools)
        self.link_text = (f"{self.address}: {len(self.tools)} tools"
                          + ("" if self.can_drive else ", none of them driving"))
        self.say("tools: " + ", ".join(self.tools) + "\n", "quiet")
        if not self.can_drive:
            # Said plainly, because the failure here is a page of live buttons on a
            # rover with no navigator behind them, and the cause is nearly always a
            # daemon started without --lidar.
            self.say("this daemon is offering no driving tools. It was probably "
                     "started without --lidar; or its lidar has not enumerated yet, "
                     "in which case the tools appear when the sensor does and "
                     "connecting again will pick them up.\n", "bad")

    def show_status(self, body: dict[str, Any]) -> None:
        if not body.get("ok"):
            self.status_rows = [[label, "-", False]
                                for _key, label, _fmt in STATUS_FIELDS]
            self.status_error = str(body.get("error", "no status"))
            self.pose_text = "-"
            # Unknown, not dead. A rover that is not answering says nothing about
            # its lidar, and offering to reset one over a link that is down would
            # be a button that cannot do anything.
            self.lidar_live = None
            self.lidar_note = ""
            return
        # The rover is there. Only a reply that says something resets this, so a
        # refusal does not read as an answer -- see mind_the_link.
        self.answered_at = time.monotonic()
        self.status_error = ""
        rows = []
        for key, label, fmt in STATUS_FIELDS:
            value = body.get(key)
            alarm = ((key in ALARM_WHEN_FALSE and not value)
                     or (key in ALARM_WHEN_TRUE and bool(value)))
            rows.append([label, fmt(value), bool(alarm)])
        self.status_rows = rows
        pose = body.get("pose") or {}
        self.heading_deg = float(pose.get("heading_deg", 0.0))
        self.pose_text = "x {:+.2f}  y {:+.2f}  {:+.1f} deg".format(
            pose.get("x_m", 0.0), pose.get("y_m", 0.0), pose.get("heading_deg", 0.0))
        self.show_lidar(body)
        self.show_move(body.get("move") or {})

    def show_lidar(self, body: dict[str, Any]) -> None:
        """Whether the sensor is talking, and what the rover has done about it.

        Two states worth a sentence rather than a row. A sensor that has stopped
        reporting is the one fault that makes every other number on the panel a
        lie, and the rover now tries to fix it by itself -- so the line has to say
        both how long it has been quiet and whether the fixing has been tried,
        or an unattended reset looks like the rover having done nothing.
        """
        self.lidar_live = bool(body.get("lidar_live"))
        note = body.get("lidar_reset_note") or ""
        resets = int(body.get("lidar_resets") or 0)
        if self.lidar_live:
            # Only worth saying once it has happened, and then worth saying: a
            # rover that has replugged its own lidar twice this afternoon has a
            # cable working loose and this is the only place that would show it.
            self.lidar_note = (f"the lidar has been reset {resets} time"
                               f"{'' if resets == 1 else 's'} this session"
                               if resets else "")
            return
        age = body.get("scan_age_s")
        quiet = "not reporting" if age is None else f"quiet for {age:.0f} s"
        self.lidar_note = f"the lidar is {quiet}." + (f" Last reset: {note}"
                                                      if note else "")

    def show_move(self, move: dict[str, Any]) -> None:
        """The line under the map always; the transcript only when the rover has
        said something it has not said before.

        `seq` is the navigator's own counter of the sentences it has published, and
        it is the whole reason this can be polled: without it there is no way to
        tell a phase that has just started from the same phase read again a tenth
        of a second later, and the log would fill with the same line.

        `missed` holds anything the rover said between the last poll and this one,
        oldest first, because a phase can be shorter than the gap between two polls
        -- and the phase that usually is happens to be the replan, which is the one
        worth reading. Those go to the transcript in order; only the newest reaches
        the panel, which is a statement about now.

        A move quicker than the poll is answered before any of this arrives, and its
        commentary would then read as news about something already reported -- the
        planning line printed underneath the outcome it led to. So once a move's
        reply has gone into the log, what the rover said during that move is dropped
        rather than printed late, up to and including the record that ends it.
        Commentary about a move this console did not start is never in that state and
        is always printed, which is how a rover being driven by something else -- or
        from the other browser -- can still be watched here.
        """
        sentence = move_sentence(move)
        self.plan_text = sentence or "-"
        seq = move.get("seq")
        if seq is None or seq == self.move_seq:
            return
        self.move_seq = seq
        for record in (move.get("missed") or []) + [move]:
            if self.move_answered:
                # Still working through what the reply overtook. The ending is the
                # last of it, and anything after belongs to a move not yet answered.
                self.move_answered = record.get("phase") != "ended"
                continue
            line = move_sentence(record)
            if line and worth_logging(record):
                self.say(f"{'':10}   <~ {line}\n",
                         "note" if record.get("phase") in LOUD_PHASES else "quiet")

    def show_map(self, body: dict[str, Any]) -> None:
        if not body.get("ok"):
            self.map_error = str(body.get("error", "no map"))
            return
        try:
            self.map_png = base64.b64decode(body["png_base64"])
        except (KeyError, ValueError) as error:
            self.map_error = f"cannot show the map: {error}"
            return
        self.map_error = ""
        self.map_gen += 1
        self.map_drawn_at = time.monotonic()
        # The daemon says how big what it drew came out, under `pixels`. Where it
        # does not -- an older daemon, or the mock -- the PNG says so itself in its
        # header, which is where the daemon reads it from too. The page needs a real
        # number either way: it sets the panel's aspect ratio from it, and a wrong
        # one puts the click somewhere else in the room.
        width = int(body.get("pixels") or 0) or _png_width(self.map_png)
        self.map_shape = (width, width)
        self.map_caption = str(body.get("caption", ""))
        self.map_view = {
            "half_extent_m": float(body.get("half_extent_m", self.half_extent)),
            "scale": int(body.get("scale") or 1),
            "rover_up": bool(body.get("rover_up")),
            "pose": body.get("pose") or {"heading_deg": self.heading_deg},
        }

        # What the rover actually drew, which is not always the size asked for: a
        # cell has to be a whole number of pixels, so most sizes are only reachable
        # to within a few percent, and a very wide view cannot reach a large one at
        # all. Worth saying, because otherwise "bigger" appearing to do nothing looks
        # like a broken button rather than a picture already as big as that view can
        # be drawn.
        took = body.get("render_s")
        self.map_cost = float(took or 0.0)
        note = f"{width} px at {body.get('scale', '?')} px/cell"
        if body.get("bytes"):
            note += f", {body['bytes'] / 1000:.0f} kB"
        if took is not None:
            note += f", {took:.1f} s to draw"
        if width and abs(width - self.map_size) > self.map_size * 0.1:
            note += f" -- {self.map_size} px was not reachable here"
        self.map_note = note

    def show_picture(self, body: dict[str, Any]) -> None:
        """The frame, straight through to an `<img>`.

        The tkinter window this replaced needed OpenCV at this point, because the
        rover can only send JPEG -- there is no image library on that Pi, which is
        the same reason face detection runs on another host -- and tk reads PNG, GIF
        and PPM. A browser reads JPEG, so the decode, the resize, the BGR-to-RGB and
        the fallback that wrote the frame to a file and said where all went away,
        and the console stopped having a dependency.
        """
        if not body.get("ok"):
            self.frame_error = str(body.get("error", "no picture"))
            self.frame_note = ""
            return
        try:
            self.frame_jpeg = base64.b64decode(body.get("jpeg_base64", ""))
        except ValueError as error:
            self.frame_error = f"those bytes did not decode: {error}"
            return
        self.frame_error = ""
        self.frame_gen += 1
        where = f"pan {or_dash(body.get('pan'))}, tilt {or_dash(body.get('tilt'))}"
        size = f"{or_dash(body.get('width'))}x{or_dash(body.get('height'))}"
        # Which of the two paths it came off. They mean different things: while
        # tracking runs the loop owns the camera and this is its newest frame, which
        # is also the one the gimbal is actually pointed at.
        source = "tracking's own frame" if body.get("live") else "fresh"
        self.frame_note = (f"{size}, {body.get('bytes', 0) / 1000:.0f} kB, {where}, "
                           f"{source}, {self.frame_cost:.1f} s")

    def show_battery(self, body: dict[str, Any]) -> None:
        """Volts and percent, and how much trouble the pack is in.

        The age of the reading is shown only once it is older than the daemon's own
        cache. Inside that window every reading is a few seconds old by design, and
        a number that always carries a caveat is a number nobody reads; past it, the
        board has stopped answering, which is the one thing this panel has to be able
        to say.
        """
        if not body.get("ok"):
            self.battery = {"text": "-", "state": "",
                            "note": str(body.get("error", "no reading"))}
            return
        state = str(body.get("state", "?"))
        percent = body.get("percent")
        text = or_dash(body.get("volts"), "{:.2f} V")
        if percent is not None:
            text += f"   {percent}%"
        note = BATTERY_NOTES.get(state, state)
        age = body.get("reading_age_s") or 0.0
        if age > BATTERY_STALE_S:
            note += f", and read {age:.0f} s ago"
        self.battery = {"text": text, "state": state, "note": note}

    def show_wifi(self, body: dict[str, Any]) -> None:
        """The access point, its strength, and what else was last heard.

        The strength is the driver's dBm rather than the 0-100 figure beside each row
        in the list, and the difference is not cosmetic: measured on this rover's
        dongle, consecutive scans put the *same* association anywhere from 74 to 88
        while the driver held steady within a couple of dB. So the number that gets a
        colour and a verdict is the one worth trusting, and the column in the list is
        only there to rank the alternatives against each other.
        """
        if not body.get("ok"):
            error = str(body.get("error", "no answer"))
            if "no such tool" in error:
                # An older daemon. Say so once, in the panel, and stop asking.
                self.wifi_ok = False
                self.wifi.update({"supported": False, "text": "-", "verdict": "",
                                  "where": "this rover's daemon does not offer the "
                                           "network calls yet", "networks": [],
                                  "scanning": False})
                return
            self.wifi_ok = True         # it knows the call; it just could not answer
            self.wifi.update({"supported": True, "text": "-", "verdict": "",
                              "where": error, "scanning": False})
            return

        self.wifi_ok = True
        ssid = body.get("connected")
        level = body.get("level_dbm")
        if ssid is None:
            text, verdict = "not associated", "poor"
        else:
            text = str(ssid)
            if isinstance(level, (int, float)):
                text += f"   {level:.0f} dBm"
            verdict = wifi_verdict(level)

        where = []
        address = body.get("address")
        # An association with no address is the failure worth naming: every panel on
        # this page has gone blank and the rover looks connected from the outside.
        where.append(str(address) if address else "no address -- DHCP has not answered")
        age = body.get("list_age_s")
        if isinstance(age, (int, float)) and age > WIFI_POLL_S:
            where.append(f"list heard {age:.0f} s ago")
        join = body.get("last_join")
        if isinstance(join, dict):
            got = join.get("ssid")
            where.append(f"joined {got}" if join.get("ok")
                         else f"could not join {got}")

        networks = []
        for entry in body.get("networks") or []:
            in_use, configured = bool(entry.get("in_use")), bool(entry.get("configured"))
            networks.append({
                "ssid": str(entry.get("ssid", "?")),
                "signal": entry.get("signal", "-"),
                "in_use": in_use, "configured": configured,
                # Joinable means configured and not already the one in use.
                "joinable": configured and not in_use,
                "note": "on it" if in_use else ("" if configured
                                                else "no passphrase")})
        self.wifi.update({"supported": True, "text": text, "verdict": verdict,
                          "where": ", ".join(where), "networks": networks,
                          "scanning": False})

    def show_tracking(self, body: dict[str, Any]) -> None:
        if not body.get("ok"):
            self.track_text = str(body.get("error", "-"))
            return
        if not body.get("tracking"):
            self.track_text = "off"
            return
        # "Running" and "following somebody" are different states and the difference
        # is the whole question: a loop that is running and has locked onto nobody is
        # sweeping, which looks identical from here and quite different on the rover.
        who = ("following someone" if body.get("following_someone")
               else "sweeping, nobody yet")
        faces = body.get("faces_in_view")
        self.track_text = (f"on, {who}"
                           + ("" if faces is None else f", {faces} in view"))

    def tally_turn(self, reply: Reply) -> None:
        asked = float(reply.arguments.get("angle_deg", 0.0))
        turned = reply.body.get("turned_deg")
        ratio = (turned / asked) if (turned is not None and asked) else None
        self.turns.insert(0, {
            "asked": f"{asked:+.0f}",
            "turned": "-" if turned is None else f"{turned:+.1f}",
            "ratio": "-" if ratio is None else f"{ratio:.2f}",
            "secs": f"{reply.seconds:.1f}",
            "reason": str(reply.body.get("reason")
                          or reply.body.get("error") or "-")})
        del self.turns[TURN_ROWS:]

    # --- the transcript -------------------------------------------------------
    def log_sent(self, name: str, arguments: dict[str, Any]) -> None:
        shown = ", ".join(f"{k}={v}" for k, v in arguments.items())
        self.say(f"{time.strftime('%H:%M:%S')}  -> {name}({shown})\n", "sent")

    def log_reply(self, reply: Reply) -> None:
        body = reply.body
        ok = bool(body.get("ok"))
        head = f"{'':10}   <- {reply.seconds:5.2f}s  "
        if not ok and "error" in body:
            self.say(head + f"failed: {body['error']}\n", "bad")
            return
        summary = str(body.get("reason", "ok" if ok else "failed"))
        for key, unit in (("travelled_m", " m"), ("turned_deg", " deg"),
                          ("remaining_m", " m to go"),
                          ("clear_ahead_m", " m clear ahead")):
            if body.get(key) is not None:
                summary += f", {body[key]}{unit}"
        self.say(head + summary + "\n", "good" if ok else "bad")
        for key in ("detail", "note", "surroundings", "text"):
            if body.get(key):
                self.say(f"{'':17}{body[key]}\n", "quiet")

    def say(self, text: str, tag: str = "") -> None:
        """One line into the transcript, numbered.

        The number is what lets a browser that has been open for an hour and one
        that just arrived be served from the same list: each stream remembers how
        far it has read and is sent the rest. Trimmed from the front for the reason
        any log window is -- this is meant to be left open for an afternoon of test
        moves.
        """
        with self.lock:
            self.log_seq += 1
            self.log.append({"seq": self.log_seq, "text": text, "tag": tag})
            del self.log[:-LOG_LINES]

    # --- shutting down --------------------------------------------------------
    def close(self) -> None:
        """A console that can start a move has to be able to end one, including by
        being shut down. Sent inline rather than submitted, because the channel
        threads die with the process and a queued stop would go nowhere."""
        self.running = False
        if self.halt is not None:
            try:
                self.halt.client.call("stop_driving", {})
            except Exception:
                pass
        for channel in self.channels:
            channel.close()
