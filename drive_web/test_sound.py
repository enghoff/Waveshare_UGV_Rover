"""The rover's own sound: which Bluetooth device speaks, which one listens.

PipeWire's graph is a fixture here, shaped as `pw-dump` prints it for a speaker
on A2DP and a headset that starts in A2DP and is moved to its hands-free
profile, which is the one with a microphone. What matters is that a speaker
and a headset are never streamed at once -- on the Orin the two links together
leave the wi-fi too little to hold a conversation -- that a speaker is never
listened to through, and that nothing is recorded between conversations.
"""
from __future__ import annotations

import io
import time

import _paths  # noqa: F401 -- puts drive_web and voice_chat on the path
from test_harness import SKIP, check

SPEAKER, HEADSET = 51, 62


def _device(ident, address, name, form, profile, profiles):
    return {"id": ident, "type": "PipeWire:Interface:Device",
            "info": {"props": {"device.api": "bluez5",
                               "api.bluez5.address": address,
                               "device.description": name,
                               "device.form-factor": form},
                     "params": {"EnumProfile": [
                         {"index": i, "name": n, "available": "yes"}
                         for i, n in profiles],
                         "Profile": [{"name": profile}]}}}


def _node(ident, device, media, name):
    return {"id": ident, "type": "PipeWire:Interface:Node",
            "info": {"props": {"device.id": device, "media.class": media,
                               "node.name": name}}}


def graph(headset_profile="a2dp-sink", speaker=True):
    objects = [_device(HEADSET, "77:88:99:aa:bb:cc", "Headset", "headset",
                       headset_profile,
                       [(0, "off"), (1, "a2dp-sink"), (2, "headset-head-unit-cvsd"),
                        (3, "headset-head-unit")])]
    if headset_profile == "a2dp-sink":
        objects.append(_node(80, HEADSET, "Audio/Sink", "bluez_output.77_88.1"))
    else:
        objects += [_node(81, HEADSET, "Audio/Sink", "bluez_output.77_88.0"),
                    _node(82, HEADSET, "Audio/Source", "bluez_input.77_88.0")]
    if speaker:
        objects += [_device(SPEAKER, "11:22:33:44:55:66", "BT Speaker", "speaker",
                            "a2dp-sink", [(0, "off"), (1, "a2dp-sink")]),
                    _node(90, SPEAKER, "Audio/Sink", "bluez_output.11_22.1")]
    objects.append(_node(3, 200, "Audio/Sink", "alsa_output.usb"))   # not bluez
    return objects


def test_a_headset_takes_the_voice_off_the_speaker() -> None:
    try:
        import rover_sound
    except ImportError as exc:
        SKIP.append(f"rover sound ({type(exc).__name__})")
        return

    choose = lambda objects: rover_sound.choose(rover_sound.bt_audio(objects))  # noqa: E731
    first = choose(graph())
    check("with a speaker and a headset still in A2DP, nothing plays yet: the "
          "headset's music output is about to go away, and the speaker would be "
          "a second link", (first["outputs"], first["gate"]), ([], False))
    check("...the headset is asked into its hands-free profile, the generic one",
          first["profile"], (HEADSET, 3))
    check("...which it does not have a microphone without",
          (first["mic"], first["mic_name"]), (None, "Headset"))

    then = choose(graph("headset-head-unit"))
    check("once it is in that profile it is the microphone and the voice both, "
          "and the speaker is left alone",
          (then["mic"], then["outputs"], then["out_names"]),
          ("bluez_input.77_88.0", ["bluez_output.77_88.0"], ["Headset"]))
    check("...a worn one is listened to while it speaks", then["gate"], False)
    check("...and nothing more is asked of the headset", then["profile"], None)

    speaker = choose([o for o in graph() if o["id"] not in (HEADSET, 80)])
    check("a speaker alone takes the voice, with the browser's microphone",
          (speaker["outputs"], speaker["mic"]), (["bluez_output.11_22.1"], None))
    check("...which is not listened to while the room hears the rover",
          speaker["gate"], True)

    alone = choose(graph("headset-head-unit", speaker=False))
    check("a headset alone is the speaker and the microphone both",
          (alone["outputs"], alone["mic"]),
          (["bluez_output.77_88.0"], "bluez_input.77_88.0"))
    check("...and a worn one is listened to while it speaks", alone["gate"], False)

    nothing = choose([_node(3, 200, "Audio/Sink", "alsa_output.usb")])
    check("no Bluetooth audio is the browser, as before",
          (nothing["outputs"], nothing["mic"], nothing["gate"]), ([], None, False))


class FakePlayer:
    def __init__(self, node):
        self.node, self.stdin = node, io.BytesIO()
        self.stdin.close = lambda: None
        self.stopped = False

    def terminate(self):
        self.stopped = True


def test_the_voice_is_paced_so_an_interruption_can_take_it_back() -> None:
    try:
        import numpy as np
        import rover_sound
    except ImportError as exc:
        SKIP.append(f"local speaker ({type(exc).__name__})")
        return

    players = []
    speaker = rover_sound.LocalSpeaker(lambda node: players.append(
        FakePlayer(node)) or players[-1])
    try:
        check("with nowhere to play, the browser keeps the voice",
              speaker.available, False)
        speaker.route(["bluez_output.11_22.1"])
        check("...and with a speaker, the rover takes it", speaker.available, True)
        speaker.begin()
        speaker.write(np.zeros(int(rover_sound.OUT_RATE * 2.0), dtype=np.float32))
        time.sleep(0.3)
        written = len(players[0].stdin.getvalue()) / 2 / rover_sound.OUT_RATE
        check("two seconds of voice is not handed over at once",
              0.15 < written < 0.6, True)
        check("...and is busy while it plays", speaker.busy, True)
        heard = speaker.played_ms() / 1000.0
        check("what was heard trails what was handed over by the link",
              0.0 <= heard < written, True)
        dropped = speaker.flush()
        check("an interruption drops what had not been handed over",
              1.2 < dropped <= 2.0, True)
        time.sleep(0.15)
        check("...and nothing more is played",
              len(players[0].stdin.getvalue()) / 2 / rover_sound.OUT_RATE
              - written < 0.15, True)
        speaker.route([])
        check("an output that went away is let go of", players[0].stopped, True)
    finally:
        speaker.close()


class FakeRecorder:
    def __init__(self, node, pcm):
        self.node, self.stdout = node, io.BytesIO(pcm)
        self.done = False

    def poll(self):
        return 0 if self.done else None

    def terminate(self):
        self.done = True


def test_the_headset_is_heard_only_during_a_conversation() -> None:
    try:
        import rover_sound
    except ImportError as exc:
        SKIP.append(f"rover sound ({type(exc).__name__})")
        return

    heard, profiles, recorders = [], [], []
    state = {"graph": graph()}
    sound = rover_sound.RoverSound(
        heard.append, read=lambda: state["graph"],
        launch_player=lambda node: FakePlayer(node),
        launch_mic=lambda node: recorders.append(
            FakeRecorder(node, b"\x01\x00" * 1600)) or recorders[-1],
        set_profile=lambda device, index: profiles.append((device, index)))
    try:
        sound.refresh()
        check("nothing is done outside a conversation",
              (profiles, recorders, sound.speaker.available), ([], [], False))

        sound.start()
        sound.refresh()
        sound.refresh()
        check("in one, the headset is moved to its microphone profile once",
              profiles, [(HEADSET, 3)])
        state["graph"] = graph("headset-head-unit")
        sound.refresh()
        time.sleep(0.1)
        check("...its microphone is recorded", [r.node for r in recorders],
              ["bluez_input.77_88.0"])
        check("...and what it hears reaches the conversation",
              sum(len(b) for b in heard), 3200)
        check("...which says where its sound is going: the headset, both ways",
              sound.describe(), {"out": "Headset", "mic": "Headset", "error": ""})

        sound.speaker.write(__import__("numpy").zeros(4800, dtype="float32"))
        check("a worn headset's own voice does not stop it being listened to",
              sound.deaf(), False)
        sound.speaker.flush()

        state["graph"] = [o for o in graph() if o["id"] not in (HEADSET, 80)]
        sound.refresh()
        check("with the headset gone the speaker takes the voice, and its "
              "microphone is let go", (sound.describe()["out"], sound.has_mic),
              ("BT Speaker", False))
        sound.speaker.write(__import__("numpy").zeros(4800, dtype="float32"))
        check("...and while the speaker speaks, no microphone is listened to",
              sound.deaf(), True)

        sound.stop()
        check("the end of a conversation stops the recording",
              (recorders[0].done, sound.has_mic), (True, False))
        sound.refresh()
        check("...and a refresh that lands after it starts nothing",
              (len(recorders), sound.speaker.available), (1, False))

        def gone():
            raise FileNotFoundError("pw-dump")
        sound.start()
        sound._read = gone
        sound.refresh()
        check("a rover with no sound server says so",
              sound.describe()["error"].startswith("no sound server"), True)
    finally:
        sound.stop()
        sound.speaker.close()


def test_the_browser_microphone_gives_way_to_the_headset() -> None:
    try:
        import omni_bridge
    except ImportError as exc:
        SKIP.append(f"browser microphone ({type(exc).__name__})")
        return

    console = omni_bridge.Omni("127.0.0.1:1", lambda text, err=False: None,
                               log_path=None, sound=False)
    got = []
    console._hear = got.append
    console.on_audio(b"page")
    check("with no Bluetooth the page is the microphone", got, [b"page"])

    class Headset:
        has_mic = True

        def deaf(self):
            return False
    console.sound = Headset()
    console.on_audio(b"page")
    check("a headset's microphone takes over from the page's", got, [b"page"])
    console.log.close()


TESTS = (
    test_a_headset_takes_the_voice_off_the_speaker,
    test_the_voice_is_paced_so_an_interruption_can_take_it_back,
    test_the_headset_is_heard_only_during_a_conversation,
    test_the_browser_microphone_gives_way_to_the_headset,
)
