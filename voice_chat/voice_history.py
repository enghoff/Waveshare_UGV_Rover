"""Conversation history, pictures, and prompt trimming."""
from __future__ import annotations

import re
from typing import Any

import server as S


def _template():
    """Whatever owns the chat template: the processor when there is one.

    Multimodal models keep their template on the processor rather than the
    tokenizer, and the two do not always both have one -- so everything that
    renders a conversation goes through this rather than reaching for
    `_tokenizer` and working by luck.
    """
    return S._processor if S._processor is not None else S._tokenizer

def _image_message(image: Any, text: str | None = None) -> dict[str, Any]:
    """The picture, as a turn in the conversation.

    A user turn rather than the tool result it came from: a tool message holds a
    string, and the templates that would render an image inside one do not
    agree with each other. The text beside it is the question that took it,
    because that is now the last user message and so the one the model answers.

    A caption that only named the picture -- "This is the picture your camera
    has just taken" -- was read as the turn. After a few action tools the model
    had been saying "I switched the lights on" / "I started tracking", and look
    got the same treatment: "I took a picture of the room", with a yellow oval
    on magenta sitting right there. Same picture, empty history: it described
    the oval. The question has to sit on the picture, not one turn back.
    """
    return {
        "role": "user",
        "content": [
            {"type": "image", "image": image},
            # The user's own words, not an instruction. A sentence about what
            # to do with the picture was tried and came back out of the model's
            # mouth as a rule -- and under FRESH_PICTURE would also have been a
            # lie, because the picture does not stay.
            {"type": "text", "text": text or "This is the picture your camera has just taken."},
        ],
    }


def _images(history: list[dict[str, Any]]) -> list[Any]:
    """Every image in the conversation, in the order the template will want them."""
    found = []
    for message in history:
        content = message.get("content")
        # Only a list can hold one. Iterating a string here would walk it a
        # character at a time, on every prompt measurement, for nothing.
        if not isinstance(content, list):
            continue
        for part in content:
            if isinstance(part, dict) and part.get("type") == "image" and part.get("image"):
                found.append(part["image"])
    return found


def _textual(history: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """The same conversation with every picture reduced to the fact that it existed.

    For counting and for trimming, both of which have to render the prompt and
    neither of which should pay for an image to do it.
    """
    plain = []
    for message in history:
        content = message.get("content")
        if not isinstance(content, list):
            plain.append(message)
            continue
        text = " ".join(
            part.get("text", "") for part in content
            if isinstance(part, dict) and part.get("type") == "text"
        )
        plain.append({**message, "content": text})
    return plain


def _exchanges(history: list[dict[str, Any]]) -> list[list[dict[str, Any]]]:
    """The conversation grouped into exchanges: a spoken turn and everything answering it.

    A picture is a user message too -- that is the only role the templates will
    render one in -- so the split is on user messages holding a *string*, which
    is what an utterance is. Anything else joins the exchange it arrived in.
    """
    groups: list[list[dict[str, Any]]] = []
    for message in history:
        spoken = message.get("role") == "user" and isinstance(message.get("content"), str)
        if spoken or not groups:
            groups.append([])
        groups[-1].append(message)
    return groups


def _forget_pictures(history: list[dict[str, Any]], keep_newest: bool = True) -> None:
    """Drop the exchanges that held a picture, in place. Whole exchanges, not pictures.

    Two reasons, and they stack. A picture costs a few hundred tokens of a
    window that holds about a dozen spoken turns, so a conversation that looked
    four times would be mostly photographs of a room nobody is asking about any
    more. And with `keep_newest` false -- which is what a new turn does, since
    the camera moves between them -- it is how the rover is stopped from
    answering today's question with yesterday's view.

    Whole exchanges because everything else was measured and failed. What a
    looking turn leaves behind is not only the photograph: it is the `look` call,
    its result, and then the reply the model spoke from the picture -- and that
    reply is the one that does the damage. Take the picture away and leave the
    sentence "I see a room with two black sofas and yellow walls" in the
    transcript, and the next four questions about what is in front of the rover
    are answered from that sentence, word for word, with no call made: 0/6 on
    "what do you see now", "check your camera", "what can you see" and "take
    another photo and describe what you see". Replacing the description with a
    note that a picture was taken measures exactly the same 0/6, and the model
    then reads the note out loud -- it copies its last answer whatever the answer
    was. Leaving the question but not the reply is worse still, because a
    question the model can see it did not answer gets answered with "I took a
    picture to see what's in front of me", by a model that took none.

    With the exchange gone the same four questions take a fresh picture. The cost
    is that a looking turn leaves no trace at all, so a turn that both looked and
    did something else loses the record of the something else; `get_lights` and
    `tracking_status` exist for the state that actually matters.
    """
    groups = _exchanges(history)
    newest = max((i for i, group in enumerate(groups) if _images(group)), default=None)
    if newest is None:
        return
    history[:] = [
        message
        for index, group in enumerate(groups)
        if not _images(group) or (keep_newest and index == newest)
        for message in group
    ]


# Two lists rather than a phrase book, because the sentence comes out differently
# every time -- "I don't have the ability to read or interpret what they look
# like", "I can't tell the colour of the shirt because I can't see it", "I can
# only tell you where they are". What they share is an inability and a word about
# seeing, and it takes both, so that "I can't turn the lights on" and "I don't
# have a name" are left alone. Whole words, not substrings: "already" contains
# "read", which quietly made every sentence with an inability in it a refusal.
# "camera" is deliberately not in the second list -- "I can't reach that far, the
# camera only turns so far" is a true sentence about the gimbal, not a refusal to
# look, and a false positive here eats a turn of somebody's conversation.
_UNABLE = re.compile(
    r"\b(?:can't|cannot|can not|don't have|do not have|unable|not able|only tell)\b")
_SEEING = re.compile(
    r"\b(?:see|seen|seeing|look|looks|looking|picture|photo|image|images|"
    r"describe|describing|read|view|eyes)\b")


def _blind_refusal(reply: str) -> bool:
    """Is this the rover saying it cannot see, rather than looking?"""
    said = reply.lower()
    return bool(_UNABLE.search(said) and _SEEING.search(said))


def _forget_refusals(history: list[dict[str, Any]]) -> None:
    """Drop the exchanges where the rover said it could not see, in place.

    The same mechanism as the pictures above, and the reason this is not a prompt:
    **whatever this model said last, it says again.** One "I can't describe the
    person, I can only tell you where they are" in the transcript and every
    question after it repeats that sentence with no call made -- "but what does he
    look like" 0/6, "what colour is his shirt" 0/6, "describe him for me" 0/6 --
    until the user gives up and says the word "picture" outright, which is 6/6 and
    is what the transcript that reported this bug had to resort to. Drop the
    exchange and the same three questions take a photograph, 6/6, with
    `tracking_status` and `set_lights` still going to the tools that own them.

    Two system prompts were measured against this first, since a rule is the
    obvious repair: "never say you cannot see or describe something, take a
    picture and answer from it" and "if you have said you cannot see something,
    that was wrong". Both left all three questions at 0/6, and both then cost a
    control -- "are you still tracking them" stopped calling `tracking_status` and
    was answered from the transcript instead. The prompt has never once been the
    variable in this file.

    Only exchanges that called nothing: a turn that acted is a turn worth keeping,
    and a refusal spoken *after* a look is about a picture that has already been
    dropped by the rule above. This costs an exchange the user may remember having
    had -- the same price the pictures pay, and for the same reason.
    """
    kept: list[dict[str, Any]] = []
    for group in _exchanges(history):
        if any(m.get("tool_calls") or m.get("role") == "tool" for m in group):
            kept.extend(group)
            continue
        if any(m.get("role") == "assistant" and isinstance(m.get("content"), str)
               and _blind_refusal(m["content"]) for m in group):
            continue
        kept.extend(group)
    history[:] = kept


# Two lists again, and for the same reason: what these sentences share is a
# first person about to act, and a verb belonging to something a tool does. It
# takes both, so "I will be here when you get back" and "I am not sure what you
# mean by late time" are left alone. The present progressive is in the first
# list because the model says "I am starting to follow the person in front of
# me" as readily as "I will" -- with no call made, that is the same sentence.
_PROMISING = re.compile(
    r"\bi(?:'ll|\s+will|\s+am\s+going\s+to|'m\s+going\s+to|\s+am\s+about\s+to|"
    r"\s+am\s+\w+ing)\b")
_DOING = re.compile(
    r"\b(?:turn|turning|switch|switching|set|setting|dim|dimming|brighten|"
    r"start|starting|stop|stopping|follow|following|track|tracking|point|"
    r"pointing|aim|aiming|centre|center|check|checking|take|taking|move|"
    r"moving|look|looking)\b")


def _promised(reply: str) -> bool:
    """Is this the rover saying it is *about to* act, rather than acting?"""
    said = reply.lower()
    return bool(_PROMISING.search(said) and _DOING.search(said))


def _forget_promises(history: list[dict[str, Any]]) -> None:
    """Drop the exchanges where the rover promised an action it never took.

    The third instance of the same law, and the one that reaches the plainest
    requests there are: **whatever this model said last, it says again.** The
    reported session is the whole argument. STT garbled "can you switch the
    lights on" into "can each other lights on", the model answered *"I will turn
    the lights on."* and called nothing -- and from there the conversation was
    over, because every later turn copied the shape instead of acting. Measured
    on the same question with three transcripts in front of it:

        clean history                            6/6
        after "I will turn the lights on."       0/6
        after the same request actually carried
        out, call and result in the history      6/6

    So it is the promise and not the subject. One sentence, and a request that
    is 6/6 on its own becomes one the rover will never perform, however many
    times it is asked -- which is exactly what "nothing is happening" looks like
    from the other end.

    Not gated on vision, unlike the two rules above: a promise is about tools,
    and the rover has had those since before it had a camera.

    Only exchanges that called nothing, for the reason `_forget_refusals` gives:
    a turn that acted is a turn worth keeping, and "I'll keep following him" is
    an honest sentence when `start_tracking` is sitting in the same exchange.

    This does not stop the promise being *spoken* -- the user still hears one
    lie before the rule takes effect, and that would want re-asking the model
    within the turn instead. See the README; it is the piece still missing.
    """
    kept: list[dict[str, Any]] = []
    for group in _exchanges(history):
        if any(m.get("tool_calls") or m.get("role") == "tool" for m in group):
            kept.extend(group)
            continue
        if any(m.get("role") == "assistant" and isinstance(m.get("content"), str)
               and _promised(m["content"]) for m in group):
            continue
        kept.extend(group)
    history[:] = kept


def _measure_image_tokens() -> int:
    """What one frame of the configured size actually costs in the window.

    Measured rather than assumed, because it decides when history is trimmed and
    a wrong constant there is the quiet kind of wrong: too low and the prompt
    overruns the static cache and silently falls back to the dynamic one, too
    high and a conversation is cut short for room nobody needed.
    """
    try:
        from PIL import Image

        probe = Image.new("RGB", (S.VISION_MAX_SIDE, S.VISION_MAX_SIDE * 3 // 4), (128, 128, 128))
        message = [{"role": "user", "content": [{"type": "image"}, {"type": "text", "text": "x"}]}]
        text = _template().apply_chat_template(
            message, tokenize=False, add_generation_prompt=True)
        with_image = S._processor(text=[text], images=[probe], return_tensors="pt")
        return max(int(with_image["input_ids"].shape[-1]) - len(S._tokenizer(text)["input_ids"]), 1)
    except Exception as exc:
        print(f"[voice] cannot measure an image's token cost ({exc}); "
              f"assuming {S.IMAGE_TOKENS}", flush=True)
        return S.IMAGE_TOKENS


def _prompt_len(history: list[dict[str, Any]], tools: list[dict[str, Any]] = ()) -> int:
    """How many tokens this history would become, tool schemas included.

    The unwrapping is not defensive style, it is the fix for a bug that made
    :func:`_trim` inert for the life of this service. `apply_chat_template` with
    `tokenize=True` returns a **BatchEncoding** on transformers 5, not a list of
    ids -- so `len()` of it is 2, the number of keys, and every "does this fit
    the cache" test read `2 <= 1856` and said yes. Nothing was ever trimmed, and
    a long conversation instead quietly fell through to the dynamic cache in
    :func:`_generate`, losing the compiled decode path and getting slower rather
    than failing. Older versions did return a flat list, so both are handled.
    """
    encoded = _template().apply_chat_template(
        [{"role": "system", "content": S.SYSTEM_PROMPT}] + _textual(history),
        tools=list(tools) or None,
        tokenize=True,
        add_generation_prompt=True,
    )
    ids = encoded["input_ids"] if hasattr(encoded, "keys") else encoded
    # A batch of one, if it was asked to return tensors rather than a flat list.
    if ids and isinstance(ids[0], (list, tuple)):
        ids = ids[0]
    # Images are counted rather than rendered. A placeholder is one token in the
    # prompt and several hundred by the time the processor has expanded it, so a
    # history holding a picture measures nearly empty if this is left out -- and
    # the whole point of measuring is to know when the window is full.
    return len(ids) + S._image_tokens * len(_images(history))


def _trim(
    history: list[dict[str, Any]], tools: list[dict[str, Any]] = ()
) -> list[dict[str, Any]]:
    """Drop whole exchanges off the front until the prompt fits the static cache.

    Whole exchanges, not tokens: half a turn in the history reads to the model as
    the user being interrupted, and it starts apologising for things it did not
    do. Leaves room for the reply as well as the prompt, since both share the
    window.

    An exchange is a spoken user message and everything answering it, which is
    no longer always one assistant message -- a turn that called a tool holds
    the call and its result too, and a look holds the picture as a second user
    message. Cutting at every `role=user` splits that picture off as a new
    turn, drops the question that took it, and leaves the model answering a
    caption. Cutting a fixed two entries would strand a call with no result, or
    a result with no call, and a model shown either starts narrating tool
    plumbing out loud.
    """
    budget = S.CACHE_LEN - S.MAX_NEW_TOKENS - 32
    history = list(history)
    # Before dropping any turn off the front, drop the exchanges that looked,
    # bar the newest: a photograph of a room from four turns ago is worth less
    # than the sentences it would cost, and this is the cheaper cut of the two.
    _forget_pictures(history)
    groups = _exchanges(history)
    while len(groups) > 1:
        flat = [message for group in groups for message in group]
        if _prompt_len(flat, tools) <= budget:
            return flat
        groups = groups[1:]
    # The last exchange is left alone however long it is: trimming it away
    # would erase the utterance being answered and hand the model an empty
    # conversation. _generate falls back to the dynamic cache for that case,
    # which is slower but correct.
    return [message for group in groups for message in group]

