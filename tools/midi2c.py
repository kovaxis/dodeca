
# Library: mido v1.2.9
from mido import MidiFile
import os
import math
import sys

directory = "."
if len(sys.argv) >= 2:
    directory = sys.argv[1]

midi_ext = {'.mid', '.midi'}
files = list(filter(lambda entry: os.path.splitext(
    entry)[1].lower() in midi_ext, os.listdir(directory)))
files.sort()

if len(files) <= 0:
    print(f"No midi files found in directory \"{directory}\"")
    exit()

# Add a couple of blank lines to make copying easier
print()
print()

for path in files:
    file_name = os.path.splitext(path)[0]
    mid = MidiFile(directory+"/"+path)
    abs_time = 0
    time_acc = 0
    active_note = None
    finished = None
    out = []

    def write_out(note, secs):
        if secs > 0:
            freq = 0
            if note != None:
                freq = math.floor(440 * 2**((note - 69) / 12) + 0.5)
            dur = math.floor(secs * 1000 + 0.5)
            if len(out) > 0 and dur < 10 and freq == 0:
                last_freq, last_dur = out[-1]
                out[-1] = (last_freq, last_dur + dur)
            else:
                out.append((freq, dur))

    # Parse MIDI
    for msg in mid:
        time_acc += msg.time
        abs_time += msg.time
        if msg.type == 'note_off' or (msg.type == 'note_on' and msg.velocity == 0):
            if time_acc > 0 and active_note != None:
                write_out(active_note, time_acc)
                time_acc = 0
                active_note = None
        elif msg.type == 'note_on':
            if active_note != None:
                raise Exception(
                    f"simultaneous notes {active_note} and {msg.note} at {abs_time}!")
            if finished:
                raise Exception(f"found note after explicit finish")
            if time_acc > 0:
                write_out(None, time_acc)
                time_acc = 0
            active_note = msg.note
        elif msg.type == 'text':
            txt = msg.text.strip().lower()
            if txt == 'stop' or txt == 'loop':
                if time_acc > 0:
                    write_out(active_note, time_acc)
                    time_acc = 0
                    active_note = None
                if txt == 'loop':
                    out.append((1, 0))
                else:
                    out.append((0, 0))
                finished = True
    if active_note != None:
        raise Exception(f"note {active_note} is never stopped!")

    # Add finalizer
    if not finished:
        out.append((0, 0))

    # Print out converted MIDI
    length = sum(map(lambda pair: pair[1], out))
    print(f"// Generated from \"{path}\" ({length}ms long)")
    print(f"const PROGMEM Tone {file_name.upper()}_SEQUENCE[] = {{")
    for pair in out:
        print(f"    {{{pair[0]}, {pair[1]}}},")
    print(f"}};")
    print()
