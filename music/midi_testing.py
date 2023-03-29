import mido

midi_data = mido.MidiFile("test_midi.mid")

midi_data.print_tracks()


SEPARATION_LENGTH = 1


for track_index, track in enumerate(midi_data.tracks):

    note_list: list[tuple[int, float]] = []

    microseconds_per_beat = 500000
    previous_frequency = 0
    delta_time = 0.0

    note_count = 0
    # For each message in track:
    for message in track:
        # Adds message time to delta time
        delta_time += message.time
        # If tempo message:
        if message.type == "set_tempo":
            # Assigns microseconds per beat
            microseconds_per_beat = message.tempo
        # If note event message:
        if message.type in ("note_on", "note_off"):
            # Defines duration of seperation note
            separation_duration = float(
                SEPARATION_LENGTH * (delta_time > SEPARATION_LENGTH)
            )
            # Addes separation and previous note to list
            note_list.append((0, separation_duration))
            note_list.append((previous_frequency, delta_time - separation_duration))
            # Sets previous frequency to current by converting from midi note
            previous_frequency = int(
                bool(message.velocity) * 440 * 2 ** ((message.note - 69) / 12)
            )
            # Resets delta time
            delta_time = 0.0
            # Adds to note count
            note_count += 1

    print(f"Track {track_index}:\n{note_list}")
