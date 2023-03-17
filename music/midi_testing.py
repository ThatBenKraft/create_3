import mido

midi_data = mido.MidiFile("test_midi.mid")

midi_data.print_tracks()


for track_index, track in enumerate(midi_data.tracks):

    note_list: list[tuple[int, float]] = []

    note_count = 0
    # if track.
    for message_index, message in enumerate(track):

        if message.type == "note_on":

            if not note_count:

                # initial_rest_duration = track

                note_list.append((0, 0.0))

            note_count += 1

    print(f"Track {track_index}:\n{note_list}")

# midi_data.play()
