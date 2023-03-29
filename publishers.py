"""
Allows for use of ROS publisher nodes. Includes motor and midi publishers.
"""
import os
import time

import mido
import rclpy  # type: ignore
from builtin_interfaces.msg import Duration  # type: ignore
from geometry_msgs.msg import Twist  # type: ignore
from irobot_create_msgs.msg import AudioNote, AudioNoteVector  # type: ignore
from rclpy.node import Node  # type: ignore

__author__ = "Ben Kraft"
__copyright__ = "None"
__credits__ = "Ben Kraft, Maddie Pero"
__license__ = "MIT"
__version__ = "1.0"
__maintainer__ = "Ben Kraft"
__email__ = "benjamin.kraft@tufts.edu"
__status__ = "Prototype"


rclpy.init()


def main() -> None:
    """
    Runs default publisher actions.
    """

    WIN_SONG = os.path.join("music", "Test Robot Song.mid")
    ALERT_SONG = os.path.join("music", "Alert Robot Sound.mid")
    PIRATE_SONG = os.path.join("music", "pirate.mid")

    midi = MidiPublisher(1, PIRATE_SONG)

    time.sleep(1)
    midi.play_track(PIRATE_SONG)


class MotorPublisher(Node):
    """
    Allows for publication of movement information on motor channel.
    """

    # Defines class constructor
    def __init__(self) -> None:
        # Runs default node initializations with created name
        super().__init__("motor_publisher")
        # Creates a publisher based on the message type "Twist" that has been imported from the std_msgs module above
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)
        # Sets initial counter to zero
        self.counter = 0
        # Defines constants
        self.TURN_DEGREE_FACTOR = 0.78
        self.TURN_LOOP_COUNT = 4
        self.MOVE_DISTANCE_FACTOR = 0.1

    def publish_velocities(
        self, linear: float, angular: float, display_count: bool = False
    ):
        """
        Publishes velocity values from API to motor channel. Optional flag for
        displaying publish count.
        """
        # Creates a Twist object
        new_twist = Twist()

        print(
            f"Twist things: x:{new_twist.linear.x}   y:{new_twist.linear.y}   z:{new_twist.linear.z}"
        )
        # Assigns linear velocity components
        new_twist.linear.x = linear
        new_twist.linear.y = 0.0
        new_twist.linear.z = 0.0
        # Assigns angular velocity components
        new_twist.angular.x = 0.0
        new_twist.angular.y = 0.0
        new_twist.angular.z = angular

        # Publishes twist to topic
        self.publisher_.publish(new_twist)
        # Prints counter to console
        if display_count:
            self.get_logger().info(f"Publish #{self.counter + 1}")
        # Increments counter
        self.counter += 1

    def turn_direction(self, direction: int) -> None:
        """
        Turns robot in specified direction.
        """
        for _ in range(self.TURN_LOOP_COUNT):
            # Gives no linear and scaled angular velocity
            self.publish_velocities(0.0, direction * self.TURN_DEGREE_FACTOR)
            time.sleep(0.5)

    def move_distance(self, distance: int):
        """
        Moves robot forward specified amount.
        """

        sign = -1 if distance < 0 else 1
        for _ in range(abs(distance)):
            # Gives scaled linear and no angular velocity
            self.publish_velocities(sign * self.MOVE_DISTANCE_FACTOR, 0.0)
            time.sleep(0.5)


class MidiPublisher(Node):
    """
    Allows for publication of midi events to audio channel.
    """

    def __init__(self, separation_length: int = 1, *midi_filepaths: str) -> None:
        """
        Initializes midi publisher with specified separation length and midi
        files.
        """
        # Runs default node initializations with created name
        super().__init__("midi_publisher")
        # Creates a publisher through which audio information can be sent
        self.publisher_ = self.create_publisher(AudioNoteVector, "cmd_audio", 10)
        # Creates a dictionary of filepaths and converted note vectors
        self.vector_bundles: dict[str, list[AudioNoteVector]] = {}
        # For each path specified:
        for filepath in midi_filepaths:
            # Adds to track bundle dictionary
            self.add_file(filepath)
        # Assigns separation length
        self.SEPARATION_LENGTH = separation_length

    def add_file(self, midi_filepath: str):
        """
        Adds a midi file to publisher's accessible track bundle dictionary.
        """
        # Readies midi data from path
        midi_data = mido.MidiFile(midi_filepath)
        # Initiates a track bundle
        vector_bundle: list[AudioNoteVector] = []
        # For each track in midi data:
        for track in midi_data.tracks:
            # Converts track data into custom audio vector
            vector = self._create_vector(track, midi_data.ticks_per_beat)
            # If there are any notes in the vector:
            if vector.notes:
                # Add vector to bundle
                vector_bundle.append(vector)
        # If bundle contains any tracks with notes:
        if vector_bundle:
            # Adds to dictionary of bundles
            self.vector_bundles[midi_filepath] = vector_bundle
        else:
            print(
                f"WARNING: {midi_filepath} contains no tracks with notes! Was not added to bundles."
            )

    # def _create_vector(self, track: list, ticks_per_beat: int) -> AudioNoteVector:
    #     """
    #     Converts midi track into AudioNoteVector.
    #     """
    #     # Sets default tempo of 120BPM
    #     microseconds_per_beat = 500000

    #     # Iterates through messages to find track / note info
    #     raw_sequence: list[tuple[int, int]] = []
    #     # For each message within the first track:
    #     for message in track:
    #         # If tempo message:
    #         if message.type == "set_tempo":
    #             # Assigns microseconds per beat
    #             microseconds_per_beat: int = message.tempo
    #         # If note message:
    #         elif message.type == "note_on":
    #             # Calculates frequency from note value or sets to 0 if no velocity
    #             frequency = (
    #                 440 * (2 ** ((message.note - 69) / 12)) if message.velocity else 0
    #             )
    #             # Bundles with time and addes to raw note sequence
    #             raw_sequence.append((int(frequency), message.time))
    #     # Calculates nanoseconds per tick
    #     nanoseconds_per_tick = int(microseconds_per_beat / ticks_per_beat * 1e3)
    #     print(raw_sequence)
    #     # Sets up values to use in note creation loop
    #     sequence_length = len(raw_sequence)
    #     note_sequence: list[AudioNote] = []
    #     # For each raw note
    #     for index, raw_note in enumerate(raw_sequence):
    #         # Looks at the next raw note's delta time and uses it as the
    #         # current note's duration
    #         next_index = index + 1
    #         next_duration_ticks = (
    #             raw_sequence[next_index][1]
    #             if next_index < sequence_length
    #             else raw_note[1]
    #         )
    #         # Creates note object
    #         note = AudioNote()
    #         note.frequency = raw_note[0]
    #         # Creates duration object
    #         duration = Duration()
    #         duration.sec = 0
    #         duration.nanosec = next_duration_ticks * nanoseconds_per_tick
    #         note.max_runtime = duration
    #         # Adds note to sequence
    #         note_sequence.append(note)
    #     # Creates note vector object
    #     note_vector = AudioNoteVector()
    #     note_vector.header.stamp = rclpy.time.Time().to_msg()
    #     note_vector.notes = note_sequence

    #     return note_vector

    def _create_vector(self, track: list, ticks_per_beat: int) -> AudioNoteVector:
        """
        Converts midi track into AudioNoteVector object. Optional flag for
        separation notes.
        """
        # Calculates nanoseconds per beat from microseconds per beat
        nanoseconds_per_tick = int(500000 / ticks_per_beat * 1e3)
        previous_frequency = 0
        delta_time = 0.0
        # Creates note vector object
        note_vector = AudioNoteVector()
        note_vector.header.stamp = rclpy.time.Time().to_msg()
        # For each message in track:
        for message in track:
            # Adds message time to delta time
            delta_time += message.time
            # If tempo message:
            if message.type == "set_tempo":
                # Calculates nanoseconds per beat from microseconds per beat
                nanoseconds_per_tick = int(message.tempo / ticks_per_beat * 1e3)
            # If note event message:
            if message.type in ("note_on", "note_off"):
                # Defines duration of seperation note
                separation_length = float(
                    self.SEPARATION_LENGTH * (delta_time > self.SEPARATION_LENGTH)
                )
                # Addes separation note to vector
                separation_note = AudioNote()
                separation_note.frequency = 0
                separation_duration = Duration()
                separation_duration.sec = 0
                separation_duration.nanosec = separation_length * nanoseconds_per_tick
                separation_note.max_runtime = separation_duration
                note_vector.notes.append(separation_note)
                # Addes previous note to vector
                previous_note = AudioNote()
                previous_note.frequency = previous_frequency
                previous_duration = Duration()
                previous_duration.sec = 0
                previous_duration.nanosec = (
                    delta_time - separation_length
                ) * nanoseconds_per_tick
                previous_note.max_runtime = previous_duration
                note_vector.notes.append(previous_note)
                # Sets previous frequency to current by converting from midi note
                previous_frequency = int(
                    bool(message.velocity) * 440 * 2 ** ((message.note - 69) / 12)
                )
                # Resets delta time
                delta_time = 0.0

        print(f"Track notes: {note_vector.notes}")
        return note_vector

    def play_track(self, midi_filepath: str, track_number: int = 0) -> None:
        """
        Plays midi sequence on robot. Optional track number specification.
        """

        # If filepath not in note vector list:
        if midi_filepath not in self.vector_bundles:
            raise ValueError("Sequence does not exist for specified filepath.")
        # Acquires vector bundle from list
        bundle = self.vector_bundles[midi_filepath]
        # If track number does not exist:
        if not (0 <= track_number < len(bundle)):
            raise ValueError("Track number does not exist for specified filepath.")
        # Publish the vector
        self.publisher_.publish(bundle[track_number])


if __name__ == "__main__":
    # Runs the main function
    main()
