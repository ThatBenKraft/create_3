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

    midi = MidiPublisher(PIRATE_SONG)
    motors = MotorPublisher()

    time.sleep(1)
    midi.play_sequence(PIRATE_SONG)


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

    def __init__(self, *midi_filepaths: str) -> None:
        # Runs default node initializations with created name
        super().__init__("midi_publisher")
        # Creates a publisher through which audio information can be sent
        self.publisher_ = self.create_publisher(AudioNoteVector, "cmd_audio", 10)
        # Creates a dictionary of filepaths and converted note vectors
        self.vector_bundles: dict[str, tuple[AudioNoteVector, ...]]
        for path in midi_filepaths:
            midi_data = mido.MidiFile(path)
            bundle = tuple(
                self._create_vector(track, midi_data.ticks_per_beat)
                for track in midi_data.tracks
            )
            self.vector_bundles[path] = bundle

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
        Converts midi track into AudioNoteVector.
        """
        # Sets default tempo of 120BPM
        microseconds_per_beat = 500000

        track_length = len(track)
        # Iterates through messages to find track / note info
        note_sequence: list[AudioNote] = []
        # For each message within the first track:
        for index, message in enumerate(track):
            # If tempo message:
            if message.type == "set_tempo":
                # Assigns microseconds per beat
                microseconds_per_beat: int = message.tempo
            # If note message:
            elif message.type == "note_on":
                # Calculates frequency from note value or sets to 0 if no velocity
                frequency = (
                    440 * (2 ** ((message.note - 69) / 12)) if message.velocity else 0
                )

                delta_ticks = (
                    track[index + 1].time if index < track_length else message.time
                )

                nanoseconds_per_tick = int(microseconds_per_beat / ticks_per_beat * 1e3)

                # Creates note object
                note = AudioNote()
                note.frequency = frequency
                # Creates duration object
                duration = Duration()
                duration.sec = 0
                duration.nanosec = delta_ticks * nanoseconds_per_tick
                note.max_runtime = duration

                # Bundles with time and addes to raw note sequence
                note_sequence.append(note)

        # Calculates nanoseconds per tick
        print(note_sequence)

        # Creates note vector object
        note_vector = AudioNoteVector()
        note_vector.header.stamp = rclpy.time.Time().to_msg()
        note_vector.notes = note_sequence

        return note_vector

    def play_sequence(self, midi_filepath: str, track_number: int = 0) -> None:
        """
        Plays midi sequence on robot. Optional track number specification
        """

        # If filepath not in note vector list:
        if midi_filepath not in self.vector_bundles:
            raise ValueError("Sequence does not exist for specified filepath.")
        # Acquires vector bundle from list
        bundle: tuple[AudioNoteVector, ...] = self.vector_bundles[midi_filepath]
        # If track number does not exist:
        if not (0 <= track_number < len(bundle)):
            raise ValueError("Track number does not exist for specified filepath.")
        # Publish the vector
        self.publisher_.publish(bundle[track_number])


if __name__ == "__main__":
    # Runs the main function
    main()
