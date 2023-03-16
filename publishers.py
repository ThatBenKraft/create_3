"""
Allows for use of ROS publisher nodes. Includes motor and midi publishers.
"""
import os
import time

import rclpy  # type: ignore
from builtin_interfaces.msg import Duration  # type: ignore
from geometry_msgs.msg import Twist  # type: ignore
from irobot_create_msgs.msg import AudioNote, AudioNoteVector  # type: ignore
from mido import MidiFile
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
        self.note_vectors: dict[str, AudioNoteVector] = {
            path: self._create_vector(MidiFile(path)) for path in midi_filepaths
        }

    def _create_vector(self, midi_data: MidiFile) -> AudioNoteVector:
        """
        Converts midi data format into AudioNoteVector. Only translates
        single-track midi files.
        """
        # Sets default tempo of 120BPM
        microseconds_per_beat = 500000

        # Iterates through messages to find track / note info
        raw_sequence: list[tuple[int, int]] = []
        # For each message within the first track:
        for message in midi_data.tracks[0]:
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
                # Bundles with time and addes to raw note sequence
                raw_sequence.append((int(frequency), message.time))
        # Calculates nanoseconds per tick
        nanoseconds_per_tick = int(
            microseconds_per_beat / midi_data.ticks_per_beat * 1e3
        )

        # Sets up values to use in note creation loop
        sequence_length = len(raw_sequence)
        note_sequence: list[AudioNote] = []
        # For each raw note
        for index, raw_note in enumerate(raw_sequence):
            # Looks at the next raw note's delta time and uses it as the
            # current note's duration
            next_index = index + 1
            next_duration = (
                raw_sequence[next_index][1]
                if next_index < sequence_length
                else raw_note[1]
            )
            # Creates note object
            note = AudioNote()
            note.frequency = raw_note[0]
            # Creates duration object
            duration = Duration()
            duration.sec = 0
            duration.nanosec = next_duration * nanoseconds_per_tick
            note.max_runtime = duration
            # Adds note to sequence
            note_sequence.append(note)
        # Creates note vector object
        note_vector = AudioNoteVector()
        note_vector.header.stamp = rclpy.time.Time().to_msg()
        note_vector.notes = note_sequence

        return note_vector

    def play_sequence(self, midi_filepath: str) -> None:
        """
        Plays midi sequence on robot.
        """
        # If filepath in note vector list:
        if midi_filepath in self.note_vectors:
            # Publish the vector
            self.publisher_.publish(self.note_vectors[midi_filepath])
        else:
            raise ValueError("Sequence does not exist for specified filepath.")


def main() -> None:
    """
    Runs default publisher actions using Airtable API.
    """
    motors = MotorPublisher()

    WIN_SONG = os.path.join("music", "Test Robot Song.mid")
    ALERT_SONG = os.path.join("music", "Alert Robot Sound.mid")
    midi = MidiPublisher(WIN_SONG, ALERT_SONG)

    # fun_midi = MidiPublisher(os.path.join("music", "Test Robot Song.mid"))
    # alert_midi = MidiPublisher(os.path.join("music", "Alert Robot Sound.mid"))

    # time.sleep(1)

    # fun_midi.play_sequence()

    # # time.sleep(1)

    # # motors.move_distance(1)

    # time.sleep(3)

    # alert_midi.play_sequence()

    motors.turn_direction(-1)
    time.sleep(1)
    motors.turn_direction(1)

    time.sleep(1)

    # motors.turn_direction(1)
    # motors.turn_direction(1)

    # time.sleep(1)

    motors.move_distance(-2)
    time.sleep(2)
    motors.move_distance(2)


if __name__ == "__main__":
    # Runs the main function
    main()
