"""
Allows for use of ROS publisher nodes. Includes motor and midi publishers.
"""
import math
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

    # WIN_SONG = os.path.join("music", "Victory Robot Song.mid")
    ALERT_SONG = os.path.join("music", "Alert Robot Sound.mid")
    # PIRATE_SONG = os.path.join("music", "pirate.mid")
    midi = MidiPublisher()
    # midi.add_song(ALERT_SONG)

    RUNTIME = 2_000_000_000
    track_a = AudioNoteVector()
    track_a.notes.append(midi.create_note(300, RUNTIME))
    track_b = AudioNoteVector()
    track_b.notes.append(midi.create_note(250, RUNTIME))

    song = (track_a,)

    # midi.play_track(ALERT_SONG)

    TEST = "test_song"
    midi.songs[TEST] = song
    print(midi.songs)
    # time.sleep(1)
    midi.play_track(TEST, 0)

    time.sleep(10)
    # midi.play_track(TEST, 1)


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
        self.TURN_DEGREE_FACTOR = 30
        self.TURN_LOOP_FACTOR = 30
        self.MOVE_DISTANCE_FACTOR = 0.1

    def publish_velocities(
        self, linear: float, angular: float, display_count: bool = False
    ) -> None:
        """
        Publishes velocity values from API to motor channel. Optional flag for
        displaying publish count.
        """
        # Creates a Twist object
        new_twist = Twist()

        # Assigns linear velocity components
        new_twist.linear.x = linear
        # Assigns angular velocity components
        new_twist.angular.z = angular

        # Publishes twist to topic
        self.publisher_.publish(new_twist)
        # Prints counter to console
        if display_count:
            self.get_logger().info(f"Publish #{self.counter + 1}")
        # Increments counter
        self.counter += 1

    def turn_degrees(self, degrees: int) -> None:
        """
        Turns robot in specified direction.
        """
        # Records sign of degrees
        sign = math.copysign(1, degrees)
        # Finds divisor and remainder
        loops, remainder = divmod(abs(degrees), self.TURN_LOOP_FACTOR)

        print(f"loops {loops} remainder {remainder}")
        # Runs loop movement
        for _ in range(loops):
            # Gives no linear and scaled angular velocity
            self.publish_velocities(
                0.0, sign * self.TURN_LOOP_FACTOR / self.TURN_DEGREE_FACTOR
            )
            time.sleep(0.5)
        # Runs remainder movement
        self.publish_velocities(0.0, sign * remainder / self.TURN_DEGREE_FACTOR)

    def move_distance(self, distance: int) -> None:
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

    def __init__(self, gap_duration: int = 1, *midi_filepaths: str) -> None:
        """
        Initializes midi publisher with specified gap duration and midi
        files.
        """
        # Runs default node initializations with created name
        super().__init__("midi_publisher")
        # Creates a publisher through which audio information can be sent
        self.publisher_ = self.create_publisher(AudioNoteVector, "cmd_audio", 10)
        # Creates a dictionary of filepaths and track vectors
        self.songs: dict[str, tuple[AudioNoteVector, ...]] = {}
        # For each path specified:
        for filepath in midi_filepaths:
            # Adds to track bundle dictionary
            self.add_song(filepath)
        # Assigns gap duration
        self.GAP_DURATION = gap_duration

    def add_song(self, midi_filepath: str) -> None:
        """
        Adds a midi file to publisher's accessible songs.
        """
        # Readies midi data from path
        midi_data = mido.MidiFile(midi_filepath)
        # Initiates a track list
        song: list[AudioNoteVector] = []
        # For each track in midi data:
        for midi_track in midi_data.tracks:
            # Converts midi track data into custom audio vector
            track = self._build_track(midi_track, midi_data.ticks_per_beat)
            # If there are any notes in the vector:
            if track.notes:
                # Add track vector to song
                song.append(track)
        # If song contains any vectors with notes:
        if song:
            # Adds track list to dictionary of songs
            self.songs[midi_filepath] = tuple(song)
        else:
            print(
                f"WARNING: {midi_filepath} contains no tracks with notes! Was not added to bundles."
            )

    def _build_track(self, midi_track: list, ticks_per_beat: int) -> AudioNoteVector:
        """
        Converts midi track into AudioNoteVector object. Optional flag for
        gap notes.
        """
        # Calculates nanoseconds per tick from default microseconds per beat
        nanoseconds_per_tick = int(500000 / ticks_per_beat * 1e3)
        previous_frequency = 0
        delta_time = 0.0
        # Creates note vector object for a track
        track = AudioNoteVector()
        track.header.stamp = rclpy.time.Time().to_msg()
        # For each message in track:
        for message in midi_track:
            # Adds message time to delta time
            delta_time += message.time
            # If tempo message:
            if message.type == "set_tempo":
                # Calculates nanoseconds per beat from microseconds per beat
                nanoseconds_per_tick = int(message.tempo / ticks_per_beat * 1e3)
            # If note event message:
            if message.type in ("note_on", "note_off"):
                # Defines duration of seperation note
                gap_duration = int(self.GAP_DURATION * (delta_time > self.GAP_DURATION))
                # Addes gap note to vector
                track.notes.append(
                    self.create_note(0, gap_duration * nanoseconds_per_tick)
                )
                # Addes previous note to vector
                track.notes.append(
                    self.create_note(
                        previous_frequency,
                        (delta_time - gap_duration) * nanoseconds_per_tick,
                    )
                )
                # Sets previous frequency to current by converting from midi note
                previous_frequency = int(
                    bool(message.velocity) * 440 * 2 ** ((message.note - 69) / 12)
                )
                # Resets delta time
                delta_time = 0.0

        # print(f"Track notes: {track.notes}")
        return track

    def create_note(self, frequency: int, nanosecond_runtime: int) -> AudioNote:
        """
        Utility function that creates note through AudioNote and Duration
        objects.
        """
        runtime = Duration()
        runtime.nanosec = int(nanosecond_runtime)
        note = AudioNote()
        note.frequency = int(frequency)
        note.max_runtime = runtime
        return note

    def play_track(self, midi_filepath: str, track_number: int = 0) -> None:
        """
        Plays midi sequence on robot. Optional track number specification.
        """

        # If filepath not in note vector list:
        if midi_filepath not in self.songs:
            raise ValueError("Sequence does not exist for specified filepath.")
        # Acquires vector bundle from list
        song = self.songs[midi_filepath]
        # If track number does not exist:
        if not (0 <= track_number < len(song)):
            raise ValueError("Track number does not exist for specified filepath.")
        # Publish the vector
        print(song[track_number])
        self.publisher_.publish(song[track_number])


if __name__ == "__main__":
    # Runs the main function
    main()
