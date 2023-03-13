"""
Allows for access to the base API and fascilitates extraction.
"""

import requests

__author__ = "Ben Kraft"
__copyright__ = "None"
__credits__ = "Ben Kraft"
__license__ = "MIT"
__version__ = "1.0"
__maintainer__ = "Ben Kraft"
__email__ = "benjamin.kraft@tufts.edu"
__status__ = "Prototype"

# Defines Airtable information
BASE_ID = "appWNQwSNORmWZJQH"
TABLE_NAME = "Robot Data"
API_KEY = input("Enter Airtable API Key: ")
# Builds API url
URL = (
    "https://api.airtable.com/v0/" + BASE_ID + "/" + TABLE_NAME + "?api_key=" + API_KEY
)


def main() -> None:
    """
    Runs default actions.
    """
    # Prints sample velocities
    print(get_velocities("Linear", "Angular"))


def get_velocities(*velocity_types: str) -> tuple[float, ...]:
    """
    Acquires specified velocity components from API.
    """
    # Acquires raw data from url
    raw_records = requests.get(URL).json()["records"]
    # Returns components found
    return tuple(extract_velocity(raw_records, velocity) for velocity in velocity_types)


def extract_velocity(
    raw_records: list[dict[str, dict[str, str]]],
    velocity_type: str,
) -> float:
    """
    Searches records list for correct velocity type and returns it if found.
    """
    # Defines velocity key name
    VELOCITY_KEY = "Move State"
    IDENTIFIER_KEY = "Name"
    # For each record in list
    for record in raw_records:
        # Acquires field data
        field_data = record["fields"]
        # Verifies that both identifier and velocity key exist in record
        if not (VELOCITY_KEY in field_data and IDENTIFIER_KEY in field_data):
            continue
        # Returns corresponding provided velocity as a float
        if field_data[IDENTIFIER_KEY] == velocity_type:
            return float(field_data[VELOCITY_KEY])
    # All else return 0
    return 0


if __name__ == "__main__":
    main()
