"""
Allows for access to the base API and fascilitates extraction.
"""

import requests

BASE_ID = "appWNQwSNORmWZJQH"
TABLE_NAME = "Robot Data"
API_KEY = input("Enter Airtable API Key: ")

URL = (
    "https://api.airtable.com/v0/" + BASE_ID + "/" + TABLE_NAME + "?api_key=" + API_KEY
)


def main() -> None:
    """
    Runs default actions.
    """
    # Prints sample velocities
    print(get_velocities("Linear", "Angular"))


def get_velocities(*velocity_types: str) -> tuple[int, ...]:
    """
    Acquires specified velocity components from API.
    """
    # Acquires raw data from url
    raw_records: list[dict[str, dict[str, str]]] = requests.get(URL).json()["records"]
    # Returns components found
    return tuple(extract_velocity(raw_records, velocity) for velocity in velocity_types)


def extract_velocity(
    raw_records: list[dict[str, dict[str, str]]],
    velocity_type: str,
) -> int:
    """
    Searches records list for correct velocity type and returns it if found.
    """
    # Defines velocity key name
    VELOCITY_KEY = "Move State"
    # For each record in list
    for record in raw_records:
        # Acquires field data
        field_data = record["fields"]
        # If correct name and has velocity, returns that velocity as a float
        if field_data["Name"] == velocity_type and VELOCITY_KEY in field_data:
            return int(field_data[VELOCITY_KEY])
    # All else return 0
    return 0


if __name__ == "__main__":
    main()
