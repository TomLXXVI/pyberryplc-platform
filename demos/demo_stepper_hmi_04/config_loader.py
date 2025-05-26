import tomllib
from pathlib import Path


def load_motor_config_toml(file_path: str) -> dict:
    with Path(file_path).open("rb") as f:
        return tomllib.load(f)
