from pathlib import Path
import runpy


def main() -> None:
    entry = Path(__file__).resolve().parents[1] / "main.py"
    runpy.run_path(str(entry), run_name="__main__")
