"""Repo paths shared by GPS / time-align tests."""

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
TOOLS_DIR = REPO_ROOT / "tools"
UNPROCESSED_DIR = REPO_ROOT / "unprocessed"
DOCS_DIR = REPO_ROOT / "docs"

FAKE_GPS_CSV = UNPROCESSED_DIR / "fake_gps.csv"
FAKE_SESSION_README = UNPROCESSED_DIR / "README.md"
SENSORLOG_SCHEMA_DOC = DOCS_DIR / "SENSORLOG_CSV.md"
FIXTURE_SFDAT = UNPROCESSED_DIR / "ride_20260601_143000.sfdat"
