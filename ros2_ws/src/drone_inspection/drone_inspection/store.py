"""Durable local inspection records, transitions and atomic mission summaries."""
from collections import Counter
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
import sqlite3


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


@dataclass(frozen=True)
class InspectionRecord:
    timestamp: str
    image_timestamp: float
    plant_id: str
    classification: str
    confidence: float
    drone_position: dict[str, float]
    approximate_position: dict[str, float]
    image_path: str
    mission_id: str
    location_method: str = 'calibrated_camera_canopy_plane_intersection'

    def to_dict(self) -> dict:
        if not 0 <= self.confidence <= 1 or not math.isfinite(self.image_timestamp):
            raise ValueError('Invalid confidence or timestamp')
        if self.classification not in {'healthy', 'stressed', 'anomaly'}:
            raise ValueError('Unknown classification')
        for position in (self.drone_position, self.approximate_position):
            if set(position) != {'x', 'y', 'z'} or not all(math.isfinite(v) for v in position.values()):
                raise ValueError('Positions must contain finite x/y/z')
        result = asdict(self)
        json.dumps(result, allow_nan=False)
        return result


class MissionStore:
    def __init__(self, directory: Path):
        self.directory = Path(directory).expanduser().resolve()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.db = sqlite3.connect(self.directory / 'inspection.sqlite3')
        self.db.execute('PRAGMA journal_mode=WAL')
        self.db.execute('CREATE TABLE IF NOT EXISTS missions (id TEXT PRIMARY KEY, started TEXT NOT NULL, status TEXT NOT NULL)')
        self.db.execute('CREATE TABLE IF NOT EXISTS detections (id INTEGER PRIMARY KEY, mission_id TEXT NOT NULL, record_json TEXT NOT NULL)')
        self.db.execute('CREATE TABLE IF NOT EXISTS transitions (id INTEGER PRIMARY KEY, mission_id TEXT NOT NULL, stamp TEXT NOT NULL, state TEXT NOT NULL, reason TEXT NOT NULL)')
        self.db.commit()

    def mission_dir(self, mission_id: str) -> Path:
        if not re.fullmatch(r'[A-Za-z0-9_-]{1,80}', mission_id):
            raise ValueError('Invalid mission identifier')
        return self.directory / mission_id

    def start(self, mission_id: str) -> None:
        self.mission_dir(mission_id).mkdir(exist_ok=True)
        with self.db:
            self.db.execute('INSERT OR IGNORE INTO missions VALUES (?, ?, ?)', (mission_id, utc_now(), 'STARTED'))

    def add(self, record: InspectionRecord) -> None:
        payload = record.to_dict()
        if not self.db.execute('SELECT 1 FROM missions WHERE id=?', (record.mission_id,)).fetchone():
            raise ValueError('Record has no active mission')
        with self.db:
            self.db.execute('INSERT INTO detections (mission_id, record_json) VALUES (?, ?)',
                            (record.mission_id, json.dumps(payload, allow_nan=False)))

    def transition(self, mission_id: str, state: str, reason: str) -> None:
        with self.db:
            self.db.execute('INSERT INTO transitions (mission_id, stamp, state, reason) VALUES (?, ?, ?, ?)',
                            (mission_id, utc_now(), state, reason))
            self.db.execute('UPDATE missions SET status=? WHERE id=?', (state, mission_id))

    def summarize(self, mission_id: str, status: str, reason: str) -> Path:
        directory = self.mission_dir(mission_id)
        records = [json.loads(row[0]) for row in self.db.execute(
            'SELECT record_json FROM detections WHERE mission_id=? ORDER BY id', (mission_id,))]
        classes: dict[str, str] = {}
        # Prefer an observed anomaly over a healthy observation for the summary.
        severity = {'healthy': 0, 'stressed': 1, 'anomaly': 2}
        for record in records:
            old = classes.get(record['plant_id'], 'healthy')
            if record['plant_id'] not in classes or severity[record['classification']] >= severity[old]:
                classes[record['plant_id']] = record['classification']
        summary = {'mission_id': mission_id, 'generated_at': utc_now(), 'status': status, 'reason': reason,
                   'observation_count': len(records), 'unique_plants': len(classes),
                   'classification_counts': dict(Counter(classes.values())),
                   'location_accuracy': 'approximate; synthetic RGB classification, not a crop diagnosis',
                   'records': records,
                   'transitions': [{'timestamp': t, 'state': s, 'reason': r} for t, s, r in
                       self.db.execute('SELECT stamp, state, reason FROM transitions WHERE mission_id=? ORDER BY id', (mission_id,))]}
        path = directory / 'summary.json'
        temporary = directory / 'summary.json.tmp'
        temporary.write_text(json.dumps(summary, indent=2, allow_nan=False))
        temporary.replace(path)
        return path

    def close(self) -> None:
        self.db.close()
