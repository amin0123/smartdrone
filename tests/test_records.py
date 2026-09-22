from dataclasses import replace
import json
import pytest
from drone_inspection.store import InspectionRecord, MissionStore


def record():
    return InspectionRecord('2026-09-22T12:00:00+00:00', 1.2, 'plant_1', 'healthy', .9,
                            {'x': 4., 'y': 0., 'z': 3.}, {'x': 4., 'y': 0., 'z': .65},
                            'images/frame.png', 'mission_1')


def test_durable_serialization_and_summary(tmp_path):
    store = MissionStore(tmp_path)
    store.start('mission_1')
    store.add(record())
    store.add(replace(record(), classification='stressed'))
    store.transition('mission_1', 'DOCKED', 'landed_and_disarmed_at_station')
    store.close()
    store = MissionStore(tmp_path)
    summary = json.loads(store.summarize('mission_1', 'READY', 'charged').read_text())
    assert summary['observation_count'] == 2
    assert summary['unique_plants'] == 1
    assert summary['classification_counts'] == {'stressed': 1}
    assert summary['records'][0]['image_path'] == 'images/frame.png'
    store.close()


def test_bad_record_or_path_cannot_silently_serialize(tmp_path):
    with pytest.raises(ValueError):
        replace(record(), drone_position={'x': float('nan'), 'y': 0., 'z': 0.}).to_dict()
    with pytest.raises(ValueError):
        replace(record(), confidence=2).to_dict()
    store = MissionStore(tmp_path)
    with pytest.raises(ValueError):
        store.start('../escape')
    with pytest.raises(ValueError):
        store.add(record())
    store.close()
