import json
import os

DATA_PATH = os.path.join(os.path.dirname(__file__), '..', 'data', 'scale_example.json')

def test_scale_example_json_exists():
    assert os.path.exists(DATA_PATH), "scale example JSON missing"

def test_scale_example_json_structure():
    with open(DATA_PATH) as f:
        data = json.load(f)
    assert 'cameras' in data
    assert 'frames' in data
    assert len(data['frames']) == 5
    assert 'back_camera' in data['cameras']
    frame0 = data['frames'][0]
    assert 'pose' in frame0
    assert 'radar_points' in frame0
    assert 'point_sample' in frame0


def test_scale_example_json_images():
    with open(DATA_PATH) as f:
        data = json.load(f)
    assert 'images' in data
    cameras = data['cameras']
    frames = data['frames']
    expected_count = len(cameras) * len(frames)
    assert len(data['images']) == expected_count
