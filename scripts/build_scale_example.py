import json
import os
from plyfile import PlyData

DATASET_PATH = os.path.join(os.path.dirname(__file__), '..', 'tmp_scale', 'scale_format', 'data')
OUTPUT_PATH = os.path.join(os.path.dirname(__file__), '..', 'data', 'scale_example.json')

CAMERAS = ['back_camera', 'front_camera', 'front_left_camera']


def load_camera(camera_dir: str):
    with open(os.path.join(camera_dir, 'extrinsics.json')) as f:
        extrinsics = json.load(f)
    with open(os.path.join(camera_dir, 'intrinsics.json')) as f:
        intrinsics = json.load(f)
    return {'extrinsics': extrinsics, 'intrinsics': intrinsics}


def load_radar_points(path: str):
    points = []
    with open(path) as f:
        for line in f:
            stripped = line.strip()
            if not stripped:
                continue
            points.append([float(x) for x in stripped.split(',')])
    return points


def load_point_sample(path: str, limit: int = 100):
    ply = PlyData.read(path)
    data = ply['vertex'].data
    sample = []
    for row in data[:limit]:
        sample.append([float(row['x']), float(row['y']), float(row['z']), float(row['i'])])
    return sample


def build_dataset():
    cameras = {}
    for name in CAMERAS:
        cameras[name] = load_camera(os.path.join(DATASET_PATH, 'cameras', name))

    frames = []
    for idx in range(1, 6):
        with open(os.path.join(DATASET_PATH, 'poses', f'{idx}.json')) as f:
            pose = json.load(f)
        radar_path = os.path.join(DATASET_PATH, 'radar_points', f'{idx}.txt')
        radar = load_radar_points(radar_path)
        pc_path = os.path.join(DATASET_PATH, 'pointcloud', f'{idx}.ply')
        points = load_point_sample(pc_path)
        frames.append({'frame_id': idx, 'pose': pose, 'radar_points': radar, 'point_sample': points})

    dataset = {'cameras': cameras, 'frames': frames}
    os.makedirs(os.path.dirname(OUTPUT_PATH), exist_ok=True)
    with open(OUTPUT_PATH, 'w') as f:
        json.dump(dataset, f, indent=2)


if __name__ == '__main__':
    build_dataset()
