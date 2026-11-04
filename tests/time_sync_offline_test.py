def test_cli():
    """Real CLI regression fixtures; keep generated files for inspection (no bulk cleanup)."""
    import pathlib
    import subprocess
    import sys
    import uuid
    
    binary = pathlib.Path(sys.argv[1]).resolve()
    root = pathlib.Path(sys.argv[2]).resolve() / ('cli-fixtures-' + uuid.uuid4().hex)
    root.mkdir(parents=True)
    calibration = '''camera_matrix: [500, 0, 320, 0, 500, 240, 0, 0, 1]
imu_delay_ms: -2.5
distort_coeffs: [0, 0, 0, 0, 0]
R_camera2gimbal: [1, 0, 0, 0, 1, 0, 0, 0, 1]
t_camera2gimbal: [0, 0, 0]
R_gimbal2imubody: [1, 0, 0, 0, 1, 0, 0, 0, 1]
time_sync_board: {columns: 9, rows: 6, square_size_m: 0.02}
'''
    def fingerprint(data):
        value = 14695981039346656037
        for byte in data:
            value = ((value ^ byte) * 1099511628211) & ((1 << 64)-1)
        return f'fnv1a64:{value:016x}'
    
    def fixture(name, **changes):
        path = root / name
        path.mkdir()
        (path/'calibration.yaml').write_text(calibration)
        meta = {'schema_version': '1', 'session_id': name, 'clock': 'steady_clock',
                'timestamp_semantics': 'host_receive_v1', 'complete': 'true',
                'calibration_fingerprint': fingerprint(calibration.encode()), 'source_head': 'unknown'}
        meta.update(changes)
        counts = {'images_written':0, 'imu_written':2, 'events_written':0,
                  'images_accepted':0, 'imu_accepted':2,
                  'images_dropped':0, 'images_failed':0, 'imu_dropped':0, 'imu_failed':0,
                  'imu_invalid':0, 'events_dropped':0, 'camera_source_dropped':0}
        (path/'session.yaml').write_text(''.join(f'{k}: {v}\n' for k,v in meta.items()) +
            'counters:\n' + ''.join(f'  {k}: {v}\n' for k,v in counts.items()))
        (path/'images.csv').write_text('frame_id,t_image_ns,image_path,camera_frame_id,device_timestamp_ticks,exposure_us\n')
        (path/'imu.csv').write_text('sample_id,t_imu_receive_ns,qw,qx,qy,qz,valid\n0,0,1,0,0,0,1\n1,10000000,1,0,0,0,1\n')
        (path/'events.csv').write_text('t_event_ns,event_type,local_id,reason\n')
        return path
    
    def reject(path, needle):
        output = root / (path.name + '-output')
        proc = subprocess.run([str(binary), '--session', str(path), '--output', str(output)],
                              text=True, capture_output=True)
        assert proc.returncode != 0, (path,proc.stdout,proc.stderr)
        summary = (output/'summary.yaml').read_text()
        assert 'invalid_data' in summary and needle in summary, (path,summary)
    
    reject(fixture('empty'), 'insufficient')
    assert 'baseline_delay_ms: -2.5' in (root/'empty-output'/'summary.yaml').read_text(), 'baseline must come from capture config'
    path = fixture('truncated-imu')
    (path/'imu.csv').write_text('sample_id,t_imu_receive_ns,qw,qx,qy,qz,valid\n0,0,1,0,0,0,1\n')
    reject(path, 'record count')
    path = fixture('recorded-loss')
    meta = path/'session.yaml'
    meta.write_text(meta.read_text().replace('imu_dropped: 0', 'imu_dropped: 1'))
    reject(path, 'loss counter')
    reject(fixture('incomplete', complete='false'), 'incomplete')
    reject(fixture('wrong-clock', clock='system_clock'), 'clock')
    reject(fixture('fingerprint', calibration_fingerprint='bad'), 'fingerprint')
    path = fixture('duplicate-imu')
    with (path/'imu.csv').open('a') as f: f.write('2,10000000,1,0,0,0,1\n')
    reject(path, 'monotonic')
    path = fixture('nan-imu')
    (path/'imu.csv').write_text('sample_id,t_imu_receive_ns,qw,qx,qy,qz,valid\n0,0,nan,0,0,0,1\n')
    reject(path, 'finite')
    path = fixture('unsafe-image')
    with (path/'images.csv').open('a') as f: f.write('0,10000000,../outside.png,,,\n')
    reject(path, 'image_path')
    path = fixture('header')
    (path/'images.csv').write_text('wrong_header\n')
    reject(path, 'header')
    output = root/'existing-output'
    output.mkdir()
    (output/'preserve.txt').write_text('must remain')
    proc = subprocess.run([str(binary),'--session',str(root/'empty'),'--output',str(output)],capture_output=True)
    assert proc.returncode != 0 and (output/'preserve.txt').read_text() == 'must remain'
    print('time_offset_cli_test passed; fixtures:', root)

"""Render a stationary board and exercise the real PNG -> PnP -> delay CLI.

Fixtures are retained under the explicitly supplied test-output directory.
Requires Python OpenCV/numpy/PyYAML only for this integration test.
"""
import csv
import math
import pathlib
import subprocess
import sys
import uuid

import cv2
import numpy as np
import yaml


def fingerprint(data):
    result = 14695981039346656037
    for byte in data:
        result = ((result ^ byte) * 1099511628211) & ((1 << 64) - 1)
    return f"fnv1a64:{result:016x}"


def quaternion(t, phase):
    # Composition of rotations about different axes, with nonconstant speed.
    a = 0.075 * math.sin(2.1 * t + phase) + 0.018 * math.sin(4.7 * t)
    b = 0.065 * math.sin(1.6 * t + 0.3 + phase)
    c = 0.025 * math.sin(2.9 * t + phase)
    cx, sx = math.cos(a / 2), math.sin(a / 2)
    cy, sy = math.cos(b / 2), math.sin(b / 2)
    cz, sz = math.cos(c / 2), math.sin(c / 2)
    return np.array([cz * cy * cx + sz * sy * sx, cz * cy * sx - sz * sy * cx,
                     cz * sy * cx + sz * cy * sx, sz * cy * cx - cz * sy * sx])


def rotation(q):
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def sample_at(t, phase):
    left = math.floor(t * 100) / 100
    a, b = quaternion(left, phase), quaternion(left + .01, phase)
    fraction = (t - left) / .01
    angle = math.acos(np.clip(np.dot(a, b), -1, 1))
    if angle < 1e-9:
        return a
    return (math.sin((1-fraction)*angle)*a + math.sin(fraction*angle)*b) / math.sin(angle)


def make_session(path, delay_ms, phase):
    path.mkdir()
    (path / "images").mkdir()
    config = dict(camera_matrix=[720, 0, 320, 0, 720, 240, 0, 0, 1],
                  distort_coeffs=[0, 0, 0, 0, 0],
                  R_camera2gimbal=[1, 0, 0, 0, 1, 0, 0, 0, 1],
                  t_camera2gimbal=[0, 0, 0],
                  R_gimbal2imubody=[1, 0, 0, 0, 1, 0, 0, 0, 1],
                  time_sync_board=dict(columns=9, rows=6, square_size_m=.04))
    content = yaml.safe_dump(config).encode()
    (path / "calibration.yaml").write_bytes(content)
    meta = dict(schema_version=1, session_id=path.name, clock="steady_clock",
                timestamp_semantics="host_receive_v1", complete=True,
                calibration_fingerprint=fingerprint(content), source_head="synthetic-test")
    meta["counters"] = dict(images_written=120, imu_written=651, events_written=0,
                            images_accepted=120, imu_accepted=651, images_dropped=0,
                            images_failed=0, imu_dropped=0, imu_failed=0, imu_invalid=0,
                            events_dropped=0, camera_source_dropped=0)
    (path / "session.yaml").write_text(yaml.safe_dump(meta))
    with (path / "imu.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["sample_id", "t_imu_receive_ns", "qw", "qx", "qy", "qz", "valid"])
        for i in range(651):
            writer.writerow([i, i * 10000000, *quaternion(i / 100, phase), 1])
    (path / "events.csv").write_text("t_event_ns,event_type,local_id,reason\n")
    K = np.array(config["camera_matrix"], dtype=float).reshape(3, 3)
    scale = 3
    K[:2] *= scale
    origin = np.array([-.16, -.10, 1.8])
    with (path / "images.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["frame_id", "t_image_ns", "image_path", "camera_frame_id",
                         "device_timestamp_ticks", "exposure_us"])
        for i in range(120):
            time = .2 + i * .05
            R = rotation(sample_at(time-delay_ms/1000, phase)).T
            rvec, _ = cv2.Rodrigues(R)
            canvas = np.full((480*scale, 640*scale), 180, dtype=np.uint8)
            for y in range(-1, 6):
                for x in range(-1, 9):
                    corners = origin + .04 * np.array([[x,y,0], [x+1,y,0],
                                                       [x+1,y+1,0], [x,y+1,0]])
                    pixels, _ = cv2.projectPoints(corners, rvec, np.zeros(3), K, np.zeros(5))
                    cv2.fillConvexPoly(canvas, np.rint(pixels[:, 0]).astype(np.int32),
                                       0 if (x+y) % 2 == 0 else 255)
            image = cv2.resize(canvas, (640, 480), interpolation=cv2.INTER_AREA)
            relative = f"images/{i:06}.png"
            if not cv2.imwrite(str(path / relative), image):
                raise RuntimeError("fixture image write failed")
            writer.writerow([i, round(time*1e9), relative, "", "", 1000])


def main():
    binary = pathlib.Path(sys.argv[1]).resolve()
    root = pathlib.Path(sys.argv[2]).resolve() / ("rendered-fixtures-" + uuid.uuid4().hex)
    root.mkdir(parents=True)
    train, validation = root / "train", root / "validation"
    truth_ms = 6.4
    make_session(train, truth_ms, 0)
    make_session(validation, truth_ms, .8)
    output = root / "result"
    proc = subprocess.run([str(binary), "--session", str(train), "--output", str(output),
                           "--validation", str(validation), "--min-frames", "60",
                           "--min-duration-s", "2", "--min-delay-ms", "-12",
                           "--max-delay-ms", "12", "--step-ms", "0.2"],
                          text=True, capture_output=True, timeout=180)
    print(proc.stdout)
    print(proc.stderr)
    assert proc.returncode == 0, (proc.returncode, root)
    summary = yaml.safe_load((output / "summary.yaml").read_text())
    # Output schema checks intentionally follow the CLI contract, not private logic.
    assert summary["status"] == "candidate", summary
    assert abs(summary["candidate_imu_delay_ms"] - truth_ms) < 1.2, summary
    assert summary["offline_validation_passed"] is True, summary
    assert summary["reference_point"] == "chessboard_center", summary
    assert summary["deployable"] is False, summary
    assert "validated" != summary["status"], "Offline rendering cannot validate hardware timing"
    print("rendered time-sync end-to-end passed; retained fixtures:", root)


if __name__ == "__main__":
    test_cli()
    main()
