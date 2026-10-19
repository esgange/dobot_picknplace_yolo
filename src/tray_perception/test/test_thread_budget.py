"""Real pinned runtime initialization and drift rejection in independent processes."""
import os
from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_prefix
import pytest


def exercise(runtime, threads, scratch):
    import io
    import json
    import torch
    from item_perception_yolo.preview_protocol import send_packet, receive_packet
    from item_perception_yolo.yolo_worker_native import serve
    expected = 8 if threads == 0 else threads
    options = {} if threads == 0 else {'torch_threads': threads}
    inputs, outputs = io.BytesIO(), io.BytesIO()
    for _ in range(2):
        send_packet(inputs, {'operation': 'drift'})
    inputs.seek(0)

    def drift(*_args):
        torch.set_num_threads(expected + 1)
        return {'state': 'ok'}, b''
    with pytest.raises(RuntimeError, match='runtime drift'):
        serve(inputs, outputs, runtime, json.loads((runtime/'runtime-lock.json').read_text()),
              scratch, operations={'drift': drift}, **options)
    outputs.seek(0)
    ready, _ = receive_packet(outputs)
    assert ready['torch_threads'] == expected
    assert ready['torch_interop_threads'] == ready['opencv_threads'] == 1
    assert ready['opencl'] is False
    assert receive_packet(outputs)[0]['state'] == 'ok'
    assert 'runtime drift' in receive_packet(outputs)[0]['error']


@pytest.mark.parametrize('threads', [0, 4, 8], ids=['item-default', 'explicit-4', 'tray-8'])
def test_independent_runtime_thread_limits_and_drift(tmp_path, threads):
    runtime = Path(get_package_prefix('item_perception_yolo'))/'lib/item_perception_yolo/yolo_runtime'
    command = ('import sys,runpy; from pathlib import Path; sys.path.insert(0,sys.argv[1]); '
               'runpy.run_path(sys.argv[2])["exercise"](Path(sys.argv[1]),int(sys.argv[3]),Path(sys.argv[4]))')
    completed = subprocess.run(['/usr/bin/python3','-c', command,str(runtime),__file__,str(threads),str(tmp_path)],
        env={**os.environ,'OPENBLAS_NUM_THREADS':'1','YOLO_CONFIG_DIR':str(tmp_path),
             'MPLCONFIGDIR':str(tmp_path)}, capture_output=True,text=True,timeout=30)
    assert completed.returncode == 0, completed.stdout + completed.stderr
