# SPDX-License-Identifier: BSD-3-Clause

"""Exercise the real standalone entry point with delayed context shutdown."""

import signal
import subprocess
import sys
import tempfile
import time


def check_shutdown(executable, stop_signal):
    with tempfile.TemporaryFile(mode='w+') as output:
        child = subprocess.Popen(
            [executable], stdout=output, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline:
                output.seek(0)
                log = output.read()
                if 'shutdown probe ready' in log:
                    break
                if child.poll() is not None:
                    raise AssertionError(f'probe exited during startup:\n{log}')
                time.sleep(0.02)
            else:
                raise AssertionError(f'probe did not become ready:\n{log}')

            child.send_signal(stop_signal)
            try:
                code = child.wait(timeout=5)
            except subprocess.TimeoutExpired as error:
                raise AssertionError('shutdown did not finish within 5s') from error
            output.seek(0)
            log = output.read()
            assert code == 0, f'exit code {code}:\n{log}'
            assert 'node destroyed after shutdown finished' in log, log
        finally:
            if child.poll() is None:
                child.kill()
            child.wait()


if __name__ == '__main__':
    for _ in range(5):
        check_shutdown(sys.argv[1], getattr(signal, sys.argv[2]))
    print(f'{sys.argv[2]}: 5 clean, correctly ordered shutdowns')
