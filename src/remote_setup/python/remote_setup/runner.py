"""One disposable connection-test process; no persistent network session."""
import json
import subprocess
import sys

COMMAND = [sys.executable, '-m', 'remote_setup.probes']
LIMITATION = 'Signing/internal API keys and command routing are not verified; robot commands are disabled.'


def result_text(output, code):
    try:
        payload = json.loads(output)
        results = payload['results']
        ok = code == 0 and payload['ok'] is True and bool(results) and all(
            r['ok'] is True for r in results)
        message = '\n'.join(f"{'PASS' if r['ok'] else 'FAIL'} — {r['check']}: {r['message']}"
                            for r in results)
        return ok, message + '\n' + LIMITATION
    except (ValueError, KeyError, TypeError):
        return False, 'Connection test failed or was interrupted; no usable result.'


def run(settings):
    try:
        result = subprocess.run(COMMAND, input=json.dumps(settings.values), text=True,
                                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                timeout=settings.deadline, check=False)
        return result_text(result.stdout, result.returncode)
    except subprocess.TimeoutExpired:
        return False, 'Connection test exceeded its deadline and was stopped.'
    except OSError:
        return False, 'Could not start the connection test process.'
