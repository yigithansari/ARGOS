import json
from pathlib import Path
import subprocess
import sys
import time

root = Path(__file__).parent
host = sys.argv[1] if len(sys.argv)>1 else '192.168.137.234'
targets=tuple(map(int,sys.argv[2].split(','))) if len(sys.argv)>2 else (25,35,45)
# Operator accepted periodic variation provisionally; use the measured settling window.
duration=sys.argv[3] if len(sys.argv)>3 else '20'
out = root / ('pi_suite_' + time.strftime('%Y%m%d_%H%M%S'))
out.mkdir()
print(str(out), flush=True)
results = []
for target in targets:
    for repetition in range(1, 4):
        run = out / f'{target}rpm_run{repetition}'
        process = subprocess.run([sys.executable, str(root/'pi_raised_check.py'), str(target), str(run), host,duration],
                                 capture_output=True, text=True)
        (out / f'{target}rpm_run{repetition}.txt').write_text(process.stdout + process.stderr, encoding='utf-8')
        if process.returncode:
            print(process.stdout + process.stderr, flush=True)
            raise SystemExit('Suite halted on test/feedback failure')
        result = json.loads((run/'result.json').read_text())
        result['repetition'] = repetition
        results.append(result)
        (out/'results.json').write_text(json.dumps(results, indent=2))
        print(json.dumps(result), flush=True)
        if not result['mean_within_5_percent']:
            raise SystemExit('Suite halted: mean tracking outside +/-5%; tuning review required')
        time.sleep(2)
print('All configured forward speed runs completed; inspect variation before acceptance.', flush=True)
