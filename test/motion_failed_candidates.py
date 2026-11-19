"""Known failed baselines must be refused before touching the real device."""
import sys,tempfile
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import capture_motion,run_motion_repetitions
from motion_profile import verify_live_candidate
for rpm in (20,40,60):
    for shape in (0,1):verify_live_candidate(rpm,shape)
with tempfile.TemporaryDirectory() as temporary:
    for shape in (0,1):
        for direction in (-1,1):
            output=Path(temporary)/f'{shape}-{direction}.log'
            argv=['capture_motion.py','--rpm','10','--shape',str(shape),'--direction',str(direction),'--output',str(output)]
            with patch.object(sys,'argv',argv),patch.object(capture_motion.list_ports,'comports') as ports,patch.object(capture_motion.serial,'Serial') as serial:
                try:capture_motion.main()
                except RuntimeError as exc:assert 'Known protective-stop condition' in str(exc)
                else:raise AssertionError('Failed baseline drive was accepted')
                ports.assert_not_called();serial.assert_not_called();assert not output.exists()
        argv=['run_motion_repetitions.py','--version','165' if shape==0 else '166','--rpm','10','--shape',str(shape),'--output-prefix','must-not-run']
        with patch.object(sys,'argv',argv),patch.object(run_motion_repetitions.subprocess,'run') as process:
            try:run_motion_repetitions.main()
            except RuntimeError as exc:assert 'Known protective-stop condition' in str(exc)
            else:raise AssertionError('Failed baseline reset was accepted')
            process.assert_not_called()
print('PASS: both failed 10rpm baselines and both directions rejected before serial enumeration, output creation or reset; other measured speed stages retained')
