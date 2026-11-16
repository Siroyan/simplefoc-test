"""OpenOCD can report transport errors and still exit zero after shutdown."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from reset_board import verify_reset_result,MARKER
verify_reset_result(0,MARKER,'shutdown command invoked')
for code,out,err in [(0,'','Error: unable to select a session transport. Can\'t continue.\nshutdown command invoked'),
                     (0,MARKER,'Error: reset failed'),(1,MARKER,''),(0,'shutdown command invoked','')]:
    try:verify_reset_result(code,out,err)
    except RuntimeError:pass
    else:raise AssertionError('Unconfirmed debugger reset accepted')
print('PASS: zero-exit transport/reset errors and missing completion marker are rejected')
