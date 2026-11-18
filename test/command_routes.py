"""Reject ambiguous serial routes before hardware upload."""
from collections import Counter
from pathlib import Path
import re
source=(Path(__file__).resolve().parents[1]/'src/main.cpp').read_text(encoding='utf-8')
routes=Counter(re.findall(r"command\s*==\s*'(.)'",source))
assert routes and not {key:n for key,n in routes.items() if n!=1},'Ambiguous serial route'
for key in '#-.,/&*':assert routes[key]==1
print('PASS: every command has a single firmware route, including all standalone bench profiles')
