import math,sys,tempfile
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from motion_profile import parameters,reference
from analyze_motion import analyze_motion
for rpm in (10,20,40,60):
    for shape in (0,1):
        move,ret,duration,limit=parameters(rpm,shape)
        for sign in (-1,1):
            q,v,p=reference(100000+move/2,sign,rpm,shape)
            if shape==0:
                assert abs(q-sign*math.radians(1.875))<1e-12
                assert abs(v*8-sign*rpm*math.pi/30)<1e-10
            for t in (0,99999,100000+move,ret,ret+move,duration):
                q,v,p=reference(t,sign,rpm,shape)
                assert -1e-12<=q*sign<=math.radians(3.75)+1e-12
                if p in (0,2,4):assert v==0
        assert duration<3000000 and 5<=limit<8.284
path=Path(__file__).resolve().parents[1]/'docs/experiment-logs/2026-11-19-motion-smooth10-positive-v143-01.log'
r=analyze_motion(path)
assert not r['pass'] and not r['whole_trace'] and r['moving'][1]['FOC_samples']==0
text=path.read_text(encoding='utf-8')
with tempfile.TemporaryDirectory() as directory:
    p=Path(directory)/'corrupt.log'
    for bad in (text.replace('HOST final_stop_acknowledged=True','HOST final_stop_acknowledged=False'),
                text.replace('TRACE_END','MISSING'),text.replace('max_vector_V=0.203415','max_vector_V=0.600000')):
        p.write_text(bad,encoding='utf-8')
        try:analyze_motion(p)
        except ValueError:pass
        else:raise AssertionError('Incomplete/unbounded motion evidence accepted')
print('PASS: trajectory stages/signs, partial protective stop retained, missing stop/trace and invalid all-loop voltage rejected')
