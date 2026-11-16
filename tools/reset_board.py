"""One explicit ST-LINK reset only; no drive command, retry or upload."""
import argparse
from pathlib import Path
import re
import subprocess

MARKER='CODEX_RESET_RUN_DONE'


def verify_reset_result(returncode,stdout,stderr):
    output=stdout+'\n'+stderr
    if returncode!=0 or re.search(r'^\s*Error:',output,re.MULTILINE) or MARKER not in output:
        raise RuntimeError('OpenOCD did not confirm reset run; no drive command may follow')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    package=Path.home()/'.platformio/packages/tool-openocd'
    command=[str(package/'bin/openocd.exe'),'-s',str(package/'openocd/scripts'),
             '-f','interface/stlink.cfg','-c','transport select swd',
             '-f','target/stm32g4x.cfg','-c',f'init; reset run; echo {MARKER}; shutdown']
    # Reserve the log before touching the debugger, preserving earlier attempts.
    with args.output.open('x',encoding='utf-8') as log:
        result=subprocess.run(command,capture_output=True,text=True,timeout=15)
        log.write(result.stdout+'\n'+result.stderr)
    verify_reset_result(result.returncode,result.stdout,result.stderr)
    print('OpenOCD reset run confirmed; no drive commands sent')


if __name__=='__main__':main()
