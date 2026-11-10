#!/usr/bin/env python3
"""Run actual Runtime/MODULE fault suite and audit its output, without sockets."""
import argparse
import json
from pathlib import Path
import subprocess
from verify_fake import negative_controls,parse,verify


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('driver','parameters','missions','control'):p.add_argument('--'+name,required=True)
    p.add_argument('--output',type=Path);a=p.parse_args()
    command=[a.driver,a.parameters,a.missions,a.control]
    run=subprocess.run(command,capture_output=True,text=True,timeout=20)
    if a.output:
        a.output.mkdir(parents=True,exist_ok=False)
        (a.output/'trace.jsonl').write_text(run.stdout);(a.output/'stderr.log').write_text(run.stderr)
        (a.output/'command.json').write_text(json.dumps({'argv':command,'exit':run.returncode},indent=2)+'\n')
    if run.returncode:raise RuntimeError('actual Runtime suite failed: '+run.stderr)
    rows=[parse(line) for line in run.stdout.splitlines() if line]
    report=verify(rows);report['negative_controls_rejected']=negative_controls(rows)
    if a.output:(a.output/'audit.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
