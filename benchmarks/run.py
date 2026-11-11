#!/usr/bin/env python3
"""Sequential, equal-budget benchmark. Every measured output is checked independently."""
import argparse
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import random
import subprocess
import time

from verify import verify

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parent


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--binary',type=Path,default=REPO/'build/packer')
    p.add_argument('--cases',type=Path,default=ROOT/'cases.json')
    p.add_argument('--only',nargs='*')
    p.add_argument('--seeds',type=int,nargs='+',default=list(range(10)))
    p.add_argument('--seconds',type=float,default=5)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--cpu',type=int,default=2)
    args=p.parse_args()
    os.sched_setaffinity(0,{args.cpu})
    out=args.output.resolve(); out.mkdir(parents=True,exist_ok=True)
    if (out/'runs.jsonl').exists(): raise RuntimeError('Choose a new results directory; preserving previous runs')
    binary=args.binary.resolve()
    cases=json.loads(args.cases.read_text())
    if args.only: cases=[c for c in cases if c['name'] in args.only]
    metadata=dict(timestamp=time.strftime('%Y-%m-%dT%H:%M:%S%z'),binary=str(binary),
        binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
        git_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip(),
        platform=platform.platform(), python=platform.python_version(),
        cpu_info=next(x for x in Path('/proc/cpuinfo').read_text().splitlines() if x.startswith('model name')),
        pinned_cpu=args.cpu,seconds=args.seconds,seeds=args.seeds,cases=cases,
        packages={x:importlib.metadata.version(x) for x in ['numpy','trimesh','python-fcl','rtree','plyfile']},
        source_sha256={str(f.relative_to(REPO)):hashlib.sha256(f.read_bytes()).hexdigest()
                       for folder in ('src','benchmarks') for f in (REPO/folder).glob('*') if f.is_file()})
    (out/'environment.json').write_text(json.dumps(metadata,indent=2)+'\n')
    subprocess.run(['git','diff','--binary'],cwd=REPO,stdout=(out/'working-tree.patch').open('w'),check=True)
    # Page in each input and the executable, without recording these warm-up runs.
    for model in sorted({x for c in cases for x in c['models']}):
        subprocess.run([str(binary),'-w','2','-l','2','-H','2','-t','.01',
            '-m',str(out/'warmup.txt'),str(ROOT/'data'/f'{model}.obj')],
            stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,check=True,timeout=30)
    trials=[(seed,c) for seed in args.seeds for c in cases]
    random.Random(20261111).shuffle(trials)
    for index,(seed,c) in enumerate(trials):
        for algorithm in (['anneal','constructive'] if (index+seed)%2==0 else ['constructive','anneal']):
            run_dir=out/f'{c["name"]}-{algorithm}-seed{seed}';run_dir.mkdir()
            manifest=run_dir/'packing.json'
            x,y,z=c['size']
            command=[str(binary),'--algorithm',algorithm,'--seed',str(seed),'-w',str(x),'-l',str(y),'-H',str(z),
                '--gap',str(c['gap']),'-t',str(args.seconds),'-m',str(run_dir/'transforms.txt'),
                '--manifest',str(manifest),*[str(ROOT/'data'/f'{model}.obj') for model in c['models']]]
            row=dict(case=c['name'],algorithm=algorithm,seed=seed,budget_seconds=args.seconds,
                command=command,manifest=str(manifest.relative_to(out)),model_count=len(c['models']))
            start=time.perf_counter()
            try:
                with (run_dir/'solver.log').open('w') as log:
                    run=subprocess.run(['/usr/bin/time','-f','%M','-o',str(run_dir/'peak_rss_kib.txt'),*command],
                        stdout=log,stderr=subprocess.STDOUT,timeout=args.seconds+90)
                row.update(returncode=run.returncode,wall_seconds=time.perf_counter()-start)
                if run.returncode==0:
                    data=json.loads(manifest.read_text())
                    for k in ('bin_count','baseline_bin_count','baseline_timed_out','baseline_seconds','attempted_bin_counts','used_fallback','iterations','restarts','search_seconds','preparation_seconds'):
                        row[k]=data[k]
                    start=time.perf_counter();row['verification']=verify(manifest)
                    row['verification_seconds']=time.perf_counter()-start
                row['peak_rss_kib']=int((run_dir/'peak_rss_kib.txt').read_text().splitlines()[-1])
            except Exception as error:
                row['error']=repr(error)
            with (out/'runs.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
            print(c['name'],algorithm,seed,'bins',row.get('bin_count'),'baseline',row.get('baseline_bin_count'),
                  'wall',round(row.get('wall_seconds',0),3),'valid',row.get('verification',{}).get('valid'),row.get('error',''),flush=True)
    (out/'complete').write_text('done\n')

if __name__=='__main__':main()
