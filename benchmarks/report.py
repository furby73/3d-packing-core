#!/usr/bin/env python3
"""Generate a standalone HTML report and static charts from saved benchmark runs."""
import argparse
import csv
from collections import Counter
import html
import json
from pathlib import Path
import statistics as st

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from verify import geometry

ROOT=Path(__file__).resolve().parent
COLORS={'anneal':'#3472b7','constructive':'#cf6d36'}
NAMES={'anneal':'Annealing','constructive':'Constructive'}


def load(folder):
    return [json.loads(line) for line in (folder/'runs.jsonl').read_text().splitlines()]


def summary(rows):
    pairs={}
    for row in rows:
        pairs.setdefault((row['case'],row['seed']),{})[row['algorithm']]=row
    wins=Counter();complete_wins=Counter();mismatches=0
    for pair in pairs.values():
        if set(pair)!={'anneal','constructive'}:continue
        a,b=pair['anneal'],pair['constructive']
        if 'bin_count' not in a or 'bin_count' not in b:continue
        winner='tie' if a['bin_count']==b['bin_count'] else ('anneal' if a['bin_count']<b['bin_count'] else 'constructive')
        wins[winner]+=1
        mismatches+=a['baseline_bin_count']!=b['baseline_bin_count']
        if not a.get('baseline_timed_out',True) and not b.get('baseline_timed_out',True):complete_wins[winner]+=1
    return dict(runs=len(rows),valid=sum(r.get('verification',{}).get('valid',False) for r in rows),
        pairs=len(pairs),wins=dict(wins),completed_baseline_wins=dict(complete_wins),
        baseline_count_mismatches=mismatches,baseline_timeouts=sum(r.get('baseline_timed_out',0) for r in rows),
        fallback=sum(r.get('used_fallback',0) for r in rows),
        surface_pairs=sum(r.get('verification',{}).get('checked_pairs',0) for r in rows),
        closed_pairs=sum(r.get('verification',{}).get('closed_pairs',0) for r in rows),
        open_pairs=sum(r.get('verification',{}).get('open_pairs',0) for r in rows),
        improved_over_baseline=sum(r.get('bin_count',0)<r.get('baseline_bin_count',0) for r in rows))


def table(rows):
    result=['<table><tr><th>Case</th><th>Algorithm</th><th>Bins median [min–max]</th><th>Wall s median</th><th>Search s median</th><th>Peak MiB max</th><th>Improved / runs</th></tr>']
    for name in dict.fromkeys(c['name'] for c in json.loads((ROOT/'cases.json').read_text())):
        for alg in COLORS:
            group=[r for r in rows if r['case']==name and r['algorithm']==alg and 'bin_count' in r]
            if not group:continue
            values=[r['bin_count'] for r in group]
            improved=sum(r['bin_count']<r['baseline_bin_count'] for r in group)
            result.append(f'<tr><td>{name}</td><td>{NAMES[alg]}</td><td>{st.median(values):g} [{min(values)}–{max(values)}]</td>'
                f'<td>{st.median(r["wall_seconds"] for r in group):.3f}</td><td>{st.median(r["search_seconds"] for r in group):.3f}</td>'
                f'<td>{max(r["peak_rss_kib"] for r in group)/1024:.1f}</td><td>{improved}/{len(group)}</td></tr>')
    result.append('</table>');return ''.join(result)


def chart(rows,out):
    cases=[c['name'] for c in json.loads((ROOT/'cases.json').read_text()) if any(r['case']==c['name'] for r in rows)]
    fig,axes=plt.subplots(1,2,figsize=(12,5.6),layout='constrained')
    for index,alg in enumerate(COLORS):
        for i,c in enumerate(cases):
            group=[r for r in rows if r['case']==c and r['algorithm']==alg and 'bin_count' in r]
            for ax,key in zip(axes,['bin_count','wall_seconds']):
                values=[r[key] for r in group]
                y=np.full(len(values),i+(-.16 if index==0 else .16))+np.linspace(-.04,.04,len(values))
                ax.scatter(values,y,color=COLORS[alg],s=25,alpha=.55,label=NAMES[alg] if i==0 else None)
                ax.plot([st.median(values)]*2,[i+(-.26 if index==0 else .06),i+(-.06 if index==0 else .26)],c=COLORS[alg],lw=3)
    for ax,title,xlabel in zip(axes,['Cuboids used (lower is better)','End-to-end runtime'],['Cuboids','Seconds']):
        ax.set_yticks(range(len(cases)),cases);ax.invert_yaxis();ax.set_title(title);ax.set_xlabel(xlabel);ax.grid(axis='x',alpha=.2)
    axes[0].legend(loc='upper left');axes[0].set_xticks(range(1,1+max(r.get('bin_count',0) for r in rows)))
    fig.suptitle('5-second search budget · 10 seeds · identical meshes and constraints',fontsize=13)
    fig.savefig(out/'comparison.png',dpi=180);plt.close(fig)


def layouts(rows,folder,out,case='bunny8_medium',filename='layouts.png'):
    # A transparent example selection rule: seed zero, regardless of who wins.
    selected=[r for r in rows if r['case']==case and r['seed']==0 and 'bin_count' in r]
    if len(selected)!=2:return
    display_bins=max(r['bin_count'] for r in selected)
    fig=plt.figure(figsize=(12,5),layout='constrained')
    palette=plt.get_cmap('tab10')
    for panel,alg in enumerate(COLORS):
        row=next(r for r in selected if r['algorithm']==alg)
        data=json.loads((folder/row['manifest']).read_text());dims=np.array([data['width'],data['length'],data['height']]);stride=dims[0]*1.13
        ax=fig.add_subplot(1,2,panel+1,projection='3d')
        for p in data['placements']:
            mesh,_,_=geometry(str(Path(p['input']).resolve()))
            matrix=np.array(p['matrix']).reshape(3,4)
            vertices=np.asarray(mesh.vertices)@matrix[:,:3].T+matrix[:,3]+[p['bin']*stride,0,0]
            ax.add_collection3d(Poly3DCollection(vertices[mesh.faces],facecolors=palette(p['input_index']%10),alpha=.95,shade=True,lightsource=matplotlib.colors.LightSource(azdeg=110,altdeg=45)))
        for b in range(data['bin_count']):
            corners=np.array([[x,y,z] for x in [0,dims[0]] for y in [0,dims[1]] for z in [0,dims[2]]])+[b*stride,0,0]
            for i in range(8):
                for j in range(i):
                    if bin(i^j).count('1')==1:ax.plot(*corners[[i,j]].T,color='#384552',lw=.6,alpha=.6)
        ax.set_xlim(0,(display_bins-1)*stride+dims[0]);ax.set_ylim(0,dims[1]);ax.set_zlim(0,dims[2]);ax.set_box_aspect([display_bins*dims[0],dims[1],dims[2]],zoom=.82)
        ax.view_init(elev=25,azim=-65);ax.set_axis_off();ax.set_title(f'{NAMES[alg]} · {row["bin_count"]} cuboids · {row["wall_seconds"]:.2f}s')
    fig.suptitle(f'{case} · seed 0 · identical colors identify identical inputs');fig.savefig(out/filename,dpi=180);plt.close(fig)


def main():
    p=argparse.ArgumentParser();p.add_argument('main',type=Path);p.add_argument('--long',type=Path);p.add_argument('--output',type=Path,default=ROOT/'report');args=p.parse_args()
    out=args.output;out.mkdir(parents=True,exist_ok=True);rows=load(args.main);stats=summary(rows)
    (out/'summary.json').write_text(json.dumps(stats,indent=2)+'\n')
    with (out/'runs.csv').open('w',newline='') as f:
        fields=['case','algorithm','seed','budget_seconds','bin_count','baseline_bin_count','baseline_timed_out','wall_seconds','search_seconds','preparation_seconds','baseline_seconds','peak_rss_kib','iterations','restarts','used_fallback']
        writer=csv.DictWriter(f,fieldnames=fields+['valid','minimum_surface_distance','checked_pairs'],lineterminator='\n')
        writer.writeheader()
        for r in rows:
            record={k:r.get(k) for k in fields}
            record.update({k:r.get('verification',{}).get(k) for k in ['valid','minimum_surface_distance','checked_pairs']})
            writer.writerow(record)
    chart(rows,out);layouts(rows,args.main,out)
    env=json.loads((args.main/'environment.json').read_text());models=json.loads((ROOT/'models.json').read_text())
    model_table='<table><tr><th>Model</th><th>Triangles</th><th>Closed manifold</th><th>Boundary / nonmanifold edges</th></tr>'+''.join(f'<tr><td>{m["name"]}</td><td>{m["triangles"]:,}</td><td>{m["watertight"]}</td><td>{m["boundary_edges"]} / {m["nonmanifold_edges"]}</td></tr>' for m in models)+'</table>'
    winner_text=f'Across {stats["pairs"]} matched case/seed pairs: annealing used fewer cuboids {stats["wins"].get("anneal",0)} times, constructive {stats["wins"].get("constructive",0)} times, and {stats["wins"].get("tie",0)} tied.'
    extra=''
    long_lead=''
    if args.long:
        long_rows=load(args.long);long_stats=summary(long_rows)
        layouts(long_rows,args.long,out,case='mixed10',filename='layouts-long.png')
        long_lead=f'<p class="lead">At 20 seconds on three selected cases: constructive won {long_stats["wins"].get("constructive",0)} matched pairs, annealing won {long_stats["wins"].get("anneal",0)}, and {long_stats["wins"].get("tie",0)} tied. The mixed ten-model set fit in one cuboid with constructive search on all three seeds, versus two with annealing.</p>'
        extra='<h2>Longer search: 20 seconds, seeds 0–2</h2>'+table(long_rows)+f'<p>Independent checks passed for {long_stats["valid"]}/{long_stats["runs"]} outputs; {long_stats["surface_pairs"]} same-bin surface pairs were checked.</p>'
        extra+='<p>At the larger budget, the complete greedy initializer gives the same feasible fallback to both methods. These are only three selected cases, so this is a budget-sensitivity check, not a second broad ranking.</p><img src="layouts-long.png" alt="Twenty-second budget mixed-model seed-zero comparison">'
        (out/'long-summary.json').write_text(json.dumps(long_stats,indent=2)+'\n')
    document='''<!doctype html><html lang="en"><meta charset="utf-8"><title>3D packing benchmark</title><style>
body{max-width:1120px;margin:40px auto;padding:0 24px;font:16px/1.55 system-ui;color:#192b3a;background:#f8fafc}h1,h2{line-height:1.2}h2{margin-top:34px}table{border-collapse:collapse;width:100%;background:white;font-size:14px}td,th{padding:8px 10px;border-bottom:1px solid #d9e2ea;text-align:left}th{background:#e9f0f5}img{width:100%;background:white;border:1px solid #dde4e9}pre{background:#e9f0f5;padding:16px;overflow:auto;font-size:13px}.lead{font-size:20px;color:#224c70}.note{padding:14px 18px;border-left:4px solid #cf6d36;background:#fff3e9}a{color:#245f9e}code{font-size:14px}</style>
<h1>3D packing: annealing vs constructive backtracking</h1>'''
    document+=f'<p class="lead">{winner_text}</p>{long_lead}<p>Measured {html.escape(env["timestamp"])}. Eight fixed cases, ten seeds per method, five-second search budgets. Lower cuboid counts are better. No global optimality claim.</p>'
    document+='<img src="comparison.png" alt="Measured cuboid counts and end-to-end runtimes"><h2>Per-case results</h2>'+table(rows)
    document+=f'<p><strong>Independent verification: {stats["valid"]}/{stats["runs"]} outputs passed.</strong> {stats["surface_pairs"]} same-bin pair checks; {stats["closed_pairs"]} pairs with both meshes closed. Bounds, rigid transforms, collisions and clearance were checked. {stats["open_pairs"]} pairs include open/nonmanifold models; solid containment is not certified for those surfaces.</p>'
    document+='<h2>What the alternative does</h2><p>Constructive backtracking varies insertion order and candidate orientations, places only nonconflicting models at wall/face coordinates or nearby perturbed positions, and undoes up to three placements at a dead end. After 32 backtracks it restarts. It uses no temperature schedule and never accepts an overlapping prefix. Both methods use the same collision kernel, gap, orientation preparation and ascending cuboid-count policy.</p>'
    document+=f'<div class="note"><strong>Baseline caveat.</strong> The shared greedy initializer is limited to 20% of the search budget and runs separately in each invocation. {stats["baseline_timeouts"]}/{stats["runs"]} initializers hit that limit; {stats["baseline_count_mismatches"]}/{stats["pairs"]} pairs started with different fallback counts. Among pairs where both initializers completed, outcomes were {html.escape(str(stats["completed_baseline_wins"]))}. The full result compares the shipped pipelines; interrupted-baseline pairs do not isolate search strategy perfectly.</div>'
    document+='<h2>Example placements</h2><p>Seed zero is shown by a fixed selection rule, not selected as a best run. Container outlines show exact bounds; display spacing is not part of the packed coordinates.</p><img src="layouts.png" alt="Packed bunny layouts from both algorithms">'
    model_sizes={m['name']:m['triangles'] for m in models}
    definitions='<table><tr><th>Case</th><th>Parts</th><th>Width × length × height</th><th>Gap</th><th>Total triangles</th></tr>'
    for c in env['cases']:
        size=' × '.join(str(v) for v in c['size'])
        definitions+=f'<tr><td>{c["name"]}</td><td>{len(c["models"])}</td><td>{size}</td><td>{c["gap"]}</td><td>{sum(model_sizes[m] for m in c["models"]):,}</td></tr>'
    definitions+='</table>'
    document+='<h2>Case definitions</h2><p>The mixed set contains two each of the medium bunny, dragon, Buddha, drill and Spot. Container dimensions and gap use normalized model units; each model has longest dimension one.</p>'+definitions
    document+=extra+'<h2>Downloaded inputs</h2>'+model_table+'<p>Sources: <a href="https://graphics.stanford.edu/data/3Dscanrep/">Stanford Computer Graphics Laboratory</a> and <a href="https://www.cs.cmu.edu/~kmcrane/Projects/ModelRepository/">Keenan Crane</a>. Stanford research/free-redistribution terms; Spot CC0. Longest dimension normalized to one offline, original topology retained. Gaps are 0.02 except mixed10_gap at 0.08. No additional wall gap.</p>'
    document+=f'<h2>Timing and reproducibility</h2><p>{html.escape(env["cpu_info"])}; Linux; logical CPU {env["pinned_cpu"]}; sequential runs, balanced order, input warm-up. Search budgets include greedy initialization but exclude loading, orientation preparation, final validation and output. End-to-end times include them. Independent verification is outside the timed run. CPU frequency and desktop activity were not fixed. The seed controls randomness, while wall-clock stopping introduces variability. Exact binaries are archived; the longer sweep and final package add a defensive empty-candidate check after the main sweep.</p>'
    document+='<p>Build: GCC 15.2, C++17 Release, -O3. Detailed protocols and reproduction commands: <a href="../README.md">benchmark README</a>. Raw runs: <a href="runs.csv">CSV</a>; <a href="../results/main-5s/runs.jsonl">JSONL</a>; <a href="../results/main-5s/environment.json">environment, hashes and cases</a>; <a href="../models.json">model provenance</a>. Each raw run has its exact command, log, placement manifest and independently checked minimum surface distance.</p>'
    document+='<p>Expert design review: GLM suggested bounded backtracking, Kimi emphasized a shared baseline and geometry, and DeepSeek highlighted orientation and interlocking risks. Their review used the supplied design, not repository access. Recommendations for skipping counts or adding wall clearance were not adopted because they conflict with the requested problem.</p></html>'
    (out/'index.html').write_text(document);print(json.dumps(stats,indent=2))

if __name__=='__main__':main()
