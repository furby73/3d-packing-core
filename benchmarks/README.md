# Packing benchmark

This benchmark compares the complete `anneal` and `constructive` pipelines under
identical search budgets, container dimensions, mesh files, clearances and seeds.
The alternative is randomized constructive insertion with bounded backtracking;
it does not use simulated annealing. Both try container counts 1, 2, 3, … and
export only complete feasible layouts. Failed attempts are timeouts, not
infeasibility proofs. Neither method certifies the minimum bin count.

## Models and attribution

The bunny (three original resolutions), dragon, Happy Buddha and drill come from
the [Stanford University Computer Graphics Laboratory's scan repository](https://graphics.stanford.edu/data/3Dscanrep/).
Stanford permits research use and free redistribution with attribution; commercial
use requires their permission. Spot comes from
[Keenan Crane's model repository](https://www.cs.cmu.edu/~kmcrane/Projects/ModelRepository/)
and is dedicated under CC0. Downloaded assets are local benchmark data, excluded
from Git and the Blender package.

`download_models.py` checks archive SHA-256 values, reads explicitly selected
members without extracting an archive into the filesystem, removes unused
vertices, translates the minimum corner to zero, and uniformly scales each model's
longest bounding-box dimension to 1. It saves 17-digit OBJ coordinates. There is no
hole filling, mesh decimation, inflation, convexification or repair. Each original
level of detail is normalized separately. A gap of 0.02 therefore means 2% of the
normalized longest dimension, not 0.02 mm. No clearance from container walls is
required. `models.json` records URLs, archive/OBJ hashes, bounds, scales and topology.

The selected Stanford reconstructions include open and nonmanifold surfaces;
**do not interpret their results as certified packing of watertight printed solids**.
Spot is closed and consistently wound. Original-resolution bunny has 69,451
triangles; unused source vertices are not part of the packed geometry.

## Reproduce

Build the Release executable as described in the project README. On Linux:

```sh
python3 -m venv build/bench-venv
build/bench-venv/bin/pip install -r benchmarks/requirements.txt
build/bench-venv/bin/python benchmarks/download_models.py
build/bench-venv/bin/python benchmarks/test_verify.py
build/bench-venv/bin/python benchmarks/run.py --seconds 5 \
  --output benchmarks/results/main-5s
```

Default seeds are 0–9. `--only` selects named cases, `--seeds` selects seeds and
`--cpu` changes the pinned logical CPU (default 2). `cases.json` fixes the suite
before the main run. Each output directory must be new to protect prior evidence.
`run.py` records the executable and source hashes, dirty working-tree patch,
compiler-independent environment metadata, exact commands, logs, manifests,
peak resident memory, timings and verifier results. Models and source code must
remain unchanged during a run. The report generator may be edited independently.

Runs execute sequentially on one pinned logical CPU. There is one input/executable
warm-up per model, followed by a fixed shuffled case/seed order and balanced
algorithm order. Machine frequency and other desktop activity are not controlled;
these are workstation measurements, not universal throughput estimates.

The budget includes greedy initialization and search, but excludes loading/BVH
construction, orientation preparation, final validation and output. End-to-end
wall time includes those costs and process startup. An in-flight geometry query
may overrun the budget. Independent validation is timed separately. Timing-based
termination means the same seed is not a bit-for-bit reproducibility guarantee.

Both algorithms use the same deterministic greedy initialization code and a 20%
time allowance. Each invocation computes it afresh; a timeout can yield different
fallbacks. We record baseline count, timeout and duration, flag paired differences,
and separately summarize pairs whose baselines both completed. This is a comparison
of the shipped pipelines; it does not perfectly isolate search strategy on pairs
with interrupted baselines. Iteration counts are diagnostic only: an annealing
proposal and a constructive insertion check are different units of work.

## Independent validity

`verify.py` reads exported matrices and checks finite rigid transforms, determinant,
input/bin coverage, transformed-vertex bounds, and every same-bin pair with
Python-FCL's triangle-mesh BVH collision/distance queries. Tolerance is `1e-8` times
the largest container dimension. All benchmark gaps are positive, so surface
collision is a failure; contact classification at zero gap is outside this suite.
It also checks one representative per connected component for containment in every
closed mesh using trimesh ray casting. FCL surface distance alone cannot detect a
fully nested solid. Open/nonmanifold inputs are explicitly counted as pairs whose
solid containment is not certified. This is independent numerical validation, not
an exact-arithmetic proof.

`test_verify.py` challenges the oracle with separated, under-gap, intersecting,
fully nested, and out-of-bounds boxes before it is used on benchmark outputs.

## Longer-budget check and report

The dense coarse-bunny and two mixed cases are also measured at 20 seconds with
seeds 0–2. These cases were chosen from the pilot for difficulty before examining
the completed main sweep, not to select a winning algorithm.

```sh
build/bench-venv/bin/python benchmarks/run.py --seconds 20 --seeds 0 1 2 \
  --only bunny6_coarse mixed10 mixed10_gap --output benchmarks/results/long-20s
build/bench-venv/bin/python benchmarks/report.py benchmarks/results/main-5s \
  --long benchmarks/results/long-20s
```

The generated [report](report/index.html) includes per-case medians and ranges,
paired wins/ties, timing and memory, validity limits, a plot of every run and
seed-zero example layouts. Its CSV is a convenient flat export; raw JSONL retains
full commands and verifier diagnostics. The local results directory also preserves
the benchmarked executable and matching C++ source snapshot.

The main sweep's exact binary and C++ sources are archived with its results.
A defensive check for empty candidate-coordinate lists was added afterwards;
the longer sweep and installable package include that check. Each sweep records
its own executable/source hashes, and both methods use the same binary within it.

## Measured result

On the i7-13700H workstation, the five-second sweep contained 160 runs (eight
cases, seeds 0–9, both algorithms). Across 80 matched pairs, annealing used fewer
cuboids 17 times, constructive once, and 62 tied. At twenty seconds on three
selected cases and seeds 0–2, constructive won three pairs and six tied: the mixed
ten-model set used one cuboid for constructive versus two for annealing on every
seed. The constructive runs finished in 3.57–4.52 seconds, versus 20.15 seconds for
annealing, including preparation and output. This is not a universal ranking.

All 178 measured outputs passed independent checks, covering 3,128 same-bin mesh
pairs. Open/nonmanifold surface limitations above still apply. Release and
ASan/UBSan test suites, the Blender 5.1 integration smoke test, and extension ZIP
validation also passed. Both algorithms remain available in the CLI and Blender.
