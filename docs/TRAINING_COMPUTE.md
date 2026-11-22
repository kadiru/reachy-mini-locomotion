# What it costs to train Reachy Mini at home

Reachy's walking, turning and balancing policies use CPU MuJoCo rollouts and numerical parameter search or local feedback-model fitting. The three metered refits ran on an Apple M4 Mac mini with 10 CPU cores (4 performance and 6 efficiency), 16 GB memory, and the project-local Python environment. Each selected policy was compared to the published policy before pairing its energy with the existing behavior tests.

## Metered results

Electricity is priced at $0.300/kWh (illustrative flat rate, household tariff unconfirmed). Gross cost uses the Mac system sensor. Boundary readings are interpolated at workload start and finish.

| Fit from existing project inputs | Rollouts | Workers | Elapsed | CPU core-min | Mean / peak W | Gross Wh | Above background Wh | Gross USD | Simulation tests passed |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Continuous spin | 910 | 8 | 11.25 min | 81.6 | 46.1 / 50.5 | 8.641 | 5.450 | $0.00259 | 8/8 |
| Active-waist edge sweep | 58 | 8 | 5.17 min | 26.0 | 41.6 / 50.7 | 3.584 | 2.060 | $0.00108 | 1/8 |
| Walking heading repair | 21 | 4 | 1.07 min | 3.5 | 39.3 / 44.5 | 0.699 | 0.375 | $0.00021 | 10/10 |

The three refits used 12.92 Wh in total. At the stated rate that costs $0.00388. The waist policy reused the previously fitted edge regulator, and the walking policy retuned steering on the existing gait. The spin run reproduced the primary 910-rollout search through selection of the final policy. The rim-spin variant is a separate fit.

The separate spin evaluation ran eight matched trials, eight held-out trials and three controls. It took 1.27 min, averaged 46.9 W, and used 0.989 Wh ($0.00030). The replay compared the complete per-trial metrics with the published results. [Evaluation receipt](../experiments/reports/compute-meter/spin-evaluation.json)

The spin refit wrote the selected policy and completed every training stage. Its initial JSON comparison exited with a mismatch because the new file explicitly stored two zero-valued lean offsets. Expanding the defaults in the archived file gave identical controller parameters. The separately metered evaluation then reproduced all 19 sets of trial metrics. The raw return code is retained in the receipt.

[Spin receipt](../experiments/reports/compute-meter/spin.json) · [Waist receipt](../experiments/reports/compute-meter/waist.json) · [Walking receipt](../experiments/reports/compute-meter/walk.json)

## Gyro-peg policies

The gyro-peg comparison has three entries. The rim-roll and wide-circle entries share the initial periodic-orbit and tilt-feedback searches, so summing those rows would count that energy twice. The deep-roll entry measures the additional training on top of the previously identified side and fore/aft balance regulators. [Gyro-peg outcomes and electricity](GYRO_PEG.md)

## Earlier models

| Behavior | Indexed fit rollouts | Test suite | Passed |
| --- | ---: | --- | ---: |
| Slick-floor quarter-turn | 1490 | 10 final held-out turns | 7/10 |
| Efficient edge pivot | 2700 | 95 mixed-condition turns | 92/95 |
| Side-edge balance | 1729 | 14 regular side-balance checks | 5/14 |
| Front/back balance | — | 18 raised-start checks | 9/18 |
| Side-hit recovery | 20 | 12 held-out impacts for the selected limit | 5/12 |
| Overhead disturbance | — | 40 matched disturbance trials scored for holding the edge | 18/40 |

The earlier rollout counts cover the search stages named in the CSV. The overhead-ball study reused an existing policy for evaluation. Earlier rollouts vary in duration, physics rate and termination criteria, so multiplying their counts by a single energy-per-rollout value would give unreliable costs.

## How the meter works

The meter samples macmon system watts at roughly 1 Hz. It integrates the samples over the exact training interval using elapsed time and trapezoidal integration, including simulation initialization, failed candidates, worker startup and selection. The raw receipts retain timestamps, watts, baseline, duration, child-process CPU time and the selected policy hash. CPU time is summed across the parent training process and the workers it reaps. Wall time is the duration experienced by the user. [macmon sensor documentation](https://github.com/vladkens/macmon)

Gross energy includes other Mac activity during each refit. The background-adjusted figure subtracts the time-weighted power during the 15 seconds just before that run. Background load can vary, so that figure estimates the additional demand attributable to training. The Mac system sensor is an estimate of host energy. A plug-in meter measuring the computer's AC feed would give a closer estimate of the electricity billed by the utility. The display, Ubuntu machine, robot battery, cloud services and hardware purchase price are outside the Mac measurement.

One fully occupied core for a minute contributes one CPU-minute. Concurrent workers can accumulate several CPU-minutes in one minute of elapsed time. The reported peak is the highest sampled wattage. Subsecond spikes can be higher.

The training interval ends at policy selection. The model-specific validation used to select finalists is included. Final held-out evaluation, rendering videos, software development, and earlier experiments that supplied a warm start have separate costs. Earlier reports provide rollout counts and validation outcomes. Their original sessions have no recorded power trace or attributable energy bill. The new runs provide a reproducible cost for rebuilding the selected policies under the stated conditions.

## Home GPUs

A read-only inventory of kfuUbuntu found GPU 0, an NVIDIA GeForce RTX 4090 with a configured 480 W board limit, and GPU 1, an NVIDIA GeForce RTX 4070 with a 200 W limit. Both had existing GPU compute processes during this check. The Linux sensors exposed temperature and CPU power domains. Whole-workstation AC power would require a separate meter. The MuJoCo refits used the M4 CPU for dynamics and search and consumed zero training GPU-hours. [GPU inventory](../experiments/reports/compute-meter/gpu-inventory.json)

At the example rate of $0.300/kWh, one hour averaging 480 W on the 4090 board would cost $0.144, or $0.060 for one hour averaging 200 W on the 4070 board. These scenarios assume the board averages its configured limit for the whole hour. Actual training time and wall watts would have to be measured on the CUDA workload.

For a future CUDA trainer, the meter pins the workload to the chosen physical UUID and samples that card's board watts. It records the card name, UUID and number of compute processes present at the start and end. The workload's own backend log should confirm CUDA execution. Other jobs sharing that GPU can contribute to its reading. For a full workstation bill, measure the AC feed over the same interval. That already includes the GPU, CPU, power-supply losses and other host components. [NVIDIA power-field definitions](https://docs.nvidia.com/deploy/nvidia-smi/)

## Reproduce the measurements

From the project root, use the project environment, pinned Reachy assets, and a macmon executable. Use fresh output paths and run commands sequentially for comparable readings. Each invocation below records a separate 15-second background interval.

```sh
for model in waist spin; do
  .venv/bin/python experiments/meter_compute.py --model "$model" --phase training \
    --macmon macmon --output "experiments/generated/my-meter/$model.json" -- \
    .venv/bin/python experiments/replay_compute_training.py "$model" "experiments/generated/my-meter/$model-fit" --workers 8
done
.venv/bin/python experiments/meter_compute.py --model walk --phase training \
  --macmon macmon --output experiments/generated/my-meter/walk.json -- \
  .venv/bin/python experiments/repair_cadence_heading.py --fit-only --workers 4 \
  --out experiments/generated/my-meter/walk-fit.json
```

For a CUDA workload on Linux, substitute the GPU UUID reported by `nvidia-smi -L` and the trainer's normal command. The `gross.kwh` field then describes just that GPU board. The meter uses the price supplied through `--rate-usd-kwh` when present.

```sh
python experiments/meter_compute.py --model MODEL --phase training --gpu-uuid GPU-UUID \
  --output experiments/generated/my-meter/gpu.json -- python TRAINER.py
```

The [comparison CSV](../experiments/reports/training-compute.csv) has one row per behavior and evaluation suite, including numerator and denominator, the scope of the training count, hardware and measurement scope, power, kWh and price assumptions. Blank energy fields identify missing measurements. Success on different control tasks measures different behaviors. Keep the same test cohort when plotting a policy's improvement against training energy.

To measure the 16 spin trials and three controls separately, run this command after the spin refit. It compares the resulting metrics with the archived matched, held-out and control measurements.

```sh
.venv/bin/python experiments/meter_compute.py --model spin --phase evaluation \
  --macmon macmon --output experiments/generated/my-meter/spin-evaluation.json -- \
  .venv/bin/python experiments/replay_compute_training.py spin experiments/generated/my-meter/spin-fit --qualify-spin --workers 8
```

To verify fresh fitted-policy identities and retain their sample series, archive the three receipts together. The supplied run also records the per-machine meter and simulator versions.

```sh
.venv/bin/python experiments/archive_compute_refit.py experiments/generated/my-meter \
  --receipts experiments/generated/my-receipts --macmon macmon
```

To reprice the published refits, substitute the marginal rate from the electricity bill for the example number below. Use the training-period delivery plus generation rate for each kWh. If the marginal rate changes during training, price each time segment separately using the timestamped samples.

```sh
.venv/bin/python experiments/compute_ledger.py --rate-usd-kwh 0.30 \
  --rate-basis 'example flat tariff' --output experiments/generated/repriced-compute.csv
```

**Electricity cost in dollars = measured kWh × dollars per kWh.** A sustained 100 W load over one hour uses 0.10 kWh. The configured power limit is useful for budgeting a GPU-hour, while the observed average board watts and elapsed time determine an actual board-energy charge.
