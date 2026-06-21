# Lab 12 Rescue Part 3 Steps

Use the provided baseline mission from `moos-ivp-greece` for the simulations:

```bash
cd /home/lefteris/Marine_Autonomy/moos-ivp-greece/missions/rescue_athens
ktm; ./clean.sh; ./launch.sh -r1 --swim_file=athens_02.txt 10
```

## Step 1: Dropped Swimmers

Right-click near a swimmer in `pMarineViewer` to generate:

```text
FOUND_SWIMMER = id=<id>, finder=nature
```

Expected: `pGenRescue` removes that swimmer from the remaining tour and posts a fresh `SURVEY_UPDATE`.
If the last swimmer is removed, it posts a one-point path at the current vehicle position, which lets the helm leave the survey behavior and return.

## Step 2: Two-Vehicle Baseline

```bash
cd /home/lefteris/Marine_Autonomy/moos-ivp-greece/missions/rescue_athens
ktm; ./clean.sh; ./launch.sh -r2 -rsl 10
```

Expected: both vehicles run `pGenRescue`, receive all `FOUND_SWIMMER` updates, and re-plan as swimmers are rescued by either vehicle.

## Step 3: Planner Options

`pGenRescue` now supports these optional configuration lines in the `ProcessConfig = pGenRescue` block:

```text
planner = greedy
planner = two_hop
concede_adversary = true
concede_count = 1
concede_path_count = 1
concede_radius = 15
concede_heading_window = 35
```

`planner = two_hop` chooses each next swimmer by considering the current leg plus the nearest following leg.
`concede_adversary = true` listens to `NODE_REPORT` and temporarily removes swimmers that look easier for the opponent.
`concede_path_count = 1` estimates the opponent's greedy path and concedes the first swimmer on that path.

To compare strategies in the two-vehicle mission, configure `abe` and `ben` differently:

```text
ProcessConfig = pGenRescue
{
  AppTick   = 4
  CommsTick = 4
  vname     = $(VNAME)

#ifdef VNAME abe
  planner = two_hop
  concede_adversary = true
  concede_count = 0
  concede_path_count = 1
  concede_heading_window = 35
#elseifdef VNAME ben
  planner = greedy
  concede_adversary = false
#endif
}
```

This configuration is now installed in:

```text
/home/lefteris/Marine_Autonomy/moos-ivp-greece/missions/rescue_athens/meta_vehicle.moos
```

Run the optimized comparison with:

```bash
cd /home/lefteris/Marine_Autonomy/moos-ivp-greece/missions/rescue_athens
ktm; ./clean.sh; ./launch.sh -r2 -rsl 10
```

In this setup, `abe` is the experimental vehicle and `ben` is the greedy baseline.

## Step 4: AppCast Debugging

Open the `pGenRescue` AppCast for each vehicle while the two-vehicle mission is running.

For `abe`, confirm:

```text
Planner:             two_hop
Concede adversary:   true
Concede path N:      1
Contact position:    <x>,<y>
Conceded swimmers:   <count> [<ids>]
```

For `ben`, confirm:

```text
Planner:             greedy
Concede adversary:   false
```

The useful tell is whether `abe` has a non-empty `Conceded swimmers` list after receiving `NODE_REPORT` from `ben`, and whether its `SURVEY_UPDATE` path differs from the greedy baseline.
