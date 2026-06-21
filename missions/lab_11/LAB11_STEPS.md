# Lab 11 Messaging Steps

Run each mission from its own directory with a time warp, for example:

```bash
cd /home/lefteris/Marine_Autonomy/moos-ivp-lefteris3544/missions/lab_11/charlie_dana_message
./launch.sh 15
```

Use `--nogui --xlaunched` for headless testing.

## Assignment 2: NodeComms

```bash
cd charlie_dana_nodecomms
./launch.sh 15
```

Expected: `uFldNodeComms` runs on shoreside and shows comms pulses when the vehicles are within range.

## Assignment 3: Manual Message

```bash
cd charlie_dana_message
./launch.sh 15
```

Deploy, then send a loiter update from Charlie to Dana:

```bash
uPokeDB targ_shoreside.moos DEPLOY_ALL=true MOOS_MANUAL_OVERRIDE_ALL=false
uPokeDB targ_charlie.moos NODE_MESSAGE_LOCAL="src_node=charlie,dest_node=dana,var_name=UP_LOITER,string_val=ycenter_assign=-100"
```

If the vehicles are outside comms range, wait until they are closer and poke again.

## Assignment 4: Automatic Reassign

```bash
cd charlie_dana_reassign
./launch.sh 15
```

Expected: each vehicle periodically sends `NODE_MESSAGE_LOCAL` to reassign the other vehicle's `UP_LOITER`.

## Assignment 5: Recover

```bash
cd charlie_dana_recover
./launch.sh 15
```

Expected: vehicles still send random loiter reassign messages during the 60-90 second window. These normal loiter changes happen through `NODE_MESSAGE_LOCAL`, so they require vehicle-to-vehicle communication.

Each vehicle also runs a communication watchdog. Every valid incoming `NODE_MESSAGE` resets the watchdog. If a vehicle receives no message for 300 seconds, it posts:

```text
UP_LOITER = ycenter_assign=<random value between -70 and -50>
```

This local fallback pulls the vehicle back toward a shared center band to restore communications. Under normal communication, the watchdog keeps getting reset and this fallback does not change the patrol area.
