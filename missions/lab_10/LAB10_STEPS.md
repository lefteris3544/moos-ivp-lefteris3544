# Lab 10 Rescue Part 2 Steps

## 1. Laptop / Shoreside

From the mission directory:

```bash
cd <YOUR_REPO>/missions/lab_10
./launch_shoreside.sh --swim_file=athens_03.txt --ip=<YOUR_SHORESIDE_IP>
```

Use `-3` instead of `--swim_file=athens_03.txt` if you prefer the short form.

To find `<YOUR_SHORESIDE_IP>` on the laptop, use the WiFi address on the field network:

```bash
hostname -I
```

## 2. PABLO / BlueBoat

On the robot:

```bash
cd <YOUR_REPO>/missions/lab_10
PGEN_RESCUE=<YOUR_REPO>/bin/pGenRescue
./launch_vehicle.sh --shore=<YOUR_SHORESIDE_IP> --pgr=$PGEN_RESCUE
```

If you run the vehicle from the course `rescue_athens` mission instead, keep the same `--pgr=`
argument and point it to the built `pGenRescue` binary in your repo.

## 3. Assignment 1 Check

Confirm:

- The vehicle connects to shoreside in `pMarineViewer`.
- There are no run warnings in the PABLO terminal.
- `DEPLOY` sends the vehicle through the generated swimmer path.
- The vehicle returns when the rescue manager finishes.

## 4. Assignment 2 Check

While the vehicle is moving, click a new swimmer location in `pMarineViewer`.

Confirm:

- A new `SWIMMER_ALERT` is generated.
- `pGenRescue` posts a fresh `SURVEY_UPDATE`.
- The displayed path changes to include the new swimmer.
