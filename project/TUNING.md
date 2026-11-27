# Tuning run sheet

How to run a tuning session on the Udacity VM workspace and keep the evidence.
The VM is wiped at the end of every session, so **push before you stop**.

## Before the first session

- Copy the recommended gain ranges from the classroom page ("try between those
  values") into the table in section 5. The suggested values there are a
  conservative starting sequence, not the classroom's.
- The first build on the VM is the first time `main.cpp` is compiled with the
  controllers wired in. Expect to fix small compile errors in session 1.

## 1. Start of every session

Terminal 1, start the simulator and leave it running:

```bash
/opt/carla-simulator/CarlaUE4.sh
```

Terminal 2, get the code and build it:

```bash
git clone -b feat/pid-class https://github.com/SebAustin/nd013-c6-control-starter.git
cd nd013-c6-control-starter/project
./install-ubuntu.sh
pip3 install pandas matplotlib
cd pid_controller && cmake . && make && cd ..
```

Optional, run the unit tests (about a minute):

```bash
cd pid_controller && cmake -S tests -B build-tests && cmake --build build-tests && ctest --test-dir build-tests --output-on-failure; cd ..
```

## 2. One tuning run

All commands from `project/`.

1. Edit `pid_gains.txt`. Change **one** number compared with the previous run.
   No rebuild is needed; the program reads the file when it starts.
2. Restart the simulator in terminal 1 (Ctrl+C, then start it again). This
   removes the car left by the previous run.
3. Start the run:

   ```bash
   ./run_main_pid.sh
   ```

   The first lines printed are the gains in use. Check they match the file.
4. Watch the car. Stop with Ctrl+C once it has passed the obstacles, crashed,
   or clearly gone wrong. Then stop the controller, which keeps running in the
   background:

   ```bash
   pkill pid_controller
   ```

5. Archive the run. `run_main_pid.sh` erases the data files at the next start:

   ```bash
   ./archive_run.sh
   ```

   This creates `runs/steer_<Kp>_<Ki>_<Kd>__throttle_<Kp>_<Ki>_<Kd>/` with the
   data, the gains and two PNG plots. To repeat a run with the same gains, add
   a note: `./archive_run.sh repeat2`.
6. Write one line in the log in section 6.

If the desktop view freezes while the terminal keeps printing, close the
desktop and reopen it. If you see `bind failed. Error: Address already in use`,
run `pkill pid_controller` and start again.

## 3. End of every session

```bash
git add project/runs project/pid_gains.txt project/TUNING.md
git commit -m "test: add tuning runs from session N"
git push
```

Then press **End lab** so the VM stops using your allowance.

## 4. Run 0: the stationary car (brief, step 1)

The brief asks for a screenshot of the car standing still.

1. Set every gain to zero in `pid_gains.txt`:

   ```text
   steer    0 0 0
   throttle 0 0 0
   ```

2. Do one run as in section 2. The car must not move.
3. Take a screenshot of the simulator window and save it as
   `project/runs/stationary_car.png`.
4. Archive with `./archive_run.sh stationary`.

## 5. Run plan

Tune the speed controller first, then the heading controller. Within each:
proportional gain, then derivative, then integral. Keep the best value found
so far for every gain you are not sweeping.

The steer gains stay at their starting values during the throttle sweeps,
because the car has to steer around the obstacles for a run to last.

| # | Sweep | Values to try | Keep fixed | Look for |
|---|-------|---------------|------------|----------|
| 1 | throttle Kp | 0.1, 0.2, 0.4 | steer 0.3 0 0 | Speed error falling quickly without the command chattering between throttle and brake |
| 2 | throttle Kd | 0.05, 0.1 | best Kp | Less overshoot of the desired speed |
| 3 | throttle Ki | 0.01, 0.05 | best Kp, Kd | Speed error settling at zero instead of a constant offset |
| 4 | steer Kp | 0.15, 0.3, 0.6 | best throttle | Car turning towards the trajectory without weaving |
| 5 | steer Kd | 0.1, 0.3 | best steer Kp | Weaving damped out |
| 6 | steer Ki | 0.001, 0.01 | best steer Kp, Kd | No steady drift to one side |
| 7 | final | best of everything | | Acceptance bar below; archive with `./archive_run.sh final` and record the video |

About 16 runs. At a few minutes each this fits in one or two sessions.

Note on the derivative and integral terms: the starter measures time in whole
seconds, so most updates see a time step of zero. On those updates the
integral does not change and the derivative keeps its last value. Expect both
terms to act in coarse one-second steps.

**Acceptance bar for the final gains**, in the default scenario:

- the run completes without a collision;
- the steer command shows no growing oscillation;
- the speed error settles near zero.

A perfect trajectory is not required.

## 6. Run log

| Run folder | What changed | Collision? | What the plots show |
|------------|--------------|------------|---------------------|
| | | | |

## 7. Comparison plots for the report

After a sweep, overlay its runs. Example for the throttle Kp sweep:

```bash
python3 plot_compare.py --controller throttle --out runs/compare_throttle_kp.png \
    runs/steer_0.3_0.0_0.0__throttle_0.1_0.0_0.0 \
    runs/steer_0.3_0.0_0.0__throttle_0.2_0.0_0.0 \
    runs/steer_0.3_0.0_0.0__throttle_0.4_0.0_0.0
```

Use `--controller steer` for the steer sweeps. The folder names come from the
numbers exactly as typed in `pid_gains.txt`.

## 8. After tuning

1. Copy the final gains into the defaults in `pid_controller/main.cpp`, so the
   submission works without `pid_gains.txt`, and rebuild.
2. Record a video of one run with the final gains.
3. Write the report in `README.md` using the plots in `runs/`.
