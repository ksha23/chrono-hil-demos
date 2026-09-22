# chrono-hil-demos

Demo code for the PyChrono interactive / human-in-the-loop tutorial.

## Setup

```bash
conda env create -f environment.yml
conda activate chrono-hil
```

## Extra dependencies

Demo 3 needs two pip packages that the base Chrono install does not include
(both are in `environment.yml`):

```bash
pip install torch    # Go2 locomotion policy (demo 3a)
pip install ikpy     # Franka inverse kinematics (demo 3b), brings scipy
```

Without `torch`, demo 3a falls back to a stance PD and says so.

### Locomotion policy

A trained Go2 checkpoint is vendored at `go2_assets/go2_policy.pt`
(from [wty-yy/go2_rl_gym](https://github.com/wty-yy/go2_rl_gym), MIT).
To run your own:

```bash
GO2_POLICY_CKPT=path/to/yours.pt python demos/manipulate/main.py go2
```

## Demos

Each demo folder has its own README with controls and what to watch.

```bash
# Before anything else: the shape of a Chrono program
python demos/first_steps/hello_chrono.py

# Demo 1 -- does the clock actually matter?
python demos/driver/tutorial_HIL_driver.py REALTIME=False
python demos/driver/tutorial_HIL_driver.py REALTIME=True

# Demo 2 -- a person driving, off road
python demos/driver/tutorial_HIL_driver.py INPUT_SOURCE=keyboard KEYBOARD_MODE=held REALTIME=True

# Demo 3a -- disturbing a robot that is holding itself up
python demos/manipulate/main.py go2

# Demo 3b -- commanding a robot by its hand
python demos/manipulate/main.py arm
```

## License

BSD 3-Clause (see [LICENSE](LICENSE)). Vendored robot assets carry their own
licenses listed in [ASSETS.md](ASSETS.md).
