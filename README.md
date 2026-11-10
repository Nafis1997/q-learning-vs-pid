# Q-Learning Controller vs PID Control

This experiment compares tabular Q-learning with a tuned PID controller for setpoint
tracking and disturbance rejection. Both controllers act on the same simulated
second-order system and are evaluated under matched conditions.

## Plant and simulation

The plant is defined by:

```text
x_ddot = -a*x_dot - b*x + u + d
```

| Parameter | Setting |
| --- | --- |
| Nominal coefficients | a = 1, b = 1 |
| Target position | 1 |
| Sampling interval | 0.05 s |
| Episode duration | 10 s |
| Actuator limits | -3 to +3 |
| Evaluation disturbance | d = 0.4 from t = 5 s |
| Numerical integration | Semi-implicit Euler |

Position and velocity are fully observed. This is a simulation study of the stated
plant; hardware behavior and stability outside the tested conditions are unverified.

## Run the experiment

Use Python 3.11 or 3.12. From this project folder:

```bash
python -m pip install -r requirements.txt
python main.py --episodes 1000
python make_plots.py
```

[START_HERE.md](START_HERE.md) covers the VS Code setup.
`requirements-tested.txt` records the versions used for the included run.
Save a copy of `results/` before changing the experiment, since a new run overwrites it.

## Controllers

### PID

The control law uses proportional error, integral error and derivative on measurement:

```text
u_raw = Kp*error + Ki*integral(error) - Kd*velocity
u = clip(u_raw, -3, 3)
```

Conditional integration limits windup during saturation. A grid of 27 gain
combinations is evaluated on validation scenarios with seeds 10–19, minimizing
integrated absolute error (IAE). The selected gains are **Kp = 6, Ki = 2, Kd = 2**.
The finite search defines this baseline; it does not establish optimal PID tuning.

### Q-learning

| Component | Definition |
| --- | --- |
| State | Discretized tracking error and velocity |
| State bins | 33 per variable |
| Actions | 9 equally spaced values from -3 to +3 |
| Q-table shape | 33 × 33 × 9 |
| Learning rate | 0.12 |
| Discount factor | 0.98 |
| Exploration | Epsilon decreases from 0.8 to a floor of 0.03 |
| Training budget | 1,000 episodes per seed |
| Training seeds | 0, 1, 2 |

The reward penalizes error, input magnitude and velocity:

```text
reward = -(tracking_error² + 0.03*u² + 0.05*velocity²)
```

Training randomizes initial conditions and plant coefficients: a is sampled from
[0.4, 1.2] and b from [0.6, 1.4]. After five seconds, training disturbances are
sampled at each step from [-0.5, 0.5]. Evaluation instead uses a sustained disturbance.

## Comparison protocol

Test seeds 100–119 define 20 matched scenarios, evaluated with nominal and varied
plant parameters. Each Q-learning policy and PID receive the same initial conditions,
plant coefficients, actuator bounds and evaluation disturbance. PID additionally
uses integral memory; the RL state contains only error and velocity.

Metrics include IAE, RMSE, overshoot, final error, integrated squared control input
and settling time. The settling band is 2% of the unit target, measured across the
observed response including disturbance recovery. Unsettled trials have blank
settling times. Read the settling success rate alongside conditional average times.

PID rows are repeated in the raw table to pair with each policy seed. These duplicate
PID runs are removed for aggregate summaries. Standard deviations summarize the
observations; they are not confidence intervals across independent policy replications.

## Results

| Scenario | PID mean IAE | Q-learning mean IAE |
| --- | ---: | ---: |
| Nominal plant | 0.8732 | 6.9823 |
| Varied coefficients | 0.9410 | 5.6605 |

PID tracks the target more accurately in this experiment. The Q-learning baseline
has greater variability and larger residual errors. [RESULTS.md](RESULTS.md) reports
additional metrics and discusses the comparison.

## Limitations and follow-up experiments

The RL representation has coarse state and action bins, lacks integral memory and
omits time from the state despite finite-horizon training. Training disturbances
also differ from the evaluation disturbance. These are plausible contributors to
the observed performance; their individual effects have not been isolated.

Follow-up work can add integral error to the RL state, vary training budget and
reward weights, test unseen setpoints, and compare additional PID tuning methods.
Use development scenarios to select changes and retain a separate final evaluation.
More policy seeds would support a stronger assessment of training variability.

## Project files

| File | Purpose |
| --- | --- |
| `main.py` | Simulate the plant, train Q-learning, tune PID and evaluate |
| `make_plots.py` | Plot saved learning curves and representative responses |
| `requirements.txt` | Dependency ranges |
| `requirements-tested.txt` | Versions used for the included run |
| `results/config.json` | Plant, controller and training settings |
| `results/all_trials.csv` | Individual evaluation trials |
| `results/summary.csv` | Aggregate means and standard deviations |
| `results/settling_success_rate.csv` | Fraction of trials that settle |
| `results/training_returns.csv` | Episode returns for each training seed |
| `results/q_table_seed_*.npy` | Three trained Q tables |
| `results/step-response.png` | Representative output response |
| `results/learning-curves.png` | Smoothed returns for all policy seeds |
| `results/response-error-control.png` | Representative output, error and input |

The response plots use policy seed 0 and test seed 100. Aggregate tables cover all
specified test scenarios.

