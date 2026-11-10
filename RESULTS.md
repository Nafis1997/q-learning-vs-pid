# Executed results — Q-learning controller versus PID

Both controllers regulate the same second-order simulated plant with identical
initial conditions, actuator limits and disturbance in matched test scenarios.
PID is tuned over 27 gain combinations on validation seeds 10..19. Chosen gains:
Kp=6, Ki=2, Kd=2. Q-learning uses 1000 episodes for each of seeds 0,1,2.
Evaluation uses test seeds 100..119 with nominal and varied plant parameters.

## Measured aggregate results

Scenario | Controller | Mean IAE | Mean RMSE | Mean overshoot (%) | Mean control effort
--- | --- | ---: | ---: | ---: | ---:
nominal | PID | 0.8732 | 0.2135 | 4.20 | 9.6433
nominal | Q-learning | 6.9823 | 0.9000 | 38.60 | 26.8509
parameter_variation | PID | 0.9410 | 0.2279 | 5.27 | 10.5838
parameter_variation | Q-learning | 5.6605 | 0.7555 | 38.55 | 25.0416

IAE is integrated absolute tracking error; RMSE is root mean squared tracking error.
Overshoot is expressed relative to unit target amplitude. Control effort integrates
squared input. Lower is better for these metrics, with tradeoffs considered together.

## Finding

Tuned PID substantially outperforms this tabular Q-learning baseline. Q-learning
varies strongly across seeds and can oscillate or leave persistent tracking error.
This does not prove that PID always beats reinforcement learning; it establishes
performance of these specific implementations and training settings.

PID has integral memory while the RL state contains only discretized error/velocity.
The coarse RL state/action representation, finite training budget, lack of integral
state and differing training/evaluation disturbances are plausible limitations,
not experimentally isolated causal explanations. Follow-up experiments should isolate changes on validation scenarios and retain a separate final evaluation.

## Settling

Always read settling_success_rate.csv together with summary.csv. Conditional average
settling times exclude trials that never settle. A low conditional mean with few
successful runs is not good overall settling performance. The settling band is 2%
of the unit target and is checked across the complete 10-second observed horizon.

## Figures and reproducibility

step-response.png: one illustrative trajectory. learning-curves.png: all policy seeds.
response-error-control.png: representative output/error/action histories.
CSV files contain all trials and descriptive means/std. RL repeated scenarios and
three policy seeds do not provide independent window-like replicates; no confidence
interval or general stability guarantee is claimed. The findings apply to the simulated second-order plant and tested settings; hardware performance remains unverified.