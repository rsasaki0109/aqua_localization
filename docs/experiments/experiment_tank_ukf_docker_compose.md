# Experiment UKF tank dataset blind navigation with Docker Compose

This guide documents a reproducible way to run the existing UKF evaluation experiment using Docker Compose.

## Scope

This workflow automates an experiment that already exists in the repository:
- download/convert bag
- run UKF localization node
- record estimated and ground-truth trajectories
- compute trajectory error metrics

No algorithmic changes are introduced.

## Prerequisites

- Docker Engine + Docker Compose plugin (`docker compose`)
- Internet access (required for bag download)

## Files involved

- `docker/docker-compose.yml`
- `docker/dockerfiles/Dockerfile_tank_ukf_experiment`
- `scripts/run_experiment_tank_ukf.sh`

## Output directory parameterization

The experiment output directory is controlled by `OUT_DIR` (inside container).

Default value:
- `/root/tank_dataset/results`

Host mapping (example):
- `./experiments/tank_ukf_evaluation/results` (host) -> `/root/tank_dataset/results` (container)

You can change `OUT_DIR` in:
1. `docker-compose.yml` (`environment:`), and
2. script fallback in `run_experiment_tank_ukf.sh`:
   ```bash
   OUT_DIR="${OUT_DIR:-/root/tank_dataset/results}"
   ```


## Run

From `docker/` directory:

```bash
docker compose up --build exp_tank_ukf
```


## Verify results

On host:

```bash
ls -lah ../experiments/tank_ukf_evaluation/results
cat ../experiments/tank_ukf_evaluation/results/results_error.txt
```

Expected files:
- `estimated_trajectory.tum`
- `ground_truth_trajectory.tum`
- `results_error.txt`