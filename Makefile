# ──────────────────────────────────────────────────────────────────────────────
# multi_robot_coverage — Docker helper targets
#
# Usage:
#   make build     Build the ROS2 Docker image
#   make sim       Run ROS2 sim (noVNC at http://localhost:6080/vnc.html)
#   make web       Run ROS2 sim + React dashboard (http://localhost:5173)
#   make test      Run all 47 unit tests
#   make benchmark        Run headless benchmark via Docker (4 algos x 3 maps)
#   make benchmark-native Same, but in a local Python venv (no Docker needed)
#   make shell     Open a bash shell inside the ROS2 container
#   make clean     Remove containers, volumes, and built image
# ──────────────────────────────────────────────────────────────────────────────

IMAGE   := multi_robot_coverage:humble
COMPOSE := docker compose

.PHONY: build sim web sim-args test benchmark benchmark-native shell rebuild clean help

## Build the ROS2 Docker image (~5 min first time)
build:
	$(COMPOSE) build coverage_sim

## Run simulation only — RViz2 at http://localhost:6080/vnc.html
sim:
	@echo ""
	@echo "  RViz2 (noVNC):     http://localhost:6080/vnc.html"
	@echo "  rosbridge:         ws://localhost:9090"
	@echo ""
	$(COMPOSE) up coverage_sim

## Run simulation + React web dashboard — open http://localhost:5173
web:
	@echo ""
	@echo "  Web dashboard:     http://localhost:5173"
	@echo "  RViz2 (noVNC):     http://localhost:6080/vnc.html"
	@echo ""
	$(COMPOSE) up coverage_sim dashboard

## Run with custom launch args, e.g.:
##   make sim-args ARGS="num_robots:=4 map:=warehouse enable_failure_sim:=true"
sim-args:
	$(COMPOSE) run --rm --service-ports coverage_sim \
		ros2 launch multi_robot_coverage coverage_demo.launch.py $(ARGS)

## Run all 47 unit tests (no display needed)
test:
	$(COMPOSE) --profile test up --abort-on-container-exit test

## Run the headless benchmark suite (4 algorithms x 3 maps = 12 runs) via Docker
## JSON output lands in ./results/  --  use BENCH_ARGS=... to filter, e.g.:
##   make benchmark BENCH_ARGS="--map obstacle_room --algorithm boustrophedon"
benchmark:
	@mkdir -p results
	$(COMPOSE) --profile bench up --abort-on-container-exit benchmark

## Run the benchmark suite NATIVELY (no Docker, faster, uses local Python venv)
## Creates .venv on first run, then runs the same suite as ``make benchmark``.
benchmark-native:
	@if [ ! -d .venv ]; then \
		echo "→ First run — creating .venv and installing deps..."; \
		python3 -m venv .venv && \
		./.venv/bin/pip install --quiet --upgrade pip && \
		./.venv/bin/pip install --quiet -r multi_robot_coverage/requirements.txt; \
		echo "→ Venv ready."; \
	fi
	@mkdir -p results web_dashboard/public/benchmarks
	cd multi_robot_coverage && \
		PYTHONPATH=. ../.venv/bin/python3 -m multi_robot_coverage.benchmark \
			--output-dir ../results $(BENCH_ARGS)
	@cp results/*.json web_dashboard/public/benchmarks/ 2>/dev/null || true
	@echo "✓ Benchmark complete — JSON in results/ and web_dashboard/public/benchmarks/"

## Open an interactive bash shell inside the ROS2 container
shell:
	$(COMPOSE) run --rm coverage_sim bash

## Rebuild the colcon workspace (needed after changing package.xml / CMakeLists)
rebuild:
	$(COMPOSE) run --rm coverage_sim bash -c \
		"cd /coverage_ws && colcon build --symlink-install"

## Remove all containers and volumes for a clean slate
clean:
	$(COMPOSE) down -v
	docker rmi $(IMAGE) 2>/dev/null || true

## Print available targets
help:
	@grep -E '^##' Makefile | sed 's/^## /  /'
