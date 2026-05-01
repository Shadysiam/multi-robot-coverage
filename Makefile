# ──────────────────────────────────────────────────────────────────────────────
# multi_robot_coverage — Docker helper targets
#
# Usage:
#   make build            Build the Docker image
#   make sim              Run the default simulation (RViz2 opens automatically)
#   make sim ARGS="num_robots:=4 map:=warehouse algorithm:=boustrophedon"
#   make test             Run all 47 unit tests inside Docker
#   make shell            Open a bash shell inside the container
#   make clean            Remove containers, volumes, and the built image
# ──────────────────────────────────────────────────────────────────────────────

IMAGE   := multi_robot_coverage:humble
COMPOSE := docker compose

# ── Detect OS and set display forwarding ──────────────────────────────────────

UNAME := $(shell uname)

ifeq ($(UNAME), Darwin)
  # macOS: XQuartz uses TCP on localhost
  DISPLAY_ENV := DISPLAY=host.docker.internal:0
  XHOST_CMD   := xhost + 127.0.0.1
  X11_VOLUME  :=               # no /tmp/.X11-unix on Mac
else
  # Linux: use the Unix socket
  DISPLAY_ENV := DISPLAY=$(DISPLAY)
  XHOST_CMD   := xhost +local:docker
  X11_VOLUME  := -v /tmp/.X11-unix:/tmp/.X11-unix
endif

# ── Targets ───────────────────────────────────────────────────────────────────

.PHONY: build sim test shell clean help

## Build the Docker image (only needed once, or after Dockerfile changes)
build:
	$(COMPOSE) build coverage_sim

## Run the full simulation with RViz2
sim: _xhost
	$(DISPLAY_ENV) $(COMPOSE) up coverage_sim

## Run with extra launch args, e.g.:
##   make sim-args ARGS="num_robots:=4 map:=warehouse enable_failure_sim:=true"
sim-args: _xhost
	$(DISPLAY_ENV) $(COMPOSE) run --rm coverage_sim \
		ros2 launch multi_robot_coverage coverage_demo.launch.py $(ARGS)

## Run all unit tests (no display needed)
test:
	$(COMPOSE) --profile test up --abort-on-container-exit test

## Open an interactive shell inside the container
shell: _xhost
	$(DISPLAY_ENV) $(COMPOSE) run --rm coverage_sim bash

## Rebuild the workspace inside a running container (after changing package.xml / CMakeLists)
rebuild:
	$(COMPOSE) run --rm coverage_sim bash -c \
		"cd /coverage_ws && colcon build --symlink-install"

## Stop all containers and remove volumes (fresh start)
clean:
	$(COMPOSE) down -v
	docker rmi $(IMAGE) 2>/dev/null || true

## Print this help
help:
	@grep -E '^##' Makefile | sed 's/^## //'

# ── Internal ──────────────────────────────────────────────────────────────────

_xhost:
ifeq ($(UNAME), Darwin)
	@echo "[make] Granting XQuartz access — you may see an 'xhost' warning, that is normal."
	@xhost + 127.0.0.1 2>/dev/null || echo "[make] XQuartz not running — open XQuartz.app first."
else
	@$(XHOST_CMD) 2>/dev/null || true
endif
