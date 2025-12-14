.PHONY: help install install-dev test lint format clean build docs

help:  ## Show this help message
	@echo 'Usage: make [target]'
	@echo ''
	@echo 'Available targets:'
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-15s\033[0m %s\n", $$1, $$2}'

install:  ## Install package and dependencies
	pip install -e .

install-dev:  ## Install package with dev dependencies
	pip install -e ".[dev]"

test:  ## Run tests
	pytest tests/ -v

test-cov:  ## Run tests with coverage
	pytest tests/ -v --cov=src --cov-report=html --cov-report=term

lint:  ## Run linters
	ruff check src/ tests/
	black --check src/ tests/

format:  ## Format code
	black src/ tests/
	ruff check --fix src/ tests/

type-check:  ## Run type checker
	mypy src/

clean:  ## Clean build artifacts
	rm -rf build/
	rm -rf dist/
	rm -rf *.egg-info
	rm -rf .pytest_cache/
	rm -rf .coverage
	rm -rf htmlcov/
	find . -type d -name __pycache__ -exec rm -rf {} +
	find . -type f -name '*.pyc' -delete
	find . -type f -name '*.pyo' -delete

build:  ## Build distribution packages
	python -m build

smoke-test:  ## Run minimal smoke test
	python -c "import vio_footstep_planner; print('✓ Import successful')"
	python -c "from vio_footstep_planner import VIONavigator, FootstepPlanner; print('✓ Main classes importable')"
	python -m pytest tests/test_smoke.py -v

all: clean install-dev lint test  ## Run full development cycle
