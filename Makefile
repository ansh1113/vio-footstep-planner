.PHONY: help install install-dev test lint format clean check

help:  ## Show this help message
	@echo 'Usage: make [target]'
	@echo ''
	@echo 'Available targets:'
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-15s\033[0m %s\n", $$1, $$2}'

install:  ## Install package
	pip install -e .

install-dev:  ## Install package with development dependencies
	pip install -e ".[dev]"

test:  ## Run tests
	pytest tests/ -v

test-cov:  ## Run tests with coverage report
	pytest tests/ -v --cov=vio_footstep_planner --cov-report=term-missing

lint:  ## Run code linters
	flake8 src/vio_footstep_planner tests --max-line-length=100 --extend-ignore=E203,W503,W293

format:  ## Format code with black
	black src/vio_footstep_planner tests

format-check:  ## Check code formatting
	black --check src/vio_footstep_planner tests

check:  ## Run all checks (lint + format-check + test)
	@$(MAKE) format-check
	@$(MAKE) lint
	@$(MAKE) test

clean:  ## Clean build artifacts
	rm -rf build/
	rm -rf dist/
	rm -rf *.egg-info
	rm -rf src/*.egg-info
	find . -type d -name __pycache__ -exec rm -rf {} +
	find . -type f -name "*.pyc" -delete
	find . -type d -name ".pytest_cache" -exec rm -rf {} +
	find . -type d -name ".coverage" -exec rm -rf {} +
	find . -type d -name "htmlcov" -exec rm -rf {} +
