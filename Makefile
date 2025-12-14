.PHONY: install dev test lint format clean help

help:  ## Show this help message
	@echo "Usage: make [target]"
	@echo ""
	@echo "Available targets:"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "  %-15s %s\n", $$1, $$2}'

install:  ## Install production dependencies
	pip install -r requirements.txt
	pip install -e .

dev:  ## Install development dependencies
	pip install -r requirements.txt
	pip install -r requirements-dev.txt
	pip install -e .
	pre-commit install

test:  ## Run tests
	pytest -q

test-verbose:  ## Run tests with verbose output
	pytest -v

lint:  ## Run linters
	flake8 src tests generate_code.py

format:  ## Format code with black and isort
	black src tests generate_code.py
	isort src tests generate_code.py

pre-commit:  ## Run pre-commit hooks on all files
	pre-commit run --all-files

clean:  ## Clean build artifacts
	rm -rf build/
	rm -rf dist/
	rm -rf *.egg-info
	rm -rf .pytest_cache/
	rm -rf .mypy_cache/
	rm -rf .tox/
	find . -type d -name __pycache__ -exec rm -rf {} +
	find . -type f -name "*.pyc" -delete

docker-build:  ## Build Docker image
	docker build -t vio-footstep-planner .

docker-run:  ## Run Docker container
	docker run -it vio-footstep-planner
