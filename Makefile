.PHONY: install dev test lint format clean help

help:
	@echo "Available targets:"
	@echo "  install    - Install production dependencies"
	@echo "  dev        - Install development dependencies"
	@echo "  test       - Run tests with pytest"
	@echo "  lint       - Run flake8 linter"
	@echo "  format     - Format code with black and isort"
	@echo "  clean      - Remove build artifacts and caches"

install:
	pip install -r requirements.txt

dev:
	pip install -r requirements-dev.txt
	pre-commit install

test:
	pytest -v

lint:
	flake8 .

format:
	black .
	isort .

clean:
	find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
	find . -type d -name "*.egg-info" -exec rm -rf {} + 2>/dev/null || true
	find . -type d -name ".pytest_cache" -exec rm -rf {} + 2>/dev/null || true
	find . -type d -name ".mypy_cache" -exec rm -rf {} + 2>/dev/null || true
	find . -type d -name "build" -exec rm -rf {} + 2>/dev/null || true
	find . -type d -name "dist" -exec rm -rf {} + 2>/dev/null || true
	find . -type f -name "*.pyc" -delete
