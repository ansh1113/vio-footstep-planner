# Contributing to VIO Footstep Planner

Thank you for your interest in contributing! This document provides guidelines for contributing to the project.

## Code of Conduct

- Be respectful and inclusive
- Focus on constructive feedback
- Help create a welcoming environment

## How to Contribute

### Reporting Bugs

1. Check if the bug has already been reported in Issues
2. Create a new issue with:
   - Clear title and description
   - Steps to reproduce
   - Expected vs actual behavior
   - Environment details (OS, Python version, etc.)

### Suggesting Enhancements

1. Check existing issues and discussions
2. Create an issue describing:
   - The enhancement and its benefits
   - Possible implementation approach
   - Any potential drawbacks

### Pull Requests

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Make your changes following the style guide
4. Add tests for new functionality
5. Update documentation
6. Commit with clear messages
7. Push to your fork
8. Open a Pull Request

## Development Setup

```bash
# Clone your fork
git clone https://github.com/YOUR_USERNAME/vio-footstep-planner.git
cd vio-footstep-planner

# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install in development mode
pip install -e .
pip install -r requirements.txt

# Install development tools
pip install pytest black flake8 mypy
```

## Coding Standards

### Style Guide

- Follow PEP 8
- Use type hints for function signatures
- Maximum line length: 100 characters
- Use meaningful variable names

### Documentation

- Add docstrings to all public functions/classes
- Follow Google docstring format
- Update README for user-facing changes
- Add inline comments for complex logic

### Testing

- Write tests for new features
- Maintain or improve code coverage
- Run tests before submitting PR:

```bash
pytest tests/
```

### Code Quality

Run linting before committing:

```bash
black src/ tests/  # Format code
flake8 src/ tests/  # Check style
mypy src/          # Type checking
```

## Project Structure

See [DEVELOPER_GUIDE.md](DEVELOPER_GUIDE.md) for detailed architecture information.

## Commit Messages

Use clear, descriptive commit messages:

```
Add feature: Brief description

- Detailed point 1
- Detailed point 2
```

Good examples:
- `Add loop closure detection with ORB features`
- `Fix drift correction optimization convergence`
- `Update README with installation instructions`

## Review Process

1. Maintainers will review your PR
2. Address any requested changes
3. Once approved, PR will be merged

## Questions?

- Open a Discussion for questions
- Contact: anshbhansali5@gmail.com

## License

By contributing, you agree that your contributions will be licensed under the MIT License.
