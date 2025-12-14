from setuptools import setup, find_packages

setup(
    name="rl-locomotion-cbf",
    version="0.1.0",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    install_requires=["numpy", "scipy", "osqp", "stable-baselines3"],
)
