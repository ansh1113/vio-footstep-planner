from setuptools import setup, find_packages

setup(
    name="quadruped-ppo",
    version="0.1.0",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    install_requires=["numpy", "gym", "pybullet", "stable-baselines3"],
)
