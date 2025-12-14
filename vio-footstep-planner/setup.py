from setuptools import setup, find_packages

setup(
    name="vio-footstep-planner",
    version="0.1.0",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    install_requires=["numpy", "scipy", "opencv-python"],
)
