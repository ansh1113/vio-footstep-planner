from setuptools import setup, find_packages

setup(
    name="vio-footstep-planner",
    version="0.1.0",
    description="VIO + Footstep Planner Fusion for GPS-Denied Navigation",
    author="Ansh Bhansali",
    author_email="anshbhansali5@gmail.com",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    install_requires=[
        "numpy>=1.21.0",
        "scipy>=1.7.0",
        "opencv-python>=4.5.0",
    ],
    python_requires=">=3.8",
)
