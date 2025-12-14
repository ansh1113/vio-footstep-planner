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
        "pyyaml>=5.4.0",
    ],
    python_requires=">=3.8",
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
    ],
)
