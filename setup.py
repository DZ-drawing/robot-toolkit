from setuptools import setup, Extension, find_packages
import pybind11
import os

ext_ik = Extension(
    "robot_ik.ik_fast",
    sources=["csrc/ik_fast.cpp"],
    include_dirs=[pybind11.get_include()],
    language="c++",
    extra_compile_args=["-O3"],
)

ext_dyn = Extension(
    "robot_ik.robot_dyn_fast",
    sources=["csrc/robot_dyn_fast.cpp"],
    include_dirs=[pybind11.get_include()],
    language="c++",
    extra_compile_args=["-O3"],
)

setup(
    name="robot-ik",
    version="0.3.0",
    description="Fast 6-DOF Inverse Kinematics and Rigid Body Dynamics (C++ accelerated)",
    author="Danny Zeng",
    license="MIT",
    packages=find_packages(where="src"),
    package_dir={"robot_ik": "src/robot_ik"},
    python_requires=">=3.8",
    install_requires=["numpy>=1.24.0"],
    extras_require={
        "viz": ["matplotlib>=3.7.0"],
    },
    ext_modules=[ext_ik, ext_dyn],
    classifiers=[
        "Programming Language :: Python :: 3",
        "Programming Language :: C++",
        "Topic :: Scientific/Engineering",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
    ],
)
