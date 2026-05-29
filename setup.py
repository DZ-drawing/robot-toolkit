from setuptools import Extension, setup
import pybind11

setup(
    ext_modules=[
        Extension(
            "robot_ik.ik_fast",
            sources=["csrc/ik_fast.cpp"],
            include_dirs=[pybind11.get_include()],
            language="c++",
            extra_compile_args=["-O3"],
        ),
        Extension(
            "robot_ik.robot_dyn_fast",
            sources=["csrc/robot_dyn_fast.cpp"],
            include_dirs=[pybind11.get_include()],
            language="c++",
            extra_compile_args=["-O3"],
        ),
    ],
)
