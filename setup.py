import platform
from setuptools import Extension, setup
import pybind11

# Cross-platform C++ compilation flags
# MSVC (Windows) and GCC/Clang (Linux/macOS) use different flag syntax
is_msvc = platform.system() == "Windows"

if is_msvc:
    compile_args = ["/O2", "/std:c++17", "/EHsc"]
else:
    compile_args = ["-O3", "-std=c++17"]

setup(
    ext_modules=[
        Extension(
            "robot_ik.ik_fast",
            sources=["csrc/ik_fast.cpp"],
            include_dirs=[pybind11.get_include()],
            language="c++",
            extra_compile_args=compile_args,
        ),
        Extension(
            "robot_ik.robot_dyn_fast",
            sources=["csrc/robot_dyn_fast.cpp"],
            include_dirs=[pybind11.get_include()],
            language="c++",
            extra_compile_args=compile_args,
        ),
    ],
)
