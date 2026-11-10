import os
from glob import glob

from setuptools import find_packages, setup

package_name = "aeris_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [os.path.join("resource", package_name)],
        ),
        (os.path.join("share", package_name), ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "config", "missions"),
            glob("config/missions/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "config", "grids"),
            glob("config/grids/*.yaml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nikhil Ravi",
    maintainer_email="nikhi.ravi@smail.inf.h-brs.de",
    description="Launch files and configuration for AERIS.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)
