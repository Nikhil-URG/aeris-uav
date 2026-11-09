import os

from setuptools import find_packages, setup

package_name = "aeris_perception"

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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nikhil Ravi",
    maintainer_email="nikhi.ravi@smail.inf.h-brs.de",
    description="AERIS OpenCV perception and target localization.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "perception = aeris_perception.node:main",
        ],
    },
)
