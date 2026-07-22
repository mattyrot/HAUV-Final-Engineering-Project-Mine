from setuptools import setup

package_name = "mavlink_bridge_pkg"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pymavlink"],
    zip_safe=True,
    maintainer="up",
    maintainer_email="mattyrot@post.bgu.ac.il",
    description="MAVLink bridge for QGroundControl integration",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mavlink_bridge_node=mavlink_bridge_pkg.mavlink_bridge_node:main",
        ],
    },
)
