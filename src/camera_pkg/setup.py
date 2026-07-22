from setuptools import setup

package_name = "camera_pkg"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Matty_Rotman",
    maintainer_email="mattyrot@post.bgu.ac.il",
    description="package for ROS 2 camera",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["camera_node = camera_pkg.camera_node:main"],
    },
)
