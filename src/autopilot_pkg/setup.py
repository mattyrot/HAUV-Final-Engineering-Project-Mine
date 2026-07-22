from setuptools import setup

package_name = "autopilot_pkg"

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
    maintainer="up",
    maintainer_email="mattyrot@post.bgu.ac.il",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "guidance_node = autopilot_pkg.guidance_node:main",
            "dvl_node = autopilot_pkg.dvl_node:main",
            "subsonus_node = autopilot_pkg.subsonus_node:main",
            "health_monitor_node = autopilot_pkg.health_monitor_node:main",
        ],
    },
)
