from setuptools import find_packages, setup

package_name = "p3at_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "launch/sim.launch.py",
            "launch/nav.launch.py",
            "launch/real.launch.py",
        ]),
        ("share/" + package_name + "/config", [
            "config/bridge.yaml",
            "config/ekf.yaml",
            "config/ekf_local.yaml",
            "config/ekf_global.yaml",
            "config/navsat_transform.yaml",
            "config/slam_toolbox.yaml",
            "config/nav2.yaml",
            "config/nav2_part2.yaml",
            "config/waypoints.yaml",
            "config/waypoints_gps.yaml",
            "config/mission_policy.yaml",
            "config/mission_bounds.yaml",
        ]),
        ("share/" + package_name + "/worlds", ["worlds/james_oval.sdf"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="god",
    maintainer_email="god@example.com",
    description="Bringup and mission tooling for AUTO4508 Part 1/Part 2.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "clock_sanitizer = p3at_bringup.clock_sanitizer:main",
            "fake_gps_node = p3at_bringup.fake_gps_node:main",
            "gps_health_monitor = p3at_bringup.gps_health_monitor:main",
            "joint_state_sanitizer = p3at_bringup.joint_state_sanitizer:main",
            "mission_manager = p3at_bringup.mission_manager:main",
            "mission_orchestrator = p3at_bringup.mission_orchestrator:main",
            "odom_sanitizer = p3at_bringup.odom_sanitizer:main",
            "path_recorder = p3at_bringup.path_recorder:main",
        ],
    },
)
