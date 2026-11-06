from glob import glob

from setuptools import setup

PACKAGE = "survey_demo"

setup(
    name=PACKAGE,
    version="0.1.0",
    packages=[PACKAGE],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE}"]),
        (f"share/{PACKAGE}", ["package.xml"]),
        (f"share/{PACKAGE}/launch", glob("launch/*.launch.py")),
        (f"share/{PACKAGE}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alexli18",
    maintainer_email="alexligay1807@gmail.com",
    description="Toy survey sensor, marker detector, ready gate, bag player and launch files",
    license="MIT",
    entry_points={
        "console_scripts": [
            "toy_sensor_node = survey_demo.toy_sensor_node:main",
            "marker_detector_node = survey_demo.marker_detector_node:main",
            "ready_gate = survey_demo.ready_gate:main",
            "bag_player = survey_demo.bag_player:main",
        ],
    },
)
