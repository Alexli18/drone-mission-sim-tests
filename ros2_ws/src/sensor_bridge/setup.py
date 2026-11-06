from setuptools import setup

PACKAGE = "sensor_bridge"

setup(
    name=PACKAGE,
    version="0.1.0",
    packages=[PACKAGE],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE}"]),
        (f"share/{PACKAGE}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alexli18",
    maintainer_email="alexligay1807@gmail.com",
    description="Georeferencing of sensor captures against the vehicle pose",
    license="MIT",
    entry_points={
        "console_scripts": [
            "georeference_node = sensor_bridge.georeference_node:main",
        ],
    },
)
