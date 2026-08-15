from setuptools import setup

package_name = "moveit2_extended_studio"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # The UI is served from the share directory, so it is installed rather than read out of
        # the source tree.
        ("share/" + package_name + "/web",
         ["web/index.html", "web/studio.js", "web/studio.css"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Leow Chee Siang",
    maintainer_email="cheesiang_leow@alps-lab.org",
    description="Browser UI for the moveit2-extended Objective/Behavior layer.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "studio_server = moveit2_extended_studio.studio_server:main",
        ],
    },
)
