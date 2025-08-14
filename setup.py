# -*- coding: utf-8 -*-

from setuptools import find_packages
from setuptools import setup

import fastentrypoints

dependencies = ["click"]

config = {
    "version": "0.1",
    "name": "power_supply_igps1101a_tool",
    "url": "https://github.com/jakeogh/power-supply-igps1101a-tool",
    "license": "ISC",
    "author": "Justin Keogh",
    "author_email": "github.com@v6y.net",
    "description": "read stats from the Kimball Physics igps-1101a ion gun power supply",
    "long_description": __doc__,
    "packages": find_packages(exclude=["tests"]),
    "package_data": {"power_supply_igps1101a_tool": ["py.typed"]},
    "include_package_data": True,
    "zip_safe": False,
    "platforms": "any",
    "install_requires": dependencies,
    "entry_points": {
        "console_scripts": [
            "power-supply-igps1101a-tool=power_supply_igps1101a_tool.cli:cli",
        ],
    },
}

setup(**config)
