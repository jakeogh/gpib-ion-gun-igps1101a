# -*- coding: utf-8 -*-

from setuptools import find_packages
from setuptools import setup

import fastentrypoints

dependencies = ["click"]

config = {
    "version": "0.1",
    "name": "gpib_ion_gun_igps1101a",
    "url": "https://github.com/jakeogh/gpib-ion-gun-igps1101a",
    "license": "ISC",
    "author": "Justin Keogh",
    "author_email": "github.com@v6y.net",
    "description": "read stats from the Kimball Physics igps-1101a ion gun power supply",
    "long_description": __doc__,
    "packages": find_packages(exclude=["tests"]),
    "package_data": {"gpib_ion_gun_igps1101a": ["py.typed"]},
    "include_package_data": True,
    "zip_safe": False,
    "platforms": "any",
    "install_requires": dependencies,
    "entry_points": {
        "console_scripts": [
            "gpib-ion-gun-igps1101a=gpib_ion_gun_igps1101a.cli:cli",
        ],
    },
}

setup(**config)
