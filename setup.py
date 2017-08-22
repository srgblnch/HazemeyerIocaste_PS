# -*- coding: utf-8 -*-
# ##### BEGIN GPL LICENSE BLOCK #####
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 3
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, see <http://www.gnu.org/licenses/>
#
# ##### END GPL LICENSE BLOCK #####

from setuptools import setup, find_packages


__author__ = "Lothar Krause"
__maintainer__ = "Sergi Blanch-Torné"
__email__ = "sblanch@cells.es"
__copyright__ = "Copyright 2017, CELLS / ALBA Synchrotron"
__license__ = "GPLv3+"

__project__ = 'HazemeyerIocaste_PS'
__description__ = "Python module to control via Tango-CS the Hazemeyer "\
                  "power converter controlled called Iocaste."
__longDesc__ = """
This module has been prepared to provide control using the Tango-CS of the
power converter from the Hazemeyer manufacturer.
"""
__url__ = "https://github.com/srgblnch/HazemeyerIocaste_PS"
# we use semantic versioning (http://semver.org/) and we update it using the
# bumpversion script (https://github.com/peritus/bumpversion)
__version__ = '1.29.0-alpha'


setup(name=__project__,
      license=__license__,
      description=__description__,
      long_description=__longDesc__,
      version=__version__,
      author=__author__,
      author_email=__email__,
      classifiers=['Development Status :: 5 - Production/Stable',
                   'Intended Audience :: Developers',
                   'Intended Audience :: Science/Research',
                   'License :: OSI Approved :: '
                   'GNU General Public License v3 or later (GPLv3+)',
                   'Operating System :: POSIX',
                   'Programming Language :: Python',
                   'Topic :: Scientific/Engineering :: '
                   'Interface Engine/Protocol Translator',
                   'Topic :: Software Development :: Embedded Systems',
                   'Topic :: Software Development :: Libraries :: '
                   'Python Modules',
                   'Topic :: System :: Hardware',
                   ],
      packages=find_packages(),
      url=__url__,
      entry_points={
        'console_scripts':['HazemeyerIocaste_PS = HazemeyerIocaste_PS.main']},
      )

# for the classifiers review see:
# https://pypi.python.org/pypi?%3Aaction=list_classifiers
#
# Development Status :: 1 - Planning
# Development Status :: 2 - Pre-Alpha
# Development Status :: 3 - Alpha
# Development Status :: 4 - Beta
# Development Status :: 5 - Production/Stable
