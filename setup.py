from setuptools import setup, find_packages
import sys


setup(name='pepper_kinematics',
      version='0.0.6',
      url = 'http://www.sugarsweetrobotics.com/',
      author = 'ysuga',
      author_email = 'ysuga@ysuga.net',
      description = 'Pepper Inverse/Forward Kinematics library',
      download_url = 'https://github.com/sugarsweetrobotics/python_pepper_kinematics',
      packages = ["pepper_kinematics"],
      #py_modules = ["pepper_kinematics"],
      license = 'GPLv3',
      install_requires = ['numpy'],
      classifiers = [
        'Development Status :: 5 - Production/Stable',
        'Environment :: Console',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: GNU General Public License v3 (GPLv3)',
        'Natural Language :: English',
        'Operating System :: OS Independent',
        'Programming Language :: Python',
        'Topic :: Scientific/Engineering',
        ],
      #test_suite = "foo_test.suite",
      #package_dir = {'': 'src'}
    )
