from setuptools import setup, find_packages

setup(
    name='plutocontrol',
    version='2.0.0',
    packages=find_packages(),
    install_requires=[],
    author='Omkar Dandekar',
    author_email='omi007dandekar@gmail.com',
    description='A library for controlling Pluto drones',
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    url='https://github.com/DronaAviation/plutocontrol.git',
    license='MIT',
    classifiers=[
        'Programming Language :: Python :: 3',
        'Operating System :: OS Independent',
    ],
)
