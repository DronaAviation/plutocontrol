from setuptools import setup, find_packages

setup(
    name='plutocontrol',
    version='0.2.2',
    packages=find_packages(),
    py_modules=['plutocontrol'],
    install_requires=[],
    author='Omkar Dandekar',
    author_email='omi007dandekar@gmail.com',
    official_email='omkar.dandekar@dronaaviation.com',
    description='A library for controlling Pluto drones',
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    url='https://github.com/DronaAviation/plutocontrol.git',
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: MIT License',
        'Operating System :: OS Independent',
    ],
)
