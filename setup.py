from setuptools import setup, find_packages

with open('README.md', encoding='utf-8') as f:
    long_description = f.read()

setup(
    name='plutocontrol',
    version='2.1.0',
    packages=find_packages(),
    install_requires=[],
    python_requires='>=3.8',
    author='Omkar Dandekar',
    author_email='omi007dandekar@gmail.com',
    description='A library for controlling Pluto drones',
    long_description=long_description,
    long_description_content_type='text/markdown',
    url='https://github.com/DronaAviation/plutocontrol.git',
    license='MIT',
    classifiers=[
        'Programming Language :: Python :: 3',
        'Operating System :: OS Independent',
    ],
)
