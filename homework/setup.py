from setuptools import setup

package_name = 'homework'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Patrick Ng',
    maintainer_email='png@hawaii.edu',
    description='Homework 1 for ME696 Fall 2023',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'problem3 = homework.problem3:main',
            'problem5 = homework.problem5:main',
        ],
    },
)
