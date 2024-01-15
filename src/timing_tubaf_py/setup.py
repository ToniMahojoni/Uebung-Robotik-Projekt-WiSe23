from setuptools import find_packages, setup

package_name = 'timing_tubaf_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='toni',
    maintainer_email='toni.sand@student.tu-freiberg.de',
    description='python subscriber for the publisher/subscriber exercise',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'listener = timing_tubaf_py.subscriber_member_function:main',
        ],
    },
)
