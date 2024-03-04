from setuptools import setup

package_name = 'kimm_orchard_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='You',
    author_email='you@youremail.com',
    maintainer='YourFirstname Lastname',
    maintainer_email='your@youremail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='A simple ROS2 Python package',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pub_states = scripts.pub_states:main',
            'utm_publisher = scripts.utm_publisher:main',
            'set_entity_state = scripts.set_entity_state:main',
            'ranger_controller = ranger_controller:main',
            'ranger_controller_key = ranger_controller_key:main',
            'keyboard_teleop = scripts.keyboard_teleop:main',
            'path_publisher = scripts.path_debug.path_publisher:main',
            'base_link_state = scripts.base_link_state:main'
        ],
    },
)