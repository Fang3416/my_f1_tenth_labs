from setuptools import setup

package_name = 'my_f1_tenth_labs'

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
    maintainer='foxy',
    maintainer_email='foxy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_f1_tenth_labs.my_node:main',
            'emergency_braking = my_f1_tenth_labs.emergency_braking:main',
            'wall_follow = my_f1_tenth_labs.wall_follow:main',
            'follow_the_gap = my_f1_tenth_labs.follow_the_gap:main'
        ],
    },
)
