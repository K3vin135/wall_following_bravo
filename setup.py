from setuptools import find_packages, setup

package_name = 'wall_following_bravo'

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
    maintainer='kevin-v',
    maintainer_email='kevinvaron6@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dist_finder_bravo = wall_following_bravo.dist_finder_bravo:main',
            'control_bravo = wall_following_bravo.control_bravo:main',
        ],
    },
)