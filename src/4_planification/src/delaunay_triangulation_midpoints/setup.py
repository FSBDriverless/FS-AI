from setuptools import setup

package_name = 'delaunay_triangulation_midpoints'

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
    maintainer='dv',
    maintainer_email='driverless@fsbizkaia.com',
    description='Delaunay Triangulation trayectory planning',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_middle_path = delaunay_triangulation_midpoints.simple_middle_path:main',
            'delaunay_middle_path = delaunay_triangulation_midpoints.delaunay_middle_path:main',
        ],
    },
)
