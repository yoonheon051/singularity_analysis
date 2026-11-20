from glob import glob

from setuptools import setup

package_name = 'singularity_analysis'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yoonheon051',
    maintainer_email='yoonheon051@users.noreply.github.com',
    description='M0609 Jacobian 기반 특이점 분석',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ros2 run singularity_analysis <이름>
            'analyzer = singularity_analysis.m0609_ros_analyzer:main',
            'calculate_jacobian = singularity_analysis.calculate_jacobian:main',
            'plot_ellipsoid = singularity_analysis.plot_ellipsoid:main',
            'velocity_demo = singularity_analysis.velocity_demo:main',
            'visualize_results = singularity_analysis.visualize_results:main',
        ],
    },
)
