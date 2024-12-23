from setuptools import setup

package_name = 'final_project'

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
    maintainer='John Craft',
    maintainer_email='jjc0046@mix.wvu.edu',
    description='f1tenth final_project',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'final_project_node = final_project.final_project_node:main',
        ],
    },
)
