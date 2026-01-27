from setuptools import find_packages, setup

package_name = 'vio_frontend'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Christopher Guarino',
    maintainer_email='chris@todo.com',
    description='VIO visual frontend: feature detection, tracking, and outlier rejection',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'frontend_node = vio_frontend.frontend_node:main',
        ],
    },
)
