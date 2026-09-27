from glob import glob

from setuptools import find_packages, setup

package_name = 'tram_backup_odometry'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/data', glob('data/*')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maria Katysheva',
    maintainer_email='mkkatysheva@gmail.com',
    description='Резервная одометрия трамвая',
    license='MIT',
    entry_points={'console_scripts': ['odometry = tram_backup_odometry.node:main']},
)
