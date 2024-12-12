from setuptools import setup
import os
from glob import glob

package_name = 'image_processor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seu_nome',
    maintainer_email='seu_email@exemplo.com',
    description='Pacote para processar imagens usando ROS 2',
    license='Licença do pacote',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_publisher = image_processor.image_publisher:main',
            'image_subscriber = image_processor.image_subscriber:main',
        ],
    },
)
