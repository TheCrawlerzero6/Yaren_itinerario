import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'yaren_master'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    # Los recursos que no son codigo (arranque, mundo y configuracion)
    # deben instalarse en share/ para que $(find yaren_master) y el
    # simulador puedan localizarlos. Sin estas entradas los ficheros
    # quedarian solo en el directorio de fuentes y el paquete instalado
    # no seria utilizable.
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Equipo Yaren',
    maintainer_email='alanbajag@gmail.com',
    description=(
        'Control y simulacion del robot Yaren: mundo de trabajo, '
        'configuracion de controladores, ficheros de arranque y programas '
        'de control en Python.'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'itinerario_node = yaren_master.itinerario_node:main',
            'teleop_teclado = yaren_master.teleop_teclado:main',
            'pose_service = yaren_master.pose_service:main',
            'estado_node = yaren_master.estado_node:main',
        ],
    },
)
