from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'local_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='santosh',
    maintainer_email='santoshkonduskar6@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
			"lane = local_controller.lane:main",
			"lane1 = local_controller.lane1:main",
			"control=local_controller.control:main",
			"ransac=local_controller.ransac:main",
			"ransac_main=local_controller.ransac_MAIN:main",
            "sequence1=local_controller.sequence1:main",
            "sequence2=local_controller.sequence2:main",
            "sequence1_mod=local_controller.sequence1_mod:main",
            "state_controller=local_controller.state_controller:main"

        ],
    },
)
