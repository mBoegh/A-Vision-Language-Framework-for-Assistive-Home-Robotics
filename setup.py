from setuptools import setup
import os
from glob import glob

package_name = 'rob7_760_2024'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py'))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A custom controller for TIAGo robot in Python',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'LlmNode = rob7_760_2024.LlmNode:main',
            'ImageSegmentationNode = rob7_760_2024.ImageSegmentationNode:main',
            'SemanticPointcloudNode = rob7_760_2024.SemanticPointcloudNode:main',
            'MainNode = rob7_760_2024.MainNode:main',
            'GetCentroidsNode = rob7_760_2024.GetCentroidsNode:main'
        ],
    },
)

