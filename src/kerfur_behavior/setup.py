from setuptools import find_packages, setup

package_name = 'kerfur_behavior'

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
    maintainer='roo',
    maintainer_email='roo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'pad_to_face = kerfur_behavior.pad_to_face:main',
        'emotion_engine = kerfur_behavior.emotion_engine:main',
        'attention_selector = kerfur_behavior.attention_selector:main',
        ],
    },
)
