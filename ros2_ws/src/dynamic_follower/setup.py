from setuptools import setup

package_name = 'dynamic_follower'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/dynamic_follower']),
        ('share/dynamic_follower', ['package.xml']),
        ('share/dynamic_follower/launch', ['launch/bringup.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Seu Nome',
    maintainer_email='seu@email.com',
    description='Nó para seguir alvo dinâmico com Nav2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'follower_node = dynamic_follower.follower_node:main'
        ],
    },
)
