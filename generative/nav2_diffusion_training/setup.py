from setuptools import setup

package_name = 'nav2_diffusion_training'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ryohei Sasaki',
    maintainer_email='rsasaki0109@gmail.com',
    description='Dataset and training tools for Nav2PlannerBattle.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
