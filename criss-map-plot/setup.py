from setuptools import find_packages, setup

package_name = 'criss-map-plot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),

    ],
    install_requires=['setuptools', 'matplotlib', 'numpy'],
    zip_safe=True,
    maintainer='deer',
    maintainer_email='dheerare@proton.me',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'plot_map = criss-map-plot.plot_map:main',
        ],
    },
)
