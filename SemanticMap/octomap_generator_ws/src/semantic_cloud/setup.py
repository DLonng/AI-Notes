from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    # 这里你要引用的 pkg 目录
    packages=['semantic_cloud_generator'],
    # 包含这个文件夹的目录比如这里我是放到 src 目录下
    package_dir={'': 'include'}, 
)

setup(**setup_args)