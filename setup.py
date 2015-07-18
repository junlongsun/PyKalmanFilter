from distutils.core import setup
#from setuptools import setup, find_packages

setup(
      name="PyKalman",
      version="1.10",
      py_modules = ['Filter',
                    'Object',
                    'Observer',                    
                    'Sensor',
                    'mainMulti_demo',
                    'mainMulti_SR_demo',
                    'mainSingle_demo',                    
                    'cluster_demo',
                    'kf_cwpa_demo',
                    'playGround_demo',
                    'sensorRegistration_demo'],
      description="Object-oriented Kalman Filter in Python",
      author="Junlong Sun",
      author_email="Junlong.Sun@gmail.com",
      url="https://sites.google.com/site/junlongsun/",
      license="LGPL",
      #packages= find_packages(),
      #scripts=["scripts/test.py"],
)
