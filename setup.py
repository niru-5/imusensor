import setuptools


with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="imusensor",
    version="1.0.1",
    author="Niranjan Kukkala",
    author_email="niranjan.reddy.wo@gmail.com",
    description="Linking Raspberry pi with MPU9250",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/niru-5/imu-mpu9250/tree/master",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3',
)
