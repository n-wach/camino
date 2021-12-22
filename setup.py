import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="camino",
    version="1.0.1",
    author="Nathan Wachholz",
    author_email="n-wach@github.com",
    description="A library and framework for controlling an Arduino from Python over Serial",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/n-wach/camino",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
    install_requires=[
        'pyserial',
    ],
)
