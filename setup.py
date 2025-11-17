from setuptools import setup, find_packages

setup(
    name="aiofranka",
    version="0.1.0",
    packages=find_packages(),
    install_requires=[
        "ruckig", 
        "numpy", 
        "scipy", 

    ],
    author="MIT Improbable AI Lab",
    author_email="",
    description="A Python package for Franka robot control using asyncio.",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    url="https://github.com/Improbable-AI/bifranka",
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",  # Adjust license as needed
        "Operating System :: OS Independent",
    ],
    python_requires=">3.7",
)
