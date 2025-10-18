from setuptools import setup, find_packages

setup(
    name='cogniplan',
    version='1.0.0',
    description='CogniPlan: A cognitive planning system for robot navigation',
    author='CogniPlan Team',
    author_email='team@example.com',
    packages=find_packages(),
    install_requires=[
        'torch>=1.7.0',
        'numpy>=1.19.0',
    ],
    python_requires='>=3.6',
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Developers',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
    ],
)