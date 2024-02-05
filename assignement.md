# Instruction on technical interview (Research engineer)

1. A candidate receives this repository from DataLabs.
2. The candidate does its task (its detail is written below) in this repository.
3. The candidate submits it to DataLabs.
4. The DataLabs and the candidate have an interview.

## Task of candidate

The candidate chooses one of the programming languages (Python or C++), and does the task below.

### Python

1. Read and understand [Open3D tutorial for pointcloud registration pipeline](http://www.open3d.org/docs/release/tutorial/pipelines/global_registration.html).
   Namely,

- load pointcloud
- preprocess
  - downsample
  - compute features (normal/FPFH)
- rough registration (with RANSAC)
- fine registration (with ICP)

2. Implement the following to reproduce the pipeline above:

   - a reusable python module in `pcd_register/`
   - its unittest in `tests/`
   - application code which imports methods/classes from `pcd_register` in `main.py`

   In doing it, be careful on the following points:

   - Make `git` commit with appropriate granularity
   - write [docstring](https://realpython.com/documenting-python-code/)
   - write [type annotation](https://realpython.com/python-type-checking/)
   - Follow [PEP8](https://www.python.org/dev/peps/pep-0008/)
   - If you use external library, specify all the libraries and their version in one of the following files:
     - `pyproject.toml/poetry.lock`
     - `requirements.txt`

3. Write the necessary documents to this `README.md`, which includes:

   - how to install
   - how to run the code

For the format, see the end of this file.

### C++

_To be written..._

## Task of interviewer(s)

1. Once the candidate submits the "homework", the interviewer(s) reviews it.
2. By the date of interview, prepare list of questions/discussion topics.

## Time budget

The candidate is asked to submit her/his homeworks as soon as it is done, and up to three weeks.

## What to be reviewed by the technical interview

### From submitted repository

- Quality of code

### From interview

- Whether the candidate can explain intension of his/her design/implementation.
- Whether the candidate can constructive discussion on how to improve design/implementation

## Technical references

### software engineering

- [git [realpython]](https://realpython.com/python-git-github-intro/)
- [what is unittest? [wikipedia]](https://en.wikipedia.org/wiki/Unit_testing)

### python

- [realpython](https://realpython.com/)
- python module
- installing/managing python environment:
  - [pip [realpython]](https://realpython.com/what-is-pip/)
  - [poetry](https://python-poetry.org/)
- unittest tool ([pytest](https://docs.pytest.org/en/6.2.x/))

### C++

_To be written..._

### 3D data

- [Open3D](http://www.open3d.org/docs/release/index.html)

---

**Below the candidate should edit to make proper README for his/her codes.**

## Prerequisites

Before you begin, ensure you have met the following requirements:

### OS

...

### Other software dependencies

...

## Installation

...

## Usage

Follow these steps:

...
