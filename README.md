# pychargedup

The Drop Bears' robot code for _FIRST_ Charged Up (FRC 2023)

## Setup

### Install Dependencies

`pip install -r requirements-dev.txt`

### Setup pre-commit

Pre-commit is setup to automatically run formatters and linters when you commit.

`pre-commit install`


## Run

### Simulation

`python robot.py sim`

### Deploy to Robot

Once on robots network

`python robot.py deploy`

### Test

`python robot.py test`


## Code Structure

We use RobotPy's Magicbot framework

`robot.py`: Entry point, has mapping from driver inputs to high level robot actions.

`components/`: Abstracts hardware into robot actions.

`controllers/`: Automates robot actions, mostly with state machines.

`autonomous/`: Controls robot during autonomous period.

`ids.py`: Has CAN ids, PCM channels and other port numbers.


## Docker

### Setup

To build the image

`./docker/build_image.sh`

The built image will have the tag _dropbears_chargedup:latest_.  You should be able to see the built image by running

`docker image ls`

If the image built successfully you can create a container

`./docker/create_container.sh`

The created container will have the name _chargedup_container_.  You can check that the container started by running

`docker container ls`

To enter the container execute

`docker exec -it chargedup_container bash`

### Installing docker

If you don't have docker installed, follow the relevant instructions for your OS here -> https://docs.docker.com/get-docker/
