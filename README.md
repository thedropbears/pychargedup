# pychargedup

The Drop Bears' robot code for _FIRST_ Charged Up (FRC 2023)

## Setup

### Install Dependencies

```
pip install -r requirements-dev.txt
```

### Setup pre-commit

Pre-commit is setup to automatically run formatters and linters when you commit.

```
pre-commit install
```


## Run

### Simulation

```
python robot.py sim
```

### Deploy to Robot

Once on robots network

```
python robot.py deploy
```

### Test

```
python robot.py test
```


## Code Structure

We use RobotPy's Magicbot framework

`robot.py`: Entry point, has mapping from driver inputs to high level robot actions.

`components/`: Abstracts hardware into robot actions.

`controllers/`: Automates robot actions, mostly with state machines.

`autonomous/`: Controls robot during autonomous period.

`ids.py`: Has CAN ids, PH channels and other port numbers.
