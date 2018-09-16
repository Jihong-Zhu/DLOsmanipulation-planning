# Project Title

DLOs manipulation path planning based on energy minimum principle (must have a maximum minimum energy in the path.)

## Getting Started

Current implementation is based on Wakamatsu's IJRR paper on Linear Object Deformation. One arm is considered.

### Functions:

* nonlCon.m: nonlinear constraints (square) for the optimization problem
* nonlConCircle.m: nonlinear constraints (circle) for the optimization problem
* nonlConDual.m: nonlinear constraints for dual manipulation
* pathGen.m: Generate path for the end-effector
* pathGenBezier.m: Generate path for the end-effector using Bezier curve
* pathGenCost.m: Calculate cost of the generated path
* plotCost.m: plot the cost of the optimization (add when needed to analyse the result)

### Main scripts:
* PlanningSingle.m: Single arm manipulation based on minimization of total energy

### Prerequisites

* Require dlo_modelling
https://github.com/Jihong-Zhu/dlo_modelling
addpath('.../dlo_modelling');

### Installing


## Running

```
Run PlanningSingle.m
```

Currently dual arm planning is not working
## Authors

* **Jihong Zhu** - *Initial work*

## License

This project is licensed under the Apache License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments
