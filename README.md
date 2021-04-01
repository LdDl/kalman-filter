## Linear kalman filter
[![GoDoc](https://godoc.org/github.com/LdDl/kalman-filter?status.svg)](https://godoc.org/github.com/LdDl/kalman-filter) [![Sourcegraph](https://sourcegraph.com/github.com/LdDl/kalman-filter/-/badge.svg)](https://sourcegraph.com/github.com/LdDl/kalman-filter?badge) [![Go Report Card](https://goreportcard.com/badge/github.com/LdDl/kalman-filter)](https://goreportcard.com/report/github.com/LdDl/kalman-filter) [![GitHub tag](https://img.shields.io/github/tag/LdDl/kalman-filter.svg)](https://github.com/LdDl/kalman-filter/releases)

## Table of Contents

- [About](#about)
- [Installation](#installation)
- [Usage](#usage)
- [Support](#support-and-contribution)
- [Dependencies](#dependencies)
- [License](#license)

## About

This is just linear kalman filter implementation in Go.
It contains simple 2-d based (point) tracker also.

## Installation
Simply:
```shell
go get github.com/LdDl/kalman-filter
```

## Usage
File [point_tracker_test.go](point_tracker_test.go) two examples for usage Kalman filter with 2-d based datasets:

* [point_tracker_test.go](point_tracker_test.go#L15) - stands for example with no noise addition to data

* [point_tracker_test.go](point_tracker_test.go#L77) - stands for with noise addition to data

## Support and contribution

If you have troubles or questions please [open an issue](https://github.com/LdDl/kalman-filter/issues/new).

PR's are welcome.

## Dependencies
* Matrix computations - [gonum](https://github.com/gonum/gonum#gonum). License is **BSD 3-Clause "New" or "Revised" License**. [Link](https://github.com/gonum/gonum/blob/master/LICENSE)
* Errors wraping - [errors](https://github.com/pkg/errors#errors-----). License is **BSD 2-Clause "Simplified" License**. [Link](https://github.com/pkg/errors/blob/master/LICENSE)

## License
License of this library is [MIT](https://en.wikipedia.org/wiki/MIT_License).

You can check it [here](LICENSE.md)