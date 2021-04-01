package kalman_filter

import (
	"gonum.org/v1/gonum/mat"
)

type PointTracker struct {
	kf *KalmanFilterLinear
}

func NewPointTracker() *PointTracker {
	tracker := PointTracker{}
	tracker.kf = &KalmanFilterLinear{
		// Transition State Matrix A
		A: mat.NewDense(4, 4, []float64{
			1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1,
		}),
		B: mat.NewDense(4, 4, []float64{
			0, 0, 0, 0,
			0, 0, 0, 0,
			0, 0, 0, 0,
			0, 0, 0, 0,
		}),
		// Measure Matrix C
		C: mat.NewDense(2, 4, []float64{
			1, 0, 0, 0,
			0, 1, 0, 0,
		}),

		// State covariance
		P: mat.NewDense(4, 4, []float64{
			1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1,
		}),
		// Process covariance
		Q: mat.NewDense(4, 4, []float64{
			1e-5, 0, 0, 0,
			0, 1e-5, 0, 0,
			0, 0, 1e-5, 0,
			0, 0, 0, 1e-5,
		}),
		// Measurement covariance
		R: mat.NewDense(2, 2, []float64{
			1e-1, 0,
			0, 1e-1,
		}),
		// Initial state
		X: mat.NewDense(4, 1, []float64{
			0,
			0,
			0,
			0,
		}),
	}
	return &tracker
}

func (tracker *PointTracker) Process(u *mat.Dense, y *mat.Dense) (mat.Matrix, error) {
	return tracker.kf.Step(u, y)
}

func (tracker *PointTracker) SetTime(dT float64) {
	tracker.kf.A.Set(0, 2, dT)
	tracker.kf.A.Set(1, 3, dT)
}
