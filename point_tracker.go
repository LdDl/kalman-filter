package kalman_filter

import (
	"gonum.org/v1/gonum/mat"
)

// PointTracker Wraps KalmanFilterLinear
// It's 2-D based data *tracker*
// Useful for object tracking in 2-D space
type PointTracker struct {
	kf *KalmanFilterLinear
}

// NewPointTracker Creates new pointer to PointTracker
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

// SetStateValue Sets state value X
// x, y, velocity_x, velocity_y must be provided
func (tracker *PointTracker) SetStateValue(x, y, vx, vy float64) {
	tracker.kf.X.Set(0, 0, x)
	tracker.kf.X.Set(1, 0, y)
	tracker.kf.X.Set(2, 0, vx)
	tracker.kf.X.Set(3, 0, vy)
}

// Process Processing stage
func (tracker *PointTracker) Process(u *mat.Dense, y *mat.Dense) (mat.Matrix, error) {
	return tracker.kf.Step(u, y)
}

// SetTime Sets time delta into Transition State Matrix
func (tracker *PointTracker) SetTime(dT float64) {
	tracker.kf.A.Set(0, 2, dT)
	tracker.kf.A.Set(1, 3, dT)
}
