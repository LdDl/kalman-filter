package kalman_filter

import (
	"math"

	"github.com/pkg/errors"
	"gonum.org/v1/gonum/mat"
)

var (
	// Identity matrix
	identity2d = mat.NewDense(4, 4, []float64{
		1.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0,
	})
)

// Kalman2D is implementation of Discrete Kalman filter for case when there are two variables: X and Y.
type Kalman2D struct {
	// Transition matrix
	A *mat.Dense
	// Control matrix
	B *mat.Dense
	// Transformation (observation) matrix
	H *mat.Dense
	// Process noise covariance matrix
	Q *mat.Dense
	// Measurement noise covariance matrix
	R *mat.Dense
	// Error covariance matrix
	P *mat.Dense
	// State vector x, vx
	x *mat.Dense
	// Single cycle time
	dt float64
	// Control input
	u *mat.Dense
	// Standard deviation of acceleration
	stdDevA float64
	// Standard deviation of measurement for X
	stdDevMx float64
	// Standard deviation of measurement for Y
	stdDevMy float64

	// Preallocated memory
	ax_tmp     *mat.Dense
	bu_tmp     *mat.Dense
	ap_tmp     *mat.Dense
	hp_tmp     *mat.Dense
	htranspose mat.Matrix
	inv        *mat.Dense
	gain       *mat.Dense
	z          *mat.Dense
	r          *mat.Dense
	gain_r     *mat.Dense
	gain_h     *mat.Dense
	newp       *mat.Dense
}

// NewKalman2D creates a new Kalman2D filter.
func NewKalman2D(dt, ux, uy, stdDevA, stdDevMx, stdDevMy float64, options ...func(*Kalman2D)) *Kalman2D {

	// Ref.: Eq.(31)
	A := mat.NewDense(4, 4, []float64{
		1.0, 0.0, dt, 0.0,
		0.0, 1.0, 0.0, dt,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0,
	})

	// Ref.: Eq.(32)
	B := mat.NewDense(4, 2, []float64{
		0.5 * math.Pow(dt, 2), 0.0,
		0.0, 0.5 * math.Pow(dt, 2),
		dt, 0.0,
		0.0, dt,
	})

	// Ref.: Eq.(34)
	H := mat.NewDense(2, 4, []float64{
		1.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0,
	})

	// Ref.: Eq.(40)
	Q := mat.NewDense(4, 4, []float64{
		0.25 * math.Pow(dt, 4), 0.0, 0.5 * math.Pow(dt, 3), 0.0,
		0.0, 0.25 * math.Pow(dt, 4), 0.0, 0.5 * math.Pow(dt, 3),
		0.5 * math.Pow(dt, 3), 0.0, math.Pow(dt, 2), 0.0,
		0.0, 0.5 * math.Pow(dt, 3), 0.0, math.Pow(dt, 2),
	})
	Q.Scale(math.Pow(stdDevA, 2), Q)

	// Ref.: Eq.(41)
	R := mat.NewDense(2, 2, []float64{
		math.Pow(stdDevMx, 2), 0.0,
		0.0, math.Pow(stdDevMy, 2),
	})

	// Error covariance matrix P
	P := mat.NewDense(4, 4, []float64{
		1.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0,
	})

	// State vector x
	x := mat.NewDense(4, 1, []float64{
		0.0,
		0.0,
		0.0,
		0.0,
	})

	// Control input for X and Y
	u := mat.NewDense(2, 1, []float64{
		ux,
		uy,
	})

	k := &Kalman2D{
		dt:       dt,
		u:        u,
		stdDevA:  stdDevA,
		stdDevMx: stdDevMy,
		A:        A,
		B:        B,
		H:        H,
		Q:        Q,
		R:        R,
		P:        P,
		x:        x,
	}
	k.prealloc()
	for _, o := range options {
		o(k)
	}
	return k
}

// WithState2D sets custom initial state for state vector for 2D case
func WithState2D(x, y float64) func(*Kalman2D) {
	return func(k *Kalman2D) {
		k.x.Set(0, 0, x)
		k.x.Set(1, 0, y)
	}
}

// prealloc does preparations to reduce allocs
func (k *Kalman2D) prealloc() {
	/* Alloc for Predict: */
	arows, _ := k.A.Dims()
	_, xcols := k.x.Dims()
	ax_tmp := mat.NewDense(arows, xcols, nil)
	ax_tmp.Mul(k.A, k.x)

	brows, _ := k.B.Dims()
	_, ucols := k.u.Dims()
	bu_tmp := mat.NewDense(brows, ucols, nil)

	prows, pcols := k.P.Dims()
	ap_tmp := mat.NewDense(arows, pcols, nil)

	/* Alloc for Update: */
	hrows, hcols := k.H.Dims()
	hp_tmp := mat.NewDense(hrows, pcols, nil)
	htranspose := k.H.T()

	_, htcols := k.H.T().Dims()
	hprows, _ := hp_tmp.Dims()
	inv := mat.NewDense(hprows, htcols, nil)

	gain := mat.NewDense(prows, htcols, nil)

	z := mat.NewDense(2, 1, []float64{0, 0})

	r := mat.NewDense(hrows, xcols, nil)

	gainrows, _ := gain.Dims()
	_, rcols := r.Dims()
	gain_r := mat.NewDense(gainrows, rcols, nil)

	gain_h := mat.NewDense(gainrows, hcols, nil)

	identityrows, identitycols := identity2d.Dims()
	newp := mat.NewDense(identityrows, identitycols, nil)

	k.ax_tmp = ax_tmp
	k.bu_tmp = bu_tmp
	k.ap_tmp = ap_tmp
	k.hp_tmp = hp_tmp
	k.htranspose = htranspose
	k.inv = inv
	k.gain = gain
	k.z = z
	k.r = r
	k.gain_r = gain_r
	k.gain_h = gain_h
	k.newp = newp
}

// Predict projects the state and the error covariance ahead
// Mutates the state vector and the error covariance matrix
func (k *Kalman2D) Predict() {
	// Ref.: Eq.(5)
	k.ax_tmp.Mul(k.A, k.x)
	k.bu_tmp.Mul(k.B, k.u)

	k.x.Add(k.ax_tmp, k.bu_tmp)

	// Ref.: Eq.(6)
	k.ap_tmp.Mul(k.A, k.P)
	k.ap_tmp.Mul(k.ap_tmp, k.A.T())
	k.P.Add(k.ap_tmp, k.Q)
}

// Update computes the Kalman gain and then updates the state vector and the error covariance matrix
// Mutates the state vector and the error covariance matrix.
func (k *Kalman2D) Update(zvaluex, xvaluey float64) error {
	// Ref.: Eq.(7)
	k.hp_tmp.Mul(k.H, k.P)
	k.inv.Mul(k.hp_tmp, k.htranspose)
	k.inv.Add(k.inv, k.R)
	err := k.inv.Inverse(k.inv)
	if err != nil {
		return errors.Wrap(err, "Can't execute Update() due the error while gonum's Inverse() execution")
	}
	k.gain.Mul(k.P, k.htranspose)
	k.gain.Mul(k.gain, k.inv)
	// Ref.: Eq.(8)
	k.z.Set(0, 0, zvaluex)
	k.z.Set(1, 0, xvaluey)
	k.r.Mul(k.H, k.x)
	k.r.Sub(k.z, k.r)
	// Ref.: Eq.(9)
	k.gain_r.Mul(k.gain, k.r)
	k.x.Add(k.x, k.gain_r)
	// Ref.: Eq.(10)
	k.gain_h.Mul(k.gain, k.H)
	k.newp.Sub(identity2d, k.gain_h)
	k.P.Mul(k.newp, k.P)
	return nil
}

// GetState returns the current state (only X and Y, not Vx and Vy).
func (k *Kalman2D) GetState() (float64, float64) {
	return k.x.At(0, 0), k.x.At(1, 0)
}

// GetVectorState returns the copy current state (both (X, Y) and (Vx, Vy)).
func (k *Kalman2D) GetVectorState() *mat.Dense {
	x := mat.NewDense(4, 1, []float64{
		k.x.At(0, 0),
		k.x.At(1, 0),
		k.x.At(2, 0),
		k.x.At(3, 0),
	})
	return x
}
