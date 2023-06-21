package kalman_filter

import (
	"math"

	"github.com/pkg/errors"
	"gonum.org/v1/gonum/mat"
)

var (
	// Identity matrix
	identity = mat.NewDense(2, 2, []float64{
		1.0, 0.0,
		0.0, 1.0,
	})
)

/// Implementation of Discrete Kalman filter for case when there is only on variable X.
type Kalman1D struct {
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
	u float64
	// Standard deviation of acceleration
	stdDevA float64
	// Standard deviation of measurement
	stdDevM float64
}

// NewKalman1D creates a new Kalman1D filter.
func NewKalman1D(dt, u, stdDevA, stdDevM float64) *Kalman1D {

	// Transition matrix A
	A := mat.NewDense(2, 2, []float64{
		1.0, dt,
		0.0, 1.0,
	})

	// Control matrix B
	B := mat.NewDense(2, 1, []float64{
		0.5 * math.Pow(dt, 2),
		dt,
	})

	// Transformation matrix H
	H := mat.NewDense(1, 2, []float64{
		1.0, 0.0,
	})

	// Process noise covariance matrix Q
	Q := mat.NewDense(2, 2, []float64{
		0.25 * math.Pow(dt, 4), 0.5 * math.Pow(dt, 3),
		0.5 * math.Pow(dt, 3), math.Pow(dt, 2),
	})
	Q.Scale(math.Pow(stdDevA, 2), Q)

	// Measurement noise covariance matrix R
	R := mat.NewDense(1, 1, []float64{
		math.Pow(stdDevM, 2),
	})

	// Error covariance matrix P
	P := mat.NewDense(2, 2, []float64{
		1.0, 0.0,
		0.0, 1.0,
	})

	// State vector x
	x := mat.NewDense(2, 1, []float64{
		0.0,
		0.0,
	})

	return &Kalman1D{
		dt:      dt,
		u:       u,
		stdDevA: stdDevA,
		stdDevM: stdDevM,
		A:       A,
		B:       B,
		H:       H,
		Q:       Q,
		R:       R,
		P:       P,
		x:       x,
	}
}

func (k *Kalman1D) Predict() {
	// Ref.: Eq.(5)
	arows, _ := k.A.Dims()
	_, xcols := k.x.Dims()
	ax_tmp := mat.NewDense(arows, xcols, nil)
	ax_tmp.Mul(k.A, k.x)
	brows, bcols := k.B.Dims()
	bu_tmp := mat.NewDense(brows, bcols, nil)
	bu_tmp.Scale(k.u, k.B)
	k.x.Add(ax_tmp, bu_tmp)

	// Ref.: Eq.(6)
	_, pcols := k.P.Dims()
	ap_tmp := mat.NewDense(arows, pcols, nil)
	ap_tmp.Mul(k.A, k.P)
	ap_tmp.Mul(ap_tmp, k.A.T())
	k.P.Add(ap_tmp, k.Q)
}

// Update computes the Kalman gain and then updates the state vector and the error covariance matrix.
func (k *Kalman1D) Update(zvalue float64) error {
	// Ref.: Eq.(7)
	hrows, hcols := k.H.Dims()
	prows, pcols := k.P.Dims()
	hp_tmp := mat.NewDense(hrows, pcols, nil)
	hp_tmp.Mul(k.H, k.P)
	hprows, _ := hp_tmp.Dims()
	htranspose := k.H.T()
	_, htcols := htranspose.Dims()
	inv := mat.NewDense(hprows, htcols, nil)
	inv.Mul(hp_tmp, htranspose)
	inv.Add(inv, k.R)
	err := inv.Inverse(inv)
	if err != nil {
		return errors.Wrap(err, "Can't execute Update() due the error while gonum's Inverse() execution")
	}
	gain := mat.NewDense(prows, htcols, nil)
	gain.Mul(k.P, htranspose)
	gain.Mul(gain, inv)
	// Ref.: Eq.(8)
	z := mat.NewDense(1, 1, []float64{zvalue})
	_, xcols := k.x.Dims()
	r := mat.NewDense(hrows, xcols, nil)
	r.Mul(k.H, k.x)
	r.Sub(z, r)
	// Ref.: Eq.(9)
	gainrows, _ := gain.Dims()
	_, rcols := r.Dims()
	gain_r := mat.NewDense(gainrows, rcols, nil)
	gain_r.Mul(gain, r)
	k.x.Add(k.x, gain_r)
	// Ref.: Eq.(10)
	gain_h := mat.NewDense(gainrows, hcols, nil)
	gain_h.Mul(gain, k.H)

	identityrows, identitycols := identity.Dims()
	newp := mat.NewDense(identityrows, identitycols, nil)
	newp.Sub(identity, gain_h)
	k.P.Mul(newp, k.P)
	return nil
}

// GetState returns the current state (only X, not Vx).
func (k *Kalman1D) GetState() float64 {
	return k.x.At(0, 0)
}

// GetVectorState returns the copy current state (both X and Vx).
func (k *Kalman1D) GetVectorState() *mat.Dense {
	x := mat.NewDense(2, 1, []float64{
		k.x.At(0, 0),
		k.x.At(1, 0),
	})
	return x
}
