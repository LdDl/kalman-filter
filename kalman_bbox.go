package kalman_filter

import (
	"math"

	"github.com/pkg/errors"
	"gonum.org/v1/gonum/mat"
)

var (
	// Identity matrix for BBox (8x8)
	identityBBox = mat.NewDense(8, 8, []float64{
		1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
	})
)

// KalmanBBox is implementation of Discrete Kalman filter for bounding box tracking.
// State vector contains: center_x, center_y, width, height, velocity_cx, velocity_cy, velocity_w, velocity_h
type KalmanBBox struct {
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
	// State vector: cx, cy, w, h, vx, vy, vw, vh
	x *mat.Dense
	// Control input (accelerations for cx, cy, w, h)
	u *mat.Dense

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

	// Single cycle time
	dt float64
	// Standard deviation of acceleration
	stdDevA float64
	// Standard deviation of measurement for center X
	stdDevMCx float64
	// Standard deviation of measurement for center Y
	stdDevMCy float64
	// Standard deviation of measurement for width
	stdDevMW float64
	// Standard deviation of measurement for height
	stdDevMH float64
}

// NewKalmanBBox creates a new KalmanBBox filter
//
// dt - Single cycle time
// uCx - Control input for center X
// uCy - Control input for center Y
// uW - Control input for width
// uH - Control input for height
// stdDevA - Standard deviation of acceleration
// stdDevMCx - Standard deviation of measurement for center X
// stdDevMCy - Standard deviation of measurement for center Y
// stdDevMW - Standard deviation of measurement for width
// stdDevMH - Standard deviation of measurement for height
// options - functional parameters
func NewKalmanBBox(dt, uCx, uCy, uW, uH, stdDevA, stdDevMCx, stdDevMCy, stdDevMW, stdDevMH float64, options ...func(*KalmanBBox)) *KalmanBBox {
	dt2 := dt * dt
	dt3 := dt2 * dt
	dt4 := dt3 * dt
	stdDevA2 := stdDevA * stdDevA

	// Ref.: Eq.(53)
	A := mat.NewDense(8, 8, []float64{
		1.0, 0.0, 0.0, 0.0, dt, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0, 0.0, dt, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0, 0.0, 0.0, dt, 0.0,
		0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, dt,
		0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
	})

	// Ref.: Eq.(54)
	B := mat.NewDense(8, 4, []float64{
		0.5 * dt2, 0.0, 0.0, 0.0,
		0.0, 0.5 * dt2, 0.0, 0.0,
		0.0, 0.0, 0.5 * dt2, 0.0,
		0.0, 0.0, 0.0, 0.5 * dt2,
		dt, 0.0, 0.0, 0.0,
		0.0, dt, 0.0, 0.0,
		0.0, 0.0, dt, 0.0,
		0.0, 0.0, 0.0, dt,
	})

	// Ref.: Eq.(56)
	H := mat.NewDense(4, 8, []float64{
		1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
	})

	// Ref.: Eq.(62)
	Q := mat.NewDense(8, 8, []float64{
		0.25 * dt4 * stdDevA2, 0.0, 0.0, 0.0, 0.5 * dt3 * stdDevA2, 0.0, 0.0, 0.0,
		0.0, 0.25 * dt4 * stdDevA2, 0.0, 0.0, 0.0, 0.5 * dt3 * stdDevA2, 0.0, 0.0,
		0.0, 0.0, 0.25 * dt4 * stdDevA2, 0.0, 0.0, 0.0, 0.5 * dt3 * stdDevA2, 0.0,
		0.0, 0.0, 0.0, 0.25 * dt4 * stdDevA2, 0.0, 0.0, 0.0, 0.5 * dt3 * stdDevA2,
		0.5 * dt3 * stdDevA2, 0.0, 0.0, 0.0, dt2 * stdDevA2, 0.0, 0.0, 0.0,
		0.0, 0.5 * dt3 * stdDevA2, 0.0, 0.0, 0.0, dt2 * stdDevA2, 0.0, 0.0,
		0.0, 0.0, 0.5 * dt3 * stdDevA2, 0.0, 0.0, 0.0, dt2 * stdDevA2, 0.0,
		0.0, 0.0, 0.0, 0.5 * dt3 * stdDevA2, 0.0, 0.0, 0.0, dt2 * stdDevA2,
	})

	// Ref.: Eq.(63)
	R := mat.NewDense(4, 4, []float64{
		math.Pow(stdDevMCx, 2), 0.0, 0.0, 0.0,
		0.0, math.Pow(stdDevMCy, 2), 0.0, 0.0,
		0.0, 0.0, math.Pow(stdDevMW, 2), 0.0,
		0.0, 0.0, 0.0, math.Pow(stdDevMH, 2),
	})

	// Error covariance matrix P (identity)
	P := mat.NewDense(8, 8, []float64{
		1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
	})

	// State vector: cx, cy, w, h, vx, vy, vw, vh
	x := mat.NewDense(8, 1, []float64{
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
	})

	// Control input for cx, cy, w, h
	u := mat.NewDense(4, 1, []float64{
		uCx, uCy, uW, uH,
	})

	k := &KalmanBBox{
		dt:        dt,
		u:         u,
		stdDevA:   stdDevA,
		stdDevMCx: stdDevMCx,
		stdDevMCy: stdDevMCy,
		stdDevMW:  stdDevMW,
		stdDevMH:  stdDevMH,
		A:         A,
		B:         B,
		H:         H,
		Q:         Q,
		R:         R,
		P:         P,
		x:         x,
	}
	k.prealloc()
	for _, o := range options {
		o(k)
	}
	return k
}

// WithStateBBox sets custom initial state for state vector for BBox case
//
// Why is it needed to set the initial state to the actual first observed bounding box (sometimes)?
// When the first state vector is initialized with zeros, it assumes that the object is at the origin
// and the filter needs to estimate the position of the object from scratch, which can result in some initial inaccuracies.
// On the other hand, initializing the first state vector with the actual observed bounding box can provide
// a more accurate estimate from the beginning, which can improve the overall tracking performance of the filter
func WithStateBBox(cx, cy, w, h float64) func(*KalmanBBox) {
	return func(k *KalmanBBox) {
		k.x.Set(0, 0, cx)
		k.x.Set(1, 0, cy)
		k.x.Set(2, 0, w)
		k.x.Set(3, 0, h)
	}
}

// prealloc does preparations to reduce allocs
func (k *KalmanBBox) prealloc() {
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

	z := mat.NewDense(4, 1, []float64{0, 0, 0, 0})

	r := mat.NewDense(hrows, xcols, nil)

	gainrows, _ := gain.Dims()
	_, rcols := r.Dims()
	gain_r := mat.NewDense(gainrows, rcols, nil)

	gain_h := mat.NewDense(gainrows, hcols, nil)

	identityrows, identitycols := identityBBox.Dims()
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
// SetDt rebuilds the time-dependent matrices for a new cycle time.
//
// See Kalman2D.SetDt for the reasoning: a discrete filter is only valid for the
// sampling interval its A and Q were built for, so an uneven frame arrival has
// to be fed in here before Predict.
func (k *KalmanBBox) SetDt(dt float64) {
	if dt <= 0.0 || dt == k.dt {
		return
	}
	k.dt = dt
	dt2 := dt * dt
	dt3 := dt2 * dt
	dt4 := dt3 * dt
	stdDevA2 := k.stdDevA * k.stdDevA

	// Ref.: Eq.(53)
	k.A.SetRawMatrix(mat.NewDense(8, 8, []float64{
		1.0, 0.0, 0.0, 0.0, dt, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0, 0.0, dt, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0, 0.0, 0.0, dt, 0.0,
		0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, dt,
		0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
	}).RawMatrix())

	// Ref.: Eq.(54)
	k.B.SetRawMatrix(mat.NewDense(8, 4, []float64{
		0.5 * dt2, 0.0, 0.0, 0.0,
		0.0, 0.5 * dt2, 0.0, 0.0,
		0.0, 0.0, 0.5 * dt2, 0.0,
		0.0, 0.0, 0.0, 0.5 * dt2,
		dt, 0.0, 0.0, 0.0,
		0.0, dt, 0.0, 0.0,
		0.0, 0.0, dt, 0.0,
		0.0, 0.0, 0.0, dt,
	}).RawMatrix())

	// Ref.: Eq.(62)
	k.Q.SetRawMatrix(mat.NewDense(8, 8, []float64{
		0.25 * dt4 * stdDevA2, 0.0, 0.0, 0.0, 0.5 * dt3 * stdDevA2, 0.0, 0.0, 0.0,
		0.0, 0.25 * dt4 * stdDevA2, 0.0, 0.0, 0.0, 0.5 * dt3 * stdDevA2, 0.0, 0.0,
		0.0, 0.0, 0.25 * dt4 * stdDevA2, 0.0, 0.0, 0.0, 0.5 * dt3 * stdDevA2, 0.0,
		0.0, 0.0, 0.0, 0.25 * dt4 * stdDevA2, 0.0, 0.0, 0.0, 0.5 * dt3 * stdDevA2,
		0.5 * dt3 * stdDevA2, 0.0, 0.0, 0.0, dt2 * stdDevA2, 0.0, 0.0, 0.0,
		0.0, 0.5 * dt3 * stdDevA2, 0.0, 0.0, 0.0, dt2 * stdDevA2, 0.0, 0.0,
		0.0, 0.0, 0.5 * dt3 * stdDevA2, 0.0, 0.0, 0.0, dt2 * stdDevA2, 0.0,
		0.0, 0.0, 0.0, 0.5 * dt3 * stdDevA2, 0.0, 0.0, 0.0, dt2 * stdDevA2,
	}).RawMatrix())
}

// GetDt returns the cycle time the filter is currently built for
func (k *KalmanBBox) GetDt() float64 {
	return k.dt
}

func (k *KalmanBBox) Predict() {
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
func (k *KalmanBBox) Update(zCx, zCy, zW, zH float64) error {
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
	k.z.Set(0, 0, zCx)
	k.z.Set(1, 0, zCy)
	k.z.Set(2, 0, zW)
	k.z.Set(3, 0, zH)
	k.r.Mul(k.H, k.x)
	k.r.Sub(k.z, k.r)
	// Ref.: Eq.(9)
	k.gain_r.Mul(k.gain, k.r)
	k.x.Add(k.x, k.gain_r)
	// Ref.: Eq.(10)
	k.gain_h.Mul(k.gain, k.H)
	k.newp.Sub(identityBBox, k.gain_h)
	k.P.Mul(k.newp, k.P)
	return nil
}

// GetState returns the current state (only cx, cy, w, h - not velocities).
func (k *KalmanBBox) GetState() (float64, float64, float64, float64) {
	return k.x.At(0, 0), k.x.At(1, 0), k.x.At(2, 0), k.x.At(3, 0)
}

// GetVelocity returns the current velocity (only vx, vy, vw, vh).
func (k *KalmanBBox) GetVelocity() (float64, float64, float64, float64) {
	return k.x.At(4, 0), k.x.At(5, 0), k.x.At(6, 0), k.x.At(7, 0)
}

// GetPredictedState returns prediction without mutating the state vector and the error covariance matrix
func (k *KalmanBBox) GetPredictedState() (float64, float64, float64, float64) {
	xPred := mat.NewDense(8, 1, nil)
	xPred.Mul(k.A, k.x)
	buTmp := mat.NewDense(8, 1, nil)
	buTmp.Mul(k.B, k.u)
	xPred.Add(xPred, buTmp)
	return xPred.At(0, 0), xPred.At(1, 0), xPred.At(2, 0), xPred.At(3, 0)
}

// GetPositionUncertainty returns position uncertainty from P matrix (for center coordinates)
func (k *KalmanBBox) GetPositionUncertainty() float64 {
	return math.Sqrt(math.Pow(k.P.At(0, 0), 2) + math.Pow(k.P.At(1, 1), 2))
}

// GetVectorState returns the copy of current state (full 8-element vector).
func (k *KalmanBBox) GetVectorState() *mat.Dense {
	x := mat.NewDense(8, 1, []float64{
		k.x.At(0, 0),
		k.x.At(1, 0),
		k.x.At(2, 0),
		k.x.At(3, 0),
		k.x.At(4, 0),
		k.x.At(5, 0),
		k.x.At(6, 0),
		k.x.At(7, 0),
	})
	return x
}

// MahalanobisDistanceSquared computes the squared Mahalanobis distance between measurement and predicted state
// Ref.: Eq.(66)
//
// This is useful for data association in multi-object tracking.
// A detection passes the gate if MahalanobisDistanceSquared < threshold
// Common threshold for 4 DOF at 95% confidence: 9.488
func (k *KalmanBBox) MahalanobisDistanceSquared(zCx, zCy, zW, zH float64) (float64, error) {
	// Ref.: Eq.(64) - Innovation (residual)
	z := mat.NewDense(4, 1, []float64{zCx, zCy, zW, zH})
	hx := mat.NewDense(4, 1, nil)
	hx.Mul(k.H, k.x)
	y := mat.NewDense(4, 1, nil)
	y.Sub(z, hx)

	// Ref.: Eq.(65) - Innovation covariance
	S := mat.NewDense(4, 4, nil)
	hp := mat.NewDense(4, 8, nil)
	hp.Mul(k.H, k.P)
	S.Mul(hp, k.H.T())
	S.Add(S, k.R)

	// Ref.: Eq.(66) - Squared Mahalanobis distance
	SInv := mat.NewDense(4, 4, nil)
	err := SInv.Inverse(S)
	if err != nil {
		return 0, errors.Wrap(err, "Can't inverse matrix")
	}

	// d^2 = y^T * S^-1 * y
	yT := y.T()
	tmp := mat.NewDense(1, 4, nil)
	tmp.Mul(yT, SInv)
	result := mat.NewDense(1, 1, nil)
	result.Mul(tmp, y)

	return result.At(0, 0), nil
}

// MahalanobisDistance computes the Mahalanobis distance (square root of squared distance)
// Ref.: Eq.(67)
func (k *KalmanBBox) MahalanobisDistance(zCx, zCy, zW, zH float64) (float64, error) {
	dSquared, err := k.MahalanobisDistanceSquared(zCx, zCy, zW, zH)
	if err != nil {
		return 0, err
	}
	return math.Sqrt(dSquared), nil
}

// GetInnovationCovariance returns the innovation covariance matrix S
// Ref.: Eq.(65)
//
// S = H * P * H^T + R
func (k *KalmanBBox) GetInnovationCovariance() *mat.Dense {
	S := mat.NewDense(4, 4, nil)
	hp := mat.NewDense(4, 8, nil)
	hp.Mul(k.H, k.P)
	S.Mul(hp, k.H.T())
	S.Add(S, k.R)
	return S
}

// GatingCheck checks if a measurement passes the gating threshold
// Ref.: Eq.(70)
//
// Common thresholds for alpha = 0.95:
// - 4 DOF (bounding box): 9.488
// - 2 DOF (position only): 5.991
func (k *KalmanBBox) GatingCheck(zCx, zCy, zW, zH, threshold float64) bool {
	dSq, err := k.MahalanobisDistanceSquared(zCx, zCy, zW, zH)
	if err != nil {
		return false
	}
	return dSq < threshold
}
