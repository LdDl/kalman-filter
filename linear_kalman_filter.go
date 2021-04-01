package kalman_filter

import (
	"github.com/pkg/errors"
	"gonum.org/v1/gonum/mat"
)

// KalmanFilterLinear Implementation of linear Kalman filter
// A - Transition State Matrix
// B - Control input
// C - Measure Matrix C
// P - State covariance
// Q - Process covariance
// R - Measurement covariance
// X - State (initial indeed)
type KalmanFilterLinear struct {
	A *mat.Dense
	B *mat.Dense
	C *mat.Dense
	P *mat.Dense
	Q *mat.Dense
	R *mat.Dense
	X *mat.Dense
}

// Step Do single step for two stages: prediction and updating
func (filter *KalmanFilterLinear) Step(u *mat.Dense, y *mat.Dense) (mat.Matrix, error) {
	filter.Predict(u)
	err := filter.Update(y)
	if err != nil {
		return filter.X, errors.Wrap(err, "Can't execute Step() due the error on updating stage")
	}
	return filter.X, nil
}

// Predict Prediction stage
func (filter *KalmanFilterLinear) Predict(u *mat.Dense) {
	// Evaluate x:
	// x = A ⋅ x + B ⋅ u
	ar, _ := filter.A.Dims()
	_, xc := filter.X.Dims()
	AX := mat.NewDense(ar, xc, nil)
	AX.Mul(filter.A, filter.X)
	br, _ := filter.B.Dims()
	_, uc := u.Dims()
	BU := mat.NewDense(br, uc, nil)
	BU.Mul(filter.B, u)
	filter.X.Add(AX, BU)
	// Evaluate state covariance as:
	// P = A ⋅ P ⋅ Transponse(A) + Q
	_, pc := filter.P.Dims()
	AP := mat.NewDense(ar, pc, nil)
	AP.Mul(filter.A, filter.P)
	AP.Mul(AP, filter.A.T())
	filter.P.Add(AP, filter.Q)
}

// Update Updating stage
func (filter *KalmanFilterLinear) Update(y *mat.Dense) error {
	// Temporary result of
	// P ⋅ Transponse(C)
	Prows, _ := filter.P.Dims()
	Crows, Ccols := filter.C.Dims()
	tmpPC := mat.NewDense(Prows, Crows, nil) // using Cr since matrix C would be transponsed
	tmpPC.Mul(filter.P, filter.C.T())

	// K = tmpPC ⋅ [((C ⋅ tmpPC)  + R)^-1]
	// p.s. "^-1" - stands for inverse matrix
	tmpPC_rows, tmpPC_cols := tmpPC.Dims()
	tmpInversed := mat.NewDense(Crows, tmpPC_cols, nil)
	tmpInversed.Mul(filter.C, tmpPC)
	tmpInversed.Add(tmpInversed, filter.R)
	err := tmpInversed.Inverse(tmpInversed)
	if err != nil {
		return errors.Wrap(err, "Can't execute Update() due the error while gonum's Inverse() execution")
	}
	_, tmpInversed_cols := tmpInversed.Dims()
	K := mat.NewDense(tmpPC_rows, tmpInversed_cols, nil)
	K.Mul(tmpPC, tmpInversed)

	// Update state as:
	// x{k} = x{k-1} + K ⋅ (y - C ⋅ x{k-1})
	Krows, _ := K.Dims()
	_, Xcols := filter.X.Dims()
	CX := mat.NewDense(Crows, Xcols, nil)
	CX.Mul(filter.C, filter.X)
	yRows, yCols := y.Dims()
	yCx := mat.NewDense(yRows, yCols, nil)
	yCx.Sub(y, CX)
	KyCx := mat.NewDense(Krows, yCols, nil)
	KyCx.Mul(K, yCx)
	filter.X.Add(filter.X, KyCx)

	// Update state covariance as:
	// P{k} = (Diag(4, 1) - K ⋅ C) ⋅ P{k-1}
	diagonalFullDense := mat.NewDense(4, 4, nil)
	diagonalGonum := mat.NewDiagDense(4, []float64{1, 1, 1, 1})
	KC := mat.NewDense(Krows, Ccols, nil)
	KC.Mul(K, filter.C)
	diagonalFullDense.Sub(diagonalGonum, KC)
	filter.P.Mul(diagonalFullDense, filter.P)

	return nil
}
