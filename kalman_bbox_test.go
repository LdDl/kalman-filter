package kalman_filter

import (
	"encoding/csv"
	"fmt"
	"math"
	"os"
	"testing"
)

func TestKalmanBBox(t *testing.T) {
	dt := 0.04 // 1/25 = 25 fps - just an example
	uCx := 1.0
	uCy := 1.0
	uW := 0.0 // No control input for size
	uH := 0.0
	stdDevA := 2.0
	stdDevMCx := 0.1
	stdDevMCy := 0.1
	stdDevMW := 0.1
	stdDevMH := 0.1

	// Sample measurements for center coordinates (same as TestKalman2D)
	// Note: in this example Y-axis going from up to down
	xs := []float64{
		311, 312, 313, 311, 311, 312, 312, 313, 312, 312, 312, 312, 312, 312, 312, 312, 312,
		312, 311, 311, 311, 311, 311, 310, 311, 311, 311, 310, 310, 308, 307, 308, 308, 308,
		307, 307, 307, 308, 307, 307, 307, 307, 307, 308, 307, 309, 306, 307, 306, 307, 308,
		306, 306, 306, 305, 307, 307, 307, 306, 306, 306, 307, 307, 308, 307, 307, 308, 307,
		306, 308, 309, 309, 309, 309, 308, 309, 309, 309, 308, 311, 311, 307, 311, 307, 313,
		311, 307, 311, 311, 306, 312, 312, 312, 312, 312, 312, 312, 312, 312, 312, 312, 312,
		312, 312, 312, 312, 312, 312, 312, 312, 312, 312,
	}
	ys := []float64{
		5, 6, 8, 10, 11, 12, 12, 13, 16, 16, 18, 18, 19, 19, 20, 20, 22, 22, 23, 23, 24, 24,
		28, 30, 32, 35, 39, 42, 44, 46, 56, 58, 70, 60, 52, 64, 51, 70, 70, 70, 66, 83, 80, 85,
		80, 98, 79, 98, 61, 94, 101, 94, 104, 94, 107, 112, 108, 108, 109, 109, 121, 108, 108,
		120, 122, 122, 128, 130, 122, 140, 122, 122, 140, 122, 134, 141, 136, 136, 154, 155,
		155, 150, 161, 162, 169, 171, 181, 175, 175, 163, 178, 178, 178, 178, 178, 178, 178,
		178, 178, 178, 178, 178, 178, 178, 178, 178, 178, 178, 178, 178, 178, 178,
	}

	// Generate bounding boxes around centroids
	// Base width/height with some variation
	baseW := 40.0
	baseH := 80.0

	// Generate width/height with slight variations (simulating object size changes)
	ws := make([]float64, len(xs))
	hs := make([]float64, len(ys))
	for i := range xs {
		// Slight oscillation
		ws[i] = baseW + math.Sin(float64(i)*0.1)*5.0
		hs[i] = baseH + math.Sin(float64(i)*0.15)*8.0
	}

	// Assume that initial state matches the first measurement
	iCx := xs[0]
	iCy := ys[0]
	iW := ws[0]
	iH := hs[0]

	kalman := NewKalmanBBox(dt, uCx, uCy, uW, uH, stdDevA, stdDevMCx, stdDevMCy, stdDevMW, stdDevMH, WithStateBBox(iCx, iCy, iW, iH))

	predictions := make([][4]float64, 0, len(xs))
	updatedStates := make([][4]float64, 0, len(xs))

	for i := 0; i < len(xs); i++ {
		mCx := xs[i]
		mCy := ys[i]
		mW := ws[i]
		mH := hs[i]

		// Predict stage
		kalman.Predict()
		pCx, pCy, pW, pH := kalman.GetState()
		predictions = append(predictions, [4]float64{pCx, pCy, pW, pH})

		// Update stage
		err := kalman.Update(mCx, mCy, mW, mH)
		if err != nil {
			t.Error(err)
			return
		}
		uCx, uCy, uW, uH := kalman.GetState()
		updatedStates = append(updatedStates, [4]float64{uCx, uCy, uW, uH})
	}

	// Verify that predictions and updates are reasonable
	if len(predictions) != len(xs) {
		t.Errorf("Expected %d predictions, got %d", len(xs), len(predictions))
	}
	if len(updatedStates) != len(xs) {
		t.Errorf("Expected %d updated states, got %d", len(xs), len(updatedStates))
	}

	// Check that final state is close to final measurement
	finalState := updatedStates[len(updatedStates)-1]
	lastMCx := xs[len(xs)-1]
	lastMCy := ys[len(ys)-1]
	lastMW := ws[len(ws)-1]
	lastMH := hs[len(hs)-1]

	if math.Abs(finalState[0]-lastMCx) >= 5.0 {
		t.Errorf("Final cx too far from measurement: got %f, expected %f", finalState[0], lastMCx)
	}
	if math.Abs(finalState[1]-lastMCy) >= 5.0 {
		t.Errorf("Final cy too far from measurement: got %f, expected %f", finalState[1], lastMCy)
	}
	if math.Abs(finalState[2]-lastMW) >= 5.0 {
		t.Errorf("Final w too far from measurement: got %f, expected %f", finalState[2], lastMW)
	}
	if math.Abs(finalState[3]-lastMH) >= 5.0 {
		t.Errorf("Final h too far from measurement: got %f, expected %f", finalState[3], lastMH)
	}

	// Write results to CSV
	file, err := os.Create("./data/kalman-bbox.csv")
	if err != nil {
		t.Error(err)
		return
	}
	defer file.Close()

	writer := csv.NewWriter(file)
	defer writer.Flush()
	writer.Comma = ';'

	err = writer.Write([]string{
		"measurement cx", "measurement cy", "measurement w", "measurement h",
		"prediction cx", "prediction cy", "prediction w", "prediction h",
		"updated cx", "updated cy", "updated w", "updated h",
	})
	if err != nil {
		t.Error(err)
		return
	}
	for i := 0; i < len(xs); i++ {
		err = writer.Write([]string{
			fmt.Sprintf("%f", xs[i]),
			fmt.Sprintf("%f", ys[i]),
			fmt.Sprintf("%f", ws[i]),
			fmt.Sprintf("%f", hs[i]),
			fmt.Sprintf("%f", predictions[i][0]),
			fmt.Sprintf("%f", predictions[i][1]),
			fmt.Sprintf("%f", predictions[i][2]),
			fmt.Sprintf("%f", predictions[i][3]),
			fmt.Sprintf("%f", updatedStates[i][0]),
			fmt.Sprintf("%f", updatedStates[i][1]),
			fmt.Sprintf("%f", updatedStates[i][2]),
			fmt.Sprintf("%f", updatedStates[i][3]),
		})
		if err != nil {
			t.Error(err)
			return
		}
	}
}

func TestMahalanobisDistance(t *testing.T) {
	dt := 0.04
	kalman := NewKalmanBBox(
		dt, 1.0, 1.0, 0.0, 0.0,
		2.0, 0.1, 0.1, 0.1, 0.1,
		WithStateBBox(100.0, 50.0, 40.0, 80.0),
	)

	// After prediction, covariance increases
	kalman.Predict()

	// Measurement close to predicted state should have small Mahalanobis distance
	dClose, err := kalman.MahalanobisDistanceSquared(100.5, 50.5, 40.0, 80.0)
	if err != nil {
		t.Error(err)
		return
	}

	// Measurement far from predicted state should have large Mahalanobis distance
	dFar, err := kalman.MahalanobisDistanceSquared(200.0, 150.0, 60.0, 120.0)
	if err != nil {
		t.Error(err)
		return
	}

	if dClose >= dFar {
		t.Errorf("Close measurement should have smaller distance: dClose=%f, dFar=%f", dClose, dFar)
	}

	// Check gating
	threshold := 9.488 // 95% confidence for 4 DOF
	if !kalman.GatingCheck(100.5, 50.5, 40.0, 80.0, threshold) {
		t.Error("Close measurement should pass gate")
	}
}

func TestVelocityTracking(t *testing.T) {
	dt := 0.04
	kalman := NewKalmanBBox(
		dt, 0.0, 0.0, 0.0, 0.0, // No control input
		2.0, 0.1, 0.1, 0.1, 0.1,
		WithStateBBox(100.0, 50.0, 40.0, 80.0),
	)

	// Simulate object moving right and down with growing size
	for i := 1; i < 20; i++ {
		cx := 100.0 + float64(i)*2.0  // Moving right
		cy := 50.0 + float64(i)*1.5   // Moving down
		w := 40.0 + float64(i)*0.5    // Growing width
		h := 80.0 + float64(i)*0.3    // Growing height

		kalman.Predict()
		err := kalman.Update(cx, cy, w, h)
		if err != nil {
			t.Error(err)
			return
		}
	}

	// After tracking, velocity should reflect the motion
	vx, vy, vw, vh := kalman.GetVelocity()

	// Velocity should be positive (moving right/down, growing)
	if vx <= 0.0 {
		t.Errorf("Velocity X should be positive, got %f", vx)
	}
	if vy <= 0.0 {
		t.Errorf("Velocity Y should be positive, got %f", vy)
	}
	if vw <= 0.0 {
		t.Errorf("Velocity W should be positive, got %f", vw)
	}
	if vh <= 0.0 {
		t.Errorf("Velocity H should be positive, got %f", vh)
	}
}

func BenchmarkKalmanBBox(b *testing.B) {
	dt := 0.04
	uCx := 1.0
	uCy := 1.0
	uW := 0.0
	uH := 0.0
	stdDevA := 2.0
	stdDevMCx := 0.1
	stdDevMCy := 0.1
	stdDevMW := 0.1
	stdDevMH := 0.1

	kalman := NewKalmanBBox(dt, uCx, uCy, uW, uH, stdDevA, stdDevMCx, stdDevMCy, stdDevMW, stdDevMH, WithStateBBox(100.0, 50.0, 40.0, 80.0))

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		cx := 100.0 + float64(i%100)*0.1
		cy := 50.0 + float64(i%100)*0.1
		w := 40.0 + float64(i%50)*0.01
		h := 80.0 + float64(i%50)*0.01

		kalman.Predict()
		err := kalman.Update(cx, cy, w, h)
		if err != nil {
			b.Error(err)
			return
		}
	}
}
