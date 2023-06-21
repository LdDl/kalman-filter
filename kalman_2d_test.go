package kalman_filter

import (
	"testing"
)

func TestKalman2D(t *testing.T) {
	dt := 0.04 // 1/25 = 25 fps - just an example
	ux := 1.0
	uy := 1.0
	stdDevA := 2.0
	stdDevMx := 0.1
	stdDevMy := 0.1

	// Sample measurements
	// Note: in this example Y-axis going from up to down
	xs := []float64{311, 312, 313, 311, 311, 312, 312, 313, 312, 312, 312, 312, 312, 312, 312, 312, 312, 312, 311, 311, 311, 311, 311, 310, 311, 311, 311, 310, 310, 308, 307, 308, 308, 308, 307, 307, 307, 308, 307, 307, 307, 307, 307, 308, 307, 309, 306, 307, 306, 307, 308, 306, 306, 306, 305, 307, 307, 307, 306, 306, 306, 307, 307, 308, 307, 307, 308, 307, 306, 308, 309, 309, 309, 309, 308, 309, 309, 309, 308, 311, 311, 307, 311, 307, 313, 311, 307, 311, 311, 306, 312, 312, 312, 312, 312, 312, 312, 312, 312, 312, 312, 312, 312, 312, 312, 312, 312, 312, 312, 312, 312, 312}
	ys := []float64{5, 6, 8, 10, 11, 12, 12, 13, 16, 16, 18, 18, 19, 19, 20, 20, 22, 22, 23, 23, 24, 24, 28, 30, 32, 35, 39, 42, 44, 46, 56, 58, 70, 60, 52, 64, 51, 70, 70, 70, 66, 83, 80, 85, 80, 98, 79, 98, 61, 94, 101, 94, 104, 94, 107, 112, 108, 108, 109, 109, 121, 108, 108, 120, 122, 122, 128, 130, 122, 140, 122, 122, 140, 122, 134, 141, 136, 136, 154, 155, 155, 150, 161, 162, 169, 171, 181, 175, 175, 163, 178, 178, 178, 178, 178, 178, 178, 178, 178, 178, 178, 178, 178, 178, 178, 178, 178, 178, 178, 178, 178, 178}

	// Assume that initial X,Y coordinates match the first measurement
	ix := xs[0] // Initial state for X
	iy := ys[0] // Initial state for Y

	kalman := NewKalman2D(dt, ux, uy, stdDevA, stdDevMx, stdDevMy, WithState2D(ix, iy))

	predictions := make([][]float64, 0, len(xs))
	updatedStates := make([][]float64, 0, len(xs))
	for i := 0; i < len(xs); i++ {
		// Considering that the measurements are noisy
		mx := xs[i]
		my := ys[i]

		// Predict stage
		kalman.Predict()
		state := kalman.GetVectorState()
		predictions = append(predictions, []float64{state.At(0, 0), state.At(1, 0)})

		// Update stage
		err := kalman.Update(mx, my)
		if err != nil {
			t.Error(err)
			return
		}
		updatedState := kalman.GetVectorState()
		updatedStates = append(updatedStates, []float64{updatedState.At(0, 0), updatedState.At(1, 0)})
	}
	// fmt.Println("measurement X;measurement Y;prediction X;prediction Y;updated X;updated Y")
	// for i := 0; i < len(xs); i++ {
	// 	fmt.Printf("%f;%f;%f;%f;%f;%f\n", xs[i], ys[i], predictions[i][0], predictions[i][1], updatedStates[i][0], updatedStates[i][1])
	// }
}
