package kalman_filter

import (
	"math/rand"
	"testing"
)

func TestKalman1D(t *testing.T) {
	rand.Seed(1337)

	// Just and adoptation of https://machinelearningspace.com/object-tracking-python/
	dt := 0.1
	u := 2.0
	stdDevA := 0.25
	stdDevM := 1.2

	n := 100
	iters := int(float64(n) / dt)
	track := make([]struct {
		t float64
		x float64
	}, iters)
	v := 0.0
	for i := 0; i < iters; i++ {
		track[i] = struct {
			t float64
			x float64
		}{
			t: v,
			x: dt * (v*v - v),
		}
		v += dt
	}
	kalman := NewKalman1D(dt, u, stdDevA, stdDevM)

	measurements := make([]float64, 0, iters)
	predictions := make([]float64, 0, iters)
	for _, val := range track {
		// tm := val.t
		x := val.x

		// Add some noise to perfect track
		noise := rand.Float64()*100 - 50
		z := kalman.H.At(0, 0)*x + noise
		measurements = append(measurements, z)

		// Predict stage
		kalman.Predict()
		state := kalman.GetVectorState()
		predictions = append(predictions, state.At(0, 0))

		// Update stage
		err := kalman.Update(z)
		if err != nil {
			t.Error(err)
			return
		}
	}
	// fmt.Println("time;perfect;measurement;prediction")
	// for i := 0; i < len(track); i++ {
	// 	fmt.Printf("%f;%f;%f;%f\n", track[i].t, track[i].x, measurements[i], predictions[i])
	// }
}

func BenchmarkKalman1D(b *testing.B) {
	rand.Seed(1337)

	// Just and adoptation of https://machinelearningspace.com/object-tracking-python/
	dt := 0.1
	u := 2.0
	stdDevA := 0.25
	stdDevM := 1.2

	n := 1000
	iters := int(float64(n) / dt)
	track := make([]struct {
		t float64
		x float64
	}, iters)
	v := 0.0
	for i := 0; i < iters; i++ {
		track[i] = struct {
			t float64
			x float64
		}{
			t: v,
			x: dt * (v*v - v),
		}
		v += dt
	}
	kalman := NewKalman1D(dt, u, stdDevA, stdDevM)

	b.ResetTimer()

	for i := 0; i < b.N; i++ {
		for _, val := range track {
			// tm := val.t
			x := val.x

			// Add some noise to perfect track
			noise := rand.Float64()*100 - 50
			z := kalman.H.At(0, 0)*x + noise

			// Predict stage
			kalman.Predict()

			// Update stage
			err := kalman.Update(z)
			if err != nil {
				b.Error(err)
				return
			}
		}
	}
}
