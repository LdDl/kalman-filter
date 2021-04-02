package kalman_filter

import (
	"image"
	"image/color"
	"image/png"
	"math/rand"
	"os"
	"testing"

	"gonum.org/v1/gonum/mat"
)

// TestNoNoisePointTracker Test tracker assuming initial data already has noise due its nature
func TestNoNoisePointTracker(t *testing.T) {
	// Make result
	rand.Seed(1337)

	tracker := NewPointTracker()
	dt := 1.0

	/* Prepare image for better result representations */
	width := 640
	height := 360
	upLeft := image.Point{0, 0}
	lowRight := image.Point{width, height}
	img := image.NewRGBA(image.Rectangle{upLeft, lowRight})
	for x := 0; x < width; x++ {
		for y := 0; y < height; y++ {
			img.Set(x, y, color.Black)
		}
	}

	// Set x, y, vx, vy
	tracker.SetStateValue(pointTrackerTestData[0][0], pointTrackerTestData[0][1], 0, 0)

	for tm := 1; tm < len(pointTrackerTestData); tm++ {
		xt := pointTrackerTestData[tm][0]
		yt := pointTrackerTestData[tm][1]

		y := mat.NewDense(2, 1, []float64{
			xt,
			yt,
		})
		tracker.SetTime(dt)

		u := mat.NewDense(4, 1, []float64{
			0.0,
			0.0,
			0.0,
			0.0,
		})
		state, err := tracker.Process(u, y)
		if err != nil {
			t.Error(err)
			return
		}
		kalmanX, kalmanY := int(state.At(0, 0)), int(state.At(1, 0))

		// Index [tm-1] in validating data since we are iterating from tm=1
		correctKalmanX, correctkalmanY := int(pointTrackerValidateNoNoiseData[tm-1][0]), int(pointTrackerValidateNoNoiseData[tm-1][1])
		if kalmanX != correctKalmanX {
			t.Errorf("Step: %d. Filtered X-value should be %d, but got %d", tm-1, correctKalmanX, kalmanX)
		}
		if kalmanY != correctkalmanY {
			t.Errorf("Step: %d. Filtered Y-value should be %d, but got %d", tm-1, correctkalmanY, kalmanY)
		}
		img.Set(int(xt), int(yt), colorRed)
		img.Set(kalmanX, kalmanY, colorBlue)
	}

	// Encode as PNG.
	f, err := os.Create("point-tracker-test-no-noise.png")
	if err != nil {
		t.Error(err)
	}
	err = png.Encode(f, img)
	if err != nil {
		t.Error(err)
	}
}

// TestNoisedPointTracker Test tracker assuming initial data already has NOT noise due its nature (so we add noise)
func TestNoisedPointTracker(t *testing.T) {
	// Make random noise to be persistent
	rand.Seed(1337)

	tracker := NewPointTracker()
	dt := 1.0

	/* Prepare image for better result representations */
	width := 640
	height := 360
	upLeft := image.Point{0, 0}
	lowRight := image.Point{width, height}
	img := image.NewRGBA(image.Rectangle{upLeft, lowRight})
	for x := 0; x < width; x++ {
		for y := 0; y < height; y++ {
			img.Set(x, y, color.Black)
		}
	}

	// Set x, y, vx, vy
	tracker.SetStateValue(pointTrackerTestData[0][0], pointTrackerTestData[0][1], 0, 0)

	for tm := 1; tm < len(pointTrackerTestData); tm++ {
		xt := pointTrackerTestData[tm][0]
		yt := pointTrackerTestData[tm][1]

		noisyXt := xt + rand.Float64()*10
		noisyYt := yt + rand.Float64()*10
		y := mat.NewDense(2, 1, []float64{
			noisyXt,
			noisyYt,
		})

		tracker.SetTime(dt)

		u := mat.NewDense(4, 1, []float64{
			0.0,
			0.0,
			0.0,
			0.0,
		})
		state, err := tracker.Process(u, y)
		if err != nil {
			t.Error(err)
			return
		}

		kalmanX, kalmanY := int(state.At(0, 0)), int(state.At(1, 0))

		// Index [tm-1] in validating data since we are iterating from tm=1
		correctKalmanX, correctkalmanY := int(pointTrackerValidateNoisedData[tm-1][0]), int(pointTrackerValidateNoisedData[tm-1][1])
		if kalmanX != correctKalmanX {
			t.Errorf("Step: %d. Filtered X-value should be %d, but got %d", tm-1, correctKalmanX, kalmanX)
		}
		if kalmanY != correctkalmanY {
			t.Errorf("Step: %d. Filtered Y-value should be %d, but got %d", tm-1, correctkalmanY, kalmanY)
		}
		img.Set(int(noisyXt), int(noisyYt), colorRed)
		img.Set(kalmanX, kalmanY, colorBlue)
	}

	// Encode as PNG.
	f, err := os.Create("point-tracker-test-noised.png")
	if err != nil {
		t.Error(err)
	}
	err = png.Encode(f, img)
	if err != nil {
		t.Error(err)
	}
}

var (
	colorRed  = color.RGBA{255, 0, 0, 255.0}
	colorBlue = color.RGBA{0, 0, 255, 255.0}

	pointTrackerTestData = [][]float64{
		[]float64{311, 5},
		[]float64{312, 6},
		[]float64{313, 8},
		[]float64{311, 10},
		[]float64{311, 11},
		[]float64{312, 12},
		[]float64{312, 12},
		[]float64{313, 13},
		[]float64{312, 16},
		[]float64{312, 16},
		[]float64{312, 18},
		[]float64{312, 18},
		[]float64{312, 19},
		[]float64{312, 19},
		[]float64{312, 20},
		[]float64{312, 20},
		[]float64{312, 22},
		[]float64{312, 22},
		[]float64{311, 23},
		[]float64{311, 23},
		[]float64{311, 24},
		[]float64{311, 24},
		[]float64{311, 28},
		[]float64{310, 30},
		[]float64{311, 32},
		[]float64{311, 35},
		[]float64{311, 39},
		[]float64{310, 42},
		[]float64{310, 44},
		[]float64{308, 46},
		[]float64{307, 56},
		[]float64{308, 58},
		[]float64{308, 70},
		[]float64{308, 60},
		[]float64{307, 52},
		[]float64{307, 64},
		[]float64{307, 51},
		[]float64{308, 70},
		[]float64{307, 70},
		[]float64{307, 70},
		[]float64{307, 66},
		[]float64{307, 83},
		[]float64{307, 80},
		[]float64{308, 85},
		[]float64{307, 80},
		[]float64{309, 98},
		[]float64{306, 79},
		[]float64{307, 98},
		[]float64{306, 61},
		[]float64{307, 94},
		[]float64{308, 101},
		[]float64{306, 94},
		[]float64{306, 104},
		[]float64{306, 94},
		[]float64{305, 107},
		[]float64{307, 112},
		[]float64{307, 108},
		[]float64{307, 108},
		[]float64{306, 109},
		[]float64{306, 109},
		[]float64{306, 121},
		[]float64{307, 108},
		[]float64{307, 108},
		[]float64{308, 120},
		[]float64{307, 122},
		[]float64{307, 122},
		[]float64{308, 128},
		[]float64{307, 130},
		[]float64{306, 122},
		[]float64{308, 140},
		[]float64{309, 122},
		[]float64{309, 122},
		[]float64{309, 140},
		[]float64{309, 122},
		[]float64{308, 134},
		[]float64{309, 141},
		[]float64{309, 136},
		[]float64{309, 136},
		[]float64{308, 154},
		[]float64{311, 155},
		[]float64{311, 155},
		[]float64{307, 150},
		[]float64{311, 161},
		[]float64{307, 162},
		[]float64{313, 169},
		[]float64{311, 171},
		[]float64{307, 181},
		[]float64{311, 175},
		[]float64{311, 175},
		[]float64{306, 163},
		[]float64{312, 178},
		[]float64{312, 178},
		[]float64{312, 178},
		[]float64{312, 178},
		[]float64{312, 178},
		[]float64{312, 178},
		[]float64{312, 178},
		[]float64{312, 178},
		[]float64{312, 178},
		[]float64{312, 178},
		[]float64{312, 178},
		[]float64{312, 178},
		[]float64{312, 178},
		[]float64{312, 178},
		[]float64{312, 178},
		[]float64{312, 178},
		[]float64{312, 178},
		[]float64{312, 178},
		[]float64{312, 178},
		[]float64{312, 178},
		[]float64{312, 178},
		[]float64{312, 178},
	}

	pointTrackerValidateNoNoiseData = [][]float64{
		[]float64{311, 5},
		[]float64{312, 7},
		[]float64{311, 9},
		[]float64{311, 11},
		[]float64{311, 12},
		[]float64{311, 12},
		[]float64{312, 13},
		[]float64{312, 15},
		[]float64{312, 16},
		[]float64{312, 17},
		[]float64{312, 18},
		[]float64{312, 19},
		[]float64{312, 20},
		[]float64{312, 20},
		[]float64{312, 21},
		[]float64{312, 22},
		[]float64{312, 23},
		[]float64{311, 23},
		[]float64{311, 24},
		[]float64{311, 25},
		[]float64{311, 25},
		[]float64{311, 26},
		[]float64{311, 28},
		[]float64{310, 29},
		[]float64{310, 31},
		[]float64{310, 33},
		[]float64{310, 35},
		[]float64{310, 37},
		[]float64{310, 40},
		[]float64{309, 43},
		[]float64{309, 46},
		[]float64{308, 51},
		[]float64{308, 54},
		[]float64{308, 55},
		[]float64{307, 58},
		[]float64{307, 58},
		[]float64{307, 61},
		[]float64{307, 64},
		[]float64{307, 67},
		[]float64{306, 68},
		[]float64{306, 72},
		[]float64{306, 75},
		[]float64{306, 78},
		[]float64{306, 80},
		[]float64{306, 84},
		[]float64{306, 85},
		[]float64{306, 89},
		[]float64{306, 87},
		[]float64{306, 90},
		[]float64{306, 93},
		[]float64{306, 95},
		[]float64{306, 98},
		[]float64{305, 99},
		[]float64{305, 102},
		[]float64{305, 105},
		[]float64{305, 108},
		[]float64{305, 110},
		[]float64{305, 111},
		[]float64{305, 113},
		[]float64{305, 116},
		[]float64{305, 117},
		[]float64{305, 117},
		[]float64{306, 119},
		[]float64{306, 121},
		[]float64{306, 123},
		[]float64{306, 125},
		[]float64{306, 128},
		[]float64{306, 129},
		[]float64{306, 132},
		[]float64{306, 132},
		[]float64{307, 133},
		[]float64{307, 135},
		[]float64{307, 135},
		[]float64{307, 136},
		[]float64{308, 138},
		[]float64{308, 139},
		[]float64{308, 140},
		[]float64{308, 143},
		[]float64{308, 146},
		[]float64{309, 149},
		[]float64{309, 151},
		[]float64{309, 154},
		[]float64{309, 156},
		[]float64{309, 160},
		[]float64{310, 163},
		[]float64{309, 167},
		[]float64{310, 170},
		[]float64{310, 173},
		[]float64{309, 173},
		[]float64{310, 176},
		[]float64{310, 178},
		[]float64{310, 180},
		[]float64{311, 182},
		[]float64{311, 183},
		[]float64{311, 184},
		[]float64{311, 185},
		[]float64{311, 186},
		[]float64{312, 186},
		[]float64{312, 187},
		[]float64{312, 187},
		[]float64{312, 187},
		[]float64{312, 187},
		[]float64{312, 187},
		[]float64{312, 187},
		[]float64{312, 187},
		[]float64{312, 186},
		[]float64{312, 186},
		[]float64{312, 186},
		[]float64{312, 185},
		[]float64{312, 185},
		[]float64{312, 185},
	}

	pointTrackerValidateNoisedData = [][]float64{
		[]float64{317, 9},
		[]float64{317, 16},
		[]float64{319, 18},
		[]float64{318, 20},
		[]float64{319, 22},
		[]float64{318, 20},
		[]float64{317, 18},
		[]float64{317, 21},
		[]float64{318, 20},
		[]float64{319, 21},
		[]float64{317, 22},
		[]float64{317, 24},
		[]float64{317, 23},
		[]float64{316, 25},
		[]float64{317, 24},
		[]float64{316, 25},
		[]float64{317, 25},
		[]float64{316, 25},
		[]float64{316, 26},
		[]float64{316, 27},
		[]float64{316, 27},
		[]float64{316, 29},
		[]float64{316, 31},
		[]float64{316, 33},
		[]float64{315, 34},
		[]float64{315, 36},
		[]float64{315, 38},
		[]float64{315, 41},
		[]float64{314, 43},
		[]float64{314, 46},
		[]float64{314, 49},
		[]float64{314, 54},
		[]float64{314, 57},
		[]float64{314, 59},
		[]float64{313, 61},
		[]float64{312, 61},
		[]float64{312, 65},
		[]float64{312, 68},
		[]float64{312, 71},
		[]float64{311, 72},
		[]float64{311, 76},
		[]float64{310, 79},
		[]float64{311, 83},
		[]float64{311, 86},
		[]float64{311, 90},
		[]float64{311, 92},
		[]float64{311, 95},
		[]float64{311, 93},
		[]float64{311, 96},
		[]float64{311, 99},
		[]float64{311, 100},
		[]float64{311, 103},
		[]float64{311, 104},
		[]float64{310, 107},
		[]float64{310, 111},
		[]float64{311, 113},
		[]float64{310, 115},
		[]float64{311, 117},
		[]float64{310, 118},
		[]float64{310, 121},
		[]float64{310, 122},
		[]float64{311, 123},
		[]float64{311, 125},
		[]float64{311, 128},
		[]float64{312, 130},
		[]float64{312, 132},
		[]float64{313, 134},
		[]float64{313, 135},
		[]float64{313, 139},
		[]float64{313, 140},
		[]float64{314, 140},
		[]float64{314, 142},
		[]float64{314, 142},
		[]float64{314, 143},
		[]float64{314, 144},
		[]float64{314, 146},
		[]float64{315, 147},
		[]float64{314, 150},
		[]float64{315, 153},
		[]float64{316, 155},
		[]float64{316, 158},
		[]float64{316, 160},
		[]float64{315, 163},
		[]float64{315, 166},
		[]float64{315, 168},
		[]float64{314, 172},
		[]float64{314, 174},
		[]float64{314, 177},
		[]float64{314, 177},
		[]float64{315, 180},
		[]float64{315, 182},
		[]float64{316, 183},
		[]float64{316, 186},
		[]float64{315, 188},
		[]float64{316, 188},
		[]float64{317, 190},
		[]float64{317, 190},
		[]float64{317, 191},
		[]float64{317, 191},
		[]float64{317, 191},
		[]float64{317, 191},
		[]float64{317, 192},
		[]float64{317, 191},
		[]float64{317, 191},
		[]float64{316, 191},
		[]float64{316, 190},
		[]float64{316, 190},
		[]float64{316, 190},
		[]float64{316, 190},
		[]float64{316, 189},
		[]float64{316, 189},
	}
)
