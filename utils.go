package kalman_filter

import (
	"fmt"

	"gonum.org/v1/gonum/mat"
)

// PrintDense Pretty print for gonum.Dense
func PrintDense(d *mat.Dense) {
	for i := 0; i < d.RawMatrix().Rows; i++ {
		row := d.RawRowView(i)
		fmt.Println(row)
	}
}
