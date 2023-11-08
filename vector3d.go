package main

import (
	"math"

	"github.com/paulmach/orb"
)

type Vector3D struct {
	X, Y, Z float64
}

// crossProduct berechnet das Kreuzprodukt axb zweier Vektoren
func (a Vector3D) crossProduct(b Vector3D) Vector3D {
	return Vector3D{a.Y*b.Z - a.Z*b.Y, a.Z*b.X - a.X*b.Z, a.X*b.Y - a.Y*b.X}.Normalize()

}

// len berechnet ist Länge (Betrag/Norm) eines Vektors
func (a Vector3D) len() float64 {
	return math.Sqrt(a.X*a.X + a.Y*a.Y + a.Z*a.Z)
}

func (a Vector3D) Normalize() Vector3D {
	mag := math.Sqrt(a.X*a.X + a.Y*a.Y + a.Z*a.Z)
	if mag < math.Pow10(-14) { //Nullvektoren können nicht normalisiert werden
		return Vector3D{0, 0, 0}
	}
	return Vector3D{a.X / mag, a.Y / mag, a.Z / mag}
}

func (a Vector3D) sub(b Vector3D) Vector3D {
	return Vector3D{a.X - b.X, a.Y - b.Y, a.Z - b.Z}
}

func (a Vector3D) times(b Vector3D) float64 {
	return a.X*b.X + a.Y*b.Y + a.Z*b.Z
}

func (a Vector3D) ToSpherical() orb.Point {
	return orb.Point{math.Atan2(a.Y, a.X) * 180 / (math.Pi), math.Atan2(a.Z, math.Sqrt(a.X*a.X+a.Y*a.Y)) * 180 / (math.Pi)}
}
