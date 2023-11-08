package main

import (
	"fmt"
	"math"
	"testing"

	"github.com/paulmach/orb"
)

func TestSphericalTo3D(t *testing.T) {
	p1 := orb.Point{0, 90}

	p1_3D := sphericalTo3D(p1)
	p1_expected := Vector3D{0, 0, 1}

	if p1_3D != p1_expected {
		t.Errorf("Incorrect p1. Got %v, expected %v", p1_3D, p1_expected)
	}

	p1 = orb.Point{45, 45}
	p1_3D = sphericalTo3D(p1)
	p1_expected = Vector3D{0.5, .5, 0.7071}
	if p1_3D != p1_expected {
		t.Errorf("Incorrect p1. Got %v, expected %v", p1_3D, p1_expected)
	}
}

func TestThreeDimToSperical(t *testing.T) {
	p1 := Vector3D{0, 0, 1}
	p1S := p1.ToSpherical()
	p1S_expected := orb.Point{0, 90}
	if p1S != p1S_expected {
		t.Errorf("Incorrect p1. Got %v, expect %v", p1S, p1S_expected)
	}
}

func TestGetMiddle(t *testing.T) {
	p1 := orb.Point{10, 10}
	p2 := orb.Point{30, 30}
	p1V := sphericalTo3D(p1)
	p2V := sphericalTo3D(p2)
	p3 := getMiddle(p1V, p2V)
	t.Errorf("p1: %v\n p2: %v \n p3: %v\n Spherical p3: %v", p1V, p2V, p3, p3.ToSpherical())
}

func TestDistanceBetweenPoint(t *testing.T) {
	p1 := orb.Point{0.119, 52.205}
	p2 := orb.Point{2.351, 48.857}
	p1V := sphericalTo3D(p1)
	p2V := sphericalTo3D(p2)
	d := distanceBetweenPointVectors(p1V, p2V)
	fmt.Println("p1V, ", p1V)
	fmt.Println("p2V, ", p2V)
	fmt.Println(p1V.crossProduct(p2V).len())
	fmt.Println(p1V.times(p2V))
	fmt.Println(math.Atan2(p1V.crossProduct(p2V).len(), p1V.times(p2V)))
	d_expected := 404.3
	if d != d_expected {
		t.Errorf("Incorrect d. Got %v, expected %v", d, d_expected)
	}
}

func TestCrossProduct(t *testing.T) {
	p1 := Vector3D{1, 0, 1}
	p2 := Vector3D{0, 1, 0}
	fmt.Println(p1.crossProduct(p2))
	fmt.Println(p2.crossProduct(p1))
}
