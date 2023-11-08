package main

import (
	"fmt"
	"math"
	"os"
	"time"

	"github.com/paulmach/orb"
	"github.com/paulmach/orb/geojson"
)

type BoundingBox []float64

// createBoundingBox erstellt die BoundingBox für ein Polygon
func createBoundingBox(feature *geojson.Feature) BoundingBox {
	var bbox BoundingBox
	properties := feature.Properties

	// Zugriff auf die BBoxes aus den Properties
	bboxData := properties["bbox"]

	// Die BBox-Daten sollten als Map interpretiert werden
	bboxMap := bboxData.(map[string]interface{})

	minData := bboxMap["Min"].([]interface{})
	maxData := bboxMap["Max"].([]interface{})

	bbox = append(bbox, minData[0].(float64))
	bbox = append(bbox, minData[1].(float64))
	bbox = append(bbox, maxData[0].(float64))
	bbox = append(bbox, maxData[1].(float64))

	return bbox

}

// createBoxesMap unterteilt die Boundingboxes der Landmassen in 360 Bereiche, die den
// Längengraden entsprechen. In resultMap[-53] befinden sich die IDs aller Boundingboxes,
// die ganz oder teilweise zwischen den Längengraden -53 und -52 liegen
// Mit dieser Map muss nicht für jeden Punkte durch alle Polygone iteriert werden, was den
// check ob, ein Punkt im Wasser liegt deutlich beschleunigt
func createBoxesMap(boxes []BoundingBox) map[int]([]int) {
	resultMap := make(map[int]([]int), 360)
	for i := -180; i <= 179; i++ {
		for boxIndex, box := range boxes {
			i_float := float64(i)
			iplus_float := float64(i + 1)
			if box[0] <= i_float && box[2] >= i_float {
				resultMap[i] = append(resultMap[i], boxIndex)
			} else if box[0] <= iplus_float && box[2] >= i_float {
				resultMap[i] = append(resultMap[i], boxIndex)
			}
		}
	}
	return resultMap
}

// isPointInBoundingBox überprüft, ob ein Punkt in einer der BoundingBoxes ist
func isPointInBoundingBox(point orb.Point, bbox BoundingBox) bool {

	minLon := bbox[0]
	minLat := bbox[1]
	maxLon := bbox[2]
	maxLat := bbox[3]

	if minLon == -180.0 && maxLon == 180.0 {
		minLat = -90.0
	}

	// Prüfen, ob der Punkt innerhalb der Bounding-Box liegt
	if point[0] >= minLon && point[0] <= maxLon &&
		point[1] >= minLat && point[1] <= maxLat {
		return true
	}

	return false
}

var thirdCaseCount int = 0

func main__() {
	timeStart := time.Now()
	// GeoJSON-Datei einlesen
	raw, err := os.ReadFile("planet-bb.geojson")
	if err != nil {
		panic(err)
	}
	elapsed := time.Since(timeStart)
	fmt.Println("Öffnen der Datei:", elapsed)

	timeStart = time.Now()
	// GeoJSON-Datei parsen
	fc, err := geojson.UnmarshalFeatureCollection(raw)
	if err != nil {
		panic(err)
	}
	elapsed = time.Since(timeStart)
	fmt.Println("Parsen der Datei:", elapsed)

	// N zufällige Punkte erstellen
	N := 1000
	testPoints := make([]orb.Point, N)
	for i := 0; i < N; i++ {
		testPoints[i] = generateRandomPoint()
	}

	boundingBoxes := make([]BoundingBox, 0)
	for _, feature := range fc.Features {
		boundingBoxes = append(boundingBoxes, createBoundingBox(feature))
	}

	timeStart = time.Now()
	boxesMap := createBoxesMap(boundingBoxes)
	elapsed = time.Since(timeStart)
	fmt.Println("BoxMap erstellen: ", elapsed)
	/* testPoints := []orb.Point{
		{-45.15999934832257, -60.73832221870806},  //Land auf Insel nähe Antarktis
		{-77.51704069635798, 1.0953059224790138},  //Land Südamerika
		{-17.50008966777773, -5.65295679002385},   //Wasser bzw Ozean zwischen Afrika und Amerika
		{60.930223567211044, -36.112564910573354}, //Vor der Küste von Grönland Theta Breitengrad und Phi der Längengrad
		{5.9163635068202325, 17.968917950645505},  //Afrika Land im Niger (unser doppeltes Problem in der alten Implemntierung)
		{-59.061044273391104, 30.787101909180876}, //Wasser bzw Ozean zwischen Afrika und Amerika
		{-65.764332790228, 7.083978611621134},     //Land Venezuela
		{114.47871114702582, -80.9606692661055},   //Land in der Antarktis
		{-179.24497269951658, 68.39755722879585},  //Nahe der Datumsgrenze Land
		{0.3650969576612795, 53.318324497779884},  //Nahe Greenich aber in Wasser
		{69, -70},
	} */
	landCounter := 0
	timeStart = time.Now()
	for _, testPoint := range testPoints {
		i_int := int(math.Floor(testPoint.Lon()))
		for _, boxIndex := range boxesMap[i_int] {
			feature := fc.Features[boxIndex]
			geom := feature.Geometry
			if !isPointInBoundingBox(testPoint, boundingBoxes[boxIndex]) {
				continue
			}
			coast := geom.(orb.LineString)
			if isinPolygon(testPoint, coast) {
				landCounter++
				break
			}
		}

	}

	elapsed = time.Since(timeStart)
	fmt.Println("Zeit für Testpoints: ", elapsed)
	fmt.Println("Anzahl der dritten Fälle: ", thirdCaseCount)
	fmt.Println("Anzahl der Landpunkte ", landCounter)
}

func sphericalTo3D(p orb.Point) Vector3D {

	φ := p[1] * (math.Pi / 180.0) //p[1] = lat
	λ := p[0] * (math.Pi / 180.0)
	x := math.Cos(φ) * math.Cos(λ)
	y := math.Cos(φ) * math.Sin(λ)
	z := math.Sin(φ)
	return Vector3D{x, y, z}
}

func ApproxEqualVector(a, b Vector3D, epsilon float64) bool {
	return a.sub(b).len() < epsilon
}

func isinPolygon(point orb.Point, polygon orb.LineString) bool {
	anker := orb.Point{0, 90}
	intersectCounter := 0
	for index := range polygon {
		if index == len(polygon)-1 {
			break
		}
		if point == polygon[index] {
			return false
		}
		if doLinesIntersect(point, anker, polygon[index], polygon[index+1]) {
			intersectCounter++
		}
	}
	return !(intersectCounter%2 == 0)
}

func doLinesIntersect(tp, np, p1, p2 orb.Point) bool {

	//Fall 1: Überprüfung der Longitudes
	if (p1.Lon() <= tp.Lon() && p2.Lon() <= tp.Lon()) || (p1.Lon() >= tp.Lon() && p2.Lon() >= tp.Lon()) {

		return false
	}
	minLatP := min(p1.Lat(), p2.Lat())
	maxLatP := max(p1.Lat(), p2.Lat())
	//Fall 2: Überprüfung der Latitudes
	if tp.Lat() <= minLatP && np.Lat() >= maxLatP {

		return true
	}
	//Fall 2a
	if tp.Lat() > maxLatP {
		return false
	}
	//Teurer, genauer Check für edge cases:
	//alle punkte 3D-Vektoren umwandeln
	//Diese Vektoren sind normal zur Kugeloberfläche, können deshalb als normalenvektor bezeichnet werden
	path1StartNVector := sphericalTo3D(tp)
	path1EndeNVector := sphericalTo3D(np)
	path2StartNVector := sphericalTo3D(p1)
	path2EndeNVector := sphericalTo3D(p2)
	//großer Kreis definiert durch test und anker:
	//c1 ist der große Kreis, auf dem test und anker liegen
	c1 := path1StartNVector.crossProduct(path1EndeNVector)

	//c2 ist der große Kreis, auf dem die beiden Ecken des Polygons liegen
	c2 := path2StartNVector.crossProduct(path2EndeNVector)

	//Schnittpunkte der großen Kreise:
	i1 := c1.crossProduct(c2)
	i2 := c2.crossProduct(c1)

	midPath1 := getMiddle(path1StartNVector, path1EndeNVector)
	midPath2 := getMiddle(path2StartNVector, path2EndeNVector)

	//i1 ist Schnittpunkt der Pfade, wenn es auf beiden pfaden zwischen den gegebenen punkt liegt
	if distanceBetweenPointVectors(midPath1, i1) < distanceBetweenPointVectors(path1StartNVector, midPath1) && distanceBetweenPointVectors(midPath2, i1) < distanceBetweenPointVectors(path2StartNVector, midPath2) {
		return true
	}
	//i2 ist Schnittpunkt der Pfade, wenn es auf beiden pfaden zwischen den gegebenen punkt liegt
	if distanceBetweenPointVectors(midPath1, i2) < distanceBetweenPointVectors(path1StartNVector, midPath1) && distanceBetweenPointVectors(midPath2, i2) < distanceBetweenPointVectors(path2StartNVector, midPath2) {
		return true
	}
	//}
	return false
}

func distanceBetweenPointVectors(a, b Vector3D) float64 {
	R := 6371.0
	return R * math.Acos(a.times(b))
}

func getMiddle(a, b Vector3D) Vector3D {
	mid := Vector3D{a.X + b.X, a.Y + b.Y, a.Z + b.Z}
	return mid.Normalize()
}
