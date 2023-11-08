package main

import (
	"fmt"
	"math"
	"math/rand"
	"os"
	"runtime"
	"strconv"
	"sync"
	"time"

	"github.com/paulmach/orb"
	"github.com/paulmach/orb/geo"
	"github.com/paulmach/orb/geojson"
	"github.com/paulmach/orb/quadtree"
)

/* type Node struct {
	Point    orb.Pointer
	children [4]*Node
	Edges    []Edge
} */

type Edge struct {
	Target   orb.Point
	Source   orb.Point
	Distance int
}

type pointResult struct {
	point           orb.Point
	directionPoints map[int8]orb.Point
	edges           []Edge
}

var mu sync.Mutex
var N = 20000
var countInserts = 0

// Konstante für die maximale Distanz (in km)
const MAX_DISTANCE = 30

func main() {
	// GeoJSON-Datei einlesen
	// GeoJSON-Datei parsen
	timeStart, elapsed, fc := ReadGeojson()
	boundingBoxes := make([]BoundingBox, 0)

	timeStart = time.Now()
	for _, feature := range fc.Features {
		boundingBoxes = append(boundingBoxes, createBoundingBox(feature))
	}
	elapsed = time.Since(timeStart)
	fmt.Println("Erstellen der BoundingBoxes:", elapsed)

	timeStart = time.Now()
	boxesMap := createBoxesMap(boundingBoxes)
	elapsed = time.Since(timeStart)
	fmt.Println("BoxMap erstellen: ", elapsed)

	//Entweder mit Goroutinen 10k Points in 30-40Sek oder ohne Goroutinen und dafür direkte Integration in Quadtree in 4min
	// Vor der Generierung der Testpunkte
	pointsChan := make(chan orb.Point, N)
	doneChan := make(chan bool)
	workerCount := runtime.NumCPU()
	qt := quadtree.New(orb.Bound{Min: orb.Point{-180, -90}, Max: orb.Point{180, 90}})
	timeStart = time.Now()
	for i := 0; i < workerCount; i++ {
		go generatePoints(i, workerCount, N, fc, boxesMap, boundingBoxes, pointsChan, doneChan)

	}

	adjazenz_liste := make(map[orb.Point][]orb.Point)

	// Sammle alle Punkte aus dem Channel
	points := make([]orb.Point, 0, N)
	for i := 0; i < N; i++ {
		point := <-pointsChan
		points = append(points, point)
		adjazenz_liste[point] = make([]orb.Point, 0)
	}

	// Warte auf das Beenden der Goroutinen
	for i := 0; i < workerCount; i++ {
		<-doneChan
	}
	close(pointsChan)

	//points[N-1] = orb.Point{15.367310319225993, 42.829323890946426} //zum Testen, damit mind. ein Punkt in der Adria liegt

	elapsed = time.Since(timeStart)
	fmt.Println("Erstellen der Testpunkte:", elapsed)

	timeStart = time.Now()
	//var nearestPoints
	for _, point := range points {
		qt.Add(point)
		countInserts++
	}
	elapsed = time.Since(timeStart)
	fmt.Println("Einfügen der Testpunkte:", elapsed)
	fmt.Println("Anzahl Inserts:", countInserts)

	timeStart = time.Now()
	edgesMap := make(map[string]Edge)
	resultsChan := processPoints(qt, points, MAX_DISTANCE, &edgesMap, adjazenz_liste)
	results := make([]pointResult, 0, N)
	for i := 0; i < N; i++ {
		result := <-resultsChan
		results = append(results, result)
	}

	elapsed = time.Since(timeStart)
	fmt.Println("Suchen der Nachbarn:", elapsed)

	CreateSmallGraph(results)
	writeGraphToGeoJson(results, "big-graph.geojson")
	writeAdjToGeojson(adjazenz_liste, "")
}

// processPoints ist die Funktion, die Goroutines zum Auffinden der nächstgelegenen Punkte verwendet.
func processPoints(qt *quadtree.Quadtree, points []orb.Point, distance float64, edgesMap *map[string]Edge, adj map[orb.Point][]orb.Point) <-chan pointResult {
	//Mutex zur Synchronisierung des Zugriffs auf die edgesMap
	resultsChan := make(chan pointResult, len(points))
	var wg sync.WaitGroup

	for _, point := range points {
		wg.Add(1)
		go func(p orb.Point) {
			defer wg.Done()
			mu.Lock()
			directionPoints, newEdges := findNearestInEachDirection(qt, p, distance, edgesMap)
			for _, dpoint := range directionPoints {
				adj[point] = append(adj[point], dpoint) // Das funktioniert so nicht
				//TO-DO: Verhindern, dass Kanten doppelt eingefügt werden
			}
			mu.Unlock()
			resultsChan <- pointResult{point: p, directionPoints: directionPoints, edges: newEdges}
		}(point)
	}
	wg.Wait()
	close(resultsChan)

	return resultsChan
}

func generatePoints(workerID int, workerCount int, totalPoints int, fc *geojson.FeatureCollection, boxesMap map[int][]int, boundingBoxes []BoundingBox, pointsChan chan<- orb.Point, doneChan chan<- bool) {
	pointsToGenerate := totalPoints / workerCount
	if workerID == workerCount-1 {
		// Wenn N nicht durch Workercount teilbar ist, bekommt letzte Goroutine die "übrigen" Punkte
		pointsToGenerate += totalPoints % workerCount
	}

	for i := 0; i < pointsToGenerate; i++ {
		testPoint := generateRandomPoint()
		// Prüfe, ob der Punkt im Land liegt, usw.
		isInLand := false
		i_int := int(math.Floor(testPoint.Lon()))
		for _, boxIndex := range boxesMap[i_int] {
			if !isPointInBoundingBox(testPoint, boundingBoxes[boxIndex]) {
				continue
			}
			coast := fc.Features[boxIndex].Geometry.(orb.LineString)
			if isinPolygon(testPoint, coast) {
				isInLand = true
				break
			}
		}
		if testPoint.Lat() < -85.0 {
			isInLand = true
		}
		if !isInLand {
			pointsChan <- testPoint
		} else {
			// Wenn der Punkt im Land ist, soll dieser Durchlauf nicht gezählt werden
			i--
		}
	}
	doneChan <- true

}

func haversine(lon1, lat1, lon2, lat2 float64) float64 {
	const R = 6371 //km Erdradius
	dLat := (lat2 - lat1) * math.Pi / 180
	dLon := (lon2 - lon1) * math.Pi / 180
	lat1 = lat1 * math.Pi / 180
	lat2 = lat2 * math.Pi / 180
	a := math.Sin(dLat/2)*math.Sin(dLat/2) + math.Sin(dLon/2)*math.Sin(dLon/2)*math.Cos(lat1)*math.Cos(lat2)
	c := 2 * math.Atan2(math.Sqrt(a), math.Sqrt(1-a))
	distance := R * c
	return distance
}

func findNearestInEachDirection(qt *quadtree.Quadtree, srcPoint orb.Point, maxDistance float64, edgesMap *map[string]Edge) (map[int8]orb.Point, []Edge) {
	directionPoints := map[int8]orb.Point{}
	minDistances := map[int8]float64{
		2: math.MaxFloat64, // "NW"
		1: math.MaxFloat64, // "NE"
		4: math.MaxFloat64, // "SE"
		3: math.MaxFloat64, // "SW"
	}

	for direction := range minDistances {
		directionPoints[direction] = orb.Point{}
	}

	checkDirectionsFilled := func() bool {
		for _, distance := range minDistances {
			if distance == math.MaxFloat64 {
				return false
			}
		}
		return true
	}

	// k = 10 als erster Startwert
	k := 10
	for !checkDirectionsFilled() {
		nearestPoints := qt.KNearest(nil, srcPoint, k, maxDistance)
		for _, p := range nearestPoints {
			point := p.(orb.Point)
			bearing := geo.Bearing(srcPoint, point)
			distance := haversine(srcPoint.Lon(), srcPoint.Lat(), point.Lon(), point.Lat())

			if distance >= maxDistance || distance < 0.01 {
				continue
			}

			// Bestimmung der Richtung
			var direction int8
			if bearing >= -90 && bearing < 0 {
				direction = 2
			} else if bearing >= 0 && bearing < 90 {
				direction = 1
			} else if bearing >= 90 && bearing < 180 {
				direction = 4
			} else if bearing >= -180 && bearing < -90 {
				direction = 3
			}

			if distance < minDistances[direction] {
				directionPoints[direction] = point
				minDistances[direction] = distance
			}
		}

		// Falls benötigt, k erhöhen um erneut nach Nachbarn zu suchen
		if !checkDirectionsFilled() {
			//Der letzte Punkt in nearestPoint ist am weitesten entfernt. Ist dieser Punkt also weiter entfernt
			// als maxDistance, macht es keinen Sinn weiterzusuchen.
			point := nearestPoints[len(nearestPoints)-1].(orb.Point)
			if haversine(srcPoint.Lon(), srcPoint.Lat(), point.Lon(), point.Lat()) > maxDistance {
				break
			}
			k += 10
			if k >= 100 { // Vermutlich unnötig mit oberer Abbruchbedingung?
				break
			}
		}
	}

	var newEdges []Edge
	// Erstelle Kanten für die Richtungspunkte
	for _, point := range directionPoints {
		if point != (orb.Point{}) {
			edgeKey := fmt.Sprintf("%v_%v", srcPoint, point)
			reverseEdgeKey := fmt.Sprintf("%v_%v", point, srcPoint)
			_, exists := (*edgesMap)[reverseEdgeKey]
			if !exists {
				//Kante noch nicht vorhanden, also hinzufügen
				distance := haversine(srcPoint.Lon(), srcPoint.Lat(), point.Lon(), point.Lat())
				newEdge := Edge{Source: srcPoint, Target: point, Distance: int(distance)}
				newEdges = append(newEdges, newEdge)
				(*edgesMap)[edgeKey] = newEdge
			}
		}
	}
	// Richtungen entfernen, in denen keine Nachbarn gefunden wurden
	for dir, distance := range minDistances {
		if distance == math.MaxFloat64 {
			delete(directionPoints, dir)
		}
	}

	return directionPoints, newEdges
}

func generateRandomPoint() orb.Point {
	//Erstellen eines Zufälligen Knotens
	lon := 360.0 * (rand.Float64() - 0.5) // zwischen -180 und 180
	randomValue := 2*rand.Float64() - 1   // zwischen -1 und 1
	latRad := math.Acos(randomValue)      // in Radiant
	lat := (latRad * 180 / math.Pi) - 90  // in Grad zwischen -90 und 90
	//lat := 7*rand.Float64() + 39          // Einschränkung auf Adria
	//lon := 20*rand.Float64() + 5          // Einschränkung auf Adria
	return orb.Point{lon, lat}
}

func ReadGeojson() (time.Time, time.Duration, *geojson.FeatureCollection) {
	timeStart := time.Now()

	raw, err := os.ReadFile("planet-bb.geojson")
	if err != nil {
		panic(err)
	}
	elapsed := time.Since(timeStart)
	fmt.Println("Öffnen der Datei:", elapsed)

	timeStart = time.Now()

	fc, err := geojson.UnmarshalFeatureCollection(raw)
	if err != nil {
		panic(err)
	}
	elapsed = time.Since(timeStart)
	fmt.Println("Parsen der Datei:", elapsed)
	return timeStart, elapsed, fc
}

func CreateSmallGraph(results []pointResult) {
	//Grenzen den Adria
	upperLeftBound := orb.Point{20.658852934430058, 46.112454655486374}
	lowerRightBound := orb.Point{11.349723073245997, 40.167852045812964}
	boundComplete := orb.Bound{Max: upperLeftBound, Min: lowerRightBound}
	resultsInBound := make([]pointResult, 0)
	for _, p := range results {
		point := p.point
		if boundComplete.Contains(point) {
			resultsInBound = append(resultsInBound, p)
		}
	}
	writeGraphToGeoJson(resultsInBound, "")
}

func writeGraphToGeoJson(points []pointResult, filename string) {
	if filename == "" {
		filename = "small-graph.geojson"
	}
	file, err := os.Create(filename)
	if err != nil {
		panic(err)
	}
	writePunkte := false
	file.Write([]byte("{\"type\": \"FeatureCollection\",\"features\": [ \n"))
	//Punkte schreiben
	if writePunkte {
		for index, p := range points {
			point := p.point
			file.Write([]byte("{\"type\": \"Feature\",\"properties\": {\"marker-color\": \"#7e7e7e\",\"marker-size\": \"small\",\"marker-symbol\": \"circle\"},\"geometry\": {\"coordinates\": [" + strconv.FormatFloat(point.Lon(), 'f', -1, 64) + "," + strconv.FormatFloat(point.Lat(), 'f', -1, 64) + "],\"type\": \"Point\"}}"))
			if index != len(points)-1 {
				file.Write([]byte(",\n"))
			}
		}
	}
	//LineStrings schreiben
	first := true
	for _, p := range points {
		point := p.point
		for _, e := range p.directionPoints {
			if !first || writePunkte {
				file.Write([]byte(",\n"))
			}
			file.Write([]byte("{\"type\": \"Feature\",\"properties\": {},\"geometry\": {\"coordinates\": [[" + strconv.FormatFloat(point.Lon(), 'f', -1, 64) + "," + strconv.FormatFloat(point.Lat(), 'f', -1, 64) + "],[" + strconv.FormatFloat(e.Lon(), 'f', -1, 64) + "," + strconv.FormatFloat(e.Lat(), 'f', -1, 64) + "]],\"type\": \"LineString\"}}"))
			first = false
		}
	}

	file.Write([]byte("]}"))

}

func writeAdjToGeojson(adj map[orb.Point][]orb.Point, filename string) {
	if filename == "" {
		filename = "big-graph-adj.geojson"
	}
	file, err := os.Create(filename)
	if err != nil {
		panic(err)
	}
	file.Write([]byte("{\"type\": \"FeatureCollection\",\"features\": [ \n"))
	first := true
	for point := range adj {
		for _, neighbor := range adj[point] {
			if !first {
				file.Write([]byte(",\n"))
			}
			file.Write([]byte("{\"type\": \"Feature\",\"properties\": {},\"geometry\": {\"coordinates\": [[" + strconv.FormatFloat(point.Lon(), 'f', -1, 64) + "," + strconv.FormatFloat(point.Lat(), 'f', -1, 64) + "],[" + strconv.FormatFloat(neighbor.Lon(), 'f', -1, 64) + "," + strconv.FormatFloat(neighbor.Lat(), 'f', -1, 64) + "]],\"type\": \"LineString\"}}"))
			first = false
		}
	}

	file.Write([]byte("]}"))
}
