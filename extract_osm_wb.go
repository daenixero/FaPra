package main

import (
	"context"
	"encoding/json"
	"fmt"
	"io"
	"os"
	"strings"
	"time"

	"github.com/paulmach/orb"
	"github.com/paulmach/orb/geojson"
	"github.com/paulmach/osm"
	"github.com/paulmach/osm/osmpbf"
)

func main2() {
	start := time.Now()
	file, err := os.Open("planet-coastlinespbf-cleaned.osm.pbf")
	if err != nil {
		panic(err)
	}
	defer file.Close()
	elapsed := time.Since(start)

	fmt.Println("Öffnen der Datei:", elapsed)
	//reader := bufio.NewReader(file)
	start = time.Now()
	scanner := osmpbf.New(context.Background(), file, 16) //Anstelle von file nehmen wir reader
	fc := geojson.NewFeatureCollection()
	nodeMap := make(map[osm.NodeID]orb.Point, 62612921) // vorher ohne Komma und 10000
	coastlineMap := make(map[orb.Point]orb.LineString)  //Gruppiert die Küstenlinien orb.LineString nach Start- und Endpunkten
	//var coastlines []orb.LineString                     //eine leere Slice, die alle Küstenlinien als orb.LineString enthalten wird

	//SCanner ließt die Datei und füllt die nodeMap und endpointMap mit den entsprechenden Daten
	for scanner.Scan() {
		switch e := scanner.Object().(type) {
		case *osm.Node:
			//Für jeden Node wird seine ID und seine Koordinaten in der nodeMap gespeichert
			nodeMap[e.ID] = orb.Point{e.Lon, e.Lat}

		case *osm.Way:
			// Es werden nur die Ways beachtet, dessen Tag "natural" den Wert "coastline" hat
			if e.Tags.Find("natural") == "coastline" {
				//Erstellen der Küstenlinien LineString
				var line orb.LineString
				for idx, nd := range e.Nodes {
					//Für jeden Way wird eine neue LineString (coastline) erstellt. Sie wird aus den Koordinaten der Nodes gebildet, die dem Way zugeordnet sind.
					coord, ok := nodeMap[nd.ID]
					if !ok {
						fmt.Printf("Node ID %d not found\n", nd.ID)
						continue
					}
					line = append(line, coord)
					//Die Nodes, die nicht am Anfang oder Ende der LineString stehen, werden aus der nodeMap gelöscht, da sie nicht mehr gebraucht werden,
					//da uns nur die Nodes am Anfang und Ende der ways noch interessieren. Die Nodes dazwischen können gar nicht mehr in einem anderen Way vorkommen.
					if idx != 0 && idx != len(e.Nodes)-1 {
						delete(nodeMap, nd.ID)
					}
				}
				//Die Küstenlinien werden in der Slice coastlines gespeichert. Es wird direkt beim auslesen eines Ways überprüft, ob man die mit bestehenden Ways verbinden kann.
				newLine, closed := appendToElementInMap(line, coastlineMap)
				if closed {

					feature := geojson.NewFeature(orb.LineString(newLine))

					fc.Append(feature)

				}

			}

		}

	}
	//fmt.Println(len(nodeMap))
	elapsed = time.Since(start)
	fmt.Println("Scannen der Datei:", elapsed)
	if err := scanner.Err(); err != nil && err != io.EOF {
		panic(err)
	}
	//fmt.Println(len(coastlines))

	start = time.Now()
	createGeoJSONFile(fc)
	elapsed = time.Since(start)
	fmt.Println("Erstellen der GeoJSON-Datei:", elapsed)

}

type SimpleFeature struct {
	ID          int
	Geometry    orb.LineString
	BoundingBox orb.Bound
}

type SimpleFeatureCollection struct {
	Features []SimpleFeature
}

// Um das Format der GeoJSON-Datei zu vereinfachen, wird die Funktion MarshalJSON() aus dem Package gelöscht und durch einen eigenen Marashal ersetzt, der nur die nötigen Informationen enthält
func (sfc *SimpleFeatureCollection) MarshalJSON() ([]byte, error) {
	var sb strings.Builder
	sb.WriteString("{\"type\":\"FeatureCollection\",\"features\":[")

	for i, feature := range sfc.Features {
		if i > 0 {
			sb.WriteString(",")
		}
		sb.WriteString("{\"type\":\"Feature\",\"properties\":{")

		// Include Bounding Box in properties
		sb.WriteString("\"id\":")
		sb.WriteString(fmt.Sprint(feature.ID))
		sb.WriteString(",\"bbox\":")
		bboxJSON, err := json.Marshal(feature.BoundingBox)
		if err != nil {
			return nil, err
		}
		sb.Write(bboxJSON)

		sb.WriteString("},\"geometry\":{\"type\":\"LineString\",\"coordinates\":")
		geomJSON, err := json.Marshal(feature.Geometry)
		if err != nil {
			return nil, err
		}
		sb.Write(geomJSON)
		sb.WriteString("}}")
	}

	sb.WriteString("]}")
	return []byte(sb.String()), nil
}

// Ausgelagerte Funktion zum Erstellen des GeoJSON-Files
func createGeoJSONFile(fc *geojson.FeatureCollection) {
	file, err := os.Create("planet-bb.geojson")
	if err != nil {
		panic(err)
	}
	defer file.Close()

	sfc := &SimpleFeatureCollection{Features: []SimpleFeature{}}

	for i, feature := range fc.Features {
		geom := feature.Geometry.(orb.LineString)
		bbox := geom.Bound()
		sf := SimpleFeature{
			ID:          i,
			Geometry:    feature.Geometry.(orb.LineString),
			BoundingBox: bbox,
		}
		sfc.Features = append(sfc.Features, sf)
	}

	//Benutzen des eigenen Marshalers
	bytes, err := json.Marshal(sfc)
	if err != nil {
		panic(err)
	}

	file.Write(bytes)

}

func appendToElementInMap(line orb.LineString, coastlineMap map[orb.Point]orb.LineString) (orb.LineString, bool) {
	if line[0] == line[len(line)-1] {
		return line, true //in diesem fall ist "line" schon ein fertiges polygon und kann extra abgespeichert werden
	}
	for point, linestring := range coastlineMap { //ich tu hier mal so, als wären nur endpunkte in der map
		if line[0] == point {
			newLine := append(linestring, line[1:]...) //das nullte Element von line brauchen wir nicht anzuhängen
			delete(coastlineMap, point)                //entferne den gefundenen partner aus der map
			return appendToElementInMap(newLine, coastlineMap)
		} else if line[len(line)-1] == linestring[0] {
			newLine := append(line, linestring[1:]...)
			delete(coastlineMap, point)
			return appendToElementInMap(newLine, coastlineMap)

		}
	}

	coastlineMap[line[len(line)-1]] = line //wenn wir durch die ganze map durchiteriert haben, ohne einen "partner" zu finden, wird die line ein neuer eintrag in der endpointmap
	//der zweite rückgabeparameter beschreibt, ob ein geschlossenes polygon erzeugt wurde
	return line, false
}
