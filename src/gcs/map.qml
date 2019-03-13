import QtQuick 2.5
import QtLocation 5.6
import MapItem 1.0
import QtPositioning 5.2

MapItem {
    id: mapItem
    width: 1024
    height: 768

    signal waypointSelected(real x, real y, real clat, real clng)
    signal mapDragEvent(real x, real y, real clat, real clng, int type)
    signal updateHomeLocation(real clat, real clng)

    Map {
        id: navMap
        gesture.enabled: true
        gesture.acceptedGestures: MapGestureArea.PinchGesture | MapGestureArea.PanGesture
        copyrightsVisible: false
        width: parent.width
        height: parent.height
        objectName: "navMap"
        zoomLevel: (maximumZoomLevel > minimumZoomLevel + 1) ? minimumZoomLevel + 1 : 1
        center {
            latitude: 0.0
            longitude: 0.0
        }
        plugin: Plugin {
            name: "osm"
        }

        MouseArea {
            id: mapArea
            anchors.fill: parent
            cursorShape: Qt.CrossCursor
            onDoubleClicked: {
                var c =  navMap.toCoordinate(Qt.point(mouse.x, mouse.y))
                markerModel.createWaypoint(c.latitude, c.longitude)  // create WP
                wpLine.addCoordinate(c)
            }
        }

        // list of navigation waypoints
        MapItemView {
            model: markerModel
            delegate: MapQuickItem {
                id: marker
                anchorPoint.x: navDot.width / 2
                anchorPoint.y: navDot.height / 2
                coordinate: position
                property int radius: 8
                sourceItem: Rectangle {
                    id: navDot
                    width: marker.radius * 2
                    height: marker.radius * 2
                    color: dotColor
                    radius: marker.radius
                    antialiasing: true
                    opacity: 1
                    Text {
                        id: wpIdxTxt
                        text: (rowNumber + 1).toString()
                        font.family: "Arial"
                        anchors.centerIn: parent
                    }
                    MouseArea {
                        id: wpArea
                        anchors.fill: parent
                        onClicked: {
                            if (mouse.button == Qt.LeftButton) {
                                // map the relative position of MapItemView to Map
                                var obj = mapToItem(navMap, mouse.x, mouse.y)
                                var c = navMap.toCoordinate(Qt.point(obj.x, obj.y))
                                mapItem.waypointSelected(obj.x, obj.y, c.latitude, c.longitude)
                            }
                        }
                        drag {
                            target: marker
                            onActiveChanged: {
                                if (drag.active) {
                                    // console.log("Drag started: " + drag.target.sourceItem)
                                    mapItem.mapDragEvent(0, 0, drag.target.coordinate.latitude, drag.target.coordinate.longitude, 0)
                                } else {
                                    // console.log("Drag ended: " + drag.target.sourceItem)
                                    mapItem.mapDragEvent(0, 0, drag.target.coordinate.latitude, drag.target.coordinate.longitude, 1)
                                    // console.log('update wp#' + wpIdxTxt.text)
                                    wpLine.replaceCoordinate(parseInt(wpIdxTxt.text), drag.target.coordinate)
                                }
                            }
                        }
                    }
                }
            }
        }

        MapPolyline {
            id: wpLine
            objectName: "wpLine"
            line.width: 3
            line.color: "red"
            antialiasing: true
            path: [
                {latitude: 0, longitude: 0}
            ]
        }

        // The home icon on map
        MapQuickItem {
            property bool firstRun: true

            id: homeMarker
            anchorPoint.x: homeImage.width / 4
            anchorPoint.y: homeImage.height
            // coordinate : navMap.center
            sourceItem: Image {
                id: homeImage
                source: "res/home.png"
                MouseArea {
                    id: homeArea
                    anchors.fill: parent
                    drag {
                        target: homeMarker
                        onActiveChanged: {
                            if (!drag.active) {
                                homeMarker.coordinate = drag.target.coordinate
                                wpLine.replaceCoordinate(0, homeMarker.coordinate)
                                mapItem.updateHomeLocation(drag.target.coordinate.latitude, drag.target.coordinate.longitude)
                            }
                        }
                    }
                }
            }
        }
    }

    onUpdateZoomLevel: {
        if (zoom >= navMap.minimumZoomLevel || zoom <= navMap.maximumZoomLevel) {
            navMap.zoomLevel = zoom
        }
    }

    onUpdateCoordinate: {
        navMap.center.latitude = lat
        navMap.center.longitude = lng
        if (homeMarker.firstRun) {
            homeMarker.coordinate.latitude = lat
            homeMarker.coordinate.longitude = lng
            wpLine.replaceCoordinate(0, homeMarker.coordinate)
            homeMarker.firstRun = false
        }
    }

    onWaypointRemoved: {
        wpLine.removeCoordinate(wpNumber)
    }

    onWaypointChanged: {
        wpLine.replaceCoordinate(wpNumber, QtPositioning.coordinate(latitude, longitude))
    }
}
